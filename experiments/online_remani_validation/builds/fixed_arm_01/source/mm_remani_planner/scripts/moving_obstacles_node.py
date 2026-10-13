#!/usr/bin/env python3
"""Simple node that merges the static /map_generator/global_cloud
with a set of small moving cuboid obstacle point clouds. It can publish an
optional merged visualization cloud and a velocity-annotated point-flow cloud.
It is used by exp3.launch and the isolated neupan_fast_sim.launch test.

Note: make the script executable after creation:
  chmod +x path/to/moving_obstacles_node.py
"""
from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import math
import ast
import threading


class MovingObstaclesNode(object):
    def __init__(self):
        rospy.init_node('moving_obstacles_node')

        self.pub_rate = rospy.get_param('~pub_rate', 5.0)
        # read params and be robust to string values from <param value="[...]"/>
        moving_count_raw = rospy.get_param('~moving_count', 3)
        try:
            # handle string like "3" or "[0.4,0.4,0.6]"
            if isinstance(moving_count_raw, str):
                moving_count = int(ast.literal_eval(moving_count_raw))
            else:
                moving_count = int(moving_count_raw)
        except Exception:
            moving_count = 3
        self.moving_count = moving_count

        self.box_resolution = rospy.get_param('~box_resolution', 0.05)
        # fallback to param in exp3 if provided
        moving_cfg = rospy.get_param('/moving_obstacles', None) if rospy.has_param('/moving_obstacles') else None
        if moving_cfg:
            try:
                self.moving_count = int(moving_cfg.get('count', self.moving_count))
            except Exception:
                pass
        # store latest static cloud points
        self.static_points = []
        self.map_frame = 'world'
        # protect shared data between callback and publisher
        self._lock = threading.Lock()
        # track last publish time to avoid processing our own published cloud
        self._last_pub_time = rospy.Time(0)

        self.publish_merged = bool(
            rospy.get_param('~publish_merged', True))
        if self.publish_merged:
            rospy.Subscriber('/map_generator/global_cloud', PointCloud2,
                             self.static_cloud_cb)

        # We publish merged cloud on a separate topic so we don't race with the
        # original map_generator publisher. For exp3 we remap planner subscribers
        # to this topic so planners see the merged view.
        self.merged_topic = rospy.get_param('~merged_topic', '/map_generator/global_cloud_merged')
        self.pub = (rospy.Publisher(self.merged_topic, PointCloud2,
                                    queue_size=1)
                    if self.publish_merged else None)
        # also publish moving-only pointcloud so we can visualize moving obstacles
        self.moving_only_topic = rospy.get_param('~moving_only_topic', '/moving_obstacles_only')
        self.moving_pub = rospy.Publisher(self.moving_only_topic, PointCloud2, queue_size=1)
        # Point-flow topic consumed by the planner's shadow predictor. Each
        # surface point carries the analytic velocity of its parent obstacle.
        self.dynamic_points_topic = rospy.get_param(
            '~dynamic_points_topic', '/dynamic_points')
        self.dynamic_points_pub = rospy.Publisher(
            self.dynamic_points_topic, PointCloud2, queue_size=1)
        self.dynamic_point_fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('vx', 12, PointField.FLOAT32, 1),
            PointField('vy', 16, PointField.FLOAT32, 1),
            PointField('vz', 20, PointField.FLOAT32, 1),
            PointField('track_id', 24, PointField.UINT32, 1),
            PointField('confidence', 28, PointField.FLOAT32, 1),
        ]

        # moving object motion params
        self.map_x = rospy.get_param('/map/x_size', 8.0)
        self.map_y = rospy.get_param('/map/y_size', 8.0)

        # Motion defaults preserve exp3's original longitudinal oscillation.
        # The independent NeuPAN-fast test launch selects "linear_crossing" so
        # the velocity in PointCloud2 agrees with the constant-velocity model
        # used by the predictor during the actual encounter.
        self.motion_mode = str(
            rospy.get_param('~motion_mode', 'longitudinal')).strip().lower()
        self.motion_amplitude = float(
            rospy.get_param('~motion_amplitude', -1.0))
        self.motion_omega = float(
            rospy.get_param('~motion_omega', -1.0))
        self.motion_phase = float(
            rospy.get_param('~motion_phase', 0.0))
        self.crossing_x = float(rospy.get_param('~crossing_x', 0.0))
        self.crossing_start_y = float(
            rospy.get_param('~crossing_start_y', -3.5))
        self.crossing_end_y = float(
            rospy.get_param('~crossing_end_y', 3.5))
        self.crossing_speed = abs(float(
            rospy.get_param('~crossing_speed', 0.75)))
        self.center_z = float(rospy.get_param('~center_z', 0.9))
        self.sync_to_first_trajectory = bool(
            rospy.get_param('~sync_to_first_trajectory', False))

        if self.motion_mode not in ('longitudinal', 'crossing',
                                    'linear_crossing'):
            rospy.logwarn("Unknown motion_mode '%s'; using longitudinal",
                          self.motion_mode)
            self.motion_mode = 'longitudinal'
        if (self.motion_mode == 'linear_crossing' and
                self.crossing_speed <= 1.0e-6):
            rospy.logwarn(
                "crossing_speed is zero; linear-crossing obstacle will remain "
                "at crossing_start_y")

        # A deterministic test can hold the obstacle at the start of its
        # motion until REMANI publishes the first executable trajectory.  This
        # removes launch/goal timing jitter from the crossing experiment while
        # leaving exp3's original free-running behavior unchanged by default.
        self.start_time = rospy.Time.now()
        self.motion_started = not self.sync_to_first_trajectory
        self.trajectory_trigger_sub = None
        if self.sync_to_first_trajectory:
            self.trajectory_trigger_sub = rospy.Subscriber(
                '/planning/trajectory', rospy.AnyMsg,
                self.trajectory_trigger_cb, queue_size=1)

    def trajectory_trigger_cb(self, _msg):
        with self._lock:
            if self.motion_started:
                return
            self.start_time = rospy.Time.now()
            self.motion_started = True
        rospy.loginfo(
            "Moving-obstacle clock synchronized to the first planned trajectory")

    def static_cloud_cb(self, msg):
        # ignore messages that we ourselves just published (avoid feedback loop)
        try:
            if msg.header.stamp == self._last_pub_time:
                return
        except Exception:
            pass

        # store static points as tuples (x,y,z)
        frame = msg.header.frame_id if hasattr(msg.header, 'frame_id') else 'world'
        frame = frame or 'world'
        pts = []
        for p in pc2.read_points(msg, skip_nans=True):
            # read_points yields (x,y,z,...)
            pts.append((p[0], p[1], p[2]))
        with self._lock:
            self.static_points = pts
            self.map_frame = frame

    def box_points(self, center, size, resolution):
        # generate points within a box (including faces/volume) at given resolution
        sx, sy, sz = size
        cx, cy, cz = center
        nx = max(1, int(math.ceil(sx / resolution)))
        ny = max(1, int(math.ceil(sy / resolution)))
        nz = max(1, int(math.ceil(sz / resolution)))
        pts = []
        for i in range(nx):
            x = cx - sx/2.0 + (i + 0.5) * (sx / nx)
            for j in range(ny):
                y = cy - sy/2.0 + (j + 0.5) * (sy / ny)
                for k in range(nz):
                    z = cz - sz/2.0 + (k + 0.5) * (sz / nz)
                    pts.append((x, y, z))
        return pts

    def run(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            with self._lock:
                motion_started = self.motion_started
                start_time = self.start_time
            t = ((now - start_time).to_sec()
                 if motion_started else 0.0)

            # build moving box centers using simple trajectories
            moving_pts = []
            dynamic_points = []
            # Analytic motion keeps position and velocity on exactly the same
            # timestamp, which is important for evaluating point-flow TTC.
            for i in range(self.moving_count):
                # amplitude depends on map size but keep a safe margin
                default_amp = max(0.5, min(self.map_x * 0.2, 2.0))
                amp = (self.motion_amplitude if self.motion_amplitude > 0.0
                       else default_amp)
                omega = (self.motion_omega if self.motion_omega > 0.0
                         else 0.6 + 0.1 * i)
                phase = self.motion_phase + i * math.pi / 3.0
                y_spacing = 0.8
                if self.motion_mode in ('crossing', 'linear_crossing'):
                    x_spacing = 0.8
                    cx = (self.crossing_x -
                          (self.moving_count - 1) * x_spacing / 2.0 +
                          i * x_spacing)
                    vx = 0.0
                    if self.motion_mode == 'linear_crossing':
                        delta_y = self.crossing_end_y - self.crossing_start_y
                        span = abs(delta_y)
                        direction = 1.0 if delta_y >= 0.0 else -1.0
                        progress = min(max(t, 0.0) * self.crossing_speed,
                                       span)
                        cy = self.crossing_start_y + direction * progress
                        vy = (direction * self.crossing_speed
                              if (motion_started and
                                  self.crossing_speed > 1.0e-6 and
                                  progress < span)
                              else 0.0)
                    else:
                        cy = math.sin(omega * t + phase) * amp
                        vy = amp * omega * math.cos(omega * t + phase)
                else:
                    cx = math.sin(omega * t + phase) * amp
                    vx = amp * omega * math.cos(omega * t + phase)
                    cy = (-(self.moving_count - 1) * y_spacing / 2.0 +
                          i * y_spacing)
                    vy = 0.0
                # When synchronization is enabled the obstacle is physically
                # held at t=0, so its published velocity must also be zero.
                if not motion_started:
                    vx = 0.0
                    vy = 0.0
                cz = self.center_z

                size_param = rospy.get_param('~moving_box_size', [0.4, 0.4, 0.6])
                # moving_box_size may come in as a string from launch file - parse safely
                if isinstance(size_param, str):
                    try:
                        size = list(ast.literal_eval(size_param))
                    except Exception:
                        # fallback to default
                        size = [0.4, 0.4, 0.6]
                else:
                    size = list(size_param)

                pts = self.box_points((cx, cy, cz), size, self.box_resolution)
                moving_pts.extend(pts)
                dynamic_points.extend(
                    (x, y, z, vx, vy, 0.0, i, 1.0)
                    for x, y, z in pts)

            # Take the frame and optional static points as one consistent
            # snapshot. The fast point-flow test disables merged publishing to
            # avoid copying a large static cloud at the dynamic update rate.
            with self._lock:
                merged = (list(self.static_points)
                          if self.publish_merged else [])
                map_frame = self.map_frame or 'world'
            if self.publish_merged:
                merged.extend(moving_pts)

            header = Header()
            header.stamp = now
            header.frame_id = map_frame
            if self.publish_merged:
                cloud_msg = pc2.create_cloud_xyz32(header, merged)
                self.pub.publish(cloud_msg)
            # publish moving-only cloud for visualization/debug
            moving_header = Header()
            moving_header.stamp = now
            moving_header.frame_id = header.frame_id
            moving_cloud_msg = pc2.create_cloud_xyz32(moving_header, moving_pts)
            self.moving_pub.publish(moving_cloud_msg)
            dynamic_cloud_msg = pc2.create_cloud(
                moving_header, self.dynamic_point_fields, dynamic_points)
            self.dynamic_points_pub.publish(dynamic_cloud_msg)
            # remember our publish time so the subscriber can ignore the echoed message
            try:
                self._last_pub_time = header.stamp
            except Exception:
                pass

            rate.sleep()


if __name__ == '__main__':
    try:
        node = MovingObstaclesNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

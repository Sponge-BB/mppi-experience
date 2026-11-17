#!/usr/bin/env python3
"""Simple node that merges the static /map_generator/global_cloud
with a set of small moving cuboid obstacle point clouds and republishes
the merged cloud on /map_generator/global_cloud (so downstream nodes
see moving obstacles). This node is only launched by exp3.launch.

Note: make the script executable after creation:
  chmod +x path/to/moving_obstacles_node.py
"""
from __future__ import print_function
import rospy
from sensor_msgs.msg import PointCloud2
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

        rospy.Subscriber('/map_generator/global_cloud', PointCloud2, self.static_cloud_cb)

        # We publish merged cloud on a separate topic so we don't race with the
        # original map_generator publisher. For exp3 we remap planner subscribers
        # to this topic so planners see the merged view.
        self.merged_topic = rospy.get_param('~merged_topic', '/map_generator/global_cloud_merged')
        self.pub = rospy.Publisher(self.merged_topic, PointCloud2, queue_size=1)
        # also publish moving-only pointcloud so we can visualize moving obstacles
        self.moving_only_topic = rospy.get_param('~moving_only_topic', '/moving_obstacles_only')
        self.moving_pub = rospy.Publisher(self.moving_only_topic, PointCloud2, queue_size=1)

        # moving object motion params
        self.map_x = rospy.get_param('/map/x_size', 8.0)
        self.map_y = rospy.get_param('/map/y_size', 8.0)

        # define simple trajectories for each moving box
        self.start_time = rospy.Time.now()

    def static_cloud_cb(self, msg):
        # ignore messages that we ourselves just published (avoid feedback loop)
        try:
            if msg.header.stamp == self._last_pub_time:
                return
        except Exception:
            pass

        # store static points as tuples (x,y,z)
        frame = msg.header.frame_id if hasattr(msg.header, 'frame_id') else 'world'
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
            t = (now - self.start_time).to_sec()

            # build moving box centers using simple trajectories
            moving_pts = []
            # left-right (x-axis) oscillation for each moving box
            for i in range(self.moving_count):
                # amplitude depends on map size but keep a safe margin
                amp = max(0.5, min(self.map_x * 0.2, 2.0))
                omega = 0.6 + 0.1 * i
                phase = i * math.pi / 3.0
                cx = math.sin(omega * t + phase) * amp
                # spread boxes along y to avoid overlap
                y_spacing = 0.8
                cy = - (self.moving_count - 1) * y_spacing / 2.0 + i * y_spacing
                cz = 0.9  # fixed height for boxes

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

            # merge static and moving
            merged = list(self.static_points) if self.static_points else []
            merged.extend(moving_pts)

            header = Header()
            header.stamp = now
            # read frame under lock to be consistent with callback
            with self._lock:
                header.frame_id = self.map_frame
            cloud_msg = pc2.create_cloud_xyz32(header, merged)
            self.pub.publish(cloud_msg)
            # publish moving-only cloud for visualization/debug
            moving_header = Header()
            moving_header.stamp = now
            moving_header.frame_id = header.frame_id
            moving_cloud_msg = pc2.create_cloud_xyz32(moving_header, moving_pts)
            self.moving_pub.publish(moving_cloud_msg)
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

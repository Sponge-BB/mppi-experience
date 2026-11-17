#!/usr/bin/env python3
"""Headless standby plant: independent integration; truth is sensing/audit only."""
import csv
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_variable] = '1'
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs import point_cloud2
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Header

from mppi.active_arm_model import PiperArmModel


Q_START = np.deg2rad([0., 110., -60., 0., 30., 0.])
NAMES = ['joint%d' % i for i in range(1, 7)]
STEP = .005
MEASUREMENT_DELAY = .020
COMMAND_WATCHDOG = .30
SENSOR_MODELS = ('ideal', 'mid360')


def actor_at(elapsed, scene):
    if scene == 'empty' or elapsed < 2.:
        return None
    x = .45 if scene.startswith('navigation_') else .16
    initial_y = .95 if scene == 'navigation_arm_enter_blind' else .65
    return np.array([x, initial_y-.08*(elapsed-2.), .13, 1.05, 1.30])


def cylinder_gap(spheres, actor):
    """Analytic signed distance to a solid finite cylinder, minus sphere radii."""
    if actor is None:
        return float('inf')
    x, y, radius, low, high = actor
    radial = np.hypot(spheres[:, 0]-x, spheres[:, 1]-y)-radius
    vertical = np.abs(spheres[:, 2]-(low+high)/2.)-(high-low)/2.
    signed = (np.hypot(np.maximum(radial, 0.), np.maximum(vertical, 0.))
              + np.minimum(np.maximum(radial, vertical), 0.))
    return float(np.min(signed-spheres[:, 3]))


def obstacle_points(actor, spacing=.025):
    # Same side/cap surface sampling as the prior standby_arm_scene experiment.
    if actor is None:
        return np.empty((0, 3), dtype=np.float32)
    x, y, radius, low, high = actor
    angles = np.linspace(0., 2.*np.pi, max(12, math.ceil(2.*np.pi*radius/spacing)), endpoint=False)
    heights = np.linspace(low, high, max(2, math.ceil((high-low)/spacing))+1)
    points = [(x+radius*np.cos(a), y+radius*np.sin(a), z) for a in angles for z in heights]
    grid = np.arange(-radius, radius+.5*spacing, spacing)
    points += [(x+a, y+b, z) for z in (low, high) for a in grid for b in grid if a*a+b*b <= radius*radius]
    return np.asarray(points, dtype=np.float32)


def cylinder_occludes(points, origin, actor):
    """True when a finite solid cylinder intercepts the ray before its endpoint.

    The 10 micrometre endpoint tolerance keeps float32 surface samples from
    occluding themselves. This also removes wall points behind the cylinder.
    """
    points = np.asarray(points, dtype=float)
    if actor is None or not len(points):
        return np.zeros(len(points), dtype=bool)
    x, y, radius, low, high = actor
    direction = points-origin
    offset = np.asarray(origin[:2])-np.array([x, y])
    a = np.sum(direction[:, :2]**2, axis=1)
    b = 2.*(direction[:, :2] @ offset)
    c = float(offset @ offset-radius**2)
    radial_low, radial_high = np.full(len(points), -np.inf), np.full(len(points), np.inf)
    moving = a > 1e-24
    discriminant = b*b-4.*a*c
    radial_hit = np.where(moving, discriminant >= 0., c <= 0.)
    roots = np.sqrt(np.maximum(discriminant[moving], 0.))
    radial_low[moving] = (-b[moving]-roots)/(2.*a[moving])
    radial_high[moving] = (-b[moving]+roots)/(2.*a[moving])
    z_low, z_high = np.full(len(points), -np.inf), np.full(len(points), np.inf)
    moving_z = np.abs(direction[:, 2]) > 1e-12
    z_hit = moving_z | ((origin[2] >= low) & (origin[2] <= high))
    first = (low-origin[2])/direction[moving_z, 2]
    last = (high-origin[2])/direction[moving_z, 2]
    z_low[moving_z], z_high[moving_z] = np.minimum(first, last), np.maximum(first, last)
    entry = np.maximum(np.maximum(radial_low, z_low), 0.)
    leave = np.minimum(np.minimum(radial_high, z_high), 1.)
    tolerance = 1e-5/np.maximum(np.linalg.norm(direction, axis=1), 1e-12)
    return radial_hit & z_hit & (leave >= entry) & (leave > 0.) & (entry < 1.-tolerance)


def sensor_cloud(static_points, actor, state, lidar_offset, base_z, sensor_model):
    """Sampled geometric visibility, not a complete MID360 sensor simulator.

    mid360 applies a 360 degree azimuth / [-7, 52] degree elevation envelope,
    0.1 m minimum range and finite-cylinder occlusion. The base is planar and
    the original lidar mounting has zero roll/pitch/yaw. Robot occlusion,
    non-repeating scan pattern, return noise and reflectivity are not modeled.
    The separately published complete static map is a known prior.
    """
    if sensor_model not in SENSOR_MODELS:
        raise ValueError('unknown sensor model')
    obstacle = obstacle_points(actor)
    points = np.concatenate([static_points, obstacle])
    c, s = math.cos(state[2]), math.sin(state[2])
    inverse_rotation = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])
    base = np.array([state[0], state[1], base_z])
    origin = base+inverse_rotation.T @ lidar_offset
    local = (points-base) @ inverse_rotation.T-lidar_offset
    ranges = np.linalg.norm(local, axis=1)
    elevation = np.rad2deg(np.arctan2(local[:, 2], np.hypot(local[:, 0], local[:, 1])))
    fov = np.ones(len(points), dtype=bool)
    visible = fov.copy()
    if sensor_model == 'mid360':
        fov = ((ranges >= .1-1e-10) & (elevation >= -7.-1e-9) & (elevation <= 52.+1e-9))
        visible = fov.copy()
        visible[fov] &= ~cylinder_occludes(points[fov], origin, actor)
    n_static = len(static_points)
    actor_fov, actor_visible = fov[n_static:], visible[n_static:]
    stats = dict(sensor_model=sensor_model, actor_present=actor is not None,
        sensor_world_x_m=float(origin[0]), sensor_world_y_m=float(origin[1]), sensor_world_z_m=float(origin[2]),
        actor_sampled_points=len(obstacle), actor_in_fov_points=int(actor_fov.sum()),
        actor_visible_points=int(actor_visible.sum()),
        actor_visible_fraction=float(actor_visible.mean()) if len(obstacle) else None,
        actor_fov_rejected_points=int((~actor_fov).sum()),
        actor_occluded_points=int((actor_fov & ~actor_visible).sum()),
        static_sampled_points=n_static, static_visible_points=int(visible[:n_static].sum()))
    for prefix, values in (
            ('actor_range', ranges[n_static:]), ('actor_elevation', elevation[n_static:]),
            ('actor_visible_range', ranges[n_static:][actor_visible]),
            ('actor_visible_elevation', elevation[n_static:][actor_visible]),
            ('actor_visible_z', obstacle[actor_visible, 2])):
        unit = 'deg' if 'elevation' in prefix else 'm'
        stats[prefix+'_min_'+unit] = float(values.min()) if len(values) else None
        stats[prefix+'_max_'+unit] = float(values.max()) if len(values) else None
    return local[visible], stats


class Plant:
    """A bounded lagged plant, deliberately independent of the MPPI rollout."""
    def __init__(self, model, limits, arm_velocity, arm_acceleration):
        self.model, self.limits = model, limits
        self.vmax = np.asarray(arm_velocity, dtype=float)
        self.amax = np.asarray(arm_acceleration, dtype=float)
        for value in (self.vmax, self.amax):
            if value.shape != (6,) or not np.isfinite(value).all() or np.any(value <= 0):
                raise ValueError('six positive finite arm limits required')
        if np.any(self.vmax > model.velocity_limits) or np.any(self.amax > model.acceleration_limits):
            raise ValueError('plant limits exceed model limits')
        for key in ('vmax', 'wmax', 'accel', 'angular_accel', 'track', 'wheel_v', 'wheel_a'):
            if not math.isfinite(limits[key]) or limits[key] <= 0:
                raise ValueError('invalid base limit '+key)
        self.state = np.r_[np.zeros(5), Q_START.copy(), np.zeros(6)]
        if not model.check(Q_START)['valid']:
            raise ValueError('initial posture rejected by current model')

    def step(self, base_target, joint_target, dt=STEP):
        if not 0 < dt <= STEP:
            raise ValueError('integration step exceeds 5 ms')
        z, c = self.state, self.limits
        h = c['track']/2.
        u = np.asarray(base_target, dtype=float).copy()
        u /= max(1., abs(u[0])/c['vmax'], abs(u[1])/c['wmax'],
                 abs(u[0]-h*u[1])/c['wheel_v'], abs(u[0]+h*u[1])/c['wheel_v'])
        acceleration = (u-z[3:5])/.12
        acceleration /= max(1., abs(acceleration[0])/c['accel'], abs(acceleration[1])/c['angular_accel'],
                            abs(acceleration[0]-h*acceleration[1])/c['wheel_a'],
                            abs(acceleration[0]+h*acceleration[1])/c['wheel_a'])
        velocity = z[3:5]+dt*acceleration
        v, w = (velocity+z[3:5])/2.
        yaw_mid = z[2]+dt*w/2.
        z[:2] += dt*v*np.array([math.cos(yaw_mid), math.sin(yaw_mid)])
        z[2] = math.atan2(math.sin(z[2]+dt*w), math.cos(z[2]+dt*w))
        z[3:5] = velocity
        q, dq = z[5:11], z[11:17]
        desired = np.zeros(6) if joint_target is None else (joint_target-q)/.12
        lower_speed = -np.minimum(self.vmax, np.sqrt(2*self.amax*np.maximum(q-self.model.lower, 0.)))
        upper_speed = np.minimum(self.vmax, np.sqrt(2*self.amax*np.maximum(self.model.upper-q, 0.)))
        desired = np.clip(desired, lower_speed, upper_speed)
        ddq = np.clip((desired-dq)/.12, -self.amax, self.amax)
        next_dq = dq+dt*ddq
        z[5:11] = q+dt*(dq+next_dq)/2.
        z[11:17] = next_dq
        return ddq


class StandbySimulator:
    def __init__(self):
        self.out = Path(rospy.get_param('~out')); self.out.mkdir(parents=True, exist_ok=True)
        if (self.out/'state.csv').exists():
            raise ValueError('output already contains state.csv')
        self.scene = rospy.get_param('~scene', 'arm_crossing')
        self.sensor_model = rospy.get_param('~sensor_model', 'mid360')
        self.base_z = float(rospy.get_param('~base_z', .115))
        if self.sensor_model not in SENSOR_MODELS or not math.isfinite(self.base_z):
            raise ValueError('invalid sensor model or base height')
        self.factor = float(rospy.get_param('~realtime_factor', 1.))
        self.duration = float(rospy.get_param('~duration', 24.))
        self.ready_timeout = float(rospy.get_param('~ready_timeout_wall_s', 120.))
        if (self.scene not in ('empty', 'arm_crossing', 'navigation_arm_crossing', 'navigation_arm_enter_blind')
                or self.factor not in (.5, 1.) or not 0 < self.duration <= 120):
            raise ValueError('invalid scene, duration or realtime_factor')
        self.goal = np.asarray(rospy.get_param('~goal', []), dtype=float)
        if self.goal.shape not in ((0,), (3,)) or not np.isfinite(self.goal).all():
            raise ValueError('goal must be empty or finite x/y/yaw')
        if not math.isfinite(self.ready_timeout) or self.ready_timeout <= 0: raise ValueError('invalid readiness timeout')
        self.model = PiperArmModel.from_parameters(rospy.get_param('/remani_planner_node'))
        self.model.lower[5] = max(self.model.lower[5], -.35); self.model.upper[5] = min(self.model.upper[5], .35)
        self.plant = Plant(self.model, rospy.get_param('~base_limits'),
                           rospy.get_param('~arm_velocity'), rospy.get_param('~arm_acceleration'))
        self.lock = threading.Lock(); self.t = 0.; self.start_time = None; self.start_requested = False
        # Save actual measured states; never attach an old stamp to the current state.
        self.measurements = deque(maxlen=1+int(round(MEASUREMENT_DELAY/STEP)))
        self.base_target = np.zeros(2); self.joint_target = Q_START.copy()
        self.base_stamp = self.joint_stamp = -1e9
        self.command_counts = dict(base=0, arm=0, rejected=0)
        self.clock = rospy.Publisher('/clock', Clock, queue_size=2)
        self.odom = rospy.Publisher('/sim/odom', Odometry, queue_size=2)
        self.joints = rospy.Publisher('/sim/joint_states', JointState, queue_size=2)
        self.cloud = rospy.Publisher('/sim/cloud', PointCloud2, queue_size=1)
        self.static_cloud = rospy.Publisher('/sim/static_cloud', PointCloud2, queue_size=1, latch=True)
        self.static_sent = False
        self.gripper = rospy.Publisher('/sim/gripper_state', Bool, queue_size=1, latch=True)
        self.active = rospy.Publisher('/sim/active', Bool, queue_size=1, latch=True)
        self.tf = tf2_ros.TransformBroadcaster(); self.static_tf = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber('/sim/cmd_vel', Twist, self.on_base, queue_size=1)
        rospy.Subscriber('/sim/joint_ctrl_single', JointState, self.on_arm, queue_size=1)
        rospy.Subscriber('/sim/start', Bool, self.on_start, queue_size=1)
        offset = np.asarray(rospy.get_param('~lidar_offset', [.28, 0., .002]), dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all(): raise ValueError('invalid lidar offset')
        self.lidar_offset = offset
        transforms = []
        for parent, child, xyz in [('world', 'odom', [0., 0., 0.]), ('base_link', 'livox_frame', offset)]:
            transform = TransformStamped(); transform.header.frame_id = parent; transform.child_frame_id = child
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = xyz
            transform.transform.rotation.w = 1.; transforms.append(transform)
        self.static_tf.sendTransform(transforms)
        # Known complete static-map prior, not a claim of current sensor visibility.
        # Live wall returns below are visibility-filtered; truth uses full walls.
        self.static_points = np.array([(x, y, z) for x in np.arange(-1.2, 1.201, .08)
            for y in (-1.4, 1.4) for z in np.arange(.1, 1.61, .12)]+
            [(x, y, z) for x in (-1.2, 1.2) for y in np.arange(-1.4, 1.401, .08)
             for z in np.arange(.1, 1.61, .12)], dtype=np.float32)
        self.streams = [(self.out/(name+'.csv')).open('w', buffering=1)
                        for name in ('state', 'obstacles', 'sensor_visibility')]
        self.state_csv, self.obstacle_csv = [csv.writer(stream) for stream in self.streams[:2]]
        self.visibility_csv = None
        self.state_csv.writerow(['t', 'x', 'y', 'yaw', 'v', 'w']+NAMES+['dq%d'%i for i in range(1,7)]
                                +['external_gap_m', 'configuration_gap_m', 'command_v', 'command_w'])
        self.obstacle_csv.writerow(['t', 'x', 'y', 'r', 'id', 'vx', 'vy', 'z_min', 'z_max'])
        self.samples = 0; self.minimum_external = self.minimum_configuration = float('inf')
        self.max_base_displacement = self.base_path = self.max_yaw = self.yaw_path = self.max_arm = 0.
        self.last_record = self.plant.state.copy(); self.first_violation = None

    def on_start(self, message):
        if message.data:
            with self.lock: self.start_requested = True

    def on_base(self, message):
        command = np.array([message.linear.x, message.angular.z])
        with self.lock:
            if not np.isfinite(command).all(): self.command_counts['rejected'] += 1; return
            self.base_target = command; self.base_stamp = self.t; self.command_counts['base'] += 1

    def on_arm(self, message):
        now = rospy.get_time(); stamp = message.header.stamp.to_sec()
        with self.lock:
            try:
                indices = [message.name.index(name) for name in NAMES]
                q = np.asarray(message.position, dtype=float)[indices]
                if (not np.isfinite(q).all() or not 0. < stamp <= now+.01 or now-stamp > COMMAND_WATCHDOG
                        or np.any(q < self.model.lower) or np.any(q > self.model.upper)):
                    raise ValueError('invalid target')
                self.joint_target = q; self.joint_stamp = self.t; self.command_counts['arm'] += 1
            except (ValueError, IndexError): self.command_counts['rejected'] += 1

    def targets(self):
        with self.lock:
            if self.start_time is None: return np.zeros(2), None
            return (self.base_target.copy() if 0 <= self.t-self.base_stamp < COMMAND_WATCHDOG else np.zeros(2),
                    self.joint_target.copy() if 0 <= self.t-self.joint_stamp < COMMAND_WATCHDOG else None)

    def publish_state(self, stamp, z):
        odom = Odometry(); odom.header = Header(stamp=stamp, frame_id='odom'); odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = z[:2]
        odom.pose.pose.position.z = self.base_z
        odom.pose.pose.orientation.z = math.sin(z[2]/2.); odom.pose.pose.orientation.w = math.cos(z[2]/2.)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = z[3:5]; self.odom.publish(odom)
        joints = JointState(header=Header(stamp=stamp, frame_id='base_link'), name=NAMES,
                            position=z[5:11].tolist(), velocity=z[11:17].tolist(), effort=[0.]*6)
        self.joints.publish(joints)
        transform = TransformStamped(); transform.header = odom.header; transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = z[:2]
        transform.transform.translation.z = self.base_z
        transform.transform.rotation = odom.pose.pose.orientation; self.tf.sendTransform(transform)

    def audit(self, elapsed, actor, ddq):
        z = self.plant.state; spheres = self.model.body_spheres(z[5:11], z[:3])
        wall_gap = float(np.min(np.minimum(1.2-np.abs(spheres[:, 0]), 1.4-np.abs(spheres[:, 1]))-spheres[:, 3]))
        external = min(wall_gap, cylinder_gap(spheres, actor)); check = self.model.check(z[5:11], z[11:17], ddq)
        configuration = min(check['min_self_gap'], check['min_base_arm_gap'], check['min_ground_gap'])
        self.minimum_external = min(self.minimum_external, external); self.minimum_configuration = min(self.minimum_configuration, configuration)
        rate_valid = np.all(np.abs(z[11:17]) <= self.plant.vmax+1e-10) and np.all(np.abs(ddq) <= self.plant.amax+1e-10)
        if external < 0. or not check['valid'] or not rate_valid:
            reason = 'collision' if external < 0. or configuration < -1e-10 else 'joint_limit_violation'
            self.first_violation = dict(t=elapsed, reason=reason, external_gap_m=external,
                                       geometry=check, state=z.tolist())
            return external, configuration, reason
        return external, configuration, None

    def record(self, elapsed, actor, external, configuration, command):
        z = self.plant.state
        self.state_csv.writerow([elapsed, *z, external, configuration, *command])
        if actor is not None:
            x, y, radius, low, high = actor
            self.obstacle_csv.writerow([elapsed, x, y, radius, 1, 0., -.08, low, high])
        self.max_base_displacement = max(self.max_base_displacement, float(np.linalg.norm(z[:2])))
        self.base_path += float(np.linalg.norm(z[:2]-self.last_record[:2]))
        delta_yaw = math.atan2(math.sin(z[2]-self.last_record[2]), math.cos(z[2]-self.last_record[2]))
        self.yaw_path += abs(delta_yaw); self.max_yaw = max(self.max_yaw, abs(z[2]))
        self.max_arm = max(self.max_arm, float(np.max(np.abs(z[5:11]-Q_START))))
        self.last_record = z.copy(); self.samples += 1

    def run(self):
        wall_start = time.monotonic(); tick = 0; reason = 'shutdown'; error = None
        self.active.publish(Bool(False)); self.gripper.publish(Bool(False))
        try:
            while not rospy.is_shutdown():
                with self.lock:
                    if self.start_time is None and self.start_requested:
                        self.start_time = self.t; self.base_stamp = self.joint_stamp = -1e9
                        self.active.publish(Bool(True))
                command, q_target = self.targets(); ddq = self.plant.step(command, q_target)
                tick += 1; self.t = tick*STEP
                elapsed = -1. if self.start_time is None else self.t-self.start_time
                actor = actor_at(elapsed, self.scene)
                stamp = rospy.Time.from_sec(1000.+self.t); self.clock.publish(Clock(stamp))
                self.measurements.append((self.t, self.plant.state.copy(), None if actor is None else actor.copy()))
                measurement_ready = len(self.measurements) == self.measurements.maxlen
                measured_t, measured_z, measured_actor = self.measurements[0]
                measured_stamp = rospy.Time.from_sec(1000.+measured_t)
                if measurement_ready and tick % 2 == 0: self.publish_state(measured_stamp, measured_z)
                if measurement_ready and tick % 20 == 0:
                    if not self.static_sent:
                        self.static_cloud.publish(point_cloud2.create_cloud_xyz32(
                            Header(stamp=measured_stamp, frame_id='world'), self.static_points))
                        self.static_sent = True
                    local, visibility = sensor_cloud(self.static_points, measured_actor, measured_z,
                        self.lidar_offset, self.base_z, self.sensor_model)
                    visibility.update(t=None if self.start_time is None else measured_t-self.start_time,
                        publish_t=None if self.start_time is None else self.t-self.start_time,
                        stamp_ros_s=1000.+measured_t)
                    if self.visibility_csv is None:
                        self.visibility_csv = csv.DictWriter(self.streams[2], fieldnames=list(visibility))
                        self.visibility_csv.writeheader()
                    self.visibility_csv.writerow(visibility)
                    self.cloud.publish(point_cloud2.create_cloud_xyz32(Header(stamp=measured_stamp, frame_id='livox_frame'), local))
                external, configuration, failure = self.audit(elapsed, actor, ddq)
                if self.start_time is not None and (tick % 4 == 0 or failure or elapsed >= self.duration):
                    self.record(elapsed, actor, external, configuration, command)
                if failure: reason = failure; break
                if self.start_time is not None and elapsed >= self.duration: reason = 'completed'; break
                if self.start_time is None and time.monotonic()-wall_start > self.ready_timeout:
                    reason = 'readiness_timeout'; break
                time.sleep(max(0., wall_start+self.t/self.factor-time.monotonic()))
        except Exception as exc:
            reason = 'error'; error = type(exc).__name__+':'+str(exc)
            raise
        finally:
            self.active.publish(Bool(False))
            for stream in self.streams: stream.close()
            z = self.plant.state
            result = dict(reason=reason, error=error, scene=self.scene, completed=reason == 'completed',
                collision_physical=reason == 'collision', first_violation=self.first_violation,
                elapsed_s=None if self.start_time is None else self.t-self.start_time, simulated_s=self.t,
                wall_s=time.monotonic()-wall_start, requested_realtime_factor=self.factor, samples=self.samples,
                integration_step_s=STEP, measurement_delay_s=MEASUREMENT_DELAY,
                command_watchdog_s=COMMAND_WATCHDOG,
                sensor_model=self.sensor_model, base_link_ground_z_m=self.base_z,
                base_to_lidar_xyz_m=self.lidar_offset.tolist(),
                lidar_ground_z_m=self.base_z+float(self.lidar_offset[2]),
                static_map_is_complete_known_prior=True,
                sensing_scope='sampled surfaces; mid360 adds geometric FOV/minimum range and finite-cylinder occlusion; '
                              'no robot occlusion, scan pattern, reflectivity or noise model; no physical calibration',
                physical_base_locked=False, commands=self.command_counts,
                min_external_gap_m=self.minimum_external if math.isfinite(self.minimum_external) else None,
                min_configuration_gap_m=self.minimum_configuration if math.isfinite(self.minimum_configuration) else None,
                max_base_displacement_m=self.max_base_displacement, base_path_length_m=self.base_path,
                max_base_yaw_change_rad=self.max_yaw, base_cumulative_turn_rad=self.yaw_path,
                final_base_error_m=float(np.linalg.norm(z[:2])), max_arm_change_deg=math.degrees(self.max_arm),
                final_arm_error_deg=float(np.rad2deg(np.max(np.abs(z[5:11]-Q_START)))), final_state=z.tolist(),
                scope='independent 5ms numerical plant; sphere/cylinder truth and current local model; no mesh or hardware certification')
            if len(self.goal):
                result.update(navigation_goal=self.goal.tolist(),
                    final_goal_position_error_m=float(np.linalg.norm(z[:2]-self.goal[:2])),
                    final_goal_yaw_error_rad=abs(math.atan2(math.sin(z[2]-self.goal[2]),math.cos(z[2]-self.goal[2]))))
            (self.out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    master = urlparse(os.environ.get('ROS_MASTER_URI', ''))
    if master.hostname not in ('localhost', '127.0.0.1') or master.port != 11466:
        raise RuntimeError('simulation requires isolated localhost ROS master port 11466')
    rospy.init_node('standby_simulator')
    if not rospy.get_param('/use_sim_time', False): raise RuntimeError('/use_sim_time must be true')
    StandbySimulator().run()

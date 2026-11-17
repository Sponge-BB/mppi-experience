#!/usr/bin/env python3
"""Read-only standby MPPI observer; there is deliberately no execution mode."""
import argparse
import json
import math
from pathlib import Path
import re
import sys
import threading
import time

import numpy as np

from hardware_cloud_snapshot import (DynamicObservation, HardwareCloudSnapshot,
    JointPositionDifferencer, SurfaceObservation, finite_stamp, fresh,
    transform_points)
from pointcloud_numpy import read_points_numpy
from odometry_continuity import OdometryContinuityGate, OdometryGateConfig


def quaternion_matrix(q):
    return transform_points(np.empty((0, 3)), [0., 0., 0.], q)[1]


class HardwareShadowNode:
    def __init__(self, release, namespace):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Point
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState, PointCloud2
        from std_msgs.msg import Bool, Float64MultiArray, String
        from visualization_msgs.msg import Marker, MarkerArray
        from active_arm_model import PiperArmModel
        from standby_whole_body_mppi import StandbyWholeBodyMPPI
        self.rospy = rospy
        self.Marker, self.MarkerArray, self.Point = Marker, MarkerArray, Point
        self.String, self.Float64MultiArray = String, Float64MultiArray
        self.release = release
        self.config = json.loads((release/'config/hardware_shadow.json').read_text())
        self.namespace = namespace.rstrip('/')
        if not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*', self.namespace) or not self.namespace.startswith('/standby_trial_'):
            raise ValueError('shadow namespace must start with /standby_trial_')
        self.world = self.config.get('world_frame', 'odom')
        self.base_frame = self.config.get('base_frame', 'base_link')
        self.max_age = float(self.config.get('max_age_s', .25))
        if not 0 < self.max_age <= .5:
            raise ValueError('invalid max_age_s')
        self.model = PiperArmModel.from_parameters(release/'model/parameters.yaml', release/'model/mm.urdf')
        limits = self.config.get('limits')
        if limits is None:
            limits = json.loads((release/'config/limits.json').read_text())
        al = self.config.get('arm_limits')
        if al:
            values = [np.asarray(al[k], dtype=float) for k in ('lower', 'upper', 'velocity', 'acceleration')]
            self.model.lower, self.model.upper, self.model.velocity_limits, self.model.acceleration_limits = values
        self.model.lower[5] = max(self.model.lower[5], -.35)
        self.model.upper[5] = min(self.model.upper[5], .35)
        arm_limits = [self.model.lower, self.model.upper, self.model.velocity_limits, self.model.acceleration_limits]
        self.kernel = StandbyWholeBodyMPPI(self.model, limits, arm_limits,
            self.config.get('postures', []), enabled=True, seed=self.config.get('seed', 11),
            horizon=self.config.get('horizon', 20), step=self.config.get('step', .1),
            random_samples=self.config.get('random_samples', 6), config=self.config.get('kernel_config'))
        self.differencer = JointPositionDifferencer(self.model.lower, self.model.upper,
            self.model.velocity_limits, max_dt=self.max_age)
        self.lock = threading.Lock()
        self.base = self.q = self.dq = None
        self.odom_stamp = self.joint_stamp = 0.
        self.odom_z_ignored = None
        self.clouds = {}; self.errors = {}; self.receipt = {}; self.latencies = {}
        self.cloud_profiles = {}
        self.odom_gate = OdometryContinuityGate(OdometryGateConfig(
            **self.config.get('odom_validation', {})))
        self.healthy = False; self.health_stamp = 0.
        self.anchor = self.arm_reference = None
        self.stationary_since = None
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(15.))
        self.tf_listener = tf2_ros.TransformListener(self.tf)
        self.topics = dict(odom='/odom', joints='/joint_states_single', raw_cloud='/livox/lidar_pcl2',
            filtered_cloud='/velodyne_points', dynamic_cloud='/dynamic_points',
            static_cloud='/map_generator/global_cloud', perception_health='/dynamic_obstacle_tracker/perception_healthy')
        self.topics.update(self.config.get('topics', {}))
        # All publishers are visualization/diagnostic topics under our prefix.
        self.status_pub = rospy.Publisher(self.namespace+'/status', String, queue_size=1, latch=True)
        self.markers_pub = rospy.Publisher(self.namespace+'/markers', MarkerArray, queue_size=1)
        self.proposed_pub = rospy.Publisher(self.namespace+'/proposed_controls_never_executed', Float64MultiArray, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(self.topics['odom'], Odometry, self.odom, queue_size=1),
            rospy.Subscriber(self.topics['joints'], JointState, self.joints, queue_size=1),
            rospy.Subscriber(self.topics['perception_health'], Bool, self.health, queue_size=1)]
        for key in ('raw_cloud', 'filtered_cloud', 'static_cloud', 'dynamic_cloud'):
            self.subscribers.append(rospy.Subscriber(self.topics[key], PointCloud2,
                lambda msg, source=key: self.cloud(msg, source), queue_size=1, buff_size=8*1024**2))
        log_dir = Path(rospy.get_param('~log_dir', str(release/'logs')))
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir/('shadow_%d.jsonl' % time.time_ns())
        self.log = self.log_path.open('x', buffering=1)
        self.started = time.monotonic()

    def error(self, source, error):
        with self.lock:
            self.errors[source] = str(error)
            self.receipt[source] = time.monotonic()

    def record(self, source, stamp):
        self.errors.pop(source, None)
        self.receipt[source] = time.monotonic()
        self.latencies[source] = self.rospy.get_time()-stamp

    def odom(self, msg):
        try:
            stamp = finite_stamp(msg.header.stamp.to_sec())
            if msg.child_frame_id != self.base_frame:
                raise ValueError('unexpected_odom_child_frame')
            q = msg.pose.pose.orientation
            rotation = quaternion_matrix([q.x, q.y, q.z, q.w])
            p = msg.pose.pose.position
            xyz = np.array([[p.x, p.y, p.z]])
            if msg.header.frame_id != self.world:
                tr = self.tf.lookup_transform(self.world, msg.header.frame_id,
                    msg.header.stamp, self.rospy.Duration(.02)).transform
                xyz, r = transform_points(xyz, [tr.translation.x, tr.translation.y, tr.translation.z],
                    [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
                rotation = r @ rotation
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            base = np.array([xyz[0, 0], xyz[0, 1], yaw, msg.twist.twist.linear.x, msg.twist.twist.angular.z])
            if not np.isfinite(base).all() or not fresh(stamp, self.rospy.get_time(), self.max_age):
                raise ValueError('invalid_or_stale_odom')
            with self.lock:
                self.odom_gate.validate(stamp, self.rospy.get_time(), time.monotonic(),
                    xyz[0], [q.x, q.y, q.z, q.w])
                if stamp <= self.odom_stamp:
                    raise ValueError('nonmonotonic_odom_stamp')
                self.base = base; self.odom_stamp = stamp; self.odom_z_ignored = float(xyz[0, 2])
                self.record('odom', stamp)
        except Exception as exc:
            with self.lock:
                self.base = None
                self.anchor = self.arm_reference = None
                self.stationary_since = None
                self.errors['odom'] = str(exc)
                self.receipt['odom'] = time.monotonic()

    def joints(self, msg):
        try:
            stamp = finite_stamp(msg.header.stamp.to_sec())
            if not fresh(stamp, self.rospy.get_time(), self.max_age):
                raise ValueError('joint_stale')
            with self.lock:
                q, dq = self.differencer.update(list(msg.name), list(msg.position), stamp,
                    frame=msg.header.frame_id, allowed_frames=tuple(self.config.get('joint_frames', ['', self.base_frame])))
                self.q, self.dq, self.joint_stamp = q, dq, stamp
                self.record('joints', stamp)
        except Exception as exc:
            # Piper publishes at 200 Hz. Accumulate position differences over
            # at least 10 ms without treating intermediate frames as failures;
            # the last accepted state retains its original measurement stamp.
            if str(exc) == 'joint_dt_too_short':
                return
            self.error('joints', exc)

    def health(self, msg):
        with self.lock:
            self.healthy = bool(msg.data); self.health_stamp = self.rospy.get_time()
            self.record('perception_health', self.health_stamp)

    def cloud(self, msg, source):
        begin = time.perf_counter()
        profile = dict(callback_ros_stamp=self.rospy.get_time(), source=source,
            width=int(msg.width), height=int(msg.height), wire_bytes=len(msg.data),
            decode_ms=0., tf_lookup_ms=0., transform_ms=0., surface_ms=0., lock_wait_ms=0.)
        try:
            stamp = finite_stamp(msg.header.stamp.to_sec())
            profile['message_stamp'] = stamp
            profile['age_at_callback_entry_s'] = profile['callback_ros_stamp']-stamp
            if source != 'static_cloud' and not fresh(stamp, self.rospy.get_time(), self.max_age):
                raise ValueError('cloud_stale')
            if not msg.header.frame_id:
                raise ValueError('cloud_frame_missing')
            if source == 'dynamic_cloud':
                fields = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'confidence', 'track_id')
                schema = {field.name: field for field in msg.fields}
                if any(name not in schema for name in fields):
                    raise ValueError('dynamic_fields_missing')
                if any(schema[name].datatype != 7 or schema[name].count != 1 for name in fields[:-1]) or schema['track_id'].datatype not in (5, 6):
                    raise ValueError('dynamic_field_types_invalid')
                stage = time.perf_counter()
                rows = np.asarray(read_points_numpy(msg, fields), dtype=float).reshape((-1, 8))
                profile['decode_ms'] = 1000.*(time.perf_counter()-stage)
                if msg.header.frame_id != self.world:
                    # Absolute velocities may rotate between fixed world/odom
                    # frames. Sensor-relative velocities are never accepted.
                    if msg.header.frame_id not in ('world', 'map', 'odom') or self.world not in ('world', 'map', 'odom'):
                        raise ValueError('dynamic_absolute_velocity_frame_required')
                    stage = time.perf_counter()
                    tr = self.tf.lookup_transform(self.world, msg.header.frame_id,
                        msg.header.stamp, self.rospy.Duration(.02)).transform
                    profile['tf_lookup_ms'] = 1000.*(time.perf_counter()-stage)
                    stage = time.perf_counter()
                    rows[:, :3], rotation = transform_points(rows[:, :3],
                        [tr.translation.x, tr.translation.y, tr.translation.z],
                        [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
                    rows[:, 3:6] = rows[:, 3:6] @ rotation.T
                    profile['transform_ms'] = 1000.*(time.perf_counter()-stage)
                stage = time.perf_counter()
                value = DynamicObservation.create(rows, stamp, self.world, self.world)
                profile['surface_ms'] = 1000.*(time.perf_counter()-stage)
            else:
                stage = time.perf_counter()
                points = np.asarray(read_points_numpy(msg, ('x', 'y', 'z')), dtype=float).reshape((-1, 3))
                profile['decode_ms'] = 1000.*(time.perf_counter()-stage)
                if msg.header.frame_id != self.world:
                    stage = time.perf_counter()
                    tr = self.tf.lookup_transform(self.world, msg.header.frame_id,
                        msg.header.stamp, self.rospy.Duration(.02)).transform
                    profile['tf_lookup_ms'] = 1000.*(time.perf_counter()-stage)
                    stage = time.perf_counter()
                    points, _ = transform_points(points, [tr.translation.x, tr.translation.y, tr.translation.z],
                        [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])
                    profile['transform_ms'] = 1000.*(time.perf_counter()-stage)
                stage = time.perf_counter()
                value = SurfaceObservation.create(points, stamp, self.world, self.world, source,
                    self.config.get('static_max_points', 15000) if source == 'static_cloud' else self.config.get('cloud_max_points', 4000))
                profile['surface_ms'] = 1000.*(time.perf_counter()-stage)
            stage = time.perf_counter()
            with self.lock:
                profile['lock_wait_ms'] = 1000.*(time.perf_counter()-stage)
                old = self.clouds.get(source)
                if old is not None and stamp <= old.stamp and source != 'static_cloud':
                    raise ValueError('nonmonotonic_cloud_stamp')
                self.clouds[source] = value
                self.record(source, stamp)
                profile['accepted'] = True
        except Exception as exc:
            profile['accepted'] = False
            profile['error'] = str(exc)
            self.error(source, exc)
        finally:
            profile['callback_total_ms'] = 1000.*(time.perf_counter()-begin)
            profile['age_at_callback_exit_s'] = self.rospy.get_time()-profile.get('message_stamp', self.rospy.get_time())
            with self.lock:
                self.cloud_profiles[source] = profile

    def capture(self):
        with self.lock:
            state = None if self.base is None or self.q is None else np.r_[self.base, self.q, self.dq]
            snapshot = HardwareCloudSnapshot(self.clouds.get('raw_cloud'), self.clouds.get('filtered_cloud'),
                self.clouds.get('static_cloud'), self.clouds.get('dynamic_cloud'), self.health_stamp,
                self.healthy, max_age=self.max_age)
            cloud_inputs = {key: dict(stamp=value.stamp, frame=value.frame,
                points=value.point_count if key == 'dynamic_cloud' else len(value.points))
                for key, value in self.clouds.items()}
            return state, snapshot, self.odom_stamp, self.joint_stamp, dict(self.errors), dict(self.latencies), cloud_inputs, dict(self.cloud_profiles)

    def markers(self, state, prediction=None, measured_stamp=None, reason=None):
        Marker, MarkerArray = self.Marker, self.MarkerArray
        arr = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)
        status = Marker(); status.header.frame_id = self.world; status.header.stamp = self.rospy.Time.now()
        status.ns = 'shadow_status'; status.id = 0; status.type = Marker.TEXT_VIEW_FACING; status.action = Marker.ADD
        status.pose.orientation.w = 1.; status.pose.position.z = 1.5
        if state is not None:
            status.pose.position.x, status.pose.position.y = state[:2]
        status.scale.z = .09
        status.color.r, status.color.g, status.color.b, status.color.a = (1., .85, .2, 1.)
        status.text = 'SHADOW ONLY / No actuator output\n'+str(reason or 'waiting_inputs')
        if state is None:
            status.text += '\nWaiting fresh '+self.topics['odom']+' and '+self.topics['joints']
        status.lifetime = self.rospy.Duration(2.); arr.markers.append(status)
        if state is None:
            self.markers_pub.publish(arr)
            return
        spheres = self.model.body_spheres(state[5:11], state[:3])
        for index, sphere in enumerate(spheres):
            m = Marker(); m.header.frame_id = self.world
            m.header.stamp = self.rospy.Time.from_sec(measured_stamp) if measured_stamp else self.rospy.Time.now()
            m.ns = 'unvalidated_measured_geometry'; m.id = index; m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.orientation.w = 1.; m.pose.position.x, m.pose.position.y, m.pose.position.z = sphere[:3]
            m.scale.x = m.scale.y = m.scale.z = 2.*sphere[3]
            m.color.r, m.color.g, m.color.b, m.color.a = ((.25, .65, 1., .35) if index < 47 else (1., .65, .15, .8))
            m.lifetime = self.rospy.Duration(2.); arr.markers.append(m)
        if prediction is not None:
            future = self.model.body_spheres_batch(prediction[:, 5:11], prediction[:, :3])
            for index in (0, 47, 54, 61, 66):
                m = Marker(); m.header.frame_id = self.world; m.header.stamp = self.rospy.Time.now()
                m.ns = 'proposed_not_executed'; m.id = index; m.type = Marker.LINE_STRIP; m.action = Marker.ADD
                m.pose.orientation.w = 1.; m.scale.x = .008
                m.color.r, m.color.g, m.color.b, m.color.a = (.9, .2, 1., .9)
                m.points = [self.Point(*xyz) for xyz in future[:, index, :3]]
                m.lifetime = self.rospy.Duration(2.); arr.markers.append(m)
        self.markers_pub.publish(arr)

    def run(self):
        rospy = self.rospy
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError('hardware shadow requires wall-clock ROS time; no simulated clock')
        rate = rospy.Rate(self.config.get('rate_hz', 2.))
        try:
            while not rospy.is_shutdown():
                begin = time.monotonic(); now = rospy.get_time()
                with self.lock:
                    try:
                        self.odom_gate.expire(begin)
                    except ValueError as exc:
                        self.base = None
                        self.anchor = self.arm_reference = None
                        self.stationary_since = None
                        self.errors['odom'] = str(exc)
                state, snapshot, odom_stamp, joint_stamp, errors, latencies, cloud_inputs, cloud_profiles = self.capture()
                reason = 'waiting_real_state' if state is None else None
                if self.odom_gate.fault is not None:
                    reason = 'odometry_requires_restart'
                if reason is None:
                    if any(key in errors for key in ('odom', 'joints')):
                        reason = 'waiting_valid_inputs'
                    elif not fresh(odom_stamp, now, self.max_age) or not fresh(joint_stamp, now, self.max_age):
                        reason = 'state_stale'
                    elif abs(odom_stamp-joint_stamp) > .10:
                        reason = 'state_time_misalignment'
                if state is not None and reason is None and self.anchor is None:
                    stationary = abs(state[3]) < .02 and abs(state[4]) < .04 and np.max(np.abs(state[11:])) < .035
                    if stationary:
                        if self.stationary_since is None:
                            self.stationary_since = begin
                        if begin-self.stationary_since >= .5:
                            self.anchor = state[:3].copy(); self.arm_reference = state[5:11].copy()
                    else:
                        self.stationary_since = None
                    if self.anchor is None:
                        reason = 'waiting_stationary_anchor'
                if reason is None:
                    reason = 'waiting_valid_cloud_inputs' if errors else snapshot.rejection_reason(now)
                prediction = None; proposed = None; info = {}
                if reason is None and not self.model.check(state[5:11], state[11:17])['valid']:
                    reason = 'unvalidated_geometry_configuration_rejected'
                if reason is None:
                    reference = np.array([[*self.anchor, 0.]])
                    try:
                        planned, info = self.kernel.plan(state, reference, self.arm_reference, snapshot, now)
                    except (ValueError, FloatingPointError, RuntimeError) as exc:
                        planned = None
                        info = {'shadow_solver_error': str(exc)}
                    if planned is None:
                        reason = 'surface_candidates_rejected_self_filter_and_geometry_unvalidated'
                    elif time.monotonic()-begin > self.config.get('deadline_s', .4):
                        reason = 'shadow_solve_deadline_missed'
                    else:
                        controls, prediction = planned
                        proposed = controls[0].tolist()
                        reason = 'shadow_candidate_only_not_validated_for_execution'
                publish_now = rospy.get_time()
                if proposed is not None and (not fresh(odom_stamp, publish_now, self.max_age) or
                        not fresh(joint_stamp, publish_now, self.max_age) or not snapshot.fresh(publish_now)):
                    reason = 'shadow_inputs_stale_after_solve'
                    proposed = None; prediction = None
                if proposed is not None:
                    self.proposed_pub.publish(self.Float64MultiArray(data=proposed))
                hook = getattr(self, "candidate_hook", None)
                if hook is not None:
                    hook(state, snapshot, odom_stamp, joint_stamp, now, proposed)
                # Always clear stale geometry rather than presenting old input as live.
                visible = state if state is not None and fresh(odom_stamp, publish_now, self.max_age) and fresh(joint_stamp, publish_now, self.max_age) else None
                self.markers(visible, prediction, measured_stamp=odom_stamp, reason=reason)
                coverage = None
                if visible is not None and snapshot.raw is not None and fresh(snapshot.raw.stamp, publish_now, self.max_age):
                    arm = self.model.arm_spheres(state[5:11], state[:3])
                    distance = snapshot.raw.tree.query(arm[:, :3], k=1)[0]
                    coverage = dict(raw_nearest_arm_surface_gap_m=float(np.min(distance-arm[:, 3])),
                        meaning='unfiltered_raw_may_include_arm_self_returns_not_clearance_certificate')
                row = dict(ros_stamp=publish_now, planning_snapshot_stamp=now, monotonic_elapsed=time.monotonic()-self.started,
                    mode='shadow_only', status=reason, ready_for_actuation=False, hardware_execution_eligible=False, commands_executed=False,
                    namespace=self.namespace, sources=self.topics, errors=errors,
                    input_transport_latency_s=latencies, clouds=cloud_inputs, near_arm_raw_diagnostic=coverage,
                    cloud_callback_profiles=cloud_profiles,
                    input_age_s={key: publish_now-value['stamp'] for key, value in cloud_inputs.items()},
                    odom_stamp=odom_stamp, joint_stamp=joint_stamp, joint_velocity_source='bounded_named_position_difference_rad_per_s',
                    odometry_validation=self.odom_gate.status(),
                    base_height_convention='ground_z_zero_odom_z_not_added', odom_z_ignored=self.odom_z_ignored,
                    state=None if state is None else state.tolist(), anchor=None if self.anchor is None else self.anchor.tolist(),
                    arm_reference=None if self.arm_reference is None else self.arm_reference.tolist(), proposed_controls=proposed,
                    solve_ms=1000.*(time.monotonic()-begin), kernel_diagnostics=info,
                    limitations=['model_geometry_not_hardware_validated', 'surface_cloud_not_known_free_space',
                        'raw_self_returns_may_reject_candidates', 'cloud_downsampling_not_collision_certificate',
                        'no_actuator_publishers_or_execution_mode'])
                if hasattr(self, 'gateway'):
                    row['command_gateway'] = self.gateway.status()
                    row['mode'] = 'integration_recording' if not self.execute else 'integration_execute'
                    row['commands_executed'] = bool(self.execute and self.gateway.sent)
                    row['limitations'] = [x for x in row['limitations'] if x != 'no_actuator_publishers_or_execution_mode']
                    row['limitations'].append('hardware_commissioning_separate_from_offline_validation')
                text = json.dumps(row, allow_nan=False)
                self.status_pub.publish(self.String(data=text)); self.log.write(text+'\n')
                rate.sleep()
        finally:
            self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--release-dir', required=True, type=Path)
    parser.add_argument('--namespace', default='/standby_trial_20261105_feedback_02')
    args, ros_args = parser.parse_known_args()
    release = args.release_dir.resolve()
    sys.path.insert(0, str(release/'core'))
    import rospy
    rospy.init_node('hardware_standby_shadow', argv=[sys.argv[0]]+ros_args)
    HardwareShadowNode(release, args.namespace).run()


if __name__ == '__main__':
    main()

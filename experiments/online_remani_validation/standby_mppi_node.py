#!/usr/bin/env python3
"""ROS execution entry for a separately authorized standby MPPI extension."""
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Float64MultiArray

from active_arm_model import PiperArmModel
from standby_whole_body_mppi import StandbyWholeBodyMPPI
from standby_observed_cloud import ObservedCloudTracker
from simulator import isolation


class StandbyNode:
    def __init__(self):
        self.out = Path(rospy.get_param('~out'))
        self.config = json.loads((self.out/'config.json').read_text())
        self.model = PiperArmModel.from_parameters(self.out/'parameters.yaml', self.config['urdf_path'])
        al = self.config['arm_limits']
        arm_limits = [al[k] for k in ('lower', 'upper', 'velocity', 'acceleration')]
        limits = self.config.get('limits')
        if limits is None:
            limits = json.loads((self.out/'limits.json').read_text())
        self.kernel = StandbyWholeBodyMPPI(self.model, limits, arm_limits,
            self.config.get('postures', []), enabled=True, seed=self.config.get('seed', 11),
            random_samples=self.config.get('random_samples', 6), horizon=self.config.get('horizon', 20),
            step=self.config.get('step', .1), config=self.config.get('kernel_config'))
        anchor = np.asarray(self.config.get('standby_anchor', [0., 0., 0.]), dtype=float)
        if anchor.shape != (3,) or not np.isfinite(anchor).all():
            raise ValueError('invalid standby anchor')
        self.reference = np.array([[*anchor, 0.]])
        self.arm_reference = np.asarray(self.config['q_start'], dtype=float)
        self.tracker = ObservedCloudTracker(padding=self.config.get('surface_padding', .02),
            horizon=self.config.get('prediction_horizon', 3.), max_age=.25,
            max_speed=self.config.get('max_observed_speed', 1.))
        self.lock = threading.Lock()
        self.base = self.q = self.dq = None
        self.odom_stamp = self.joint_stamp = 0.
        self.snapshot = None; self.cloud_error = None
        self.active = False; self.estop = True; self.estop_received = False
        self.permission_revision = 0; self.sequence = 0
        self.log = (self.out/'cycles.jsonl').open('w', buffering=1)
        self.pub = rospy.Publisher('/sim/whole_body_command', Float64MultiArray, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom, queue_size=1)
        rospy.Subscriber('/mm/joint_states', JointState, self.joints, queue_size=1)
        rospy.Subscriber('/sim/observed_cloud_world', PointCloud2, self.cloud, queue_size=1,
                         buff_size=8*1024**2)
        rospy.Subscriber('/sim/experiment_active', Bool, self.set_active, queue_size=1)
        rospy.Subscriber('/sim/experiment_estop', Bool, self.set_estop, queue_size=1)

    def odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        base = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                         msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        if not np.isfinite(base).all():
            return
        with self.lock:
            self.base = base; self.odom_stamp = msg.header.stamp.to_sec()

    def joints(self, msg):
        try:
            ids = [msg.name.index('joint'+str(i)) for i in range(1, 7)]
            q = np.array([msg.position[i] for i in ids]); dq = np.array([msg.velocity[i] for i in ids])
            if not np.isfinite(np.r_[q, dq]).all():
                return
        except (ValueError, IndexError):
            return
        with self.lock:
            self.q = q; self.dq = dq; self.joint_stamp = msg.header.stamp.to_sec()

    def cloud(self, msg):
        try:
            if msg.header.frame_id not in ('world', 'map'):
                raise ValueError('observed cloud must be in world coordinates')
            points = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                                             skip_nans=False)), dtype=float).reshape((-1, 3))
            with self.lock:
                self.snapshot = self.tracker.update(points, msg.header.stamp.to_sec())
                self.cloud_error = None
        except (ValueError, TypeError) as exc:
            with self.lock:
                self.cloud_error = str(exc)

    def set_active(self, msg):
        with self.lock:
            if self.active != bool(msg.data):
                self.permission_revision += 1
            self.active = bool(msg.data)

    def set_estop(self, msg):
        with self.lock:
            if self.estop != bool(msg.data) or not self.estop_received:
                self.permission_revision += 1
            self.estop = bool(msg.data); self.estop_received = True

    def authority(self):
        if not self.estop_received:
            return 'estop_not_received'
        if self.estop:
            return 'experiment_estop'
        if not self.active:
            return 'experiment_inactive'
        if self.cloud_error:
            return 'observed_cloud_invalid'
        return 'ok'

    def capture(self):
        with self.lock:
            state = None if self.base is None or self.q is None else np.r_[self.base, self.q, self.dq]
            return state, self.odom_stamp, self.joint_stamp, self.snapshot, self.permission_revision, self.authority()

    @staticmethod
    def fresh_reason(state, odom, joint, snapshot, now):
        if state is None:
            return 'state_not_received'
        if not -.05 <= now-odom <= .25:
            return 'odom_stale'
        if not -.05 <= now-joint <= .25:
            return 'joint_stale'
        if snapshot is None or not snapshot.fresh(now):
            return 'observed_cloud_stale'
        return 'ok'

    def run(self):
        rate = rospy.Rate(self.config.get('rate_hz', 5.))
        deadline = self.config.get('deadline_s', .4)
        epoch = 1000.+self.config.get('warmup_s', 7.)
        try:
            while not rospy.is_shutdown():
                begin = time.monotonic(); now = rospy.get_time(); self.sequence += 1
                state, odom, joint, snapshot, revision, reason = self.capture()
                info = {}; planned = None; validation_stamp = None; prediction = None
                if reason == 'ok':
                    reason = self.fresh_reason(state, odom, joint, snapshot, now)
                if reason == 'ok' and not self.model.check(state[5:11], state[11:17])['valid']:
                    reason = 'configuration_invalid'
                if reason == 'ok':
                    planned, info = self.kernel.plan(state, self.reference, self.arm_reference, snapshot, now)
                    if planned is None:
                        reason = 'no_feasible_standby_candidate'
                if planned is not None:
                    controls, prediction = planned
                    latest, odom2, joint2, snapshot2, revision2, reason2 = self.capture()
                    validation_stamp = rospy.get_time()
                    if revision != revision2 or reason2 != 'ok':
                        reason = 'permission_changed'
                    else:
                        reason = self.fresh_reason(latest, odom2, joint2, snapshot2, validation_stamp)
                    if reason == 'ok':
                        _, valid, gap, checked = self.kernel.evaluate(latest, controls[None],
                            self.reference, self.arm_reference, snapshot2, validation_stamp)
                        lease_ok, lease_gap = self.kernel.lease_brake(latest, controls, snapshot2,
                                                                      validation_stamp, lease=.25)
                        if not valid[0]:
                            reason = 'latest_rollout_rejected'
                        elif not lease_ok:
                            reason = 'lease_braking_rejected'
                        elif time.monotonic()-begin > deadline:
                            reason = 'deadline_missed'
                        else:
                            state, odom, joint, snapshot = latest, odom2, joint2, snapshot2
                            prediction = checked[0]; info['latest_min_gap'] = float(gap[0])
                            info['lease_brake_gap'] = float(lease_gap)
                stamp = rospy.get_time()
                with self.lock:
                    # Final authorization and publication share callbacks' lock.
                    # Repeated latched active messages cannot revoke a plan, but
                    # any actual permission transition invalidates the result.
                    if reason == 'ok':
                        if revision != self.permission_revision or self.authority() != 'ok':
                            reason = 'permission_changed_after_validation'
                        else:
                            reason = self.fresh_reason(state, odom, joint, snapshot, stamp)
                    if reason == 'ok' and time.monotonic()-begin > deadline:
                        reason = 'deadline_missed'
                    command = controls[0] if planned is not None and reason == 'ok' else np.zeros(8)
                    self.pub.publish(Float64MultiArray(data=[1., stamp, .25, float(self.sequence), *command.tolist()]))
                row = dict(t=stamp-epoch, ros_stamp=stamp, cycle=self.sequence,
                    reason=reason, status=reason, state=state.tolist() if state is not None else None,
                    command=command.tolist(), final_command=command.tolist(),
                    odom_stamp=odom, joint_stamp=joint, cloud_stamp=snapshot.stamp if snapshot else None,
                    observed_velocity=snapshot.velocity.tolist() if snapshot else None,
                    observed_points=len(snapshot.points) if snapshot else 0,
                    solve_ms=1000*(time.monotonic()-begin), callback_ms=1000*(time.monotonic()-begin),
                    publication_age=None if validation_stamp is None else stamp-validation_stamp,
                    permission_revision=revision, **info)
                if prediction is not None and reason == 'ok':
                    row['rollout'] = prediction[::2].tolist()
                self.log.write(json.dumps(row, allow_nan=False)+'\n')
                rate.sleep()
        finally:
            self.sequence += 1
            self.pub.publish(Float64MultiArray(data=[1., rospy.get_time(), .25, float(self.sequence), *np.zeros(8)]))
            self.log.close()


if __name__ == '__main__':
    isolation(); rospy.init_node('standby_whole_body_mppi')
    if not rospy.get_param('/online_validation/enabled', False) or not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('missing isolated numerical experiment interlock')
    StandbyNode().run()

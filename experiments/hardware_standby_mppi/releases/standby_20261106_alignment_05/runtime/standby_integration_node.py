#!/usr/bin/env python3
"""ROS input adapter for release 04. Default output is bounded recording only."""
import json
from pathlib import Path
import threading
import time

from base_lease_gate import validate_readiness
from hardware_command_guard import GuardConfig
from hardware_shadow_node import HardwareShadowNode
from mppi_command_gateway import (InputCohort, MppiCommandGateway,
    NativeLeaseValidator, PlanEnvelope, RecordingSink)

RELEASE = Path(__file__).resolve().parents[1]
NAMESPACE = '/standby_trial_20261106_alignment_05'


class RosCommandSink:
    """Lease metadata and position command are paired in the arm driver.

    Both drivers independently expire their lease. ROS publication is not an
    actuator acknowledgement: status must be fresh and enabled before sending.
    No subscribe/publish operation here can change readiness or auto-enable.
    """
    def __init__(self, rospy):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, String
        self.ros, self.JointState, self.Bool, self.String = rospy, JointState, Bool, String
        self.status = {}
        self.lock = threading.RLock()
        self.arm_lease = rospy.Publisher(NAMESPACE+'/arm/command_lease', String, queue_size=1)
        self.arm_joint = rospy.Publisher(NAMESPACE+'/arm/joint_ctrl_single', JointState, queue_size=1)
        self.base = rospy.Publisher(NAMESPACE+'/base/leased_command', String, queue_size=1)
        self.enable_pubs = [rospy.Publisher(NAMESPACE+'/'+name+'/enable_flag', Bool, queue_size=1)
                            for name in ('arm', 'base')]
        self.subs = [rospy.Subscriber(NAMESPACE+'/'+name+'/gate_status', String,
                     lambda msg, n=name: self.on_status(n, msg), queue_size=1)
                     for name in ('arm','base')]

    def on_status(self, name, msg):
        try:
            data = json.loads(msg.data)
            enabled = data.get('enabled') is True
        except (TypeError, ValueError, AttributeError):
            enabled = False
        with self.lock:
            self.status[name] = (enabled, time.monotonic())

    def enable(self, requested):
        for publisher in self.enable_pubs:
            publisher.publish(self.Bool(data=bool(requested)))

    def ready(self):
        now = time.monotonic()
        with self.lock:
            return all(name in self.status and self.status[name][0] and
                       now-self.status[name][1] < .15 for name in ('arm','base'))

    def send(self, packet):
        if not self.ready():
            raise RuntimeError('fresh_enabled_driver_status_required')
        if time.time() >= packet.expires_at:
            raise RuntimeError('expired_before_ros_publish')
        metadata = packet.lease_metadata()
        msg = self.JointState()
        msg.header.seq = packet.sequence
        msg.header.stamp = self.ros.Time.from_sec(packet.source_stamp)
        msg.name = ['joint%d' % i for i in range(1,7)]
        msg.position = list(packet.joint_positions)
        # SDK position transport requires velocity/effort EMPTY, not MPPI dq.
        msg.velocity, msg.effort = [], []
        self.arm_lease.publish(self.String(data=json.dumps(metadata, allow_nan=False)))
        self.arm_joint.publish(msg)
        base = dict(metadata, base_v=packet.base_v, base_w=packet.base_w)
        self.base.publish(self.String(data=json.dumps(base, allow_nan=False)))

    def stop(self, reason):
        self.enable(False)


class HardwareIntegrationNode(HardwareShadowNode):
    def __init__(self):
        super().__init__(RELEASE, NAMESPACE)
        self.execute = self.rospy.get_param('~execute', False)
        if type(self.execute) is not bool:
            raise ValueError('execute must be boolean')
        if self.execute:
            ready, reason = validate_readiness(RELEASE/'config/hardware_readiness.json')
            if not ready:
                raise RuntimeError('hardware readiness: '+reason)
        self.sink = RosCommandSink(self.rospy) if self.execute else RecordingSink()
        config = GuardConfig(tuple(self.model.lower), tuple(self.model.upper))
        self.gateway = MppiCommandGateway(config, NativeLeaseValidator(self.kernel, config=config),
                                           self.sink, frame_id=self.world)
        self.sequence = 0
        self.enable_pending = False
        from std_msgs.msg import Bool
        self.enable_sub = self.rospy.Subscriber(NAMESPACE+'/gateway/enable', Bool,
                                                self.request_enable, queue_size=1)
        self.stop_event = threading.Event()
        self.watchdog = threading.Thread(target=self.watch, name='gateway_watchdog', daemon=True)
        self.watchdog.start()
        self.rospy.on_shutdown(self.close)

    def request_enable(self, msg):
        # Set once after a fresh capture. A callback never reuses a cached plan.
        with self.lock:
            if msg.data and not self.gateway.enabled:
                self.enable_pending = True
            elif not msg.data:
                self.enable_pending = False
                self.gateway.set_enabled(False)

    def candidate_hook(self, state, snapshot, odom_stamp, joint_stamp, source_stamp, control):
        if state is None or control is None:
            if self.gateway.enabled:
                self.gateway.set_enabled(False)
            return
        state_stamp = min(odom_stamp, joint_stamp)
        latest_state, _, latest_odom, latest_joint, errors, *_ = self.capture()
        if (latest_state is None or 'odom' in errors or 'joints' in errors or
                self.odom_gate.fault is not None):
            self.gateway.set_enabled(False)
            return
        if not self.gateway.update_feedback(latest_state, min(latest_odom, latest_joint)):
            return
        with self.lock:
            pending, self.enable_pending = self.enable_pending, False
        if pending:
            self.gateway.set_enabled(True)
            if self.execute and self.gateway.enabled:
                self.sink.enable(True)
            return  # The enabling capture can NEVER execute an older plan.
        if not self.gateway.enabled:
            return
        if self.execute and not self.sink.ready():
            return  # Wait for driver acknowledgement within the SAME lease.
        obstacle_stamp = min(snapshot.raw.stamp, snapshot.filtered.stamp,
                             snapshot.dynamic.stamp, snapshot.health_stamp)
        expiry = min(state_stamp + .15, obstacle_stamp + snapshot.max_age,
                     source_stamp + .15)
        self.sequence += 1
        cohort = InputCohort('capture-%d'%self.sequence, tuple(state), state_stamp,
                            obstacle_stamp, expiry, self.world)
        plan = PlanEnvelope(self.sequence, source_stamp, expiry, cohort.cohort_id, tuple(control))
        self.gateway.submit(plan, cohort, snapshot)

    def watch(self):
        while not self.stop_event.wait(.02):
            self.gateway.tick()

    def close(self):
        self.stop_event.set()
        self.gateway.set_enabled(False)


def main():
    import rospy
    rospy.init_node('standby_integration', anonymous=False)
    if rospy.get_name() != NAMESPACE+'/standby_integration':
        raise RuntimeError('isolated integration namespace required')
    node = HardwareIntegrationNode()
    try:
        node.run()
    finally:
        node.close()


if __name__ == '__main__':
    main()

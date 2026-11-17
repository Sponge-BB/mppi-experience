#!/usr/bin/python3
"""Distributed visualization and native-reference checks; no command sink."""
import json
import math
from pathlib import Path
import time
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from quadrotor_msgs.msg import PolynomialTraj
from std_msgs.msg import String

from hardware_shadow_node import HardwareShadowNode
from native_reference_adapter import NativeReferenceAdapter

RELEASE = Path(__file__).resolve().parents[1]
NS = '/standby_trial_20261106_distributed_08'


class DistributedShadow(HardwareShadowNode):
    def __init__(self):
        super().__init__(RELEASE, NS)
        self.kernel = NativeReferenceAdapter(self.kernel)
        self.native_goal_pub = rospy.Publisher(NS+'/native/accepted_goal', PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher(NS+'/native/path', RosPath, queue_size=1, latch=True)
        self.reference_pub = rospy.Publisher(NS+'/native/reference_status', String, queue_size=1, latch=True)
        self.native_log = (Path(rospy.get_param('~log_dir'))/'native_reference.jsonl').open('x', buffering=1)
        self.goal_sub = rospy.Subscriber(NS+'/goal', PoseStamped, self.on_goal, queue_size=1)
        self.trajectory_sub = rospy.Subscriber(NS+'/native/trajectory', PolynomialTraj,
                                               self.on_trajectory, queue_size=20)
        self.last_markers = 0.
        self.timer = rospy.Timer(rospy.Duration(.2), self.publish_path)

    def record_native(self, event, result, stamp):
        data = dict(event=event, result=result, original_stamp=stamp,
                    receipt_stamp=rospy.get_time(), hardware_commands_sent=False)
        self.native_log.write(json.dumps(data)+'\n')
        self.reference_pub.publish(String(data=json.dumps(data)))

    def on_goal(self, msg):
        result = self.kernel.feed_goal(msg, rospy.get_time())
        state, snapshot, odom_stamp, joint_stamp, errors, *_ = self.capture()
        now = rospy.get_time()
        if result == 'accepted':
            # Native INIT only requires odometry, so do not let an early RViz
            # click plan from its configured placeholder arm state.
            if (state is None or errors or self.odom_gate.fault is not None or
                    not 0. <= now-min(odom_stamp,joint_stamp) < .15 or not snapshot.fresh(now)):
                result = 'not_forwarded_inputs_unready_send_new_goal_when_ready'
            else:
                self.native_goal_pub.publish(msg)  # Original frame and timestamp.
        self.record_native('goal', result, msg.header.stamp.to_sec())

    def on_trajectory(self, msg):
        result = self.kernel.feed_trajectory(msg, rospy.get_time())
        self.record_native('trajectory', result, msg.header.stamp.to_sec())
        # Keep coefficients for an independent replay of what was actually
        # received; visualization never invents a reference when planning fails.
        self.native_log.write(json.dumps(dict(event='native_coefficients',
            action=msg.action, trajectory_id=msg.trajectory_id, singul=msg.singul,
            stamp=msg.header.stamp.to_sec(), frame=msg.header.frame_id,
            pieces=[dict(duration=p.duration,num_dim=p.num_dim,num_order=p.num_order,
                         data=list(p.data)) for p in msg.trajectory]))+'\n')

    def publish_path(self, event):
        with self.lock:
            yaw = float(self.base[2]) if self.base is not None else 0.
        result = self.kernel.snapshot_path(yaw, now=rospy.get_time())
        path = RosPath(); path.header.frame_id = 'world'; path.header.stamp = rospy.Time.now()
        if result is not None:
            for row in result:
                pose = PoseStamped(); pose.header = path.header
                pose.pose.position.x, pose.pose.position.y = float(row[0]), float(row[1])
                pose.pose.orientation.z = math.sin(float(row[2])/2.)
                pose.pose.orientation.w = math.cos(float(row[2])/2.)
                path.poses.append(pose)
        self.path_pub.publish(path)

    def markers(self, *args, **kwargs):
        # Only display is limited to 3 Hz. Every actual planning/acceptance
        # cycle remains unchanged and uses fresh observations.
        now = time.monotonic()
        if now-self.last_markers >= 1./3.:
            if kwargs.get('reason') == 'surface_candidates_rejected_self_filter_and_geometry_unvalidated':
                native = self.kernel.status(rospy.get_time())
                if native['reference_status'] != 'native_reference_ready':
                    kwargs['reason'] = native['reference_status']
            super().markers(*args, **kwargs)
            self.last_markers = now


class RecordingDistributedShadow(DistributedShadow):
    """Reuse07 delivery checks with an in-memory sink; never creates drivers."""
    from standby_integration_node import HardwareIntegrationNode as _Integration
    candidate_hook = _Integration.candidate_hook
    request_enable = _Integration.request_enable
    current_inputs_valid = _Integration.current_inputs_valid
    refresh_feedback = _Integration.refresh_feedback
    watch = _Integration.watch
    close = _Integration.close

    def cohort_aligned(self, capture):
        if not super().cohort_aligned(capture):
            return False
        # A new scan can arrive a few ms before its odometry. Reserve the
        # existing command-age budget within the existing feedback-age limit;
        # the parent still waits at most20ms inside its unchanged95ms budget.
        available_age = self.gateway.guard.config.feedback_max_age-self.gateway.guard.config.command_max_age
        return 0. <= rospy.get_time()-min(capture[2], capture[3]) <= available_age

    def on_goal(self, msg):
        # This timing probe covers standby delivery only. Navigation reference
        # revision/cancellation must be bound to a packet before that next stage.
        gateway = getattr(self, 'gateway', None)
        if gateway is not None:
            gateway.set_enabled(False)
        self.record_native('goal', 'recording_probe_is_standby_only', msg.header.stamp.to_sec())

    def __init__(self):
        super().__init__()
        from hardware_command_guard import GuardConfig
        from mppi_command_gateway import MppiCommandGateway, NativeLeaseValidator, RecordingSink
        from std_msgs.msg import Bool
        self.execute = False
        self.sink = RecordingSink()
        config = GuardConfig(tuple(self.model.lower), tuple(self.model.upper))
        self.gateway = MppiCommandGateway(config,
            NativeLeaseValidator(self.kernel.standby, config=config), self.sink,
            frame_id=self.world, input_guard=self.current_inputs_valid)
        self.sequence = 0
        self.enable_pending = False
        self.enable_sub = rospy.Subscriber(NS+'/gateway/enable', Bool,
                                          self.request_enable, queue_size=1)
        self.stop_event = threading.Event()
        self.watchdog = threading.Thread(target=self.watch, name='recording_watchdog', daemon=True)
        self.watchdog.start()
        rospy.on_shutdown(self.close)


def main():
    rospy.init_node('distributed_shadow')
    node = RecordingDistributedShadow() if rospy.get_param('~record_gateway', False) else DistributedShadow()
    try:
        node.run()
    finally:
        if isinstance(node, RecordingDistributedShadow):
            node.close()
        node.timer.shutdown()
        node.native_log.close()


if __name__ == '__main__':
    main()

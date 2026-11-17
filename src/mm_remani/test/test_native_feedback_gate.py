"""Actual native bridge callbacks/control loop; no ROS master or device access."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT/'src/mm_remani/scripts'), str(ROOT/'src/mm_controller/scripts')]
from real_base_tracker import RealBaseTracker
from joint_cmd_bridge import JointBridge


class NativeFeedbackGateTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.params = {'~tracking_recover_frames': 3}
        self.patches = [
            patch.object(rospy, 'get_param', side_effect=lambda name, default=None: self.params.get(name, default)),
            patch.object(rospy.Time, 'now', side_effect=lambda: rospy.Time.from_sec(self.now)),
        ]
        self.patches += [patch.object(rospy, name) for name in
                        ('init_node', 'Subscriber', 'Timer', 'on_shutdown', 'spin',
                         'loginfo', 'logwarn', 'logerr', 'loginfo_throttle', 'logwarn_throttle')]
        self.patches.append(patch.object(rospy, 'Publisher', side_effect=lambda *a, **kw: Mock()))
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.node = RealBaseTracker()

    def odom(self, stamp=None):
        msg = Odometry()
        msg.header.stamp = rospy.Time.from_sec(self.now if stamp is None else stamp)
        msg.pose.pose.orientation.w = 1.
        return msg

    def joints(self, stamp=None):
        msg = JointState()
        msg.header.stamp = rospy.Time.from_sec(self.now if stamp is None else stamp)
        msg.name = ['joint%d' % i for i in range(1, 7)]
        msg.position = [0.]*6
        msg.velocity = [0.]*6
        return msg

    def feed(self):
        self.node.odom_cb(self.odom())
        self.node.joint_cb(self.joints())
        target = Twist()
        target.linear.x = .06  # A real correcting command below the recovery threshold.
        self.node.car_cmd_cb(target)

    def output(self):
        msg = self.node.cmd_pub.publish.call_args[0][0]
        return msg.linear.x, msg.angular.z

    def test_arm_feedback_loss_stops_base_despite_reference_position_error(self):
        self.feed()
        self.node.control_loop(None)
        self.assertGreater(self.output()[0], 0.)
        self.now += .26
        self.node.odom_cb(self.odom())
        self.node.control_loop(None)
        self.assertEqual(self.output(), (0., 0.))
        self.assertTrue(self.node.tracking_fault)

    def test_recovery_needs_new_feedback_frames_and_aligned_reference(self):
        self.feed()
        self.now += .26
        self.node.control_loop(None)
        self.now += .01
        self.feed()
        for _ in range(10):
            self.node.control_loop(None)
        self.assertTrue(self.node.tracking_fault)
        self.assertEqual(self.node.tracking_recover_count, 1)
        self.assertEqual(self.output(), (0., 0.))
        for _ in range(2):
            self.now += .02
            self.feed()
            self.node.control_loop(None)
        self.assertFalse(self.node.tracking_fault)
        self.assertGreater(self.output()[0], 0.)

    def test_new_invalid_joint_frame_cannot_be_undone_by_queued_old_valid_frame(self):
        self.feed()
        previous = self.joints()
        self.now += .02
        invalid = self.joints()
        invalid.position[2] = float('nan')
        self.node.joint_cb(invalid)
        self.node.joint_cb(previous)
        same_stamp_valid = self.joints()
        self.node.joint_cb(same_stamp_valid)
        self.node.control_loop(None)
        self.assertFalse(self.node.have_joints)
        self.assertEqual(self.output(), (0., 0.))

    def test_queued_feedback_keeps_source_age_and_missing_velocity_is_invalid(self):
        self.feed()
        old = self.joints()
        self.now += .26
        self.node.joint_cb(old)
        self.node.odom_cb(self.odom())
        self.node.control_loop(None)
        self.assertEqual(self.output(), (0., 0.))
        missing = self.joints()
        missing.velocity = []
        self.node.joint_cb(missing)
        self.assertFalse(self.node.have_joints)

    def test_future_and_zero_source_do_not_grant_execution(self):
        self.feed()
        for offset in (1., -100.):
            self.node.joint_cb(self.joints(self.now+offset))
            self.node.control_loop(None)
            self.assertEqual(self.output(), (0., 0.))
        self.now += .01
        self.node.joint_cb(self.joints())
        self.assertTrue(self.node.have_joints)  # Bad future stamp did not poison the clock.

    def test_observe_mode_never_creates_a_base_command_publisher(self):
        self.params['~execute'] = False
        self.node = RealBaseTracker()
        self.feed()
        self.node.control_loop(None)
        self.node.publish_stop()
        self.assertIsNone(self.node.cmd_pub)

    def command(self):
        msg = JointTrajectoryControllerState()
        msg.header.stamp = rospy.Time.from_sec(self.now)
        msg.joint_names = ['joint%d' % i for i in range(1, 7)]
        msg.desired.positions = [.01]*6
        msg.desired.velocities = [0.]*6
        return msg

    def test_arm_bridge_preserves_source_and_rejects_queued_incomplete_or_duplicate_targets(self):
        bridge = JointBridge()
        msg = self.command()
        bridge.joint_cb(msg)
        self.assertEqual(bridge.pub.publish.call_count, 1)
        self.assertEqual(bridge.pub.publish.call_args[0][0].header.stamp, msg.header.stamp)
        self.now += .3
        bridge.joint_cb(msg)
        self.assertEqual(bridge.pub.publish.call_count, 1)
        invalid = self.command()
        invalid.desired.velocities = []
        bridge.joint_cb(invalid)
        self.assertEqual(bridge.pub.publish.call_count, 1)
        msg = self.command()
        bridge.joint_cb(msg)
        self.now += .21
        bridge.joint_cb(msg)
        self.assertEqual(bridge.pub.publish.call_count, 2)

    def test_observe_mode_never_creates_an_arm_command_publisher(self):
        self.params['~execute'] = False
        bridge = JointBridge()
        bridge.joint_cb(self.command())
        self.assertIsNone(bridge.pub)


if __name__ == '__main__':
    unittest.main()

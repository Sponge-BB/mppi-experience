"""Driver-to-controller source expiry and stop propagation; no ROS master/CAN."""
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from whole_body_mppi import WholeBodyNode


def message(stamp, until=None, enabled=True, valid=True, stopped=False, command_active=False):
    return NS(header=NS(stamp=NS(to_sec=lambda: stamp)),
              valid_until=NS(to_sec=lambda: stamp+.15 if until is None else until),
              enabled=enabled, feedback_valid=valid, stop_latched=stopped,
              command_active=command_active)


class ExecutionGateTests(unittest.TestCase):
    def setUp(self):
        self.node = NS(lock=threading.RLock(), execution_stamp=0., execution_valid_until=0.,
                       execution_enabled=False, execution_latched=False, execution_revision=0,
                       stop_base=Mock())
        self.node.execution_ready = lambda now: WholeBodyNode.execution_ready(self.node, now)

    def feed(self, msg, now=100.):
        with patch('whole_body_mppi.rospy.get_time', return_value=now):
            WholeBodyNode.on_execution_state(self.node, msg)

    def ready(self, now):
        return WholeBodyNode.execution_ready(self.node, now)

    def test_queued_healthy_packet_does_not_get_a_new_lease(self):
        self.feed(message(99.9, until=100.02))
        self.assertTrue(self.ready(100.01))
        self.assertFalse(self.ready(100.02))
        self.feed(message(99.9, until=100.04), now=100.01)
        self.assertEqual(self.node.execution_valid_until, 100.02)

    def test_stop_without_new_can_frame_latches_and_stops_base(self):
        self.feed(message(100.))
        self.feed(message(100., stopped=True))
        self.node.stop_base.assert_called_once()
        self.assertFalse(self.ready(100.))
        self.feed(message(100.01), now=100.01)
        self.assertFalse(self.ready(100.01))

    def test_invalid_status_cannot_be_undone_by_older_healthy_packet(self):
        self.feed(message(99.95))
        self.feed(message(100., valid=False))
        self.feed(message(99.99))
        self.assertFalse(self.ready(100.))
        self.feed(message(100.01), now=100.01)
        self.assertTrue(self.ready(100.01))

    def test_already_expired_command_lease_stops_immediately(self):
        self.feed(message(100.))
        self.feed(message(100.01, until=100.015), now=100.02)
        self.node.stop_base.assert_called_once()
        self.assertFalse(self.ready(100.02))

    def test_future_stale_or_extended_source_expiry_is_rejected(self):
        for msg in (message(100.01), message(99.8), message(100., until=100.151),
                    message(100., until=float('nan')), message(100., enabled=False)):
            with self.subTest(msg=msg):
                self.setUp()
                self.feed(msg)
                self.assertFalse(self.ready(100.))
                self.node.stop_base.assert_called_once()

    def test_normal_feedback_refresh_keeps_authority_but_reenable_changes_it(self):
        self.feed(message(100.))
        revision = self.node.execution_revision
        self.feed(message(100.01), now=100.01)
        self.assertEqual(self.node.execution_revision, revision)
        self.feed(message(100.02, enabled=False), now=100.02)
        self.feed(message(100.03), now=100.03)
        self.assertTrue(self.ready(100.03))
        self.assertNotEqual(self.node.execution_revision, revision)

    def test_source_gap_also_invalidates_an_inflight_permission(self):
        self.feed(message(100.))
        revision = self.node.execution_revision
        self.feed(message(100.2), now=100.2)
        self.assertTrue(self.ready(100.2))
        self.assertNotEqual(self.node.execution_revision, revision)


class ExecutionDeliveryTests(unittest.TestCase):
    """Exercise the real cycle/gate/publication path with an isolated solver."""
    def setUp(self):
        self.now = 100.
        node = self.node = WholeBodyNode.__new__(WholeBodyNode)
        node.lock = threading.RLock()
        node.execute, node.simulation = True, False
        node.execution_stamp = node.execution_valid_until = 0.
        node.execution_enabled = node.execution_latched = False
        node.execution_revision = 0
        node.base = np.zeros(5); node.joints = np.zeros(12)
        node.base_stamp = node.joint_stamp = 99.99
        node.snapshot = NS(fresh=lambda now: True)
        node.anchor = np.zeros(3); node.arm_anchor = np.zeros(6)
        node.kernel = NS(arm_position_servo=False, _servo_native=None)
        node.limits = dict(vmax=.05, wmax=.15, track=.37, wheel_v=.2296)
        node.safety_speed_limit = None
        node.deadline = 1.; node.last_visual = float('inf')
        node.arm_enabled = True; node.seed = 11
        node.sent_command = False; node.sent_navigation_goal = None
        for name in ('base_pub', 'arm_pub', 'fault_pub', 'info_pub'):
            setattr(node, name, Mock())
        node.reference = NS(lock=threading.RLock(), revision=0, active=False,
                            goal_stamp=None, plan=Mock(side_effect=self.solve))
        self.during_solve = lambda: None
        self.predicted = np.zeros((2, 17))
        self.checked = np.zeros(17); self.checked[5] = .01

    def solve(self, *_):
        self.during_solve()
        return (np.array([[.02, 0., .05, 0., 0., 0., 0., 0.]]), self.predicted), {'feasible': 1}

    def feed(self, msg, now=100.):
        self.now = now
        with patch('whole_body_mppi.rospy.get_time', return_value=now):
            self.node.on_execution_state(msg)

    def cycle(self):
        with patch('whole_body_mppi.rospy.get_time', side_effect=lambda: self.now), \
                patch('whole_body_mppi.checked_first_step', return_value=self.checked):
            self.node.cycle()

    def assert_no_motion_delivery(self):
        self.node.arm_pub.publish.assert_not_called()
        for call in self.node.base_pub.publish.call_args_list:
            command = call.args[0]
            self.assertEqual((command.linear.x, command.linear.y, command.angular.z), (0., 0., 0.))
        self.assertFalse(self.node.sent_command)

    def test_first_enable_cannot_authorize_a_plan_started_without_permission(self):
        # This hook would reproduce the old enable-during-solve race. The
        # initial gate must stop before invoking the solver at all.
        self.during_solve = lambda: self.feed(message(100.01), now=100.01)
        self.cycle()
        self.node.reference.plan.assert_not_called()
        self.assert_no_motion_delivery()
        self.assertIn('waiting_explicit_arm_enable_or_normal_status',
                      self.node.info_pub.publish.call_args.args[0].data)

    def test_invalid_then_reenabled_during_solve_rejects_both_commands(self):
        self.feed(message(100.))
        def change_authority():
            self.feed(message(100.01, valid=False), now=100.01)
            self.feed(message(100.02), now=100.02)
        self.during_solve = change_authority
        self.cycle()
        self.node.reference.plan.assert_called_once()
        self.assertTrue(self.node.execution_ready(self.now))
        self.assert_no_motion_delivery()

    def test_normal_source_refresh_during_solve_delivers_both_commands(self):
        self.feed(message(100.))
        self.during_solve = lambda: self.feed(message(100.01), now=100.01)
        self.cycle()
        self.node.arm_pub.publish.assert_called_once()
        self.node.base_pub.publish.assert_called_once()
        arm = self.node.arm_pub.publish.call_args.args[0]
        base = self.node.base_pub.publish.call_args.args[0]
        self.assertEqual(arm.header.stamp.to_sec(), 100.)
        self.assertEqual(len(arm.position), 6)
        self.assertEqual(arm.position[0], .01)
        self.assertEqual(base.linear.x, .02)
        self.assertTrue(self.node.sent_command)


if __name__ == '__main__':
    unittest.main()

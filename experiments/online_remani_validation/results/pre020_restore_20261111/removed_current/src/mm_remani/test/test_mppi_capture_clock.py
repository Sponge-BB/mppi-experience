"""Feedback age is checked at lock acquisition, not an earlier cycle clock."""
import threading
import unittest
from unittest.mock import patch

import numpy as np

import test_mppi_execution_gate as gates


class OneShotLock:
    """Deterministically deliver a callback while the cycle is waiting."""
    def __init__(self, callback):
        self.lock = threading.RLock()
        self.callback = callback

    def __enter__(self):
        self.lock.acquire()
        callback, self.callback = self.callback, None
        if callback is not None:
            callback()
        return self

    def __exit__(self, *args):
        self.lock.release()


class CaptureClockTests(unittest.TestCase):
    def setUp(self):
        self.fixture = gates.ExecutionDeliveryTests(methodName='runTest')
        self.fixture.setUp()
        self.node = self.fixture.node

    def capture(self, source_stamp, current_time, cycle_time=100.):
        self.node.base_stamp = self.node.joint_stamp = source_stamp
        with patch('whole_body_mppi.rospy.get_time', return_value=current_time):
            return self.node.capture(cycle_time)

    def test_new_feedback_received_after_cycle_start_is_not_future(self):
        state, _, _, _ = self.capture(100.01, 100.02)
        np.testing.assert_array_equal(state, np.r_[self.node.base, self.node.joints])

    def test_feedback_that_expired_while_waiting_is_rejected_before_solve(self):
        with self.assertRaisesRegex(ValueError, '^state_feedback_stale$'):
            self.capture(99.86, 100.02)

    def test_actual_future_feedback_still_rejected(self):
        with self.assertRaisesRegex(ValueError, '^state_feedback_stale$'):
            self.capture(100.03, 100.02)

    def test_map_that_expired_while_waiting_is_rejected_before_solve(self):
        self.node.snapshot.fresh = lambda now: now <= 100.01
        with self.assertRaisesRegex(ValueError, '^collision_snapshot_unavailable$'):
            self.capture(99.99, 100.02)

    def test_new_health_and_feedback_after_lock_wait_deliver_with_original_stamp(self):
        self.fixture.feed(gates.message(100.))

        def newer_callback():
            self.fixture.feed(gates.message(100.01), now=100.02)
            self.node.base_stamp = self.node.joint_stamp = 100.01

        self.node.lock = OneShotLock(newer_callback)
        self.fixture.cycle()
        self.node.reference.plan.assert_called_once()
        self.node.arm_pub.publish.assert_called_once()
        self.node.base_pub.publish.assert_called_once()
        arm = self.node.arm_pub.publish.call_args.args[0]
        self.assertEqual(arm.header.stamp.to_sec(), 100.)
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].linear.x, .02)

    def test_expired_authority_after_lock_wait_does_not_start_solver(self):
        self.fixture.feed(gates.message(100.))
        self.node.lock = OneShotLock(lambda: setattr(self.fixture, 'now', 100.16))
        self.fixture.cycle()
        self.node.reference.plan.assert_not_called()
        self.fixture.assert_no_motion_delivery()

    def prepare_native_gate(self):
        self.node.startup_complete = False
        self.node.native_handoff_sent = False
        self.node.startup_target = np.zeros(6)

    def test_native_gate_uses_new_health_without_sending_startup_commands(self):
        self.prepare_native_gate()
        self.fixture.feed(gates.message(100.01), now=100.02)
        self.node.joint_stamp = 100.01
        with patch('whole_body_mppi.rospy.get_time', return_value=100.02):
            self.node.native_startup_gate(100.)
        self.assertFalse(self.node.startup_complete)
        self.node.arm_pub.publish.assert_not_called()
        self.node.base_pub.publish.assert_not_called()

    def test_native_gate_rejects_authority_expired_while_waiting(self):
        self.prepare_native_gate()
        self.fixture.feed(gates.message(100.))
        with patch('whole_body_mppi.rospy.get_time', return_value=100.16), \
                self.assertRaisesRegex(ValueError, '^waiting_native_arm_startup$'):
            self.node.native_startup_gate(100.)

    def test_native_gate_still_rejects_actual_future_joints(self):
        self.prepare_native_gate()
        self.fixture.feed(gates.message(100.02), now=100.02)
        self.node.joint_stamp = 100.03
        with patch('whole_body_mppi.rospy.get_time', return_value=100.02), \
                self.assertRaisesRegex(ValueError, '^native_startup_joint_feedback_unavailable$'):
            self.node.native_startup_gate(100.02)


if __name__ == '__main__':
    unittest.main()

"""Reference waits renew only freshly validated holds; no ROS master or CAN."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import test_mppi_execution_gate as gates
from whole_body_mppi import WholeBodyNode, REFERENCE_WAIT_HOLD_REASONS, reference_wait_hold


class ReferenceWaitDeliveryTests(unittest.TestCase):
    solve = gates.ExecutionDeliveryTests.solve
    feed = gates.ExecutionDeliveryTests.feed
    cycle = gates.ExecutionDeliveryTests.cycle

    def setUp(self):
        gates.ExecutionDeliveryTests.setUp(self)
        node = self.node
        node.kernel.H = 8
        node.kernel.lease_brake_states = Mock(side_effect=lambda state, controls, lease:
                                              np.tile(state, (4, 1)))
        node.reference.active = True
        node.reference.goal_stamp = 95.
        node.reference.navigation = node.kernel
        node.reference.status = Mock(return_value={'reference_status': 'waiting_native_progress'})
        self.wait_reason = 'waiting_native_progress'
        node.reference.plan = Mock(side_effect=self.waiting_solver)

    def waiting_solver(self, *_):
        self.during_solve()
        return None, {'reference_status': self.wait_reason, 'reference_revision': 0}

    def test_seven_hundred_ms_wait_keeps_real_command_cadence_without_extending_timeout(self):
        # Model the unchanged .30 s source-stamped watchdog, rather than merely
        # asserting a hold helper exists. A .70 s future activation used to
        # leave every arm command absent throughout this interval.
        driver_deadline = 100.3
        for i in range(9):
            now = 100.+.1*i
            self.assertLess(now, driver_deadline)
            self.feed(gates.message(now), now=now)
            self.node.base_stamp = self.node.joint_stamp = now
            self.cycle()
            arm = self.node.arm_pub.publish.call_args.args[0]
            self.assertAlmostEqual(arm.header.stamp.to_sec(), now)
            driver_deadline = arm.header.stamp.to_sec()+.30
        self.assertEqual(self.node.arm_pub.publish.call_count, 9)
        for call in self.node.base_pub.publish.call_args_list:
            self.assertEqual((call.args[0].linear.x, call.args[0].angular.z), (0., 0.))
        self.node.fault_pub.publish.assert_not_called()
        self.assertIsNone(self.node.sent_navigation_goal)

    def test_hold_uses_latest_feedback_and_collision_check_instead_of_old_arm_target(self):
        self.feed(gates.message(100.))
        self.during_solve = lambda: setattr(self.node, 'joints', np.r_[np.full(6, .12), np.zeros(6)])
        self.checked[5:11] = .121
        with patch('whole_body_mppi.rospy.get_time', return_value=100.), \
                patch('whole_body_mppi.checked_first_step', return_value=self.checked) as checked:
            self.node.cycle()
        np.testing.assert_allclose(checked.call_args.args[1][5:11], .12)
        np.testing.assert_array_equal(checked.call_args.args[2], np.zeros(8))
        self.assertIs(checked.call_args.args[3], self.node.snapshot)
        np.testing.assert_allclose(self.node.arm_pub.publish.call_args.args[0].position, .121)

    def test_invalid_latest_map_does_not_get_a_hold_heartbeat(self):
        self.feed(gates.message(100.))
        with patch('whole_body_mppi.rospy.get_time', return_value=100.), \
                patch('whole_body_mppi.checked_first_step', side_effect=ValueError('unsafe_hold')):
            self.node.cycle()
        self.node.arm_pub.publish.assert_not_called()

    def test_stale_feedback_or_cloud_does_not_get_a_hold_heartbeat(self):
        for invalid in ('state', 'cloud'):
            with self.subTest(invalid=invalid):
                self.setUp(); self.feed(gates.message(100.))
                if invalid == 'state':
                    self.node.joint_stamp = 99.
                else:
                    self.node.snapshot.fresh = lambda now: False
                self.cycle()
                self.node.reference.plan.assert_not_called()
                self.node.arm_pub.publish.assert_not_called()

    def test_stop_arriving_during_wait_plan_still_latches_and_blocks_delivery(self):
        self.feed(gates.message(100.))
        self.during_solve = lambda: self.feed(gates.message(100.01, stopped=True), now=100.01)
        self.cycle()
        self.assertTrue(self.node.execution_latched)
        self.node.arm_pub.publish.assert_not_called()

    def test_cancel_invalid_progress_and_collision_failures_never_fall_back_to_hold(self):
        for reason in ('native_reference_cancelled', 'native_progress_stale',
                       'invalid_native_progress', 'no_feasible_candidate'):
            with self.subTest(reason=reason):
                self.setUp(); self.feed(gates.message(100.))
                self.wait_reason = reason
                self.cycle()
                self.node.arm_pub.publish.assert_not_called()
                self.node.kernel.lease_brake_states.assert_not_called()

    def test_progress_becoming_ready_during_checked_hold_does_not_clear_recovery(self):
        self.feed(gates.message(100.))
        self.node.reference.status.return_value = {'reference_status': 'native_reference_ready'}
        self.cycle()
        self.node.arm_pub.publish.assert_called_once()
        self.node.fault_pub.publish.assert_not_called()

    def test_only_explicit_reference_waits_are_eligible_and_helper_outputs_zero_intent(self):
        self.assertNotIn('native_progress_stale', REFERENCE_WAIT_HOLD_REASONS)
        controls, _ = reference_wait_hold(self.node.kernel, np.zeros(17))
        np.testing.assert_array_equal(controls, np.zeros((8, 8)))


if __name__ == '__main__':
    unittest.main()

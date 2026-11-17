"""Deadline classification through the real cycle; no ROS master or hardware."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_mppi_execution_gate as delivery_fixture


class DeadlinePriorityTests(unittest.TestCase):
    def setUp(self):
        # Compose the existing fixture rather than inheriting/recollecting its
        # tests. Publishers and the solver stay isolated from the ROS graph.
        self.fixture = delivery_fixture.ExecutionDeliveryTests(methodName='runTest')
        self.fixture.setUp()
        self.node = self.fixture.node
        self.fixture.feed(delivery_fixture.message(100.))
        self.node.deadline = .095
        self.node.reference.active = True
        self.node.reference.goal_stamp = 99.
        self.node.reference.navigation = self.node.kernel
        self.node.reference.status = Mock(return_value=dict(
            reference_status='native_reference_ready', native_gear_block=0))
        self.node.sent_command = True
        self.node.sent_navigation_goal = 99.
        self.clock = 0.
        self.solve_seconds = .05
        self.check_seconds = 0.
        self.reject_seconds = 0.
        self.has_result = True
        self.feasible = 1
        self.reference_status = 'native_reference_ready'
        self.during_solve = lambda: None
        self.solve_error = None
        self.node.reference.plan = Mock(side_effect=self.solve)
        # Make rejection take controlled extra time. Its bookkeeping must not
        # change the elapsed value used for the plan-return decision.
        self.node.base_pub.publish.side_effect = self.on_base_publish

    def solve(self, *_):
        self.clock += self.solve_seconds
        self.during_solve()
        if self.solve_error is not None:
            raise ValueError(self.solve_error)
        result = None
        if self.has_result:
            result = (np.array([[.02, 0., .05, 0., 0., 0., 0., 0.]]),
                      self.fixture.predicted)
        return result, dict(feasible=self.feasible,
                            reference_status=self.reference_status,
                            reference_revision=self.node.reference.revision,
                            native_gear_block=0)

    def on_base_publish(self, command):
        if command.linear.x == 0. and command.angular.z == 0.:
            self.clock += self.reject_seconds

    def checked_first_step(self, *_):
        self.clock += self.check_seconds
        return self.fixture.checked

    def cycle(self):
        with patch('whole_body_mppi.time.monotonic', side_effect=lambda: self.clock), \
                patch('whole_body_mppi.rospy.get_time', side_effect=lambda: self.fixture.now), \
                patch('whole_body_mppi.checked_first_step', side_effect=self.checked_first_step):
            self.node.cycle()
        return json.loads(self.node.info_pub.publish.call_args.args[0].data)

    def assert_zero_without_arm(self):
        self.node.arm_pub.publish.assert_not_called()
        self.assertGreater(self.node.base_pub.publish.call_count, 0)
        for call in self.node.base_pub.publish.call_args_list:
            command = call.args[0]
            self.assertEqual((command.linear.x, command.linear.y, command.angular.z),
                             (0., 0., 0.))

    def assert_active_reference_preserved(self):
        self.assertTrue(self.node.reference.active)
        self.assertEqual(self.node.reference.goal_stamp, 99.)
        self.assertEqual(self.node.reference.revision, 0)
        self.assertTrue(self.node.sent_command)
        self.assertEqual(self.node.sent_navigation_goal, 99.)

    def test_late_normal_result_is_local_retry_with_or_without_candidate(self):
        for has_result in (False, True):
            with self.subTest(has_result=has_result):
                self.setUp()
                self.solve_seconds = .120
                self.reject_seconds = .040
                self.has_result = has_result
                self.feasible = int(has_result)
                record = self.cycle()
                self.assertEqual(record['state'], 'planning_deadline_exceeded')
                self.assert_zero_without_arm()
                self.node.fault_pub.publish.assert_not_called()
                self.assert_active_reference_preserved()
                diagnostic = record['candidate_diagnostics']
                self.assertEqual(diagnostic['planning_elapsed_ms'], 120.)
                self.assertEqual(diagnostic['deadline_ms'], 95.)
                self.assertEqual(diagnostic['feasible'], int(has_result))
                self.assertEqual(diagnostic['reference_status'], 'native_reference_ready')
                self.assertNotIn('delivery_elapsed_ms', diagnostic)
                self.assertEqual(record['cycle_ms'], 160.)

    def test_timely_no_candidate_still_requests_recovery_for_started_goal(self):
        self.has_result = False
        self.feasible = 0
        record = self.cycle()
        self.assertEqual(record['state'], 'no_feasible_candidate')
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_called_once()
        self.assertTrue(self.node.fault_pub.publish.call_args.args[0].data)
        self.assertEqual(record['candidate_diagnostics']['planning_elapsed_ms'], 50.)
        self.assertEqual(record['candidate_diagnostics']['reference_status'],
                         'native_reference_ready')

    def test_timely_solve_but_late_delivery_remains_local_retry(self):
        self.check_seconds = .060
        self.reject_seconds = .040
        record = self.cycle()
        self.assertEqual(record['state'], 'delivery_deadline_exceeded')
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_not_called()
        self.assert_active_reference_preserved()
        diagnostic = record['candidate_diagnostics']
        self.assertEqual(diagnostic['planning_elapsed_ms'], 50.)
        self.assertEqual(diagnostic['delivery_elapsed_ms'], 110.)
        self.assertEqual(diagnostic['deadline_ms'], 95.)
        self.assertEqual(record['cycle_ms'], 150.)

    def test_explicit_solver_failure_is_not_reclassified_by_elapsed_time(self):
        self.solve_seconds = .120
        self.solve_error = 'command_hold_or_braking_rejected_by_latest_state_or_map'
        record = self.cycle()
        self.assertEqual(record['state'], self.solve_error)
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_called_once()
        self.assertTrue(self.node.fault_pub.publish.call_args.args[0].data)
        # No normal plan return took place, so no return-time sample exists.
        self.assertNotIn('planning_elapsed_ms', record['candidate_diagnostics'])

    def test_joint_feedback_gap_pauses_then_recovers_the_same_navigation_goal(self):
        self.node.model = NS(lower=np.full(6, -1.), upper=np.ones(6))
        self.node.input_errors = {}
        self.node.joint_stamp = 99.8
        with patch('whole_body_mppi.rospy.get_time', return_value=100.):
            self.node.on_joints(delivery_fixture.joint_message(99.82))
        record = self.cycle()
        self.assertEqual(record['state'], 'state_feedback_stale')
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_not_called()
        self.assert_active_reference_preserved()

        # Fresh source-stamped state and authority allow a new solve. The old
        # goal/revision stays intact, with no automatic hardware enable/reset.
        self.fixture.now = 100.01
        self.node.base_stamp = 100.01
        with patch('whole_body_mppi.rospy.get_time', return_value=100.01):
            self.node.on_joints(delivery_fixture.joint_message(100.01))
        self.fixture.feed(delivery_fixture.message(100.01), now=100.01)
        self.node.base_pub.publish.reset_mock()
        record = self.cycle()
        self.assertEqual(record['state'], 'command_delivered')
        self.node.arm_pub.publish.assert_called_once()
        self.assertFalse(self.node.fault_pub.publish.call_args.args[0].data)
        self.assert_active_reference_preserved()

    def test_queued_joint_packet_during_solve_does_not_abort_navigation(self):
        self.node.model = NS(lower=np.full(6, -1.), upper=np.ones(6))
        self.node.input_errors = {}
        self.during_solve = lambda: self.node.on_joints(delivery_fixture.joint_message(99.8))
        record = self.cycle()
        self.assertEqual(record['state'], 'command_delivered')
        self.assertEqual(self.node.joint_stamp, 99.99)
        self.node.arm_pub.publish.assert_called_once()
        self.assertFalse(self.node.fault_pub.publish.call_args.args[0].data)
        self.assert_active_reference_preserved()

    def test_feedback_authority_gap_pauses_without_fault_and_recovers_next_cycle(self):
        self.fixture.feed(delivery_fixture.message(100., valid=False), now=100.)
        record = self.cycle()
        self.assertEqual(record['state'], 'waiting_explicit_arm_enable_or_normal_status')
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_not_called()
        self.assert_active_reference_preserved()
        self.assertFalse(self.node.execution_latched)

        self.fixture.now = 100.01
        self.node.base_stamp = self.node.joint_stamp = 100.01
        self.fixture.feed(delivery_fixture.message(100.01), now=100.01)
        self.node.base_pub.publish.reset_mock()
        record = self.cycle()
        self.assertEqual(record['state'], 'command_delivered')
        self.node.arm_pub.publish.assert_called_once()
        self.assertFalse(self.node.fault_pub.publish.call_args.args[0].data)
        self.assert_active_reference_preserved()

    def test_cancel_during_late_return_stays_cancelled_on_following_cycle(self):
        self.solve_seconds = .120
        self.has_result = False
        self.feasible = 0

        def cancel_reference():
            # Same cancellation state transition visible to cycle(): active
            # goal stays pending while its trajectory and revision change.
            self.node.reference.revision += 1
            self.reference_status = 'native_reference_cancelled'

        self.during_solve = cancel_reference
        record = self.cycle()
        self.assertEqual(record['state'], 'planning_deadline_exceeded')
        self.assertEqual(record['candidate_diagnostics']['reference_status'],
                         'native_reference_cancelled')
        self.assertEqual(self.node.reference.revision, 1)
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_not_called()

        self.during_solve = lambda: None
        self.solve_seconds = .050
        self.feasible = None
        next_record = self.cycle()
        self.assertEqual(next_record['state'], 'native_reference_cancelled')
        self.assertEqual(self.node.reference.revision, 1)
        self.assert_zero_without_arm()
        self.node.fault_pub.publish.assert_not_called()
        self.assertEqual(self.node.reference.plan.call_count, 2)

    def test_exact_deadline_keeps_existing_strict_greater_than_boundary(self):
        self.solve_seconds = .095
        record = self.cycle()
        self.assertEqual(record['state'], 'command_delivered')
        self.node.arm_pub.publish.assert_called_once()
        self.node.base_pub.publish.assert_called_once()
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].linear.x, .02)
        diagnostic = record['candidate_diagnostics']
        self.assertEqual(diagnostic['planning_elapsed_ms'], 95.)
        self.assertEqual(diagnostic['delivery_elapsed_ms'], 95.)


if __name__ == '__main__':
    unittest.main()

"""Failure attribution and deadline precedence; fake geometry/ROS, no hardware."""
import json
import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_mppi_collision as collision
import test_mppi_execution_gate as gates
from active_arm_mppi import Snapshot
from whole_body_mppi import checked_first_step


class CommandDiagnosticTests(unittest.TestCase):
    def setUp(self):
        fixture = collision.MoveJIntentTests(methodName='runTest')
        fixture.setUp()
        self.kernel, self.state, self.control = fixture.kernel, fixture.state, fixture.control
        self.scene = fixture.Scene()

    def rejected(self):
        detail = {}
        with self.assertRaisesRegex(ValueError, '^command_hold_or_braking_rejected_by_latest_state_or_map$'):
            checked_first_step(self.kernel, self.state, self.control, self.scene, 100., detail)
        return detail['command_check']

    def test_success_does_not_add_diagnostics_or_geometry_queries(self):
        expected = checked_first_step(self.kernel, self.state, self.control, self.scene, 100.)
        self.scene.query = Mock(wraps=self.scene.query)
        self.kernel.model.configuration_gaps_batch = Mock(wraps=self.kernel.model.configuration_gaps_batch)
        detail = {'previous': 7}
        actual = checked_first_step(self.kernel, self.state, self.control, self.scene, 100., detail)
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(detail, {'previous': 7})
        self.scene.query.assert_called_once()
        self.kernel.model.configuration_gaps_batch.assert_called_once()

    def test_external_rejection_keeps_independent_configuration_and_bounds_evidence(self):
        self.control[2] = .15
        self.scene.obstacle = (.015, 100.55)
        detail = self.rejected()
        self.assertTrue(detail['external_rejected'])
        self.assertLess(detail['min_external_gap_m'], 0.)
        self.assertFalse(detail['configuration_rejected'])
        self.assertTrue(detail['bounds_valid'])
        self.assertEqual(detail['measured_state'], self.state.tolist())
        self.assertEqual(detail['selected_control'], self.control.tolist())
        self.assertAlmostEqual(detail['joint_target'][0], .015)

    def test_configuration_and_joint_limits_are_distinguished(self):
        self.state[11] = -.534
        self.kernel.model.configuration_floor = -.02
        detail = self.rejected()
        self.assertFalse(detail['external_rejected'])
        self.assertTrue(detail['configuration_rejected'])
        self.assertTrue(detail['bounds_valid'])
        self.kernel.model.configuration_floor = -float('inf')
        self.state[5] = -1.99
        detail = self.rejected()
        self.assertFalse(detail['external_rejected'])
        self.assertFalse(detail['configuration_rejected'])
        self.assertFalse(detail['bounds_valid'])

    def test_real_snapshot_forecast_expiry_is_identified_at_ros_epoch(self):
        now = 1794310000.
        data = collision.wire(resolution=.5, origin=(-1., -1., -1.), value=10.,
                    dynamic=[[100., 100., 100., 0., 0., 0., 1.]])
        data[3] = data[18] = now-.1
        data[15] = .35
        snapshot = Snapshot.decode(data)
        self.assertTrue(snapshot.fresh(now))
        self.state[3] = .2  # Real overspeed is retained, not clipped to profile.
        detail = {}
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel, self.state, self.control, snapshot, now, detail)
        record = detail['command_check']
        self.assertTrue(record['external_rejected'])
        self.assertFalse(record['configuration_rejected'])
        self.assertTrue(record['bounds_valid'])
        self.assertFalse(record['dynamic_prediction_in_range'])
        self.assertGreater(record['dynamic_prediction_max_age_s'], 2.)
        self.assertEqual(record['map_header'], data[:32].tolist())


class CycleDiagnosticTests(unittest.TestCase):
    def prepare(self):
        fixture = gates.ExecutionDeliveryTests(methodName='runTest')
        fixture.setUp()
        fixture.feed(gates.message(100.))
        node = fixture.node
        node.reference.active = True
        node.reference.goal_stamp = node.sent_navigation_goal = 10.
        node.reference.navigation = node.kernel
        node.sent_command = True
        node.deadline = .095
        return fixture, node

    def failed_plan(self, elapsed):
        fixture, node = self.prepare()
        wall = [0.]
        info = dict(feasible=0, candidates=2, structured_valid=[False, False],
                    structured_external_gaps=[-.1, -.2],
                    structured_configuration_gaps=[.01, .02], min_gap=-.1)
        def solve(*args):
            wall[0] = elapsed
            return None, info
        node.reference.plan.side_effect = solve
        with patch('whole_body_mppi.rospy.get_time', return_value=100.), \
                patch('whole_body_mppi.time.monotonic', side_effect=lambda: wall[0]):
            node.cycle()
        node.arm_pub.publish.assert_not_called()
        for call in node.base_pub.publish.call_args_list:
            self.assertEqual((call.args[0].linear.x, call.args[0].angular.z), (0., 0.))
        record = json.loads(node.info_pub.publish.call_args.args[0].data)
        self.assertEqual(record['candidate_diagnostics']['structured_external_gaps'], [-.1, -.2])
        self.assertEqual(record['candidate_diagnostics']['structured_valid'], [False, False])
        self.assertAlmostEqual(record['candidate_diagnostics']['solve_ms'], elapsed*1000)
        return node, record

    def test_expired_none_result_is_only_local_deadline_retry(self):
        for elapsed in (.17083, .17507):
            with self.subTest(elapsed=elapsed):
                node, record = self.failed_plan(elapsed)
                self.assertEqual(record['state'], 'planning_deadline_exceeded')
                node.fault_pub.publish.assert_not_called()

    def test_timely_real_no_feasible_still_requests_navigation_recovery(self):
        node, record = self.failed_plan(.070)
        self.assertEqual(record['state'], 'no_feasible_candidate')
        node.fault_pub.publish.assert_called_once()
        self.assertTrue(node.fault_pub.publish.call_args.args[0].data)

    def test_real_cycle_exports_failed_command_check(self):
        fixture, node = self.prepare()
        geometry = collision.MoveJIntentTests(methodName='runTest')
        geometry.setUp()
        node.reference.active = False
        node.kernel = geometry.kernel
        node.snapshot = geometry.Scene((.005, 100.))
        with patch('whole_body_mppi.rospy.get_time', return_value=100.):
            node.cycle()
        node.arm_pub.publish.assert_not_called()
        record = json.loads(node.info_pub.publish.call_args.args[0].data)
        self.assertEqual(record['state'], 'command_hold_or_braking_rejected_by_latest_state_or_map')
        self.assertTrue(record['candidate_diagnostics']['command_check']['external_rejected'])


if __name__ == '__main__':
    unittest.main()

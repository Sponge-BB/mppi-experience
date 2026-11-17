"""Bounded final-pose settling; fake messages, no ROS master or hardware."""
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

import test_mppi_execution_gate as gates
from test_mppi_native_phase import Kernel, goal, trajectory, progress
from native_reference_adapter import NativeReferenceAdapter
from active_arm_mppi import WholeBodyMPPI
from whole_body_mppi import WholeBodyNode


CONFIG = dict(position_tolerance=.15, yaw_tolerance=.15, arm_tolerance=.12,
              timeout=8., gain=1., max_w=.15)


def arm_kernel(kernel):
    kernel.enabled = True
    kernel.arm_position_servo = False
    kernel.dt = .1
    kernel.vmax = np.full(6, .3)
    kernel.qmin, kernel.qmax = np.full(6, -3.), np.full(6, 3.)
    return kernel


def ready(adapter, now=100., x=1.6, yaw=.5):
    adapter.finish()
    request = goal()
    request.pose.orientation.z = np.sin(yaw/2)
    request.pose.orientation.w = np.cos(yaw/2)
    adapter.feed_goal(request, 10.01)
    adapter.feed_trajectory(trajectory(x=x), 10.11)
    adapter.feed_progress(progress(now, phase=2., scale=0.), now)
    state = np.zeros(17)
    state[0], state[5] = 2., 2.
    return state


class TerminalEligibilityTests(unittest.TestCase):
    def setUp(self):
        self.adapter = NativeReferenceAdapter(arm_kernel(Kernel()), arm_kernel(Kernel()))
        self.state = ready(self.adapter)

    def command(self, now=100.):
        return self.adapter.terminal_pose_command(self.state, now, CONFIG)

    def test_at_arm_endpoint_only_bounded_yaw_intent_and_no_translation(self):
        command = self.command()
        self.assertAlmostEqual(command[1], .15)
        np.testing.assert_array_equal(command[np.r_[0, 2:8]], 0.)
        self.state[2] = .4
        np.testing.assert_array_equal(self.command(), np.zeros(8))

    def test_yaw_uses_goal_quaternion_and_shortest_wrapped_direction(self):
        self.state = ready(self.adapter, yaw=-np.pi+.2)
        self.state[2] = np.pi-.2
        self.assertGreater(self.command()[1], 0.)

    def test_local_endpoint_inside_arrival_radius_is_not_final_goal(self):
        self.state = ready(self.adapter, x=1.55)
        self.assertIsNone(self.command())

    def test_controller_completion_tolerance_enters_alignment_but_earlier_pause_does_not(self):
        for remaining, expired in ((.5e-6, True), (1e-6, True), (2e-6, False)):
            with self.subTest(remaining=remaining):
                self.setUp()
                self.adapter.progress = (100., 2.-remaining, 0.)
                status = self.adapter.status(100.)['reference_status']
                self.assertEqual(status == 'native_reference_expired', expired)
                self.assertEqual(self.command() is not None, expired)

    def test_position_or_arm_outside_local_capture_never_aligns(self):
        for index, delta in ((0, .151), (5, .301)):
            with self.subTest(index=index):
                self.setUp()
                self.state[index] += delta
                self.assertIsNone(self.command())

    def test_small_arm_error_beyond_finish_tolerance_converges_without_translation(self):
        self.state[5] += .121
        self.state[11] = -.2
        command = self.command()
        self.assertEqual(command[0], 0.)
        self.assertAlmostEqual(command[2], -1.8*.121+.2*.2)
        self.assertLess(self.state[5]+self.adapter.navigation.dt*command[2], self.state[5])
        self.assertGreater(self.state[5]+self.adapter.navigation.dt*command[2], 2.)

    def test_arm_velocity_step_and_position_bounds_match_both_execution_models(self):
        for servo in (False, True):
            with self.subTest(servo=servo):
                self.setUp()
                kernel = self.adapter.navigation
                kernel.arm_position_servo = servo
                self.state[5] = 1.701
                command = self.command()
                lead = .12+.5*kernel.dt if servo else kernel.dt
                self.assertLessEqual(np.max(np.abs(command[2:])), .3)
                self.assertAlmostEqual(command[2]*lead, .02)
                # A large measured velocity must not make the delivered small
                # position lead cross the endpoint in either execution model.
                self.state[5], self.state[11] = 1.999, -1.
                command = self.command()
                target = (WholeBodyMPPI.position_target(kernel, self.state, command) if servo else
                          np.clip(self.state[5:11]+kernel.dt*command[2:], kernel.qmin, kernel.qmax))
                self.assertGreater(target[0], self.state[5])
                self.assertLessEqual(target[0], 2.)
                self.state[5], self.state[11], kernel.qmax[0] = 1.98, 0., 1.99
                command = self.command()
                self.assertLessEqual(self.state[5]+lead*command[2], 1.99)

    def test_arm_disabled_preserves_yaw_only_behavior(self):
        self.state[5] += .13
        self.adapter.navigation.enabled = False
        command = self.command()
        self.assertGreater(command[1], 0.)
        np.testing.assert_array_equal(command[2:], 0.)

    def test_terminal_window_is_bounded_and_stale_progress_does_not_restart_it(self):
        self.assertIsNotNone(self.command())
        self.assertIsNone(self.command(100.251))
        self.adapter.feed_progress(progress(107.999, phase=2., scale=0.), 107.999)
        self.assertIsNotNone(self.command(107.999))
        self.adapter.feed_progress(progress(108., phase=2., scale=0.), 108.)
        self.assertIsNone(self.command(108.))
        self.adapter.feed_progress(progress(109., phase=2., scale=0.), 109.)
        self.assertIsNone(self.command(109.))
        self.assertEqual(self.adapter.terminal_window[1], 100.)

    def test_accepted_new_trajectory_has_its_own_window_and_old_reference_cannot_renew_it(self):
        self.command()
        self.assertEqual(self.adapter.feed_trajectory(trajectory(x=1.6), 100.01), 'stale_reference')
        self.assertEqual(self.adapter.terminal_window[1], 100.)
        self.assertEqual(self.adapter.feed_trajectory(trajectory(start=108., x=1.6), 108.), 'replace')
        self.assertIsNone(self.adapter.terminal_window)
        self.adapter.feed_progress(progress(108.1, phase=2., scale=0., start=108.), 108.1)
        self.assertIsNotNone(self.command(108.1))
        self.assertEqual(self.adapter.terminal_window[1], 108.1)

    def test_missing_stale_pending_or_unfinished_progress_never_turns(self):
        for case in ('missing', 'stale', 'future', 'pending', 'unfinished'):
            with self.subTest(case=case):
                self.setUp()
                now = 100.
                if case == 'missing': self.adapter.progress = None
                elif case == 'stale': now = 100.251
                elif case == 'future': now = 99.99
                elif case == 'pending': self.adapter.pending = {}
                else: self.adapter.progress = (100., 1.9, 0.)
                self.assertIsNone(self.command(now))

    def test_new_goal_cancel_finish_and_invalid_goal_invalidate_alignment(self):
        for case in ('new_goal', 'cancel', 'finish', 'invalid'):
            with self.subTest(case=case):
                self.setUp()
                if case == 'new_goal': self.adapter.feed_goal(goal(100.01), 100.01)
                elif case == 'cancel':
                    msg = trajectory(); msg.action = 2
                    self.adapter.feed_trajectory(msg, 100.01)
                elif case == 'finish': self.adapter.finish()
                else:
                    msg = goal(100.01); msg.pose.orientation.w = 0.
                    self.adapter.feed_goal(msg, 100.01)
                self.assertIsNone(self.command(100.02))
                if case in ('finish', 'invalid'):
                    self.assertIsNone(self.adapter.goal_pose)


class TerminalDeliveryTests(unittest.TestCase):
    solve = gates.ExecutionDeliveryTests.solve
    feed = gates.ExecutionDeliveryTests.feed

    def setUp(self):
        gates.ExecutionDeliveryTests.setUp(self)
        node = self.node
        arm_kernel(node.kernel)
        node.kernel.H = 8
        node.kernel.lease_brake_states = Mock(side_effect=lambda state, controls, lease:
                                              np.tile(state, (4, 1)))
        node.reference = NativeReferenceAdapter(node.kernel, node.kernel, clock=lambda: self.now)
        self.state = ready(node.reference)
        node.base, node.joints = self.state[:5].copy(), self.state[5:].copy()
        node.terminal_alignment = dict(CONFIG)
        self.checked = self.state.copy()
        self.feed(gates.message(100.))

    def cycle(self, failure=None):
        with patch('whole_body_mppi.rospy.get_time', side_effect=lambda: self.now), \
                patch('whole_body_mppi.checked_first_step', return_value=self.checked,
                      side_effect=failure) as checked:
            self.node.cycle()
        return checked

    def mutate_after_candidate(self, action):
        def prediction(state, controls, lease):
            action()
            return np.tile(state, (4, 1))
        self.node.kernel.lease_brake_states.side_effect = prediction

    def assert_stopped(self):
        self.node.arm_pub.publish.assert_not_called()
        for call in self.node.base_pub.publish.call_args_list:
            self.assertEqual((call.args[0].linear.x, call.args[0].angular.z), (0., 0.))

    def test_alignment_runs_existing_collision_check_and_keeps_source_stamp(self):
        checked = self.cycle()
        self.assertAlmostEqual(checked.call_args.args[2][1], .15)
        np.testing.assert_array_equal(checked.call_args.args[2][2:], np.zeros(6))
        self.assertIs(checked.call_args.args[3], self.node.snapshot)
        self.assertEqual(self.node.arm_pub.publish.call_args.args[0].header.stamp.to_sec(), 100.)
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].linear.x, 0.)
        self.assertAlmostEqual(self.node.base_pub.publish.call_args.args[0].angular.z, .15)
        self.assertEqual(json.loads(self.node.info_pub.publish.call_args.args[0].data)['mode'],
                         'terminal_pose_alignment')

    def test_arm_settling_uses_same_checked_command_and_published_bounded_target(self):
        def check(kernel, state, command, snapshot, now):
            result = state.copy()
            result[5:11] = (WholeBodyMPPI.position_target(kernel, state, command)
                           if kernel.arm_position_servo else
                           np.clip(state[5:11]+kernel.dt*command[2:], kernel.qmin, kernel.qmax))
            return result
        for servo in (False, True):
            for actual in (2.13, 1.701):
                with self.subTest(servo=servo, actual=actual):
                    self.setUp()
                    self.node.kernel.arm_position_servo = servo
                    self.node.joints[0] = actual
                    with patch('whole_body_mppi.rospy.get_time', side_effect=lambda: self.now), \
                            patch('whole_body_mppi.checked_first_step', side_effect=check) as checked:
                        self.node.cycle()
                    command = checked.call_args.args[2]
                    self.assertEqual(command[0], 0.)
                    self.assertGreater(command[2]*(2.-actual), 0.)
                    target = self.node.arm_pub.publish.call_args.args[0].position[0]
                    self.assertLess(abs(target-2.), abs(actual-2.))
                    self.assertLessEqual(abs(target-actual), .02+1e-12)

    def test_timeout_keeps_zero_hold_and_collision_failure_does_not_renew_window(self):
        self.cycle(ValueError('command_hold_or_braking_rejected_by_latest_state_or_map'))
        self.assertEqual(self.node.reference.terminal_window[1], 100.)
        self.feed(gates.message(108.), now=108.)
        self.node.base_stamp = self.node.joint_stamp = 108.
        self.node.reference.feed_progress(progress(108., phase=2., scale=0.), 108.)
        checked = self.cycle()
        np.testing.assert_array_equal(checked.call_args.args[2], 0.)
        record = json.loads(self.node.info_pub.publish.call_args.args[0].data)
        self.assertEqual(record['mode'], 'reference_wait_hold')
        self.assertEqual(self.node.reference.terminal_window[1], 100.)

    def test_latest_yaw_is_recomputed_before_collision_check(self):
        self.mutate_after_candidate(lambda: self.node.base.__setitem__(2, .4))
        checked = self.cycle()
        self.assertEqual(checked.call_args.args[2][1], 0.)
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].angular.z, 0.)

    def test_latest_arm_is_recomputed_before_collision_check(self):
        self.checked[5] = 2.11
        self.mutate_after_candidate(lambda: self.node.joints.__setitem__(0, 2.13))
        checked = self.cycle()
        self.assertLess(checked.call_args.args[2][2], 0.)

    def test_zero_safety_limit_blocks_rotation_before_collision_check(self):
        self.node.safety_speed_limit = 0.
        checked = self.cycle()
        self.assertEqual(checked.call_args.args[2][1], 0.)
        self.assertEqual(self.node.base_pub.publish.call_args.args[0].angular.z, 0.)

    def test_latest_goal_position_or_progress_loss_blocks_delivery(self):
        for case in ('position', 'progress', 'arm'):
            with self.subTest(case=case):
                self.setUp()
                action = ((lambda: self.node.base.__setitem__(0, 2.2)) if case == 'position' else
                          (lambda: self.node.joints.__setitem__(0, 2.301)) if case == 'arm' else
                          (lambda: setattr(self.node.reference, 'progress', None)))
                self.mutate_after_candidate(action)
                self.cycle()
                self.assert_stopped()

    def test_cancel_or_new_goal_after_candidate_preserves_revision_gate(self):
        for case in ('cancel', 'new_goal'):
            with self.subTest(case=case):
                self.setUp()
                def invalidate():
                    if case == 'new_goal': self.node.reference.feed_goal(goal(100.01), 100.01)
                    else:
                        msg = trajectory(); msg.action = 2
                        self.node.reference.feed_trajectory(msg, 100.01)
                self.mutate_after_candidate(invalidate)
                self.cycle()
                self.assert_stopped()

    def test_collision_rejection_and_driver_stop_never_deliver_alignment(self):
        self.cycle(ValueError('command_hold_or_braking_rejected_by_latest_state_or_map'))
        self.assert_stopped()
        self.setUp()
        self.mutate_after_candidate(lambda: self.feed(gates.message(100.01, stopped=True), now=100.01))
        self.cycle()
        self.assert_stopped()

    def test_expired_map_never_delivers_alignment(self):
        self.mutate_after_candidate(lambda: setattr(self.node.snapshot, 'fresh', lambda now: False))
        self.cycle()
        self.assert_stopped()

    def test_goal_transform_failure_clears_stored_goal_pose(self):
        msg = goal(100.01); msg.header.frame_id = 'missing_frame'
        self.node.tf = NS(lookup_transform=Mock(side_effect=ValueError('unavailable')))
        self.node.on_goal(msg)
        self.assertIsNone(self.node.reference.goal_pose)
        self.assertFalse(self.node.reference.native.refs)

    def test_finish_after_capture_rejects_old_anchor_before_planning(self):
        capture = self.node.capture
        self.node.anchor = np.array([-10., 0., 0.])
        def finish_after_capture(now):
            result = capture(now)
            self.node.on_finish(NS(data=True))
            return result
        with patch.object(self.node, 'capture', side_effect=finish_after_capture), \
                patch.object(self.node.reference, 'plan', wraps=self.node.reference.plan) as plan:
            self.cycle()
        plan.assert_not_called()
        self.assert_stopped()
        self.assertIsNone(self.node.anchor)
        self.assertEqual(json.loads(self.node.info_pub.publish.call_args.args[0].data)['state'],
                         'reference_changed_during_plan')

    def test_finish_inside_plan_cannot_replace_capture_revision_with_new_revision(self):
        def finish_during_plan(*args):
            self.node.on_finish(NS(data=True))
            return (np.zeros((8, 8)), np.zeros((9, 17))), dict(
                feasible=1, reference_revision=self.node.reference.revision)
        with patch.object(self.node.reference, 'plan', side_effect=finish_during_plan):
            self.cycle()
        self.assert_stopped()
        self.assertEqual(json.loads(self.node.info_pub.publish.call_args.args[0].data)['state'],
                         'reference_changed_during_plan')


if __name__ == '__main__':
    unittest.main()

"""Offline virtual-phase tests with fake messages and kernels; no ROS master."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'/'mppi'))
from native_reference_adapter import NativeReferenceAdapter


def header(stamp):
    return NS(stamp=NS(to_sec=lambda: stamp), frame_id='world')


def goal(stamp=10.):
    return NS(header=header(stamp), pose=NS(position=NS(x=2., y=0., z=0.),
        orientation=NS(x=0., y=0., z=0., w=1.)))


def trajectory(start=10.1, ident=1, x=1., gear=1, arm_offset=0.):
    coefficients = np.zeros((8, 8))
    coefficients[0, -2:] = [.2*gear, x]
    coefficients[2, -2:] = [1., arm_offset]
    piece = NS(num_dim=8, num_order=7, duration=2., data=coefficients.ravel(order='F').tolist())
    msg = NS(header=header(start), action=1, trajectory_id=ident, singul=gear, trajectory=[piece])
    msg.serialize = lambda stream: stream.write(repr((start, ident, gear)).encode()+coefficients.tobytes())
    return msg


def progress(stamp, phase=.4, scale=.5, start=10.1):
    return NS(header=header(stamp), vector=NS(x=start, y=phase, z=scale))


class Kernel:
    H, dt = 8, .1

    def __init__(self):
        self.previous = None
        self.calls = []
        self.prediction_times = []
        self.tracking_calls = []
        self.during_plan = lambda: None

    def plan(self, state, reference, arm, snapshot, now, **kwargs):
        self.calls.append((reference.copy(), arm.copy()))
        self.prediction_times.append(now)
        self.tracking_calls.append(kwargs.get('tracking'))
        self.during_plan()
        return (np.zeros((8, 8)), np.zeros((9, 17))), {'feasible': 1}


class NativePhaseTests(unittest.TestCase):
    def setUp(self):
        self.standby, self.navigation = Kernel(), Kernel()
        self.adapter = NativeReferenceAdapter(self.standby, self.navigation)

    def ready(self, now=20., phase=.4, scale=.5):
        self.assertEqual(self.adapter.feed_goal(goal(), 10.01), 'accepted')
        self.assertEqual(self.adapter.feed_trajectory(trajectory(), 10.11), 'replace')
        self.assertEqual(self.adapter.feed_progress(progress(now, phase, scale), now), 'accepted')

    def plan(self, now=20.05):
        return self.adapter.plan(np.zeros(17), np.zeros((1, 4)), np.zeros(6), None, now)

    def test_standby_then_goal_waits_for_real_reference_and_progress(self):
        self.assertIsNotNone(self.plan()[0])
        self.adapter.feed_goal(goal(), 10.01)
        self.assertIsNone(self.plan()[0])
        self.adapter.feed_trajectory(trajectory(), 10.11)
        self.assertEqual(self.plan()[1]['reference_status'], 'waiting_native_progress')
        self.assertEqual(self.adapter.feed_progress(progress(20., start=9.), 20.), 'stale_progress')
        self.assertIsNone(self.plan()[0])

    def test_virtual_phase_drives_base_and_arm_after_wall_duration_has_elapsed(self):
        self.ready()
        result, info = self.plan()
        self.assertIsNotNone(result)
        path, arm = self.navigation.calls[-1]
        phase = .4+.5*.05
        self.assertAlmostEqual(path[0, 0], 1.+.2*phase)
        self.assertAlmostEqual(path[-1, 0], 1.+.2*(phase+.5*.1*8))
        np.testing.assert_allclose(arm[:, 0], phase+.5*.1*np.arange(1, 9))
        np.testing.assert_allclose(path[:, 3], .1)
        self.assertAlmostEqual(info['native_phase'], phase)

    def test_pause_holds_phase_and_current_pose_but_remains_usable(self):
        self.ready(scale=0.)
        self.assertIsNotNone(self.plan()[0])
        path, arm = self.navigation.calls[-1]
        np.testing.assert_allclose(path[0], path[-1])
        np.testing.assert_allclose(path[:, 3], 0.)
        np.testing.assert_allclose(arm[:, 0], .4)

    def test_progress_expires_by_header_and_invalid_values_cannot_refresh_it(self):
        self.ready()
        self.assertEqual(self.plan(20.251)[1]['reference_status'], 'native_progress_stale')
        for bad_scale in (-.1, 1.1, float('nan')):
            self.assertEqual(self.adapter.feed_progress(progress(20.3, scale=bad_scale), 20.3), 'invalid')
            self.assertIsNone(self.plan(20.31)[0])

    def test_future_trajectory_does_not_start_early(self):
        self.adapter.feed_goal(goal(), 10.01)
        self.adapter.feed_trajectory(trajectory(start=11.), 10.11)
        self.adapter.feed_progress(progress(10.5, start=11.), 10.5)
        self.assertEqual(self.plan(10.55)[1]['reference_status'], 'native_reference_not_started')

    def test_future_replacement_preserves_active_and_routes_progress_by_start(self):
        self.ready(now=20., phase=.4, scale=.5)
        original_revision = self.adapter.revision
        self.adapter.feed_trajectory(trajectory(start=20.7, x=4., arm_offset=9.), 20.02)
        self.assertAlmostEqual(self.adapter.native.start, 10.1)
        self.assertEqual(self.adapter.revision, original_revision)
        # The native controller already announces its future reference. This
        # must not overwrite the old curve's phase or its arm polynomial.
        self.adapter.feed_progress(progress(20.3, phase=0., scale=0., start=20.7), 20.3)
        result, info = self.plan(20.31)
        self.assertIsNotNone(result)
        self.assertEqual(info['native_phase_source'], 'handoff_bridge')
        self.assertAlmostEqual(info['native_phase'], .4+.5*.31)
        self.assertLess(np.max(self.navigation.calls[-1][1][:, 0]), 2.)
        # Both base and arm switch with the matching new clock at activation.
        self.adapter.feed_progress(progress(20.7, phase=0., scale=1., start=20.7), 20.7)
        result, info = self.plan(20.71)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(info['source_stamp'], 20.7)
        self.assertGreater(self.navigation.calls[-1][0][0, 0], 4.)
        self.assertGreater(np.min(self.navigation.calls[-1][1][:, 0]), 9.)
        self.assertIsNone(self.adapter.pending)

    def test_future_bridge_does_not_renew_stale_controller_progress(self):
        self.ready(now=20.)
        self.adapter.feed_trajectory(trajectory(start=20.7), 20.02)
        self.adapter.feed_progress(progress(20.1, phase=0., scale=0., start=20.7), 20.1)
        self.assertEqual(self.plan(20.4)[1]['reference_status'], 'native_progress_stale')

    def test_future_prefix_appends_are_kept_separate_from_active(self):
        self.ready(now=20.)
        self.adapter.feed_trajectory(trajectory(start=20.7, x=4.), 20.02)
        self.adapter.feed_trajectory(trajectory(start=20.7, ident=2, x=4.4, gear=-1), 20.03)
        self.assertEqual(self.adapter.native.last_id, 1)
        self.assertEqual(self.adapter.pending['native'].last_id, 2)
        self.adapter.feed_progress(progress(20.7, phase=0., scale=1., start=20.7), 20.7)
        self.assertEqual(self.adapter.native.last_id, 2)

    def test_crossing_pending_activation_during_solve_discards_old_result(self):
        self.ready(now=20.)
        self.adapter.feed_trajectory(trajectory(start=20.7), 20.02)
        self.adapter.feed_progress(progress(20.6, phase=0., scale=0., start=20.7), 20.6)
        self.adapter.clock = iter((20.69, 20.71)).__next__
        result, info = self.plan(20.69)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'reference_changed_during_plan')

    def test_future_candidate_without_new_progress_cannot_execute_old_after_activation(self):
        self.ready(now=20.)
        self.adapter.feed_trajectory(trajectory(start=20.7), 20.02)
        result, info = self.plan(20.71)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'waiting_native_progress')

    def test_cancel_clears_active_and_pending_references(self):
        self.ready(now=20.)
        self.adapter.feed_trajectory(trajectory(start=20.7), 20.02)
        msg = trajectory(); msg.action = 2
        self.adapter.feed_trajectory(msg, 20.03)
        self.assertIsNone(self.adapter.pending)
        self.assertFalse(self.adapter.native.refs)

    def test_one_clock_tick_ahead_does_not_invalidate_navigation(self):
        self.ready()
        revision = self.adapter.revision
        self.assertEqual(self.adapter.feed_progress(progress(20.015, .4075), 20.01), 'accepted')
        self.assertEqual(self.adapter.revision, revision)
        self.assertAlmostEqual(self.adapter.status(20.01)['native_phase'], .405)
        self.navigation.during_plan = lambda: self.adapter.feed_progress(
            progress(20.02, .41), 20.015)
        with patch('native_reference_adapter.time.monotonic', side_effect=[100., 100.005]):
            result, _ = self.plan(20.01)
        self.assertIsNotNone(result)
        # Larger future timestamps and old input retain the rejection contract.
        self.assertEqual(self.adapter.feed_progress(progress(20.03), 20.024), 'invalid')
        self.assertIsNone(self.plan(20.03)[0])

    def test_current_gear_does_not_jump_to_return_path_and_arm_stops_at_boundary(self):
        self.ready(phase=1.8, scale=1.)
        self.assertEqual(self.adapter.feed_trajectory(
            trajectory(ident=2, x=1.4, gear=-1, arm_offset=9.), 10.12), 'append')
        result, info = self.plan()
        self.assertIsNotNone(result)
        path, arm = self.navigation.calls[-1]
        self.assertTrue(np.all(path[:2, 3] > 0.))
        np.testing.assert_array_equal(path[2:, 3], 0.)
        self.assertGreater(path[0, 0], 1.36)
        self.assertAlmostEqual(path[-1, 0], 1.4)
        np.testing.assert_allclose(arm[:, 0], np.minimum(2., 1.85+.1*np.arange(1, 9)))
        self.assertEqual(info['native_arm_tail_policy'], 'hold_current_gear_block_endpoint')
        self.adapter.feed_progress(progress(20.1, phase=2.1, scale=1.), 20.1)
        self.assertIsNotNone(self.plan(20.15)[0])
        path, arm = self.navigation.calls[-1]
        self.assertTrue(np.all(path[:, 3] < 0.))
        self.assertLess(path[-1, 0], path[0, 0])
        np.testing.assert_allclose(arm[:, 0], 9.+.15+.1*np.arange(1, 9))

    def test_exact_gear_boundary_matches_native_previous_block_convention(self):
        self.ready(phase=2., scale=0.)
        self.assertEqual(self.adapter.feed_trajectory(
            trajectory(ident=2, x=1.4, gear=-1, arm_offset=9.), 10.12), 'append')
        result, info = self.plan(20.)
        self.assertIsNotNone(result)
        self.assertEqual(info['native_gear'], 1)
        self.assertEqual(info['native_gear_block'], 0)
        np.testing.assert_array_equal(self.navigation.calls[-1][0][:, 3], 0.)
        np.testing.assert_allclose(self.navigation.calls[-1][1][:, 0], 2.)
        self.adapter.feed_progress(progress(20.1, phase=2.+1e-8, scale=0.), 20.1)
        result, info = self.plan(20.1)
        self.assertIsNotNone(result)
        self.assertEqual(info['native_gear'], -1)
        self.assertEqual(info['native_gear_block'], 1)

    def test_new_goal_and_replacement_clear_previous_progress_and_caches(self):
        self.ready()
        self.navigation.previous = self.standby.previous = np.ones((2, 2))
        self.adapter.feed_goal(goal(20.1), 20.11)
        self.assertIsNone(self.adapter.progress)
        self.assertIsNone(self.navigation.previous)
        self.assertEqual(self.adapter.feed_trajectory(trajectory(), 20.12), 'stale_reference')
        self.adapter.feed_trajectory(trajectory(start=20.2), 20.21)
        self.assertEqual(self.adapter.feed_progress(progress(20.3), 20.3), 'stale_progress')
        self.assertIsNone(self.plan(20.31)[0])

    def test_cancel_and_finish_do_not_reuse_old_trajectory(self):
        self.ready()
        cancel = trajectory(); cancel.action = 2
        self.assertEqual(self.adapter.feed_trajectory(cancel, 20.1), 'cancel')
        self.assertIsNone(self.adapter.progress)
        self.assertIsNone(self.plan(20.11)[0])
        self.adapter.finish()
        self.assertFalse(self.adapter.active)
        self.assertIsNotNone(self.plan(20.2)[0])
        self.assertEqual(len(self.standby.calls), 1)
        self.assertEqual(self.adapter.feed_trajectory(trajectory(20.3), 20.31), 'ignored_without_valid_goal')

    def test_phase_at_prefix_end_is_expired_even_when_wall_clock_is_early(self):
        self.ready(now=10.2, phase=2., scale=0.)
        self.assertEqual(self.plan(10.21)[1]['reference_status'], 'native_reference_expired')

    def test_post_solve_freshness_uses_actual_ros_clock_not_wall_extrapolation(self):
        self.ready()
        self.adapter.clock = lambda: 20.2
        self.navigation.during_plan = lambda: self.adapter.feed_progress(
            progress(20.2, phase=.5), 20.2)
        with patch('native_reference_adapter.time.monotonic', return_value=100.):
            result, info = self.plan(20.05)
        self.assertIsNotNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_ready')

    def test_new_progress_since_cycle_start_uses_actual_time_for_phase_and_collision(self):
        self.ready(now=20.)
        # A callback updates progress while this cycle waits to sample it.
        self.assertEqual(self.adapter.feed_progress(progress(20.01, phase=.405), 20.02), 'accepted')
        def clock():
            self.assertTrue(self.adapter.plan_lock.locked())
            self.assertTrue(self.adapter.lock._is_owned())
            return 20.02
        self.adapter.clock = clock
        result, info = self.plan(20.)
        self.assertIsNotNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_ready')
        self.assertAlmostEqual(info['native_phase'], .410)
        self.assertEqual(self.navigation.prediction_times, [20.02])
        self.assertEqual(self.adapter.progress[0], 20.01)  # No source restamping.

    def test_without_clock_preserves_caller_prediction_time(self):
        self.ready()
        self.assertIsNotNone(self.plan(20.05)[0])
        self.assertEqual(self.navigation.prediction_times, [20.05])

    def test_true_future_progress_is_still_rejected(self):
        self.ready()
        self.adapter.clock = lambda: 20.02
        self.assertEqual(self.adapter.feed_progress(progress(20.03), 20.02), 'invalid')
        result, info = self.plan(20.)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_progress_stale')
        self.assertEqual(self.navigation.calls, [])

    def test_actual_clock_rollback_is_not_clamped_to_cycle_time(self):
        self.ready(now=20.01)
        self.adapter.clock = lambda: 20.
        result, info = self.plan(20.02)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_progress_stale')
        self.assertEqual(self.navigation.calls, [])

    def test_actual_clock_does_not_extend_stale_progress(self):
        self.ready()
        self.adapter.clock = lambda: 20.4
        with patch('native_reference_adapter.time.monotonic', return_value=100.):
            result, info = self.plan(20.05)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_progress_stale')
        self.assertEqual(self.navigation.calls, [])

    def test_actual_clock_progress_expiring_during_solve_is_still_rejected(self):
        self.ready()
        self.adapter.clock = iter((20.05, 20.4)).__next__
        result, info = self.plan(20.05)
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_progress_stale_during_plan')
        self.assertEqual(self.navigation.prediction_times, [20.05])

    def test_revision_change_during_plan_rejects_result(self):
        self.ready()
        self.navigation.during_plan = self.adapter.finish
        self.assertEqual(self.plan()[1]['reference_status'], 'reference_changed_during_plan')

    def test_ordinary_progress_updates_do_not_invalidate_every_plan(self):
        self.ready()
        self.navigation.during_plan = lambda: self.adapter.feed_progress(progress(20.04, .42, .5), 20.04)
        self.assertIsNotNone(self.plan()[0])
        self.navigation.during_plan = lambda: self.adapter.feed_progress(progress(20.045, .4225, 0.), 20.045)
        result, info = self.plan()
        self.assertIsNotNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_ready')
        # The in-flight tracking objective is retained, then the next cycle
        # samples the frozen phase. Catch-up and arm avoidance remain possible.
        self.navigation.during_plan = lambda: None
        self.assertIsNotNone(self.plan(20.06)[0])
        path, arm = self.navigation.calls[-1]
        np.testing.assert_allclose(path[0], path[-1])
        np.testing.assert_allclose(path[:, 3], 0.)
        np.testing.assert_allclose(arm[:, 0], .4225)

    def test_continuously_decreasing_progress_does_not_starve_control(self):
        self.ready(now=20., phase=.4, scale=1.)
        revision = self.adapter.revision
        for i in range(1, 11):
            now = 20.+.01*i
            scale = 1.-.1*i
            self.navigation.during_plan = lambda now=now, scale=scale, i=i: self.adapter.feed_progress(
                progress(now, .4+.005*i, scale), now)
            with patch('native_reference_adapter.time.monotonic', side_effect=[100., 100.005]):
                result, info = self.plan(now)
            self.assertIsNotNone(result)
            self.assertEqual(info['reference_status'], 'native_reference_ready')
            self.assertEqual(self.adapter.revision, revision)

    def test_progress_must_still_be_fresh_after_solving(self):
        self.ready()
        with patch('native_reference_adapter.time.monotonic', side_effect=[100., 100.3]):
            result, info = self.plan()
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_progress_stale_during_plan')


if __name__ == '__main__':
    unittest.main()

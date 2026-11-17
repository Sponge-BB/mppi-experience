"""Offline reference provenance and rejection checks; no ROS or hardware."""
import hashlib
import io
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

RELEASE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(RELEASE/'runtime'), str(RELEASE/'core')]
from native_reference_adapter import NativeReferenceAdapter
from active_arm_mppi import WholeBodyMPPI


def header(stamp):
    return NS(stamp=NS(to_sec=lambda: stamp), frame_id='world')


def goal(stamp=10.):
    return NS(header=header(stamp), pose=NS(position=NS(x=2., y=0., z=0.),
        orientation=NS(x=0., y=0., z=0., w=1.)))


def trajectory(stamp=10.1, ident=1, x=1., duration=2., moving_arm=False):
    c = np.zeros((8, 8)); c[0, -2:] = [.2, x]
    c[2:, -1] = np.arange(6)*.1
    if moving_arm:
        c[2, [0, 5, 6]] = [.01, .02, .03]
    p = NS(num_dim=8, num_order=7, duration=duration, data=c.ravel(order='F').tolist())
    msg = NS(header=header(stamp), action=1, trajectory_id=ident, singul=1, trajectory=[p])
    msg.serialize = lambda stream: stream.write(
        repr((stamp, ident, duration)).encode()+c.astype('<f8').tobytes())
    return msg


class Kernel:
    def __init__(self):
        self.previous = None
        self.H, self.dt = 8, .1
        self.calls = []
        self.during_plan = lambda: None

    def plan(self, state, reference, arm, snapshot, now):
        self.calls.append((reference.copy(), arm.copy()))
        self.during_plan()
        return (np.zeros((8, 8)), np.zeros((9, 17))), {'feasible': 1}


class NativeReferenceAdapterTests(unittest.TestCase):
    def setUp(self):
        self.standby, self.navigation = Kernel(), Kernel()
        self.adapter = NativeReferenceAdapter(self.standby, self.navigation)
        self.state, self.anchor, self.arm = np.zeros(17), np.zeros((1, 4)), np.zeros(6)

    def plan(self, now=10.15):
        return self.adapter.plan(self.state, self.anchor, self.arm, None, now)

    def ready(self):
        self.assertEqual(self.adapter.feed_goal(goal(), 10.01), 'accepted')
        msg = trajectory()
        self.assertEqual(self.adapter.feed_trajectory(msg, 10.11), 'replace')
        return msg

    def test_standby_before_goal_and_no_synthetic_path_after_goal(self):
        self.assertIsNotNone(self.plan()[0])
        self.assertEqual(len(self.standby.calls), 1)
        self.adapter.feed_goal(goal(), 10.01)
        result, info = self.plan()
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'waiting_native_reference_after_goal')
        self.assertEqual(self.navigation.calls, [])

    def test_native_coefficients_arm_and_exact_serialized_provenance_are_used(self):
        msg = self.ready()
        result, info = self.plan()
        self.assertIsNotNone(result)
        path, arm = self.navigation.calls[-1]
        np.testing.assert_allclose(path[[0, -1], 0], [1., 1.4])
        np.testing.assert_allclose(path[:, 3], .2)
        np.testing.assert_allclose(arm, np.tile(np.arange(6)*.1, (8, 1)))
        stream = io.BytesIO(); msg.serialize(stream)
        self.assertEqual(info['message_hashes'], [hashlib.sha256(stream.getvalue()).hexdigest()])
        self.assertEqual(info['source_stamp'], 10.1)
        self.assertFalse(info['native_prefix_atomic'])
        self.assertIsNotNone(info['reference_hash'])

    def test_new_goal_clears_reference_and_rejects_old_generated_trajectory(self):
        self.ready()
        self.adapter.feed_goal(goal(10.2), 10.21)
        self.assertEqual(self.adapter.feed_trajectory(trajectory(), 10.22), 'stale_reference')
        self.assertIsNone(self.plan(10.3)[0])
        self.assertEqual(self.adapter.feed_trajectory(trajectory(10.25), 10.26), 'replace')
        self.assertIsNotNone(self.plan(10.31)[0])

    def test_generation_must_follow_goal_receipt_and_late_old_replace_cannot_roll_back(self):
        self.adapter.feed_goal(goal(), 10.2)
        self.assertEqual(self.adapter.feed_trajectory(trajectory(), 10.3), 'stale_reference')
        self.adapter.feed_trajectory(trajectory(10.3), 10.31)
        self.assertEqual(self.adapter.feed_trajectory(trajectory(10.25), 10.4), 'stale_reference')
        self.assertEqual(self.adapter.status(10.4)['source_stamp'], 10.3)

    def test_received_prefix_waits_for_quiet_and_rejects_expiry(self):
        self.ready()
        self.assertIsNone(self.plan(10.12)[0])
        self.assertEqual(self.adapter.feed_trajectory(trajectory(ident=2, x=1.4), 10.16), 'append')
        self.assertIsNone(self.plan(10.17)[0])
        self.assertIsNotNone(self.plan(10.21)[0])
        self.assertEqual(self.adapter.status(10.21)['native_prefix_segments'], 2)
        self.assertIsNone(self.plan(14.11)[0])
        self.assertEqual(self.adapter.status(14.11)['reference_status'], 'native_reference_expired')

    def test_time_varying_native_arm_is_sampled_at_original_phase_not_endpoint(self):
        self.ready()
        self.assertEqual(self.adapter.feed_trajectory(trajectory(10.2, moving_arm=True), 10.21), 'replace')
        self.assertIsNotNone(self.plan(10.3)[0])
        arm = self.navigation.calls[-1][1]
        t = .1+.1*np.arange(1, 9)
        np.testing.assert_allclose(arm[:, 0], .01*t**7+.02*t**2+.03*t)
        np.testing.assert_allclose(arm[:, 1:], np.tile(np.arange(1, 6)*.1, (8, 1)))
        self.assertGreater(np.ptp(arm[:, 0]), .03)
        info = self.adapter.status(10.3)
        self.assertEqual(info['source_stamp'], 10.2)
        np.testing.assert_allclose(info['native_arm_sample_times'], 10.3+.1*np.arange(1, 9))
        self.assertEqual(info['reference_hash'], self.adapter.status(10.4)['reference_hash'])

    def test_arm_sampling_crosses_appended_pieces_and_holds_only_beyond_prefix_end(self):
        self.ready()
        msg = trajectory(ident=2, x=1.4, moving_arm=True)
        self.assertEqual(self.adapter.feed_trajectory(msg, 10.16), 'append')
        self.assertIsNotNone(self.plan(11.95)[0])
        t = np.clip(11.95-10.1+.1*np.arange(1, 9)-2., 0., 2.)
        np.testing.assert_allclose(self.navigation.calls[-1][1][:, 0], .01*t**7+.02*t**2+.03*t)
        self.assertIsNotNone(self.plan(13.95)[0])
        t = np.clip(13.95-10.1+.1*np.arange(1, 9)-2., 0., 2.)
        np.testing.assert_allclose(self.navigation.calls[-1][1][:, 0], .01*t**7+.02*t**2+.03*t)
        self.assertIsNone(self.plan(14.11)[0])

    def test_out_of_order_prefix_and_abort_clear_previously_usable_reference(self):
        self.ready()
        self.assertEqual(self.adapter.feed_trajectory(trajectory(ident=3, x=1.4), 10.16), 'invalid')
        self.assertIsNone(self.plan(10.2)[0])
        self.adapter.feed_trajectory(trajectory(10.3), 10.31)
        cancelled = trajectory(); cancelled.action = 2
        self.assertEqual(self.adapter.feed_trajectory(cancelled, 10.4), 'cancel')
        self.assertIsNone(self.plan(10.5)[0])
        self.assertEqual(self.adapter.feed_trajectory(trajectory(10.3), 10.5), 'stale_reference')

    def test_goal_change_during_solve_discards_old_result(self):
        self.ready()
        self.navigation.during_plan = lambda: self.adapter.feed_goal(goal(10.2), 10.2)
        result, info = self.plan()
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'reference_changed_during_plan')

    def test_malformed_goal_cannot_reactivate_previous_goal_via_late_trajectory(self):
        self.ready()
        invalid = goal(10.2); invalid.header.frame_id = 'bad_frame'
        self.assertEqual(self.adapter.feed_goal(invalid, 10.21), 'invalid')
        self.assertEqual(self.adapter.feed_trajectory(trajectory(10.3), 10.31), 'ignored_without_valid_goal')
        self.assertIsNone(self.plan(10.4)[0])

    def test_reference_expiring_during_solve_is_not_delivered(self):
        self.ready()
        with patch('native_reference_adapter.time.monotonic', side_effect=[100., 103.]):
            result, info = self.plan()
        self.assertIsNone(result)
        self.assertEqual(info['reference_status'], 'native_reference_expired_during_plan')


class NavigationArmReferenceTests(unittest.TestCase):
    def fixture(self):
        model = NS(body_spheres_batch=lambda q, pose: np.zeros((len(q), 1, 4)),
                   configuration_gaps_batch=lambda q: np.ones(len(q)))
        limits = dict(track=.5, vmax=.5, wmax=1., wheel_v=1., accel=.5,
                      angular_accel=1., wheel_a=1.)
        kernel = WholeBodyMPPI(model, limits, (-np.ones(6), np.ones(6),
            np.ones(6)*.4, np.ones(6)*.8), np.zeros((1, 6)), horizon=8, step=.1)
        state = np.zeros(17)
        reference = np.array([[0., 0., 0., .1], [.3, 0., 0., .1], [1., 0., 0., .1]])
        snapshot = NS(query=lambda spheres, times: (np.ones(len(spheres)), np.zeros(len(spheres))))
        return kernel, state, reference, snapshot

    def test_constant_six_vector_and_horizon_matrix_have_identical_rollouts_cost_and_acceptance(self):
        kernel, state, reference, snapshot = self.fixture()
        q = np.arange(6)*.02
        matrix = np.tile(q, (kernel.H, 1))
        u, labels = kernel.proposals(state, reference, q)
        uh, labelsh = kernel.proposals(state, reference, matrix)
        self.assertEqual(labels, labelsh)
        np.testing.assert_array_equal(u, uh)
        scalar_result = kernel.evaluate(state, u, reference, q, snapshot, 10.)
        matrix_result = kernel.evaluate(state, uh, reference, matrix, snapshot, 10.)
        for scalar, expanded in zip(scalar_result, matrix_result):
            np.testing.assert_array_equal(scalar, expanded)

    def test_varying_target_changes_later_proposal_steps_without_using_final_target_early(self):
        kernel, state, reference, _ = self.fixture()
        matrix = np.zeros((kernel.H, 6)); matrix[4:, 0] = .2
        controls, _ = kernel.proposals(state, reference, matrix, [(1., 0.)])
        np.testing.assert_array_equal(controls[0, :4, 2:], np.zeros((4, 6)))
        self.assertGreater(controls[0, 4, 2], .3)


if __name__ == '__main__':
    unittest.main()

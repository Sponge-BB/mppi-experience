"""Native reverse direction must survive phase slowdown, pause and zero speed."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'/'mppi'))
from active_arm_mppi import WholeBodyMPPI
from native_reference_adapter import NativeReferenceAdapter
from test_mppi_native_phase import goal, header, progress
from test_mppi_navigation_priority import TwoSphereRobot


class Clear:
    def query(self, spheres, times):
        return np.full(len(spheres), 10.), np.zeros(len(spheres))


def trajectory(gear):
    coefficients = np.zeros((8, 8))
    coefficients[0, -2] = .2*gear
    piece = NS(num_dim=8, num_order=7, duration=2., data=coefficients.ravel(order='F').tolist())
    msg = NS(header=header(10.1), action=1, trajectory_id=1, singul=gear, trajectory=[piece])
    msg.serialize = lambda stream: stream.write(coefficients.tobytes()+bytes([gear+1]))
    return msg


class NativeReverseGearTests(unittest.TestCase):
    def setUp(self):
        self.kernel = WholeBodyMPPI(TwoSphereRobot(),
            dict(vmax=.20, wmax=.35, accel=.25, angular_accel=.8,
                 track=.37, wheel_v=.2296, wheel_a=.2975),
            (np.full(6, -1.), np.ones(6), np.full(6, .15), np.full(6, .35)),
            [], horizon=7, step=.1, random_samples=0, seed=11)
        self.state = np.zeros(17)

    def ready(self, gear, scale):
        adapter = NativeReferenceAdapter(self.kernel, self.kernel, clock=lambda: 20.)
        target = goal(); target.pose.position.x = .4*gear
        self.assertEqual(adapter.feed_goal(target, 10.01), 'accepted')
        self.assertEqual(adapter.feed_trajectory(trajectory(gear), 10.11), 'replace')
        self.assertEqual(adapter.feed_progress(progress(20., phase=.4, scale=scale), 20.), 'accepted')
        self.state[0] = .04*gear  # Four centimetres behind the sampled phase.
        return adapter

    def test_reverse_native_path_does_not_turn_around_when_phase_slows_or_pauses(self):
        for scale in (1., .1, .05, .02, 0.):
            with self.subTest(scale=scale):
                adapter = self.ready(-1, scale)
                answer, info = adapter.plan(self.state, None, None, Clear(), 20.)
                self.assertIsNotNone(answer)
                controls, states = answer
                self.assertEqual(info['native_gear'], -1)
                self.assertEqual(info['mode'], 'nominal')
                self.assertLess(controls[0, 0], 0.)
                np.testing.assert_allclose(controls[:, 1], 0., atol=1e-12)
                self.assertLess(states[-1, 0], self.state[0])
                if scale == 0.:
                    path = adapter.snapshot_path(0., 20.)
                    np.testing.assert_array_equal(path[0], path[-1])
                    np.testing.assert_array_equal(path[:, 3], 0.)

    def test_forward_native_path_retains_direction_at_all_scales(self):
        for scale in (1., .1, .02, 0.):
            with self.subTest(scale=scale):
                adapter = self.ready(1, scale)
                (controls, states), info = adapter.plan(self.state, None, None, Clear(), 20.)
                self.assertEqual(info['native_gear'], 1)
                self.assertGreater(controls[0, 0], 0.)
                np.testing.assert_allclose(controls[:, 1], 0., atol=1e-12)
                self.assertGreater(states[-1, 0], self.state[0])

    def test_paused_pose_already_reached_does_not_resume_the_remaining_curve(self):
        adapter = self.ready(-1, 0.)
        self.state[0] = -.08
        (controls, states), _ = adapter.plan(self.state, None, None, Clear(), 20.)
        np.testing.assert_allclose(controls, 0., atol=1e-12)
        np.testing.assert_allclose(states[:, :3], np.tile(self.state[:3], (self.kernel.H+1, 1)), atol=1e-12)

    def test_explicit_direction_is_independent_of_tiny_speed_and_both_zero_signs(self):
        for speed in (-.004, -1e-12, -0., 0., 1e-12):
            with self.subTest(speed=speed):
                path = np.c_[np.linspace(0., -.4, 21), np.zeros((21, 2)), np.full(21, speed)]
                guide = self.kernel.guide(self.state, path, reference_gear=-1)
                controls, _ = self.kernel.proposals(self.state, path, np.zeros(6),
                                                    [(1., 0.)], reference_gear=-1)
                self.assertLess(guide[0], 0.)
                self.assertAlmostEqual(guide[1], 0.)
                self.assertLess(controls[0, 0, 0], 0.)
                np.testing.assert_allclose(controls[0, :, 1], 0., atol=1e-12)

    def test_whole_body_fallback_keeps_native_direction(self):
        adapter = self.ready(-1, .02)
        obstacle = Clear()
        obstacle.query = lambda spheres, times: (np.full(len(spheres), -1.), np.zeros(len(spheres)))
        generated = []
        original = self.kernel.proposals
        def capture(*args, **kwargs):
            result = original(*args, **kwargs)
            generated.append(result[0])
            return result
        with patch.object(self.kernel, 'proposals', side_effect=capture):
            answer, info = adapter.plan(self.state, None, None, obstacle, 20.)
        self.assertIsNone(answer, 'Explicit gear must not bypass collision rejection')
        self.assertEqual(info['mode'], 'whole_body_fallback')
        self.assertEqual(len(generated), 2)
        for controls in generated:
            self.assertLess(controls[0, 0, 0], 0.)
            self.assertAlmostEqual(controls[0, 0, 1], 0.)

    def test_unscaled_legacy_direction_and_control_remain_identical(self):
        for gear in (-1, 1):
            path = np.c_[np.linspace(0., .4*gear, 21), np.zeros((21, 2)), np.full(21, .2*gear)]
            old, _ = self.kernel.plan(self.state, path, np.zeros(6), Clear(), 20.)
            new, _ = self.kernel.plan(self.state, path, np.zeros(6), Clear(), 20., gear)
            np.testing.assert_array_equal(old[0], new[0])
            np.testing.assert_array_equal(old[1], new[1])


if __name__ == '__main__':
    unittest.main()

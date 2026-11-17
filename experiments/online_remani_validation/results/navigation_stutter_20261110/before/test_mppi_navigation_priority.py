"""Offline allocation regressions, not evidence of real-robot avoidance.

Small analytic geometry isolates arm/base allocation while exercising the real
MPPI proposals, rollout dynamics, collision evaluation and weighted selection.
No ROS, hardware, recorded maps or mocked planner decisions are involved.
"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'/'mppi'))
from active_arm_mppi import WholeBodyMPPI


class TwoSphereRobot:
    """A base sphere and an arm sphere moved laterally by joint zero."""
    def body_spheres_batch(self, q, base):
        result = np.zeros((len(q), 2, 4))
        result[:, :, :2] = base[:, None, :2]
        result[:, 1, 0] += .2*np.cos(base[:, 2])-q[:, 0]*np.sin(base[:, 2])
        result[:, 1, 1] += .2*np.sin(base[:, 2])+q[:, 0]*np.cos(base[:, 2])
        result[:, 1, 2] = .5
        result[:, :, 3] = .04
        return result

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class MovingSphere:
    """Constant-velocity obstacle, initially clear and crossing at t=0.6 s."""
    def __init__(self, x, z):
        self.initial = np.array([x, .3, z])

    def query(self, spheres, times):
        elapsed = np.asarray(times)-100.
        center = self.initial+elapsed[:, None]*np.array([0., -.5, 0.])
        distance = np.linalg.norm(spheres[:, :, :3]-center[:, None, :], axis=-1)
        gap = distance-spheres[:, :, 3]-.04
        soft = 180.*np.sum(np.maximum(0., .08-gap)**3, axis=-1)
        return gap.min(axis=-1), soft


class NavigationPriorityTests(unittest.TestCase):
    def setUp(self):
        limits = dict(track=.4, vmax=.5, wmax=1., wheel_v=.8,
                      accel=1., angular_accel=2., wheel_a=1.5)
        self.planner = WholeBodyMPPI(
            TwoSphereRobot(), limits,
            (np.full(6, -1.), np.ones(6), np.full(6, .8), np.full(6, 3.)),
            [[-.6, 0., 0., 0., 0., 0.]], horizon=12, step=.1,
            random_samples=8, seed=11)
        self.state = np.zeros(17)
        self.state[3] = .16
        self.reference = np.c_[np.linspace(0., 1., 51),
                               np.zeros((51, 2)), np.full(51, .16)]
        self.arm_reference = np.zeros(6)

    def first_stage(self, obstacle):
        controls, _ = self.planner.proposals(
            self.state, self.reference, self.arm_reference, [(1., 0.)])
        result = self.planner.evaluate(
            self.state, controls, self.reference, self.arm_reference, obstacle, 100.)
        initial_gap, _ = obstacle.query(self.planner.spheres(self.state[None]),
                                        np.array([100.]))
        self.assertGreater(initial_gap[0], .12)
        self.assertGreater(controls[0, :, 0].min(), .1)  # Moving reference.
        return controls, result

    def plan(self, obstacle):
        answer, info = self.planner.plan(
            self.state, self.reference, self.arm_reference, obstacle, 100.)
        self.assertIsNotNone(answer)
        controls, states = answer
        _, valid, _, checked = self.planner.evaluate(
            self.state, controls[None], self.reference, self.arm_reference,
            obstacle, 100.)
        self.assertTrue(valid[0], 'The selected weighted controls must remain feasible')
        np.testing.assert_array_equal(states, checked[0])
        return controls, states, info

    def test_arm_avoidance_preserves_complete_nominal_base_sequence(self):
        obstacle = MovingSphere(.3, .5)
        nominal, (_, valid, _, predicted) = self.first_stage(obstacle)
        self.assertFalse(valid[0], 'Nominal arm must encounter the future obstacle')
        self.assertTrue(valid[1:].any(), 'An arm-only candidate must be feasible')

        controls, states, info = self.plan(obstacle)
        self.assertEqual(info['mode'], 'arm_only')
        self.assertTrue(info['intervention'])
        self.assertGreater(np.max(np.abs(controls[:, 2:])), .1)
        self.assertLess(states[-1, 5], -.1)
        np.testing.assert_array_equal(controls[:, :2], nominal[0, :, :2])
        np.testing.assert_array_equal(states[:, :5], predicted[0, :, :5])

    def test_base_fallback_requires_all_arm_candidates_to_be_infeasible(self):
        obstacle = MovingSphere(.12, 0.)
        nominal, (_, valid, _, predicted) = self.first_stage(obstacle)
        self.assertFalse(valid.any(), 'Arm motion cannot move the base sphere')

        controls, states, info = self.plan(obstacle)
        self.assertEqual(info['mode'], 'whole_body_fallback')
        self.assertTrue(info['intervention'])
        self.assertGreater(np.max(np.abs(controls[:, :2]-nominal[0, :, :2])), .02)
        self.assertGreater(np.max(np.abs(states[:, :3]-predicted[0, :, :3])), .01)

    def test_clear_environment_keeps_nominal_motion(self):
        obstacle = MovingSphere(10., .5)
        nominal, (_, valid, _, predicted) = self.first_stage(obstacle)
        self.assertTrue(valid.all())

        controls, states, info = self.plan(obstacle)
        self.assertEqual(info['mode'], 'nominal')
        self.assertFalse(info['intervention'])
        np.testing.assert_array_equal(controls, nominal[0])
        np.testing.assert_array_equal(states, predicted[0])
        np.testing.assert_array_equal(controls[:, 2:], 0.)

    def test_noise_can_find_a_safe_arm_action_when_posture_library_fails(self):
        # A narrow grazing encounter needs a small joint correction absent
        # from the posture library. Slow base motion cannot clear it in time.
        self.planner.postures = np.empty((0, 6))
        self.planner.limits['vmax'] = self.planner.limits['wmax'] = .0001
        self.state[3] = 0.
        obstacle = MovingSphere(.2, .57999)
        seeds, _ = self.planner.proposals(self.state, self.reference, self.arm_reference)
        _, valid, _, _ = self.planner.evaluate(self.state, seeds, self.reference,
                                               self.arm_reference, obstacle, 100.)
        self.assertFalse(valid.any())
        _, _, info = self.plan(obstacle)  # Independently rechecks selected controls.
        self.assertFalse(any(info['structured_valid']))
        self.assertTrue(any(info['random_valid']))

    def test_noise_does_not_override_an_initial_collision(self):
        obstacle = MovingSphere(0., 0.)
        obstacle.initial[:] = 0.
        answer, info = self.planner.plan(self.state, self.reference,
                                        self.arm_reference, obstacle, 100.)
        self.assertIsNone(answer)
        self.assertEqual(info['feasible'], 0)
        self.assertFalse(any(info['random_valid']))

    def test_slowdown_is_distinct_when_reference_speed_exceeds_hardware_cap(self):
        self.planner.limits['vmax'] = .05
        self.state[3] = .05
        self.reference[:, 3] = .08
        controls, _ = self.planner.proposals(self.state, self.reference, self.arm_reference)
        self.assertAlmostEqual(controls[0, 0, 0], .05)
        self.assertAlmostEqual(controls[1, 0, 0], .0325)
        a = self.planner.advance(self.state, controls[0, 0])
        b = self.planner.advance(self.state, controls[1, 0])
        self.assertLess(b[0], a[0])

    def test_structured_only_rejection_does_not_require_random_samples(self):
        self.planner.random_samples = 0
        obstacle = MovingSphere(0., 0.)
        obstacle.initial[:] = 0.
        answer, info = self.planner.plan(self.state, self.reference,
                                        self.arm_reference, obstacle, 100.)
        self.assertIsNone(answer)
        self.assertEqual(info['random_valid'], [])


if __name__ == '__main__':
    unittest.main()

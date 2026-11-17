"""Offline regressions for nominal-first acceptance; no ROS or hardware.

Run against an isolated copy before deployment:
 MPPI_TEST_TARGET=/absolute/path/after.py /usr/bin/python3 THIS_FILE
Absent that variable, test the deployed Python source. Small analytic geometry
comes from the existing allocation tests; actual rollout/evaluate/plan run.
"""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

import copy
import importlib.util
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents
            if (p/'src/mm_remani/scripts/mppi/active_arm_mppi.py').is_file())
SOURCE = ROOT/'src/mm_remani/scripts/mppi/active_arm_mppi.py'
sys.path[:0] = [str(SOURCE.parent), str(ROOT/'src/mm_remani/test')]
import active_arm_mppi as deployed
from test_mppi_navigation_priority import TwoSphereRobot, MovingSphere

TARGET = Path(os.environ.get('MPPI_TEST_TARGET', str(SOURCE))).resolve()
if TARGET == SOURCE.resolve():
    module = deployed
else:
    spec = importlib.util.spec_from_file_location('tested_nominal_shortcut', TARGET)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    # Relocating a Python copy must not silently change optional native kernels.
    module._load_position_servo_library = deployed._load_position_servo_library
    module._load_legacy_velocity_library = deployed._load_legacy_velocity_library


class TimedScene:
    """Controlled external gap: permits exact sample/tail coverage assertions."""
    def __init__(self, collision_at=None, gap=1.):
        self.collision_at, self.gap = collision_at, gap
        self.calls = []

    def query(self, spheres, times):
        times = np.asarray(times)
        self.calls.append(times.copy())
        gap = np.full(len(times), self.gap)
        if self.collision_at is not None:
            gap[np.isclose(times, self.collision_at, rtol=0., atol=1e-8)] = -.02
        return gap, np.zeros(len(times))


class NominalFirstTests(unittest.TestCase):
    def fixture(self, servo=False, *, horizon=12, random_samples=4):
        limits = dict(track=.4, vmax=.5, wmax=1., wheel_v=.8,
                      accel=1., angular_accel=2., wheel_a=1.5)
        kernel = module.WholeBodyMPPI(TwoSphereRobot(), limits,
            (np.full(6, -1.), np.ones(6), np.full(6, .8), np.full(6, 3.)),
            [[-.6, 0., 0., 0., 0., 0.]], horizon=horizon, step=.1,
            random_samples=random_samples, seed=11, arm_position_servo=servo)
        state = np.zeros(17)
        state[3] = .16
        reference = np.c_[np.linspace(0., 1., 51),
                          np.zeros((51, 2)), np.full(51, .16)]
        return kernel, state, reference, np.zeros(6)

    def assert_same_evaluation(self, full, split):
        # These controlled inputs make both the rollout and cost arithmetic
        # exactly equal. No mocked evaluate result or copied plan is involved.
        for actual, expected in zip(split, full):
            np.testing.assert_array_equal(actual, expected)

    def test_actual_evaluate_split_preserves_every_result_and_diagnostic(self):
        for servo in (False, True):
            for tracked in (False, True):
                with self.subTest(servo=servo, tracked=tracked):
                    kernel, state, reference, arm = self.fixture(servo)
                    state[:] = 0.
                    controls = np.zeros((3, kernel.H, 8))
                    controls[1, :, 0] = .4
                    controls[2, :, 2] = .7
                    tracking = None
                    if tracked:
                        samples = np.zeros((kernel.H+1, 6))
                        samples[:, 0] = np.arange(kernel.H+1)*kernel.dt*.16
                        samples[:, 3] = samples[:, 5] = .16
                        tracking = dict(base_samples=samples,
                                        resume_distance=.05, yaw_hold_speed=.04)
                    # Candidate0 has already stopped before this time. Other
                    # candidates have longer tails: their batch padding must
                    # never create a spurious collision for candidate0.
                    scene = TimedScene(100.+kernel.H*kernel.dt+.10)
                    args = (state, controls, reference, arm, scene, 100.)
                    full = kernel.evaluate(*args, tracking=tracking)
                    external = kernel.last_external_gaps.copy()
                    diagnostic = copy.deepcopy(kernel.last_evaluation_diagnostics)
                    parts, externals, diagnostics = [], [], []
                    for subset in (controls[:1], controls[1:]):
                        parts.append(kernel.evaluate(state, subset, reference, arm,
                                                     scene, 100., tracking=tracking))
                        externals.append(kernel.last_external_gaps.copy())
                        diagnostics.append(copy.deepcopy(kernel.last_evaluation_diagnostics))
                    split = tuple(np.concatenate([part[i] for part in parts], axis=0)
                                  for i in range(4))
                    self.assert_same_evaluation(full, split)
                    np.testing.assert_array_equal(np.concatenate(externals), external)
                    self.assertEqual({k: diagnostics[0][k]+diagnostics[1][k]
                                      for k in diagnostic}, diagnostic)
                    self.assertTrue(full[1][0])
                    self.assertAlmostEqual(diagnostic['braking_tail_s'][0], kernel.dt/2)
                    self.assertGreater(max(diagnostic['braking_tail_s'][1:]),
                                       diagnostic['braking_tail_s'][0])
                    self.assertIsNone(diagnostic['first_external_violation_s'][0])
                    self.assertTrue(any(t is not None for t in
                                        diagnostic['first_external_violation_s'][1:]))

    def test_clear_shortcut_checks_one_complete_nominal_and_keeps_rng(self):
        for servo in (False, True):
            with self.subTest(servo=servo):
                kernel, state, reference, arm = self.fixture(servo)
                controls, _ = kernel.proposals(state, reference, arm, [(1., 0.)])
                expected = kernel.evaluate(state, controls[:1], reference, arm,
                                           TimedScene(), 100.)
                self.assertTrue(expected[1][0])
                rng_before = copy.deepcopy(kernel.rng.get_state())
                kernel.capture_rollouts = True
                scene = TimedScene()
                with patch.object(kernel, 'evaluate', wraps=kernel.evaluate) as evaluate:
                    result, info = kernel.plan(state, reference, arm, scene, 100.)
                self.assertEqual([len(c.args[1]) for c in evaluate.call_args_list], [1])
                self.assertEqual((info['mode'], info['intervention']), ('nominal', False))
                self.assertEqual((info['feasible'], info['candidates']), (1, 1))
                self.assertIsNotNone(result)
                np.testing.assert_array_equal(result[0], controls[0])
                np.testing.assert_array_equal(result[1], expected[3][0])
                np.testing.assert_array_equal(kernel.previous, controls[0])
                np.testing.assert_array_equal(kernel.visual_rollouts[0], expected[3][:, :, :3])
                np.testing.assert_array_equal(kernel.visual_rollouts[1], expected[1])
                self.assertGreater(scene.calls[0].max(), 100.+kernel.H*kernel.dt)
                self.assertEqual(len(scene.calls[0]),
                    int(round(kernel.last_evaluation_diagnostics['checked_duration_s'][0]
                              /(kernel.dt/2)))+1)
                after = kernel.rng.get_state()
                self.assertEqual(rng_before[0], after[0])
                np.testing.assert_array_equal(rng_before[1], after[1])
                self.assertEqual(rng_before[2:], after[2:])

    def test_terminal_and_braking_tail_collisions_activate_remaining_candidates(self):
        for servo in (False, True):
            for phase in ('terminal', 'braking_tail'):
                with self.subTest(servo=servo, phase=phase):
                    kernel, state, reference, arm = self.fixture(servo)
                    controls, _ = kernel.proposals(state, reference, arm, [(1., 0.)])
                    kernel.evaluate(state, controls[:1], reference, arm, TimedScene(), 100.)
                    tail = kernel.last_evaluation_diagnostics['braking_tail_s'][0]
                    self.assertGreater(tail, kernel.dt/2)
                    when = 100.+kernel.H*kernel.dt
                    if phase == 'braking_tail':
                        when += kernel.dt/2
                    scene = TimedScene(when)
                    nominal = kernel.evaluate(state, controls[:1], reference, arm, scene, 100.)
                    self.assertFalse(nominal[1][0])
                    self.assertAlmostEqual(
                        kernel.last_evaluation_diagnostics['first_external_violation_s'][0],
                        when-100.)
                    with patch.object(kernel, 'evaluate', wraps=kernel.evaluate) as evaluate:
                        _, info = kernel.plan(state, reference, arm, scene, 100.)
                    sizes = [len(c.args[1]) for c in evaluate.call_args_list]
                    self.assertEqual(sizes[:2], [1, len(controls)-1])
                    # Existing no-feasible returns do not carry intervention.
                    # Actual extra evaluations and a non-nominal mode prove
                    # that the shortcut did not accept this unsafe nominal.
                    self.assertNotEqual(info['mode'], 'nominal')
                    self.assertTrue(all(n > 0 for n in sizes))

    def test_intervention_merges_structured_diagnostics_in_original_order(self):
        for servo in (False, True):
            with self.subTest(servo=servo):
                kernel, state, reference, arm = self.fixture(servo)
                scene = MovingSphere(.28, .57) if servo else MovingSphere(.3, .5)
                controls, labels = kernel.proposals(state, reference, arm, [(1., 0.)])
                full = kernel.evaluate(state, controls, reference, arm, scene, 100.)
                self.assertFalse(full[1][0])
                self.assertTrue(full[1][1:].any(), 'An arm-only seed must be feasible')
                external = kernel.last_external_gaps.copy()
                diagnostic = copy.deepcopy(kernel.last_evaluation_diagnostics)
                with patch.object(kernel, 'evaluate', wraps=kernel.evaluate) as evaluate:
                    _, info = kernel.plan(state, reference, arm, scene, 100.)
                self.assertEqual(len(evaluate.call_args_list[0].args[1]), 1)
                self.assertEqual(info['mode'], 'arm_only')
                self.assertEqual(info['structured_labels'], [list(x) for x in labels])
                np.testing.assert_array_equal(info['structured_costs'], full[0])
                np.testing.assert_array_equal(info['structured_valid'], full[1])
                np.testing.assert_array_equal(info['structured_external_gaps'], external)
                for key, value in diagnostic.items():
                    self.assertEqual(info['structured_'+key], value)

    def test_single_candidate_intervention_never_evaluates_empty_remainder(self):
        for servo in (False, True):
            with self.subTest(servo=servo):
                kernel, state, reference, arm = self.fixture(servo, random_samples=0)
                kernel.postures = np.empty((0, 6))
                controls, _ = kernel.proposals(state, reference, arm, [(1., 0.)])
                self.assertEqual(len(controls), 1)
                with patch.object(kernel, 'evaluate', wraps=kernel.evaluate) as evaluate:
                    result, info = kernel.plan(state, reference, arm, TimedScene(gap=.05), 100.)
                self.assertIsNotNone(result)
                self.assertTrue(info['intervention'])
                self.assertTrue(all(len(c.args[1]) > 0 for c in evaluate.call_args_list))
                self.assertEqual(info['structured_valid'], [True])


if __name__ == '__main__':
    unittest.main(verbosity=2)

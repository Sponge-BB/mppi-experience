"""Offline navigation response checks; no ROS master or hardware calibration.

The production simulator's independent 5 ms Plant is loaded as pure Python
definitions. Small geometry keeps the tests focused on response and acceptance.
"""
import ast
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'/'mppi'))
from active_arm_mppi import WholeBodyMPPI
from native_reference_adapter import NativeReferenceAdapter
from standby_whole_body_mppi import StandbyWholeBodyMPPI


def load_plant():
    path = PACKAGE/'scripts'/'standby_simulator.py'
    tree = ast.parse(path.read_text())
    definitions = [node for node in tree.body if
        (isinstance(node, ast.ClassDef) and node.name == 'Plant') or
        (isinstance(node, ast.Assign) and any(isinstance(name, ast.Name) and
            name.id in ('Q_START', 'STEP') for name in node.targets))]
    namespace = {'np': np, 'math': math}
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['Plant'], namespace['Q_START']


Plant, Q_START = load_plant()
METRICS = {}


class SmallModel:
    lower, upper = np.full(6, -4.), np.full(6, 4.)
    velocity_limits, acceleration_limits = np.ones(6), np.full(6, 2.)

    def check(self, q):
        return {'valid': bool(np.all((q >= self.lower) & (q <= self.upper)))}

    def body_spheres_batch(self, q, base):
        spheres = np.zeros((len(q), 2, 4))
        spheres[:, 0, :3], spheres[:, 1, :3] = base, q[:, :3]
        spheres[:, :, 3] = .01
        return spheres

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class Environment:
    def __init__(self, arrival=None):
        self.arrival = arrival

    def query(self, spheres, times):
        self.spheres, self.times = np.array(spheres), np.array(times)
        gap = np.ones(len(times)) if self.arrival is None else self.arrival-(self.times-100.)
        return gap, np.zeros_like(gap)


class NavigationServoTests(unittest.TestCase):
    def setUp(self):
        self.model = SmallModel()
        self.limits = dict(track=.37, vmax=.05, wmax=.15, wheel_v=.2296,
                           accel=.10, angular_accel=.30, wheel_a=.2975)
        self.state = np.r_[np.zeros(5), Q_START.copy(), np.zeros(6)]
        self.reference = np.c_[np.linspace(0., 1., 21), np.zeros((21, 2)), np.full(21, .05)]

    def kernel(self, servo=True, horizon=8, acceleration=.35, **kwargs):
        return WholeBodyMPPI(self.model, self.limits,
            (self.model.lower, self.model.upper, np.full(6, .15), np.full(6, acceleration)),
            [Q_START+.02], horizon=horizon, step=.1, arm_position_servo=servo, **kwargs)

    def evaluate(self, kernel, controls, environment):
        return kernel.evaluate(self.state, controls, self.reference, Q_START, environment, 100.)

    def test_one_target_per_period_and_zero_velocity_brake_tail(self):
        kernel, environment = self.kernel(), Environment()
        self.state[3:5] = [.03, .07]
        controls = np.zeros((1, kernel.H, 8)); controls[0, :, :2] = [.05, .12]
        controls[0, :, 3] = .15
        _, valid, _, predicted = self.evaluate(kernel, controls, environment)
        self.assertTrue(valid[0])
        fixed = self.state.copy(); regenerated = self.state.copy()
        full, expected = [fixed.copy()], [fixed.copy()]
        for command in controls[0]:
            target = kernel.position_target(fixed, command)
            for _ in range(2):
                fixed = kernel.advance(fixed, command, .05, arm_target=target)
                regenerated = kernel.advance(regenerated, command, .05)
                full.append(fixed.copy())
            expected.append(fixed.copy())
        self.assertGreater(np.max(np.abs(fixed[5:11]-regenerated[5:11])), 1e-5,
                           'This fixture must expose half-step target regeneration')
        for _ in range(40):
            fixed = kernel.advance_position_brake(fixed, .05)
            full.append(fixed.copy())
            if max(np.max(np.abs(fixed[3:5])), np.max(np.abs(fixed[11:17]))) < .003:
                break
        np.testing.assert_allclose(predicted[0], expected, rtol=0, atol=1e-12)
        np.testing.assert_allclose(environment.spheres, kernel.spheres(np.array(full)), rtol=0, atol=1e-12)
        np.testing.assert_allclose(environment.times, 100.+.05*np.arange(len(full)), rtol=0, atol=1e-12)

    def test_two_second_navigation_against_independent_5ms_plant(self):
        kernel = self.kernel(horizon=20)
        self.state[3:5] = [.03, .07]
        controls = np.zeros((1, kernel.H, 8)); controls[0, :, :2] = [.05, .12]
        controls[0, :, 3] = .15
        _, valid, _, predicted = self.evaluate(kernel, controls, Environment())
        self.assertTrue(valid[0])
        plant = Plant(self.model, self.limits, kernel.vmax, kernel.amax)
        plant.state = self.state.copy()
        actual = [plant.state.copy()]
        for command in controls[0]:
            target = kernel.position_target(plant.state, command)
            for _ in range(20):
                plant.step(command[:2], target, .005)
            actual.append(plant.state.copy())
        error = predicted[0]-np.array(actual)
        measured = dict(max_q_error_rad=float(np.max(np.abs(error[:, 5:11]))),
                        max_base_xy_error_m=float(np.max(np.linalg.norm(error[:, :2], axis=1))),
                        max_yaw_error_rad=float(np.max(np.abs(error[:, 2]))))
        budgets = dict(max_q_error_rad=.002, max_base_xy_error_m=.001, max_yaw_error_rad=.002)
        for name, budget in budgets.items():
            self.assertLess(measured[name], budget, name)
        self.assertGreater(actual[-1][0], .08)
        self.assertGreater(actual[-1][2], .15)
        METRICS['navigation_2s_5ms_plant'] = dict(measured=measured, regression_budgets=budgets,
            scope='Numerical approximation for this fixture, not physical robot calibration')

    def test_each_braking_tail_is_independent_of_batch_partners(self):
        for servo in (False, True):
            with self.subTest(position_servo=servo):
                kernel = self.kernel(servo=servo)
                controls = np.zeros((2, kernel.H, 8)); controls[1, :, 3] = .15
                # Resting candidate ends at .85 s; the moving one must keep
                # checking its own braking after the future obstacle at 1 s.
                environment = Environment(arrival=1.)
                batch = self.evaluate(kernel, controls, environment)
                external = kernel.last_external_gaps.copy()
                self.assertTrue(batch[1][0]); self.assertFalse(batch[1][1])
                for index in range(2):
                    single = self.evaluate(kernel, controls[index:index+1], environment)
                    for actual, expected in zip(single, batch):
                        np.testing.assert_allclose(actual[0], expected[index], rtol=0, atol=1e-12)
                    self.assertAlmostEqual(kernel.last_external_gaps[0], external[index], places=12)
                reverse = self.evaluate(kernel, controls[::-1], environment)
                for actual, expected in zip(reverse, batch):
                    np.testing.assert_allclose(actual[::-1], expected, rtol=0, atol=1e-12)

    def test_unfinished_braking_is_rejected_in_both_response_modes(self):
        self.state[12] = .15
        for servo in (False, True):
            with self.subTest(position_servo=servo):
                kernel = self.kernel(servo=servo, acceleration=.0001)
                controls = np.zeros((1, kernel.H, 8))
                _, valid, _, _ = self.evaluate(kernel, controls, Environment())
                self.assertFalse(valid[0], 'Clear geometry cannot authorize an unfinished brake')

    def test_adapter_inherits_model_parameters_and_seed_without_hardware_access(self):
        for servo in (False, True):
            with self.subTest(position_servo=servo):
                standby = StandbyWholeBodyMPPI(self.model, self.limits,
                    (self.model.lower, self.model.upper, np.full(6, .15), np.full(6, .35)),
                    [Q_START+.02], enabled=False, seed=37, horizon=6, step=.08,
                    random_samples=3, arm_position_servo=servo)
                standby.rng.normal(size=7)  # Inherit original seed, not current RNG progress.
                parameter_read = Mock(side_effect=AssertionError('unexpected ROS/hardware parameter read'))
                with patch.dict(sys.modules, {'rospy': SimpleNamespace(get_param=parameter_read)}), \
                     patch('builtins.open', side_effect=AssertionError('unexpected configuration/device open')):
                    adapter = NativeReferenceAdapter(standby)
                navigation = adapter.navigation
                parameter_read.assert_not_called()
                self.assertIs(navigation.model, standby.model)
                self.assertEqual(navigation.limits, standby.limits)
                for name in ('H', 'dt', 'random_samples', 'enabled', 'arm_position_servo', 'seed'):
                    self.assertEqual(getattr(navigation, name), getattr(standby, name), name)
                for name in ('qmin', 'qmax', 'vmax', 'amax', 'postures'):
                    np.testing.assert_array_equal(getattr(navigation, name), getattr(standby, name))
                self.assertIsNot(navigation.rng, standby.rng)
                np.testing.assert_array_equal(navigation.rng.normal(size=12), np.random.RandomState(37).normal(size=12))
                self.assertFalse(adapter.active)


if __name__ == '__main__':
    outcome = unittest.main(exit=False)
    print(json.dumps(METRICS, sort_keys=True))
    sys.exit(not outcome.result.wasSuccessful())

"""Position-interface regressions against the independent 5 ms plant; no ROS."""
import ast
from collections import deque
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'/'mppi'))
from active_arm_model import PiperArmModel
from active_arm_mppi import WholeBodyMPPI
import active_arm_mppi


def definitions(path, names):
    """Load only pure definitions, avoiding the simulator's ROS imports/main."""
    tree = ast.parse(path.read_text()); selected = []
    for node in tree.body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names:
            selected.append(node)
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in node.targets):
            selected.append(node)
    namespace = {'np': np, 'math': math}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)
    return namespace


PLANT = definitions(PACKAGE/'scripts/standby_simulator.py', {'Plant', 'Q_START', 'STEP'})
checked_first_step = definitions(PACKAGE/'scripts/whole_body_mppi.py', {'checked_first_step'})['checked_first_step']
METRICS = {}


class EmptyEnvironment:
    def query(self, spheres, times):
        self.times = np.asarray(times)
        return np.ones(len(spheres)), np.zeros(len(spheres))


class PositionServoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        def merge(target, source):
            for key, value in source.items():
                if isinstance(value, dict) and isinstance(target.get(key), dict): merge(target[key], value)
                else: target[key] = value
        parameters = {}
        for name in ('exp0_param.yaml', 'piper_param.yaml', 'remani_planner_param.yaml', 'fixed_collision_spheres_b.yaml'):
            merge(parameters, yaml.safe_load((PACKAGE.parent/'mm_remani_planner/plan_manage/config'/name).read_text()))
        cls.model = PiperArmModel.from_parameters(parameters)
        cls.model.lower[5], cls.model.upper[5] = -.35, .35
        cls.config = yaml.safe_load((PACKAGE/'config/mppi.yaml').read_text())
        cls.state = np.r_[np.zeros(5), PLANT['Q_START'], np.zeros(6)]
        cls.control = np.zeros(8); cls.control[3] = .15

    def kernel(self, servo=True, dt=.1):
        c, m = self.config, self.model
        return WholeBodyMPPI(m, c['base_limits'],
            (m.lower, m.upper, c['arm_velocity'], c['arm_acceleration']), [],
            step=dt, arm_position_servo=servo)

    def plant(self):
        return PLANT['Plant'](self.model, self.config['base_limits'],
                              self.config['arm_velocity'], self.config['arm_acceleration'])

    def advance_plant(self, plant, target, count=20):
        for _ in range(count):
            acceleration = plant.step(np.zeros(2), target, .005)
            check = self.model.check(plant.state[5:11], plant.state[11:17], acceleration)
            self.assertTrue(check['valid'], check)
            self.assertLessEqual(float(np.max(np.abs(acceleration)-plant.amax)), 1e-10)
            self.assertLessEqual(float(np.max(np.abs(plant.state[11:17])-plant.vmax)), 1e-10)

    def test_first_100ms_and_two_halves_use_one_target(self):
        kernel, plant = self.kernel(), self.plant()
        target = kernel.position_target(self.state, self.control)
        self.assertAlmostEqual(target[1]-self.state[6], .0255)
        prediction = kernel.advance(self.state, self.control, arm_target=target)
        half = kernel.advance(self.state, self.control, .05, arm_target=target)
        halves = kernel.advance(half, self.control, .05, arm_target=target)
        self.advance_plant(plant, target)
        np.testing.assert_allclose(prediction[5:17], halves[5:17], atol=1e-12, rtol=0)
        np.testing.assert_allclose(prediction[5:11], plant.state[5:11], atol=.001, rtol=0)
        METRICS['first_100ms'] = dict(target_lead_rad=float(target[1]-self.state[6]),
            predicted_q_progress_rad=float(prediction[6]-self.state[6]),
            plant_q_progress_rad=float(plant.state[6]-self.state[6]),
            maximum_q_error_rad=float(np.max(np.abs(prediction[5:11]-plant.state[5:11]))))

    def test_two_second_periodic_targets_against_independent_plant(self):
        kernel, plant = self.kernel(), self.plant(); prediction = self.state.copy(); maximum_error = 0.
        single_step_q_errors = []; single_step_velocity_errors = []
        for _ in range(20):
            target = kernel.position_target(plant.state, self.control)
            single_step = kernel.advance(plant.state, self.control, arm_target=target)
            prediction = kernel.advance(prediction, self.control)
            self.advance_plant(plant, target)
            maximum_error = max(maximum_error, float(np.max(np.abs(prediction[5:11]-plant.state[5:11]))))
            single_step_q_errors.append(float(np.max(np.abs(single_step[5:11]-plant.state[5:11]))))
            single_step_velocity_errors.append(float(np.max(np.abs(single_step[11:17]-plant.state[11:17]))))
        # Explicit numerical approximation budget: 10 ms prediction versus
        # independent 5 ms plant, 2 mrad over this 2 s diagnostic (not calibration).
        self.assertLess(maximum_error, .002)
        self.assertLess(max(single_step_q_errors), .002)
        METRICS['periodic_2s'] = dict(predicted_q_progress_rad=float(prediction[6]-self.state[6]),
            plant_q_progress_rad=float(plant.state[6]-self.state[6]),
            plant_final_dq_rad_s=float(plant.state[12]), maximum_q_error_rad=maximum_error,
            regression_q_error_budget_rad=.002, feedback_reset_single_step_q_errors_rad=single_step_q_errors,
            feedback_reset_max_q_error_rad=max(single_step_q_errors),
            feedback_reset_max_velocity_error_rad_s=max(single_step_velocity_errors))

    def test_two_second_periodic_targets_with_real_20ms_feedback_delay(self):
        kernel, plant = self.kernel(), self.plant()
        history = deque([self.state.copy() for _ in range(5)], maxlen=5)
        for tick in range(400):
            if tick % 20 == 0: target = kernel.position_target(history[0], self.control)
            self.advance_plant(plant, target, count=1); history.append(plant.state.copy())
        self.assertGreater(plant.state[6]-self.state[6], .20)
        METRICS['periodic_2s_20ms_feedback'] = dict(plant_q_progress_rad=float(plant.state[6]-self.state[6]),
                                                   plant_final_dq_rad_s=float(plant.state[12]))

    def test_watchdog_holds_one_target_then_brakes_without_return(self):
        for lease in (.30, .275):
            kernel, plant = self.kernel(), self.plant()
            target = kernel.position_target(self.state, self.control)
            states = kernel.position_lease_brake_states(self.state, self.control, lease)
            maximum_error = 0.
            for index in range(1, len(states)):
                for substep in range(10):
                    t=(index-1)*.05+substep*.005
                    self.advance_plant(plant, target if t < lease-1e-10 else None, count=1)
                maximum_error = max(maximum_error, float(np.max(np.abs(states[index,5:11]-plant.state[5:11]))))
            self.assertLess(maximum_error, .001)
            self.assertLess(float(np.max(np.abs(states[-1,11:17]))), .003)
            after = states[int(math.ceil(lease/.05)):, 6]
            self.assertTrue(np.all(np.diff(after) >= -1e-12))
            METRICS['watchdog_'+str(lease)] = dict(samples=len(states), maximum_q_error_rad=maximum_error,
                final_joint_velocity_rad_s=float(states[-1,12]))

    def test_position_targets_obey_joint_bounds_speed_and_003_lead(self):
        kernel = self.kernel(dt=.5)
        state = np.tile(self.state, (3,1)); control = np.zeros((3,8)); control[:,2:] = np.array([100., -100., .001])[:,None]
        state[0,5:11] = self.model.upper-.001; state[1,5:11] = self.model.lower+.001
        targets = kernel.position_target(state, control)
        self.assertTrue(np.all(targets >= kernel.qmin) and np.all(targets <= kernel.qmax))
        self.assertLessEqual(float(np.max(np.abs(targets-state[:,5:11]))), .03+1e-12)
        np.testing.assert_allclose(targets[2]-state[2,5:11], .00037, atol=1e-12)
        resting = self.state.copy(); resting[5:11] = self.model.upper-.0001
        predicted = kernel.advance(resting, np.zeros(8), .1, arm_target=self.model.upper)
        self.assertTrue(np.all(predicted[5:11] <= self.model.upper+1e-10))
        self.assertTrue(np.all(np.abs(predicted[11:17]) <= kernel.vmax+1e-10))

    def test_default_legacy_velocity_response_and_base_remain_unchanged(self):
        kernel = self.kernel(servo=False); self.assertFalse(kernel.arm_position_servo)
        first = kernel.advance(self.state, self.control)
        self.assertAlmostEqual(first[6]-self.state[6], .00175)
        self.assertAlmostEqual(first[12], .035)
        z = self.state.copy()
        for _ in range(20):
            z = kernel.advance(kernel.advance(z, self.control, .05), self.control, .05)
        self.assertAlmostEqual(z[6]-self.state[6], .26775, places=12)
        moving = self.control.copy(); moving[:2] = [.05,.15]
        a = kernel.advance(self.state, moving); b = self.kernel().advance(self.state, moving)
        np.testing.assert_array_equal(a[:5], b[:5])
        np.testing.assert_allclose(a[:5], [.0005*math.cos(.00075), .0005*math.sin(.00075), .0015, .01, .03], atol=1e-12)
        held = kernel.lease_brake_states(self.state, self.control[None], lease=.3)
        self.assertAlmostEqual(held[6,6]-self.state[6], .01575, places=12)

    def test_checked_delivery_returns_position_goal_not_predicted_pose(self):
        kernel = self.kernel(); target = kernel.position_target(self.state, self.control)
        result = checked_first_step(kernel, self.state, self.control, EmptyEnvironment(), 100.)
        np.testing.assert_array_equal(result[5:11], target)
        predicted = kernel.advance(self.state, self.control, arm_target=target)
        self.assertGreater(np.max(np.abs(result[5:11]-predicted[5:11])), .01)

    def test_checked_delivery_rejects_collision_only_during_post_lease_braking(self):
        class BrakingObstacle(EmptyEnvironment):
            def query(self, spheres, times):
                self.times = np.asarray(times)
                return np.where(self.times > 100.30+1e-9, -.01, 1.), np.zeros(len(spheres))
        snapshot = BrakingObstacle()
        with self.assertRaisesRegex(ValueError, 'command_hold_or_braking'):
            checked_first_step(self.kernel(), self.state, self.control, snapshot, 100.)
        self.assertGreater(snapshot.times.max(), 100.30)

    def test_native_matches_numpy_random_limits_high_speeds_hold_and_brake(self):
        kernel = self.kernel(); self.assertIsNotNone(kernel._servo_native)
        rng = np.random.RandomState(8241)
        original = np.tile(self.state, (4,60,1))
        original[...,5:11] = rng.uniform(kernel.qmin, kernel.qmax, (4,60,6))
        original[...,11:17] = rng.uniform(-.7,.7,(4,60,6))
        original[:,0,5:11] = kernel.qmin+1e-10; original[:,1,5:11] = kernel.qmax-1e-10
        target = np.minimum(np.maximum(original[...,5:11]+rng.uniform(-.1,.1,(4,60,6)), kernel.qmin), kernel.qmax)
        maximum = 0.
        for goal in (target, None):
            native, reference = original.copy(), original.copy()
            for dt in (.05,.1,.007,.025,.1,.05,.033,.05):
                kernel._advance_position_arm(native, goal, dt)
                kernel._advance_position_arm_numpy(reference, goal, dt)
                maximum = max(maximum, float(np.max(np.abs(native-reference))))
                np.testing.assert_allclose(native, reference, rtol=0, atol=1e-12)
        METRICS['native_numpy_equivalence'] = dict(states=240, hold_and_brake=True,
            update_comparisons=3840, maximum_absolute_error=maximum)

    def test_native_supports_single_state_and_broadcast_targets(self):
        kernel = self.kernel()
        for shape in ((17,), (8,17), (3,8,17)):
            state = np.broadcast_to(self.state, shape).copy()
            for target in (self.state[5:11]+.01,
                           np.broadcast_to(self.state[5:11]-.01, shape[:-1]+(6,))):
                native = kernel._advance_position_arm(state.copy(), target, .05)
                reference = kernel._advance_position_arm_numpy(state.copy(), target, .05)
                np.testing.assert_allclose(native, reference, rtol=0, atol=1e-12)

    def test_native_rejects_wrong_state_dtype_layout_and_shape(self):
        kernel = self.kernel()
        for state in (self.state.astype(np.float32), np.zeros((3,34))[:,::2], np.zeros(16)):
            with self.assertRaisesRegex(ValueError, 'float64'):
                kernel._advance_position_arm(state, None, .05)
        with self.assertRaises(ValueError):
            kernel._advance_position_arm(self.state.copy(), np.zeros(7), .05)

    def test_missing_library_falls_back_and_legacy_never_loads_it(self):
        with patch.object(active_arm_mppi.ctypes, 'CDLL', side_effect=OSError('library absent')) as loader:
            kernel = self.kernel(); self.assertIsNone(kernel._servo_native); self.assertGreater(loader.call_count,0)
        target = kernel.position_target(self.state, self.control)
        actual = kernel.advance(self.state, self.control, arm_target=target)
        expected = kernel._advance_position_arm_numpy(self.state.copy(), target, .1)
        np.testing.assert_array_equal(actual, expected)
        with patch.object(active_arm_mppi, '_load_position_servo_library', side_effect=AssertionError('unexpected load')):
            legacy = self.kernel(servo=False)
            self.assertAlmostEqual(legacy.advance(self.state,self.control)[12], .035)

    def old_position_step(self, kernel, state, control, dt, target=None, brake=False):
        """Pre-fusion complete update: original NumPy base and arm arithmetic."""
        if brake:
            control = np.zeros(np.asarray(state).shape[:-1]+(8,))
        elif target is None:
            target = kernel.position_target(state, control)
        result, _ = kernel._advance_base(state, control, dt)
        return kernel._advance_position_arm_numpy(result, target, dt)

    def test_fused_full_step_matches_old_common_base_limits_and_wrapped_angles(self):
        kernel = self.kernel(); self.assertIsNotNone(kernel._servo_native.whole_body)
        rng = np.random.RandomState(4316)
        states = rng.uniform(-3., 3., (4,60,17))
        states[...,2] = rng.uniform(-40*np.pi,40*np.pi,(4,60))
        states[:,0:4,2] = [-np.pi, np.pi, -8*np.pi, 9*np.pi]
        states[...,5:11] = rng.uniform(kernel.qmin, kernel.qmax, (4,60,6))
        states[:,0,5:11] = kernel.qmin+1e-10; states[:,1,5:11] = kernel.qmax-1e-10
        states[...,11:17] = rng.uniform(-.7,.7,(4,60,6))
        controls = rng.uniform(-2.,2.,(4,60,8)); controls[:,0,:2] = 0.
        maximum = 0.; original_limits = kernel.limits.copy()
        for override in ({}, {'vmax': .01, 'accel': .01},
                         {'wmax': .01, 'angular_accel': .01},
                         {'wheel_v': .015, 'wheel_a': .02}):
            kernel.limits = dict(original_limits, **override)
            for dt in (.05,.1,.007,.025):
                actual = kernel.advance(states, controls, dt)
                expected = self.old_position_step(kernel, states, controls, dt)
                maximum = max(maximum, float(np.max(np.abs(actual-expected))))
                np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
        METRICS['fused_full_step_equivalence'] = dict(states=240, limit_sets=4,
            update_comparisons=3840, maximum_absolute_error=maximum, absolute_tolerance=1e-12)

    def test_fused_multistep_and_braking_match_old_complete_advance(self):
        kernel = self.kernel(); rng = np.random.RandomState(3117)
        actual = np.tile(self.state, (24,1)); actual[:,:5] = rng.uniform(-4.,4.,(24,5))
        reference = actual.copy(); maximum = 0.
        for index in range(80):
            dt = (.05,.007,.025,.1)[index % 4]
            controls = rng.uniform(-1.,1.,(24,8))
            if index % 4 == 3:
                actual = kernel.advance_position_brake(actual, dt)
                reference = self.old_position_step(kernel, reference, None, dt, brake=True)
            else:
                actual = kernel.advance(actual, controls, dt)
                reference = self.old_position_step(kernel, reference, controls, dt)
            maximum = max(maximum, float(np.max(np.abs(actual-reference))))
            # libm trigonometry and repeated roundoff may accumulate; this is
            # an equivalence allowance, not a changed model/safety threshold.
            np.testing.assert_allclose(actual, reference, rtol=0, atol=1e-10)
        METRICS['fused_multistep_equivalence'] = dict(states=24, steps=80,
            maximum_absolute_error=maximum, absolute_tolerance=1e-10, includes_braking=True)

    def test_fused_broadcasts_preserve_inputs_and_independent_samples(self):
        kernel = self.kernel(); controls = self.control.copy(); controls[:2] = [.7,-.8]
        for shape in ((17,), (8,17), (3,8,17)):
            state = np.broadcast_to(self.state, shape).copy()
            variants = (controls, np.broadcast_to(controls, shape[:-1]+(8,)))
            for control in variants:
                target = self.state[5:11]+.01
                saved = (state.copy(), control.copy(), target.copy())
                actual = kernel.advance(state, control, .05, arm_target=target)
                # The old base method required matching batch dimensions;
                # explicitly broadcast its input for the same per-row physics.
                expected = self.old_position_step(kernel, state,
                    np.broadcast_to(control, shape[:-1]+(8,)), .05, target=target)
                np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-12)
                for original, before in zip((state,control,target), saved):
                    np.testing.assert_array_equal(original, before)
                self.assertFalse(np.shares_memory(actual,state))
        with self.assertRaises(ValueError): kernel.advance(self.state,np.zeros(7))
        with self.assertRaises(ValueError): kernel.advance(self.state.astype(np.float32),self.control)
        with self.assertRaises(ValueError): kernel.advance(self.state,self.control,dt=float('nan'))

    def test_old_arm_only_library_keeps_complete_python_base_fallback(self):
        kernel = self.kernel(); kernel._servo_native.whole_body = None
        controls = self.control.copy(); controls[:2] = [-.7,.9]
        expected = self.old_position_step(kernel,self.state,controls,.05)
        np.testing.assert_allclose(kernel.advance(self.state,controls,.05),expected,rtol=0,atol=1e-12)
        kernel._servo_native = None
        np.testing.assert_array_equal(kernel.advance(self.state,controls,.05),expected)


if __name__ == '__main__':
    outcome = unittest.main(exit=False)
    print(json.dumps(METRICS, sort_keys=True))
    sys.exit(not outcome.result.wasSuccessful())

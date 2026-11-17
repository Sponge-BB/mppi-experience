"""Native phase-consistent guidance regressions, using the production adapter.

Small geometry fixtures verify control contracts only; they are not real-vehicle
or full-robot collision validation. Coupled recorded-clock replay lives in the
corresponding experiment result directory.
"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'/'mppi'))
from active_arm_mppi import WholeBodyMPPI, wrap
from native_reference_adapter import NativeReferenceAdapter
import test_mppi_native_phase as phases
import test_mppi_navigation_priority as priority


class Model:
    def body_spheres_batch(self, q, base):
        spheres = np.zeros((len(q), 1, 4))
        spheres[:, 0, :2], spheres[:, 0, 3] = base[:, :2], .01
        return spheres

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class Scene:
    def __init__(self, gap=1.):
        self.gap = gap

    def query(self, spheres, times):
        return np.full(len(times), self.gap), np.zeros(len(times))


class RecordingKernel(WholeBodyMPPI):
    def plan(self, *args, **kwargs):
        self.tracking_received = kwargs.get('tracking')
        self.reference_received = np.array(args[1], copy=True)
        return super().plan(*args, **kwargs)


def ready(gear=1, curve=True, horizon=8):
    kernel = RecordingKernel(Model(),
        dict(track=.37, vmax=.30, wmax=.35, wheel_v=.2296,
             accel=.1, angular_accel=.3, wheel_a=.2975),
        (np.full(6, -3.), np.full(6, 3.), np.full(6, .15), np.full(6, .35)),
        [], enabled=False, horizon=horizon, random_samples=0)
    kernel._legacy_native = None
    adapter = NativeReferenceAdapter(kernel, kernel, resume_distance=.05)
    adapter.feed_goal(phases.goal(), 10.01)
    message = phases.trajectory(gear=gear)
    if curve:
        coefficients = np.asarray(message.trajectory[0].data).reshape((8, 8), order='F').copy()
        coefficients[1, -3] = .03
        message.trajectory[0].data = coefficients.ravel(order='F').tolist()
    adapter.feed_trajectory(message, 10.11)
    return adapter, kernel


def run(adapter, state, scale, stamp=20., phase=.4, scene=None):
    adapter.clock = lambda: stamp
    adapter.feed_progress(phases.progress(stamp, phase=phase, scale=scale), stamp)
    result, info = adapter.plan(state, np.zeros((2, 4)), state[5:11],
                                Scene() if scene is None else scene, stamp)
    return result, info


def pose_at(phase, gear=1):
    return np.array([1.+gear*.2*phase, .03*phase*phase,
                     np.arctan2(gear*.06*phase, .2)])


class TimedGuidanceTests(unittest.TestCase):
    def test_adapter_samples_exact_polynomial_at_current_and_forecast_phases(self):
        for gear in (-1, 1):
            adapter, kernel = ready(gear)
            state = np.zeros(17); state[:3] = pose_at(.4, gear)
            result, _ = run(adapter, state, .37)
            self.assertIsNotNone(result)
            samples = np.asarray(kernel.tracking_received['base_samples'])
            self.assertEqual(samples.shape, (kernel.H+1, 6))
            t = .4+.37*kernel.dt*np.arange(kernel.H+1)
            raw_speed = np.hypot(.2, .06*t)
            np.testing.assert_allclose(samples[:, 0], 1.+gear*.2*t, atol=1e-12)
            np.testing.assert_allclose(samples[:, 1], .03*t*t, atol=1e-12)
            np.testing.assert_allclose(samples[:, 2], np.arctan2(gear*.06*t, .2), atol=1e-12)
            np.testing.assert_allclose(samples[:, 3], gear*.37*raw_speed, atol=1e-12)
            np.testing.assert_allclose(samples[:, 4], gear*.012*.37/raw_speed**2, atol=1e-12)
            np.testing.assert_allclose(samples[:, 5], raw_speed, atol=1e-12)

    def test_zero_and_tiny_clock_scales_have_continuous_first_control(self):
        for gear in (-1, 1):
            for lateral in (-.08, -.02, 0., .02, .08):
                with self.subTest(gear=gear, lateral=lateral):
                    commands = []
                    for scale in (0., 1e-12, 1e-9, 1e-6):
                        adapter, kernel = ready(gear)
                        state = np.zeros(17); state[:3] = pose_at(.4, gear)
                        state[0] -= gear*.02; state[1] += lateral; state[2] += .2
                        result, _ = run(adapter, state, scale)
                        self.assertIsNotNone(result)
                        commands.append(result[0][0, :2])
                    np.testing.assert_allclose(commands[:3], np.tile(commands[0], (3, 1)), atol=1e-7, rtol=0.)
                    np.testing.assert_allclose(commands[3], commands[0], atol=1e-5, rtol=0.)

    def test_forward_and_reverse_gear_are_preserved_through_pause_and_resume(self):
        for gear in (-1, 1):
            for scale in (0., 1e-9, .001, .1, 1.):
                adapter, kernel = ready(gear, curve=False)
                state = np.zeros(17); state[0] = 1.+gear*.2*.4-gear*.02
                result, _ = run(adapter, state, scale)
                self.assertIsNotNone(result)
                self.assertEqual(kernel.tracking_received['gear'], gear)
                self.assertGreater(gear*result[0][0, 0], 0., (gear, scale))
                self.assertAlmostEqual(result[0][0, 1], 0., places=12)

    def test_old_two_point_five_centimetre_boundary_has_no_steering_jump(self):
        for gear in (-1, 1):
            for scale in (0., 1e-9, .2, 1.):
                controls = []
                for distance in (.025-1e-7, .025+1e-7):
                    adapter, _ = ready(gear, curve=False)
                    state = np.zeros(17)
                    state[0] = 1.+gear*.2*.4-gear*distance; state[2] = .2
                    result, _ = run(adapter, state, scale)
                    self.assertIsNotNone(result)
                    controls.append(result[0][0, :2])
                np.testing.assert_allclose(controls[0], controls[1], atol=1e-5, rtol=0.)

    def test_equivalent_yaw_representations_produce_identical_control(self):
        for gear in (-1, 1):
            controls = []
            for yaw in (-np.pi+.03, np.pi+.03):
                adapter, _ = ready(gear)
                state = np.zeros(17); state[:3] = pose_at(.4, gear); state[2] = yaw
                result, _ = run(adapter, state, .1)
                self.assertIsNotNone(result)
                controls.append(result[0][0, :2])
            np.testing.assert_allclose(controls[0], controls[1], atol=1e-12, rtol=0.)

    def test_forecast_stays_in_current_gear_and_holds_endpoint(self):
        adapter, kernel = ready(1, curve=False)
        adapter.feed_trajectory(phases.trajectory(ident=2, x=1.4, gear=-1), 10.12)
        state = np.zeros(17); state[0] = 1.38
        result, _ = run(adapter, state, 1., phase=1.9)
        self.assertIsNotNone(result)
        samples = np.asarray(kernel.tracking_received['base_samples'])
        self.assertEqual(kernel.tracking_received['gear'], 1)
        np.testing.assert_allclose(samples[1:, 0], 1.4, atol=1e-12)
        np.testing.assert_array_equal(samples[1:, 3:5], 0.)
        # Match C++ strict t > duration: equality still belongs to the old gear.
        result, _ = run(adapter, state, 1., stamp=20.1, phase=2.)
        self.assertIsNotNone(result)
        self.assertEqual(kernel.tracking_received['gear'], 1)
        np.testing.assert_array_equal(kernel.tracking_received['base_samples'][:, 3:5], 0.)
        # Only measured phase beyond the boundary owns the reverse block.
        result, _ = run(adapter, state, 1., stamp=20.2, phase=2.+1e-8)
        self.assertIsNotNone(result)
        self.assertEqual(kernel.tracking_received['gear'], -1)
        self.assertLess(kernel.tracking_received['base_samples'][0, 3], 0.)

    def test_candidate_forecasts_do_not_change_measured_state_or_each_other(self):
        adapter, kernel = ready(1)
        state = np.zeros(17); state[:3] = pose_at(.4); state[1] += .065
        result, _ = run(adapter, state, .001)
        self.assertIsNotNone(result)
        context = kernel.tracking_received
        reference = kernel.reference_received
        arm = adapter._arm_samples(.4+.001*kernel.dt*np.arange(1, kernel.H+1))
        modes = [(1., 0.), (.65, .2), (0., 0.)]
        before = state.copy(); frozen_samples = context['base_samples'].copy()
        first, _ = kernel.proposals(state, reference, arm, modes, tracking=context)
        second, _ = kernel.proposals(state, reference, arm, modes, tracking=context)
        reversed_modes, _ = kernel.proposals(state, reference, arm, modes[::-1], tracking=context)
        np.testing.assert_array_equal(first, second)
        # The reusable previous sequence is appended after structured modes.
        count = len(modes)
        np.testing.assert_array_equal(first[:count], reversed_modes[:count][::-1])
        np.testing.assert_array_equal(first[count:], reversed_modes[count:])
        np.testing.assert_array_equal(state, before)
        np.testing.assert_array_equal(context['base_samples'], frozen_samples)

    def test_actual_plan_and_lease_checks_still_reject_collision(self):
        adapter, kernel = ready(-1, curve=False)
        state = np.zeros(17); state[0] = 1.-.2*.4+.02
        result, _ = run(adapter, state, .2)
        self.assertIsNotNone(result)
        self.assertTrue(kernel.lease_brake(state, result[0], Scene(), 20.)[0])
        self.assertFalse(kernel.lease_brake(state, result[0], Scene(-1.), 20.)[0])
        rejected, _ = run(adapter, state, .2, stamp=20.1, scene=Scene(-1.))
        self.assertIsNone(rejected)

    def test_timed_cost_prefers_yaw_correction_at_identical_positions(self):
        adapter, kernel = ready(1, curve=False)
        state = np.zeros(17); state[0] = 1.08; state[2] = .3
        result, _ = run(adapter, state, 0.)
        self.assertIsNotNone(result)
        controls = np.zeros((2, kernel.H, 8))
        controls[0, :, 1], controls[1, :, 1] = -.2, .2
        arm = adapter._arm_samples(np.full(kernel.H, .4))
        context, reference = kernel.tracking_received, kernel.reference_received
        costs, valid, _, predicted = kernel.evaluate(state, controls, reference, arm,
                                                     Scene(), 20., tracking=context)
        self.assertTrue(valid.all())
        np.testing.assert_array_equal(predicted[0, :, :2], predicted[1, :, :2])
        self.assertLess(costs[0], costs[1], 'Equal-position rollouts must favor correcting native yaw')
        legacy_costs, _, _, _ = kernel.evaluate(state, controls, reference, arm, Scene(), 20.)
        self.assertAlmostEqual(legacy_costs[0], legacy_costs[1], places=12)

    def test_native_timed_arm_only_selection_preserves_full_nominal_base_sequence(self):
        # Reuse the established crossing-sphere geometry, but enter through the
        # actual native adapter and its timed scoring branch this time.
        scene_fixture = priority.NavigationPriorityTests()
        scene_fixture.setUp()
        kernel, state = scene_fixture.planner, scene_fixture.state
        adapter = NativeReferenceAdapter(kernel, kernel)
        adapter.feed_goal(phases.goal(), 10.01)
        message = phases.trajectory(x=0.)
        coefficients = np.zeros((8, 8)); coefficients[0, -2] = .16
        message.trajectory[0].data = coefficients.ravel(order='F').tolist()
        adapter.feed_trajectory(message, 10.11)
        captured = {}
        original_plan = kernel.plan
        def record_plan(*args, **kwargs):
            captured.update(reference=args[1], arm=args[2], tracking=kwargs['tracking'])
            return original_plan(*args, **kwargs)
        kernel.plan = record_plan
        obstacle = priority.MovingSphere(.3, .5)
        result, info = run(adapter, state, 1., stamp=100., phase=0., scene=obstacle)
        self.assertIsNotNone(result)
        self.assertEqual(info['mode'], 'arm_only')
        self.assertTrue(info['intervention'])
        reference, arm, context = captured['reference'], captured['arm'], captured['tracking']
        nominal, _ = kernel.proposals(state, reference, arm, [(1., 0.)], tracking=context)
        _, valid, _, predicted = kernel.evaluate(state, nominal, reference, arm, obstacle,
                                                  100., tracking=context)
        self.assertFalse(valid[0]); self.assertTrue(valid[1:].any())
        controls, states = result
        self.assertGreater(np.max(np.abs(controls[:, 2:])), .1)
        np.testing.assert_array_equal(controls[:, :2], nominal[0, :, :2])
        np.testing.assert_array_equal(states[:, :5], predicted[0, :, :5])
        _, selected_valid, _, checked = kernel.evaluate(state, controls[None], reference,
                                      arm, obstacle, 100., tracking=context)
        self.assertTrue(selected_valid[0])
        np.testing.assert_array_equal(states, checked[0])


if __name__ == '__main__':
    unittest.main()

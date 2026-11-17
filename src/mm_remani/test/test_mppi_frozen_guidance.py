"""Offline frozen-reference regressions; these are not hardware validation."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'/'mppi'))
from active_arm_mppi import WholeBodyMPPI, wrap
from native_reference_adapter import NativeReferenceAdapter
import test_mppi_native_phase as phases


class SmallRobot:
    def body_spheres_batch(self, q, base):
        spheres = np.zeros((len(q), 1, 4))
        spheres[:, 0, :2] = base[:, :2]
        spheres[:, 0, 3] = .01
        return spheres

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class Scene:
    def __init__(self, gap=1.):
        self.gap = gap

    def query(self, spheres, times):
        return np.full(len(times), self.gap), np.zeros(len(times))


def kernel(horizon=12):
    planner = WholeBodyMPPI(SmallRobot(),
        dict(track=.37, vmax=.05, wmax=.35, wheel_v=.2296,
             accel=.1, angular_accel=.3, wheel_a=.2975),
        (np.full(6, -3.), np.full(6, 3.), np.full(6, .15), np.full(6, .35)),
        [], enabled=False, horizon=horizon, random_samples=0)
    planner._legacy_native = None
    return planner


def frozen(pose=(0., 0., 1.)):
    return np.tile(np.r_[pose, 0.], (2, 1))


def tracking(reference, horizon=12, gear=1, key=('goal', 'native'), resume=.05):
    # A frozen virtual clock still has a geometrically defined native tangent.
    samples = np.tile(np.r_[reference[0], 0., .2], (horizon+1, 1))
    return dict(gear=gear, identity=key, resume_distance=resume,
                base_samples=samples, yaw_hold_speed=.04)


class FrozenGuidanceTests(unittest.TestCase):
    def test_near_point_uses_one_yaw_on_both_sides_of_old_threshold(self):
        planner, reference = kernel(), frozen()
        context = tracking(reference)
        for distance in (.024, .026, .034, .041, .049, .0251, .0249):
            state = np.zeros(17); state[0] = distance
            scalar = planner.guide(state, reference, tracking=context)
            controls, _ = planner.proposals(state, reference, state[5:11],
                                            [(1., 0.)], tracking=context)
            np.testing.assert_array_equal(scalar, controls[0, 0, :2])
            self.assertAlmostEqual(scalar[0], -.9*distance)
            self.assertEqual(scalar[1], planner.limits['wmax'])

    def test_geometry_capture_hysteresis_scales_with_native_resume_radius(self):
        for resume in (.03, .05, .10):
            planner, reference = kernel(), frozen()
            state = np.zeros(17)
            context = tracking(reference, resume=resume)
            state[1] = 1.01*resume
            planner.guide(state, reference, tracking=context)
            self.assertTrue(planner._tracking_capture)
            state[1] = .80*resume
            planner.guide(state, reference, tracking=context)
            self.assertTrue(planner._tracking_capture)
            state[1] = .69*resume
            planner.guide(state, reference, tracking=context)
            self.assertFalse(planner._tracking_capture)
            state[1] = .99*resume
            planner.guide(state, reference, tracking=context)
            self.assertFalse(planner._tracking_capture)

    def test_explicit_reverse_gear_survives_zero_and_tiny_scaled_speeds(self):
        state = np.zeros(17)
        for gear in (-1, 1):
            for scale in (0., 1e-9, .001, .02, 1.):
                planner = kernel()
                reference = frozen((gear*.15, .02, .1))
                reference[:, 3] = gear*.2*scale
                context = tracking(reference, gear=gear)
                controls, _ = planner.proposals(state, reference, state[5:11],
                                                [(1., 0.)], tracking=context)
                self.assertGreater(gear*controls[0, 0, 0], 0., (gear, scale))
                self.assertLess(abs(controls[0, 0, 1]), planner.limits['wmax'])

    def test_forecast_stages_are_independent_and_do_not_advance_measured_stage(self):
        planner, reference = kernel(horizon=80), frozen((0., .06, 0.))
        state = np.zeros(17)
        modes = [(1., 0.), (0., 0.)]
        context = tracking(reference, horizon=planner.H)
        controls, _ = planner.proposals(state, reference, state[5:11], modes, tracking=context)
        self.assertTrue(np.any(controls[0, :, 0] > .001))
        np.testing.assert_array_equal(controls[1, :, 0], 0.)
        self.assertTrue(planner._tracking_capture, 'A predicted arrival is not measured arrival')
        again, _ = planner.proposals(state, reference, state[5:11], modes, tracking=context)
        np.testing.assert_array_equal(controls, again)
        reordered, _ = planner.proposals(state, reference, state[5:11], modes[::-1], tracking=context)
        np.testing.assert_array_equal(controls, reordered[::-1])

    def test_new_reference_or_gear_resets_measured_stage_but_phase_scale_do_not(self):
        for reset in ('goal', 'native', 'gear'):
            with self.subTest(reset=reset):
                planner, reference = kernel(), frozen((0., .06, 0.))
                state = np.zeros(17)
                context = tracking(reference)
                planner.guide(state, reference, tracking=context)
                self.assertTrue(planner._tracking_capture)
                state[1] = .02  # 4 cm inside capture hysteresis, outside exit.
                planner.previous = np.ones((planner.H, 8))
                context['base_samples'][:, 3] = .00000001
                planner.guide(state, reference, tracking=context)
                self.assertTrue(planner._tracking_capture)
                self.assertIsNotNone(planner.previous)
                if reset == 'gear': context['gear'] = -1
                else: context['identity'] = (reset, 'changed')
                planner.guide(state, reference, tracking=context)
                self.assertFalse(planner._tracking_capture)
                self.assertIsNone(planner.previous)

    def test_yaw_wrap_uses_short_rotation_and_equivalent_pi_representations(self):
        state = np.zeros(17); state[2] = np.pi-.04
        commands = []
        for yaw in (-np.pi+.03, np.pi+.03):
            reference = frozen((0., 0., yaw))
            commands.append(kernel().guide(state, reference, tracking=tracking(reference)))
        np.testing.assert_allclose(commands[0], [0., 1.8*.07], atol=1e-14)
        np.testing.assert_allclose(commands[0], commands[1], atol=1e-14)

    def test_frozen_pose_recovers_both_native_clock_conditions_in_closed_loop(self):
        for gear in (-1, 1):
            for yaw in (0., np.pi-.03, -np.pi+.03):
                with self.subTest(gear=gear, yaw=yaw):
                    planner = kernel()
                    state = np.zeros(17); state[2] = wrap(yaw+.8)
                    state[3:5] = [gear*.025, -.05]
                    heading = np.array([np.cos(yaw), np.sin(yaw)])
                    lateral = np.array([-heading[1], heading[0]])
                    target = gear*.15*heading+.06*lateral
                    reference = frozen((*target, yaw))
                    context = tracking(reference, gear=gear)
                    for cycle in range(600):
                        command = planner.guide(state, reference, tracking=context)
                        for _ in range(2):
                            state, _ = planner._advance_base(state, np.r_[command, np.zeros(6)], .05)
                        if (np.linalg.norm(state[:2]-target) <= .05
                                and abs(wrap(state[2]-yaw)) <= .15): break
                    else:
                        self.fail('Frozen pose never reached both native clock recovery thresholds')

    def test_real_plan_retains_collision_and_lease_braking_checks(self):
        planner, state = kernel(), np.zeros(17)
        reference = frozen((-.15, 0., 0.)); context = tracking(reference, gear=-1)
        result, _ = planner.plan(state, reference, state[5:11], Scene(), 100., tracking=context)
        self.assertIsNotNone(result)
        self.assertLess(result[0][0, 0], 0.)
        self.assertTrue(planner.lease_brake(state, result[0], Scene(), 100.)[0])
        self.assertFalse(planner.lease_brake(state, result[0], Scene(-1.), 100.)[0])
        result, _ = planner.plan(state, reference, state[5:11], Scene(-1.), 100., tracking=context)
        self.assertIsNone(result)

    def test_ordinary_guide_and_proposals_keep_pre_fix_numerical_outputs(self):
        # Golden base controls from the saved pre-fix source, with NumPy plant.
        planner, state = kernel(horizon=4), np.zeros(17)
        state[:5] = [.1, -.03, 2.8, .01, -.02]
        reference = np.c_[np.linspace(0., .5, 11), .08*np.sin(np.linspace(0., 1., 11)),
                          np.linspace(.1, .3, 11), np.full(11, -.06)]
        np.testing.assert_allclose(planner.guide(state, reference),
                                   [-.04054686399765087, .35], atol=1e-15, rtol=0.)
        controls, _ = planner.proposals(state, reference, state[5:11], [(1., 0.), (.65, .2)])
        expected = np.array([
            [[-.04054686399765087, .35], [-.04057570086582901, .35],
             [-.04066947298167536, .35], [-.04082747258770803, .35]],
            [[-.022052047102965714, .35], [-.02207891918563483, .35],
             [-.022165446812282493, .35], [-.022311076061647565, .35]]])
        np.testing.assert_allclose(controls[:, :, :2], expected, atol=1e-15, rtol=0.)


class AdapterTrackingTests(unittest.TestCase):
    def setUp(self):
        self.standby, self.navigation = phases.Kernel(), phases.Kernel()
        self.adapter = NativeReferenceAdapter(self.standby, self.navigation, resume_distance=.06)
        self.adapter.feed_goal(phases.goal(), 10.01)
        self.adapter.feed_trajectory(phases.trajectory(gear=-1), 10.11)

    def plan(self, stamp, phase=.4, scale=0.):
        self.adapter.feed_progress(phases.progress(stamp, phase=phase, scale=scale), stamp)
        result, _ = self.adapter.plan(np.zeros(17), np.zeros((2, 4)), np.zeros(6), None, stamp)
        self.assertIsNotNone(result)
        return self.navigation.tracking_calls[-1]

    def test_reverse_gear_is_separate_from_scaled_velocity_and_tracking_identity(self):
        first = self.plan(20.)
        self.assertEqual(first['gear'], -1)
        self.assertEqual(first['resume_distance'], .06)
        np.testing.assert_array_equal(self.navigation.calls[-1][0][:, 3], 0.)
        self.assertEqual(self.plan(20.1)['identity'], first['identity'])
        advanced = self.plan(20.2, phase=.401)
        self.assertEqual(advanced['identity'], first['identity'])
        moving = self.plan(20.3, phase=.401, scale=1e-9)
        self.assertEqual(moving['identity'], first['identity'])
        self.assertEqual(moving['gear'], -1)
        self.assertTrue(np.all(self.navigation.calls[-1][0][:, 3] < 0.))

    def test_standby_keeps_its_original_plan_signature(self):
        self.adapter.finish()
        result, _ = self.adapter.plan(np.zeros(17), frozen(), np.zeros(6), None, 20.)
        self.assertIsNotNone(result)
        self.assertIsNone(self.standby.tracking_calls[-1])


if __name__ == '__main__':
    unittest.main()

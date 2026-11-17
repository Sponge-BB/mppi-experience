"""Acceptance checks for the separately named reactive standby extension."""
import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial import cKDTree

from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot, WholeBodyMPPI
from active_arm_scene import Q_START, Q_RETRACT
from standby_whole_body_mppi import StandbyWholeBodyMPPI


def sensed_snapshot(moving=False):
    """Synthetic sensor observation, without access to a scenario/controller."""
    h = np.zeros(32); h[:2] = [3, 1]; h[2:8] = [1, 1000, 1, 120, 120, 50]
    h[8:12] = [-2., -3., -.1, .05]; h[12] = 67
    h[14:20] = [1, .35, 3., 1000, 1000, 1]; h[20] = 1
    groups = []
    if moving:
        groups = [(np.array([0., .1, 0.]), cKDTree(np.array([[.14, -.14, 1.065]])))]
    return Snapshot(h, np.full((120, 120, 50), 10.), groups)


class StandbyWholeBodyAcceptance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = PiperArmModel.from_defaults()
        cls.limits = json.loads((Path(__file__).parent/
            'results/short_prefix_rejoin_20261101_01/midcross_B/limits.json').read_text())
        cls.reference = np.array([[0., 0., 0., 0.]])
        cls.state = np.r_[np.zeros(5), Q_START, np.zeros(6)]

    def kernel(self, postures=None, **kwargs):
        m = self.model
        return StandbyWholeBodyMPPI(m, self.limits,
            [m.lower, m.upper, m.velocity_limits, m.acceleration_limits],
            [Q_RETRACT] if postures is None else postures,
            random_samples=6, seed=11, **kwargs)

    def test_zero_anchor_does_not_erase_evasive_base_candidates(self):
        k = self.kernel()
        controls, labels = k.proposals(self.state, self.reference, Q_START)
        by_label = {label: u for label, u in zip(labels, controls)}
        self.assertGreater(by_label[('reference', 'forward')][0, 0], 0.)
        self.assertLess(by_label[('reference', 'reverse')][0, 0], 0.)
        self.assertGreater(by_label[('reference', 'turn_left')][0, 1], 0.)
        self.assertLess(by_label[('reference', 'turn_right')][0, 1], 0.)
        self.assertGreater(np.max(np.abs(controls[:, :, 2:])), .1)

    def test_eight_dimensional_noise_and_normalized_exponential_weights(self):
        k = self.kernel()
        planned, info = k.plan(self.state, self.reference, Q_START, sensed_snapshot(), 1000.)
        self.assertIsNotNone(planned)
        self.assertEqual(info['sample_dimensions'], 8)
        self.assertEqual(info['random_samples'], 6)
        self.assertTrue(np.all(np.asarray(info['noise_rms_per_coordinate']) > 0.))
        self.assertTrue(np.all(np.asarray(info['base_noise_rms']) > 0.))
        self.assertAlmostEqual(sum(info['weights']), 1., places=12)
        scores = np.asarray(info['weighted_pool_costs'])
        expected = np.exp(-(scores-scores.min())/info['lambda_temperature'])
        np.testing.assert_allclose(info['weights'], expected/expected.sum())

    def test_arm_clearance_option_competes_with_nonzero_base_options(self):
        k = self.kernel(); snapshot = sensed_snapshot(moving=True)
        controls, labels = k.proposals(self.state, self.reference, Q_START)
        costs, valid, _, _ = k.evaluate(self.state, controls, self.reference, Q_START, snapshot, 1000.)
        index = {label: i for i, label in enumerate(labels)}
        self.assertFalse(valid[index[('reference', 'hold')]])
        self.assertTrue(valid[index[('reference', 'forward')]])
        self.assertTrue(valid[index[('posture_0', 'hold')]])
        planned, info = k.plan(self.state, self.reference, Q_START, snapshot, 1000.)
        self.assertIsNotNone(planned)
        self.assertGreater(info['arm_motion'], .1)
        # Measure the selected base contribution; it is an optimization
        # outcome, while the preceding checks prove alternatives existed.
        self.assertLess(info['selected_base_displacement_m'], .05)
        self.assertLess(costs[index[('posture_0', 'hold')]], costs[index[('reference', 'forward')]])
        self.assertGreaterEqual(info['min_gap'], 0.)

    def test_base_can_be_selected_when_no_arm_posture_channel_is_supplied(self):
        k = self.kernel(postures=[])
        planned, info = k.plan(self.state, self.reference, Q_START, sensed_snapshot(moving=True), 1000.)
        self.assertIsNotNone(planned)
        self.assertGreater(info['selected_base_displacement_m'], .02)
        self.assertGreater(np.max(np.abs(planned[0][:, :2])), .01)

    def test_initial_collision_cannot_be_escaped_by_ignoring_current_state(self):
        k = self.kernel(); snapshot = sensed_snapshot()
        center = k.spheres(self.state[None])[0, -1, :3]
        snapshot.groups = [(np.array([2., 0., 0.]), cKDTree(center[None]))]
        planned, info = k.plan(self.state, self.reference, Q_START, snapshot, 1000.)
        self.assertIsNone(planned)
        self.assertEqual(info['selection'], 'none')

    def test_original_dynamics_and_first_command_lease_validation_are_inherited(self):
        self.assertIs(StandbyWholeBodyMPPI.advance, WholeBodyMPPI.advance)
        self.assertIs(StandbyWholeBodyMPPI.lease_brake, WholeBodyMPPI.lease_brake)
        k = self.kernel(); state = self.state.copy()
        state[5] = self.model.upper[0]-.13; state[11] = .30
        controls = np.zeros((k.H, 8)); controls[:, 2] = -.45; controls[0, 2] = .45
        valid, _ = k.lease_brake(state, controls, sensed_snapshot(), 1000., lease=.25)
        self.assertFalse(valid)

    def test_rejects_disabled_base_sampling_configuration(self):
        with self.assertRaises(ValueError):
            self.kernel(config={'base_modes': [['hold', 0., 0.]]})
        with self.assertRaises(ValueError):
            self.kernel(config={'noise_sigma': [0., .08, .035, .035, .035, .035, .035, .035]})


if __name__ == '__main__':
    unittest.main()

"""Offline speed/forecast compatibility; imports ROS messages but starts no node.

The small robot isolates speed constraints from physical collision geometry.
Actual Snapshot queries retain the dynamic forecast's two-second boundary.
"""
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'))
sys.path.insert(0, str(PACKAGE/'scripts'/'mppi'))
from active_arm_mppi import Snapshot, WholeBodyMPPI
from whole_body_mppi import base_command


class ClearRobot:
    def body_spheres_batch(self, q, base):
        spheres = np.zeros((len(q), 1, 4))
        spheres[:, 0, :2] = base[:, :2]
        spheres[:, 0, 3] = .01
        return spheres

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class SpeedCoverageTests(unittest.TestCase):
    def setUp(self):
        self.profile = yaml.safe_load((PACKAGE/'config'/'mppi.yaml').read_text())
        p = self.profile
        self.kernel = WholeBodyMPPI(ClearRobot(), p['base_limits'],
            ([-3.]*6, [3.]*6, p['arm_velocity'], p['arm_acceleration']), [],
            horizon=p['horizon'], step=1./p['rate_hz'], random_samples=0)

    def test_reference_and_delivered_speed_cover_remani(self):
        real = yaml.safe_load((PACKAGE/'config'/'real_speed.yaml').read_text())
        # The existing robot wheel radius is 0.07 m.
        remani_v = real['remani_planner_node']['mm']['mobile_base_max_wheel_omega']*.07
        self.assertAlmostEqual(remani_v, .2296)
        self.assertGreater(self.kernel.limits['vmax'], remani_v)
        self.assertGreater(self.kernel.limits['wheel_v'], remani_v)
        state = np.zeros(17)
        reference = np.c_[np.linspace(0., 2., 101), np.zeros((101, 2)),
                          np.full(101, remani_v)]
        self.assertAlmostEqual(self.kernel.guide(state, reference)[0], remani_v)
        controls, _ = self.kernel.proposals(state, reference, np.zeros(6))
        np.testing.assert_allclose(controls[0, :, 0], remani_v, atol=1e-12)
        for sign in (-1., 1.):
            v, w = base_command([sign, 0.], self.kernel.limits)
            self.assertAlmostEqual(v, sign*.25)
            self.assertEqual(w, 0.)
        self.assertEqual(self.kernel.limits['wmax'], .35)
        self.assertEqual(self.kernel.limits['accel'], .25)
        self.assertEqual(self.kernel.limits['angular_accel'], .80)
        self.assertEqual(self.kernel.limits['wheel_a'], .2975)
        self.assertEqual(self.profile['rate_hz'], 10.)
        self.assertEqual(self.profile['arm_velocity'], [.15]*6)
        self.assertEqual(self.profile['arm_acceleration'], [.35]*6)

    def test_all_bounded_base_states_fit_dynamic_forecast_after_braking(self):
        k = self.kernel
        self.assertEqual(k.H, 5)
        h = k.limits['track']/2
        states = []
        for v in np.linspace(-k.limits['vmax'], k.limits['vmax'], 17):
            for w in np.linspace(-k.limits['wmax'], k.limits['wmax'], 17):
                if max(abs(v-h*w), abs(v+h*w)) <= k.limits['wheel_v']+1e-12:
                    state = np.zeros(17)
                    state[3:5] = v, w
                    states.append(state)
        z = np.asarray(states)
        controls = np.zeros((len(z), 8))
        controls[:, :2] = z[:, 3:5]
        # Include steady bounded arm velocities in the stopping witness.
        z[:, 11:17] = .15
        controls[:, 2:] = .15
        for _ in range(2*k.H):
            z = k.advance(z, controls, k.dt/2)
        stopped = np.zeros(len(z), dtype=bool)
        tail_s = np.zeros(len(z))
        checked, elapsed = [], []
        for j in range(40):
            z = k.advance(z, np.zeros_like(controls), k.dt/2)
            active = ~stopped
            checked.append(z[active].copy())
            elapsed.extend([k.H*k.dt+(j+1)*k.dt/2]*int(active.sum()))
            quiet = ((np.max(np.abs(z[:, 3:5]), axis=1) < .003) &
                     (np.max(np.abs(z[:, 11:17]), axis=1) < .003))
            tail_s[quiet & ~stopped] = (j+1)*k.dt/2
            stopped |= quiet
            if stopped.all():
                break
        self.assertTrue(stopped.all())
        self.assertAlmostEqual(float(tail_s.max()), 1.10)
        max_age = .35+k.H*k.dt+float(tail_s.max())
        self.assertLessEqual(max_age, 2.)
        # Use real-sized ROS epochs: float quantization is ~0.24 microseconds.
        now = 1794440000.
        shape = (64, 64, 4)
        header = np.zeros(32)
        header[:2] = 3., 1.
        header[3:5] = now-.35, 1.
        header[5:8], header[8:11], header[11] = shape, [-1.6, -1.6, -.1], .05
        header[12:17] = 47, 1, 1, .35, 2.
        header[18:21], header[28] = [now-.35, 1., 1.], 1.
        wire = np.r_[header, np.zeros(47*7), [5., 5., 0., 0., 0., 0., 1.],
                     np.ones(int(np.prod(shape)))]
        snapshot = Snapshot.decode(wire)
        checked = np.concatenate(checked)
        times = now+np.asarray(elapsed)
        self.assertLessEqual(float(np.max(times-snapshot.header[18])), 2.+1e-6)
        gaps, _ = snapshot.query(k.spheres(checked), times)
        self.assertTrue((gaps >= 0.).all(), 'Bounded braking must stay inside the dynamic forecast')
        outside, _ = snapshot.query(k.spheres(checked[:1]),
                                    np.array([snapshot.header[18]+2.05]))
        self.assertLess(outside[0], -100., 'Forecast overrun must still reject')


if __name__ == '__main__':
    unittest.main()

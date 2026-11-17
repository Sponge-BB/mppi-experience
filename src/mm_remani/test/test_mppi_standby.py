"""Offline batch-invariance checks; no ROS imports, nodes, or hardware.

Set MPPI_STANDBY_RECORDED_RUN to a completed run containing a full
first_dynamic_snapshot.npy to also check every real structured candidate.
"""
import csv
import json
import os
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'scripts'/'mppi'))
from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot
from standby_whole_body_mppi import StandbyWholeBodyMPPI


class ClearConfiguration:
    """Isolate temporal acceptance while retaining the real motion integrator."""
    def body_spheres_batch(self, q, base):
        spheres = np.zeros((len(q), 1, 4))
        spheres[:, 0, :2] = base[:, :2]
        spheres[:, 0, 3] = .05
        return spheres

    def configuration_gaps_batch(self, q):
        return np.ones(len(q))


class ApproachingObstacle:
    def query(self, spheres, times):
        # The hold candidate stops at .85 s; a moving arm stops at 1.25 s.
        # An obstacle arriving at 1 s exposes checking the hold's batch padding.
        gap = 1.-(np.asarray(times)-100.)
        return gap, np.zeros_like(gap)


def kernel(model, profile, lower, upper):
    return StandbyWholeBodyMPPI(model, profile['base_limits'],
        (lower, upper, profile['arm_velocity'], profile['arm_acceleration']),
        profile['postures'], horizon=profile['horizon'], step=1./profile['rate_hz'],
        random_samples=profile['random_samples'], config=profile.get('standby', {}))


class StandbyTailTests(unittest.TestCase):
    def test_hold_is_identical_alone_or_beside_a_long_braking_candidate(self):
        profile = yaml.safe_load((PACKAGE/'config'/'mppi.yaml').read_text())
        # Keep the fixed 1 s obstacle between the two candidates' stopping times.
        profile['horizon'] = 7
        planner = kernel(ClearConfiguration(), profile, np.full(6, -3.), np.full(6, 3.))
        state = np.zeros(17)
        controls = np.zeros((2, planner.H, 8))
        controls[1, :, 2] = .15
        reference = np.zeros((2, 4)); arm_reference = np.zeros(6)
        arguments = (reference, arm_reference, ApproachingObstacle(), 100.)
        alone = planner.evaluate(state, controls[:1], *arguments)
        alone_external = planner.last_external_gaps.copy()
        alone_diagnostics = planner.last_evaluation_diagnostics
        batch = planner.evaluate(state, controls, *arguments)
        batch_diagnostics = planner.last_evaluation_diagnostics

        for single, together in zip(alone, batch):
            np.testing.assert_array_equal(single[0], together[0])
        self.assertTrue(batch[1][0])
        self.assertFalse(batch[1][1])  # Keep the moving candidate's full brake check.
        np.testing.assert_array_equal(alone_external, planner.last_external_gaps[:1])
        for key, values in alone_diagnostics.items():
            self.assertEqual(values[0], batch_diagnostics[key][0], key)
        self.assertIsNone(batch_diagnostics['first_external_violation_s'][0])
        self.assertAlmostEqual(batch_diagnostics['first_external_violation_s'][1], 1.05)
        self.assertAlmostEqual(batch_diagnostics['checked_duration_s'][0], planner.H*planner.dt+.05)
        self.assertAlmostEqual(batch_diagnostics['checked_duration_s'][1], planner.H*planner.dt+.45)

        moving_alone = planner.evaluate(state, controls[1:], *arguments)
        for single, together in zip(moving_alone, batch):
            np.testing.assert_array_equal(single[0], together[1])
        self.assertFalse(moving_alone[1][0])

    @unittest.skipUnless(os.environ.get('MPPI_STANDBY_RECORDED_RUN'),
                         'Set MPPI_STANDBY_RECORDED_RUN for a recorded dynamic snapshot')
    def test_recorded_dynamic_snapshot_all_candidates_match_singletons(self):
        run = Path(os.environ['MPPI_STANDBY_RECORDED_RUN'])
        self.assertTrue((run/'result.json').is_file(), 'Run must be finished')
        profile = json.loads((run/'experiment.json').read_text())['controller_profile']
        model = PiperArmModel.from_parameters(run/'planner_params.yaml')
        model.lower[5] = max(model.lower[5], -.35)
        model.upper[5] = min(model.upper[5], .35)
        planner = kernel(model, profile, model.lower, model.upper)
        snapshot = Snapshot.decode(np.load(run/'first_dynamic_snapshot.npy', allow_pickle=False))
        self.assertTrue(snapshot.groups, 'Expected a real dynamic observation')
        now = float(snapshot.header[3])
        relative = now-json.loads((run/'runner.json').read_text())['start_ros_time']
        with (run/'state.csv').open() as stream:
            rows = csv.DictReader(stream)
            first = next(rows)
            row = min([first]+list(rows), key=lambda value: abs(float(value['t'])-relative))
        joint_names = ['joint%d' % i for i in range(1, 7)]
        fields = ['x', 'y', 'yaw', 'v', 'w']+joint_names+['dq%d' % i for i in range(1, 7)]
        state = np.array([float(row[key]) for key in fields])
        arm_reference = np.array([float(first[key]) for key in joint_names])
        reference = np.tile([float(first[key]) for key in ('x', 'y', 'yaw')]+[0.], (2, 1))
        controls, labels = planner.proposals(state, reference, arm_reference)
        arguments = (reference, arm_reference, snapshot, now)
        batch = planner.evaluate(state, controls, *arguments)
        batch_external = planner.last_external_gaps.copy()
        batch_diagnostics = planner.last_evaluation_diagnostics
        for index, label in enumerate(labels):
            with self.subTest(candidate=label):
                single = planner.evaluate(state, controls[index:index+1], *arguments)
                for actual, expected in zip(single, batch):
                    np.testing.assert_array_equal(actual[0], expected[index])
                self.assertEqual(planner.last_external_gaps[0], batch_external[index])
                for key, values in planner.last_evaluation_diagnostics.items():
                    self.assertEqual(values[0], batch_diagnostics[key][index], key)


if __name__ == '__main__':
    unittest.main()

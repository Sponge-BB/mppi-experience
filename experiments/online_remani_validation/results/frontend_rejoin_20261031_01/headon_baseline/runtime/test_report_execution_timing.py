"""Small report-metric checks; not additional navigation experiments."""
import unittest
from types import SimpleNamespace
import gzip
import json
from pathlib import Path
import tempfile

import numpy as np

from report_execution_timing import measured_stops, validated_actions, maneuver_events, report_batch
from test_report_native_realtime import realtime_fixture


class Metrics(unittest.TestCase):
    def test_stops_are_measured_and_clipped_not_commanded(self):
        state = np.array([(0., 0., 0.), (1., 0., .1), (2., 0., 0.), (3., .1, 0.), (4., 0., 0.)],
                         dtype=[('t', float), ('v', float), ('w', float)])
        result = measured_stops(state, .5, 3.5)
        self.assertAlmostEqual(result['total_s'], 1.5)
        self.assertAlmostEqual(result['longest_s'], 1.)

    def test_proposal_is_not_nominal_or_physical_motion(self):
        common = dict(reference_points=5, validation_code=0, validated_proposal=[.2, 0.])
        rows = [dict(common, authorized=False, reason='deadline_miss', applied_v=0., applied_w=0.),
                dict(common, authorized=True, reason='ok', applied_v=.2, applied_w=0.),
                dict(reference_points=5, validation_code=0, nominal_v=.3)]
        result = validated_actions(rows)
        self.assertEqual(result['validated_nonzero_publication_ratio'], .5)
        self.assertEqual(result['proposal_missing_cycles'], 1)

    def test_first_sideways_event_and_missing_refresh(self):
        state = np.array([(0., 0.), (1., .01), (2., .07), (3., .1)], dtype=[('t', float), ('y', float)])
        trial = SimpleNamespace(state=state, end=3., refs=[dict(t=.1, revision=1), dict(t=2.5, revision=2)])
        value = maneuver_events(trial, 1.)
        self.assertEqual(value['first_actual_sideways_t'], 2.)
        self.assertEqual(value['sideways_minus_refresh_s'], -.5)
        self.assertTrue(value['sideways_before_refresh'])
        trial.refs = trial.refs[:1]
        missing = maneuver_events(trial, 1.)
        self.assertIsNone(missing['sideways_before_refresh'])
        self.assertTrue(missing['sideways_without_refresh'])

    def test_completed_pair_report_uses_new_manifest_and_preserves_sources(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            batch = root/'batch'
            batch.mkdir()
            (batch/'plan.json').write_text(json.dumps(dict(seed=11)))
            for group, deadline, lease in [('current', .05, .075), ('matched', .2, .25)]:
                trial = batch/('dynamic_head_on_center_%s_seed11' % group)
                realtime_fixture(trial, True)
                manifest = json.loads((trial/'manifest.json').read_text())
                manifest.update(scene='dynamic_head_on_center', native_dynamic_fast=dict(enabled=True),
                    execution_timing_pair=dict(group=group, callback_deadline_s=deadline, command_lease_s=lease))
                (trial/'manifest.json').write_text(json.dumps(manifest))
                (trial/'state.csv').write_text('t,x,y,yaw,v,w\n0,0,0,0,0,0\n1,.1,0,0,.1,0\n2,.2,0,0,.1,0\n')
                with gzip.open(trial/'mppi_diagnostics.json.gz', 'wt') as stream:
                    json.dump([dict(t=1., reference_points=5, code=0, reason='deadline_miss',
                        validation_code=0, validated_proposal=[.2, 0.],
                        authorized=False, applied_v=0., applied_w=0., callback_ms=80., kernel_ms=70.)], stream)
            with self.assertRaisesRegex(ValueError, 'finish both'):
                report_batch(batch, root/'blocked', render=False)
            (batch/'complete.json').write_text('{}')
            value = report_batch(batch, root/'report', render=False)
            self.assertEqual(len(value['trials']), 2)
            self.assertEqual(value['trials'][0]['validated_actions']['validated_nonzero_publication_ratio'], 0.)
            self.assertTrue((root/'report/comparison.png').is_file())
            self.assertIn('不是Gazebo或实车', (root/'report/实验总结.txt').read_text())


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Read-only runner contracts, no ROS launch or physical publication."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import run_feasibility_stop as runner


class RunnerContracts(unittest.TestCase):
    def test_predeclared_jobs_and_phases(self):
        self.assertEqual(runner.JOBS, ({'seed': 11, 'ordinal': 2}, {'seed': 12, 'ordinal': 2}))
        self.assertEqual(runner.PHASES,
            ('delivery_nominal_060', 'delivery_stop_on_060', 'delivery_repeat_060'))

    def test_environment_isolated_and_no_new_objective(self):
        result = runner.environment(Path('/tmp/stop_gate_fixture'), 11, 2, inherited={})
        self.assertEqual(result['ROS_MASTER_URI'], 'http://127.0.0.1:11441')
        self.assertEqual(result['FEASIBILITY_STOP_GATE'], '1')
        for key in ('COUPLING_EXEC_WEIGHT', 'COUPLING_HAND_WEIGHT',
                    'COUPLING_FEASIBILITY_RECOVERY', 'COLLAB_PREFIX', 'COLLAB_ROLLING'):
            self.assertEqual(result[key], '0')
        self.assertEqual(result['SEED_PARAMETERIZATION_FIX'], '1')
        self.assertEqual(result['DELIVERY_REPLAY_REFRESH_ORDINAL'], '2')

    def test_sensitive_inherited_values_are_not_copied(self):
        result = runner.environment(Path('/tmp/stop_gate_fixture'), 12, 2,
            inherited={'EXAMPLE_SECRET': 'dummy', 'COUPLING_EXEC_WEIGHT': '99'})
        self.assertNotIn('EXAMPLE_SECRET', result)
        self.assertEqual(result['COUPLING_EXEC_WEIGHT'], '0')

    def test_prepare_changes_only_metadata(self):
        with tempfile.TemporaryDirectory(prefix='feasibility_stop_contract_') as tmp:
            out = Path(tmp)
            def parent_prepare(path, build, seed, ordinal):
                (path/'coupling_parameters.json').write_text(json.dumps({'scene': 'static'}))
                (path/'manifest.json').write_text(json.dumps({'build': str(build)}))
                return {'helper.py': 'frozen'}
            with patch.object(runner, 'ORIGINAL_PREPARE', side_effect=parent_prepare):
                got = runner.prepare(out, Path('/tmp/immutable_build'), 11, 2)
            self.assertEqual(got, {'helper.py': 'frozen'})
            parameters = json.loads((out/'coupling_parameters.json').read_text())
            manifest = json.loads((out/'manifest.json').read_text())
            self.assertEqual(parameters['diagnostic_budgets_s'], [.6, .6, .6])
            self.assertEqual(parameters['diagnostic_treatment_order'], ['OFF', 'ON', 'OFF'])
            self.assertFalse(parameters['candidate_publication'])
            self.assertEqual(manifest['parameters_sha256'], runner.sha(out/'coupling_parameters.json'))
            self.assertFalse(manifest['feasibility_stop_environment']['normal_navigation_enabled'])


if __name__ == '__main__':
    unittest.main()

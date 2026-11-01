#!/usr/bin/env python3
"""Runner contracts; these tests do not launch ROS or publish commands."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import run_budget_scaling as runner


class RunnerContracts(unittest.TestCase):
    def test_frozen_order_and_one_factor_contrasts(self):
        self.assertEqual(tuple((j['seed'],j['ordinal']) for j in runner.JOBS),
                         ((11,1),(12,2),(11,2),(12,1)))
        self.assertEqual(runner.PHASES, ('budget_original_060','budget_reference_060',
            'budget_extended_300','budget_scaled_300','budget_repeat_060'))
        self.assertEqual(runner.BUDGETS, (.6,.6,3.,3.,.6))

    def test_isolated_environment_no_objective_or_unsafe_switch_leak(self):
        result=runner.environment(Path('/tmp/budget_scaling_fixture'),11,1,
            inherited={'BUDGET_OVERRIDE':'bad','FEASIBILITY_OTHER':'bad',
                       'SOLVER_SCALING_X':'bad','COUPLING_EXEC_WEIGHT':'999',
                       'EXAMPLE_SECRET':'dummy','DELIVERY_REPLAY_KIND':'bad'})
        for key in ('BUDGET_OVERRIDE','FEASIBILITY_OTHER','SOLVER_SCALING_X','EXAMPLE_SECRET'):
            self.assertNotIn(key,result)
        self.assertEqual(result['ROS_MASTER_URI'],'http://127.0.0.1:11441')
        self.assertEqual(result['FEASIBILITY_STOP_GATE'],'1')
        self.assertEqual(result['DELIVERY_REPLAY_KIND'],'first_refresh')
        for key in ('COUPLING_EXEC_WEIGHT','COUPLING_HAND_WEIGHT',
                    'COUPLING_FEASIBILITY_RECOVERY','COLLAB_PREFIX','COLLAB_ROLLING'):
            self.assertEqual(result[key],'0')

    def test_prepare_only_metadata_and_hash(self):
        with tempfile.TemporaryDirectory(prefix='budget_runner_contract_') as tmp:
            out=Path(tmp)
            def parent(path,build,seed,ordinal):
                (path/'coupling_parameters.json').write_text(json.dumps({'scene':'static'}))
                (path/'manifest.json').write_text(json.dumps({'build':str(build)}))
                return {'runtime.py':'frozen'}
            with patch.object(runner,'ORIGINAL_PREPARE',side_effect=parent):
                runtime=runner.prepare(out,Path('/tmp/frozen_build'),12,2)
            params=json.loads((out/'coupling_parameters.json').read_text())
            manifest=json.loads((out/'manifest.json').read_text())
            self.assertEqual(runtime,{'runtime.py':'frozen'})
            self.assertEqual(params['diagnostic_budgets_s'],list(runner.BUDGETS))
            self.assertEqual(params['diagnostic_phases'],list(runner.PHASES))
            self.assertTrue(params['objective_unchanged'])
            self.assertTrue(params['initialization_unchanged'])
            self.assertTrue(params['constraints_unchanged'])
            self.assertTrue(params['scaling_preparation_inside_budget'])
            self.assertFalse(params['candidate_publication'])
            self.assertFalse(params['navigation_metrics_valid'])
            self.assertEqual(manifest['parameters_sha256'],runner.sha(out/'coupling_parameters.json'))
            self.assertFalse(manifest['feasibility_stop_environment']['normal_navigation_enabled'])


if __name__=='__main__':
    unittest.main()

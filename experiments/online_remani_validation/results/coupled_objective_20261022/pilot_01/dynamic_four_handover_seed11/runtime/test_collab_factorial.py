#!/usr/bin/env python3
import ast
from pathlib import Path
import tempfile
import unittest

from build_collab_factorial import transform_fsm, transform_manager
from run_collab_factorial import GROUPS, patch_runtime
from collab_seed_export import goal_token

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/dual_timeline_v12_01/source'


class FactorialContracts(unittest.TestCase):
    def test_four_independent_factors(self):
        self.assertEqual(GROUPS,{'A':(False,False),'B':(False,True),'C':(True,False),'D':(True,True)})

    def test_transform_keeps_safety_and_retry(self):
        source=(BASE/'remani_replan_fsm.cpp').read_text();changed=transform_fsm(source)
        for anchor in ('next_cooperative_attempt_=ros::Time::now()+ros::Duration(.75);',
                       'bool refreshed=planFromLocalTraj(true,false,dynamic_astar_replan_wall_time_,false);',
                       'beginDynamicPointPlanningSnapshot(', 'checkPredictedDynamicObstacleCollision('):
            self.assertEqual(source.count(anchor),changed.count(anchor))
        self.assertIn('collab::pairedOnce',changed)

    def test_manager_no_budget_or_final_acceptance_relaxation(self):
        source=(BASE/'planner_manager.cpp').read_text();changed=transform_manager(source)
        self.assertIn('collab::prefixSearch',changed)
        for anchor in ('OptimizeTrajectory_lbfgs(', 'if(!flag_success)', 'trajectory_start_time'):
            self.assertEqual(source.count(anchor),changed.count(anchor))
        self.assertNotIn('setMaxWallTime(',changed)

    def test_runtime_feedback_after_command_and_validation(self):
        import json, shutil
        with tempfile.TemporaryDirectory(prefix='collab_contract_') as temporary:
            root=Path(temporary);(root/'runtime').mkdir()
            for name in ('local_execution.py','mppi_node.py','search_recording.py'):
                shutil.copy2(HERE/name,root/'runtime'/name)
            (root/'manifest.json').write_text(json.dumps({'experiment_sources_sha256':{}}))
            patch_runtime(root,7)
            for path in (root/'runtime').glob('*.py'):ast.parse(path.read_text())
            text=(root/'runtime/local_execution.py').read_text()
            self.assertLess(text.index('node.local_cmd_pub.publish(issued)'),text.index('publish_seed(node,'))
            self.assertIn('if reason!=\'ok\':result[:2]=0;validated_count=0',text)

    def test_task_token_not_python_random_hash(self):
        self.assertEqual(goal_token(''),1469598103934665603 & 0xffffffff)
        self.assertEqual(goal_token('abc'),goal_token('abc'))
        self.assertNotEqual(goal_token('abc'),goal_token('abd'))


if __name__=='__main__':unittest.main()

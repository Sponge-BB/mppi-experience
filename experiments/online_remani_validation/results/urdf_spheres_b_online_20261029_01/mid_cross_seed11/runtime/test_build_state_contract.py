from pathlib import Path
import unittest

import build_state_contract as builder

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/domain_recovery_20261023_01/source'


class StateBuilderContracts(unittest.TestCase):
    def test_fsm_keeps_original_floor_inside_disabled_branch(self):
        original=(BASE/'remani_replan_fsm.cpp').read_text()
        text=builder.transform_fsm(original)
        begin=original.index('    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){')
        end=original.index('\n\n    mm_car_yaw_rate_',begin)
        self.assertIn(original[begin:end],text)
        self.assertEqual(text.count('state_contract::signedRegularization('),1)
        self.assertIn('true,0); // No history/hysteresis.',text)
        self.assertLess(text.index('dual_probe::Request dual_request('),
                        text.index('state_contract_trace::Request state_contract_request'))
        self.assertLess(text.index('state_contract::receive('),text.index('mm_state_pos_(0) = msg->pose.pose.position.x;'))
        for gate in ('projectedCandidate(', 'late_moving_handoff', 'waitForRequiredESDF(',
                     'checkCollision(', 'checkDynamicObstacleCollision('):
            self.assertEqual(text.count(gate),original.count(gate))

    def test_optimizer_objective_and_original_clock_unchanged(self):
        original=(BASE/'poly_traj_optimizer.cpp').read_text()
        text=builder.transform_optimizer(original)
        for key in ('optimization_start_wall_time_ = ros::WallTime::now();',
                    'max_wall_time_', 'costFunctionCallback(', 'domain_lbfgs::lbfgs_optimize(',
                    'IsTrajSafe(', 'coupling_native::addGeometry(', 'coupling_bridge::addPrediction('):
            self.assertEqual(text.count(key),original.count(key))
        self.assertNotIn('clearance_native',text)
        self.assertLess(text.index('coupling_geometry::Scope coupled_geometry;'),
                        text.index('state_contract_trace::BackendRecord'))
        self.assertLess(text.index('state_contract_trace::mincoHead('),
                        text.index('frontendMJ_container[i].reset(headState'))

    def test_manager_queues_after_return_but_never_flushes_before_delivery(self):
        original=(BASE/'planner_manager.cpp').read_text()
        text=builder.transform_manager(original)
        call=text.index('flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(')
        returned=text.index('"state_contract_solver_return"',call)
        self.assertLess(call,returned)
        self.assertNotIn('state_contract_trace::flush();',text)
        for key in ('ros::Duration(delta_replan_time).sleep();',
                    'traj_start_time = trajectory_start_time;',
                    'traj_start_time = ros::Time::now().toSec();'):
            self.assertEqual(text.count(key),original.count(key))

    def test_unknown_source_fails_not_silent_hook_omission(self):
        for function in (builder.transform_fsm,builder.transform_manager,
                         builder.transform_optimizer,builder.transform_search):
            with self.assertRaises((ValueError,RuntimeError)):
                function('not the frozen source')

    def test_real_search_threshold_is_observed_not_replaced(self):
        source=HERE/'builds/fixed_arm_01/source/mm_remani_planner/path_searching/src/kino_astar.cpp'
        original=source.read_text();text=builder.transform_search(original)
        self.assertIn('cur_node->singul = getSingularity(start_state_(3));\n  state_contract_trace::nativeRoot',text)
        for key in ('ros::Time::now()', 'fabs(vel) > non_siguav_', 'start_state(3) =', 'if(cur_node->singul == 0)'):
            self.assertEqual(text.count(key),original.count(key))


if __name__=='__main__':unittest.main()

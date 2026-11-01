#!/usr/bin/env python3
"""Pure source-contract tests; no compiler, ROS graph, or robot commands."""
import hashlib
from pathlib import Path
import unittest

import patch_feasibility_stop as patch

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/seed_parameterization_20261025_01/source'

class PatchTests(unittest.TestCase):
    def test_optimizer_inverse(self):
        original=(BASE/'poly_traj_optimizer.cpp').read_text()
        self.assertEqual(hashlib.sha256(original.encode()).hexdigest(),patch.BASE_OPTIMIZER_SHA256)
        modified=patch.transform_optimizer(original)
        self.assertEqual(patch.project_off(modified),original)

    def test_lbfgs_inverse(self):
        original=(BASE/'domain_lbfgs.hpp').read_text()
        self.assertEqual(hashlib.sha256(original.encode()).hexdigest(),patch.BASE_LBFGS_SHA256)
        modified=patch.transform_lbfgs(original)
        self.assertEqual(patch.project_off(modified,'lbfgs'),original)
        self.assertLess(modified.index(patch.INCLUDE),modified.index('namespace domain_lbfgs'))

    def test_fail_closed_drift_and_repeat(self):
        for filename,transform in [('poly_traj_optimizer.cpp',patch.transform_optimizer),
                                   ('domain_lbfgs.hpp',patch.transform_lbfgs)]:
            original=(BASE/filename).read_text()
            for wrong in [original+'\n',original.replace('double','float',1),transform(original)]:
                with self.assertRaises(ValueError):transform(wrong)

    def test_only_original_delta_stop_is_hooked(self):
        original=(BASE/'domain_lbfgs.hpp').read_text()
        modified=patch.transform_lbfgs(original)
        self.assertEqual(modified.count('feasibility_stop_gate::allowCostStop(x)'),1)
        self.assertEqual(modified.count('feasibility_stop_gate::abortRequested()'),1)
        self.assertIn('if (rate < param.delta)',modified)
        self.assertIn('if (feasibility_stop_gate::allowCostStop(x))',modified)
        for untouched in ['if (ls < 0)','gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon',
                          'ret = LBFGS_CANCELED;','pf(k % param.past) = fx;']:
            self.assertIn(untouched,modified)

    def test_diagnostic_scope_and_fixed_budget(self):
        self.assertIn('delivery_replay::running() && delivery_replay::phase()=="delivery_stop_on_060"',patch.SESSION)
        self.assertIn('std::string(feasibility_switch)=="1"',patch.SESSION)
        self.assertIn('force_stop_type_=STOP_FOR_TIMEOUT',patch.SESSION)
        self.assertNotIn('optimization_start_wall_time_ =',patch.SESSION)
        self.assertNotIn('max_wall_time_=',patch.SESSION)
        self.assertNotIn('clearance_native',patch.SESSION)
        self.assertIn('if(!delivery_replay::running())return;',patch.SESSION)
        modified=patch.transform_optimizer((BASE/'poly_traj_optimizer.cpp').read_text())
        self.assertIn('const double budgets[]={.60,.60,.60};',modified)
        self.assertIn('"delivery_nominal_060","delivery_stop_on_060","delivery_repeat_060"',modified)

    def test_actual_native_and_static_projection_gates(self):
        self.assertIn('delivery_replay::RestoreFailure restore;',patch.SESSION)
        self.assertIn('check.native_pass=IsTrajSafe(candidate)',patch.SESSION)
        self.assertIn('!shared_collision::fresh(*frame,frame_now)',patch.SESSION)
        self.assertIn('!frame->obstacles.empty()',patch.SESSION)
        original=(BASE/'coupling_bridge.hpp').read_text()
        for exact in ['const int steps=std::max(1,int(std::ceil(part.duration/.02)));',
                      'for(int k=0;k<=steps;++k){const double q=part.duration*k/steps;',
                      'auto p=part.traj.getPos(q);auto v=part.traj.getVel(q);',
                      'const double yaw=std::atan2(part.singul*v(1),part.singul*v(0));Eigen::Vector3d pose(p(0),p(1),yaw);']:
            self.assertIn(exact,original)
            self.assertIn(exact,patch.SESSION)
        self.assertIn('s.gap<-1e-8',patch.SESSION)
        self.assertIn('staticProjectedPose(*frame,pose)',patch.SESSION)
        self.assertLess(patch.SESSION.index('if(!check.native_pass)'),
                        patch.SESSION.index('const double projection_start'))

    def test_callback_cache_record_after_finite_check(self):
        modified=patch.transform_optimizer((BASE/'poly_traj_optimizer.cpp').read_text())
        callback=modified[modified.index('  double PolyTrajOptimizer::costFunctionCallback'):]
        self.assertLess(callback.index('if(non_finite)'),callback.index('recordEvaluated(x)'))
        self.assertLess(callback.index('recordEvaluated(x)'),callback.index('return costall;'))
        header=(HERE/'feasibility_stop_gate.hpp').read_text()
        self.assertIn('vectorHash(x)!=s->last_hash',header)
        self.assertIn('std::memcmp(x.data(),s->last_evaluated.data()',header)
        self.assertIn('if(!s)return true;',header)
        self.assertIn('throw std::runtime_error("feasibility stop accepted-x/cache mismatch")',header)
        self.assertIn('s->stats.last.budget_expired',header)

    def test_common_final_deadline_and_summary(self):
        modified=patch.transform_optimizer((BASE/'poly_traj_optimizer.cpp').read_text())
        self.assertIn('if(delivery_replay::running() &&',patch.FINAL_DEADLINE)
        self.assertNotIn('feasibility_enabled',patch.FINAL_DEADLINE)
        self.assertIn('diagnosis.complete=false;probe.success=false;return false;',patch.FINAL_DEADLINE)
        self.assertLess(modified.index('optT_container.push_back(SnapOpt_container_[i].get_T1());'),
                        modified.index('original_deadline_exhausted_at_return'))
        self.assertLess(modified.index('original_deadline_exhausted_at_return'),
                        modified.index('diagnosis.complete=good_traj;'))
        self.assertIn('feasibility_decision_elapsed_s',patch.SESSION)
        self.assertIn('original_budget_s',patch.SESSION)

    def test_no_objective_or_safety_gate_changes(self):
        original=(BASE/'poly_traj_optimizer.cpp').read_text()
        modified=patch.transform_optimizer(original)
        for exact in ['double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost + extra_geometry + extra_prediction;',
                      'good_traj = IsTrajSafe(singul_traj_data);',
                      'lbfgs_params.delta = 1e-3;',
                      'optimization_start_wall_time_ = ros::WallTime::now();']:
            self.assertEqual(modified.count(exact),original.count(exact))
        for name in ['max_wheel_omega_','max_wheel_alpha_','mobile_base_wheel_radius_','iniState_container_']:
            self.assertEqual(modified.count(name),original.count(name))

if __name__=='__main__':
    unittest.main()

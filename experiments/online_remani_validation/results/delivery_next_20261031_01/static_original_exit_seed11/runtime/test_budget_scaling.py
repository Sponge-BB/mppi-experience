#!/usr/bin/env python3
"""Exact source contracts; actual C++ numerical contracts are in the .cpp test."""
from pathlib import Path
import unittest
import patch_budget_scaling as patch

HERE=Path(__file__).resolve().parent
PARENT=HERE/'builds/feasibility_stop_20261026_01/source'

class PatchTests(unittest.TestCase):
    def test_exact_inverse(self):
        for name,kind,transform in [('poly_traj_optimizer.cpp','optimizer',patch.transform_optimizer),
                                    ('delivery_replay_probe.hpp','replay',patch.transform_replay)]:
            original=(PARENT/name).read_text();modified=transform(original)
            self.assertEqual(patch.project_off(modified,kind),original)

    def test_drift_repeat_fail_closed(self):
        for name,transform in [('poly_traj_optimizer.cpp',patch.transform_optimizer),
                               ('delivery_replay_probe.hpp',patch.transform_replay)]:
            original=(PARENT/name).read_text()
            for text in (original+'\n',original.replace('double','float',1),transform(original)):
                with self.assertRaises(ValueError):transform(text)

    def test_five_fixed_phases(self):
        modified=patch.transform_optimizer((PARENT/'poly_traj_optimizer.cpp').read_text())
        self.assertEqual(patch.BUDGETS,(.6,.6,3.,3.,.6))
        self.assertIn('const double budgets[]={.60,.60,3.00,3.00,.60};',modified)
        self.assertIn('"'+ '","'.join(patch.PHASES) +'"',modified)
        self.assertIn('for(size_t i=0;i<5;++i){',modified)
        replay=patch.transform_replay((PARENT/'delivery_replay_probe.hpp').read_text())
        self.assertIn('if(s.size()!=5)return false;',replay)

    def test_off_direct_call_unchanged(self):
        self.assertEqual(patch.SCALED_CALL.count(patch.ORIGINAL_CALL),1)
        self.assertIn('}else{\n'+patch.ORIGINAL_CALL,patch.SCALED_CALL)
        self.assertIn('delivery_replay::running() &&',patch.SCALED_CALL)
        self.assertIn('budget_scaling::scaledPhase(delivery_replay::phase())',patch.SCALED_CALL)

    def test_objective_limits_and_native_gate_unchanged(self):
        original=(PARENT/'poly_traj_optimizer.cpp').read_text()
        modified=patch.transform_optimizer(original)
        for anchor in ['double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost + extra_geometry + extra_prediction;',
                       'lbfgs_params.delta = 1e-3;', 'lbfgs_params.g_epsilon = 0.0;']:
            self.assertEqual(modified.count(anchor),original.count(anchor))
        # Full native hard checker is byte-for-byte untouched.
        start=original.index('  bool PolyTrajOptimizer::IsTrajSafe(')
        stop=original.index('  double PolyTrajOptimizer::costFunctionCallback',start)
        self.assertIn(original[start:stop],modified)
        self.assertNotIn('max_wheel_omega_=',patch.SCALED_CALL)
        self.assertNotIn('max_wheel_alpha_=',patch.SCALED_CALL)

    def test_common_final_gate_and_all_return_marker(self):
        modified=patch.transform_optimizer((PARENT/'poly_traj_optimizer.cpp').read_text())
        self.assertIn('budget_scaling::OnExit budget_final_report(delivery_replay::running()',modified)
        self.assertLess(modified.index('budget_scaling::OnExit'),modified.index('result = domain_lbfgs::lbfgs_optimize('))
        for field in ['budget_final_validation_v1','solver_result_known','native_known','projected_known',
                      'elapsed_s','budget_s','final_x_hash','last_canonical_x_hash','last_accepted_x_hash',
                      'cache_identity_match','output_hash','output_constructed','allowed','success']:
            self.assertIn(field,patch.FINAL_SCOPE)
        self.assertIn('validation_schema',patch.FINAL_SCOPE)
        self.assertNotIn(r'\"schema\":',patch.FINAL_SCOPE)
        for anchor in ['std::ceil(part.duration/.02)','sample.gap<-1e-8',
                       'shared_collision::fresh(*frame,epoch)','!frame->obstacles.empty()']:
            self.assertIn(anchor,patch.FINAL_PROJECTION)
        self.assertIn('budget_final.decision_elapsed_s>=max_wall_time_',patch.FINAL_SUCCESS)
        self.assertIn('force_stop_type_!=DONT_STOP',patch.FINAL_SUCCESS)
        self.assertLess(modified.index('resource_probe::hashMatrices(budget_final.output_hash,optT_container)'),
                        modified.index('original_deadline_exhausted_at_return'))

    def test_native_cache_and_progress_canonical(self):
        modified=patch.transform_optimizer((PARENT/'poly_traj_optimizer.cpp').read_text())
        callback=modified[modified.index('  double PolyTrajOptimizer::costFunctionCallback'):]
        self.assertLess(callback.index('if(non_finite)'),callback.index('budget_scaling::recordCanonical(x)'))
        self.assertLess(callback.index('budget_scaling::recordCanonical(x)'),callback.index('feasibility_stop_gate::recordEvaluated(x)'))
        self.assertIn('if(opt->force_stop_type_==DONT_STOP)budget_scaling::recordAccepted(x);',callback)
        header=(HERE/'budget_scaling.hpp').read_text()
        self.assertIn('c.native_progress(c.native,c.last_x,c.last_g,cost,step,k,ls)',header)
        self.assertIn('if(z[i]!=0.)x[i]+=diagonal[i]*z[i];',header)

    def test_only_input_times_determine_scale(self):
        header=(HERE/'budget_scaling.hpp').read_text()
        start=header.index('inline Plan makePlan');stop=header.index('struct Context',start)
        plan=header[start:stop]
        self.assertIn('7.*',plan)
        for forbidden in ['gradient','costFunction','snapshot','native_evaluate','max_wheel','weight']:
            self.assertNotIn(forbidden,plan)

if __name__=='__main__':unittest.main()

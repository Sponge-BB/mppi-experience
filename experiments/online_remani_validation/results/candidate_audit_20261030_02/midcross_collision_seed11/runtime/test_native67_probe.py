import unittest
from pathlib import Path
from patch_native67_probe import transform_optimizer, transform_replay, transform_lbfgs

BASE=Path(__file__).resolve().parent/'builds/urdf_sphere_timing_20261029_01/source'


class Native67ProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.before=(BASE/'poly_traj_optimizer.cpp').read_text()
        cls.after=transform_optimizer(cls.before)

    def test_no_historical_extra_gate_or_scaling(self):
        for token in ('budget_final.', 'feasibility_stop_gate::Session',
                      'budget_scaling::Context', 'staticProjectedPose(',
                      'final_qualification_budget_timeout', 'final_native_cache_mismatch'):
            self.assertNotIn(token,self.after)
        self.assertIn('const double budgets[]={.60,3.00};',self.after)
        self.assertIn('native67_eligible && delivery_replay::requested',self.after)
        self.assertIn('!native67_context->dynamic_repair',self.after)
        self.assertIn('return false; // Private captured candidates NEVER escape',self.after)

    def test_objective_algebra_unchanged(self):
        start='  double PolyTrajOptimizer::costFunctionCallback('
        end='  int PolyTrajOptimizer::earlyExitCallback('
        old=self.before[self.before.index(start):self.before.index(end)]
        new=self.after[self.after.index(start):self.after.index(end)]
        new=new.replace('    native67_probe::Callback native67_callback;\n','')
        new=new.replace('    native67_probe::completed(total_smoo_cost,total_obs_feas_costs,total_time_cost);\n','')
        new=new.replace('    // native67: no diagnostic stop-veto cache.\n',
            '    budget_scaling::recordCanonical(x);\n    feasibility_stop_gate::recordEvaluated(x);\n')
        self.assertEqual(old,new)

    def test_native_hard_gate_and_wheel_formula_unchanged(self):
        start='  bool PolyTrajOptimizer::IsNotFeasibie('
        end='  /* callbacks by the L-BFGS optimizer */'
        old=self.before[self.before.index(start):self.before.index(end)]
        new=self.after[self.after.index(start):self.after.index(end)]
        self.assertEqual(old,new)

    def test_exact_line_search_counter_without_search_change(self):
        old=(BASE/'domain_lbfgs.hpp').read_text();new=transform_lbfgs(old)
        new=new.replace('#include "native67_probe.hpp"\n','',1)
        new=new.replace('                native67_probe::trial();\n','',1)
        self.assertEqual(old,new)

    def test_two_signatures_and_bounded_post_solver_audits(self):
        new=transform_replay((BASE/'delivery_replay_probe.hpp').read_text())
        self.assertIn('if(s.size()!=2)return false;',new)
        self.assertIn('accepted.reserve(3)',new)
        self.assertNotIn('accepted.size()>=4096',new)
        self.assertIn('"solver_return_x"',new)
        self.assertIn('native67_probe::active()=nullptr;',new)
        self.assertIn('if(auto* f=failure_probe::active())f->profile_enabled=true;',new)
        enable_at=new.index('if(auto* f=failure_probe::active())f->profile_enabled=true;')
        self.assertLess(new.index('if(!enabled)return;active()=this;'),enable_at)


if __name__=='__main__':unittest.main()

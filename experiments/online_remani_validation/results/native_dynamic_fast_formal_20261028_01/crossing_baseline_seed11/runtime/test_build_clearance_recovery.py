from pathlib import Path
import unittest
from build_clearance_recovery import transform_optimizer

HERE=Path(__file__).resolve().parent


class ClearanceBuilder(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=(HERE/'builds/domain_recovery_20261023_01/source/poly_traj_optimizer.cpp').read_text()
        cls.text=transform_optimizer(cls.original)

    def test_one_original_clock_and_one_restart(self):
        s=self.text
        self.assertEqual(s.count('optimization_start_wall_time_ = ros::WallTime::now();'),1)
        self.assertEqual(s.count('clearance_window.window.claim('),1)
        self.assertEqual(s.count('clearance_native::prepareMultipliers('),1)
        self.assertEqual(s.count('domain_lbfgs::lbfgs_optimize('),2)
        self.assertEqual(s.count('max_wall_time_ ='),self.original.count('max_wall_time_ ='))

    def test_gradient_enters_before_original_minco_chain(self):
        s=self.text
        self.assertLess(s.index('extra_clearance=clearance_native::addPenalty'),s.index('getGrad2TP(gradT_container'))
        self.assertIn(' + extra_geometry + extra_prediction + extra_clearance;',s)
        self.assertIn('gp.bottomRows(opt->manipulator_dof_).setZero();',s)

    def test_no_consumption_after_bad_restart_and_cache_regenerated(self):
        s=self.text
        bad=s.index('if(result<0 || force_stop_type_!=DONT_STOP)')
        regen=s.index('final_cost=PolyTrajOptimizer::costFunctionCallback(this,x,coherent_gradient);')
        native=s.index('good_traj = IsTrajSafe(singul_traj_data);')
        self.assertLess(bad,regen);self.assertLess(regen,native)
        self.assertIn('return false;',s[bad:regen])
        self.assertIn('if(clearance_control::stopRecovery())',s)
        self.assertIn('if(clearance_control::stopRecovery() ||',s)

    def test_strict_budget_gate_before_success_and_margin_preserved(self):
        s=self.text
        gate=s.index('"original_budget_exhausted_in_native_validation"')
        self.assertLess(gate,s.index('    probe.success=good_traj;'))
        self.assertIn('restored_clearance.gap < -1e-8',s)
        self.assertIn('nominal_clearance.gap < -1e-8',s)
        self.assertIn('coupling_geometry::enabled()',s)

    def test_unreviewed_source_drift_fails(self):
        with self.assertRaises(RuntimeError):transform_optimizer('not the frozen optimizer')


if __name__=='__main__':unittest.main()

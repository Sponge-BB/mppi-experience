from pathlib import Path
import unittest
from build_domain_recovery import transform_optimizer

HERE=Path(__file__).resolve().parent


class BuilderContracts(unittest.TestCase):
    def test_actual_frozen_optimizer_transforms_fail_closed(self):
        text=transform_optimizer((HERE/'builds/delivery_diagnostics_20261022_01/source/poly_traj_optimizer.cpp').read_text())
        self.assertIn('domain_lbfgs::lbfgs_optimize',text)
        self.assertNotIn('= lbfgs::lbfgs_optimize',text)
        self.assertEqual(text.count('domain_step::reset();'),1)
        self.assertIn('"domain_initial_reject"',text)
        self.assertIn('"domain_solver_status"',text)
        self.assertLess(text.index('if(result<0){'),text.index('    // test collision'))
        self.assertLess(text.index('if(result<0){'),text.index('singul_traj_data.addSingulTraj(SnapOpt_container_'))
        typed=text.index('    }catch(const domain_step::RejectedTrial&){')
        fatal=text.index('    }catch(const std::exception& error){')
        self.assertLess(typed,fatal)
        self.assertIn('throw;',text[typed:fatal])

    def test_unreviewed_source_drift_is_rejected(self):
        with self.assertRaises(RuntimeError):
            transform_optimizer('not the frozen optimizer')


if __name__=='__main__':
    unittest.main()

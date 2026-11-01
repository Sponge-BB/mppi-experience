#!/usr/bin/env python3
import unittest
from pathlib import Path
from patch_delivery_candidate import transform_optimizer

HERE = Path(__file__).resolve().parent


class CandidatePatchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original = (HERE / 'builds/native67_probe_20261030_02/source/poly_traj_optimizer.cpp').read_text()
        cls.patched = transform_optimizer(cls.original)

    def test_native_gate_once_and_no_relaxed_constraints(self):
        self.assertEqual(self.patched.count('good_traj = IsTrajSafe(singul_traj_data);'), 1)
        native_begin = self.original.index('  bool PolyTrajOptimizer::IsNotFeasibie(')
        native_end = self.original.index('  /* callbacks by the L-BFGS optimizer */')
        self.assertIn(self.original[native_begin:native_end], self.patched)

    def test_timeout_only_and_rebuild_cache(self):
        self.assertIn('if(result<0&&!salvage_attempted)', self.patched)
        self.assertIn('if(!accepted_candidate.usable())return false;', self.patched)
        self.assertIn('cps_container_[i].points=SnapOpt_container_[i].getInitConstrainPoints(cps_num_prePiece_);', self.patched)
        self.assertEqual(self.patched.count('delivery_candidate::fail();'), 3)
        self.assertNotIn('DELIVERY_REPLAY_LOG', self.patched[self.patched.index('delivery_candidate::Scope accepted_candidate;'):])

    def test_single_application(self):
        with self.assertRaises((ValueError, AssertionError)):
            transform_optimizer(self.patched)


if __name__ == '__main__':
    unittest.main()

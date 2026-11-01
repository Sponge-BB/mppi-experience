#!/usr/bin/env python3
"""Source invariants only; no compiler or ROS invocation."""
from pathlib import Path
import unittest
from patch_delivery_replay import transform_optimizer

HERE = Path(__file__).resolve().parent
PARENT = HERE / 'builds/boundary_admission_20261024_02/source/poly_traj_optimizer.cpp'


class PatchTests(unittest.TestCase):
    def setUp(self):
        self.source = PARENT.read_text()
        self.result = transform_optimizer(self.source)

    def test_selected_entry_precedes_nominal_diagnosis(self):
        self.assertLess(self.result.index('failure_probe::PauseOwned delivery_paused'),
                        self.result.index('failure_probe::Frame diagnosis('))
        self.assertIn('const double budgets[]={.60,.60,1.50};', self.result)
        self.assertIn('raw==grid_map_->getDynamicObservationSeq()', self.result)
        self.assertIn('coupling_geometry::activeFrame().snapshot==frozen_frame.snapshot', self.result)
        self.assertIn('delivery_replay::predictionText()==frozen_prediction', self.result)

    def test_no_diagnostic_output_published(self):
        self.assertIn('return false; // Private captured candidates NEVER escape', self.result)
        self.assertNotIn('const std::vector<double> budgets={.60,3.0', self.result)
        self.assertNotIn('max_wheel_omega_=3.50;', self.result)
        self.assertNotIn('for(double limit:{3.28,3.50})', self.result)

    def test_progress_is_canonical_and_rebuilt(self):
        self.assertIn('delivery_replay::progress(x,g,fx,step,k,ls);', self.result)
        self.assertIn('delivery_replay::first(costall,x,g);', self.result)
        self.assertIn('SnapOpt_container_[n].generate(points[n],times[n],ini,fin);', self.result)
        self.assertIn('native_IsTrajSafe_pass', self.result)
        self.assertIn('diagnosis_now->reason=saved_reason;', self.result)
        self.assertIn('delivery_replay::RestoreFailure restore_native_verdict;', self.result)

    def test_ordinal_is_not_outcome_selected(self):
        header = (HERE / 'delivery_replay_probe.hpp').read_text()
        self.assertIn('DELIVERY_REPLAY_REFRESH_ORDINAL', header)
        self.assertIn('const auto current=++refreshOrdinal();', header)
        selection = header[header.index('inline bool requested('):header.index('inline std::string num(')]
        self.assertNotIn('reason', selection)
        self.assertNotIn('outcome', selection)
        self.assertIn('current==chosen', selection)

    def test_native_dynamic_snapshots_are_observed_not_assumed(self):
        self.assertIn('getDynamicPointSnapshot(native_query_time)', self.result)
        self.assertIn('observe_native_dynamic("before_pass")', self.result)
        self.assertIn('observe_native_dynamic("after_pass")', self.result)
        self.assertIn('max_wheel_omega_==saved_wheel&&native_dynamic_stable', self.result)
        self.assertIn('native_dynamic_observed_stable', self.result)
        self.assertIn('!request_context||request_context->dynamic_repair', self.result)
        self.assertIn('if(native_overlay_owned)grid_map_->endDynamicPointPlanningSnapshot()', self.result)
        header = (HERE / 'delivery_replay_probe.hpp').read_text()
        for field in ('stamp.toNSec()', 'receive_time.toNSec()', 'frame_id', 'invalid_reason',
                      'p.position(k)', 'p.velocity(k)', 'p.track_id', 'p.confidence'):
            self.assertIn(field, header)
        self.assertIn('native_dynamic_KDtree_dumped', self.result)

    def test_native_constraints_remain(self):
        native = self.source[self.source.index('  bool PolyTrajOptimizer::IsTrajSafe('):]
        transformed = self.result[self.result.index('  bool PolyTrajOptimizer::IsTrajSafe('):]
        # Only the two callback recording hooks below this point differ.
        transformed = transformed.replace('\n    delivery_replay::first(costall,x,g);\n', '')
        transformed = transformed.replace('    delivery_replay::progress(x,g,fx,step,k,ls);\n', '')
        self.assertEqual(native, transformed)

    def test_missing_or_double_anchor_fails_closed(self):
        with self.assertRaises(ValueError):
            transform_optimizer(self.source.replace('    failure_probe::Frame diagnosis(', '    failure_probe::Frame changed('))
        with self.assertRaises(ValueError):
            transform_optimizer(self.source + '\n#include "state_contract_trace.hpp"\n')


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Read-only transformation tests. No compiler, ROS graph, model or hardware."""
import unittest
from pathlib import Path
from build_boundary_admission import (DEFAULT_BASE,INCLUDE,INIT,PRECHECK,ANCHOR,
    FSM_CONTEXT,FSM_CONTEXT_ANCHOR,transform_fsm,transform_manager,self_test)


class BoundaryAdmissionTransformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.original=(DEFAULT_BASE/'source/remani_replan_fsm.cpp').read_text()
        cls.generated=transform_fsm(cls.original)
        cls.manager_original=(DEFAULT_BASE/'source/planner_manager.cpp').read_text()
        cls.manager=transform_manager(cls.manager_original)

    def test_exact_off_projection(self):
        projected=self.generated.replace(INCLUDE,'',1).replace(INIT,'',1).replace(FSM_CONTEXT,'',1)
        self.assertEqual(projected,self.original)
        projected=self.manager.replace(INCLUDE,'',1).replace(PRECHECK,'',1)
        self.assertEqual(projected,self.manager_original)

    def test_no_new_io_or_action(self):
        for token in ('.publish(','.sleep(','.setZero(','.setMaxWallTime(','.setAstarMaxSearchTime(',
                      'state_contract_trace::flush','return true','next_cooperative_attempt_='):
            self.assertNotIn(token,PRECHECK)
        self.assertIn('return false;',PRECHECK)
        self.assertNotIn('continous_failures_count_++',PRECHECK)
        self.assertIn('continous_failures_count_before',PRECHECK)
        self.assertIn('continous_failures_count_after',PRECHECK)

    def test_exact_callpoint(self):
        self.assertEqual(self.manager.index(PRECHECK)+len(PRECHECK),self.manager.index(ANCHOR))
        self.assertLess(self.manager.index('iniStates_container.push_back(headState);'),self.manager.index(PRECHECK))
        self.assertLess(self.manager.index('t_init = ros::Time::now() - t_start;'),self.manager.index(PRECHECK))
        self.assertLess(self.manager.index(PRECHECK),self.manager.index('flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs('))
        self.assertLess(self.generated.index(FSM_CONTEXT_ANCHOR),self.generated.index(FSM_CONTEXT))
        self.assertNotIn('boundary_admission_decision.defer',self.generated)
        self.assertNotIn('boundary_admission::precheck(',self.generated)

    def test_frozen_retries_and_provenance(self):
        self_test(DEFAULT_BASE)

    def test_ambiguous_missing_or_reinstrumented_rejected(self):
        for text in ('',self.original+FSM_CONTEXT_ANCHOR,self.generated):
            with self.assertRaises(ValueError):transform_fsm(text)
        for text in ('',self.manager_original+ANCHOR,self.manager):
            with self.assertRaises(ValueError):transform_manager(text)

    def test_header_deferred_io_and_explicit_unknown(self):
        text=(Path(__file__).parent/'boundary_admission.hpp').read_text()
        self.assertIn('record.postsolve=',text)
        self.assertNotIn('state_contract_trace::flush(',text)
        self.assertIn('boundary_admission_v2_actual_fixed',text)
        self.assertIn('actual_fixed_not_exact_current_processed_measurement',text)
        self.assertIn('unknown_actual_fixed_heading',text)
        self.assertIn('same_outer_frozen_frame',text)
        self.assertIn('const int gear=signs.empty()?0:signs.front()',text)
        self.assertIn('fixedHead(head,gear)',text)
        self.assertIn('const RequestContext*& currentRequest()',text)
        self.assertNotIn('fixedHead(head,desired_gear)',text)

    def test_native_acceptance_tolerance_is_reused_and_reported(self):
        text=(Path(__file__).parent/'boundary_admission.hpp').read_text()
        native=(DEFAULT_BASE/'source/coupling_bridge.hpp').read_text()
        self.assertIn('if(!s.valid||s.gap<-1e-8)',native)
        self.assertIn('constexpr double same_native_gap_tolerance=1e-8;',text)
        self.assertIn('gap<-same_native_gap_tolerance',text)
        self.assertIn('native_gap_tolerance_accepted',text)
        self.assertIn('Category::native_tolerance',text)
        self.assertIn('.add("same_native_gap_tolerance",same_native_gap_tolerance)',text)


if __name__=='__main__':unittest.main()

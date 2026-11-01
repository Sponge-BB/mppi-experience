#!/usr/bin/env python3
"""Pure string/source tests: never compiles, starts ROS or modifies source."""
import unittest
from pathlib import Path
from patch_delivery_search import (DEFAULT_SOURCE,ROOT,PATCHES,INCLUDE,WRAPPER_INIT,
                                   NATIVE_INIT,transform_search,project_off,self_test)

class SearchExitTransformTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source=DEFAULT_SOURCE.read_text()
        cls.generated=transform_search(cls.source)

    def test_exact_off_projection(self):
        self.assertEqual(project_off(self.generated),self.source)
        self.assertEqual(self_test()['native_exits'],6)

    def test_default_off_and_deferred_only(self):
        header=(ROOT/'search_exit_probe.hpp').read_text()
        self.assertIn('state_contract_trace::enabled() &&',header)
        self.assertIn('flag("DELIVERY_SEARCH_PROBE")',header)
        self.assertIn('record.postsolve=',header)
        self.assertIn('state_contract_trace::enqueue(std::move(record))',header)
        for token in ('.publish(','.sleep(','.emit(','.flush(','.setParam(',
                      'state_contract_trace::flush(','.setMaxWallTime('):
            self.assertNotIn(token,header)

    def test_all_native_return_sites_marked(self):
        native=self.generated.split('int KinoAstar::search(',1)[1].split('int KinoAstar::yawToIndex',1)[0]
        self.assertEqual(native.count('delivery_search_scope.finish('),6)
        for reason in ('start_collision','goal_collision','oneshot_reach_end',
                       'time_budget_exhausted','node_capacity_exhausted','open_set_exhausted'):
            self.assertEqual(native.count('"'+reason+'"'),1)

    def test_wrapper_records_final_sampling_outcome_and_bypass(self):
        wrapper=self.generated.split('int KinoAstar::KinoAstarSearchAndGetSimplePath',1)[1].split('void KinoAstar::visPath',1)[0]
        self.assertEqual(wrapper.count('delivery_search_scope.finish('),3)
        self.assertIn('"arm_sampling_or_rrt_failed"',wrapper)
        self.assertIn('"astar_then_arm_sampling" : "rrt_fallback"',wrapper)
        self.assertIn('"continuous_failure_threshold" : "start_goal_close_arm_change"',wrapper)
        self.assertEqual(wrapper.count('search(start_state, end_state, init_ctrl);'),1)

    def test_copies_start_before_native_mutates_it_and_clock_unchanged(self):
        self.assertLess(self.generated.index(NATIVE_INIT),self.generated.index('  double start_time = ros::Time::now().toSec();'))
        self.assertEqual(self.source.count('ros::Time::now().toSec() - start_time > max_search_time_'),
                         self.generated.count('ros::Time::now().toSec() - start_time > max_search_time_'))
        self.assertEqual(self.source.count('start_state = end_state;'),self.generated.count('start_state = end_state;'))
        header=(ROOT/'search_exit_probe.hpp').read_text()
        self.assertIn('start_=start;end_=end;',header)
        self.assertIn('probe_parent_call_id',header)
        self.assertIn('arm/RRT internal exit not observed',header)

    def test_reapplication_missing_and_ambiguous_anchor_rejected(self):
        for source in ('',self.generated,self.source+'\n'+PATCHES[2][0]):
            with self.assertRaises(ValueError):transform_search(source)
        with self.assertRaises(ValueError):project_off(self.source)

    def test_native_layout_and_return_sequence_preserved(self):
        # Projection establishes more than token equality: all budgets, return
        # expressions, collision guards, fallback and class layouts are original.
        self.assertEqual(project_off(self.generated),self.source)
        self.assertNotIn('class KinoAstar',self.generated)
        self.assertIn('sampling_failure_scope',(ROOT/'search_exit_probe.hpp').read_text())

if __name__=='__main__':unittest.main()

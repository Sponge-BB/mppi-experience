#!/usr/bin/env python3
"""Tiny synthetic applicability/transition fixtures; no real trial analysis."""
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import report_guide_applicability as report
from test_report_state_contract import evidence, fixture, write_json, write_lines


def guide_rows():return [[0.,0.,0.,.2],[1.,0.,0.,.2],[2.,0.,0.,.2],[3.,0.,0.,.2]]


def inputs():
    cycles,odom,refs,commits,observer=evidence();array=guide_rows()
    digest=hashlib.sha256(b''.join(struct.pack('<4d',*r) for r in array)).hexdigest()
    refs[0].update(reference=array,times=[0.,1.,2.,3.],sha256=digest)
    cycles[0]['reference_array_sha256']=digest
    state=[1.5,.1,.05,.25,0.]
    for name in ('solve_state','validation_state'):cycles[0][name]=state.copy()
    for name in ('solve_odom_identity','validation_odom_identity'):cycles[0][name]['state']=state.copy()
    odom[0]['odom_identity']['state']=state.copy()
    return cycles,odom,refs,commits,observer


def cycle(base,ident,v,w,reason='ok',stamp=None):
    row=copy.deepcopy(base);row.update(cycle_id=ident,command_v=v,command_w=w,reason=reason)
    stamp=1002.+ident*.05 if stamp is None else stamp
    row.update(command_publish_before_ros_time=stamp,command_publish_after_ros_time=stamp+.001,
        first_kernel_cycle_for_reference=False,first_validated_command_cycle_for_reference=False)
    return row


class GuideApplicabilityTests(unittest.TestCase):
    def test_projected_local_error_is_not_p0_residual(self):
        result=report.sampled_neighborhood([1.5,.1,.05,.25,0.],guide_rows(),[0.,1.,2.,3.])
        self.assertEqual(result['status'],'unique_sampled_same_gear_neighborhood')
        error=result['nearest_errors']
        self.assertAlmostEqual(error['distance_m'],.1);self.assertAlmostEqual(error['lateral_error_m'],.1)
        self.assertAlmostEqual(error['heading_error_rad'],.05);self.assertAlmostEqual(error['speed_error_mps'],.05)
        self.assertEqual(error['segment_index'],1);self.assertAlmostEqual(error['fraction'],.5)
        self.assertFalse(result['true_splice_proven']);self.assertFalse(result['temporal_phase_window_applied'])

    def test_reverse_body_heading_not_replaced_by_travel_tangent(self):
        rows=[[2.,0.,0.,-.2],[1.,0.,0.,-.2],[0.,0.,0.,-.2]]
        result=report.sampled_neighborhood([1.5,.1,0.,-.15,0.],rows,[0.,1.,2.])
        error=result['nearest_errors'];self.assertEqual(error['gear'],-1)
        self.assertAlmostEqual(error['heading_error_rad'],0.);self.assertAlmostEqual(error['lateral_error_m'],.1)
        self.assertAlmostEqual(error['ordered_tangent_cross_track_m'],-.1)

    def test_crossing_retains_disjoint_ordered_branch_ambiguity(self):
        rows=[[-1.,0.,0.,.2],[1.,0.,0.,.2],[2.,2.,0.,.2],[0.,-1.,math.pi/2,.2],[0.,1.,math.pi/2,.2]]
        result=report.sampled_neighborhood([0.,0.,0.,.2,0.],rows,[0.,1.,2.,3.,4.],dict(branch_distance_tolerance_m=.001))
        self.assertEqual(result['status'],'ambiguous_sampled_branches')
        self.assertEqual(len(result['candidate_branches']),2);self.assertIsNone(result['nearest_errors'])

    def test_zero_measured_gear_unknown_not_guessed_from_yaw(self):
        result=report.sampled_neighborhood([1.,.1,0.,0.,0.],guide_rows(),[0.,1.,2.,3.])
        self.assertEqual(result['status'],'current_gear_unknown');self.assertIsNone(result['current_measured_gear'])
        self.assertIsNone(result['nearest_errors']);self.assertFalse(result['candidate_branches'][0]['same_gear_proven'])

    def test_opposite_gear_and_cross_gear_bridge_not_usable(self):
        result=report.sampled_neighborhood([.5,0.,0.,-.2,0.],guide_rows(),[0.,1.,2.,3.])
        self.assertEqual(result['status'],'no_same_gear_fragment');self.assertIsNone(result['nearest_errors'])
        rows=[[0.,0.,0.,.2],[1.,0.,0.,-.2]]
        self.assertEqual(report.sampled_neighborhood([.5,0.,0.,.2,0.],rows,[0.,1.])['eligible_segments'],0)
        rows=[[0.,0.,0.,.2],[1.,0.,0.,.2]]
        self.assertEqual(report.sampled_neighborhood([.5,0.,0.,.2,0.],rows,[0.,0.])['eligible_segments'],0)
        self.assertEqual(report.sampled_neighborhood([.5,0.,0.,.2,0.],rows,[1.,0.])['status'],'unknown')

    def test_payload_sha_order_and_missing_data_are_explicit(self):
        data=inputs();payload,rows,times=report.sampled_payload(data[2][0])
        self.assertTrue(payload['payload_sha256_verified']);self.assertTrue(payload['ordered_times_valid'])
        self.assertIsNone(payload['native_polynomial_evaluation'])
        for change in ('hash','times','array'):
            record=copy.deepcopy(data[2][0])
            if change=='hash':record['sha256']='0'*64
            elif change=='times':record['times']=[0.,2.,1.,3.]
            else:record['reference'][0][0]=None
            _,rows,times=report.sampled_payload(record)
            self.assertIsNone(rows);self.assertIsNone(times)

    def test_actual_first_kernel_exact_chain_and_identity_not_nearest_stamp(self):
        data=inputs();result=report.analyze_guides(*data)['records'][0]
        self.assertTrue(result['exact_association']);self.assertTrue(result['first_flag_consistent'])
        self.assertEqual(result['first_kernel_cycle_id'],1)
        self.assertEqual(result['applicability']['status'],'unique_sampled_same_gear_neighborhood')
        data[1][0]['odom_identity']['message_hash']='99'
        result=report.analyze_guides(*data)['records'][0]
        self.assertFalse(result['exact_association']);self.assertIsNone(result['applicability'])
        self.assertIsNone(report.analyze_guides(None,None,None,None,None)['any_exact_guide'])
        data=inputs();data[0][0]['first_kernel_cycle_for_reference']=False
        self.assertIsNone(report.analyze_guides(*data)['records'][0]['applicability'])

    def test_command_transition_zero_reasons_and_recovery_not_motion_claim(self):
        data=inputs();base=data[0][0]
        before=cycle(base,1,.2,0.);before['guide_revision']=2
        first=cycle(base,2,0.,0.,'deadline_miss');first['first_kernel_cycle_for_reference']=True
        zero=cycle(base,3,0.,0.,'local_validation_failed');moving=cycle(base,4,.12,0.)
        for row in (first,zero,moving):
            row['validation_state'][3:]=[0.,0.];row['validation_odom_identity']['state']=row['validation_state'].copy()
        data[1][0]['odom_identity']['state']=first['validation_state'].copy()
        transition=report.transition_context([before,first,zero,moving],1,data[1])
        self.assertEqual(transition['command_delta_v_w'],[-.2,0.])
        self.assertEqual(transition['first_nonzero_command']['cycle_id'],4)
        self.assertEqual(transition['zero_command_reasons'],{'deadline_miss':1,'local_validation_failed':1})
        run=transition['zero_command_run_at_first_kernel'];self.assertFalse(run['right_censored'])
        self.assertAlmostEqual(run['publication_duration_bounds_s'][0],.099)
        self.assertAlmostEqual(run['publication_duration_bounds_s'][1],.101)
        self.assertFalse(transition['measured_motion_observed_at_first_nonzero_command'])
        self.assertTrue(transition['measured_low_motion_run_at_first_kernel']['right_censored'])

    def test_gaps_and_missing_clocks_never_get_nearest_or_fifty_ms_duration(self):
        data=inputs();base=data[0][0]
        rows=[cycle(base,1,.2,0.),cycle(base,3,0.,0.),cycle(base,4,.1,0.)]
        del rows[2]['command_publish_before_ros_time']
        transition=report.transition_context(rows,1,data[1])
        self.assertIsNone(transition['before']);self.assertIsNone(transition['command_delta_v_w'])
        self.assertIsNone(transition['zero_command_run_at_first_kernel']['publication_duration_bounds_s'])

    def test_stop_can_predate_first_use_and_guide_change_censors_recovery(self):
        data=inputs();base=data[0][0]
        rows=[cycle(base,1,0.,0.,'native_abort'),cycle(base,2,0.,0.),cycle(base,3,.1,0.)]
        rows[0]['guide_revision']=2;rows[2]['guide_revision']=4
        transition=report.transition_context(rows,1,data[1])
        self.assertTrue(transition['zero_command_run_at_first_kernel']['started_before_first_kernel'])
        self.assertTrue(transition['zero_command_run_at_first_kernel']['right_censored'])
        self.assertIsNone(transition['first_nonzero_command'])

    def test_offline_guard_before_any_large_log_analysis_or_plot(self):
        with tempfile.TemporaryDirectory(prefix='guide_report_test_') as directory:
            root=Path(directory);fixture(root,'off',False);fixture(root,'on',True)
            write_json(root/'plan.json',{});write_json(root/'complete.json',dict(complete=True,completed=[dict(trial='off'),dict(trial='on')]))
            out=root/'report'
            with patch.object(report,'assert_plot_quiet',side_effect=RuntimeError('ROS live')) as guard,patch.object(report,'trial_report') as analyze:
                with self.assertRaises(RuntimeError):report.main(['--root',str(root),'--out',str(out)])
                self.assertEqual(guard.call_count,1);analyze.assert_not_called()
            self.assertFalse(out.exists())
            with patch.object(report,'assert_plot_quiet',side_effect=RuntimeError('ROS live')):
                with self.assertRaises(RuntimeError):report.plot_report({},root/'never.png')
            self.assertFalse((root/'never.png').exists())

    def test_tiny_completed_batch_outputs_json_text_only(self):
        with tempfile.TemporaryDirectory(prefix='guide_report_test_') as directory:
            root=Path(directory)
            for name,enabled in (('off',False),('on',True)):
                trial=fixture(root,name,enabled);data=inputs()
                for filename,records in zip(('state_mppi_cycles.jsonl','state_odom_events.jsonl','consumed_references.jsonl','reference_events.jsonl','observer_events.jsonl'),data):write_lines(trial/filename,records)
            write_json(root/'plan.json',{});write_json(root/'complete.json',dict(complete=True,completed=[dict(trial='off'),dict(trial='on')]))
            out=root/'report'
            with patch.object(report,'assert_plot_quiet'),patch.object(report,'plot_report') as plot,patch('sys.stdout',new=io.StringIO()):
                self.assertEqual(report.main(['--root',str(root),'--out',str(out)]),0);plot.assert_not_called()
            result=json.loads((out/'summary.json').read_text())
            self.assertFalse(result['true_splice_proven']);self.assertFalse(result['paired_same_input'])
            self.assertIn('不是p0残差', (out/'实验结论.txt').read_text())
            self.assertFalse(list(out.glob('*.png')))


if __name__=='__main__':unittest.main()

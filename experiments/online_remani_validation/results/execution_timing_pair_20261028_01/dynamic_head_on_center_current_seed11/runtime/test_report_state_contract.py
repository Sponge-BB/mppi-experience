#!/usr/bin/env python3
"""Tiny synthetic evidence fixtures only: no ROS, live trials, model or plots."""
import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import report_state_contract as report


MESSAGE='a'*64
ARRAY='b'*64
TRANSACTION='c'*64
GOAL='01020304'


def identity(**changes):
    row=dict(algorithm=report.IDENTITY_ALGORITHM,message_hash='17000000000000000001',
        header_seq=7,stamp_sec=1002,stamp_nsec=123456789,frame_id='world',child_frame_id='base_link',
        state=[1.,2.,.4,.15,-.1],validity=dict(serialized=True,state_finite=True,all_numeric_finite=True))
    row.update(changes);return row


def evidence():
    measured=identity()
    odom=[dict(event='odom_callback',callback_ros_time=1002.15,callback_monotonic=20.,odom_identity=measured)]
    cycle=dict(event='mppi_cycle',cycle_id=1,goal_generation=4,guide_revision=3,guide_epoch=2,
        message_hashes=[MESSAGE],reference_array_sha256=ARRAY,
        solve_state=measured['state'].copy(),validation_state=measured['state'].copy(),
        solve_odom_identity=copy.deepcopy(measured),validation_odom_identity=copy.deepcopy(measured),
        online_step_invoked=True,online_step_succeeded=True,kernel_code=0,
        validation_invoked=True,validation_code=0,validated_command_published=True,reason='ok',
        first_kernel_cycle_for_reference=True,first_validated_command_cycle_for_reference=True,
        solve_ros_time=1002.16,validation_ros_time=1002.17,command_ros_time=1002.18,command_v=.12,command_w=.01)
    cycle.update(online_step_entry_ros_time=1002.165,online_step_entry_monotonic=20.165,
        command_publish_before_ros_time=1002.18,command_publish_after_ros_time=1002.181)
    refs=[dict(t=2.16,revision=3,epoch=2,message_hashes=[MESSAGE],sha256=ARRAY)]
    commits=[dict(t=2.14,event='replace',revision=3,epoch=2,message_hashes=[MESSAGE],goal_key=GOAL)]
    observer=[dict(event='reference_observer_receipt',valid_framing=True,message_hashes=[MESSAGE],
        goal_sha256=hashlib.sha256(GOAL.encode()).hexdigest(),transaction_sha256=TRANSACTION,transaction_fnv64='42')]
    return [cycle],odom,refs,commits,observer


def write_json(path,row):path.write_text(json.dumps(row),encoding='utf-8')
def write_lines(path,rows):path.write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')


def fixture(root,name='off',enabled=False):
    trial=root/name;trial.mkdir()
    write_json(trial/'manifest.json',dict(scene='static',build='same-build',build_manifest_sha256='d'*64,
        experiment_sources_sha256={'mppi_node.py':'e'*64},coupling_experiment=dict(
            seed=11,signed_low_speed=enabled,state_contract_capture=True,domain_recovery=True)))
    write_json(trial/'run_status.json',dict(returncode=0,result_written=True,monitored_node_pids={'/fake':999999}))
    write_json(trial/'trial_audit.json',dict(valid_experiment=True))
    write_json(trial/'result.json',dict(reason='timeout',collision_conservative_projection=False))
    cycles,odom,refs,commits,observer=evidence()
    for name,rows in zip(('state_mppi_cycles.jsonl','state_odom_events.jsonl','consumed_references.jsonl',
                          'reference_events.jsonl','observer_events.jsonl'),(cycles,odom,refs,commits,observer)):
        write_lines(trial/name,rows)
    return trial


def native_fixture(trial, search=False):
    measured=identity();yaw=.4;velocity=[.15*math.cos(yaw),.15*math.sin(yaw)]
    raw=dict(raw_odom_missing=False,raw_odom_message_hash_algorithm=measured['algorithm'],
        raw_odom_message_fnv64=measured['message_hash'],raw_odom_header_seq=7,
        raw_odom_stamp_sec=1002,raw_odom_stamp_nsec=123456789,
        raw_odom_frame_id='world',raw_odom_child_frame_id='base_link',
        raw_odom_x=1.,raw_odom_y=2.,raw_odom_yaw=yaw,raw_odom_body_vx=.15,
        raw_odom_processed_vx=velocity[0],raw_odom_processed_vy=velocity[1],raw_odom_processed_gear=1,
        planner_p=[1.,2.],planner_v=velocity,planner_a=[0.,0.],planner_j=[0.,0.],planner_yaw=yaw,planner_gear=1)
    rows=[]
    for index,stage in enumerate(report.CRITICAL_STAGES):
        row=dict(pid=101,request_id=1,event='state_contract_'+stage,
                 capture_mono=20.+index*.01,capture_sim=1002.13+index*.01)
        if stage in report.RAW_STAGES:row.update(raw)
        rows.append(row)
    by_stage={report.event_stage(row):row for row in rows}
    by_stage['solver_return']['success']=True;by_stage['request_end']['success']=True
    by_stage['epoch_chosen'].update(chosen_start_epoch=1002.15,requested_start_epoch=1002.14)
    by_stage['reference_publish'].update(transaction_fnv64='42',actual_start_epoch=1002.15,candidate_empty=False)
    by_stage['candidate_ready']['candidate_empty']=False
    matrix=[[1.1,velocity[0],0.,0.],[2.,velocity[1],0.,0.]]
    snapshot=dict(schema='delivery_snapshot_v1',nx=8,ny=8,gx=0.,gy=-3.,res=1.,margin=.1,
        max_age=.35,max_prediction=3.,healthy=True,sequence=4,stamp=1002.1,
        body=[[0.,0.,.2]],obstacles=[],field=[float(x) for _ in range(8) for x in range(8)])
    directory=trial/'delivery_diagnostics/snapshots';directory.mkdir(parents=True,exist_ok=True)
    path=directory/'fixture.json';write_json(path,snapshot)
    payload_hash=format(int(report.fnv1a64(path.read_bytes())),'016x')
    backend=by_stage['backend_fixed'];backend.update(initial_states=[matrix],final_states=[matrix],gears=[1],
        planner_p=[1.1,2.],fixed_yaw=yaw,fixed_gear=1,state_contract_backend_id=1,state_contract_backend_id_valid=True,input_hash='99',
        backend_frame=dict(now=1002.15,snapshot='snapshots/fixture.json',snapshot_payload_hash=payload_hash,hash_algorithm='fnv1a64_json'),
        raw_static=dict(valid=True,physical=.8,gap=.7),fixed_static=dict(valid=True,physical=.9,gap=.8))
    rows.extend([dict(pid=101,request_id=1,event='state_contract_backend_scope_link',
        capture_mono=20.035,capture_sim=1002.165,state_contract_backend_id=1,scope_id=9,input_hash='99'),
        dict(pid=101,request_id=1,event='coupling_scope',scope_id=9,input_hash=99),
        dict(pid=101,request_id=1,event='stage_begin',stage='initialization',poly_init=search)])
    if search:
        for stage,fields in [('native_astar_input',{}),('native_astar_root',dict(native_root_gear=1,native_signed_speed=.15)),
            ('native_astar_result',dict(status=0)),('native_minco_head',dict(block_index=0,head_state=matrix,native_first_gear=1))]:
            rows.append(dict(pid=101,request_id=1,event='state_contract_'+stage,capture_mono=20.02,capture_sim=1002.15,**fields))
    return rows


class StateReportTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(prefix='state_contract_report_test_')
        self.addCleanup(self.temporary.cleanup);self.root=Path(self.temporary.name)

    def test_exact_cycle_reference_and_state_chain(self):
        result=report.analyze_cycles(*evidence())
        self.assertTrue(result['all_cycles_exact']);self.assertEqual(result['exact_validated_cycles'],1)
        row=result['rows'][0]
        self.assertTrue(row['solve']['state_equals_identity'])
        self.assertEqual(row['reference']['transaction_sha256'],TRANSACTION)

    def test_every_identity_component_is_required_no_nearest_time(self):
        for field,value in [('message_hash','17000000000000000002'),('header_seq',8),('stamp_sec',1003),
            ('stamp_nsec',123456788),('frame_id','odom'),('child_frame_id','other'),('algorithm','other')]:
            with self.subTest(field=field):
                inputs=evidence();inputs[0][0]['solve_odom_identity'][field]=value
                result=report.analyze_cycles(*inputs)
                self.assertIsNot(result['all_cycles_exact'],True)
        for field in ('message_hash','header_seq','stamp_sec','stamp_nsec','frame_id','child_frame_id'):
            row=identity();del row[field]
            self.assertIsNone(report.identity_key(row))

    def test_same_stamp_different_payload_not_matched(self):
        inputs=evidence();inputs[1][0]['odom_identity']['message_hash']='99'
        result=report.analyze_cycles(*inputs)
        self.assertFalse(result['rows'][0]['solve']['exact_message_observed'])
        self.assertFalse(result['all_cycles_exact'])

    def test_absent_and_empty_evidence_never_certifies(self):
        for value in (None,[]):
            result=report.analyze_cycles(value,value,value,value,value)
            self.assertIsNone(result['all_cycles_exact']);self.assertIsNone(result['exact_validated_cycles'])
        for index in range(1,5):
            inputs=evidence();inputs=list(inputs);inputs[index]=None
            self.assertIsNot(report.analyze_cycles(*inputs)['all_cycles_exact'],True)

    def test_null_nonfinite_and_state_disagreement_are_not_exact(self):
        for value in (None,float('nan'),float('inf'),99.):
            inputs=evidence();inputs[0][0]['solve_state'][0]=value
            self.assertIsNot(report.analyze_cycles(*inputs)['all_cycles_exact'],True)

    def test_reference_needs_array_revision_epoch_message_and_goal_hash(self):
        for field,value in [('sha256','d'*64),('revision',4),('epoch',3),('message_hashes',['e'*64])]:
            inputs=evidence();inputs[2][0][field]=value
            self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])
        inputs=evidence();inputs[4][0]['goal_sha256']='f'*64
        self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])
        inputs=evidence();inputs[0][0]['message_hashes']=[];inputs[2][0]['message_hashes']=[]
        self.assertIsNot(report.analyze_cycles(*inputs)['all_cycles_exact'],True)

    def test_duplicate_commit_consumption_or_transaction_fails_closed(self):
        for index in (2,3):
            inputs=evidence();inputs[index].append(copy.deepcopy(inputs[index][0]))
            self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])
        inputs=evidence();inputs[4].append(dict(inputs[4][0],transaction_sha256='f'*64))
        self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])

    def test_identical_payload_receipts_not_unique_receive_time(self):
        inputs=evidence();inputs[1].append(dict(inputs[1][0],callback_ros_time=1003.))
        result=report.analyze_cycles(*inputs)
        self.assertTrue(result['all_cycles_exact'])
        self.assertEqual(result['rows'][0]['solve']['matching_receipts'],2)
        self.assertIsNone(result['rows'][0]['solve']['unique_callback_ros_time'])

    def test_invocation_success_validation_and_nonzero_motion_are_distinct(self):
        for field,value in [('online_step_invoked',False),('online_step_succeeded',False),('kernel_code',4),
            ('validation_invoked',False),('validated_command_published',False),('validation_code',5),('reason','deadline_miss')]:
            inputs=evidence();inputs[0][0][field]=value
            self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])
        inputs=evidence();inputs[0][0]['command_v']=0.;inputs[0][0]['command_w']=0.
        self.assertTrue(report.analyze_cycles(*inputs)['all_cycles_exact'])

    def test_first_flags_and_cycle_ids_checked_without_vacuous_success(self):
        inputs=evidence();second=copy.deepcopy(inputs[0][0]);second['cycle_id']=2
        second['first_kernel_cycle_for_reference']=False;second['first_validated_command_cycle_for_reference']=False
        inputs[0].append(second)
        self.assertTrue(report.analyze_cycles(*inputs)['all_cycles_exact'])
        second['first_kernel_cycle_for_reference']=True
        self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])
        second['cycle_id']=1
        self.assertFalse(report.analyze_cycles(*inputs)['all_cycles_exact'])

    def test_trial_missing_files_are_unknown_not_zero(self):
        trial=fixture(self.root)
        (trial/'state_odom_events.jsonl').unlink()
        result=report.summarize_trial(trial)
        self.assertIsNone(result['observer_odom_records'])
        self.assertIsNot(result['python_chain']['all_cycles_exact'],True)
        self.assertIsNone(result['native_chain']['observed_requests'])

    def batch(self):
        fixture(self.root,'off',False);fixture(self.root,'on',True)
        write_json(self.root/'plan.json',dict(state_contract=True,paired_same_input=False))
        write_json(self.root/'complete.json',dict(complete=True,completed=[dict(trial='off'),dict(trial='on')]))

    def test_complete_batch_text_json_only_no_plot_or_speed_claim(self):
        self.batch();result=report.build_report(self.root)
        self.assertFalse(result['performance_comparison']);self.assertFalse(result['paired_same_input'])
        self.assertTrue(result['pair_consistency']['runtime_sha256'])
        self.assertIn('不是速度benchmark',report.report_text(result))
        out=self.root/'report'
        with patch.object(report,'plot_report') as plot:
            self.assertEqual(report.main(['--root',str(self.root),'--out',str(out)]),0)
            plot.assert_not_called()
        self.assertTrue((out/'summary.json').is_file());self.assertTrue((out/'实验结论.txt').is_file())
        self.assertFalse(list(out.glob('*.png')))

    def test_incomplete_batch_cannot_create_output(self):
        write_json(self.root/'plan.json',{});write_json(self.root/'complete.json',dict(complete=False,completed=[]))
        out=self.root/'report'
        with self.assertRaises(ValueError):report.main(['--root',str(self.root),'--out',str(out)])
        self.assertFalse(out.exists())

    def test_plot_guard_precedes_output_or_matplotlib(self):
        self.batch();out=self.root/'report'
        with patch.object(report,'assert_plot_quiet',side_effect=RuntimeError('ROS still live')) as guard:
            with self.assertRaises(RuntimeError):report.main(['--root',str(self.root),'--out',str(out),'--plot'])
            self.assertEqual(guard.call_count,1)
        self.assertFalse(out.exists())
        with patch.object(report,'assert_plot_quiet',side_effect=RuntimeError('ROS still live')):
            with self.assertRaises(RuntimeError):report.plot_report({},self.root/'forbidden.png')
        self.assertFalse((self.root/'forbidden.png').exists())

    def test_empty_runtime_hashes_not_pair_proof(self):
        self.batch()
        for name in ('off','on'):
            path=self.root/name/'manifest.json';data=json.loads(path.read_text());data['experiment_sources_sha256']={};write_json(path,data)
        self.assertIsNone(report.build_report(self.root)['pair_consistency']['runtime_sha256'])

    def test_snapshot_traversal_and_missing_are_explicit(self):
        for path in ('../../outside.json','snapshots/not-recorded.json'):
            result=report.snapshot_evidence(self.root,dict(backend_frame=dict(snapshot=path)))
            self.assertFalse(result['present']);self.assertIn('error',result)
        self.assertIsNone(report.snapshot_evidence(self.root,{})['present'])

    def test_complete_native_critical_chain_and_independent_three_endpoints(self):
        rows=native_fixture(self.root);python=report.analyze_cycles(*evidence())
        result=report.analyze_native(rows,evidence()[1],python,self.root)['requests'][0]
        self.assertTrue(result['complete_native_to_validated_command'])
        self.assertEqual(result['frontend_branch'],'warm')
        snapshot=result['backend_snapshot']
        self.assertTrue(snapshot['payload_hash_verified'])
        self.assertTrue(snapshot['raw_online_agreement']);self.assertTrue(snapshot['fixed_online_agreement'])
        for endpoint,gap in [('raw',.7),('selected',.7),('fixed',.8)]:
            self.assertAlmostEqual(snapshot['independent_'+endpoint+'_static']['gap'],gap)
        self.assertAlmostEqual(result['metrics']['first_kernel_entry_odom_age_s'],1002.165-1002.123456789)
        self.assertAlmostEqual(result['metrics']['publish_lateness_s'],1002.20-1002.15)
        self.assertIsNone(result['metrics']['gears']['native_root'])

    def test_missing_or_duplicate_critical_stage_cannot_be_complete(self):
        original=native_fixture(self.root);python=report.analyze_cycles(*evidence())
        for stage in report.CRITICAL_STAGES:
            rows=[row for row in original if report.event_stage(row)!=stage]
            result=report.analyze_native(rows,evidence()[1],python,self.root)['requests'][0]
            self.assertIsNot(result['complete_native_to_validated_command'],True,stage)
        rows=copy.deepcopy(original);rows.append(copy.deepcopy(next(row for row in rows if report.event_stage(row)=='backend_fixed')))
        self.assertIsNot(report.analyze_native(rows,evidence()[1],python,self.root)['requests'][0]['complete_native_to_validated_command'],True)

    def test_native_root_required_for_actual_search_not_fabricated_zero(self):
        rows=native_fixture(self.root,search=True);python=report.analyze_cycles(*evidence())
        result=report.analyze_native(rows,evidence()[1],python,self.root)['requests'][0]
        self.assertTrue(result['complete_native_to_validated_command']);self.assertEqual(result['metrics']['gears']['native_root'],1)
        rows=[r for r in rows if report.event_stage(r)!='native_astar_root']
        result=report.analyze_native(rows,evidence()[1],python,self.root)['requests'][0]
        self.assertFalse(result['frontend_chain_complete']);self.assertIsNone(result['metrics']['gears']['native_root'])

    def test_missing_entry_clock_not_replaced_by_state_copy_checkpoint(self):
        inputs=evidence();del inputs[0][0]['online_step_entry_ros_time']
        python=report.analyze_cycles(*inputs)
        self.assertIsNone(python['rows'][0]['entry_odom_age_s'])
        rows=native_fixture(self.root)
        metrics=report.analyze_native(rows,inputs[1],python,self.root)['requests'][0]['metrics']
        self.assertIsNone(metrics['publish_marker_to_kernel_entry_s'])
        self.assertIsNone(metrics['first_kernel_entry_odom_age_s'])


if __name__=='__main__':unittest.main()

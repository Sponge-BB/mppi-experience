#!/usr/bin/env python3
"""Synthetic report/render regressions only; no ROS, native binaries or controls."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from PIL import Image
import report_shared_collision as reporting
import compare_shared_panels as panels


def fixture(path,mode='remani'):
    path.mkdir(parents=True)
    names='t,x,y,yaw,v,w,trajectory_count,safety_limit'
    states=np.zeros((9,8));states[:,0]=np.arange(9.);states[:,1]=.5;states[:,2]=.1
    states[3,5]=.3;states[4,4]=.1;states[:,6]=2;states[:,7]=1.
    np.savetxt(path/'state.csv',states,delimiter=',',header=names,comments='')
    (path/'obstacles.csv').write_text('t,x,y,r,id,vx,vy\n')
    body=np.array([[-.1,0.,.2],[.1,0.,.2]])
    np.savetxt(path/'geometry.csv',body,delimiter=',')
    (path/'geometry.json').write_text(json.dumps(dict(body_disks_xy_r=body.tolist(),
        base_length=.4,base_width=.3,arm_collision_points_xyz=[])))
    reference=np.column_stack((np.linspace(0,3,101),np.zeros((101,2)),np.full(101,.1)))
    refs=[dict(t=1.,revision=1,epoch=1,reference=reference.tolist(),times=np.linspace(0,10,101).tolist()),
          dict(t=2.,revision=2,epoch=2,reference=reference.tolist(),times=np.linspace(0,10,101).tolist()),
          dict(t=6.,revision=4,epoch=3,reference=reference.tolist(),times=np.linspace(0,10,101).tolist())]
    events=[dict(t=1.,revision=1,epoch=1,event='replace'),dict(t=2.,revision=2,epoch=2,event='replace'),
            dict(t=5.,revision=3,epoch=2,event='cancel'),dict(t=6.,revision=4,epoch=3,event='replace')]
    for name,values in (('consumed_references.jsonl',refs),('reference_events.jsonl',events)):
        (path/name).write_text(''.join(json.dumps(value)+'\n' for value in values))
    result=dict(reason='goal',motion_anchor=1.,goal_sent=.5,elapsed=8.,scene='empty',
                min_clearance=.8,trajectory_bundles=3,collision_conservative_projection=False)
    for name,value in (('result.json',result),('run_status.json',dict(required_process_failed=False)),
                       ('mppi_diagnostics.json',[]),('trajectories.json',[]),('manifest.json',dict(mode=mode))):
        (path/name).write_text(json.dumps(value))
    return path


class SharedReportTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='shared_comparison_test_')
        self.root=Path(self.temp.name)

    def tearDown(self):self.temp.cleanup()

    def test_native_stop_is_measured_and_rotation_is_not_stop(self):
        source=fixture(self.root/'native')
        values=reporting.trial_metrics(source,write=False)
        self.assertAlmostEqual(values['full_stop_away_goal_s'],5.)
        self.assertAlmostEqual(values['longest_full_stop_s'],3.)
        self.assertEqual(values['kernel_p95_ms'],None)
        self.assertFalse((source/'reference_deviation.npz').exists())

    def test_cancelled_and_uncommitted_references_are_not_active(self):
        refs=[dict(t=1.,revision=1,epoch=1)]
        events=[dict(t=1.,revision=1,event='replace'),dict(t=2.,revision=2,event='cancel'),
                dict(t=3.,revision=3,event='replace')]
        self.assertIs(reporting.active_reference(refs,events,1.5),refs[0])
        self.assertIsNone(reporting.active_reference(refs,events,2.5))
        self.assertIsNone(reporting.active_reference(refs,events,3.5))
        self.assertIsNone(reporting.active_reference(refs,events,.5))

    def test_cancel_interval_is_excluded_from_error_samples(self):
        source=fixture(self.root/'native')
        values=reporting.trial_metrics(source,write=False)
        self.assertEqual(values['reference_error_samples'],7)
        self.assertAlmostEqual(values['after_new_reference_p95_m'],.1)

    def test_append_does_not_reset_ordered_projection(self):
        source=fixture(self.root/'native')
        refs=[json.loads(s) for s in (source/'consumed_references.jsonl').read_text().splitlines()]
        refs=refs[:2];refs[1]['epoch']=1
        events=[dict(t=1.,revision=1,event='replace'),dict(t=2.,revision=2,event='append')]
        (source/'consumed_references.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in refs))
        (source/'reference_events.jsonl').write_text(''.join(json.dumps(v)+'\n' for v in events))
        previous=[]
        def projection(state,reference,times,old):
            previous.append(old);return old+1.,0.,.1
        with patch.object(reporting,'project',side_effect=projection):reporting.trial_metrics(source,write=False)
        self.assertEqual(previous,list(np.arange(8.)))

    def test_no_finite_ordered_values_are_reported_as_missing(self):
        source=fixture(self.root/'native')
        with patch.object(reporting,'project',return_value=(0.,0.,float('inf'))):
            values=reporting.trial_metrics(source,write=False)
        self.assertIsNone(values['ordered_reference_p95_m'])
        self.assertEqual(values['ordered_reference_valid_samples'],0)

    def test_zero_motion_anchor_and_missing_motion_are_distinguished(self):
        self.assertEqual(reporting.measured_anchor(dict(motion_anchor=0.,goal_sent=3.)),0.)
        self.assertEqual(reporting.measured_anchor(dict(motion_anchor=None,goal_sent=3.)),3.)
        source=fixture(self.root/'native');r=json.loads((source/'result.json').read_text());r['motion_anchor']=None
        (source/'result.json').write_text(json.dumps(r))
        self.assertFalse(reporting.trial_metrics(source,write=False)['measured_motion_started'])

    def test_measured_baseline_status_is_not_waiting(self):
        trial=panels.load(fixture(self.root/'native'));trial['manifest']=dict(mode='remani')
        self.assertIn('静止',panels.measured_status(trial,trial['state'][1],1.))
        self.assertIn('转向',panels.measured_status(trial,trial['state'][3],3.))
        self.assertIn('行进',panels.measured_status(trial,trial['state'][4],4.))

    def test_panels_use_measured_metrics_and_preserve_sources(self):
        native=fixture(self.root/'native');local=fixture(self.root/'local','mppi')
        before=panels.source_hashes(native)
        values=panels.compare(native,local,self.root/'comparison')
        self.assertAlmostEqual(values['old']['full_stop_away_goal_s'],5.)
        self.assertEqual(values['new']['full_stop_away_goal_s'],values['old']['full_stop_away_goal_s'])
        self.assertEqual(before,panels.source_hashes(native))
        self.assertTrue(values['source_unchanged'])
        with Image.open(self.root/'comparison/comparison.gif') as image:
            self.assertGreater(image.n_frames,1)
            for i in range(image.n_frames):image.seek(i);image.load()

    def test_single_analysis_failure_does_not_prevent_report(self):
        suite=self.root/'suite';suite.mkdir()
        source=fixture(suite/'empty/remani')
        (source/'consumed_references.jsonl').unlink()
        plan=[dict(key='empty/remani',scene='empty',variant='remani',repeat=1),
              dict(key='empty/remani_shared',scene='empty',variant='remani_shared',repeat=1)]
        (suite/'suite_manifest.json').write_text(json.dumps(dict(specs=plan)))
        rows=reporting.report(suite,render=False)
        self.assertEqual(rows[0]['result'],'goal')
        self.assertIn('analysis_error',rows[0])
        self.assertEqual(rows[1]['result'],'incomplete')
        self.assertTrue((suite/'README.md').is_file())
        self.assertFalse(json.loads((suite/'report_status.json').read_text())['derived_report_complete'])

    def test_single_pair_render_failure_does_not_prevent_other_pairs(self):
        suite=self.root/'suite';suite.mkdir();plan=[]
        for variant in ('remani','v12','remani_shared','v12_shared'):
            fixture(suite/'empty'/variant,'remani' if variant.startswith('remani') else 'mppi')
            plan.append(dict(key='empty/'+variant,scene='empty',variant=variant,repeat=2))
        # Render only the suite pair comparisons; avoid importing ROS audits.
        for item in plan:item['repeat']=1
        (suite/'suite_manifest.json').write_text(json.dumps(dict(specs=plan)))
        import sys
        from types import ModuleType
        reference_audit=ModuleType('audit_native_reference');execution_audit=ModuleType('audit_execution')
        search=ModuleType('render_native_search')
        def reference(path,render_images=False):(path/'reference_audit.json').write_text(json.dumps(dict(reference_contract_pass=True)))
        def execution(path):(path/'execution_audit.json').write_text(json.dumps(dict(execution_contract_pass=True)))
        reference_audit.audit=reference;execution_audit.audit=execution;search.render=lambda *args:None
        calls=[]
        def comparator(*args):
            calls.append(args[2].name)
            if len(calls)==1:raise ValueError('synthetic renderer failure')
        with patch.dict(sys.modules,{'audit_native_reference':reference_audit,'audit_execution':execution_audit,'render_native_search':search}), \
             patch.object(reporting,'summary_image',return_value=None),patch.object(reporting,'compare',side_effect=comparator):
            rows=reporting.report(suite,render=True)
        self.assertEqual(calls,['original','shared','v12_change'])
        self.assertEqual(len(rows),4)
        self.assertTrue((suite/'README.md').is_file())
        self.assertEqual(json.loads((suite/'report_status.json').read_text())['errors'][0]['stage'],'pair_render')

    def test_matplotlib_canvas_compatibility(self):
        from matplotlib.figure import Figure
        rows=[dict(scene='empty',variant='remani',repeat=1,full_stop_away_goal_s=1.,
              after_new_reference_p95_m=None,min_clearance_m=.1)]
        with patch.object(Figure,'savefig',lambda figure,*args,**kwargs:figure.canvas.draw()):
            reporting.summary_image(self.root/'no_output',rows)

    def test_shared_native_input_pass_does_not_claim_local_execution_verified(self):
        suite=self.root/'suite';suite.mkdir();source=fixture(suite/'empty/remani_shared')
        (source/'manifest.json').write_text(json.dumps(dict(mode='remani',shared_collision=True,local_execution_owner=False)))
        (suite/'suite_manifest.json').write_text(json.dumps(dict(specs=[dict(key='empty/remani_shared',scene='empty',variant='remani_shared',repeat=1)])))
        shared=dict(passed=True,validated_transports=3,native_reference_segments=2,local_approved_records=0,
                    native_segments_over_max_age=1,native_input_age_s=dict(max=.9),warnings=[],errors=[])
        with patch('audit_shared_collision.audit',return_value=shared) as audit:
            rows=reporting.report(suite,render=False)
        audit.assert_called_once_with(source)
        self.assertTrue(rows[0]['shared_input_contract_pass'])
        self.assertEqual(rows[0]['shared_input_status'],'checked_native_input_only')
        self.assertIn('NOT verified',rows[0]['local_shared_input_verification'])
        self.assertEqual(rows[0]['native_shared_segments_over_max_age'],1)

    def test_shared_without_approved_local_execution_is_not_passed(self):
        suite=self.root/'suite';suite.mkdir();source=fixture(suite/'empty/remani_shared','mppi')
        (source/'manifest.json').write_text(json.dumps(dict(mode='mppi',shared_collision=True,local_execution_owner=True)))
        (suite/'suite_manifest.json').write_text(json.dumps(dict(specs=[dict(key='empty/remani_shared',scene='empty',variant='remani_shared',repeat=1)])))
        shared=dict(passed=True,validated_transports=3,native_reference_segments=2,local_approved_records=0,warnings=[],errors=[])
        with patch('audit_shared_collision.audit',return_value=shared):rows=reporting.report(suite,render=False)
        self.assertFalse(rows[0]['shared_input_contract_pass'])
        self.assertTrue(rows[0]['shared_input_recorded_checks_pass'])
        self.assertEqual(rows[0]['shared_input_status'],'insufficient_coverage')
        self.assertIn('no approved local shared execution',rows[0]['shared_input_coverage_gaps'])

    def test_shared_input_audit_exception_is_retained_without_crashing_report(self):
        suite=self.root/'suite';suite.mkdir();source=fixture(suite/'empty/remani_shared')
        (source/'manifest.json').write_text(json.dumps(dict(mode='remani',shared_collision=True)))
        (suite/'suite_manifest.json').write_text(json.dumps(dict(specs=[dict(key='empty/remani_shared',scene='empty',variant='remani_shared',repeat=1)])))
        with patch('audit_shared_collision.audit',side_effect=RuntimeError('synthetic incomplete audit data')):
            rows=reporting.report(suite,render=False)
        self.assertFalse(rows[0]['shared_input_contract_pass'])
        self.assertEqual(rows[0]['shared_input_status'],'audit_failed')
        self.assertIn('shared_input_audit_error',rows[0])
        self.assertEqual(json.loads((suite/'report_status.json').read_text())['errors'][0]['stage'],'shared_input_audit')


if __name__=='__main__':unittest.main()

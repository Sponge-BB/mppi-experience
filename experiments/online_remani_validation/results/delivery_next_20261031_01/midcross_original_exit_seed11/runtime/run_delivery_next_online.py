#!/usr/bin/env python3
"""Four isolated ROS SIL runs: common scaling/cusp fix, exit policy A/B only."""
import argparse
import ast
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_delivery_next as builder
import run_coupled_objective as lifecycle
from native_collision_runtime import replace_once
from run_cheap_scaling_online import clean_environment as previous_environment
from run_cheap_scaling_online import patch_trial as patch_common
from run_native_collision_3d import trial_kwargs
from run_sphere_scenarios import register_scenes

HERE=Path(__file__).resolve().parent
DEFAULT=HERE/'results/delivery_next_20261031_01'
RESERVE=256*1024**2
TRIAL_ALLOWANCE=40*1024**2
BUDGET=1.2

# Generated into each NEW runtime only. One immutable bound frame is retained
# at an actual phase transition or sustained non-goal stop. Disk serialization
# happens on shutdown, not on a solver/callback/control critical path.
CAPTURE_SOURCE=r'''"""Event-only cusp telemetry and at most one complete bound collision input."""
import ctypes as ct
import hashlib
import json
import time
import numpy as np

class Capture:
    def __init__(self,node):
        self.node=node;self.events=[];self.previous=None;self.saved=None;self.closed=False
        self.ready=hasattr(node.lib,'online_cusp_phase_debug')
        if not self.ready:raise RuntimeError('required online_cusp_phase_debug ABI missing')
        node.lib.online_cusp_phase_debug.argtypes=[ct.c_void_p,ct.POINTER(ct.c_double)]
        node.lib.online_cusp_phase_debug.restype=ct.c_int

    def sample(self,cycle,now,state,reference,observations,age,field,result,controls,reason,guide):
        if self.closed:return
        values=np.zeros(14,dtype=np.float64)
        code=int(self.node.lib.online_cusp_phase_debug(self.node.handle,values.ctypes.data_as(ct.POINTER(ct.c_double))))
        if code!=14 or values[0]!=1:return
        # Field names/signature are validated against the child header by the
        # runner. The event key is reference identity plus measured phase.
        key=(getattr(guide,'epoch',None),getattr(guide,'revision',None),
            int(values[12]),int(values[13]),int(values[1]),int(values[9]))
        changed=self.previous is not None and key!=self.previous
        phase_changed=changed and key[:4]==self.previous[:4]
        if self.previous is None or changed:
            self.events.append(dict(event='cusp_phase',mono=time.monotonic(),t=now-1000,
                cycle_id=cycle,epoch=key[0],revision=key[1],debug=values.tolist(),
                debug_columns=['active','phase','phase_count','begin','end','gear','end_distance',
                    'end_yaw_error','remaining_arc','transitions','event','confirm','hash_high','hash_low'],
                measured_phase_transition=phase_changed,reason=reason))
        self.previous=key
        stopped=getattr(self.node,'stall_streak',0)>=30
        trigger='measured_phase_transition' if phase_changed else 'sustained_non_goal_stop' if stopped else None
        payload=getattr(self.node,'delivery_next_solve_payload',None)
        if self.saved is not None or trigger is None or payload is None:return
        if payload[20]!=1 or state is None or reference is None:return
        self.saved=dict(payload=payload,state=state.copy(),reference=reference.copy(),
            observations=observations.copy(),field=field.copy(),result=result.copy(),controls=controls.copy(),
            metadata=dict(event=trigger,cycle_id=cycle,t=now-1000,mono=time.monotonic(),
                solve_ros_time=getattr(self.node,'delivery_next_solve_clock',None),
                legacy_observed_obstacle_age=age,
                native_dynamic_age_of_bound_payload=max(0.,float(self.node.delivery_next_solve_clock-payload[18])),
                kernel_age_argument_recorded=False,
                epoch=key[0],revision=key[1],cusp_debug=values.tolist(),reason=reason,
                grid=[self.node.nx,self.node.ny,self.node.gx,self.node.gy,self.node.res],
                complete_native3d_esdf=True,exact_bound_solve_frame=True,
                rng_and_sampler_history_recorded=False,
                result_controls_semantics='final publication/validation choice; not necessarily raw kernel winner',
                scope='one exact bound native3d collision snapshot and kernel inputs; not bitwise MPPI decision replay'))

    def close(self):
        if self.closed:return
        self.closed=True
        with (self.node.out/'cusp_phase_events.jsonl').open('w') as stream:
            for event in self.events:stream.write(json.dumps(event,allow_nan=False)+'\n')
        summary=dict(recorded=self.saved is not None,maximum_snapshots=1,
            serialization='on shutdown only; no per-frame ESDF disk export',events=len(self.events))
        if self.saved is not None:
            sample=self.saved;metadata=sample.pop('metadata')
            path=self.node.out/'native3d_event_input.npz'
            np.savez(path,**sample)
            metadata['npz_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
            (self.node.out/'native3d_event_input.json').write_text(json.dumps(metadata,indent=2)+'\n')
            summary.update(trigger=metadata['event'],file=path.name,bytes=path.stat().st_size)
        (self.node.out/'delivery_event_capture.json').write_text(json.dumps(summary,indent=2)+'\n')

def record(node,*args):
    if not hasattr(node,'delivery_event_capture'):
        import rospy
        node.delivery_event_capture=Capture(node)
        rospy.on_shutdown(node.delivery_event_capture.close)
    node.delivery_event_capture.sample(*args)
'''


@contextmanager
def clean_environment():
    prefixes=('MPPI_CUSP_', 'NATIVE_CUSP_')
    saved={k:v for k,v in os.environ.items() if k.startswith(prefixes)}
    old_floor=lifecycle.HARD_FREE_BYTES
    try:
        for key in saved:del os.environ[key]
        with previous_environment():
            lifecycle.HARD_FREE_BYTES=RESERVE
            yield
    finally:
        lifecycle.HARD_FREE_BYTES=old_floor
        for key in list(os.environ):
            if key.startswith(prefixes):del os.environ[key]
        os.environ.update(saved)


def patch_event_sources(sources):
    changed=dict(sources)
    # Only retain a reference to the immutable array actually bound at solver
    # entry. No map copy/hash/serialization is added to bind_collision.
    changed['native_collision_runtime.py']=replace_once(changed['native_collision_runtime.py'],
        "    code = node.lib.online_set_native_collision_clock(node.handle, now)\n    entry['clock_code'] = int(code)\n",
        "    if phase == 'solve':\n"
        "        node.delivery_next_solve_payload=payload\n"
        "        node.delivery_next_solve_clock=now\n"
        "    code = node.lib.online_set_native_collision_clock(node.handle, now)\n"
        "    entry['clock_code'] = int(code)\n")
    changed['local_execution.py']=replace_once(changed['local_execution.py'],
        '        node.stall_streak=node.stall_streak+1 if stalled else 0\n',
        '        node.stall_streak=node.stall_streak+1 if stalled else 0\n'
        '        if online_step_invoked and guide is not None:\n'
        '            from delivery_next_capture import record as record_delivery_event\n'
        '            record_delivery_event(node,cycle_id,now,state,reference,solve_obstacles,solve_age,\n'
        '                delivery_solve_field,result,controls,reason,guide)\n')
    changed['local_execution.py']=replace_once(changed['local_execution.py'],
        '        solve_odom_stamp=odom_stamp\n',
        '        solve_odom_stamp=odom_stamp\n'
        '        delivery_solve_field=field\n')
    # Flush deterministically before native handle destruction, rather than
    # depending solely on rospy shutdown callback ordering.
    changed['local_execution.py']=replace_once(changed['local_execution.py'],
        '    node.lib.online_destroy(node.handle)\n',
        "    if hasattr(node,'delivery_event_capture'):node.delivery_event_capture.close()\n"
        '    node.lib.online_destroy(node.handle)\n')
    changed['delivery_next_capture.py']=CAPTURE_SOURCE
    for source in changed.values():ast.parse(source)
    return changed


def set_env(node,key,value):
    found=node.findall("env[@name='%s']" % key)
    if len(found)>1:raise ValueError('Ambiguous environment '+key)
    if found:found[0].set('value',str(value))
    else:ET.SubElement(node,'env',name=key,value=str(value))


def patch_trial(out,settings,scene,early_exit):
    out=Path(out)
    patch_common(out,settings,scene,True)
    tree=ET.parse(out/'experiment.launch')
    nodes={node.get('name'):node for node in tree.getroot().findall('node')}
    set_env(nodes['remani_planner_node'],'DELIVERY_EARLY_EXIT',int(early_exit))
    set_env(nodes['base_mppi'],'MPPI_CUSP_PHASE_ASSOCIATION',1)
    tree.write(out/'experiment.launch',encoding='unicode')
    names=('native_collision_runtime.py','local_execution.py')
    sources={name:(out/'runtime'/name).read_text() for name in names}
    changed=patch_event_sources(sources)
    for name,text in changed.items():(out/'runtime'/name).write_text(text)
    shutil.copy2(Path(__file__),out/'runtime'/Path(__file__).name)
    settings.update(delivery_early_exit=bool(early_exit),mppi_cusp_phase_association=True,
        cheap_variable_scaling=True,common_budget_s=BUDGET,early_gate_inside_budget=True)
    (out/'coupling_parameters.json').write_text(json.dumps(settings,ensure_ascii=False,indent=2)+'\n')
    path=out/'manifest.json';manifest=json.loads(path.read_text())
    manifest.update(coupling_experiment=settings,parameters_sha256=lifecycle.sha(out/'coupling_parameters.json'))
    for name in (*changed,Path(__file__).name):
        manifest['experiment_sources_sha256'][name]=lifecycle.sha(out/'runtime'/name)
    manifest['delivery_next_online']=dict(early_exit=bool(early_exit),same_binary=True,
        common_cheap_scaling=True,common_cusp_phase_association=True,
        only_group_difference='DELIVERY_EARLY_EXIT',solver_prepare_budget_s=BUDGET,
        group_A_final_gate_outside_solver_budget=True,group_B_live_gate_inside_solver_budget=True,
        postprocessing_native_gate_still_measured=True,hard_deadline_guaranteed=False,
        native_final_gate_unchanged=True,FSM_latest_gate_unchanged=True,
        cusp_scope='measured reference phase association only; not proposal/objective redesign',
        event_capture_maximum_complete_inputs=1,event_capture_serialization='shutdown only',
        clocks='actual monotonic event times; buffered early checks use explicit check_begin/end_mono_s',
        numerical_ROS_SIL_only=True)
    path.write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')


def prepare_one(build,out,scene,early_exit,duration=55.):
    register_scenes()
    settings=lifecycle.prepare_trial(out,build,'sphere_timing_'+scene,'engineering',11,duration,0.,0.,False)
    patch_trial(out,settings,scene,early_exit)
    return out


def run_batch(build,out,gate_path):
    build,out=Path(build).resolve(),Path(out).resolve()
    register_scenes();manifest=builder.check(build)
    if manifest.get('prepared_only') or not manifest.get('delivery_next_experiment'):
        raise ValueError('A compiled delivery-next child is required')
    gate=Path(gate_path).resolve();evidence=json.loads(gate.read_text())
    if evidence.get('online_authorized') is not True:raise ValueError('Root online authorization gate required')
    digest=lifecycle.sha(build/'build_manifest.json')
    if evidence.get('build_manifest_sha256')!=digest:raise ValueError('Gate refers to a different build')
    out.mkdir(parents=True,exist_ok=False)
    jobs=[dict(scene=scene,early_exit=enabled,duration_s=55.,
        trial='%s_%s_seed11' % (scene,'early_exit' if enabled else 'original_exit'))
        for index,scene in enumerate(('static','midcross'))
        for enabled in ((False,True) if index==0 else (True,False))]
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),build_manifest_sha256=digest,
        gate=str(gate),gate_sha256=lifecycle.sha(gate),jobs=jobs,seed=11,goal=[6.,0.],
        groups=['original_exit','early_exit'],common_cusp_fix=True,common_cheap_scaling=True,
        shared_prepare_solver_and_live_gate_budget_s=BUDGET,hard_deadline_guaranteed=False,
        original_initializer=True,spheres=67,wheel_omega=4.1,wheel_alpha=4.25,
        storage_reserve_mib=256,per_trial_allowance_mib=40,all_outcomes_retained=True,
        online=True,paused=False,hardware=False,gazebo=False)
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    completed=[];runtime_sources=None
    try:
        for job in jobs:
            with clean_environment():
                lifecycle.check_space(out,RESERVE+TRIAL_ALLOWANCE)
                trial=out/job['trial']
                valid=lifecycle.run_trial(trial,build,'sphere_timing_'+job['scene'],'engineering',11,
                    job['duration_s'],0.,0.,False,digest,expected_runtime_sources=runtime_sources,
                    reserve_bytes=RESERVE+TRIAL_ALLOWANCE,
                    prepare_hook=lambda p,s,j=job:patch_trial(p,s,j['scene'],j['early_exit']),**trial_kwargs(manifest))
            sources=json.loads((trial/'manifest.json').read_text())['experiment_sources_sha256']
            if runtime_sources is None:runtime_sources=sources
            result=json.loads((trial/'result.json').read_text())
            completed.append(dict(**job,valid_interface=valid,reason=result['reason']))
            (out/'progress.json').write_text(json.dumps(completed,indent=2)+'\n')
            if not valid:raise RuntimeError('Invalid run retained; inspect before continuing')
        (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=completed),indent=2)+'\n')
    except Exception as error:
        (out/'stopped.json').write_text(json.dumps(dict(completed=completed,reason=str(error)),indent=2)+'\n')
        raise
    return completed


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,default=builder.DEFAULT)
    parser.add_argument('--out',type=Path,default=DEFAULT)
    parser.add_argument('--gate',type=Path)
    parser.add_argument('--prepare-only',action='store_true')
    parser.add_argument('--scene',choices=('static','midcross'),default='static')
    parser.add_argument('--early-exit',action='store_true')
    parser.add_argument('--duration',type=float,default=55.)
    args=parser.parse_args()
    if args.prepare_only:
        builder.check(args.build)
        print(prepare_one(args.build.resolve(),args.out.resolve(),args.scene,args.early_exit,args.duration))
    else:
        if args.gate is None:parser.error('--gate required for execution')
        print(run_batch(args.build,args.out,args.gate))

#!/usr/bin/env python3
"""Eight fresh numerical ROS SIL trials; same binary, receiver OFF/ON."""
from datetime import datetime, timezone
import ast
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

TOOLS=Path(__file__).resolve().parent
OUT=TOOLS.parent
HERE=TOOLS.parents[2]
sys.path.insert(0,str(HERE))
import run_delivery_next_online as common
import build_state_handoff as builder
from native_collision_runtime import replace_once
from run_native_collision_3d import trial_kwargs


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def patch(trial,settings,scene,independent):
    common.patch_trial(trial,settings,scene,True)
    tree=ET.parse(trial/'experiment.launch')
    planner=next(n for n in tree.getroot().findall('node') if n.get('name')=='remani_planner_node')
    common.set_env(planner,'INDEPENDENT_ODOM_RECEIVER',int(independent))
    tree.write(trial/'experiment.launch',encoding='unicode')
    # Same event-only capture in both arms, first actual adopted replan only.
    path=trial/'runtime/delivery_next_capture.py';text=path.read_text()
    text=replace_once(text,'        phase_changed=changed and key[:4]==self.previous[:4]\n',
        '        phase_changed=changed and key[:4]==self.previous[:4]\n'
        '        reference_changed=changed and key[:2]!=self.previous[:2]\n')
    text=replace_once(text,'                measured_phase_transition=phase_changed,reason=reason))\n',
        '                measured_phase_transition=phase_changed,reason=reason,\n'
        '                solve_state=state.tolist() if state is not None else None,\n'
        '                guide_start=guide.start,guide_age=now-guide.start))\n')
    text=replace_once(text,
        "        trigger='measured_phase_transition' if phase_changed else 'sustained_non_goal_stop' if stopped else None\n",
        "        trigger='first_replan_reference_adoption' if reference_changed and key[0]>=2 else \\\n"
        "            'measured_phase_transition' if phase_changed else 'sustained_non_goal_stop' if stopped else None\n")
    text=replace_once(text,
        '            observations=observations.copy(),field=field.copy(),result=result.copy(),controls=controls.copy(),\n',
        '            observations=observations.copy(),field=field.copy(),result=result.copy(),controls=controls.copy(),\n'
        '            reference_times=__import__("numpy").concatenate(guide.times),\n')
    text=replace_once(text,
        '                epoch=key[0],revision=key[1],cusp_debug=values.tolist(),reason=reason,\n',
        '                epoch=key[0],revision=key[1],cusp_debug=values.tolist(),reason=reason,\n'
        '                guide_start=guide.start,guide_age=now-guide.start,\n')
    ast.parse(text);path.write_text(text)
    settings.update(independent_odom_receiver=bool(independent),
        state_receiver_group='B' if independent else 'A',
        immutable_solver_boundary=True,live_handoff_state_refreshed=bool(independent),
        new_acceptance_threshold=False,original_initialization=True)
    save(trial/'coupling_parameters.json',settings)
    mp=trial/'manifest.json';m=json.loads(mp.read_text())
    m['coupling_experiment']=settings
    m['parameters_sha256']=common.lifecycle.sha(trial/'coupling_parameters.json')
    m['experiment_sources_sha256']['delivery_next_capture.py']=common.lifecycle.sha(path)
    m['state_handoff_ab']=dict(group='B' if independent else 'A',same_child_binary=True,
        only_group_switch='INDEPENDENT_ODOM_RECEIVER',receiver_thread_writes_only_immutable_mailbox=True,
        planner_and_model_single_threaded=True,solver_boundary_frozen=True,
        original_callbacks_and_direction_rules=True,acceptance_thresholds_unchanged=True,
        FSM_safety_timers_still_share_default_queue=True,
        capture='event only, at most one full adoption frame, shutdown serialization',
        scope='fixed-arm full REMANI+MPPI numerical ROS SIL; not Gazebo/hardware')
    save(mp,m)


def main():
    if (OUT/'plan.json').exists():raise RuntimeError('Batch exists; never overwrite a measured run')
    common.register_scenes();m=builder.check()
    if m['prepared_only']:raise RuntimeError('Build incomplete')
    digest=common.lifecycle.sha(builder.OUT/'build_manifest.json')
    jobs=[]
    for scene,order in [('static',[('A',1),('B',1),('B',2),('A',2)]),
                        ('midcross',[('B',1),('A',1),('A',2),('B',2)])]:
        for group,repeat in order:
            jobs.append(dict(scene=scene,group=group,repeat=repeat,seed=11,duration_s=55.,
                trial='%s_%s_repeat%02d_seed11'%(scene,group,repeat)))
    plan=dict(created_utc=datetime.now(timezone.utc).isoformat(),jobs=jobs,
        purpose='Isolate state reception latency from slow optimization and measure actual handoff',
        build=str(builder.OUT),build_manifest_sha256=digest,
        runner_sha256=common.lifecycle.sha(Path(__file__)),
        budget_s=1.2,spheres=67,wheel_omega=4.1,wheel_alpha=4.25,goal=[6.,0.],
        early_feasible_exit=True,cusp_association=True,original_initializer=True,
        experimental_factor='independent odom inbox + main-thread consume after solve',
        same_binary=True,same_runtime_sources=True,search_recording=False,
        predefined_gif_selection='repeat01 of each scene, A and B side by side',
        outcomes_all_retained=True,scope='numerical ROS SIL, fixed arm, full REMANI+MPPI',
        github_upload_remains_paused=True,production_modified=False)
    save(OUT/'plan.json',plan)
    completed=[];runtime=None
    try:
        for job in jobs:
            trial=OUT/job['trial']
            with common.clean_environment():
                valid=common.lifecycle.run_trial(trial,builder.OUT,'sphere_timing_'+job['scene'],
                    'engineering',11,55.,0.,0.,False,digest,expected_runtime_sources=runtime,
                    reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
                    prepare_hook=lambda p,s,j=job:patch(p,s,j['scene'],j['group']=='B'),
                    **trial_kwargs(m))
            runtime=json.loads((trial/'manifest.json').read_text())['experiment_sources_sha256']
            result=json.loads((trial/'result.json').read_text())
            completed.append(dict(**job,valid_interface=valid,reason=result['reason']))
            save(OUT/'progress.json',completed)
            print('STATE_AB_PROGRESS',len(completed),len(jobs),job['trial'],result['reason'],flush=True)
            if not valid:raise RuntimeError('Interface-invalid run retained; inspect before continuing')
        builder.check();save(OUT/'complete.json',dict(complete=True,completed=completed))
        print('STATE_AB_COMPLETE',OUT,flush=True)
    except Exception as e:
        save(OUT/'stopped.json',dict(completed=completed,reason=str(e)))
        raise


if __name__=='__main__':main()

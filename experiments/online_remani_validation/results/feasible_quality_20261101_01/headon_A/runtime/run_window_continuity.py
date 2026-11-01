#!/usr/bin/env python3
"""One unpaused head-on comparison of the old and continuous dynamic objective."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_window_continuity as builder
import run_delivery_feedback as previous
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE=Path(__file__).resolve().parent
OUT=HERE/'results/window_continuity_20261101_01'


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def patch(trial,settings,enabled):
    previous.previous.builder.OUT=builder.OUT
    previous.patch(trial,settings,True)
    tree=ET.parse(trial/'experiment.launch')
    planner=next(n for n in tree.getroot().findall('node') if n.get('name')=='remani_planner_node')
    environment=dict(DYNAMIC_WINDOW_CONTINUITY=int(enabled),CANDIDATE_EXIT_AUDIT=0,
        CANDIDATE_EXIT_RECOVER=0,CANDIDATE_COLLISION_REPAIR=0)
    for key,value in environment.items():common.set_env(planner,key,value)
    tree.write(trial/'experiment.launch',encoding='unicode')
    settings.update(dynamic_window_continuity=bool(enabled),diagnostic_pause=False,
        comparison_group='B' if enabled else 'A',optimization_budget_s=1.2,
        dynamic_objective='fixed CV window environmental increment integral' if enabled else 'previous normalized piece sampling',
        original_native_and_latest_gates_retained=True,production_modified=False)
    save(trial/'coupling_parameters.json',settings)
    path=trial/'manifest.json';manifest=json.loads(path.read_text())
    manifest['coupling_experiment']=settings
    manifest['parameters_sha256']=common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['window_continuity']=dict(enabled=bool(enabled),same_binary=True,
        planner_environment=environment,diagnostic_pause=False,
        physical_spheres=67,optimization_budget_s=1.2,mppi_budget_ms=200.,command_lease_ms=250.,
        scope='fixed-arm complete REMANI+MPPI numerical ROS SIL, unpaused closed loop',
        objective='native static/self integral plus dynamic environment increment clipped to frozen CV window',
        production_modified=False,hardware=False,gazebo=False)
    shutil.copy2(Path(__file__),trial/'runtime'/Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name]=common.lifecycle.sha(Path(__file__))
    save(path,manifest)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--enabled',type=int,choices=(0,1),required=True)
    p.add_argument('--tag',required=True);args=p.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT/'plan.json').exists():
        save(OUT/'plan.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),
            core_problem='slow-layer usable delivery while fast MPPI is moving; fix proven dynamic objective discontinuity first',
            experiment=['saved native input exact check and equal-input old/new continuation',
                        'one unpaused online headon_A/headon_B pair'],
            hypothesis='fixed-window integration removes penalty jumps and permits accepted descent; native/latest gates determine usable delivery',
            optimization_budget_s=1.2,prediction_window_s=3.,physical_spheres=67,
            seed=11,goal=[6.,0.],duration_s=55.,
            unchanged=['native hard checks','MPPI algorithm','frontend','wheel limits','delivery feedback weight 1'],
            previous_paused_trial_is_performance_baseline=False,
            previous_pause_artifact='SIGSTOP followed by 1.22023s wall-clock catch-up without intermediate sensing/control',
            primary_metrics=['same-input line-search descent and native feasibility',
                'request-to-new-reference-validated-control','failed requests',
                'reference adoption residual and following stops','physical contact/gap'],
            online_random_seed_does_not_ensure_identical_asynchronous_inputs=True,
            production_modified=False,hardware=False,gazebo=False,github_upload_remains_paused=True))
    manifest=builder.check();common.register_scenes()
    with common.clean_environment():
        valid=common.lifecycle.run_trial(OUT/args.tag,builder.OUT,
            'sphere_timing_headon','engineering',11,55.,0.,0.,False,
            common.lifecycle.sha(builder.OUT/'build_manifest.json'),
            reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial,settings:patch(trial,settings,bool(args.enabled)),
            **trial_kwargs(manifest))
    print('WINDOW_CONTINUITY_ONLINE_COMPLETE',args.tag,valid,flush=True)
    if not valid:raise RuntimeError('Interface-invalid trial retained')


if __name__=='__main__':main()

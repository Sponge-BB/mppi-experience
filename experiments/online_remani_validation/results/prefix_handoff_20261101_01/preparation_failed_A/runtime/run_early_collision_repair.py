#!/usr/bin/env python3
"""Same one-shot collision repair, triggered late or at its first native reject."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_early_collision_repair as builder
import run_delivery_feedback as previous
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE=Path(__file__).resolve().parent
OUT=HERE/'results/early_collision_repair_20261101_01'


def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def patch(trial,settings,early):
    previous.previous.builder.OUT=builder.OUT
    previous.patch(trial,settings,True)
    tree=ET.parse(trial/'experiment.launch')
    planner=next(n for n in tree.getroot().findall('node') if n.get('name')=='remani_planner_node')
    environment=dict(DYNAMIC_WINDOW_CONTINUITY=1,CANDIDATE_EXIT_AUDIT=0,
        CANDIDATE_EXIT_RECOVER=0,CANDIDATE_COLLISION_REPAIR=1,EARLY_COLLISION_REPAIR=int(early))
    for key,value in environment.items():common.set_env(planner,key,value)
    tree.write(trial/'experiment.launch',encoding='unicode')
    settings.update(dynamic_window_continuity=True,diagnostic_pause=False,
        collision_repair=True,early_collision_repair=bool(early),
        comparison_group='B' if early else 'A',optimization_budget_s=1.2,
        repair_factor=10.,repair_attempt_limit=1,
        original_native_and_latest_gates_retained=True,production_modified=False)
    save(trial/'coupling_parameters.json',settings)
    path=trial/'manifest.json';manifest=json.loads(path.read_text())
    manifest['coupling_experiment']=settings
    manifest['parameters_sha256']=common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['early_collision_repair']=dict(enabled=bool(early),repair_enabled_in_both=True,
        same_binary=True,planner_environment=environment,diagnostic_pause=False,
        trigger=('first actual accepted iterate passing wheels but failing full native environmental check'
                 if early else 'final native environmental rejection after ordinary optimizer stop'),
        repair='existing one-shot failed-piece environment weight10 continuation with new L-BFGS history',
        shared_original_deadline_s=1.2,deadline_reset_between_phases=False,
        trigger_is_safe_certificate=False,physical_spheres=67,
        mppi_budget_ms=200.,command_lease_ms=250.,
        scope='fixed-arm complete REMANI+MPPI numerical ROS SIL, unpaused closed loop',
        half_speed_check='existing added experimental acceptance contract, not an original REMANI constraint',
        production_modified=False,hardware=False,gazebo=False)
    shutil.copy2(Path(__file__),trial/'runtime'/Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name]=common.lifecycle.sha(Path(__file__))
    save(path,manifest)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--early',type=int,choices=(0,1),required=True);p.add_argument('--tag',required=True)
    args=p.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT/'plan.json').exists():
        save(OUT/'plan.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),
            core_problem='slow-layer cost stopping with dynamic infeasibility delays a usable reference for moving MPPI',
            hypothesis='existing native checks can transfer the accepted colliding iterate to feasibility repair earlier, leaving meaningful time under the same deadline',
            comparison='A late repair enabled / B early repair enabled; same repair factor10 and original 1.2s deadline',
            experiment=['one complete saved-input old/new continuation pair within .831149821s remaining',
                        'one unpaused headon_A/headon_B closed-loop pair'],
            no_fixed_time_split=True,no_additional_budget=True,repair_attempt_limit=1,
            original_budget_s=1.2,physical_spheres=67,prediction_window_s=3.,
            seed=11,goal=[6.,0.],duration_s=55.,
            unchanged=['fixed-window dynamic objective','native hard checks','MPPI algorithm','frontend','wheel limits','delivery feedback weight1','repair factor10'],
            early_collision_is_not_proof_normal_phase_would_fail=True,
            half_speed_contract_necessity_not_demonstrated=True,
            online_seed_does_not_ensure_identical_asynchronous_inputs=True,
            primary_metrics=['same-input native pass within deadline','repair trigger and elapsed budget',
                'first request to validated new-reference control','failed requests',
                'adoption residual and following stops','actual path and physical gap'],
            production_modified=False,hardware=False,gazebo=False,github_upload_remains_paused=True))
    manifest=builder.check();common.register_scenes()
    with common.clean_environment():
        valid=common.lifecycle.run_trial(OUT/args.tag,builder.OUT,
            'sphere_timing_headon','engineering',11,55.,0.,0.,False,
            common.lifecycle.sha(builder.OUT/'build_manifest.json'),
            reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial,settings:patch(trial,settings,bool(args.early)),
            **trial_kwargs(manifest))
    print('EARLY_COLLISION_REPAIR_ONLINE_COMPLETE',args.tag,valid,flush=True)
    if not valid:raise RuntimeError('Interface-invalid trial retained')


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Two unpaused SIL runs: slow dual-speed admission versus nominal-only admission."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml
import build_half_speed_contract as builder
import run_delivery_feedback as previous
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE=Path(__file__).resolve().parent
OUT=HERE/'results/half_speed_contract_20261101_01'


def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


def patch(trial,settings,nominal_only):
    previous.previous.builder.OUT=builder.OUT
    previous.patch(trial,settings,True)
    parameters=yaml.safe_load((trial/'parameters.yaml').read_text())
    scale=parameters.get('/remani_planner_node/dynamic_point_flow/active_resume_scale',1.)
    if abs(float(scale)-1.)>1e-12:raise ValueError('This nominal-clock ablation requires the common configured resume scale=1')
    tree=ET.parse(trial/'experiment.launch')
    planner=next(n for n in tree.getroot().findall('node') if n.get('name')=='remani_planner_node')
    environment=dict(DYNAMIC_WINDOW_CONTINUITY=1,CANDIDATE_EXIT_AUDIT=0,
        CANDIDATE_EXIT_RECOVER=0,CANDIDATE_COLLISION_REPAIR=0,EARLY_COLLISION_REPAIR=0,
        SLOW_REFERENCE_NOMINAL_ONLY=int(nominal_only))
    for key,value in environment.items():common.set_env(planner,key,value)
    tree.write(trial/'experiment.launch',encoding='unicode')
    settings.update(dynamic_window_continuity=True,diagnostic_pause=False,
        collision_repair=False,early_collision_repair=False,
        slow_reference_nominal_only=bool(nominal_only),
        comparison_group='B' if nominal_only else 'A',optimization_budget_s=1.2,
        actual_execution_validator_changed=False,production_modified=False)
    save(trial/'coupling_parameters.json',settings)
    path=trial/'manifest.json';manifest=json.loads(path.read_text())
    manifest['coupling_experiment']=settings
    manifest['parameters_sha256']=common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['half_speed_contract']=dict(nominal_only=bool(nominal_only),same_binary=True,
        planner_environment=environment,diagnostic_pause=False,
        changed='only half-speed hard rejection for slow references at native and latest candidate gates',
        slow_objective_execution_scales=[1.,.5],slow_hard_execution_scales=[1.] if nominal_only else [1.,.5],
        prediction_window_s=3.,physical_spheres=67,optimization_budget_s=1.2,
        original_static_self_wheel_checks_retained=True,latest_current_pose_check_retained=True,
        mppi_rollout_brake_checker_and_library_unchanged=True,mppi_budget_ms=200.,command_lease_ms=250.,
        earlier_collision_repair_disabled_in_both=True,
        scope='fixed-arm complete REMANI+MPPI numerical ROS SIL; contract ablation, not an algorithm-strength claim',
        production_modified=False,hardware=False,gazebo=False)
    shutil.copy2(Path(__file__),trial/'runtime'/Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name]=common.lifecycle.sha(Path(__file__))
    save(path,manifest)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--nominal-only',type=int,choices=(0,1),required=True);p.add_argument('--tag',required=True)
    args=p.parse_args()
    if not args.tag.replace('_','').isalnum():raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT/'plan.json').exists():
        save(OUT/'plan.json',dict(created_utc=datetime.now(timezone.utc).isoformat(),
            core_problem='Does slow fixed-half-clock admission block references although MPPI executes independently checked local controls?',
            experiment=['same recorded historical candidates full/half dynamic replay and actual adopted-reference phase evidence',
                        'one unpaused same-binary headon_A/headon_B pair',
                        'offline half-clock counterfactual on B candidates already passed by actual native nominal gate'],
            comparison='A both-speed hard admission / B nominal-only slow hard admission',
            baseline='continuous-window objective, collision repair disabled in both',
            unchanged=['soft full+half dynamic objective','physical 67 spheres','static/self/wheel gates',
                'latest measured-pose gate','MPPI actual rollout and brake validation','weights and budgets','frontend'],
            prediction_window_s=3.,original_budget_s=1.2,physical_spheres=67,
            seed=11,goal=[6.,0.],duration_s=55.,
            historical_failed_candidate_execution_claim=False,
            changing_contract_does_not_prove_stronger_optimization=True,
            actual_control_safety_must_be_evaluated=True,
            online_seed_does_not_ensure_identical_asynchronous_inputs=True,
            production_modified=False,hardware=False,gazebo=False,github_upload_remains_paused=True))
    manifest=builder.check();common.register_scenes()
    with common.clean_environment():
        valid=common.lifecycle.run_trial(OUT/args.tag,builder.OUT,
            'sphere_timing_headon','engineering',11,55.,0.,0.,False,
            common.lifecycle.sha(builder.OUT/'build_manifest.json'),
            reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial,settings:patch(trial,settings,bool(args.nominal_only)),
            **trial_kwargs(manifest))
    print('HALF_SPEED_CONTRACT_ONLINE_COMPLETE',args.tag,valid,flush=True)
    if not valid:raise RuntimeError('Interface-invalid trial retained')


if __name__=='__main__':main()

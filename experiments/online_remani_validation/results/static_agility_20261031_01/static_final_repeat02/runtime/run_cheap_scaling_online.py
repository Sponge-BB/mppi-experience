#!/usr/bin/env python3
"""Fresh cheap-scaling OFF/ON numerical ROS SIL; private master, no drivers."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml

import build_cheap_scaling as builder
import run_coupled_objective as lifecycle
from native_collision_runtime import replace_once
from run_native_collision_3d import trial_kwargs
from run_native_dynamic_fast import clean_environment as previous_environment
from run_sphere_scenarios import patch_trial as patch_scene, register_scenes

HERE=Path(__file__).resolve().parent
DEFAULT=HERE/'results/cheap_scaling_online_20261030_01'
RESERVE=64*1024**2
TRIAL_ALLOWANCE=18*1024**2
SOLVE_BUDGET=1.2


@contextmanager
def clean_environment():
    prefixes=('CHEAP_', 'CANDIDATE_AUDIT_', 'ENDPOINT_', 'WHEEL_CAUSE_', 'DELIVERY_', 'FEASIBILITY_', 'BUDGET_', 'SOLVER_SCALING_')
    saved={k:v for k,v in os.environ.items() if k.startswith(prefixes)}
    old_floor=lifecycle.HARD_FREE_BYTES
    try:
        for k in saved:del os.environ[k]
        # Storage policy for this isolated batch; all algorithm settings unchanged.
        lifecycle.HARD_FREE_BYTES=RESERVE
        with previous_environment():yield
    finally:
        lifecycle.HARD_FREE_BYTES=old_floor
        for k in list(os.environ):
            if k.startswith(prefixes):del os.environ[k]
        os.environ.update(saved)


def patch_trial(out, settings, scene, enabled):
    out=Path(out)
    patch_scene(out,settings,scene,'b47')
    params_path=out/'parameters.yaml'
    params=yaml.safe_load(params_path.read_text())
    overrides={
        '/remani_planner_node/optimization/max_wall_time':SOLVE_BUDGET,
        '/remani_planner_node/fsm/dynamic_astar_replan_wall_time':SOLVE_BUDGET,
    }
    params.update(overrides)
    if params['/remani_planner_node/mm/mobile_base_max_wheel_omega']!=4.1 or params['/remani_planner_node/mm/mobile_base_max_wheel_alpha']!=4.25:
        raise ValueError('Expected unchanged 4.1/4.25 wheel limits')
    params_path.write_text(yaml.safe_dump(params,sort_keys=True))
    tree=ET.parse(out/'experiment.launch')
    planner=next(n for n in tree.getroot().findall('node') if n.get('name')=='remani_planner_node')
    environment=dict(CHEAP_VARIABLE_SCALING=str(int(enabled)),ENDPOINT_TIME_FIX='0',
        ENDPOINT_TIME_REPLAY='0',WHEEL_CAUSE_REPLAY='0',CANDIDATE_AUDIT_TARGET='',
        CANDIDATE_AUDIT_FREEZE_SOLVER='1',DELIVERY_REPLAY_KIND='',
        DELIVERY_FAST_QUERY='1',DELIVERY_ACCEPT_CANDIDATE='1',FEASIBILITY_STOP_GATE='0',FAILURE_PROFILE_DISABLED='1')
    for key,value in environment.items():
        current=planner.findall("env[@name='%s']" % key)
        if current:current[0].set('value',value)
        else:ET.SubElement(planner,'env',name=key,value=value)
    tree.write(out/'experiment.launch',encoding='unicode')
    # Existing timing capture already records actual first kernel/validated use.
    # Add receipt/export clocks only at reference events, not a new loop probe.
    path=out/'runtime/local_execution.py';text=path.read_text()
    text=replace_once(text,"node.reference_events.write(json.dumps({'t':rospy.get_time()-1000,'event':event,",
        "node.reference_events.write(json.dumps({'mono':time.monotonic(),'t':rospy.get_time()-1000,'event':event,")
    text=replace_once(text,"node.reference_log.write(json.dumps({'t':now-1000,'revision':guide.revision,'epoch':guide.epoch,",
        "node.reference_log.write(json.dumps({'cycle_start_monotonic':wall,'export_monotonic':time.monotonic(),'t':now-1000,'revision':guide.revision,'epoch':guide.epoch,")
    path.write_text(text)
    shutil.copy2(Path(__file__),out/'runtime'/Path(__file__).name)
    settings.update(cheap_variable_scaling=bool(enabled),backend_total_budget_s=SOLVE_BUDGET,
        scale_preparation_inside_budget=True,endpoint_time_fix=False,solver_overlay_pin=True,
        diagnostic_pause=False,genuine_online_closed_loop=True)
    (out/'coupling_parameters.json').write_text(json.dumps(settings,ensure_ascii=False,indent=2)+'\n')
    path=out/'manifest.json';m=json.loads(path.read_text())
    m.update(coupling_experiment=settings,parameters_sha256=lifecycle.sha(out/'coupling_parameters.json'))
    m['parameter_overrides'].update(overrides)
    m['experiment_sources_sha256']['local_execution.py']=lifecycle.sha(out/'runtime/local_execution.py')
    m['experiment_sources_sha256'][Path(__file__).name]=lifecycle.sha(out/'runtime'/Path(__file__).name)
    m['cheap_scaling_online']=dict(enabled=bool(enabled),same_binary=True,group_switch=environment,
        total_prepare_plus_solver_budget_s=SOLVE_BUDGET,solver_budget_does_not_include_search_or_acceptance=True,
        original_frontend_initialization=True,full_native_objective_and_gates_unchanged=True,
        overlay_pin_common_to_both_groups=True,entire_map_frozen=False,
        latest_environment_FSM_acceptance_unchanged=True,
        full_candidate_audits=False,leaf_cost_profiling=False,
        native_publish_event='dual_timeline.reference_publish mono',
        reference_receipt='reference_events mono; revision and message_hashes',
        actual_use='state_mppi_cycles online_step_entry_monotonic, first_kernel_cycle_for_reference',
        valid_execution='state_mppi_cycles command_publish_after_monotonic, first_validated_command_cycle_for_reference',
        startup_precheck=scene=='empty',numerical_ROS_SIL_only=True)
    path.write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')


def prepare_one(build,out,scene,enabled,duration):
    register_scenes()
    settings=lifecycle.prepare_trial(out,build,'sphere_timing_'+scene,'engineering',11,duration,0.,0.,False)
    patch_trial(out,settings,scene,enabled)
    return out


def run_batch(build,out,replay_gate):
    build,out=Path(build).resolve(),Path(out).resolve()
    register_scenes()
    manifest=builder.check(build)
    if manifest.get('prepared_only'):raise ValueError('Compile child first')
    gate=Path(replay_gate).resolve()
    evidence=json.loads(gate.read_text())
    if evidence.get('online_authorized') is not True:
        raise ValueError('Root must confirm cheap replay before online execution')
    if evidence.get('helper_sha256')!=lifecycle.sha(HERE/'cheap_variable_scaling.hpp'):
        raise ValueError('Replay gate refers to a different cheap-scaling helper')
    if evidence.get('build_manifest_sha256')!=lifecycle.sha(build/'build_manifest.json'):
        raise ValueError('Replay gate refers to a different online child')
    out.mkdir(parents=True,exist_ok=False)
    jobs=[dict(scene=s,enabled=e,duration_s=24. if s=='empty' else 55.)
          for i,s in enumerate(('empty','static','midcross')) for e in ((False,True) if i%2==0 else (True,False))]
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),
        build_manifest_sha256=lifecycle.sha(build/'build_manifest.json'),
        gate=str(gate),gate_sha256=lifecycle.sha(gate),jobs=jobs,seed=11,goal=[6.,0.],
        groups=['unscaled','cheap'],budget_s=SOLVE_BUDGET,preparation_included=True,
        storage_reserve_mib=64,per_trial_allowance_mib=18,all_outcomes_retained=True,
        online=True,paused=False,hardware=False)
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    completed=[];runtime_sources=None
    try:
        for job in jobs:
            with clean_environment():
                lifecycle.check_space(out,RESERVE+TRIAL_ALLOWANCE)
                name='%s_%s_seed11' % (job['scene'],'cheap' if job['enabled'] else 'unscaled')
                trial=out/name
                valid=lifecycle.run_trial(trial,build,'sphere_timing_'+job['scene'],'engineering',11,
                    job['duration_s'],0.,0.,False,plan['build_manifest_sha256'],
                    expected_runtime_sources=runtime_sources,reserve_bytes=RESERVE+TRIAL_ALLOWANCE,
                    prepare_hook=lambda p,s,j=job:patch_trial(p,s,j['scene'],j['enabled']),**trial_kwargs(manifest))
            current=json.loads((trial/'manifest.json').read_text())['experiment_sources_sha256']
            if runtime_sources is None:runtime_sources=current
            result=json.loads((trial/'result.json').read_text())
            completed.append(dict(**job,trial=name,valid_interface=valid,reason=result['reason']))
            (out/'progress.json').write_text(json.dumps(completed,indent=2)+'\n')
            if not valid:raise RuntimeError('Invalid measured run retained; inspect before continuing')
        (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=completed),indent=2)+'\n')
    except Exception as error:
        (out/'stopped.json').write_text(json.dumps(dict(completed=completed,reason=str(error)),indent=2)+'\n')
        raise
    return completed


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,default=builder.DEFAULT)
    p.add_argument('--out',type=Path,default=DEFAULT);p.add_argument('--replay-gate',type=Path)
    p.add_argument('--prepare-only',action='store_true');p.add_argument('--scene',choices=('empty','static','midcross'),default='empty')
    p.add_argument('--enabled',action='store_true');p.add_argument('--duration',type=float,default=24.)
    a=p.parse_args()
    if a.prepare_only:
        builder.check(a.build);print(prepare_one(a.build.resolve(),a.out.resolve(),a.scene,a.enabled,a.duration))
    else:
        if a.replay_gate is None:p.error('--replay-gate is required for online execution')
        print(run_batch(a.build,a.out,a.replay_gate))

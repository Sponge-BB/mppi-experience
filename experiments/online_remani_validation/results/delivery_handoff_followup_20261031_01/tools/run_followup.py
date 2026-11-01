#!/usr/bin/env python3
"""Two fresh repetitions of the current delivery policy; no algorithm change."""
from datetime import datetime, timezone
import ast
import hashlib
import json
from pathlib import Path
import shutil
import sys

OUT = Path(__file__).resolve().parent.parent
HERE = OUT.parents[1]
sys.path.insert(0, str(HERE))
import run_delivery_next_online as common
from native_collision_runtime import replace_once
from run_native_collision_3d import trial_kwargs

BUILD = HERE/'builds/delivery_next_20261031_01'
OLD = HERE/'results/delivery_next_20261031_01/midcross_early_exit_seed11'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def patch(trial, settings):
    common.patch_trial(trial, settings, 'midcross', True)
    path=trial/'runtime/delivery_next_capture.py'
    text=path.read_text()
    text=replace_once(text,
        '        phase_changed=changed and key[:4]==self.previous[:4]\n',
        '        phase_changed=changed and key[:4]==self.previous[:4]\n'
        '        reference_changed=changed and key[:2]!=self.previous[:2]\n')
    text=replace_once(text,
        '                measured_phase_transition=phase_changed,reason=reason))\n',
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
        '                guide_start=guide.start,guide_age=now-guide.start,\n'
        '                capture_scope="first adopted replan solve input; not a rejected-candidate execution replay",\n')
    ast.parse(text)
    path.write_text(text)
    manifest_path=trial/'manifest.json'
    manifest=json.loads(manifest_path.read_text())
    manifest['experiment_sources_sha256']['delivery_next_capture.py']=common.lifecycle.sha(path)
    manifest['delivery_handoff_followup']=dict(
        current_early_exit=True,current_cusp_association=True,
        algorithm_changes=False,rejected_candidates_not_executed=True,
        only_runtime_difference='event-only observation at first adopted replan',
        snapshot_count_limit=1,snapshot_disk_write='shutdown only',
        complete_sampler_history_recorded=False,hardware=False,gazebo=False)
    baseline=json.loads((OLD/'manifest.json').read_text())
    required=(*common.lifecycle.RUNTIME_CONTRACT,'native_collision_runtime.py')
    mismatches=[name for name in required
        if manifest['experiment_sources_sha256'].get(name)!=baseline['experiment_sources_sha256'].get(name)]
    if mismatches:
        raise RuntimeError('Actual control source differs from previous current-policy run: '+str(mismatches))
    manifest['delivery_handoff_followup']['unchanged_control_sources']=list(required)
    save(manifest_path,manifest)


def main():
    resume='--resume-unstarted' in sys.argv
    if (OUT/'plan.json').exists() and not resume:
        raise RuntimeError('This batch is already prepared; do not overwrite a measured run')
    common.register_scenes()
    manifest=common.builder.check(BUILD)
    digest=common.lifecycle.sha(BUILD/'build_manifest.json')
    jobs=[dict(scene='midcross',early_exit=True,seed=11,duration_s=55.,
        trial='midcross_repeat%02d_seed11'%i) for i in (1,2)]
    plan=dict(created_utc=datetime.now(timezone.utc).isoformat(),
        purpose='Check current-policy delivery repeatability and actual handoff, before changing any gate',
        build=str(BUILD),build_manifest_sha256=digest,jobs=jobs,
        goal=[6.,0.],same_scene=True,same_seed=True,original_current_policy_trial=str(OLD),
        budget_s=1.2,spheres=67,wheel_omega=4.1,wheel_alpha=4.25,
        algorithm_changes=False,all_outcomes_retained=True,
        scope='fixed-arm native REMANI+MPPI ROS numerical SIL; not Gazebo or hardware',
        scope_limit='two fresh repetitions do not establish general statistical reliability',
        runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        github_upload_remains_paused=True)
    if resume:
        original=json.loads((OUT/'plan.json').read_text())
        stopped=json.loads((OUT/'stopped.json').read_text())
        if stopped['completed'] or original['build_manifest_sha256']!=digest:
            raise RuntimeError('Resume applies only to an unchanged, unstarted batch')
        first=OUT/jobs[0]['trial']
        if any((first/name).exists() for name in ('manifest.json','owned_pids.txt','launch_stdout.log','run_status.json')):
            raise RuntimeError('A measured or fully prepared trial must not be moved by this recovery')
        archived=first.with_name(first.name+'_preparation_missing_overlay')
        shutil.move(str(first),str(archived))
        save(OUT/'startup_recovery.json',dict(
            original_plan_sha256=common.lifecycle.sha(OUT/'plan.json'),
            original_stop_sha256=common.lifecycle.sha(OUT/'stopped.json'),
            cause='ROS package overlay not loaded; no ROS process started',
            preserved_directory=archived.name,runner_sha256=plan['runner_sha256'],
            algorithm_outcomes_discarded=0))
    else:
        save(OUT/'plan.json',plan)
    completed=[];runtime=None
    try:
        for job in jobs:
            trial=OUT/job['trial']
            with common.clean_environment():
                valid=common.lifecycle.run_trial(trial,BUILD,'sphere_timing_midcross',
                    'engineering',11,55.,0.,0.,False,digest,
                    expected_runtime_sources=runtime,
                    reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
                    prepare_hook=patch,**trial_kwargs(manifest))
            runtime=json.loads((trial/'manifest.json').read_text())['experiment_sources_sha256']
            result=json.loads((trial/'result.json').read_text())
            completed.append(dict(**job,valid_interface=valid,reason=result['reason']))
            save(OUT/'progress.json',completed)
            if not valid:
                raise RuntimeError('Invalid interface run preserved; inspect before continuing')
        save(OUT/'complete.json',dict(complete=True,completed=completed))
        print('FOLLOWUP_COMPLETE',str(OUT),flush=True)
    except Exception as error:
        save(OUT/'stopped.json',dict(completed=completed,reason=str(error)))
        raise


if __name__=='__main__':
    main()

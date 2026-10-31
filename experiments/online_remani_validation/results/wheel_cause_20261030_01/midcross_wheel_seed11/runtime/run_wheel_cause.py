#!/usr/bin/env python3
"""One fresh seed11 midcross wheel capture, then three paused same-input replays.

Numerical ROS SIL only. Native gates/limits remain unchanged; diagnostic
candidates are not published. Every outcome, including missing capture, is kept.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_candidate_audit as audit
from build_wheel_cause import check as check_child
from run_coupled_objective import HERE, check_build, check_space, sha


BUILD = HERE / 'builds/wheel_cause_20261030_01'
OUT = HERE / 'results/wheel_cause_20261030_01'
TARGET = 'wheel'
SEED = 11
GROUPS = (
    dict(name='original_060', time_scale=1., budget_s=.6),
    dict(name='original_120', time_scale=1., budget_s=1.2),
    dict(name='time150_060', time_scale=1.5, budget_s=.6),
)
PREDICATE = ('first normal 0.6s refresh solve rejected by unchanged native wheel '
             'speed/acceleration acceptance; one fresh midcross seed11 run, '
             'with solver overlay pinned; all three replays use that paused fixed input')
ORIGINAL_PREPARE = audit.prepare_capture
RESERVE_BYTES = 400 * 1024 ** 2
CAPTURE_ALLOWANCE_BYTES = 12 * 1024 ** 2


def set_planner_environment(tree, out):
    values = audit.set_planner_environment(tree, out, TARGET)
    values.update(CANDIDATE_AUDIT_FREEZE_SOLVER='1', WHEEL_CAUSE_REPLAY='1')
    planners = tree.getroot().findall("node[@name='remani_planner_node']")
    if len(planners) != 1:
        raise ValueError('Exactly one private planner required')
    for key in ('CANDIDATE_AUDIT_FREEZE_SOLVER', 'WHEEL_CAUSE_REPLAY'):
        existing = planners[0].findall("env[@name='%s']" % key)
        if len(existing) > 1:
            raise ValueError('Ambiguous planner environment: ' + key)
        if existing:
            existing[0].set('value', values[key])
        else:
            ET.SubElement(planners[0], 'env', name=key, value=values[key])
    return values


def prepare_capture(out, build, target=TARGET, seed=SEED, ordinal=1):
    if target != TARGET or seed != SEED or ordinal != 1:
        raise ValueError('Only the predeclared single wheel seed11 capture is allowed')
    out = Path(out)
    manifest = ORIGINAL_PREPARE(out, build, target, seed, ordinal)
    tree = ET.parse(out / 'experiment.launch')
    environment = set_planner_environment(tree, out)
    tree.write(out / 'experiment.launch', encoding='unicode')
    settings = json.loads((out / 'coupling_parameters.json').read_text())
    settings.update(wheel_cause_experiment=True, capture_predicate=PREDICATE,
        diagnostic_phases=[group['name'] for group in GROUPS],
        diagnostic_budgets_s=[group['budget_s'] for group in GROUPS],
        diagnostic_time_scales=[group['time_scale'] for group in GROUPS],
        wheel_cause_groups=list(GROUPS), solver_overlay_freeze=True,
        boundary_pvaj_unchanged=True, gear_unchanged=True,
        constraints_relaxed=False, native_acceptance_unchanged=True,
        objective_unchanged=True, initialization_unchanged=False,
        initialization_scope='same native seed except prespecified duration-only x1.5 group',
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        scope='three paused same-input wheel-cause treatments; not an online performance comparison')
    (out / 'coupling_parameters.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2) + '\n')
    manifest.update(coupling_experiment=settings,
        parameters_sha256=sha(out / 'coupling_parameters.json'),
        wheel_cause_experiment=True, navigation_metrics_valid=False,
        diagnostic_candidates_published=False)
    manifest['candidate_audit'].update(capture_predicate=PREDICATE,
        planner_environment=environment, solver_overlay_freeze=True)
    manifest['wheel_cause'] = dict(groups=list(GROUPS), same_paused_input=True,
        planner_environment=environment, output='candidate_audit/wheel_cause_replay.jsonl',
        all_outcomes_retained=True, no_outcome_selected_reruns=True,
        boundary_pvaj_unchanged=True, gear_unchanged=True, native_limits_unchanged=True,
        native_acceptance_unchanged=True, diagnostic_candidates_published=False,
        post_failure_replays_outside_online_budget=True,
        scope='cause diagnosis only; neither online timing nor navigation success evidence')
    shutil.copy2(Path(__file__), out / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = sha(Path(__file__))
    (out / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return manifest


def run_capture(out, build):
    previous_prepare = audit.prepare_capture
    previous_environment = audit.lifecycle.diagnostic_environment

    def diagnostic_environment(*args, **kwargs):
        environment = previous_environment(*args, **kwargs)
        # Diagnostic switches are explicit planner launch env, never inherited.
        return {key: value for key, value in environment.items()
                if not key.startswith(('CANDIDATE_AUDIT_', 'WHEEL_CAUSE_'))}

    audit.prepare_capture = prepare_capture
    audit.lifecycle.diagnostic_environment = diagnostic_environment
    try:
        status = audit.run_capture(Path(out), Path(build), TARGET, SEED)
    finally:
        audit.prepare_capture = previous_prepare
        audit.lifecycle.diagnostic_environment = previous_environment
    output = Path(out) / 'candidate_audit/wheel_cause_replay.jsonl'
    status.update(wheel_cause_experiment=True,
        wheel_cause_replay_written=output.is_file(),
        wheel_cause_replay_sha256=sha(output) if output.is_file() else None,
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        scope='one native wheel rejection and paused same-input diagnosis; no online timing claim')
    (Path(out) / 'run_status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2) + '\n')
    if status['target_failure_captured'] and not status['wheel_cause_replay_written']:
        raise RuntimeError('Native target captured but wheel-cause output missing; all output retained')
    return status


def run(build, out):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest = check_child(build)
    check_build(build)
    if not manifest.get('wheel_cause_experiment') or manifest.get('prepared_only'):
        raise ValueError('Completed isolated wheel-cause build required')
    free = check_space(out.parent, RESERVE_BYTES + CAPTURE_ALLOWANCE_BYTES)
    print('STORAGE free_mib=%d reserve_mib=400 capture_allowance_mib=12' %
          (free // 1024 ** 2), flush=True)
    out.mkdir(parents=True, exist_ok=False)
    job = dict(directory='midcross_wheel_seed11', target=TARGET, seed=SEED)
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build / 'build_manifest.json'), trials=[job],
        scene='sphere_timing_midcross', duration_s=55., goal=[6., 0.],
        bounds=[-1.5, 7.5, -2., 2.], actor=[3., -1.6, 0., .18, .23],
        actor_appearance_after_motion_s=2., collision_spheres=67,
        deadline_ms=200., lease_ms=250., refresh_budget_s=.6,
        capture_predicate=PREDICATE, groups=list(GROUPS),
        candidate_audit_freeze_solver=1, wheel_cause_replay=1,
        same_paused_input=True, boundary_pvaj_unchanged=True, gear_unchanged=True,
        native_constraints_unchanged=True, native_acceptance_unchanged=True,
        storage_reserve_mib=400, capture_allowance_mib=12,
        no_outcome_selected_reruns=True, all_outcomes_retained=True,
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        online_performance_report=False,
        limitations=['one fresh seed11 run; target rejection may not occur',
            'groups share the captured paused input, not three independent online runs',
            'only native seed durations or solver budget change; boundary pvaj, gear and limits stay fixed',
            'solver pin and paused replay add diagnostic work; whole-wall timing is contaminated',
            'diagnostic candidates are never published; no live handoff or navigation improvement claim',
            'numerical ROS SIL only; not Gazebo or hardware'])
    (out / 'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2) + '\n')
    print('WHEEL_CAUSE_START', job['directory'], flush=True)
    try:
        status = run_capture(out / job['directory'], build)
    except Exception as error:
        (out / 'stopped.json').write_text(json.dumps(dict(trial=job, reason=str(error),
            failed_output_retained=True, no_rerun=True), ensure_ascii=False, indent=2) + '\n')
        raise
    completed = dict(**job, target_failure_captured=status['target_failure_captured'],
        diagnostic_complete=status['diagnostic_complete'],
        wheel_cause_replay_written=status['wheel_cause_replay_written'],
        candidate_audit_done=status['candidate_audit_done'])
    (out / 'complete.json').write_text(json.dumps(dict(complete=True,
        navigation_metrics_valid=False, online_performance_report=False,
        completed=[completed]), ensure_ascii=False, indent=2) + '\n')
    print('WHEEL_CAUSE_DONE', job['directory'],
          'captured=' + str(status['target_failure_captured']), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    args = parser.parse_args()
    run(args.build, args.out)

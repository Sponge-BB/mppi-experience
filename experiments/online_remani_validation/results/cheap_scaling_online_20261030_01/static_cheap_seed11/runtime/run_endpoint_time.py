#!/usr/bin/env python3
"""One fresh wheel rejection, then original/endpoint-only paused seed replays.

Numerical ROS SIL diagnosis only. Native gates and limits remain unchanged;
neither diagnostic candidate is published or tested against the latest live
delivery environment. All outcomes, including missing capture, are retained.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_candidate_audit as audit
from build_endpoint_time import check as check_child
from run_coupled_objective import HERE, check_build, check_space, sha


BUILD = HERE / 'builds/endpoint_time_20261030_01'
OUT = HERE / 'results/endpoint_time_20261030_01'
TARGET = 'wheel'
SEED = 11
GROUPS = (
    dict(name='original_060', endpoint_piece_time_factor=1., budget_s=.6),
    dict(name='endpoint150_060', endpoint_piece_time_factor=1.5, budget_s=.6),
)
ENDPOINT_RULE = 'multiply unique first/last piece indices within each gear segment once; leave every other piece unchanged'
PREDICATE = ('first normal 0.6s refresh solve rejected by unchanged native wheel '
             'speed/acceleration acceptance; one fresh midcross seed11 run, '
             'solver overlay pinned; both 0.6s replays use that paused fixed input')
OUTPUT = Path('candidate_audit/endpoint_time_replay.jsonl')
ORIGINAL_PREPARE = audit.prepare_capture
RESERVE_BYTES = 250 * 1024 ** 2
CAPTURE_ALLOWANCE_BYTES = 40 * 1024 ** 2


def set_planner_environment(tree, out):
    values = audit.set_planner_environment(tree, out, TARGET)
    diagnostic = dict(CANDIDATE_AUDIT_FREEZE_SOLVER='1', WHEEL_CAUSE_REPLAY='0',
                      ENDPOINT_TIME_REPLAY='1', ENDPOINT_TIME_FIX='0')
    values.update(diagnostic)
    planners = tree.getroot().findall("node[@name='remani_planner_node']")
    if len(planners) != 1:
        raise ValueError('Exactly one private planner required')
    for key, value in diagnostic.items():
        existing = planners[0].findall("env[@name='%s']" % key)
        if len(existing) > 1:
            raise ValueError('Ambiguous planner environment: ' + key)
        if existing:
            existing[0].set('value', value)
        else:
            ET.SubElement(planners[0], 'env', name=key, value=value)
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
    settings.update(endpoint_time_experiment=True, capture_predicate=PREDICATE,
        diagnostic_phases=[group['name'] for group in GROUPS],
        diagnostic_budgets_s=[group['budget_s'] for group in GROUPS],
        endpoint_time_groups=list(GROUPS), endpoint_piece_rule=ENDPOINT_RULE,
        solver_overlay_freeze=True, boundary_pvaj_unchanged=True,
        gear_unchanged=True, constraints_relaxed=False,
        native_acceptance_unchanged=True, objective_unchanged=True,
        initialization_unchanged=False,
        initialization_scope='only unique first/last piece durations per gear segment are multiplied by 1.5 in the treatment',
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        latest_environment_acceptance_exercised=False,
        delivery_improvement_demonstrated=False,
        scope='two paused same-input endpoint-time treatments; not online performance or delivery evidence')
    (out / 'coupling_parameters.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2) + '\n')
    manifest.update(coupling_experiment=settings,
        parameters_sha256=sha(out / 'coupling_parameters.json'),
        endpoint_time_experiment=True, navigation_metrics_valid=False,
        diagnostic_candidates_published=False,
        latest_environment_acceptance_exercised=False)
    manifest['candidate_audit'].update(capture_predicate=PREDICATE,
        planner_environment=environment, solver_overlay_freeze=True)
    manifest['endpoint_time'] = dict(groups=list(GROUPS), endpoint_piece_rule=ENDPOINT_RULE,
        same_paused_input=True, planner_environment=environment, output=str(OUTPUT),
        all_outcomes_retained=True, no_outcome_selected_reruns=True,
        boundary_pvaj_unchanged=True, gear_unchanged=True, native_limits_unchanged=True,
        native_acceptance_unchanged=True, diagnostic_candidates_published=False,
        post_failure_replays_outside_online_budget=True,
        latest_environment_acceptance_exercised=False,
        scope='diagnosis only; latest-environment live acceptance and delivery were not exercised')
    shutil.copy2(Path(__file__), out / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = sha(Path(__file__))
    (out / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    return manifest


def run_capture(out, build):
    previous_prepare = audit.prepare_capture
    previous_environment = audit.lifecycle.diagnostic_environment
    previous_space_check = audit.lifecycle.check_space

    def diagnostic_environment(*args, **kwargs):
        environment = previous_environment(*args, **kwargs)
        return {key: value for key, value in environment.items()
                if not key.startswith(('CANDIDATE_AUDIT_', 'WHEEL_CAUSE_', 'ENDPOINT_TIME_'))}

    audit.prepare_capture = prepare_capture
    audit.lifecycle.diagnostic_environment = diagnostic_environment
    audit.lifecycle.check_space = lambda path: previous_space_check(path, RESERVE_BYTES)
    try:
        status = audit.run_capture(Path(out), Path(build), TARGET, SEED)
    finally:
        audit.prepare_capture = previous_prepare
        audit.lifecycle.diagnostic_environment = previous_environment
        audit.lifecycle.check_space = previous_space_check
    output = Path(out) / OUTPUT
    status.update(endpoint_time_experiment=True,
        endpoint_time_replay_written=output.is_file(),
        endpoint_time_replay_sha256=sha(output) if output.is_file() else None,
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        latest_environment_acceptance_exercised=False,
        delivery_improvement_demonstrated=False,
        scope='one native wheel rejection and paused same-input diagnosis; no live delivery or timing claim')
    (Path(out) / 'run_status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2) + '\n')
    if status['target_failure_captured'] and not status['endpoint_time_replay_written']:
        raise RuntimeError('Native target captured but endpoint-time output missing; all output retained')
    return status


def run(build, out):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest = check_child(build)
    check_build(build)
    if not manifest.get('endpoint_time_experiment') or manifest.get('prepared_only'):
        raise ValueError('Completed isolated endpoint-time build required')
    free = check_space(out.parent, RESERVE_BYTES + CAPTURE_ALLOWANCE_BYTES)
    print('STORAGE free_mib=%d reserve_mib=250 capture_allowance_mib=40' %
          (free // 1024 ** 2), flush=True)
    out.mkdir(parents=True, exist_ok=False)
    job = dict(directory='midcross_wheel_seed11', target=TARGET, seed=SEED)
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build / 'build_manifest.json'), trials=[job],
        scene='sphere_timing_midcross', duration_s=55., goal=[6., 0.],
        bounds=[-1.5, 7.5, -2., 2.], actor=[3., -1.6, 0., .18, .23],
        actor_appearance_after_motion_s=2., collision_spheres=67,
        deadline_ms=200., lease_ms=250., refresh_budget_s=.6,
        capture_predicate=PREDICATE, groups=list(GROUPS), endpoint_piece_rule=ENDPOINT_RULE,
        candidate_audit_freeze_solver=1, endpoint_time_replay=1,
        wheel_cause_replay=0, normal_capture_endpoint_time_fix=0,
        same_paused_input=True, boundary_pvaj_unchanged=True, gear_unchanged=True,
        native_constraints_unchanged=True, native_acceptance_unchanged=True,
        storage_reserve_mib=250, capture_allowance_mib=40,
        historical_files_compressed=False, historical_files_deleted=False,
        storage_policy_note='40MiB is an estimate for this two-group capture; historical outcomes are retained unmodified',
        no_outcome_selected_reruns=True, all_outcomes_retained=True,
        diagnostic_candidates_published=False, navigation_metrics_valid=False,
        online_performance_report=False, latest_environment_acceptance_exercised=False,
        delivery_improvement_demonstrated=False,
        limitations=['one fresh seed11 run; target rejection may not occur',
            'two groups share a paused captured input; they are not independent online runs',
            'both solver budgets remain 0.6s; only unique endpoint-piece seed durations change',
            'p/v/a/j boundaries, gear signs, constraints and native gates stay unchanged',
            'diagnostic candidates are never published; latest-environment acceptance and live handoff are not exercised',
            'solver pin and paused audits contaminate whole-wall timing; no online performance claim',
            'numerical ROS SIL only; not Gazebo or hardware'])
    (out / 'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2) + '\n')
    print('ENDPOINT_TIME_START', job['directory'], flush=True)
    try:
        status = run_capture(out / job['directory'], build)
    except Exception as error:
        (out / 'stopped.json').write_text(json.dumps(dict(trial=job, reason=str(error),
            failed_output_retained=True, no_rerun=True), ensure_ascii=False, indent=2) + '\n')
        raise
    completed = dict(**job, target_failure_captured=status['target_failure_captured'],
        diagnostic_complete=status['diagnostic_complete'],
        endpoint_time_replay_written=status['endpoint_time_replay_written'],
        candidate_audit_done=status['candidate_audit_done'])
    (out / 'complete.json').write_text(json.dumps(dict(complete=True,
        navigation_metrics_valid=False, online_performance_report=False,
        latest_environment_acceptance_exercised=False,
        completed=[completed]), ensure_ascii=False, indent=2) + '\n')
    print('ENDPOINT_TIME_DONE', job['directory'],
          'captured=' + str(status['target_failure_captured']), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    args = parser.parse_args()
    run(args.build, args.out)

#!/usr/bin/env python3
"""Capture the first native collision and wheel failures in two fresh ROS SIL runs."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_native67_probe as lifecycle
from run_coupled_objective import HERE, check_build, check_space, sha


BUILD = HERE / 'builds/candidate_audit_20261030_01'
OUT = HERE / 'results/candidate_audit_20261030_01'
TARGETS = ('collision', 'wheel')
ORIGINAL_PREPARE = lifecycle.prepare_capture
PREDICATE = ('first normal 0.6s refresh solve whose unchanged native acceptance '
             'rejects the selected failure class; collision or either wheel '
             'speed/acceleration; one fresh run per predeclared target')


def set_planner_environment(tree, out, target):
    if target not in TARGETS:
        raise ValueError('Unknown native failure target: '+str(target))
    planners = tree.getroot().findall("node[@name='remani_planner_node']")
    if len(planners) != 1:
        raise ValueError('Exactly one private planner required')
    values = dict(CANDIDATE_AUDIT_TARGET=target,
        CANDIDATE_AUDIT_DIR=str(out/'candidate_audit'),
        CANDIDATE_AUDIT_DONE_PATH=str(out/'diagnostic_complete.json'),
        DELIVERY_FAST_QUERY='1', DELIVERY_ACCEPT_CANDIDATE='1',
        DELIVERY_REPLAY_KIND='', FAILURE_PROFILE_DISABLED='1',
        FAILURE_OWNED_PID_FILE=str(out/'owned_pids.txt'))
    for key, value in values.items():
        existing = planners[0].findall("env[@name='%s']" % key)
        if len(existing) > 1:
            raise ValueError('Ambiguous planner environment: '+key)
        if existing:
            existing[0].set('value', value)
        else:
            ET.SubElement(planners[0], 'env', name=key, value=value)
    return values


def prepare_capture(out, build, target, seed=11, ordinal=1):
    out = Path(out)
    manifest = ORIGINAL_PREPARE(out, build, seed, ordinal)
    tree = ET.parse(out/'experiment.launch')
    environment = set_planner_environment(tree, out, target)
    tree.write(out/'experiment.launch', encoding='unicode')
    settings = json.loads((out/'coupling_parameters.json').read_text())
    settings.update(diagnostic_phases=[], diagnostic_budgets_s=[.6],
        refresh_ordinal=None, capture_predicate=PREDICATE, candidate_audit_target=target,
        diagnostic_pause=True, genuine_online_closed_loop=False,
        navigation_metrics_valid=False, original_stop_policy=False,
        diagnostic_candidates_published=False, constraints_unchanged=True,
        objective_unchanged=True, initialization_unchanged=True,
        timeout_candidate_recovery_enabled=True, fast_dynamic_query_enabled=True,
        detailed_profiling_only_inside_paused_solver=False,
        scope='normal REMANI+MPPI until selected native rejection; subsequent frozen candidate diagnosis')
    (out/'coupling_parameters.json').write_text(json.dumps(settings, ensure_ascii=False, indent=2)+'\n')
    manifest.update(coupling_experiment=settings,
        parameters_sha256=sha(out/'coupling_parameters.json'),
        candidate_audit_experiment=True, navigation_metrics_valid=False,
        diagnostic_candidates_published=False,
        delivery_diagnostics=dict(enabled=False, candidate_audit=True,
            scope='cheap accepted-iterate recording before rejection; detailed snapshots and audits after owned simulator/MPPI pause'))
    manifest['candidate_audit'] = dict(target=target, capture_predicate=PREDICATE,
        planner_environment=environment, all_outcomes_retained=True,
        normal_refresh_budget_s=.6, initial_budget_s=1.5,
        post_failure_audits_outside_optimizer_budget=True,
        whole_resource_wall_contaminated_by_paused_audits=True,
        native_collision_and_motion_limits_unchanged=True,
        scope='candidate cause diagnosis; no online timing or navigation improvement claim')
    manifest['timing_only_capture'].update(common_to_both_collision_models=False,
        scope='light runtime recording until target rejection, then paused detailed diagnostic; not an online performance comparison')
    shutil.copy2(Path(__file__), out/'runtime'/Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = sha(Path(__file__))
    (out/'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    return manifest


def run_capture(out, build, target, seed=11):
    if target not in TARGETS:
        raise ValueError('Unknown native failure target: '+str(target))
    previous = lifecycle.prepare_capture
    lifecycle.prepare_capture = lambda path, child, trial_seed, ordinal: prepare_capture(
        path, child, target, trial_seed, ordinal)
    try:
        status = lifecycle.run_capture(out, build, seed, 1)
    finally:
        lifecycle.prepare_capture = previous
    done_path = Path(out)/'diagnostic_complete.json'
    done = json.loads(done_path.read_text()) if done_path.exists() else None
    status.update(candidate_audit_target=target, target_failure_captured=done is not None,
        candidate_audit_done=done,
        navigation_metrics_valid=False,
        scope='native candidate rejection diagnosis; post-failure frozen audits contaminate whole-wall timing')
    (Path(out)/'run_status.json').write_text(json.dumps(status, ensure_ascii=False, indent=2)+'\n')
    return status


def run(build, out):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest, _ = check_build(build)
    if not manifest.get('candidate_audit_experiment') or manifest.get('prepared_only'):
        raise ValueError('Completed isolated candidate audit build required')
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    jobs = [dict(directory='midcross_'+target+'_seed11', target=target, seed=11)
            for target in TARGETS]
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'), trials=jobs,
        scene='sphere_timing_midcross', duration_s=55., goal=[6., 0.],
        bounds=[-1.5, 7.5, -2., 2.], actor=[3., -1.6, 0., .18, .23],
        actor_appearance_after_motion_s=2., collision_spheres=67,
        deadline_ms=200., lease_ms=250., refresh_budget_s=.6,
        delivery_fast_query=1, delivery_accept_candidate=1,
        capture_predicate=PREDICATE, storage_reserve_mib=400,
        no_outcome_selected_reruns=True, diagnostic_candidates_published=False,
        navigation_metrics_valid=False,
        limitations=['one fresh seed11 run per failure target; target may not occur',
            'accepted-iterate recording adds some work before capture; no performance comparison',
            'after capture simulator and MPPI pause; whole resource wall is not normal solve time',
            'frozen maps are checked for stability; latest live publication checks are not exercised',
            'numerical ROS SIL; not Gazebo or hardware'])
    (out/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    completed = []
    for job in jobs:
        check_space(out)
        print('CANDIDATE_AUDIT_START', job['directory'], flush=True)
        try:
            status = run_capture(out/job['directory'], build, job['target'], job['seed'])
        except Exception as error:
            (out/'stopped.json').write_text(json.dumps(dict(trial=job,
                reason=str(error), previous_completed=completed,
                failed_output_retained=True), ensure_ascii=False, indent=2)+'\n')
            raise
        completed.append(dict(**job, target_failure_captured=status['target_failure_captured'],
            diagnostic_complete=status['diagnostic_complete'],
            candidate_audit_done=status['candidate_audit_done']))
        (out/'progress.json').write_text(json.dumps(completed, ensure_ascii=False, indent=2)+'\n')
        print('CANDIDATE_AUDIT_DONE', job['directory'],
              'captured='+str(status['target_failure_captured']), flush=True)
    (out/'complete.json').write_text(json.dumps(dict(complete=True,
        navigation_metrics_valid=False, completed=completed), ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    args = parser.parse_args()
    run(args.build, args.out)

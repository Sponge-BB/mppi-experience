#!/usr/bin/env python3
"""One fresh midcross pair with the same solver, observing KD pin OFF versus ON."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import yaml

from run_coupled_objective import HERE, check_build, check_space, run_trial, sha
from run_delivery_optimization import patch_trial as delivery_patch
from run_native_collision_3d import trial_kwargs
from run_native_dynamic_fast import clean_environment
from run_sphere_scenarios import register_scenes


BUILD = HERE/'builds/candidate_audit_20261030_03'
OUT = HERE/'results/candidate_solver_pin_20261030_01'
SWITCH = 'CANDIDATE_AUDIT_FREEZE_SOLVER'
GROUPS = ('pin0', 'pin1')


def set_pin_environment(tree, out, enabled):
    planners = tree.getroot().findall("node[@name='remani_planner_node']")
    if len(planners) != 1:
        raise ValueError('Exactly one private planner required')
    values = dict(CANDIDATE_AUDIT_TARGET='observe',
        CANDIDATE_AUDIT_DIR=str(out/'candidate_audit'),
        CANDIDATE_AUDIT_DONE_PATH=str(out/'diagnostic_complete.json'),
        CANDIDATE_AUDIT_FREEZE_SOLVER=str(int(enabled)),
        DELIVERY_FAST_QUERY='1', DELIVERY_ACCEPT_CANDIDATE='1',
        DELIVERY_REPLAY_KIND='')
    for key, value in values.items():
        existing = planners[0].findall("env[@name='%s']" % key)
        if len(existing) > 1:
            raise ValueError('Ambiguous planner environment: '+key)
        if existing:
            existing[0].set('value', value)
        else:
            ET.SubElement(planners[0], 'env', name=key, value=value)
    return values


def patch_trial(out, settings, group, runtime_trace=False):
    if group not in GROUPS:
        raise ValueError('Unknown solver pin group: '+str(group))
    out = Path(out)
    delivery_patch(out, settings, 'midcross', 'optimized')
    tree = ET.parse(out/'experiment.launch')
    environment = set_pin_environment(tree, out, group == 'pin1')
    tree.write(out/'experiment.launch', encoding='unicode')
    shutil.copy2(Path(__file__), out/'runtime'/Path(__file__).name)
    path = out/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['candidate_solver_pin'] = dict(group=group, pin_requested=group == 'pin1',
        planner_environment=environment, same_binary=True,
        algorithm_difference=SWITCH,
        recorder_enabled_in_both=True, recorder_target='observe',
        recorder_memory_and_instrumentation_same=True,
        diagnostic_pause=False, candidate_audit_dump=False,
        genuine_online_closed_loop=True, navigation_metrics_valid=True,
        pin_scope='present dynamic overlay retained through eligible ordinary 0.6s solver and native acceptance; released on function exit',
        live_fsm_acceptance_unchanged=True,
        runtime_pin_counter_available=bool(runtime_trace),
        scope=('single fresh numerical ROS SIL pair; per-request pin and overlay-change counters available for post-run verification'
               if runtime_trace else
               'single fresh numerical ROS SIL pair; pin requested from environment and frozen source, not per-request runtime ownership proof'))
    manifest['experiment_sources_sha256'][Path(__file__).name] = sha(Path(__file__))
    manifest['timing_only_capture']['common_to_both_solver_pin_groups'] = True
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')


def normalized_launch(path):
    root = ET.parse(path/'experiment.launch').getroot()
    for node in root.findall('node'):
        if node.get('name') == 'remani_planner_node':
            values = node.findall("env[@name='%s']" % SWITCH)
            if len(values) != 1:
                raise ValueError('Exactly one recorded pin switch required')
            node.remove(values[0])
    return ET.tostring(root, encoding='unicode').replace(str(path), '<trial>')


def check_pair_inputs(left, right):
    paths = [Path(left), Path(right)]
    for name in ('parameters.yaml', 'limits.json', 'scenario.json', 'geometry.json',
                 'shared_collision_config.json'):
        loader = yaml.safe_load if name.endswith('.yaml') else json.loads
        if loader((paths[0]/name).read_text()) != loader((paths[1]/name).read_text()):
            raise ValueError('Unexpected pair input difference: '+name)
    if normalized_launch(paths[0]) != normalized_launch(paths[1]):
        raise ValueError('Unexpected launch difference beyond solver pin and trial paths')
    manifests = [json.loads((path/'manifest.json').read_text()) for path in paths]
    for key in ('experiment_sources_sha256', 'native_binaries_sha256', 'mppi_library_sha256'):
        if manifests[0][key] != manifests[1][key]:
            raise ValueError('Pair source/binary differs: '+key)
    bodies = [json.loads((path/'native_collision_3d_body.json').read_text())['body'] for path in paths]
    if bodies[0] != bodies[1] or len(bodies[0]) != 67:
        raise ValueError('Actual B67 collision body differs')
    for path, enabled in zip(paths, (False, True)):
        planner = ET.parse(path/'experiment.launch').getroot().find("node[@name='remani_planner_node']")
        for name, expected in ((SWITCH, str(int(enabled))), ('CANDIDATE_AUDIT_TARGET', 'observe'),
                ('DELIVERY_FAST_QUERY', '1'), ('DELIVERY_ACCEPT_CANDIDATE', '1'),
                ('DELIVERY_REPLAY_KIND', '')):
            values = planner.findall("env[@name='%s']" % name)
            if len(values) != 1 or values[0].get('value') != expected:
                raise ValueError('Incorrect comparison environment: '+str(path)+'/'+name)
        if (path/'diagnostic_complete.json').exists() or (path/'candidate_audit/case.json').exists():
            raise ValueError('Observe pair unexpectedly invoked paused failure diagnostic')
    runtime_trace = all(item['candidate_solver_pin']['runtime_pin_counter_available']
                        for item in manifests)
    return dict(passed=True, same_binary=True, same_runtime=True,
        same_parameters=True, same_native_spheres=67,
        sole_planner_switch_difference=SWITCH,
        diagnostic_pause=False, navigation_metrics_valid=True,
        runtime_pin_counter_available=runtime_trace,
        scope=('fresh online inputs may diverge; per-request pin/overlay counters require post-run verification'
               if runtime_trace else
               'fresh online inputs may diverge; pin execution asserted by frozen source/environment, no per-request runtime pin counter'))


def run(build, out):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest, _ = check_build(build)
    if not manifest.get('candidate_audit_consistency_experiment') or manifest.get('prepared_only'):
        raise ValueError('Completed isolated candidate consistency build required')
    register_scenes()
    digest = sha(build/'build_manifest.json')
    trace_fields = set(manifest.get('candidate_solver_pin_runtime_observation_fields', []))
    runtime_trace = bool(manifest.get('candidate_solver_pin_runtime_trace')) or {
        'solver_overlay_freeze_requested', 'solver_overlay_pin_owned',
        'overlay_evaluate_complete_count', 'overlay_evaluate_changed_count',
        'solver_overlay_entry_key', 'solver_overlay_return_key'}.issubset(trace_fields)
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    jobs = [dict(scene='midcross', group=group, seed=11,
                 directory='midcross_'+group+'_seed11') for group in GROUPS]
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=digest, trials=jobs, duration_s=55.,
        groups=list(GROUPS), seed=11, goal=[6., 0.], bounds=[-1.5, 7.5, -2., 2.],
        actor=[3., -1.6, 0., .18, .23], actor_appearance_after_motion_s=2.,
        collision_spheres=67, wheel_omega=4.10, deadline_ms=200., lease_ms=250.,
        initial_budget_s=1.5, refresh_budget_s=.6,
        sole_algorithm_switch=SWITCH, delivery_fast_query=1, delivery_accept_candidate=1,
        recorder_target='observe', recorder_enabled_in_both=True,
        runtime_pin_counter_available=runtime_trace,
        diagnostic_pause=False, genuine_online_closed_loop=True,
        navigation_metrics_valid=True, storage_reserve_mib=400,
        no_outcome_selected_reruns=True,
        scope='fixed-arm full REMANI+MPPI numerical ROS SIL; not Gazebo or hardware',
        limitations=['one seed and fixed OFF/ON order; not a statistical success-rate comparison',
            'same recorder memory and instrumentation in both arms; recorder overhead remains',
            'pin holds the present overlay during eligible solve/native gate; live FSM keeps its current-map acceptance',
            ('per-request pin/overlay counters are recorded; post-run analysis must verify actual ownership and changes'
             if runtime_trace else
             'compiled source and launch verify the requested switch; no per-request runtime pin ownership counter'),
            'online states and later optimizer inputs can differ between arms'])
    (out/'plan.json').write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n')
    completed = []
    for job in jobs:
        check_space(out)
        trial = out/job['directory']
        print('CANDIDATE_SOLVER_PIN_START', job['directory'], flush=True)
        try:
            with clean_environment():
                valid = run_trial(trial, build, 'sphere_timing_midcross', 'engineering',
                    job['seed'], 55., 0., 0., True, digest,
                    prepare_hook=lambda path, settings, group=job['group']: patch_trial(
                        path, settings, group, runtime_trace),
                    **trial_kwargs(manifest))
        except Exception as error:
            (out/'stopped.json').write_text(json.dumps(dict(trial=job,
                reason=str(error), previous_completed=completed,
                failed_output_retained=True), ensure_ascii=False, indent=2)+'\n')
            raise
        result = json.loads((trial/'result.json').read_text()) if (trial/'result.json').exists() else {}
        completed.append(dict(**job, valid_experiment=valid, termination=result.get('reason')))
        (out/'progress.json').write_text(json.dumps(completed, ensure_ascii=False, indent=2)+'\n')
        print('CANDIDATE_SOLVER_PIN_DONE', job['directory'], 'valid='+str(valid),
              result.get('reason'), flush=True)
        if not valid:
            (out/'stopped.json').write_text(json.dumps(dict(trial=job,
                reason='invalid setup/interface; original output retained',
                previous_completed=completed), ensure_ascii=False, indent=2)+'\n')
            raise RuntimeError('Invalid online trial retained; inspect before further work')
    (out/'pair_check.json').write_text(json.dumps(check_pair_inputs(
        out/jobs[0]['directory'], out/jobs[1]['directory']), ensure_ascii=False, indent=2)+'\n')
    (out/'complete.json').write_text(json.dumps(dict(complete=True,
        navigation_metrics_valid=True, completed=completed), ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=BUILD)
    parser.add_argument('--out', type=Path, default=OUT)
    args = parser.parse_args()
    run(args.build, args.out)

#!/usr/bin/env python3
"""Fresh paired native ROS numerical closed loops; never vehicle drivers."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import build_native_dynamic_fast as builder
from native_realtime_runtime import patch_trial as patch_realtime, STALL_WINDOW_S, MIN_STOP_SIM_S
from run_coupled_objective import check_space, run_trial, sha
from run_native_collision_3d import trial_kwargs
from run_native_realtime import clean_environment as parent_environment, startup_check

GROUPS = ('baseline', 'fast')
SCENES = ('static', 'crossing', 'dynamic_head_on_center')


@contextmanager
def clean_environment():
    saved = {key: value for key, value in os.environ.items() if key.startswith('NATIVE_DYNAMIC_')}
    try:
        for key in saved:
            del os.environ[key]
        with parent_environment():
            yield
    finally:
        for key in list(os.environ):
            if key.startswith('NATIVE_DYNAMIC_'):
                del os.environ[key]
        os.environ.update(saved)


def patch_trial(out, enabled):
    out = Path(out)
    patch_realtime(out, True)
    tree = ET.parse(out / 'experiment.launch')
    targets = [node for node in tree.getroot().findall('node')
               if node.get('name') in ('base_mppi', 'remani_planner_node')]
    if len(targets) != 2:
        raise ValueError('Expected exactly the isolated planner and MPPI nodes')
    for node in targets:
        ET.SubElement(node, 'env', name='NATIVE_DYNAMIC_FAST', value=str(int(enabled)))
    tree.write(out / 'experiment.launch', encoding='unicode')
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['native_realtime']['collision_and_dynamic_branches_unchanged'] = not enabled
    manifest['native_dynamic_fast'] = dict(enabled=bool(enabled), binary_switch='NATIVE_DYNAMIC_FAST',
        same_freepath_demand=True, geometry_and_search_unchanged=True,
        native_and_execution_budgets_unchanged=True, runtime_sources_identical=True,
        runner_sha256=sha(Path(__file__)),
        changes='prune rejected collision cost and reuse exact nominal query results',
        scope='new online closed loop, not replay; fixed-arm numerical SIL, not Gazebo/hardware')
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


def run_batch(build, out, stage='precheck', replay=None, precheck=None):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest = builder.check(build)
    if manifest.get('prepared_only'):
        raise ValueError('Completed dynamic-fast build required')
    if stage not in ('precheck', 'formal'):
        raise ValueError('Unknown stage')
    if replay is None:
        raise ValueError('Same-input replay gate is required before online trials')
    replay = Path(replay).resolve()
    replay_result = json.loads(replay.read_text())
    if not replay_result.get('passed'):
        raise ValueError('Same-input replay did not pass; no online trials started')
    # Bind the gate to this exact new binary, not just an unrelated passing file.
    libraries = replay_result.get('libraries', {})
    if (libraries.get('child', {}).get('sha256') != manifest['mppi_library_sha256']
            or libraries.get('parent', {}).get('sha256') != manifest['native_dynamic_fast_parent_mppi_sha256']
            or not replay_result.get('records')):
        raise ValueError('Replay gate does not attest this MPPI binary')
    digest = sha(build / 'build_manifest.json')
    runtime_sources = None
    if stage == 'formal':
        if precheck is None:
            raise ValueError('Formal matrix requires completed same-build empty precheck')
        precheck = Path(precheck).resolve()
        pp = json.loads((precheck / 'plan.json').read_text())
        if (pp.get('stage') != 'precheck' or pp.get('build_manifest_sha256') != digest
                or not (precheck / 'complete.json').is_file()):
            raise ValueError('Precheck incomplete or different build')
        for group in GROUPS:
            trial = precheck / ('empty_%s_seed11' % group)
            if not startup_check(trial)['passed']:
                raise ValueError('Both switches must pass empty startup: ' + group)
        runtime_sources = json.loads((precheck / 'empty_fast_seed11/manifest.json').read_text())['experiment_sources_sha256']
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    scenes, duration = (('empty',), 12.) if stage == 'precheck' else (SCENES, 55.)
    jobs = [(scene, group) for index, scene in enumerate(scenes)
            for group in (GROUPS if index % 2 == 0 else tuple(reversed(GROUPS)))]
    plan = dict(created=datetime.now(timezone.utc).isoformat(), stage=stage,
        build=str(build), build_manifest_sha256=digest, seed=11, duration_s=duration,
        groups=list(GROUPS), scenes=list(scenes), ordered_jobs=jobs,
        switches={'NATIVE_REALTIME_DEMAND': 1, 'NATIVE_DYNAMIC_FAST': {'baseline': 0, 'fast': 1}},
        same_binary=True, full_online_native_planner=True, profile=False, paused=False,
        replay_gate=str(replay), replay_gate_sha256=sha(replay),
        precheck=str(precheck) if precheck else None, state_policy=trial_kwargs(manifest),
        unchanged=['160 native body samples', 'native hard/soft distances', 'CV horizon',
                   'candidate count/order and RNG', '50ms deadline', '75ms command lease',
                   'native solver budgets and 300 iterations', 'actor motion'],
        common_early_stop=dict(reason='deadline_zero_deadlock', continuous_s=STALL_WINDOW_S,
                               minimum_sim_s=MIN_STOP_SIM_S, early_stop_is_success=False),
        outcome_policy='retain goal, collision, timeout; no replacement seeds or outcome filtering',
        scope='paired seed11 diagnostic ROS numerical SIL; not statistical success rate, Gazebo or hardware')
    (out / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    completed = []
    try:
        for scene, group in jobs:
            name = '%s_%s_seed11' % (scene, group)
            print('START', name, flush=True)
            with clean_environment():
                valid = run_trial(out / name, build, scene, 'engineering', 11, duration, 0., 0., True,
                    digest, expected_runtime_sources=runtime_sources,
                    prepare_hook=lambda trial, settings, fast=group == 'fast': patch_trial(trial, fast),
                    **trial_kwargs(manifest))
            current = json.loads((out / name / 'manifest.json').read_text())['experiment_sources_sha256']
            if runtime_sources is None:
                runtime_sources = current
            result = json.loads((out / name / 'result.json').read_text())
            record = dict(trial=name, scene=scene, group=group, valid_interface=valid,
                          termination=result['reason'], startup=startup_check(out / name))
            completed.append(record)
            (out / 'progress.json').write_text(json.dumps(completed, indent=2) + '\n')
            print('DONE', name, result['reason'], 'valid=', valid, flush=True)
            if not valid:
                raise RuntimeError('Interface/audit failed; retain result, do not silently retry')
        (out / 'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2) + '\n')
    except Exception as exc:
        (out / 'stopped.json').write_text(json.dumps(dict(completed=completed,
            reason='%s: %s' % (type(exc).__name__, exc)), indent=2) + '\n')
        raise
    return completed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=builder.DEFAULT_OUT)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--stage', choices=('precheck', 'formal'), default='precheck')
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--precheck', type=Path)
    args = parser.parse_args()
    run_batch(args.build, args.out, args.stage, args.replay, args.precheck)


if __name__ == '__main__':
    main()

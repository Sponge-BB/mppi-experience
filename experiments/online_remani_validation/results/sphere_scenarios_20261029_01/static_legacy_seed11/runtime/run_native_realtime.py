#!/usr/bin/env python3
"""Unpaused native3D original/demand SIL: empty qualification then fixed matrix."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path

import numpy as np

from native_realtime_runtime import patch_trial, STALL_WINDOW_S, MIN_STOP_SIM_S
from run_coupled_objective import check_build, check_space, run_trial, sha
from run_native_collision_3d import clean_online_environment, trial_kwargs

GROUPS = ('original', 'demand')
SCENES = ('static', 'crossing', 'dynamic_head_on_center')


@contextmanager
def clean_environment():
    saved = {key: value for key, value in os.environ.items() if key.startswith(('NATIVE_REALTIME_', 'NATIVE_MPPI_'))}
    try:
        for key in saved:
            del os.environ[key]
        with clean_online_environment(parity=False):
            os.environ['NATIVE_REALTIME_PROFILE'] = '0'
            os.environ['NATIVE_REALTIME_REPLAY'] = '0'
            yield
    finally:
        for key in list(os.environ):
            if key.startswith(('NATIVE_REALTIME_', 'NATIVE_MPPI_')):
                del os.environ[key]
        os.environ.update(saved)


def startup_check(trial):
    result = json.loads((trial/'result.json').read_text())
    status = json.loads((trial/'run_status.json').read_text())
    audit = json.loads((trial/'trial_audit.json').read_text())
    states = np.atleast_1d(np.genfromtxt(trial/'state.csv', names=True, delimiter=','))
    displacement = float(np.max(np.hypot(states['x']-states['x'][0], states['y']-states['y'][0])))
    monitor = result.get('native3d_truth_monitor', {})
    ready = bool(monitor.get('geometry_available') and monitor.get('checks', 0) > 0 and not monitor.get('error'))
    valid = bool(audit.get('valid_experiment') and status.get('result_written') and
                 not status.get('required_process_failed') and not status.get('runner_error'))
    return dict(passed=bool(valid and ready and result.get('motion_anchor') is not None and displacement >= .10
                    and not monitor.get('sample_collision')),
        max_measured_displacement_m=displacement, motion_anchor=result.get('motion_anchor'),
        truth_monitor_ready=ready, native3d_sample_collision=monitor.get('sample_collision'),
        valid_interface=valid, navigation_success_claim=False,
        criterion='actual displacement >=0.10m and motion anchor, valid interface, available 3D truth monitor, no sampled contact')


def validate_precheck(precheck, build_hash):
    precheck = Path(precheck).resolve()
    plan = json.loads((precheck/'plan.json').read_text())
    if plan.get('stage') != 'precheck' or plan.get('build_manifest_sha256') != build_hash or not (precheck/'complete.json').is_file():
        raise ValueError('requires complete same-build empty precheck')
    if plan.get('seed') != 11 or plan.get('scenes') != ['empty'] or plan.get('groups') != list(GROUPS):
        raise ValueError('precheck must retain declared seed11 and both empty groups')
    trial = precheck/('empty_demand_seed%d' % plan['seed'])
    if not startup_check(trial)['passed']:
        raise ValueError('new demand policy did not qualify; retain precheck and do not start formal matrix')
    return json.loads((trial/'manifest.json').read_text())['experiment_sources_sha256'], plan


def run_batch(build, out, stage='precheck', precheck=None):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest, _ = check_build(build)
    if not manifest.get('native_realtime_experiment'):
        raise ValueError('requires immutable native_realtime_experiment child build')
    if manifest.get('native_realtime_prepared_only'):
        raise ValueError('prepared-only child has not completed compilation/validation')
    if not manifest.get('native_collision_3d_experiment'):
        raise ValueError('native3D collision family required')
    digest = sha(build/'build_manifest.json')
    runtime_sources = None
    if stage == 'formal':
        if precheck is None:
            raise ValueError('formal stage requires --precheck')
        runtime_sources, previous = validate_precheck(precheck, digest)
    elif stage != 'precheck':
        raise ValueError('unknown stage')
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    scenes, duration = (('empty',), 12.) if stage == 'precheck' else (SCENES, 55.)
    plan = dict(created=datetime.now(timezone.utc).isoformat(), stage=stage,
        build=str(build), build_manifest_sha256=digest, groups=list(GROUPS), scenes=list(scenes),
        seed=11, duration_s=duration, common_native_collision_3d=True, native_export=True,
        collision_geometry_unchanged=True, same_child_binary=True, full_online_native_planner=True,
        coupling_weights=[0., 0.], state_policy=trial_kwargs(manifest),
        demand_switch='NATIVE_REALTIME_DEMAND', diagnostic_profile=False, paused_replay=False,
        original_branch='native3D original full-sampling free-path branch',
        demand_branch='existing bounded candidates when native3D free path; conflicts/dynamic unchanged',
        common_early_stop=dict(reason='deadline_zero_deadlock', continuous_s=STALL_WINDOW_S,
            minimum_sim_s=MIN_STOP_SIM_S, measured_motion_during_window_max_m=.001,
            requires='reference, nonzero nominal, successful rollout and brake validation, deadline zero command',
            early_stop_is_success=False),
        actual_obstacle_exposure_required_for_dynamic_claims=True,
        predeclared_formal=dict(scenes=list(SCENES), groups=list(GROUPS), seed=11, duration_s=55.),
        precheck=str(Path(precheck).resolve()) if precheck else None,
        unchanged=['native3D geometry/hard/soft kernels', '50ms command deadline', '75ms command lease',
                   'native iteration cap 300', 'native solver budget', 'physical wheel limits', 'actor motion'],
        scope='fixed-arm differential-drive ROS numerical SIL; not Gazebo or hardware; not a statistical success-rate experiment')
    (out/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    done = []
    try:
        for scene in scenes:
            for group in GROUPS:
                name = '%s_%s_seed11' % (scene, group)
                with clean_environment():
                    valid = run_trial(out/name, build, scene, 'engineering', 11, duration, 0., 0., True,
                        digest, expected_runtime_sources=runtime_sources,
                        prepare_hook=lambda trial, settings, demand=group == 'demand': patch_trial(trial, demand),
                        **trial_kwargs(manifest))
                current = json.loads((out/name/'manifest.json').read_text())['experiment_sources_sha256']
                if runtime_sources is None:
                    runtime_sources = current
                record = dict(trial=name, group=group, scene=scene, valid_interface=valid,
                    startup=startup_check(out/name))
                done.append(record)
                (out/'progress.json').write_text(json.dumps(done, indent=2)+'\n')
                if not valid:
                    raise RuntimeError('interface failure retained; no silent retry or seed selection')
        qualified = stage != 'precheck' or done[-1]['startup']['passed']
        (out/'complete.json').write_text(json.dumps(dict(complete=True, completed=done,
            qualified_for_formal=qualified if stage == 'precheck' else None), indent=2)+'\n')
        return done
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=done, reason='%s: %s' % (type(exc).__name__, exc)), indent=2)+'\n')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--stage', choices=('precheck', 'formal'), default='precheck')
    parser.add_argument('--precheck', type=Path)
    args = parser.parse_args()
    run_batch(args.build, args.out, args.stage, args.precheck)


if __name__ == '__main__':
    main()


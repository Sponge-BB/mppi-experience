#!/usr/bin/env python3
"""Six genuine unpaused REMANI + MPPI numerical SIL trials; never hardware."""
import argparse
from datetime import datetime, timezone
import json
import os
from contextlib import contextmanager
from pathlib import Path

from native_collision_runtime import patch_trial
from run_coupled_objective import check_build, check_space, run_trial, sha

SCENES = ('static', 'crossing', 'dynamic_head_on_center')


def trial_kwargs(manifest):
    state = bool(manifest.get('state_contract_experiment'))
    return dict(diagnostic_capture=state, state_contract=state,
                domain_recovery=bool(manifest.get('domain_recovery_experiment')),
                signed_low_speed=True, clearance_recovery=False)


@contextmanager
def clean_online_environment(parity=False):
    prefixes = ('DELIVERY_REPLAY_', 'FEASIBILITY_', 'BUDGET_', 'SOLVER_SCALING_', 'NATIVE_COLLISION_')
    saved = {key: value for key, value in os.environ.items() if key.startswith(prefixes)}
    try:
        for key in saved:
            del os.environ[key]
        os.environ['FEASIBILITY_STOP_GATE'] = '0'
        os.environ['DELIVERY_REPLAY_KIND'] = ''
        os.environ['NATIVE_COLLISION_PARITY'] = str(int(parity))
        yield
    finally:
        for key in list(os.environ):
            if key.startswith(prefixes):
                del os.environ[key]
        os.environ.update(saved)


def run_batch(control, treatment, out, duration=55., scenes=SCENES, seed=11, preflight=False):
    control, treatment, out = map(lambda p: Path(p).resolve(), (control, treatment, out))
    if control != treatment:
        raise ValueError('OFF/ON must use the SAME new build with common native3D export; parent build is rollback only')
    manifests = {name: check_build(path)[0] for name, path in (('control', control), ('native3d', treatment))}
    if not manifests['native3d'].get('native_collision_3d_experiment'):
        raise ValueError('treatment must explicitly declare native_collision_3d_experiment')
    if not 1 <= duration <= 120 or not scenes or any(scene not in SCENES for scene in scenes):
        raise ValueError('invalid duration or scene selection')
    if any(trial_kwargs(m) != trial_kwargs(manifests['control']) for m in manifests.values()):
        raise ValueError('parent state/diagnostic/domain policy differs between comparison builds')
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    builds = {'control': control, 'native3d': treatment}
    hashes = {name: sha(path / 'build_manifest.json') for name, path in builds.items()}
    plan = dict(created=datetime.now(timezone.utc).isoformat(), builds={k: str(v) for k, v in builds.items()},
        build_manifest_sha256=hashes, scenes=list(scenes), seed=seed, duration_s=duration,
        groups=['native3d'] if preflight else ['control', 'native3d'], coupling_group='engineering', execution_weight=0., handover_weight=0.,
        paused=False, complete_native_online_planner=True, search=True, paired_random_seed=True,
        scheduling_determinism_guaranteed=False, collision_geometry_factor='old supplemental 2D vs native 3D',
        iteration_cap_unchanged=300, control_lease_s=.075, online_solve_budget_unchanged=True,
        same_new_binary_off_on=True, common_native3d_export=True,
        preflight_only=bool(preflight), navigation_performance_claims=not preflight,
        state_policy=trial_kwargs(manifests['control']),
        scope='fixed-arm differential-drive ROS numerical SIL; NOT Gazebo or physical robot',
        safety_metric_note='old 2D projection overlap retained but no longer terminates either group; independent sampled 3D truth audit separately')
    (out / 'plan.json').write_text(json.dumps(plan, indent=2) + '\n')
    completed = []
    runtime_sources = None
    try:
        for scene in scenes:
            for group in plan['groups']:
                name = '%s_%s_seed%d' % (scene, group, seed)
                with clean_online_environment(parity=preflight):
                    valid = run_trial(out / name, builds[group], scene, 'engineering', seed,
                        duration, 0., 0., True, hashes[group], expected_runtime_sources=runtime_sources,
                        prepare_hook=lambda trial, settings, enabled=group == 'native3d': patch_trial(trial, enabled, parity=preflight),
                        **trial_kwargs(manifests[group]))
                sources = json.loads((out / name / 'manifest.json').read_text())['experiment_sources_sha256']
                if runtime_sources is None:
                    runtime_sources = sources
                completed.append(dict(trial=name, scene=scene, group=group, valid=valid))
                (out / 'progress.json').write_text(json.dumps(completed, indent=2) + '\n')
                if not valid:
                    raise RuntimeError('interface/audit invalid; data retained without rerun or outcome filtering')
    except Exception as exc:
        (out / 'stopped.json').write_text(json.dumps(dict(completed=completed,
            reason='%s: %s' % (type(exc).__name__, exc)), indent=2) + '\n')
        raise
    (out / 'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2) + '\n')
    return completed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--control-build', type=Path, required=True)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=55.)
    parser.add_argument('--scenes', nargs='+', choices=SCENES, default=SCENES)
    parser.add_argument('--seed', type=int, default=11)
    parser.add_argument('--preflight', action='store_true', help='one native3d static development run with parity enabled; NOT in formal six-run statistics')
    args = parser.parse_args()
    run_batch(args.control_build, args.build, args.out,
              min(args.duration, 20.) if args.preflight else args.duration,
              ('static',) if args.preflight else args.scenes, args.seed, args.preflight)


if __name__ == '__main__':
    main()

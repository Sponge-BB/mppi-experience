#!/usr/bin/env python3
"""Compare native initialization with the existing checked MPPI-prefix initializer."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_fast_refresh_feedback as parent
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE/'results/prefix_handoff_20261101_01'
builder = parent.builder


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def patch(trial, settings, enabled):
    parent.patch(trial, settings, True)
    tree = ET.parse(trial/'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    common.set_env(planner, 'COLLAB_PREFIX', int(enabled))
    tree.write(trial/'experiment.launch', encoding='unicode')
    settings.update(prefix_initializer=bool(enabled), legacy_prefix=bool(enabled),
        comparison_group='B' if enabled else 'A',
        baseline='completed fast-refresh-feedback B; original soft cost and latest gates retained',
        fast_refresh_feedback=True, slow_reference_nominal_only=True,
        changed='native A* initializer versus existing MPPI near-prefix plus native continuation',
        prefix_is_committed_execution=False, production_modified=False)
    save(trial/'coupling_parameters.json', settings)
    path = trial/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['legacy_factorial_runtime']['prefix'] = bool(enabled)
    manifest['prefix_handoff'] = dict(enabled=bool(enabled), same_binary=True,
        runtime_switch='COLLAB_PREFIX', existing_implementation=True,
        prefix_horizon_s=1., compressed_points=3, prefix_pieces=2,
        initialization='original measured head p/v/a/j; same-gear MPPI near-prefix; A* from prefix endpoint; one merged native block',
        fallback='original native search if the existing prefix conversion or continuation is unavailable',
        prediction_source='post-command validated MPPI rollout; same ROS main callback thread as frozen request',
        timing_scope='source relative sample durations provide an initialization only; no fixed future execution commitment',
        known_timing_limit='existing prefixRows uses first eligible recorded row and its relative clock; no exact-time matching objective added',
        unchanged=['original native/MPPI binaries and geometry', 'measured starting state and final goal',
            'fast avoidance feedback trigger', 'soft weights and delivery delay 1.1s',
            'native 67 spheres, static/self/wheel and latest dynamic gates',
            'MPPI actual rollout and lease-brake validator',
            '1.2s solver budget, 200ms MPPI deadline, 250ms command lease',
            'original native search fallback and retry timing'],
        prefix_use_and_side_must_be_measured=True, diagnostic_pause=False,
        search_candidates_recorded=False, production_modified=False, hardware=False, gazebo=False)
    shutil.copy2(Path(__file__), trial/'runtime'/Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = common.lifecycle.sha(Path(__file__))
    save(path, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enabled', type=int, choices=(0, 1), required=True)
    parser.add_argument('--tag', required=True)
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT/'plan.json').exists():
        save(OUT/'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Does initializing the slow planner from the checked fast near-prefix fix opposite-side candidates and improve real delivery?',
            groups='A original native initialization; B existing MPPI near-prefix initialization',
            unique_switch='COLLAB_PREFIX 0/1', same_existing_binary=True,
            both_fast_refresh_feedback=True, both_nominal_only_slow_gate=True,
            no_new_objective_weight_or_gate=True, no_new_frontend_algorithm=True,
            seed=11, goal=[6., 0.], duration_s=55., planned_trials=['headon_A', 'headon_B'],
            physical_spheres=67, solver_budget_s=1.2, mppi_deadline_ms=200., command_lease_ms=250.,
            primary_measurements=['prefix actually used and fallback reason',
                'native initial and final candidate side relative to frozen MPPI',
                'publication phase/time mapping and latest rejection',
                'first approved intervention feedback to effective new-reference control',
                'all request outcomes, physical gaps, stopping and actual execution validation'],
            scope='fixed-arm complete REMANI+MPPI ROS numerical SIL, not Gazebo or hardware',
            single_async_pair_not_identical_input_or_statistical_performance_proof=True,
            prefix_initialization_is_not_a_committed_execution_or_solved_time_coupling=True,
            production_modified=False, github_upload_remains_paused=True))
    manifest = builder.check()
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT/args.tag, builder.OUT,
            'sphere_timing_headon', 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT/'build_manifest.json'),
            reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('PREFIX_HANDOFF_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

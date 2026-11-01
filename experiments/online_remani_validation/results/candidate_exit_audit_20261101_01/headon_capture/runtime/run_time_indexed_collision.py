#!/usr/bin/env python3
"""One fresh fixed-arm ROS SIL run for the time-indexed backend comparison."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_time_indexed_collision as builder
import run_frontend_rejoin as previous
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/time_indexed_collision_20261031_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, scene, enabled):
    previous.builder.OUT = builder.OUT
    previous.patch(trial, settings, scene, 'penalty')
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node')
                   if n.get('name') == 'remani_planner_node')
    values = dict(TIME_INDEXED_COLLISION=int(enabled),
                  TEMPORAL_COLLISION_CAPTURE=1,
                  TEMPORAL_COLLISION_CAPTURE_PATH=trial / 'temporal_collision.jsonl',
                  HANDOFF_ALIGNMENT_PROBE=1,
                  COUPLING_EXEC_WEIGHT=0, COUPLING_HAND_WEIGHT=0)
    for name, value in values.items():
        previous.prior.common.set_env(planner, name, value)
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(time_indexed_collision=bool(enabled), execution_weight=0.,
        handover_weight=0., temporal_capture=True,
        planning_epoch='requested candidate epoch; final activation may differ',
        unchanged_latest_prediction_validation=True)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = previous.prior.common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['time_indexed_collision'] = dict(enabled=bool(enabled),
        group='B' if enabled else 'A', same_binary=True,
        dynamic_scope='backend objective, native gate and prepublication planning check',
        static_geometry_and_wheel_limits_unchanged=True,
        frontend_and_mppi_unchanged=True, latest_cv_acceptance_unchanged=True,
        planning_epoch='requested epoch; not a committed fast-layer future boundary',
        prediction_window_fallback='original swept query outside available CV horizon',
        frozen_input_capture=True, production_modified=False,
        scope='fixed-arm complete REMANI+MPPI numerical ROS SIL')
    manifest['frontend_rejoin']['mppi_and_backend_unchanged'] = not enabled
    shutil.copy2(Path(__file__), trial / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = previous.prior.common.lifecycle.sha(Path(__file__))
    save(path, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('headon', 'static'), required=True)
    parser.add_argument('--enabled', type=int, choices=(0, 1), required=True)
    parser.add_argument('--tag', required=True)
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Test whether temporal union occupancy causes unnecessary candidate rejection',
            ordered_trials=['headon_A', 'headon_B', 'static_B', 'static_A'],
            online_gate='A frozen same-input evidence first; retain all completed outcomes',
            same_input='same polynomial, exact source snapshot, equal guide inflation; nominal/half-speed/hold mappings',
            changed='backend dynamic time queries and their objective derivatives',
            unchanged=['frontend', 'MPPI', '67 collision spheres', 'wheel limits',
                       '1.2s solver budget', 'latest CV acceptance including half-speed and hold'],
            seed=11, duration_s=55., goal=[6., 0.],
            backend_budget_s=1.2, mppi_budget_ms=200., lease_ms=250.,
            execution_weight=0., handover_weight=0.,
            same_binary=True, production_modified=False,
            scope='fixed-arm complete REMANI+MPPI numerical ROS SIL',
            github_upload_remains_paused=True))
    manifest = builder.check()
    common = previous.prior.common
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(
            OUT / args.tag, builder.OUT, 'sphere_timing_' + args.scene,
            'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, args.scene, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('TIME_INDEXED_COLLISION_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

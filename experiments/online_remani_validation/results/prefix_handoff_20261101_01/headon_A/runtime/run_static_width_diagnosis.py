#!/usr/bin/env python3
"""Trace search/initial/final geometry in isolated full REMANI+MPPI SIL."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_static_agility as prior
import build_static_width_diagnosis as builder
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/static_width_diagnosis_20261031_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, enabled):
    prior.builder.OUT = builder.OUT
    prior.patch(trial, settings, 'static', enabled)
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(node for node in tree.getroot().findall('node')
                   if node.get('name') == 'remani_planner_node')
    prior.common.set_env(planner, 'STATIC_WIDTH_TRACE_PATH', trial / 'static_width_trace.jsonl')
    prior.common.set_env(planner, 'STATE_CONTRACT_CAPTURE', 1)
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(static_width_diagnosis=True, navigation_algorithm_unchanged=True,
                    diagnosis_not_speed_benchmark=True)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = prior.common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['static_width_diagnosis'] = dict(
        stages=['native_search_points', 'initial_polynomial', 'optimized_polynomial', 'consumed_reference'],
        record_per_request=True, record_on_solver_iterations=False,
        purpose='Locate wider geometry before changing optimizer/controller policy',
        no_constraint_or_cost_change=True, production_modified=False)
    shutil.copy2(Path(__file__), trial / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = prior.common.lifecycle.sha(Path(__file__))
    save(path, manifest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--enabled', type=int, choices=[0, 1], required=True)
    parser.add_argument('--tag', required=True)
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    plan = OUT / 'plan.json'
    if not plan.exists():
        save(plan, dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Locate native search / polynomial interpolation / optimization width increase',
            primary_trials=['static_enabled_repeat01', 'static_disabled_repeat01'],
            further_trials='Only if the first pair fails to localize the issue',
            build=str(builder.OUT), goal=[6., 0.], static_obstacle=[3., 0., .23],
            wheel_omega=4.1, wheel_alpha=4.25, native_budget_s=1.2,
            spheres=67, native_initializer=True, mppi_prefix=False,
            scope='fixed-arm full REMANI+MPPI numerical ROS SIL; not Gazebo or hardware',
            production_modified=False, github_upload_remains_paused=True))
    prior.common.register_scenes()
    manifest = builder.check()
    with prior.common.clean_environment():
        valid = prior.common.lifecycle.run_trial(
            OUT / args.tag, builder.OUT, 'sphere_timing_static', 'engineering',
            11, 55., 0., 0., False,
            prior.common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=prior.common.RESERVE + prior.common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('STATIC_WIDTH_TRIAL_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained for inspection')


if __name__ == '__main__':
    main()

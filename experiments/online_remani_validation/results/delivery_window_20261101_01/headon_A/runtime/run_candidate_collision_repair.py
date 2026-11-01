#!/usr/bin/env python3
"""One head-on A/B: repair a rejected candidate once within its original budget."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_candidate_collision_repair as builder
import run_delivery_feedback as previous
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/candidate_collision_repair_20261031_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, enabled):
    # Inherit the last experiment's B in both arms, using this new same binary.
    previous.previous.builder.OUT = builder.OUT
    previous.patch(trial, settings, True)
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node')
                   if n.get('name') == 'remani_planner_node')
    common.set_env(planner, 'CANDIDATE_COLLISION_REPAIR', int(enabled))
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(candidate_collision_repair=bool(enabled),
        comparison_group='B' if enabled else 'A',
        baseline='latest complete coverage model with delivery soft feedback enabled',
        repair_scope='once after final planning collision, warm-start from actual candidate, fixed penalty factor 10 in failed piece',
        repair_uses_original_remaining_budget=True,
        repair_attempt_limit=1, repair_collision_weight_factor=10.,
        optimization_budget_s=1.2, execution_weight=0., handover_weight=1.,
        delivery_target_delay_s=1.1)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['delivery_feedback'].update(group='common enabled configuration', enabled=True)
    manifest['candidate_collision_repair'] = dict(enabled=bool(enabled),
        group='B' if enabled else 'A', same_binary=True,
        runtime_flag='CANDIDATE_COLLISION_REPAIR',
        warm_start='last actual solver vector, regenerated polynomial caches',
        maximum_attempts=1, failed_piece_collision_penalty_factor=10.,
        original_budget_s=1.2, budget_start_not_reset=True,
        unchanged=['frozen CV input within request', 'native full/half dynamic checks',
                   'native hard collision/wheel bounds', 'frontend initialization',
                   '67 physical collision spheres', 'MPPI and braking validation',
                   '200ms/250ms execution timing', 'delivery feedback weight 1'],
        scope='fixed-arm complete REMANI+MPPI numerical ROS SIL',
        production_modified=False)
    shutil.copy2(Path(__file__), trial / 'runtime' / Path(__file__).name)
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
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Test whether repairing a collision-rejected candidate within the original request improves usable trajectory delivery',
            planned_trials=['headon_A', 'headon_B'],
            baseline='latest coverage-alignment model plus active delivery-state soft feedback',
            changed='one failed-piece collision repair attempt off/on',
            repair_collision_penalty_factor=10., maximum_repair_attempts=1,
            repair_budget='remaining original 1.2s optimizer budget; never reset',
            same_binary=True, seed=11, goal=[6., 0.], duration_s=55.,
            execution_weight=0., handover_weight=1., delivery_delay_s=1.1,
            physical_spheres=67, mppi_budget_ms=200., command_lease_ms=250.,
            same_run_metrics=['final pre-repair collision and solver stop reason',
                'failed-piece soft collision cost and gradient',
                'pre/post candidates against the same frozen CV',
                'remaining budget and repair cost', 'all native/latest gate failures',
                'actual new-reference consumption and validated control publication',
                'physical contact, path, stops, full MPPI loop time'],
            output='one recorded A/B GIF, simple data plot and concise Chinese TXT',
            additional_online_trials_planned=False,
            production_modified=False, github_upload_remains_paused=True,
            scope='fixed-arm complete REMANI+MPPI numerical ROS SIL'))
    manifest = builder.check()
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT / args.tag, builder.OUT,
            'sphere_timing_headon', 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('CANDIDATE_COLLISION_REPAIR_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

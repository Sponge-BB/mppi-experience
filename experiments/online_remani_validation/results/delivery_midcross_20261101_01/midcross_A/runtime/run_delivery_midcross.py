#!/usr/bin/env python3
"""One existing crossing scenario: slow CV3 versus CV4.5 delivery coverage."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
from unittest.mock import patch as patch_object
import xml.etree.ElementTree as ET

import run_coverage_alignment as coverage
import run_delivery_window as previous
from run_native_collision_3d import trial_kwargs
from run_sphere_scenarios import ACTORS

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/delivery_midcross_20261101_01'
SCENE = 'sphere_timing_midcross'
builder = previous.builder
common = previous.common
save = previous.save


def patch(trial, settings, enabled):
    # The historical coverage runner hard-codes headon in its preparation
    # chain. Forward the real scene at that boundary, once, without editing
    # any frozen ancestor, double-injecting actor code, or changing algorithms.
    original = coverage.previous.patch

    def forward_scene(path, configuration, legacy_scene, active):
        return original(path, configuration, 'midcross', active)

    with patch_object.object(coverage.previous, 'patch', forward_scene):
        previous.patch(trial, settings, enabled)

    scenario = json.loads((trial / 'scenario.json').read_text())
    manifest = json.loads((trial / 'manifest.json').read_text())
    tree = ET.parse(trial / 'experiment.launch')
    simulator = next(node for node in tree.getroot().findall('node')
                     if node.get('name') == 'algorithm_simulator')
    actual_scene = simulator.find("param[@name='scene']").get('value')
    expected_actors = [dict(id=i+1, x=a[0], y=a[1], vx=a[2], vy=a[3], r=a[4])
                       for i, a in enumerate(ACTORS['midcross'])]
    if not (actual_scene == scenario['scene'] == manifest['scene'] == SCENE
            and manifest['scenario'] == scenario
            and scenario['actors'] == expected_actors
            and scenario['appearance_s'] == 2.):
        raise ValueError('Crossing simulator, declaration or actor mismatch')

    settings.update(scene=SCENE, experiment='delivery coverage crossing transfer')
    save(trial / 'coupling_parameters.json', settings)
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['delivery_crossing'] = dict(scene=SCENE,
        actual_simulator_and_declared_actor_verified=True,
        actor_start=[3., -1.6], actor_velocity=[0., .18], physical_radius=.23,
        appearance_after_motion_anchor_s=2., goal=[6., 0.],
        scope='Transfer check of the existing slow-only CV window change; not a new planner',
        only_group_difference='SLOW_DELIVERY_COVERAGE 0/1',
        historical_scene_forwarding='local preparation wrapper, original restored before launch',
        no_hardware=True, no_gazebo=True)
    for source in (Path(__file__), HERE / 'report_delivery_midcross.py'):
        if source.exists():
            shutil.copy2(source, trial / 'runtime' / source.name)
            manifest['experiment_sources_sha256'][source.name] = common.lifecycle.sha(source)
    save(trial / 'manifest.json', manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--enabled', type=int, choices=(0, 1), required=True)
    args = parser.parse_args()
    OUT.mkdir(exist_ok=True)
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            key_task='Useful slow-to-fast reference delivery without premature crossing detours',
            scene=SCENE, actors=ACTORS['midcross'], goal=[6., 0.], seed=11, duration_s=55.,
            planned_trials=['midcross_A', 'midcross_B'],
            groups='A slow3s / B slow4.5s refresh-only', unique_switch='SLOW_DELIVERY_COVERAGE',
            global_latest_and_mppi_horizon_s=3., optimization_budget_s=1.2,
            initial_plan_common=True, same_existing_compiled_build=True,
            physical_spheres=67, mppi_deadline_ms=200., command_lease_ms=250.,
            measurements=['first feedback to truly adopted new control including retries',
                'native/latest rejection and actual phase-corrected coverage',
                'actual motion at first approved intervention, path deviation and goal outcome',
                'physical sampled gap, stops and MPPI loop timing'],
            decision='If A already delivers, report non-regression rather than claim fewer rejections',
            limits='One unpaused numerical ROS pair, not identical-input population statistics',
            production_modified=False, hardware=False, gazebo=False,
            github_upload_remains_paused=True, shutdown_requested_this_turn=False))
    manifest = builder.check()
    common.register_scenes()
    tag = 'midcross_B' if args.enabled else 'midcross_A'
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT / tag, builder.OUT,
            SCENE, 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('DELIVERY_MIDCROSS_COMPLETE', tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained for inspection')


if __name__ == '__main__':
    main()

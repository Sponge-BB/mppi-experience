#!/usr/bin/env python3
"""Existing crossing pair: ordinary fallback versus fast reverse intent."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_short_prefix_rejoin as builder
import run_delivery_midcross as previous
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/short_prefix_rejoin_20261101_01'
common = previous.common
save = previous.save


def patch(trial, settings, enabled):
    # Reuse exactly one historical actor/geometry preparation, including the
    # corrected crossing-scene forwarding. Both groups retain slow CV4.5.
    previous.builder = builder
    previous.previous.builder = builder
    previous.patch(trial, settings, True)
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(node for node in tree.getroot().findall('node') if node.get('name') == 'remani_planner_node')
    common.set_env(planner, 'SHORT_PREFIX_REJOIN', int(enabled))
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(short_prefix_rejoin=bool(enabled), comparison_group='B' if enabled else 'A',
        experiment='fast reverse hint at short-prefix fallback', common_slow_cv_horizon_s=4.5,
        only_change='uniform temporary search switch coefficient when eligible fast reverse hint exists')
    save(trial / 'coupling_parameters.json', settings)
    manifest = json.loads((trial / 'manifest.json').read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['slow_delivery_window']['comparison_role'] = 'enabled common policy in both groups'
    manifest['delivery_crossing']['only_group_difference'] = 'SHORT_PREFIX_REJOIN 0/1'
    manifest['short_prefix_rejoin'] = dict(enabled=bool(enabled), runtime_switch='SHORT_PREFIX_REJOIN',
        trigger='original short_prefix and fresh same-task measured-start matched seed predicts opposite gear within1s',
        change='uniform per-search gear-switch penalty0; scope exit restores normal policy',
        common_slow_prediction_horizon_s=4.5, common_latest_mppi_horizon_s=3.,
        first_only_credit=False, new_search_history_state=False,
        start_state_and_initial_gear_unchanged=True, physical_constraints_unchanged=True,
        mppi_changed=False, search_selected_nodes_capture='common lightweight postsolve record',
        prediction_is_not_execution_commitment=True, production_modified=False,
        hardware=False, gazebo=False)
    for source in (Path(__file__), HERE / 'report_short_prefix_rejoin.py'):
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
            key_task='Avoid looping slow initializers when the fast layer brakes or changes gear',
            hypothesis='Fast reverse intent lost after short-prefix fallback can bias ordinary search toward all-forward detours',
            comparison='A existing fallback / B hint-triggered temporary search switch coefficient0',
            sole_switch='SHORT_PREFIX_REJOIN', scene=previous.SCENE,
            planned_trials=['midcross_A', 'midcross_B'], seed=11, duration_s=55., goal=[6., 0.],
            common_slow_prediction_horizon_s=4.5, common_latest_mppi_horizon_s=3.,
            common_optimization_budget_s=1.2, physical_spheres=67,
            mppi_deadline_ms=200., command_lease_ms=250.,
            measurements=['actual hint eligibility and branch activation',
                'selected search node chain, resampled path, initial polynomial and actual returned/adopted reference',
                'all native/latest failures, feedback to actual valid new control',
                'actual path, motion time, stops, physical sampled gaps and MPPI time'],
            causal_limits='One asynchronous numerical pair, not identical-input statistical inference; no activation means inconclusive for intervention',
            production_modified=False, hardware=False, gazebo=False,
            github_upload_remains_paused=True, shutdown_requested_this_turn=False))
    manifest = builder.check()
    common.register_scenes()
    tag = 'midcross_B' if args.enabled else 'midcross_A'
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT / tag, builder.OUT,
            previous.SCENE, 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('SHORT_PREFIX_REJOIN_COMPLETE', tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

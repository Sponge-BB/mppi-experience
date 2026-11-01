#!/usr/bin/env python3
"""One SIL pair: original slow CV window versus delivery-time coverage."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_delivery_window as builder
import run_deferred_quality as parent
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE/'results/delivery_window_20261101_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def patch(trial, settings, enabled):
    parent.builder = builder
    parent.patch(trial, settings, True)
    tree = ET.parse(trial/'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    common.set_env(planner, 'SLOW_DELIVERY_COVERAGE', int(enabled))
    tree.write(trial/'experiment.launch', encoding='unicode')
    settings.update(slow_delivery_coverage=bool(enabled), comparison_group='B' if enabled else 'A',
        common_deferred_quality_check=True, global_prediction_limit_s=3.,
        slow_refresh_prediction_limit_s=4.5 if enabled else 3.,
        slow_cv_extrapolation_allowance_s=1.5 if enabled else 0.,
        changed='slow refresh CV temporal coverage only; original clocks and downstream validity unchanged')
    save(trial/'coupling_parameters.json', settings)
    path = trial/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['deferred_quality']['comparison_role'] = 'common enabled policy in both groups'
    manifest['slow_delivery_window'] = dict(enabled=bool(enabled), same_binary=True,
        runtime_switch='SLOW_DELIVERY_COVERAGE', refresh_only=True,
        source_configured_horizon_s=3., planned_cv_horizon_s=4.5 if enabled else 3.,
        delivery_extension_s=1.5 if enabled else 0.,
        assumption='slow-only frozen constant-velocity extrapolation; not newly certified observation validity',
        allowance_basis='preceding source-to-latest age about1.33s, with1.5s fixed experimental allowance',
        snapshot_and_requested_epochs_unchanged=True,
        global_buffer_mppi_and_latest_limits_unchanged=True,
        common_initial_plan_first_safe=True, common_refresh_deferred_check=True,
        common_soft_execution_scales=[1., .5], common_native_hard_execution_scales=[1.],
        soft_half_window_expands_with_same_cv_horizon=True,
        optimization_budget_s=1.2, deferred_check_reserve_s=.15,
        actual_delivery_coverage_margin_must_be_measured=True,
        production_modified=False, hardware=False, gazebo=False,
        one_async_pair_not_frozen_input_performance_comparison=True)
    for source in (Path(__file__), HERE/'report_delivery_window.py'):
        if source.exists():
            shutil.copy2(source, trial/'runtime'/source.name)
            manifest['experiment_sources_sha256'][source.name] = common.lifecycle.sha(source)
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
            research_task='Cover near-future execution at actual slow-to-fast handover',
            groups='A frozen slow3s; B frozen slow4.5s with1.5s delivery allowance',
            unique_switch='SLOW_DELIVERY_COVERAGE 0/1',
            initial_plan_and_deferred_check_common=True,
            global_latest_and_mppi_cv_horizon_s=3., snapshot_epochs_not_rebased=True,
            old_candidate_replay='Same frozen source and candidate; explicit4.5s CV counterfactual, not complete solver replay',
            scene='sphere_timing_headon', seed=11, goal=[6., 0.], duration_s=55.,
            planned_trials=['headon_A', 'headon_B'], physical_spheres=67,
            optimization_budget_s=1.2, deferred_reserve_s=.15,
            mppi_deadline_ms=200., command_lease_ms=250.,
            primary_measurements=['common initial reference polynomial identity',
                'actual source/slow/latest windows and coverage margin at validation',
                'first feedback through retries to first actually adopted new control',
                'native and latest failures with precise branch and time',
                'adopted curve quality and post-handover motion', 'MPPI timing and physical sampled gaps'],
            scope='fixed-arm complete REMANI+MPPI ROS numerical SIL with constant-velocity actor',
            prediction_extension_is_model_assumption_not_hardware_reliability_proof=True,
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
    print('DELIVERY_WINDOW_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

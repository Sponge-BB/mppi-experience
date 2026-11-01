#!/usr/bin/env python3
"""One unpaused pair: nominal-only slow admission, early avoidance feedback off/on."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_fast_refresh_feedback as builder
import run_half_speed_contract as parent
import run_delivery_next_online as common
from fast_refresh_runtime import patch_runtime
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE/'results/fast_refresh_feedback_20261101_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def patch(trial, settings, enabled):
    # The parent's preparation hook selects its builder only through OUT.
    parent.builder = builder
    parent.patch(trial, settings, True)
    patch_runtime(trial)
    source = HERE/'fast_refresh_runtime.py'
    shutil.copy2(source, trial/'runtime'/source.name)
    tree = ET.parse(trial/'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    common.set_env(planner, 'FAST_REFRESH_FEEDBACK', int(enabled))
    tree.write(trial/'experiment.launch', encoding='unicode')
    settings.update(fast_refresh_feedback=bool(enabled), comparison_group='B' if enabled else 'A',
        slow_reference_nominal_only=True, same_avoidance_episode_consumed_once=True,
        avoidance_feedback='post-command actual validated dynamic intervention; kernel hysteresis unchanged',
        actual_execution_validator_changed=False, production_modified=False)
    save(trial/'coupling_parameters.json', settings)
    path = trial/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['fast_refresh_feedback'] = dict(enabled=bool(enabled), same_binary=True,
        topic='/planning/local_refresh_request', hint_size=9,
        signal='actual validated control plus existing dynamic intervention mode; not yaw/error/TTC threshold',
        identity='frozen goal token, generation, reference revision, freshness',
        consumed='once per avoidance episode across reference changes',
        unchanged=['nominal-only slow hard admission', 'full/half soft dynamic objective',
            'native 67 spheres and static/self/wheel gates', 'frontend and optimizer',
            'MPPI algorithm and actual rollout/lease-brake checker',
            '1.2s solver budget', '200ms MPPI deadline and 250ms command lease',
            '.25s cooperative poll and .75s existing cooldown/retry'],
        diagnostic_pause=False, production_modified=False, hardware=False, gazebo=False)
    for source in (Path(__file__), HERE/'fast_refresh_runtime.py'):
        shutil.copy2(source, trial/'runtime'/source.name)
        manifest['experiment_sources_sha256'][source.name] = common.lifecycle.sha(source)
    manifest['experiment_sources_sha256']['local_execution.py'] = common.lifecycle.sha(trial/'runtime/local_execution.py')
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
            purpose='Can already-approved MPPI dynamic intervention trigger slow reference delivery before old-guide blockage?',
            groups='A previous nominal-only gate plus old trigger; B same plus episode feedback',
            planned_trials=['headon_A', 'headon_B'], seed=11, goal=[6., 0.], duration_s=55.,
            physical_spheres=67, slow_nominal_only=True, optimization_budget_s=1.2,
            mppi_deadline_ms=200., command_lease_ms=250.,
            no_new_delay_error_speed_or_cost_threshold=True, no_new_retry_state_machine=True,
            runtime_and_binary_identical_between_groups=True,
            metrics=['first actual validated avoidance to trigger/request/new effective control',
                'feedback consumed count and original guide blockage timing',
                'all request outcomes, reference adoption and physical gaps',
                'actual command validator and lease-brake evidence, stops and MPPI wall time'],
            scope='fixed-arm complete REMANI+MPPI numerical ROS SIL, not Gazebo/hardware',
            single_async_pair_not_statistical_causal_performance_estimate=True,
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
    print('FAST_REFRESH_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

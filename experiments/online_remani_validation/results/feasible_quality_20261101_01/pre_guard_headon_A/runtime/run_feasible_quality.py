#!/usr/bin/env python3
"""One unpaused A/B pair: first-safe return versus same-budget safe polishing."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_feasible_quality as builder
import run_fast_refresh_feedback as parent
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE/'results/feasible_quality_20261101_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def patch(trial, settings, enabled):
    parent.builder = builder
    parent.patch(trial, settings, True)
    tree = ET.parse(trial/'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    common.set_env(planner, 'COLLAB_PREFIX', 1)
    common.set_env(planner, 'FEASIBLE_QUALITY_POLISH', int(enabled))
    tree.write(trial/'experiment.launch', encoding='unicode')
    settings.update(feasible_quality_polish=bool(enabled), prefix_initializer=True,
        legacy_prefix=True, comparison_group='B' if enabled else 'A',
        fast_refresh_feedback=True, solver_budget_s=1.2,
        changed='retain first native-safe accepted iterate and continue within original budget only in B',
        production_modified=False)
    save(trial/'coupling_parameters.json', settings)
    path = trial/'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial/'coupling_parameters.json')
    manifest['legacy_factorial_runtime']['prefix'] = True
    manifest['feasible_quality'] = dict(enabled=bool(enabled), same_binary=True,
        switch='FEASIBLE_QUALITY_POLISH', first_safe_retained=True,
        replacement='strictly lower original objective; accepted x; original wheel/native pass before budget',
        final_and_latest_gates_unchanged=True,
        unchanged=['original 1.2s budget, objective and weights',
            'MPPI prefix initializer and original search fallback',
            'fast avoidance feedback, nominal-only slow hard gate',
            'native 67 spheres, static/self/wheel and latest dynamic gates',
            'actual MPPI rollout and lease-brake validator',
            '200ms MPPI deadline, 250ms command lease',
            'original cooldown, retry and 1.1s delivery delay'],
        candidate_search_recorded=False, diagnostic_pause=False,
        async_pair_not_identical_input=True, hardware=False, gazebo=False,
        production_modified=False)
    for source in (Path(__file__), HERE/'report_feasible_quality.py'):
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
            purpose='Does first-safe early exit explain the wide and slow native candidate?',
            groups='A first-safe early return; B keep first-safe and polish within same budget',
            unique_switch='FEASIBLE_QUALITY_POLISH 0/1',
            prefix_and_fast_feedback_enabled_both=True,
            seed=11, goal=[6., 0.], duration_s=55.,
            planned_trials=['headon_A', 'headon_B'], physical_spheres=67,
            solver_budget_s=1.2, mppi_deadline_ms=200., command_lease_ms=250.,
            primary_measurements=['same-solve initial, first-safe and selected original cost components',
                'same-solve first-safe and selected duration, geometry and native verdict',
                'all requests and latest acceptance; actual new-reference control',
                'physical sample gap, actual path/time/stopping and MPPI wall time'],
            interpretation='lower original cost need not mean shorter path or time; no improvement is informative',
            incomplete_old_logs_not_full_input_replay=True,
            scope='fixed-arm complete REMANI+MPPI ROS numerical SIL; not Gazebo/hardware',
            single_async_pair_not_statistical_causal_estimate=True,
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
    print('FEASIBLE_QUALITY_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

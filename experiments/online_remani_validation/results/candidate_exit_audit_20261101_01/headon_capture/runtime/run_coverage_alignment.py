#!/usr/bin/env python3
"""One head-on A/B; collect coverage, boundary, adoption and execution together."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_coverage_alignment as builder
import run_time_indexed_collision as previous
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/coverage_alignment_20261031_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, enabled):
    previous.builder.OUT = builder.OUT
    # A is exactly the preceding time-indexed prototype, with its old short
    # window and epoch rules. B switches the coordinated consistency changes.
    previous.patch(trial, settings, 'headon', True)
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node')
                   if n.get('name') == 'remani_planner_node')
    common = previous.previous.prior.common
    common.set_env(planner, 'COVERAGE_ALIGNMENT', int(enabled))
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(coverage_alignment=bool(enabled),
        comparison_group='B' if enabled else 'A',
        baseline='previous time-indexed prototype: 1.2s guide, guide padding and original epoch rules',
        coordinated_change='continuous native-CV model, complete valid prediction window, measured boundary epoch and phase-based reference acceptance',
        mppi_control_and_braking_validation_unchanged=True)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['coverage_alignment'] = dict(enabled=bool(enabled),
        group='B' if enabled else 'A', runtime_switch='COVERAGE_ALIGNMENT',
        same_binary=True, execution_weight=0., handover_weight=0.,
        baseline='preceding time-indexed experimental B, not the unmodified REMANI baseline',
        changed=dict(prediction_window='complete valid frozen CV window',
            continuous_radius='native configured uncertainty; no swept-sampling guide padding',
            boundary_epoch='timestamp of the measured start state',
            reference_acceptance='current-state check and newly projected candidate phase'),
        unchanged=['67 physical collision spheres', 'static ESDF and self-collision',
                   'wheel constraints', '1.2s optimization budget', 'frontend search',
                   'MPPI algorithm and execution/braking validation', '200ms/250ms execution settings'],
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
            purpose='Resolve prediction-window and start-time inconsistency with one head-on pair',
            planned_trials=['headon_A', 'headon_B'],
            baseline='previous time-indexed prototype, not original REMANI',
            single_run_captures=['exact frozen dynamic input', 'boundary measurement timestamp',
                'candidate and acceptance phase', 'native/latest gate outcomes',
                'actual new-reference adoption', 'MPPI execution and braking failures',
                'complete loop timing and command-publication intervals'],
            offline='use the same recorded candidates to distinguish window, radius and time-mapping effects',
            seed=11, goal=[6., 0.], duration_s=55., collision_spheres=67,
            optimization_budget_s=1.2, mppi_budget_ms=200., command_lease_ms=250.,
            same_binary=True, production_modified=False,
            scope='fixed-arm complete REMANI+MPPI numerical ROS SIL',
            gif='one actual-record head-on A/B GIF', github_upload_remains_paused=True))
    manifest = builder.check()
    common = previous.previous.prior.common
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT / args.tag, builder.OUT, 'sphere_timing_headon',
            'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('COVERAGE_ALIGNMENT_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

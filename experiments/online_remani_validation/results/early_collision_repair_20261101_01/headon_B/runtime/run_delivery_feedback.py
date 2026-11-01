#!/usr/bin/env python3
"""One fresh head-on pair: current coverage model, delivery feedback off/on."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import run_coverage_alignment as previous
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/delivery_feedback_20261031_01'
DELAY_S = 1.1


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, enabled):
    # Both arms inherit the completed coverage-alignment B configuration.
    previous.patch(trial, settings, True)
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node')
                   if n.get('name') == 'remani_planner_node')
    common = previous.previous.previous.prior.common
    for key, value in dict(COUPLING_EXEC_WEIGHT=0,
                           COUPLING_HAND_WEIGHT=int(enabled),
                           COUPLING_DELIVERY_DELAY=DELAY_S).items():
        common.set_env(planner, key, value)
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(execution_weight=0., handover_weight=float(enabled),
        comparison_group='B' if enabled else 'A',
        baseline='completed coverage-alignment B with feedback disabled',
        delivery_feedback=bool(enabled), delivery_target_delay_s=DELAY_S,
        delay_rationale='preselected from preceding request-to-acceptance about 1.11s; estimate, not an execution promise',
        feedback_semantics='frozen checked MPPI position, heading and speed; soft spatial matching in native first three pieces; not time synchronization',
        coverage_alignment=True)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['coverage_alignment'].update(group='common enabled configuration',
        handover_weight=float(enabled), execution_weight=0.,
        baseline='completed coverage-alignment B')
    manifest['delivery_feedback'] = dict(enabled=bool(enabled),
        group='B' if enabled else 'A', same_binary=True,
        execution_weight=0., handover_weight=float(enabled),
        target_delay_s=DELAY_S, target_clock='source-rollout time plus measured seed age plus 1.1s',
        target_is_future_ground_truth=False,
        feedback='existing frozen soft spatial position/heading/velocity objective',
        unchanged=['complete frozen CV window', 'measured boundary epoch',
                   'phase-based latest acceptance', 'native 67 physical spheres',
                   'wheel limits', 'frontend', '1.2s optimization budget',
                   'MPPI algorithm', '200ms/250ms execution timing'],
        production_modified=False,
        scope='fixed-arm complete REMANI+MPPI numerical ROS SIL')
    for source in (Path(__file__),):
        shutil.copy2(source, trial / 'runtime' / source.name)
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
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Test whether fast-layer delivery-state feedback improves actual reference connection',
            planned_trials=['headon_A', 'headon_B'],
            baseline='latest coverage-alignment B, not original REMANI',
            changed='only delivery-state soft weight 0 versus 1',
            execution_weight=0., delivery_target_delay_s=DELAY_S,
            delay_rationale='preceding measured request-to-acceptance about 1.11s',
            same_binary=True, seed=11, goal=[6., 0.], duration_s=55.,
            physical_spheres=67, optimization_budget_s=1.2,
            mppi_budget_ms=200., command_lease_ms=250.,
            same_run_metrics=['feedback eligibility and nonzero cost/gradient',
                'frozen predicted target versus actual state at its timestamp',
                'actual reference adoption position/heading/speed residuals',
                'motion and command changes for two seconds after adoption',
                'request-to-first-new-reference-valid-control',
                'all failed requests', 'goal/path/physical gaps',
                'complete MPPI loop and solver times'],
            output='one paired GIF, simple data plot, concise Chinese TXT',
            production_modified=False, github_upload_remains_paused=True,
            scope='fixed-arm complete REMANI+MPPI numerical ROS SIL'))
    builder = previous.builder
    manifest = builder.check()
    common = previous.previous.previous.prior.common
    common.register_scenes()
    with common.clean_environment():
        # The preparation hook sets the treatment; both groups retain the same
        # engineering preset and all other lifecycle options.
        valid = common.lifecycle.run_trial(OUT / args.tag, builder.OUT,
            'sphere_timing_headon', 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, bool(args.enabled)),
            **trial_kwargs(manifest))
    print('DELIVERY_FEEDBACK_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Fresh paired SIL trial with one frozen short-path feedback weight."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_handoff_alignment as builder
import run_frontend_rejoin as previous
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/handoff_alignment_20261031_01'


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, scene, weight):
    previous.builder.OUT = builder.OUT
    previous.patch(trial, settings, scene, 'penalty')
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node') if n.get('name') == 'remani_planner_node')
    for key, value in dict(HANDOFF_ALIGNMENT_PROBE=1, COUPLING_EXEC_WEIGHT=weight,
                           COUPLING_HAND_WEIGHT=0).items():
        previous.prior.common.set_env(planner, key, value)
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(execution_weight=float(weight), handover_weight=0.,
        handoff_alignment_feedback='frozen short-path spatial soft objective',
        no_future_state_hard_boundary=True, original_time_scaled_validation=True,
        failure_sphere_provenance=True, feedback_is_execution_commitment=False)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = previous.prior.common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['handoff_alignment'] = dict(execution_weight=weight, handover_weight=0.,
        collision_and_wheel_limits_unchanged=True, acceptance_and_retry_policy_unchanged=True,
        frontend_penalty_mode=True, feedback='one frozen MPPI prediction per native request',
        scope='fixed-arm full REMANI+MPPI numerical ROS SIL', production_modified=False)
    manifest['frontend_rejoin']['mppi_and_backend_unchanged'] = (weight == 0.)
    shutil.copy2(Path(__file__), trial / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = previous.prior.common.lifecycle.sha(Path(__file__))
    save(path, manifest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', choices=('headon', 'static', 'midcross'), default='headon')
    parser.add_argument('--weight', type=float, choices=(0., 1.), required=True)
    parser.add_argument('--tag', required=True)
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial tag')
    OUT.mkdir(exist_ok=True)
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Identify swept-vs-static rejection and test frozen fast-path feedback under original acceptance',
            offline='same rejected candidate and latest observed CV snapshot; time-mapping counterfactual only',
            initial_online=['headon_baseline', 'headon_feedback'],
            followup='static regression if feedback is active and useful; targeted failure investigation otherwise',
            frozen_execution_targets_s=[.2, .4, .6, .8, 1.],
            execution_weights=[0., 1.], handover_weight=0.,
            original_dynamic_validation_scales=[1., .5], backend_budget_s=1.2,
            mppi_budget_ms=200., lease_ms=250., spheres=67,
            production_modified=False, scope='fixed-arm full REMANI+MPPI numerical ROS SIL'))
    manifest = builder.check()
    common = previous.prior.common
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT / args.tag, builder.OUT, 'sphere_timing_' + args.scene,
            'execution' if args.weight else 'engineering', 11, 55., args.weight, 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, args.scene, args.weight),
            **trial_kwargs(manifest))
    print('HANDOFF_ALIGNMENT_ONLINE_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

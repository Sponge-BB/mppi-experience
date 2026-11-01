#!/usr/bin/env python3
"""Small frozen-frontend diagnosis followed by fresh full online SIL trials."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import yaml

import build_frontend_rejoin_v2 as builder
import run_static_agility as prior
import run_native67_probe as probe
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/frontend_rejoin_20261031_01'
MODES = ('baseline', 'penalty', 'yaw', 'both')


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, scene, mode, diagnostic=False):
    prior.builder.OUT = builder.OUT
    prior.patch(trial, settings, scene, True)
    tree = ET.parse(trial / 'experiment.launch')
    node = next(n for n in tree.getroot().findall('node')
                if n.get('name') == 'remani_planner_node')
    values = dict(FRONTEND_REJOIN_MODE=mode, FRONTEND_REJOIN_PROBE=int(diagnostic),
        FRONTEND_REJOIN_LOG=trial / 'frontend_rejoin.jsonl',
        FRONTEND_REJOIN_DONE_PATH=trial / 'diagnostic_complete.json',
        STATIC_WIDTH_TRACE_PATH=trial / 'static_width_trace.jsonl',
        STATE_CONTRACT_CAPTURE=1, DELIVERY_REPLAY_KIND='')
    for key, value in values.items():
        prior.common.set_env(node, key, value)
    tree.write(trial / 'experiment.launch', encoding='unicode')
    params = yaml.safe_load((trial / 'parameters.yaml').read_text())
    prefix = '/remani_planner_node/search/'
    resolved = {key: float(params[prefix + key]) for key in (
        'traj_steer_penalty', 'traj_steer_change_penalty', 'yaw_resolution', 'lambda_heu')}
    if mode in ('penalty', 'both'):
        resolved['traj_steer_penalty'] *= .25
        resolved['traj_steer_change_penalty'] *= .25
    if mode in ('yaw', 'both'):
        resolved['yaw_resolution'] = .3
    settings.update(frontend_rejoin_mode=mode, diagnostic_pause=diagnostic,
                    genuine_online_closed_loop=not diagnostic,
                    navigation_metrics_valid=not diagnostic,
                    frontend_probe=diagnostic, resolved_frontend_parameters=resolved)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = prior.common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['frontend_rejoin'] = dict(mode=mode, probe=diagnostic,
        resolved_online_parameters=resolved,
        parameter_note='C++ mode applies after ROS param load; ROS dump alone shows pre-override values',
        static_early_detour=True, geometry_and_feasibility_limits_unchanged=True,
        mppi_and_backend_unchanged=True, production_modified=False,
        scope='native frontend scoring/binning; fixed-arm numerical ROS SIL, not hardware')
    shutil.copy2(Path(__file__), trial / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = prior.common.lifecycle.sha(Path(__file__))
    save(path, manifest)


def run_probe(trial):
    previous_prepare, previous_env = probe.prepare_capture, probe.diagnostic_environment

    def prepare(out, build, seed=11, ordinal=1):
        settings = prior.common.lifecycle.prepare_trial(
            out, build, 'sphere_timing_static', 'engineering', seed, 55., 0., 0., False)
        patch(out, settings, 'static', 'baseline', True)
        return json.loads((out / 'manifest.json').read_text())

    def environment(out, seed, ordinal, inherited=None):
        env = previous_env(out, seed, ordinal, inherited)
        env.update(DELIVERY_REPLAY_KIND='', FRONTEND_REJOIN_MODE='baseline',
                   FRONTEND_REJOIN_PROBE='1')
        return env

    try:
        probe.prepare_capture, probe.diagnostic_environment = prepare, environment
        status = probe.run_capture(trial, builder.OUT, 11, 1)
    finally:
        probe.prepare_capture, probe.diagnostic_environment = previous_prepare, previous_env
    if not status['diagnostic_complete']:
        raise RuntimeError('Frontend capture did not occur; records retained')
    print('FRONTEND_SAME_INPUT_COMPLETE', trial, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('probe', 'online'))
    parser.add_argument('--mode', choices=MODES, default='baseline')
    parser.add_argument('--scene', choices=('static', 'headon', 'midcross'), default='static')
    parser.add_argument('--tag', required=True)
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial name')
    if args.action == 'probe' and (args.scene != 'static' or args.mode != 'baseline'):
        raise ValueError('Probe owns the fixed static factorial; use baseline mode')
    OUT.mkdir(exist_ok=True)
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Resolve static wide detour with same-input frontend diagnosis and immediate closed-loop verification',
            same_input=['baseline', 'penalty', 'yaw', 'both', 'baseline_repeat'],
            selection='Prefer the smallest successful frontend-only change; all attempted outcomes retained',
            online=['static_baseline', 'static_selected', 'headon_selected'],
            allowed_additional='A targeted check only if the selected online result exposes a concrete failure',
            native_budget_s=1.2, mppi_budget_ms=200, lease_ms=250,
            spheres=67, goal=[6., 0.], production_modified=False,
            scope='fixed-arm REMANI+MPPI numerical ROS SIL; not Gazebo or real vehicle'))
    manifest = builder.check()
    prior.common.register_scenes()
    with prior.common.clean_environment():
        if args.action == 'probe':
            run_probe(OUT / args.tag)
        else:
            valid = prior.common.lifecycle.run_trial(
                OUT / args.tag, builder.OUT, 'sphere_timing_' + args.scene, 'engineering',
                11, 55., 0., 0., False,
                prior.common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
                reserve_bytes=prior.common.RESERVE + prior.common.TRIAL_ALLOWANCE,
                prepare_hook=lambda trial, settings: patch(trial, settings, args.scene, args.mode),
                **trial_kwargs(manifest))
            print('FRONTEND_ONLINE_COMPLETE', args.tag, valid, flush=True)
            if not valid:
                raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

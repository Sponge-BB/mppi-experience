#!/usr/bin/env python3
"""One native first-refresh candidate audit; optional justified online recovery."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import build_candidate_exit_audit as builder
import run_delivery_feedback as previous
import run_delivery_next_online as common
from run_native_collision_3d import trial_kwargs

HERE = Path(__file__).resolve().parent
OUT = HERE / 'results/candidate_exit_audit_20261101_01'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def patch(trial, settings, recover):
    previous.previous.builder.OUT = builder.OUT
    previous.patch(trial, settings, True)
    tree = ET.parse(trial / 'experiment.launch')
    planner = next(n for n in tree.getroot().findall('node')
                   if n.get('name') == 'remani_planner_node')
    environment = dict(CANDIDATE_EXIT_AUDIT=int(not recover),
        CANDIDATE_EXIT_RECOVER=int(recover),
        CANDIDATE_EXIT_AUDIT_DIR=str(trial / 'exit_audit'),
        FAILURE_OWNED_PID_FILE=str(trial / 'owned_pids.txt'),
        CANDIDATE_COLLISION_REPAIR=0)
    for key, value in environment.items():
        common.set_env(planner, key, value)
    tree.write(trial / 'experiment.launch', encoding='unicode')
    settings.update(candidate_exit_audit=not recover, candidate_exit_recover=bool(recover),
        capture='first dynamic refresh, independent of solver return code',
        diagnostic_pause=not recover, online_navigation_timing_valid=bool(recover),
        optimization_budget_s=1.2, previous_collision_repair_enabled=False,
        continuation='conditional same-accepted-x weight1/10 restart, equal original remaining budget',
        production_modified=False)
    save(trial / 'coupling_parameters.json', settings)
    path = trial / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['coupling_experiment'] = settings
    manifest['parameters_sha256'] = common.lifecycle.sha(trial / 'coupling_parameters.json')
    manifest['candidate_exit_audit'] = dict(enabled=not recover,
        planner_environment=environment, recovery_enabled=bool(recover),
        original_budget_s=1.2,
        capture='first dynamic refresh regardless of solver exit result',
        candidate='actual last accepted solver vector, independently reconstructed polynomial',
        snapshot='same-process native ESDF, frozen CV, boundary, feedback and candidate epoch',
        continuation='only environmental native collision: fresh weight1 and weight10 solver from same vector',
        equal_remaining_budget=True, diagnostic_candidates_published=False,
        original_native_and_latest_gates_retained=True,
        whole_run_online_timing_valid=bool(recover),
        scope=('fixed-arm complete REMANI+MPPI numerical ROS SIL, normal online candidate recovery'
               if recover else 'fixed-arm complete REMANI+MPPI numerical ROS SIL with one paused candidate mechanism audit'),
        production_modified=False)
    shutil.copy2(Path(__file__), trial / 'runtime' / Path(__file__).name)
    manifest['experiment_sources_sha256'][Path(__file__).name] = common.lifecycle.sha(Path(__file__))
    save(path, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', default='headon_capture')
    parser.add_argument('--online-recover', dest='recover', action='store_true')
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        raise ValueError('Invalid trial name')
    OUT.mkdir(exist_ok=True)
    if not (OUT / 'plan.json').exists():
        save(OUT / 'plan.json', dict(created_utc=datetime.now(timezone.utc).isoformat(),
            purpose='Determine whether slow-layer failure discarded a feasible accepted iterate or retained a genuinely infeasible candidate',
            planned_capture='headon_capture', additional_online_trial='only if mechanism result warrants a concrete recovery change',
            seed=11, goal=[6., 0.], duration_s=55.,
            physical_spheres=67, optimization_budget_s=1.2,
            mppi_budget_ms=200., command_lease_ms=250.,
            capture='first dynamic refresh independent of result; same-process full native audit',
            comparison='if collision, unmodified restart versus weighted restart at identical accepted x and remaining budget',
            diagnostic_timing_is_not_online_delivery=True,
            primary_metrics=['original solver exit code', 'accepted iterate full native verdict',
                'common frozen input consistency', 'conditional restart full native verdict and time'],
            eventual_online_metrics=['request to validated new-reference control',
                'position/heading mismatch at adoption', 'post-adoption stop and reversal', 'physical gap'],
            production_modified=False, hardware=False, gazebo=False,
            github_upload_remains_paused=True, shutdown_after_reports_requested=True))
    manifest = builder.check()
    common.register_scenes()
    with common.clean_environment():
        valid = common.lifecycle.run_trial(OUT / args.tag, builder.OUT,
            'sphere_timing_headon', 'engineering', 11, 55., 0., 0., False,
            common.lifecycle.sha(builder.OUT / 'build_manifest.json'),
            reserve_bytes=common.RESERVE + common.TRIAL_ALLOWANCE,
            prepare_hook=lambda trial, settings: patch(trial, settings, args.recover),
            **trial_kwargs(manifest))
    print('CANDIDATE_EXIT_AUDIT_COMPLETE', args.tag, valid, flush=True)
    if not valid:
        raise RuntimeError('Interface-invalid trial retained')


if __name__ == '__main__':
    main()

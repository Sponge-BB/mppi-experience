#!/usr/bin/env python3
"""Two NEW same-process paused cost-stop diagnostics, no vehicle I/O.

OFF / ON / OFF at the same .6s budget and exact captured inputs. This changes
only eligibility for relative-cost stopping, not objective/initialization/limits.
This is not a historical replay, online delivery benchmark or full recovery.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path

import run_backend_feasibility as common
from run_coupled_objective import HERE, check_build, check_space, sha
from run_dual_timescale import verify_production

JOBS = (dict(seed=11, ordinal=2), dict(seed=12, ordinal=2))
PHASES = ('delivery_nominal_060', 'delivery_stop_on_060', 'delivery_repeat_060')
ORIGINAL_ENV = common.diagnostic_environment
ORIGINAL_PREPARE = common.prepare_capture


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def environment(out, seed, ordinal, inherited=None):
    env = ORIGINAL_ENV(out, seed, ordinal, inherited)
    # In C++ this flag is insufficient on its own: running() and the exact
    # private diagnostic phase are also required. Normal navigation stays OFF.
    env['FEASIBILITY_STOP_GATE'] = '1'
    return env


def prepare(out, build, seed, ordinal):
    runtime = ORIGINAL_PREPARE(out, build, seed, ordinal)
    path = out/'coupling_parameters.json'
    settings = json.loads(path.read_text())
    settings.update(feasibility_stop_diagnostic=True,
        diagnostic_budgets_s=[.6, .6, .6], diagnostic_phases=list(PHASES),
        diagnostic_treatment_order=['OFF', 'ON', 'OFF'],
        treatment='gate only relative-cost stop by unchanged native and projected hard checks',
        objective_unchanged=True, initialization_unchanged=True,
        constraints_unchanged=True, total_budget_reset_on_rejected_stop=False,
        full_native_map_cross_process_replay=False,
        navigation_metrics_valid=False, candidate_publication=False)
    write_json(path, settings)
    manifest_path = out/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest.update(coupling_experiment=settings, parameters_sha256=sha(path),
        feasibility_stop_diagnostic=True,
        feasibility_stop_environment={'FEASIBILITY_STOP_GATE': '1',
            'phase_only': 'delivery_stop_on_060', 'normal_navigation_enabled': False})
    write_json(manifest_path, manifest)
    return runtime


def validate_build(build):
    manifest, _ = check_build(build)
    if not manifest.get('feasibility_stop_experiment') or manifest.get('prepared_only'):
        raise ValueError('Requires a completed isolated feasibility-stop build')
    if not manifest.get('seed_parameterization_experiment'):
        raise ValueError('Seed-fixed diagnostic parent required')
    if manifest.get('feasibility_stop_diagnostic_phases', list(PHASES)) != list(PHASES):
        raise ValueError('Unexpected replay phase contract')
    verify_production(build)
    return manifest


def run(build, out):
    build, out = build.resolve(), out.resolve()
    manifest = validate_build(build)
    check_space(out.parent, 2*1024**3)
    out.mkdir(parents=True, exist_ok=False)
    build_hash = sha(build/'build_manifest.json')
    write_json(out/'plan.json', dict(created=datetime.now(timezone.utc).isoformat(),
        scope=__doc__, build=str(build), build_manifest_sha256=build_hash,
        diagnostic_jobs=list(JOBS), phases=list(PHASES), budgets_s=[.6, .6, .6],
        treatment_order=['OFF', 'ON', 'OFF'], source_sha256=sha(Path(__file__)),
        execution_weight=0., handover_weight=0., constraints_relaxed=False,
        no_outcome_selected_rerun=True, diagnostic_candidates_published=False,
        native_map_buffer_dumped=False, navigation_metrics_valid=False,
        limitations=['Fresh same-process captures, not replay of old request IDs.',
            'Captured ordinal follows current execution; lack of a capture or no gate activation is retained.',
            'Guard scan/logging cost counts against unchanged solver deadline; outside-budget posterior audits are separate.',
            'No new restoration objective, no hardware, no Gazebo, no online-delivery improvement claim.']))
    before = verify_production(build)
    completed, runtime = [], None
    common.diagnostic_environment, common.prepare_capture = environment, prepare
    try:
        for job in JOBS:
            target = out/('static_stop_seed%d_refresh%d' % (job['seed'], job['ordinal']))
            observed, captured = common.run_capture(target, build, job['seed'], job['ordinal'],
                build_hash, runtime)
            if runtime is None:
                runtime = observed
            completed.append(dict(trial=target.name, kind='paused', captured=captured, **job))
            write_json(out/'progress.json', completed)
    except Exception as error:
        write_json(out/'stopped.json', dict(completed=completed,
            reason=type(error).__name__+': '+str(error), retained_incomplete=True))
        raise
    finally:
        common.diagnostic_environment, common.prepare_capture = ORIGINAL_ENV, ORIGINAL_PREPARE
    if verify_production(build) != before or sha(build/'build_manifest.json') != build_hash:
        raise RuntimeError('Frozen build or production inputs changed')
    write_json(out/'complete.json', dict(complete=True, completed=completed,
        navigation_metrics_valid=False, diagnostic_candidates_published=False))
    print('FEASIBILITY_STOP_COMPLETE', out, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.build, args.out)

#!/usr/bin/env python3
"""Fresh paused fixed-input budget/scaling diagnostic; never publishes candidates.

Five paired solves per capture separate original stopping, feasibility stopping,
wall budget, and solver-coordinate scaling. This is not online navigation.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path

import run_backend_feasibility as common
from run_coupled_objective import check_build, check_space, sha
from run_dual_timescale import verify_production

JOBS = (dict(seed=11, ordinal=1), dict(seed=12, ordinal=2),
        dict(seed=11, ordinal=2), dict(seed=12, ordinal=1))
PHASES = ('budget_original_060', 'budget_reference_060', 'budget_extended_300',
          'budget_scaled_300', 'budget_repeat_060')
BUDGETS = (.6, .6, 3., 3., .6)
TREATMENTS = ('original_stop', 'feasibility_stop', 'extended_budget',
              'extended_budget_scaled', 'repeat_feasibility_stop')
ORIGINAL_ENV = common.diagnostic_environment
ORIGINAL_PREPARE = common.prepare_capture


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def environment(out, seed, ordinal, inherited=None):
    source = os.environ if inherited is None else inherited
    clean = {k:v for k,v in source.items() if not k.startswith(
        ('FEASIBILITY_', 'BUDGET_', 'SOLVER_SCALING_', 'SEED_PARAMETERIZATION_'))}
    env = ORIGINAL_ENV(out, seed, ordinal, clean)
    env['FEASIBILITY_STOP_GATE'] = '1'
    return env


def prepare(out, build, seed, ordinal):
    runtime = ORIGINAL_PREPARE(out, build, seed, ordinal)
    path = out/'coupling_parameters.json'
    settings = json.loads(path.read_text())
    settings.update(budget_scaling_diagnostic=True,
        diagnostic_budgets_s=list(BUDGETS), diagnostic_phases=list(PHASES),
        diagnostic_treatment_order=list(TREATMENTS),
        objective_unchanged=True, initialization_unchanged=True,
        constraints_unchanged=True, total_budget_reset_on_rejected_stop=False,
        scaling_phase='budget_scaled_300', scaling_fixed_per_solve=True,
        scaling_recipe='x=x0+D*z; position D from adjacent initial times, exponent 7; time/angle/arm D=1',
        scaling_preparation_inside_budget=True,
        full_native_map_cross_process_replay=False,
        navigation_metrics_valid=False, candidate_publication=False)
    write_json(path, settings)
    manifest_path = out/'manifest.json'
    manifest = json.loads(manifest_path.read_text())
    manifest.update(coupling_experiment=settings, parameters_sha256=sha(path),
        budget_scaling_diagnostic=True,
        feasibility_stop_environment={'FEASIBILITY_STOP_GATE':'1',
            'phases':list(PHASES[1:]), 'normal_navigation_enabled':False})
    write_json(manifest_path, manifest)
    return runtime


def validate_build(build):
    manifest, _ = check_build(build)
    if not manifest.get('budget_scaling_experiment') or manifest.get('prepared_only'):
        raise ValueError('Requires a completed isolated budget/scaling build')
    if manifest.get('budget_scaling_diagnostic_phases') != list(PHASES):
        raise ValueError('Unexpected five-phase replay contract')
    verify_production(build)
    return manifest


def run(build, out):
    build, out = build.resolve(), out.resolve()
    validate_build(build)
    check_space(out.parent, 2*1024**3)
    out.mkdir(parents=True, exist_ok=False)
    build_hash = sha(build/'build_manifest.json')
    write_json(out/'plan.json', dict(created=datetime.now(timezone.utc).isoformat(),
        scope=__doc__, build=str(build), build_manifest_sha256=build_hash,
        diagnostic_jobs=list(JOBS), phases=list(PHASES), budgets_s=list(BUDGETS),
        treatment_order=list(TREATMENTS), source_sha256=sha(Path(__file__)),
        execution_weight=0., handover_weight=0., constraints_relaxed=False,
        no_outcome_selected_rerun=True, diagnostic_candidates_published=False,
        native_map_buffer_dumped=False, navigation_metrics_valid=False,
        primary_endpoint='normal solver return + native pass + unchanged full projection pass, all before deadline; private backend only',
        limitations=['Fresh same-process captures, not replay of old request IDs.',
            'Missing capture, missing gate activation and failed jobs are retained, never replaced by selected reruns.',
            'Initialization, physical objective, wheel limits, boundary conditions and hard gates unchanged.',
            'Coordinate scaling preparation and feasibility-stop checks count inside each wall budget.',
            'Posterior iterate audits outside budget do not count as delivery.',
            'Three seconds is an experiment upper limit, not a production/default change.',
            'No robot commands, no Gazebo, no online-delivery or dynamic-safety claim.']))
    before = verify_production(build)
    completed, runtime = [], None
    common.diagnostic_environment, common.prepare_capture = environment, prepare
    try:
        for job in JOBS:
            target = out/('static_budget_seed%d_refresh%d' % (job['seed'], job['ordinal']))
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
    print('BUDGET_SCALING_COMPLETE', out, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    run(args.build, args.out)

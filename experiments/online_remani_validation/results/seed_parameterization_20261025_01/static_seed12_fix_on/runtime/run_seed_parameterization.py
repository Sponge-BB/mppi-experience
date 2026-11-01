#!/usr/bin/env python3
"""Four predeclared static OFF/ON loops; no hardware or paused replay."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path

from run_coupled_objective import check_build, check_space, run_trial, sha


def settings_hook(enabled):
    def hook(out, settings):
        settings.update(seed_parameterization_fix=enabled,
            seed_parameterization_scope='geometry/time parameterization only; exact OFF projection',
            seed_parameterization_search_probe=True, seed_parameterization_replay=False)
    return hook


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    build, out = a.build.resolve(), a.out.resolve()
    m, _ = check_build(build)
    if not m.get('seed_parameterization_experiment') or m.get('prepared_only'):
        raise ValueError('Compiled isolated seed build required')
    check_space(out.parent, 2*1024**3)
    out.mkdir(parents=True, exist_ok=False)
    jobs = [dict(seed=11, fix=False), dict(seed=11, fix=True),
            dict(seed=12, fix=True), dict(seed=12, fix=False)]
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'), jobs=jobs,
        duration_s=48., scene='static', group='coupled', execution_weight=0., handover_weight=0.,
        diagnostic_capture=True, domain_recovery=True, state_contract=True,
        signed_low_speed=True, boundary_admission=False, clearance_recovery=False,
        no_outcome_selected_rerun=True, production_sources_sha256=m['production_sources_sha256'],
        scope='fixed-arm native ROS numerical SIL, not Gazebo/hardware; closed-loop requests diverge',
        single_treatment='SEED_PARAMETERIZATION_FIX', paused_replay=False)
    (out/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    completed, runtime = [], None
    saved = {k:v for k,v in os.environ.items() if k.startswith(('DELIVERY_', 'SEED_PARAMETERIZATION_'))}
    try:
        for key in list(os.environ):
            if key.startswith(('DELIVERY_', 'SEED_PARAMETERIZATION_')):
                os.environ.pop(key)
        os.environ['DELIVERY_SEARCH_PROBE'] = '1'
        for job in jobs:
            enabled, seed = job['fix'], job['seed']
            name = 'static_seed%d_fix_%s' % (seed, 'on' if enabled else 'off')
            os.environ['SEED_PARAMETERIZATION_FIX'] = str(int(enabled))
            valid = run_trial(out/name, build, 'static', 'coupled', seed, 48., 0., 0., False,
                plan['build_manifest_sha256'], expected_runtime_sources=runtime,
                reserve_bytes=2*1024**3, diagnostic_capture=True, domain_recovery=True,
                state_contract=True, signed_low_speed=True, boundary_admission=False,
                prepare_hook=settings_hook(enabled))
            observed = json.loads((out/name/'manifest.json').read_text())['experiment_sources_sha256']
            if runtime is None:
                runtime = observed
            if observed != runtime:
                raise RuntimeError('Full runtime snapshot drift')
            completed.append(dict(trial=name, seed=seed, fix=enabled, valid=valid))
            (out/'progress.json').write_text(json.dumps(completed, indent=2)+'\n')
            if not valid:
                raise RuntimeError('Interface failure; all records retained, no rerun')
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=completed, reason=str(exc)), indent=2)+'\n')
        raise
    finally:
        for key in list(os.environ):
            if key.startswith(('DELIVERY_', 'SEED_PARAMETERIZATION_')):
                os.environ.pop(key)
        os.environ.update(saved)
    (out/'complete.json').write_text(json.dumps(dict(complete=True, completed=completed), indent=2)+'\n')
    print('SEED_PAIR_COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()

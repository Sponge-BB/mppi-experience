#!/usr/bin/env python3
"""Three small online reproductions; only the simulated command topic is used."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from run_coupled_objective import run_trial, check_build, sha, check_space


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    build, out = a.build.resolve(), a.out.resolve()
    manifest, _ = check_build(build)
    if not manifest.get('delivery_diagnostics'):
        raise ValueError('Requires diagnostic-only build')
    check_space(out.parent)
    out.mkdir(parents=True, exist_ok=False)
    jobs = [('static','coupled',11), ('static','engineering',11), ('dynamic_four','engineering',11)]
    plan = dict(created=datetime.now(timezone.utc).isoformat(), build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'), jobs=jobs, duration_s=48,
        diagnostic_only=True, not_performance_comparison=True, algorithm_parameters_unchanged=True,
        purpose='classify projected candidate rejection, invalid objective sample and target-entry rejection')
    (out/'diagnostic_plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    completed = []
    try:
        for scene, group, seed in jobs:
            trial = out/('%s_%s_seed%02d' % (scene,group,seed))
            valid = run_trial(trial, build, scene, group, seed, 48., 1., 1., False,
                              plan['build_manifest_sha256'], diagnostic_capture=True)
            completed.append(dict(trial=trial.name, valid=valid))
            if not valid:
                raise RuntimeError('Interface/setup failure; retained without rerunning')
    except Exception as exc:
        (out/'diagnostic_stopped.json').write_text(json.dumps(dict(completed=completed,reason=str(exc)), indent=2)+'\n')
        raise
    (out/'diagnostic_complete.json').write_text(json.dumps(dict(complete=True,completed=completed), indent=2)+'\n')
    print('DIAGNOSTIC_BATCH_COMPLETE', out, flush=True)


if __name__ == '__main__':
    main()

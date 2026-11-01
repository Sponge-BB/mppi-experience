#!/usr/bin/env python3
"""Two diagnostic SIL trials, not a speed benchmark; no hardware control."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from run_coupled_objective import run_trial, check_build, check_space, sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build=a.build.resolve();out=a.out.resolve()
    m,_=check_build(build)
    if not m.get('domain_recovery_experiment'):
        raise ValueError('Requires private domain-recovery build')
    check_space(out.parent,2*1024**3)
    out.mkdir(parents=True,exist_ok=False)
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),
        build_manifest_sha256=sha(build/'build_manifest.json'),
        jobs=[dict(scene='static',group='coupled',seed=11,domain_recovery=mode) for mode in (False,True)],
        duration_s=48.,performance_comparison=False,
        purpose='Whether a finite inadmissible optimizer trial can backtrack without whole-solve STOP_FOR_ERROR',
        common_pair_safety_change=m['common_pair_safety_change'],
        unchanged=['body projection','0.10m hard margin','native collision/wheel checks','solve budget','MPPI policy','weights'])
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    done=[]
    try:
        for job in plan['jobs']:
            mode=job['domain_recovery'];name='static_coupled_seed11_recovery_'+('on' if mode else 'off')
            check_space(out,2*1024**3)
            valid=run_trial(out/name,build,'static','coupled',11,48.,1.,1.,False,
                plan['build_manifest_sha256'],diagnostic_capture=True,domain_recovery=mode,
                reserve_bytes=2*1024**3)
            done.append(dict(trial=name,valid=valid))
            (out/'progress.json').write_text(json.dumps(done,indent=2)+'\n')
            if not valid:
                raise RuntimeError('Interface/audit failure; retained without rerun')
    except Exception as exc:
        (out/'stopped.json').write_text(json.dumps(dict(completed=done,reason=str(exc)),indent=2)+'\n')
        raise
    (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=done),indent=2)+'\n')
    print('DOMAIN_DIAGNOSTIC_COMPLETE',out,flush=True)


if __name__=='__main__':
    main()

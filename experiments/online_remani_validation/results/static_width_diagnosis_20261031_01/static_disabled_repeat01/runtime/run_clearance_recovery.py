#!/usr/bin/env python3
"""Static diagnostic pair with exactly one bounded constraint restoration."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
from run_coupled_objective import run_trial,check_build,check_space,sha


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build=a.build.resolve();out=a.out.resolve()
    m,_=check_build(build)
    if not m.get('clearance_recovery_experiment'):raise ValueError('Requires clearance build')
    check_space(out.parent,2*1024**3);out.mkdir(parents=True,exist_ok=False)
    plan=dict(created=datetime.now(timezone.utc).isoformat(),build=str(build),
      build_manifest_sha256=sha(build/'build_manifest.json'),scene='static',seed=11,group='coupled',
      duration_s=48.,domain_recovery=True,modes=[False,True],rho=1e5,restart_limit=1,
      validation_reserve_s=.04,performance_comparison=False,
      constraint_weighting='fixed K8 omega/K collocation nodes and immutable one-update multipliers',
      common_budget_change=m['common_pair_budget_change'],
      unchanged=['full body','0.10m hard margin','native guards','original optimization deadline','MPPI','nominal objective'])
    (out/'plan.json').write_text(json.dumps(plan,indent=2)+'\n');done=[]
    try:
      for mode in plan['modes']:
        check_space(out,2*1024**3)
        name='static_coupled_seed11_restoration_'+('on' if mode else 'off')
        valid=run_trial(out/name,build,'static','coupled',11,48.,1.,1.,False,
          plan['build_manifest_sha256'],diagnostic_capture=True,domain_recovery=True,
          clearance_recovery=mode,reserve_bytes=2*1024**3)
        done.append(dict(trial=name,valid=valid));(out/'progress.json').write_text(json.dumps(done,indent=2)+'\n')
        if not valid:raise RuntimeError('Interface/audit failure, retained without rerun')
    except Exception as exc:
      (out/'stopped.json').write_text(json.dumps(dict(completed=done,reason=str(exc)),indent=2)+'\n');raise
    (out/'complete.json').write_text(json.dumps(dict(complete=True,completed=done),indent=2)+'\n')
    print('CLEARANCE_DIAGNOSTIC_COMPLETE',out,flush=True)


if __name__=='__main__':main()

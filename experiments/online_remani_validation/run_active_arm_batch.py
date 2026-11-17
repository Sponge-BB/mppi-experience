#!/usr/bin/env python3
"""A bounded fresh comparison; all development attempts remain on disk."""
from datetime import datetime,timezone
import argparse
import hashlib
import json
from pathlib import Path

from run_active_arm import ROOT,HERE,run_trial,builder


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(name=''):
    ROOT.mkdir(exist_ok=True)
    jobs=[dict(tag='A_fixed_seed11',enabled=False,seed=11),
          dict(tag='B_active_seed11',enabled=True,seed=11),
          dict(tag='B_active_seed23',enabled=True,seed=23),
          dict(tag='A_fixed_seed23',enabled=False,seed=23)]
    if name:
        if not name.replace('_','').isalnum():raise ValueError('invalid comparison suffix')
        jobs=[dict(job,tag=name+'_'+job['tag']) for job in jobs]
    plan=dict(created=datetime.now(timezone.utc).isoformat(),jobs=jobs,duration_s=45.,
        active_sources_sha256={p.name:sha(p) for p in HERE.glob('active_arm*') if p.is_file()},
        runner_sha256=sha(HERE/'run_active_arm.py'),build_manifest_sha256=sha(builder.OUT/'build_manifest.json'),
        scope='two seeds, same numerical scene and native binary, asynchronous online inputs; mechanism demonstration, not statistical generalization',
        development_attempts_retained=True,shutdown_requested=True,complete=False)
    path=ROOT/('comparison_plan_'+name+'.json' if name else 'comparison_plan.json')
    if path.exists():raise FileExistsError(str(path))
    path.write_text(json.dumps(plan,indent=2)+'\n');completed=[]
    try:
        for job in jobs:
            if sha(HERE/'run_active_arm.py')!=plan['runner_sha256']:raise RuntimeError('runner changed during comparison')
            for source_name,digest in plan['active_sources_sha256'].items():
                if sha(HERE/source_name)!=digest:raise RuntimeError('controller source changed during comparison: '+source_name)
            out=run_trial(job['tag'],job['enabled'],job['seed'],45.,True,False)
            result=json.loads((out/'result.json').read_text())
            completed.append(dict(**job,reason=result['reason'],collision=result['collision_physical'],
                min_external_gap_m=result['min_external_gap_m']))
            (ROOT/('comparison_progress_'+name+'.json' if name else 'comparison_progress.json')).write_text(json.dumps(completed,indent=2)+'\n')
            if result['collision_physical'] or result['joint_limit_violation']:
                raise RuntimeError('physical or joint violation retained; investigate before another trial')
        plan.update(complete=True,completed=completed)
    except Exception as exc:
        plan.update(complete=False,completed=completed,stopped_reason=str(exc));raise
    finally:path.write_text(json.dumps(plan,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',default='');a=p.parse_args();main(a.name)

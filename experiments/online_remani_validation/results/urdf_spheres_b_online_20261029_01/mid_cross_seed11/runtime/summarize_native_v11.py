#!/usr/bin/env python3
"""Summarize explicitly named SIL trials, including failures; no best-run selection."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import numpy as np
from compare_native_v9 import load,metrics

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

def summary(case):
    trial=load(case);m=metrics(trial);diag=trial['diag']
    anchor=trial['result']['motion_anchor'] or trial['result']['goal_sent'] or 0
    diag=[d for d in diag if d['t']>=anchor]
    status=json.loads((case/'run_status.json').read_text())
    fault=any(re.search(r'received signal SIG(?:SEGV|ABRT|BUS|ILL|FPE)',p.read_text(errors='replace'))
        for p in (case/'roslogs').glob('*/remani_planner_node-*-stdout.log'))
    m.update(required_process_failed=status['required_process_failed'] or bool(fault),
        native_reference_versions=len(trial['refs']),
        conservative_projection_collision=trial['result']['collision_conservative_projection'],
        callback_p95_ms=float(np.percentile([d['callback_ms'] for d in diag],95)),
        accepted_alternative_cycles=sum(d['reason']=='ok' and d.get('accepted_candidate',-1)>0 for d in diag),
        native_invalid_seed_rejections=len(re.findall(r'\[SampleMani\]\[InvalidSeed\]',(case/'launch_stdout.log').read_text())))
    for name in ('reference_audit','execution_audit'):
        if (case/(name+'.json')).exists():m[name]=json.loads((case/(name+'.json')).read_text())
    return m

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',type=Path,action='append',required=True)
    p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args()
    manifest=json.loads((args.build/'build_manifest.json').read_text())
    changes=[s for s,h in manifest['production_sources_sha256'].items()
        if hashlib.sha256((ROOT/s).read_bytes()).hexdigest()!=h]
    report={'build':str(args.build),'production_files_checked':len(manifest['production_sources_sha256']),
        'production_changed':changes,'cases':{str(c):summary(c) for c in args.case},
        'scope':'ROS numerical SIL with actual online native planner, fixed arm, base MPPI, no CBF; not Gazebo/hardware/statistical superiority'}
    with args.out.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False)
    print(json.dumps(report,indent=2,ensure_ascii=False))

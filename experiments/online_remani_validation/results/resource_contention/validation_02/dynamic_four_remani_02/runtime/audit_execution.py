#!/usr/bin/env python3
"""Audit actual local command authorization; keep task success separate."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from compare_native_v9 import load,metrics


def audit(out):
    trial=load(out);diagnostics=trial['diag'];failures=[]
    records=[json.loads(s) for s in (out/'execution_rollouts.jsonl').read_text().splitlines()]
    authorized=0
    for record in records:
        command=np.asarray(record['command']);rows=np.asarray(record['rollout']);report=record['validation']
        moving=np.max(np.abs(command))>1e-9
        if record['reason']!='ok' and moving:failures.append('nonzero command after rejection')
        if moving and not len(rows):failures.append('nonzero command without reviewed rollout')
        if len(rows):
            authorized+=1
            if record['reason']!='ok' or report[0]!=1 or report[5]!=1:failures.append('missing rollout/brake approval')
            if not np.allclose(record['controls'][0],command,rtol=0,atol=1e-9):failures.append('issued command differs from checked first action')
            if rows.shape[0]!=report[3] or not np.all(np.isfinite(rows)):failures.append('invalid reviewed rollout')
            if not -.05<=1000+record['t']-record['cloud_stamp']<=.35:failures.append('obsolete collision frame authorized')
            if not -.05<=1000+record['t']-record['odom_stamp']<=.2:failures.append('obsolete measured state authorized')
    pending=[d for d in diagnostics if d.get('pending_reference_updates',0)>0]
    motions=[d for d in pending if abs(d['applied_v'])>.02]
    result={'execution_contract_pass':not failures,'failures':sorted(set(failures)),
        'authorized_cycles':authorized,'checked_cycle_count':len(records),
        'delayed_reference_cycles':len(pending),'moving_while_reference_delayed_cycles':len(motions),
        'delayed_window_s':[pending[0]['t'],pending[-1]['t']] if pending else None,
        'validation_codes':dict(Counter(d.get('validation_code') for d in diagnostics)),
        'metrics':metrics(trial),'scope':'recorded discrete-model execution contract, NOT continuous-time or hardware safety proof'}
    (out/'execution_audit.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(result,indent=2,ensure_ascii=False));return not failures


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('out',type=Path);a=p.parse_args()
    raise SystemExit(0 if audit(a.out) else 1)

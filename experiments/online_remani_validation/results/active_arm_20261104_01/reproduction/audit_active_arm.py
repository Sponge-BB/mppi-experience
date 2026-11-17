#!/usr/bin/env python3
"""Audit completed active-arm evidence independently from the online selector."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(root,plan_name='comparison_plan_v2.json'):
    root=Path(root);plan=json.loads((root/plan_name).read_text())
    assert plan['complete'],'comparison incomplete'
    records=[]
    for job in plan['jobs']:
        p=root/job['tag'];res=json.loads((p/'result.json').read_text())
        status=json.loads((p/'run_status.json').read_text());graph=json.loads((p/'graph_audit.json').read_text())
        cfg=json.loads((p/'active_arm_config.json').read_text())
        a=np.genfromtxt(p/'state.csv',delimiter=',',names=True)
        q=np.column_stack([a['q'+str(i)] for i in range(1,7)])
        dq=np.column_stack([a['dq'+str(i)] for i in range(1,7)])
        al=cfg['arm_limits'];lo=np.asarray(al['lower']);hi=np.asarray(al['upper'])
        vm=np.asarray(al['velocity']);am=np.asarray(al['acceleration'])
        assert status['result_written'] and not status['required_process_failed']
        assert not graph['unknown_nodes'] and graph['sole_whole_body_publisher']==['/active_arm_mppi']
        assert res['reason']=='goal' and not res['collision_physical'] and not res['joint_limit_violation']
        assert res['maximum_integration_step_s']<=.005+1e-12
        assert np.all(q>=lo-1e-8) and np.all(q<=hi+1e-8) and np.all(np.abs(dq)<=vm+1e-8)
        acc=np.diff(dq,axis=0)/np.diff(a['t'])[:,None]
        assert np.all(np.abs(acc)<=am+1e-6)
        cycles=[json.loads(s) for s in (p/'active_arm_cycles.jsonl').read_text().splitlines()]
        assert all(c['reason']=='ok' or not np.any(c['command']) for c in cycles)
        assert all(not np.any(c['command']) or c.get('lease_brake_gap',-1)>=0 for c in cycles)
        if not job['enabled']:assert np.max(np.abs(q-q[0]))<1e-12
        timeline=[json.loads(s) for s in (p/'dual_timeline.jsonl').read_text().splitlines()]
        used=[e for e in timeline if e.get('event')=='collab_prefix' and e.get('used')]
        costs=[e for e in timeline if e.get('event')=='coupling_cost' and e.get('active')]
        if job['enabled']:assert used and costs,job['tag']
        for source,digest in plan['active_sources_sha256'].items():
            f=p/'runtime'/source
            if f.exists():assert sha(f)==digest,(job['tag'],source)
        ms=[c['callback_ms'] for c in cycles if c['reason']=='ok']
        record=dict(trial=job['tag'],reason=res['reason'],enabled=job['enabled'],
            path_m=float(np.hypot(np.diff(a['x']),np.diff(a['y'])).sum()),
            max_lateral_m=float(np.max(np.abs(a['y']))),motion_to_goal_s=res['elapsed']-res['motion_anchor'],
            external_gap_m=res['min_external_gap_m'],
            max_joint_displacement_deg=np.rad2deg(np.max(np.abs(q-q[0]),axis=0)).tolist(),
            max_joint_acceleration=np.max(np.abs(acc),axis=0).tolist(),
            successful_callback_p95_ms=float(np.percentile(ms,95)),successful_callback_max_ms=float(max(ms)),
            all_callback_max_ms=float(max(c['callback_ms'] for c in cycles)),
            callback_above_200ms=sum(c['callback_ms']>200 for c in cycles),
            reasons=dict(Counter(c['reason'] for c in cycles)),whole_body_prefix_used=len(used),
            active_handoff_records=len(costs),audit_substeps=res['audit_substeps'])
        records.append(record)
    result=dict(pass_all=True,plan=plan_name,plan_sha256=sha(root/plan_name),trials=records,
        limitations='two seeds; native-DH sphere numerical SIL, <=5ms physical audit; no continuous safety or statistical guarantee; whole-callback 200ms is not a proven bound')
    path=root/'verification/closed_loop_audit_v2.json';path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(result,indent=2)+'\n')
    # Recover human progress metadata from authoritative completed job records.
    # The earlier batch had a loop-variable filename bug; its file is retained.
    (root/'comparison_progress_v2.json').write_text(json.dumps(plan['completed'],indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    r=audit(a.out)
    print(json.dumps(r,ensure_ascii=False,indent=2))

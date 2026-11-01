#!/usr/bin/env python3
"""Summarize this specific three-case diagnosis, not a generic benchmark.

The narrative describes the observed 20261022_01 batch and therefore refuses
different case counts or numerical outcomes instead of reusing stale prose.
The lower-level recheck_delivery_diag.py is the generic analysis tool.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from recheck_delivery_diag import load_json, resolve_snapshot, snapshot_arrays, evaluate_pose


def summarize(batch):
    batch = Path(batch).resolve()
    summaries = []
    errors = []
    total = agreement = 0
    for report in sorted((batch/'reports').glob('*/recheck.json')):
        record = load_json(report)
        rows = list(csv.DictReader((report.parent/'samples.csv').open()))
        root = Path(record['source'])
        for audit in record['audits']:
            total += 1
            agreement += bool(audit['replay_gate_status_agrees'])
            path = Path(audit['path'])
            candidate = load_json(path)
            maps = {name: snapshot_arrays(resolve_snapshot(root, candidate[name])) for name in ('solve_frame','accept_frame')}
            blocks = []
            for s in audit['summaries']:
                selected = [r for r in rows if r['candidate']==path.name and r['frame']==s['frame'] and r['sampling']==s['sampling'] and r['static_gap']]
                minimum = min(selected, key=lambda r: float(r['static_gap'])) if selected else None
                checked = None
                if minimum:
                    m = maps[s['frame']]
                    checked = evaluate_pose(m, float(minimum['x']), float(minimum['y']), float(minimum['yaw']), 0.)
                    checked.update(t=float(minimum['t']), x=float(minimum['x']), y=float(minimum['y']),
                                   yaw=float(minimum['yaw']), speed=float(minimum['speed']), part=int(minimum['part']),
                                   piece=int(minimum['piece']), sample=int(minimum['sample']))
                blocks.append(dict(frame=s['frame'], sampling=s['sampling'], counts=s['counts'],
                                   first_failure=s['first_failure'], full_sampled_path_minimum_static=checked))
            solve, accept = maps['solve_frame'], maps['accept_frame']
            geometry_equal = bool(solve is not None and accept is not None and
                                  np.array_equal(solve['field'],accept['field']) and
                                  np.array_equal(solve['body'],accept['body']) and
                                  all(solve[k]==accept[k] for k in ('nx','ny','gx','gy','res','margin')))
            summaries.append(dict(trial=report.parent.name, request_id=audit['request_id'], scope_id=audit['scope_id'],
                                  candidate_path=str(path), accepted=audit['recorded_accepted'],
                                  gate_status_agrees=audit['replay_gate_status_agrees'],
                                  first_failure_comparison=audit['first_failure_comparison'],
                                  first_failure_online=audit['recorded_failure'],
                                  solve_accept_static_geometry_exactly_equal=geometry_equal,
                                  checks=blocks))
        errors.extend(dict(trial=report.parent.name, **e) for e in record['geometry_error_audits'])
    if not summaries:
        raise ValueError('No diagnostic rechecks found')
    expected_trials = {'static_coupled_seed11','static_engineering_seed11','dynamic_four_engineering_seed11'}
    rejected = [a for a in summaries if not a['accepted']]
    if ({a['trial'] for a in summaries} != expected_trials or total != 6 or agreement != 6 or
            len(rejected) != 1 or rejected[0]['request_id'] != 5 or
            len(errors) != 1 or errors[0]['request_id'] != 2):
        raise ValueError('This narrative is specific to the observed 20261022_01 diagnosis; use the generic per-trial rechecker for a different outcome')
    minimum = next(c['full_sampled_path_minimum_static']['gap'] for c in rejected[0]['checks']
                   if c['frame']=='accept_frame' and c['sampling']=='dense')
    if abs(minimum - (-.016310559354607318)) > 1e-10:
        raise ValueError('Observed clearance changed; refusing to reuse fixed diagnostic narrative')
    output = dict(schema='delivery_recheck_batch_v1', batch=str(batch), diagnostic_only=True,
                  not_performance_comparison=True, no_reoptimization_or_retiming=True,
                  candidates=total, gate_status_agreements=agreement,
                  audits=summaries, geometry_errors=errors,
                  limits=['One repetition per selected diagnostic case; do not infer relative solver speed or success rate.',
                          'Collision means overlap in recorded fixed-arm disk-union projection, not physical ground truth.',
                          'Full sampled path does not prove continuous-time collision freedom.',
                          'K8 keeps candidate clock; it is not the online dynamic objective replay.',
                          'The dynamic case delivered only its initial candidate; it does not validate dynamic handover.'])
    target = batch/'同候选复查汇总.json'
    if target.exists() or (batch/'同候选复查结论.txt').exists():
        raise FileExistsError('Refusing to overwrite existing summary')
    target.write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = [
        '同候选复查：仅诊断，不是性能对比；没有重新优化、改变轨迹时钟或放宽验收。',
        '3次隔离数值SIL，6条记录候选；接受/拒绝状态6/6与独立重查一致。',
        '',
        '1. 静态拒收不是“已经碰撞”，而是10 cm安全余量不足。',
        'static_coupled request5：首个不合格点物理净距9.9779 cm，比门槛少0.0221 cm；',
        '但不能只看首个点：全候选0.02 s密采样最低物理净距8.3689 cm，实际少1.6311 cm。',
        'K8也检出余量不足（最低缺1.4515 cm），不是只有密采样才发现的问题。',
        '所有已复查静态样本均未出现物理净距<0；这不等于连续时间或实车无碰撞证明。',
        '',
        '2. 这条拒收候选不是被交付时更新的地图“突然判坏”。',
        '求解与验收快照的静态场、投影外形、margin逐项相同；同候选两次结果相同。',
        '在线/独立首失败的query_time、yaw、physical、gap绝对差均为0。',
        '新增投影代价是软惩罚，仍可能交出违反硬余量的曲线；不能以优化收敛代替硬安全通过。',
        '',
        '3. 新增几何异常已锁定为优化试探越出距离场，不是CPU慢或NaN。',
        'static_coupled request2/scope2：block0 piece2 sample3，机器人中心(0.423699,4.909638)m，',
        '投影disk1中心y=5.031306m，越过双线性场有效上界4.95m；因此静态查询invalid。',
        '系数独立重建位置差0、速度差1.11e-16；输入是有限数。',
        '该次forecast=10.6879s虽超3s，但直接异常先发生在静态场，不应归因CV预测。',
        '当前处理把这次不合法优化试探升级成整次求解中止，是下一步应修的鲁棒性问题。',
        '',
        '4. 其余两条静态重规划候选通过，但一个仅比门槛多0.2467 mm，属于贴边解，不能称鲁棒。',
        '四动态场景只有初始候选进入投影验收；不能由本报告证明动态新轨迹交付已经解决。',
        '',
        '下一步：保持10 cm门槛与物理外形不变，先解决越界试探中止；',
        '再使慢层投影软代价与最终硬验收获得足够一致的可行余量，验证同类候选是否可正确交付。',
        '动态目标入口的拒绝来源由另一个目标probe报告分析，本文件不越权推断。',
        '',
        '图：reports/static_coupled_seed11/candidate_01.png',
        '完整逐候选数据：同候选复查汇总.json；各组reports/下保留逐样本CSV与原始快照SHA256。',
    ]
    (batch/'同候选复查结论.txt').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    return output


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    args=parser.parse_args()
    summary=summarize(args.batch)
    print(json.dumps(dict(candidates=summary['candidates'], gate_status_agreements=summary['gate_status_agreements'], batch=str(args.batch)), ensure_ascii=False))

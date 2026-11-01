#!/usr/bin/env python3
"""Explain one covered-window rejection using the actual saved candidate."""
import json
from pathlib import Path

import numpy as np

from replay_delivery_window import dynamic_query, sha256
import replay_temporal_collision as native

HERE = Path(__file__).resolve().parent
ROOT = HERE / 'results/delivery_midcross_20261101_01'


def main():
    trial = ROOT / 'midcross_B'
    names = ('temporal_collision.jsonl', 'dual_timeline.jsonl')
    before = {name: sha256(trial / name) for name in names}
    captures = native.rows(trial / names[0])
    timeline = native.rows(trial / names[1])

    def unique(rows):
        if len(rows) != 1:
            raise ValueError('The recorded q2 association must be unique')
        return rows[0]

    def capture(stage):
        return unique([row for row in captures if row.get('request_id') == 2
                       and row.get('stage') == stage])

    plan, final, acceptance = [capture(stage) for stage in ('planning', 'final_candidate', 'acceptance')]
    reject = unique([row for row in timeline if row.get('request_id') == 2
                     and row.get('event') == 'latest_prediction_reject_detail'])
    window = unique([row for row in timeline if row.get('request_id') == 2
                     and row.get('event') == 'slow_delivery_window'])
    candidate = native.Candidate(final['candidate'], plan)
    if candidate.geometry_hash() != native.Candidate(acceptance['candidate'], plan).geometry_hash():
        raise ValueError('Final and latest-validation candidate geometry differs')
    overlay = plan['overlay']
    frozen, latest = overlay['source_snapshot'], acceptance['latest_snapshot']
    local_t = reject['local_t']
    clocks = dict(backend=plan['requested_epoch'] + local_t, latest=reject['query_stamp'])
    sources = dict(frozen=(frozen, window['planning_max_prediction_s']),
                   latest=(latest, overlay['maximum_prediction_s']))
    checks = {source_name + '_' + clock_name:
              dynamic_query(candidate, local_t, query, source, overlay, limit)
              for source_name, (source, limit) in sources.items()
              for clock_name, query in clocks.items()}

    source_geometry = {}
    for name, (source, limit) in sources.items():
        points = np.asarray(source['points'], dtype=float).reshape((-1, 8))
        at_query = points[:, :3] + (clocks['latest'] - native.seconds(source)) * points[:, 3:6]
        source_geometry[name] = dict(sequence=source['sequence'], points=len(points),
            stamp=native.seconds(source), query_limit_s=limit,
            velocities_min=points[:, 3:6].min(axis=0).tolist(),
            velocities_max=points[:, 3:6].max(axis=0).tolist(),
            track_ids=sorted(set(points[:, 6].tolist())),
            predicted_minimum_x_m=float(at_query[:, 0].min()))

    if before != {name: sha256(trial / name) for name in names}:
        raise RuntimeError('Original logs changed during audit')
    result = dict(schema='delivery_midcross_q2_reject_v1', input_sha256=before,
        input_unchanged=True, script_sha256=sha256(Path(__file__)),
        helper_sha256={name: sha256(HERE / name) for name in
            ('replay_delivery_window.py', 'replay_temporal_collision.py')},
        request_id=2, input_hash=plan['input_hash'], sphere_count=len(candidate.body),
        native_final_pass=final['native_success'],
        candidate_geometry_sha256=candidate.geometry_hash(),
        rejected_local_t_s=local_t, query_epoch_difference_s=clocks['latest'] - clocks['backend'],
        clocks=clocks, source_geometry=source_geometry, checks=checks,
        scope='Exact saved fixed-arm candidate and sphere/point model at the single recorded rejected point. Not a full optimizer, ESDF, execution or physical-contact replay. A source/clock swap is a labeled diagnostic comparison.')
    lines = ['横穿 B 第一次最新拒绝核对', '',
        '只比较同一条候选、同一个被拒轨迹点；没有重跑优化器，没有改日志。',
        '冻结点云和最新点云分别为 %d / %d 点，速度均约 (0,0.18) 米/秒，目标 ID 都为 1。' % (
            source_geometry['frozen']['points'], source_geometry['latest']['points']),
        '两种时间关联相差 %.3f 秒。' % result['query_epoch_difference_s'], '',
        '同一轨迹点的最小球点模型间隙：']
    for key, label in (
        ('frozen_backend', '冻结外形＋慢层原时钟'),
        ('frozen_latest', '冻结外形＋最新验收时钟'),
        ('latest_backend', '最新外形＋慢层原时钟'),
        ('latest_latest', '最新外形＋最新验收时钟')):
        item = checks[key]
        lines.append('%s：%+.3f 厘米，时间查询有效 %s。' % (
            label, item['minimum_model_gap_m'] * 100., item['query_valid']))
    lines += ['',
        '结论：被拒点已经在 B 的 4.5 秒窗口内，不能再归因为预测窗不够长。',
        '主要变化是冻结外形与最新外形不同；最新外形朝机器人一侧多显露约 %.2f 厘米。' % (
            100. * (source_geometry['frozen']['predicted_minimum_x_m'] - source_geometry['latest']['predicted_minimum_x_m'])),
        '时间关联差进一步把最新外形下的小正间隙变成负间隙。不是本次 CV 速度估计突然跳变。',
        '负间隙是该预测模型下相交，不代表实车或数值机器人已发生接触。',
        '这项核查不支持继续盲目延长预测窗，也不能只凭一次边缘相交就放宽安全验收。']
    output = ROOT / 'report'
    (output / '最新拒绝核对.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    (output / '最新拒绝核对.txt').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()

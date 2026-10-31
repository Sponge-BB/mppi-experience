#!/usr/bin/env python3
"""Summarize all predeclared native collision OFF/ON trials, failures included."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from report_coupled_objective import lines, read


def percentile(values, p):
    return float(np.percentile(values, p)) if len(values) else None


def summary(batch, out):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if not (batch/'complete.json').exists():
        raise ValueError('All declared runs must finish before summary')
    plan = read(batch/'plan.json')
    rows = []
    for scene in plan['scenes']:
        for group in plan['groups']:
            trial = batch/('%s_%s_seed%d' % (scene, group, plan['seed']))
            result, analysis = read(trial/'result.json'), read(trial/'analysis/summary.json')
            audit, status = read(trial/'trial_audit.json'), read(trial/'run_status.json')
            if not audit['valid_experiment'] or not status['production_hashes_unchanged']:
                raise ValueError('Unqualified trial: '+str(trial))
            state = np.genfromtxt(trial/'state.csv', delimiter=',', names=True)
            diag = [d for d in read(trial/'mppi_diagnostics.json', []) if d['t'] >= result['goal_sent']]
            kernel = [d['kernel_ms'] for d in diag if d.get('code') == 0 and d.get('kernel_ms', 0) > 0]
            callback = [d['callback_ms'] for d in diag if d.get('code') == 0 and d.get('callback_ms', 0) > 0]
            solves = [s for s in lines(trial/'failure_trace.jsonl') if s.get('kind') == 'solve' and not s.get('diagnostic_only')]
            obstacles = np.genfromtxt(trial/'obstacles.csv', delimiter=',', names=True)
            path_length = float(np.hypot(np.diff(state['x']), np.diff(state['y'])).sum())
            row = dict(scene=scene, group=group, termination=result['reason'],
                reached_goal=result['reason'] == 'goal',
                started_moving=result['motion_anchor'] is not None,
                actor_scene_triggered=bool(obstacles.size),
                goal_to_end_s=result['elapsed']-result['goal_sent'],
                path_length_m=path_length,
                goal_error_m=float(np.hypot(result['final_state'][0]-3., result['final_state'][1])),
                trajectory_bundles=result['trajectory_bundles'],
                native3d_physical_sample_collision=result['native3d_truth_monitor']['sample_collision'],
                native3d_min_sample_gap_m=result['native3d_truth_monitor']['min_physical_sample_gap'],
                kernel_samples=len(kernel), kernel_p50_ms=percentile(kernel, 50), kernel_p95_ms=percentile(kernel, 95),
                callback_p95_ms=percentile(callback, 95),
                deadline_miss_count=sum(d['reason'] == 'deadline_miss' for d in diag),
                reasons=dict(Counter(d['reason'] for d in diag)),
                native_optimizer_outcomes=dict(Counter(s.get('reason') for s in solves)),
                native_optimizer_samples=len(solves),
                native_optimizer_p50_ms=percentile([s['wall_ms'] for s in solves], 50),
                native_optimizer_p95_ms=percentile([s['wall_ms'] for s in solves], 95),
                native_refresh_latency=analysis['latency'], stop_analysis=analysis['stop_analysis'],
                trial=str(trial), interface_audit_valid=True, production_unchanged=True)
            rows.append(row)
    out.mkdir(parents=True, exist_ok=False)
    result = dict(scope='fixed-arm differential-drive isolated ROS numerical SIL; NOT Gazebo/hardware',
        sample_size='one seed per scene/group; no statistical superiority claim',
        comparison='same instrumented child binary OFF/ON; NOT pure REMANI-only vs MPPI',
        no_motion_note='No motion means the motion-triggered actors did not appear; this is a realtime readiness failure, not a successful avoidance trial.',
        diagnostics_note='Inherited state/trajectory diagnostic capture and common raw snapshot recording remain enabled. Timing is instrumented, not an uninstrumented benchmark.',
        geometry_note='Native 3-D sampled robot geometry/hard thresholds and spatial cubic soft cost adapted; native dynamic swept overlay vs fast CV-time predictions are not a proof of full spatiotemporal feasible-set equivalence.',
        source_plan_sha256=hashlib.sha256((batch/'plan.json').read_bytes()).hexdigest(),
        reporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), trials=rows)
    (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    keys = ['scene', 'group', 'termination', 'started_moving', 'actor_scene_triggered',
        'goal_to_end_s', 'path_length_m', 'goal_error_m', 'trajectory_bundles',
        'native3d_physical_sample_collision', 'kernel_p50_ms', 'kernel_p95_ms',
        'deadline_miss_count', 'native_optimizer_samples', 'native_optimizer_p50_ms']
    with (out/'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    text = ['原生三维碰撞适配实验总结', '',
        '本轮只验证固定机械臂、差速底盘的ROS数值闭环；不是Gazebo或实车。',
        '比较同一新二进制的OFF旧附加二维门与ON原生三维模型，不是“纯REMANI vs MPPI”。',
        '只跑预先声明的seed11，所有失败保留，不能据此声称统计意义上的优越性。', '',
        '改动：撤销后来给REMANI增加的二维整机投影资格门/空间代价；',
        '让MPPI候选评分、硬检查、发布前与制动复核使用原生三维球采样、ESDF、对应阈值与三次软代价。',
        '保留原生ESDF膨胀、地面/自碰撞策略、轮约束、优化预算、50ms发布期限和指令租约。',
        '新增同一原生快照接口核验、丢帧/失效闭锁、地图窗口缓存回归、独立三维球/圆柱/墙碰撞监测、原始快照留证。', '',
        '场景 / 版本 / 结束 / 障碍场景实际触发 / MPPI中位耗时ms / 超期次数：']
    for row in rows:
        text.append('%s / %s / %s / %s / %.2f / %d' % (row['scene'], row['group'], row['termination'],
            '是' if row['actor_scene_triggered'] else '否', row['kernel_p50_ms'] or 0., row['deadline_miss_count']))
    text += ['', '注意：没有起步就没有触发按起步计时的障碍物，不能把“没碰撞”当作避障成功。',
        '碰撞几何的一致性不等于代价/时标/优化目标全部相同，也不保证高频局部搜索来得及完成。',
        '生产src/devel和旧父构建未修改；本轮不接实车。Piper继承的自碰撞覆盖限制仍存在。',
        'GIF与静态图由真实记录绘制，灰细线只是真实候选子集；失败不删。']
    (out/'实验总结.txt').write_text('\n'.join(text)+'\n')
    return rows


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    for row in summary(args.batch, args.out):
        print(row['scene'], row['group'], row['termination'], row['kernel_p50_ms'], row['deadline_miss_count'])

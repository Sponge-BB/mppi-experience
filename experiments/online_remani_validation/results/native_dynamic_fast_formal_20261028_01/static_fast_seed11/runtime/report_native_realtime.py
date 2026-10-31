#!/usr/bin/env python3
"""Read-only startup, deadline, scene-exposure and sampled-3D truth report."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from render_coupled_objective import SourceReader
from render_native_collision_3d import Trial as NativeTrial, truth_report
from dynamic_scenarios import CASES


class Trial(NativeTrial):
    def __init__(self, source, group):
        super().__init__(source, 'native3d')
        recorded = self.manifest.get('native_realtime', {}).get('demand')
        if recorded != (group == 'demand'):
            raise ValueError('native realtime panel switch differs from recorded manifest')
        self.group = group
        self.title = '原生3D：原全采样分支' if group == 'original' else '原生3D：按需已有候选'


def percentiles(rows, key):
    values = [row[key] for row in rows if row.get(key) is not None]
    return dict(zip(('min', 'p50', 'p95', 'max'), np.percentile(values, [0, 50, 95, 100]).tolist())) if values else None


def native_prediction_stats(reader, records):
    """Post-run raw-prefix audit distinguishes points from CV motion evidence."""
    output = dict(prediction_points_received=bool(records), prediction_snapshot_count=len(records),
        nonzero_velocity_snapshot_count=0, nonzero_velocity_point_records=0,
        first_nonzero_velocity_t=None, maximum_observed_point_speed_mps=0.)
    if not records:
        return output
    path = reader.locate('native3d_snapshots/frames.bin', required=True)
    with path.open('rb') as stream:
        for record in records:
            stream.seek(record['offset'])
            raw = stream.read(record['length'])
            if hashlib.sha256(raw).hexdigest() != record.get('prefix_sha256'):
                raise ValueError('recorded native prediction prefix hash mismatch')
            values = np.frombuffer(raw, dtype='<f8')
            nb, nd = int(values[12]), int(values[13])
            if len(values) != 32+7*(nb+nd):
                raise ValueError('incomplete recorded native prediction prefix')
            dynamic = values[32+7*nb:].reshape((-1, 7))
            velocities = dynamic[:, 3:6]
            if not np.isfinite(velocities).all():
                raise ValueError('nonfinite recorded native prediction velocity')
            speeds = np.linalg.norm(velocities, axis=1)
            moving = int(np.count_nonzero(speeds > 1e-9))
            output['maximum_observed_point_speed_mps'] = max(output['maximum_observed_point_speed_mps'], float(np.max(speeds, initial=0.)))
            if moving:
                output['nonzero_velocity_snapshot_count'] += 1
                output['nonzero_velocity_point_records'] += moving
                t = float(values[3]-1000)
                first = output['first_nonzero_velocity_t']
                output['first_nonzero_velocity_t'] = t if first is None else min(first, t)
    return output


def summarize_trial(source, group):
    trial = Trial(source, group)
    report = truth_report(trial)
    diagnostics = trial.diagnostics
    active = [row for row in diagnostics if row.get('reference_points', 0) > 0]
    solved = [row for row in active if row.get('code') == 0]
    approved = [row for row in active if row.get('reason') == 'ok' and row.get('authorized')]
    commands = [row for row in approved if abs(row.get('applied_v', 0))+abs(row.get('applied_w', 0)) > 1e-9]
    evidence = trial.result.get('native_realtime_evidence', {})
    snapshot_index = trial.reader.jsonl('native3d_snapshots/index.jsonl')
    valid_dynamic = [row for row in snapshot_index if row.get('kind') == 'snapshot'
        and len(row.get('header', [])) >= 32 and row['header'][13] > 0
        and row['header'][4] == 1 and row['header'][19] == 1
        and -.05 <= row['header'][3]-row['header'][18] <= .35]
    first_native_dynamic = min((row['header'][3]-1000 for row in valid_dynamic), default=None)
    prediction = native_prediction_stats(trial.reader, valid_dynamic)
    scene_entered = evidence.get('first_truth_actor_t') is not None
    dynamic_scene = trial.manifest['scene'] == 'crossing' or trial.manifest['scene'] in CASES
    real_dynamic_sample = bool(dynamic_scene and scene_entered and trial.result.get('motion_anchor') is not None
                               and prediction['nonzero_velocity_snapshot_count'] > 0)
    displacement = float(np.max(np.hypot(trial.state['x']-trial.state['x'][0], trial.state['y']-trial.state['y'][0])))
    report.update(active_reference_cycles=len(active), solver_return0_cycles=len(solved),
        nominal_nonzero_cycles=sum(abs(row.get('nominal_v', 0))+abs(row.get('nominal_w', 0)) > 1e-9 for row in solved),
        validated_rollout_cycles=sum(row.get('validation_code') == 0 for row in active),
        actual_nonzero_command_cycles=len(commands), reason_counts=dict(Counter(row.get('reason') for row in active)),
        kernel_ms=percentiles(solved, 'kernel_ms'), callback_ms=percentiles(solved, 'callback_ms'),
        deadline_miss_cycles=sum(row.get('reason') == 'deadline_miss' for row in active),
        first_nonzero_command_t=min((row['t'] for row in commands), default=None),
        motion_anchor=trial.result.get('motion_anchor'), scene_entered=scene_entered,
        max_measured_displacement_m=displacement,
        first_truth_actor_t=evidence.get('first_truth_actor_t'),
        first_observed_moving_t=evidence.get('first_observed_moving_t'),
        first_valid_native_dynamic_t=first_native_dynamic, native_dynamic_snapshot_count=len(valid_dynamic),
        native_prediction_points_received=prediction['prediction_points_received'],
        native_prediction_snapshot_count=prediction['prediction_snapshot_count'],
        native_nonzero_velocity_snapshot_count=prediction['nonzero_velocity_snapshot_count'],
        native_nonzero_velocity_point_records=prediction['nonzero_velocity_point_records'],
        first_native_nonzero_velocity_t=prediction['first_nonzero_velocity_t'],
        maximum_native_point_speed_mps=prediction['maximum_observed_point_speed_mps'],
        declared_dynamic_scene=dynamic_scene,
        prediction_point_presence_is_not_cv_motion=True,
        dynamic_exposure_valid=real_dynamic_sample,
        early_deadline_zero_deadlock=trial.result.get('reason') == 'deadline_zero_deadlock',
        original_reference_preserved=True, collisions_not_inferred_from_2d_projection=True,
        early_stop_does_not_prove_obstacle_handling=True)
    trial.verify_unchanged()
    report['input_sha256'] = trial.reader.hashes
    return report


def report_batch(batch, out):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('finish timed batch before reporting')
    if out == batch or batch in out.parents:
        raise ValueError('report must be outside measured source batch')
    plan = json.loads((batch/'plan.json').read_text())
    out.mkdir(parents=True, exist_ok=False)
    rows = [summarize_trial(batch/('%s_%s_seed%d' % (scene, group, plan['seed'])), group)
            for scene in plan['scenes'] for group in plan['groups']]
    result = dict(plan=plan, trials=rows, superiority_claim=False,
        safety_scope='independent analytic native body spheres vs finite truth cylinders/walls; sampled, not mesh/continuous/real-robot certificate')
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    text = ['# 原生3D按需计算实验', '', '两组都使用原生三维碰撞；差异仅空旷分支候选调用。单种子案例，不是统计成功率。', '',
        '| 场景/组 | 结束原因 | 实际运动触发 | 障碍实际出现 | 有效3D动态输入 | 3D采样接触 | 最小3D净距/m | 内核P50/ms | deadline周期 |',
        '|---|---|---|---|---|---|---:|---:|---:|']
    for row in rows:
        text.append('| %s/%s | %s | %s | %s | %s | %s | %.4f | %s | %d |' %
            (Path(row['trial']).name.rsplit('_'+row['group'], 1)[0], row['group'], row['termination'],
             row['motion_anchor'] is not None, row['scene_entered'], row['dynamic_exposure_valid'],
             row['native3d_physical_sample_collision'], row['native3d_independent_min_physical_sample_gap_m'],
             '%.2f' % row['kernel_ms']['p50'] if row['kernel_ms'] else 'n/a', row['deadline_miss_cycles']))
    text += ['', 'deadline_zero_deadlock是共同早停失败，不是到达或避障通过。没有实际运动/障碍暴露的场次，不能被当作动态安全样本。',
        '真实3D采样接触/净距与旧二维投影指标分别保存在JSON；不把投影相交等同三维接触。失败不删除、不替换种子。']
    (out/'README.md').write_text('\n'.join(text)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    report_batch(args.batch, args.out)


if __name__ == '__main__':
    main()


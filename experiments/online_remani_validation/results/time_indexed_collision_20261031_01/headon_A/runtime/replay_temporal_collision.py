#!/usr/bin/env python3
"""Audit newly captured native candidates against exactly recorded CV inputs.

Dynamic model checks only: no historical ESDF replay, no physical-contact or
controller-execution guarantee. No candidate, radius, scale or waiting interval
is altered to obtain a pass. Source snapshots and native overlay float32 input
hashes make same-input claims reviewable.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def fnv(data):
    value = 14695981039346656037
    for byte in data:
        value = ((value ^ byte) * 1099511628211) & 0xffffffffffffffff
    return str(value)


def seconds(snapshot):
    return snapshot['stamp_nsec'] / 1e9


def duration_nsec(value):
    whole = math.floor(value)
    return int(whole)*1000000000+int(math.floor((value-whole)*1e9+.5))


def uncertainty_parameters(overlay):
    u0, u1, u2 = overlay['uncertainty_at_0_1_2_s']
    acceleration = max(0., u2 - 2*u1 + u0)
    velocity = max(0., u1-u0-.5*acceleration)
    return max(0., u0), velocity, acceleration


def uncertainty(overlay, dt):
    p, v, a = uncertainty_parameters(overlay)
    dt = max(0., dt)
    return p + v*dt + .5*a*dt*dt


def time_samples(end, step, begin=0.):
    """Match the native increment/min(end, t+step) loops, including endpoint."""
    if end < begin-1e-9:
        return []
    result, t = [], begin
    while t <= end+1e-9:
        result.append(t)
        if t >= end-1e-9:
            break
        t = min(end, t+step)
    return result


def regenerate_overlay(overlay):
    snapshot = overlay['source_snapshot']
    if not snapshot or not snapshot['valid'] or not snapshot['points']:
        cloud = np.empty((0, 4), dtype='<f4')
        return cloud, dict(count=0, fnv1a64=fnv(cloud.tobytes()), hash_matches=(overlay['point_count'] == 0))
    points = np.asarray(snapshot['points'], dtype=float)
    age = max(0., (overlay['refresh_nsec']-snapshot['stamp_nsec'])/1e9)
    horizon = min(overlay['horizon_s'], max(0., overlay['maximum_prediction_s']-age))
    interval = min(horizon, overlay['step_s'])
    maximum_speed = float(np.max(np.linalg.norm(points[:, 3:6], axis=1)))
    padding = .5*maximum_speed*interval
    edge = uncertainty(overlay, age+horizon)+overlay['extra_inflation_m']+padding
    lower, upper = np.asarray(overlay['map_minimum'])-edge, np.asarray(overlay['map_maximum'])+edge
    slices = []
    for ahead in time_samples(horizon, overlay['step_s']):
        query_nsec = overlay['refresh_nsec']+duration_nsec(ahead)
        dt = (query_nsec-snapshot['stamp_nsec'])/1e9
        if dt < -1e-6 or dt > overlay['maximum_prediction_s']+1e-6:
            continue
        dt = min(overlay['maximum_prediction_s'], max(0., dt))
        positions = points[:, :3]+dt*points[:, 3:6]
        keep = np.all((positions >= lower) & (positions <= upper), axis=1)
        positions = positions[keep]
        radius = min(uncertainty(overlay, max(0., age+ahead))+overlay['extra_inflation_m']+padding,
                     overlay['maximum_inflation_m'])
        slices.append(np.column_stack((positions, np.full(len(positions), radius))))
    cloud = np.asarray(np.concatenate(slices) if slices else np.empty((0, 4)), dtype='<f4')
    value = fnv(cloud.tobytes())
    return cloud, dict(count=int(len(cloud)), fnv1a64=value,
        hash_matches=(len(cloud) == overlay['point_count'] and value == overlay['float32_xyzw_fnv1a64']),
        exact_source_recorded=overlay['source_exact'], horizon_s=horizon,
        maximum_speed_mps=maximum_speed, sampling_padding_m=padding,
        clipping_edge_allowance_m=edge, snapshot_sha256=digest(snapshot))


class Candidate:
    def __init__(self, candidate, plan):
        self.record = candidate
        self.duration = candidate['duration_s']
        self.body = np.asarray(plan['fixed_arm_body'], dtype=float)
        self.q = np.asarray(plan['fixed_arm_q'], dtype=float)
        self.pieces = [(part['gear'], piece['T'], np.asarray(piece['coeffs'], dtype=float))
                       for part in candidate['parts'] for piece in part['pieces']]
        if self.body.ndim != 2 or self.body.shape[1] != 5 or not self.pieces:
            raise ValueError('missing full native fixed-arm spheres / final candidate')
        # The captured base-frame arm spheres are valid only for this fixed arm.
        self.arm_max_deviation = 0.
        for _, duration, co in self.pieces:
            for t in (0., duration*.25, duration*.5, duration*.75, duration):
                value = np.array([np.polyval(row, t) for row in co])[2:]
                self.arm_max_deviation = max(self.arm_max_deviation, float(np.max(np.abs(value-self.q))))
        if self.arm_max_deviation > 1e-4:
            raise ValueError('arm moved; fixed-arm sphere replay is inapplicable')

    def state(self, local_t):
        t = float(local_t)
        for i, (gear, duration, co) in enumerate(self.pieces):
            if t <= duration or i == len(self.pieces)-1:
                p = np.array([np.polyval(row, t) for row in co])
                v = np.array([np.polyval(np.polyder(row), t) for row in co])
                return p, math.atan2(gear*v[1], gear*v[0])
            t -= duration
        raise ValueError('outside polynomial')

    def centers(self, t):
        p, yaw = self.state(t)
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        return self.body[:, :3] @ rotation.T + np.array([p[0], p[1], 0.])

    def geometry_hash(self):
        return digest([dict(gear=gear, T=duration, coeffs=co.tolist()) for gear, duration, co in self.pieces])


def gap_record(candidate, t, query, positions, radii):
    centers = candidate.centers(t)
    if not len(positions):
        return dict(local_t_s=float(t), query_epoch=float(query), minimum_model_gap_m=None,
                    model_collision=False, nearest_body_index=None, nearest_point_index=None)
    gaps = np.linalg.norm(centers[:, None, :]-positions[None, :, :], axis=2)
    gaps -= candidate.body[:, 3, None]
    gaps -= np.asarray(radii)
    index = np.unravel_index(int(np.argmin(gaps)), gaps.shape)
    minimum = float(gaps[index])
    return dict(local_t_s=float(t), query_epoch=float(query), minimum_model_gap_m=minimum,
                model_collision=minimum < 0., nearest_body_index=int(index[0]), nearest_point_index=int(index[1]),
                body_type=int(candidate.body[index[0], 4]),
                body_center=centers[index[0]].tolist(), predicted_point=np.asarray(positions[index[1]]).tolist())


def summarize(samples, **metadata):
    valid = [s for s in samples if s.get('query_valid')]
    finite = [s for s in valid if s.get('minimum_model_gap_m') is not None]
    return dict(metadata, sample_count=len(samples), valid_sample_count=len(valid),
        invalid_sample_count=len(samples)-len(valid),
        minimum=min(finite, key=lambda s: s['minimum_model_gap_m']) if finite else None,
        first_collision=next((s for s in valid if s.get('model_collision')), None),
        first_invalid=next((s for s in samples if not s.get('query_valid')), None),
        first_failure=next((s for s in samples if not s.get('query_valid') or s.get('model_collision')), None),
        samples=samples)


def temporal_gap(candidate, t, query, snapshot, overlay, model):
    dt = query-seconds(snapshot)
    result = dict(local_t_s=float(t), query_epoch=float(query), query_valid=False)
    if not snapshot['valid'] or dt < -1e-6 or dt > overlay['maximum_prediction_s']+1e-6:
        return result
    points = np.asarray(snapshot['points'], dtype=float).reshape((-1, 8))
    dt = min(overlay['maximum_prediction_s'], max(0., dt))
    predicted = points[:, :3]+dt*points[:, 3:6]
    radius = uncertainty(overlay, dt)
    if model in ('same_guide_radius', 'backend_guide_radius'):
        epoch = overlay['refresh_nsec']/1e9
        age = max(0., epoch-seconds(snapshot))
        horizon = min(overlay['horizon_s'], max(0., overlay['maximum_prediction_s']-age))
        # This paired interpretation has exactly the guide's finite time domain.
        if query < epoch-1e-6 or query > epoch+horizon+1e-6:
            result['invalid_reason'] = 'outside_original_swept_guide_time_domain'
            return result
        vmax = float(np.max(np.linalg.norm(points[:, 3:6], axis=1))) if len(points) else 0.
        padding = .5*vmax*min(horizon, overlay['step_s'])
        edge = uncertainty(overlay, age+horizon)+overlay['extra_inflation_m']+padding
        lower = np.asarray(overlay['map_minimum'])-edge
        upper = np.asarray(overlay['map_maximum'])+edge
        if model == 'same_guide_radius':
            keep = np.all((predicted >= lower) & (predicted <= upper), axis=1)
            predicted = predicted[keep]
        radius = min(radius+overlay['extra_inflation_m']+padding, overlay['maximum_inflation_m'])
        if model == 'same_guide_radius':
            # Same PCL float32 coordinates/radii as the original guide slices.
            predicted = predicted.astype(np.float32).astype(float)
            radius = float(np.float32(radius))
    result.update(gap_record(candidate, t, query, predicted, radius))
    result.update(query_valid=True, uncertainty_or_guide_radius_m=float(radius))
    return result


def temporal_check(candidate, snapshot, overlay, now, epoch, scale, step, model, mapping_name):
    horizon = overlay['maximum_prediction_s']-max(0., now-seconds(snapshot))
    delay = max(0., epoch-now)
    if horizon <= delay+1e-3:
        return dict(mapping=mapping_name, model=model, execution_scale=scale,
                    insufficient_horizon=True, start_delay_s=delay, valid_wall_horizon_s=horizon,
                    samples=[], sample_count=0, valid_sample_count=0, invalid_sample_count=0,
                    minimum=None, first_collision=None, first_invalid=None)
    samples = []
    for hold in time_samples(min(delay, horizon), step):
        sample = temporal_gap(candidate, 0., now+hold, snapshot, overlay, model)
        sample['branch'] = 'hold'
        samples.append(sample)
    local_horizon = min(candidate.duration, (horizon-delay)*scale)
    for t in time_samples(local_horizon, step):
        sample = temporal_gap(candidate, t, now+delay+t/scale, snapshot, overlay, model)
        sample['branch'] = 'trajectory'
        samples.append(sample)
    return summarize(samples, mapping=mapping_name, model=model, execution_scale=scale,
                     insufficient_horizon=False, start_delay_s=delay,
                     valid_wall_horizon_s=horizon, trajectory_horizon_s=local_horizon,
                     now_epoch=now, candidate_epoch=epoch, local_step_s=step)


def swept_check(candidate, cloud, end):
    # Exact minimum over per-radius KD-trees; unlike nearest-center-only it is
    # also correct when guide inflation varies by prediction slice.
    times = time_samples(min(candidate.duration, end), .01)
    centers = np.asarray([candidate.centers(t) for t in times])
    flat = centers.reshape((-1, 3))
    best = np.full(len(flat), np.inf)
    point_indices = np.full(len(flat), -1, dtype=int)
    if len(cloud):
        for radius in np.unique(cloud[:, 3]):
            indices = np.flatnonzero(cloud[:, 3] == radius)
            distance, local = cKDTree(cloud[indices, :3].astype(float)).query(flat)
            gaps = distance-float(radius)
            better = gaps < best
            best[better] = gaps[better]
            point_indices[better] = indices[local[better]]
    gaps = best.reshape(centers.shape[:2])-candidate.body[:, 3][None, :]
    samples = []
    for i, t in enumerate(times):
        sphere = int(np.argmin(gaps[i]))
        minimum = float(gaps[i, sphere])
        samples.append(dict(local_t_s=t, query_valid=True,
            minimum_model_gap_m=minimum if math.isfinite(minimum) else None,
            model_collision=minimum < 0., nearest_body_index=sphere,
            nearest_point_index=int(point_indices[i*len(candidate.body)+sphere])))
    return summarize(samples, local_step_s=.01,
        start_sample_separate_from_native_gate=True,
        native_gate_first_local_t_s=.05,
        native_gate_first_collision=next((s for s in samples if s['local_t_s'] >= .05-1e-9 and s['model_collision']), None))


def backend_check(candidate, overlay, swept, epoch, model):
    """Exact requested clock, both scales and original swept fallback.

    Unlike the final publication validator, this backend has no hold interval
    and does not restart an already-past requested epoch at receipt time.
    Only dynamic components are replayed; static/self/wheel checks stay outside.
    """
    source = overlay['source_snapshot']
    samples = []
    for sweep in swept['samples']:
        t = sweep['local_t_s']
        branch_samples = []
        effective = []
        for scale in (1., .5):
            q = temporal_gap(candidate, t, epoch+t/scale, source, overlay, model)
            q['execution_scale'] = scale
            branch_samples.append(q)
            effective.append(q if q.get('query_valid') else sweep)
        finite = [s for s in effective if s.get('minimum_model_gap_m') is not None]
        selected = min(finite, key=lambda s: s['minimum_model_gap_m']) if finite else sweep
        samples.append(dict(local_t_s=t, query_valid=True,
            minimum_model_gap_m=selected.get('minimum_model_gap_m'),
            model_collision=any(s.get('model_collision') for s in effective),
            used_swept_fallback=any(not s.get('query_valid') for s in branch_samples),
            branches=branch_samples))
    return summarize(samples, model=model,
        mapping='strict_requested_epoch_plus_local_t_over_scale_no_hold',
        original_scales=[1., .5], requested_epoch=epoch,
        local_step_s=.01, native_gate_first_local_t_s=.05,
        fallback_sample_count=sum(s['used_swept_fallback'] for s in samples),
        native_gate_first_collision=next((s for s in samples if s['local_t_s'] >= .05-1e-9 and s['model_collision']), None),
        scope='dynamic model component, first five seconds maximum; native static/self/wheel checks not replayed')


def audit_trial(source):
    captured = rows(source/'temporal_collision.jsonl')
    plans = {r['plan_ordinal']: r for r in captured if r['stage'] == 'planning'}
    finals = [r for r in captured if r['stage'] == 'final_candidate']
    acceptances = {r['plan_ordinal']: r for r in captured if r['stage'] == 'acceptance'}
    timeline = rows(source/'dual_timeline.jsonl')
    result = []
    for final in finals:
        ordinal = final['plan_ordinal']
        plan = plans[ordinal]
        overlay = final['final_native_overlay']
        cloud, reconstruction = regenerate_overlay(overlay)
        if not overlay['source_exact']:
            result.append(dict(request_id=plan['request_id'], plan_ordinal=ordinal,
                skipped=True, reason='native final overlay lacks exact source provenance',
                native_success=final['native_success'], native_reason=final['native_reason']))
            continue
        candidate = Candidate(final['candidate'], plan)
        entry = dict(request_id=plan['request_id'], plan_ordinal=ordinal,
            mode=plan['mode'], input_hash=plan['input_hash'], native_success=final['native_success'],
            native_reason=final['native_reason'], requested_epoch=plan['requested_epoch'],
            final_candidate_geometry_sha256=candidate.geometry_hash(), arm_max_deviation_rad=candidate.arm_max_deviation,
            sphere_count=len(candidate.body), base_spheres=int(np.sum(candidate.body[:, 4] == 0)),
            arm_spheres=int(np.sum(candidate.body[:, 4] == 1)),
            source_reconstruction=reconstruction,
            entry_to_final_overlay_unchanged=(plan['overlay']['sequence'] == overlay['sequence'] and
                plan['overlay']['refresh_nsec'] == overlay['refresh_nsec'] and
                plan['overlay']['float32_xyzw_fnv1a64'] == overlay['float32_xyzw_fnv1a64']),
            swept_union=swept_check(candidate, cloud, min(candidate.duration, 5.)),
            requested_epoch_checks=[], requested_backend_checks=[])
        frozen = overlay['source_snapshot']
        for model in ('same_guide_radius', 'backend_guide_radius'):
            entry['requested_backend_checks'].append(backend_check(candidate, overlay,
                entry['swept_union'], plan['requested_epoch'], model))
        for model in ('same_guide_radius', 'native_cv_uncertainty'):
            for scale in (0.5, 1.):
                entry['requested_epoch_checks'].append(temporal_check(candidate, frozen, overlay,
                    plan['capture_sim'], plan['requested_epoch'], scale, .05, model,
                    'planning_validator_clock_max_now_requested_with_original_wait_and_scale'))
        acceptance = acceptances.get(ordinal)
        if acceptance:
            accepted_candidate = Candidate(acceptance['candidate'], plan)
            now = acceptance['validation_now_nsec']/1e9
            latest = acceptance['latest_snapshot']
            entry['acceptance'] = dict(now_epoch=now, candidate_epoch=acceptance['candidate']['start_epoch'],
                geometry_unchanged=(accepted_candidate.geometry_hash() == candidate.geometry_hash()),
                planning_snapshot_sequence=frozen['sequence'], latest_snapshot_sequence=latest['sequence'],
                planning_snapshot_sha256=digest(frozen), latest_snapshot_sha256=digest(latest), checks=[])
            scales = sorted(set((acceptance['resume_scale'], max(.1, .5*acceptance['resume_scale']))))
            for snapshot_name, snapshot in (('planning_snapshot', frozen), ('latest_snapshot', latest)):
                for mapping_name, clock_now, epoch in (
                    ('planning_requested_clock', plan['capture_sim'], plan['requested_epoch']),
                    ('original_publication_validator_mapping', now, acceptance['candidate']['start_epoch'])):
                    for scale in scales:
                        check = temporal_check(accepted_candidate, snapshot, overlay, clock_now, epoch,
                            scale, acceptance['check_step_s'], 'native_cv_uncertainty', mapping_name)
                        check['snapshot'] = snapshot_name
                        entry['acceptance']['checks'].append(check)
            entry['acceptance']['logged_rejections'] = [r for r in timeline
                if r.get('request_id') == plan['request_id'] and r['event'] == 'latest_prediction_reject_detail']
            originals = [c for c in entry['acceptance']['checks']
                         if c['snapshot'] == 'latest_snapshot' and c['mapping'] == 'original_publication_validator_mapping']
            failure = next((c.get('first_failure') for c in originals if c.get('first_failure')), None)
            logs = entry['acceptance']['logged_rejections']
            if logs and failure:
                logged = logs[0]
                entry['acceptance']['original_rejection_reproduced'] = (
                    abs(failure['local_t_s']-logged['local_t']) < 1e-8 and
                    abs(failure['query_epoch']-logged['query_stamp']) < 1e-6 and
                    failure['branch'] == logged['branch'].split('_')[0] and
                    bool(failure.get('query_valid')) == bool(logged['query_valid']))
                entry['acceptance']['original_reproduced_first_failure'] = failure
            else:
                entry['acceptance']['original_rejection_reproduced'] = None
        result.append(entry)
    return dict(schema='temporal_collision_replay_v1', trial=str(source),
        scope='Same recorded native final polynomials, exact overlay source CV snapshot and hard sphere geometry. Dynamic model replay only; no ESDF replay, physical-contact proof or execution guarantee. Same-guide checks retain cap/extra/sampling padding; native-CV checks are separately labeled.',
        plans_recorded=len(plans), final_candidates_recorded=len(finals), acceptances_recorded=len(acceptances),
        plans_without_final=[p for p in plans if p not in {r['plan_ordinal'] for r in finals}],
        source_sha256={name: hashlib.sha256((source/name).read_bytes()).hexdigest()
                       for name in ('temporal_collision.jsonl', 'dual_timeline.jsonl')},
        candidates=result)


def compact_check(check):
    minimum = check.get('minimum')
    return ('碰撞' if check.get('first_collision') else '未见模型碰撞') + (
        '，最小间隙%.1f cm' % (100*minimum['minimum_model_gap_m']) if minimum else '，没有有效点对') + (
        '，另有%d个时域外样本' % check.get('invalid_sample_count', 0))


def narrative(report):
    lines = ['同输入动态碰撞复算', '',
        '本报告只判断记录下来的预测模型与碰撞球是否相交，不等于实测物理碰撞，也不能直接授权机器人执行。',
        '扫掠并集把多个预测时刻放在一起；单时刻检查将机器人和障碍物放到对应时刻比较。',
        '同引导膨胀保留原来的额外膨胀、采样间隔补偿和上限；原生CV另列，不能把缩小半径带来的变化算成时间化收益。', '']
    for candidate in report['candidates']:
        if candidate.get('skipped'):
            lines.append('请求q%d：缺少可核对的精确扫掠源，跳过同输入复算。' % candidate['request_id'])
            continue
        reconstruction = candidate['source_reconstruction']
        lines.append('请求q%d：原生最终检查%s（%s）。' % (candidate['request_id'],
            '通过' if candidate['native_success'] else '未通过', candidate['native_reason']))
        lines.append('原扫掠云重建摘要%s，优化入口到最终检查的扫掠输入%s。' % (
            '一致' if reconstruction['hash_matches'] else '不一致：不能声称精确复现',
            '未变化' if candidate['entry_to_final_overlay_unchanged'] else '发生变化，复算采用最终实际源'))
        sweep = candidate['swept_union']
        lines.append('扫掠并集：%s。' % compact_check(sweep))
        for check in candidate['requested_backend_checks']:
            name = '与原引导完全相同的膨胀/裁点/float32' if check['model'] == 'same_guide_radius' else 'B后端使用的同膨胀double预测点'
            at_zero = check['samples'][0] if check['samples'] else None
            at_native_start = next((s for s in check['samples'] if s['local_t_s'] >= .05-1e-9), None)
            lines.append('严格按请求启用时刻、全速和半速同时检查（%s）：%s；时域外回退原扫掠%d个样本。' % (
                name, compact_check(check), check['fallback_sample_count']))
            if at_zero and at_native_start:
                def gap_text(sample):
                    gap = sample.get('minimum_model_gap_m')
                    return '无有效点对' if gap is None else '%.2f cm' % (100*gap)
                lines.append('起点模型间隙%s，原生硬检查起始t=0.05秒处间隙%s。' % (
                    gap_text(at_zero), gap_text(at_native_start)))
        for check in candidate['requested_epoch_checks']:
            name = '同引导膨胀' if check['model'] == 'same_guide_radius' else '原生CV膨胀'
            lines.append('按规划阶段验收映射max(规划时刻,请求时刻)、含等待段、比例%.1f、%s：%s。' % (
                check['execution_scale'], name, compact_check(check)))
        acceptance = candidate.get('acceptance')
        if acceptance:
            lines.append('发布前复算：规划快照seq%d，最新快照seq%d；候选曲线%s。' % (
                acceptance['planning_snapshot_sequence'], acceptance['latest_snapshot_sequence'],
                '相同' if acceptance['geometry_unchanged'] else '不同，不能直接归因'))
            if acceptance['original_rejection_reproduced'] is not None:
                lines.append('原发布首拒时刻及分支复现%s。' % (
                    '一致' if acceptance['original_rejection_reproduced'] else '不一致，需继续检查复算条件'))
            for check in acceptance['checks']:
                lines.append('  %s / %s / 比例%.1f：%s。' % (
                    '规划快照' if check['snapshot'] == 'planning_snapshot' else '最新快照',
                    '规划时钟与请求时刻' if check['mapping'] == 'planning_requested_clock' else '原发布检查映射',
                    check['execution_scale'], compact_check(check)))
        lines.append('')
    lines += ['说明：缺少最终候选的请求不补造结果。时域外样本表示无法比较，不能视为安全。',
              '发布检查保留原来的等待段和全速/半速两组假设；过去的轨迹时间戳不会自动推进候选相位。']
    return '\n'.join(lines)+'\n'


def plot(report, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    from render_dual_timescale import FONT
    font = FontProperties(fname=FONT)
    candidates = [c for c in report['candidates'] if not c.get('skipped')]
    chosen = next((c for c in candidates if c['native_reason'] == 'planning_collision'),
                  candidates[0] if candidates else None)
    if chosen is None:
        return
    figure, axis = plt.subplots(figsize=(8, 4.3))
    sweep = chosen['swept_union']['samples']
    axis.plot([s['local_t_s'] for s in sweep],
              [None if s['minimum_model_gap_m'] is None else 100*s['minimum_model_gap_m'] for s in sweep],
              color='#777777', label='原扫掠并集')
    end = .1
    for scale, color in ((1., '#3274a1'), (.5, '#e1812c')):
        check = next(c for c in chosen['requested_backend_checks'] if c['model'] == 'same_guide_radius')
        samples = [b for s in check['samples'] for b in s['branches']
                   if b['execution_scale'] == scale and b.get('query_valid')]
        if samples:
            end = max(end, max(s['local_t_s'] for s in samples))
            axis.plot([s['local_t_s'] for s in samples],
                      [None if s['minimum_model_gap_m'] is None else 100*s['minimum_model_gap_m'] for s in samples],
                      marker='o', markersize=3, color=color, label='同膨胀按时刻 / 速度比例%.1f' % scale)
    axis.axhline(0., color='#a22', linewidth=.9)
    axis.axvline(.05, color='#bbb', linestyle=':', linewidth=1.)
    axis.set_xlim(0., min(2., end))
    axis.set_xlabel('候选局部时间 / 秒', fontproperties=font)
    axis.set_ylabel('预测模型最小间隙 / cm', fontproperties=font)
    axis.set_title('请求 q%d：同一候选、同一快照、同一碰撞球与引导膨胀' % chosen['request_id'], fontproperties=font)
    axis.legend(prop=font, fontsize=9)
    axis.grid(alpha=.15)
    figure.text(.02, .015, '负值表示该模型下相交；曲线截止处为原引导预测时域边界，不能外推为安全。',
                fontproperties=font, fontsize=9)
    figure.tight_layout(rect=(0, .05, 1, 1))
    figure.savefig(path, dpi=150)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('trial', type=Path)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    output = args.out or args.trial/'temporal_replay'
    output.mkdir(parents=True, exist_ok=True)
    report = audit_trial(args.trial)
    (output/'同输入动态碰撞复算.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    (output/'同输入动态碰撞复算.txt').write_text(narrative(report))
    plot(report, output/'扫掠与按时刻检查.png')
    print(json.dumps(dict(output=str(output), candidates=len(report['candidates']),
        exact_cloud_reconstructions=sum(c['source_reconstruction']['hash_matches'] for c in report['candidates']
                                        if not c.get('skipped'))), ensure_ascii=False))


if __name__ == '__main__':
    main()

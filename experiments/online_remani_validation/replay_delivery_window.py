#!/usr/bin/env python3
"""Check a saved candidate's missing slow-layer CV time coverage.

This is a dynamic sphere/point geometry check, not a complete optimizer or map
replay. The recorded prediction limit is never edited. An explicit second limit
labels a counterfactual CV continuation beyond the recorded source's 3 s limit.
Run with PYTHONNOUSERSITE=1 on the experiment host to use its system NumPy/SciPy.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

import replay_temporal_collision as native


HERE = Path(__file__).resolve().parent
DEFAULT_TRIAL = HERE / 'results/deferred_quality_20261101_01/headon_B'
DEFAULT_OUT = HERE / 'results/delivery_window_20261101_01/replay'


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dynamic_query(candidate, local_t, query_epoch, snapshot, overlay, limit):
    """Use native CV positions/radii, with an explicitly supplied domain limit."""
    original_limit = float(overlay['maximum_prediction_s'])
    dt = float(query_epoch - native.seconds(snapshot))
    original_domain = bool(snapshot['valid'] and -1e-6 <= dt <= original_limit + 1e-6)
    result = dict(local_t_s=float(local_t), query_epoch=float(query_epoch),
                  seconds_after_source=dt, prediction_limit_s=float(limit),
                  recorded_prediction_limit_s=original_limit,
                  inside_recorded_prediction_domain=original_domain,
                  assumes_cv_continuation_beyond_recorded_limit=bool(dt > original_limit + 1e-6),
                  query_valid=False,
                  scope='dynamic geometry under the explicitly stated CV time limit')
    if not snapshot['valid'] or dt < -1e-6 or dt > limit + 1e-6:
        result.update(dynamic_dispatch='static_only_outside_dynamic_domain',
                      static_or_self_collision_replayed=False)
        return result
    clamped_dt = min(limit, max(0., dt))
    points = np.asarray(snapshot['points'], dtype=float).reshape((-1, 8))
    positions = points[:, :3] + clamped_dt * points[:, 3:6]
    radius = native.uncertainty(overlay, clamped_dt)
    result.update(native.gap_record(candidate, local_t, query_epoch, positions, radius))
    result.update(query_valid=True, dynamic_dispatch='time_indexed_native_cv',
                  uncertainty_radius_m=float(radius), point_count=len(points))
    return result


def scan_summary(samples):
    finite = [r for r in samples if r.get('minimum_model_gap_m') is not None]
    return dict(sample_count=len(samples),
                valid_sample_count=sum(r['query_valid'] for r in samples),
                collision_sample_count=sum(r.get('model_collision', False) for r in samples),
                first_collision=next((r for r in samples if r.get('model_collision')), None),
                minimum=min(finite, key=lambda r: r['minimum_model_gap_m']) if finite else None,
                samples=samples)


def audit(trial, request_id, extended_limit):
    input_names = ('temporal_collision.jsonl', 'dual_timeline.jsonl')
    before_hashes = {name: sha256(trial / name) for name in input_names}
    captures = native.rows(trial / input_names[0])
    timeline = native.rows(trial / input_names[1])
    def captured(stage):
        found = [r for r in captures if r.get('stage') == stage and r.get('request_id') == request_id]
        if len(found) != 1:
            raise ValueError('expected exactly one %s record for q%d' % (stage, request_id))
        return found[0]
    plan, final, acceptance = (captured(stage) for stage in ('planning', 'final_candidate', 'acceptance'))
    overlay = plan['overlay']
    source = overlay['source_snapshot']
    if not overlay['source_exact'] or not source['valid']:
        raise ValueError('the frozen source is not exact and valid')
    original_limit = float(overlay['maximum_prediction_s'])
    if extended_limit <= original_limit:
        raise ValueError('the explicit counterfactual limit must exceed the original limit')
    candidate = native.Candidate(final['candidate'], plan)
    accepted_candidate = native.Candidate(acceptance['candidate'], plan)
    if accepted_candidate.geometry_hash() != candidate.geometry_hash():
        raise ValueError('final and publication candidates differ; a same-candidate comparison is unavailable')
    reject = next(r for r in timeline if r.get('event') == 'latest_prediction_reject_detail'
                  and r.get('request_id') == request_id)
    local_t = float(reject['local_t'])
    backend_epoch = float(plan['requested_epoch']) + local_t
    original = dynamic_query(candidate, local_t, backend_epoch, source, overlay, original_limit)
    extended = dynamic_query(candidate, local_t, backend_epoch, source, overlay, extended_limit)
    latest = dynamic_query(candidate, local_t, reject['query_stamp'],
                           acceptance['latest_snapshot'], overlay, original_limit)
    frozen_at_publication = dynamic_query(candidate, local_t, reject['query_stamp'],
                                          source, overlay, extended_limit)
    # 0.01 s increments in source-relative time; nominal backend local time has
    # the same spacing. The exact old right boundary is excluded from the new
    # interval, so an edge-tolerance sample is not mislabeled as newly covered.
    source_epoch = native.seconds(source)
    original_samples, extended_samples = [], []
    for dt in native.time_samples(extended_limit, .01, original_limit + .01):
        query = source_epoch + dt
        t = query - float(plan['requested_epoch'])
        if t > candidate.duration:
            break
        original_samples.append(dynamic_query(candidate, t, query, source, overlay, original_limit))
        extended_samples.append(dynamic_query(candidate, t, query, source, overlay, extended_limit))
    _, reconstruction = native.regenerate_overlay(overlay)
    if not reconstruction['hash_matches']:
        raise ValueError('recorded overlay reconstruction hash does not match')
    after_hashes = {name: sha256(trial / name) for name in input_names}
    if before_hashes != after_hashes:
        raise RuntimeError('an input changed during analysis')
    return dict(schema='delivery_window_dynamic_geometry_replay_v1',
        trial=str(trial.resolve()), request_id=request_id,
        original_input_hash=plan['input_hash'], plan_ordinal=plan['plan_ordinal'],
        input_sha256=before_hashes, original_files_unchanged=True,
        script_sha256=sha256(Path(__file__)), helper_sha256=sha256(Path(native.__file__)),
        original_candidate_geometry_sha256=candidate.geometry_hash(),
        frozen_snapshot_sha256=native.digest(source),
        latest_snapshot_sha256=native.digest(acceptance['latest_snapshot']),
        overlay_reconstruction=reconstruction,
        sphere_count=len(candidate.body), base_spheres=int(np.sum(candidate.body[:, 4] == 0)),
        arm_spheres=int(np.sum(candidate.body[:, 4] == 1)),
        frozen_source_sequence=source['sequence'], frozen_source_point_count=len(source['points']),
        latest_source_sequence=acceptance['latest_snapshot']['sequence'],
        latest_source_point_count=len(acceptance['latest_snapshot']['points']),
        original_native_success=final['native_success'], original_native_reason=final['native_reason'],
        original_prediction_limit_s=original_limit, counterfactual_slow_limit_s=float(extended_limit),
        recorded_uncertainty_at_0_1_2_s=overlay['uncertainty_at_0_1_2_s'],
        frozen_source_epoch=source_epoch, requested_epoch=plan['requested_epoch'],
        original_frozen_end_epoch=source_epoch + original_limit,
        counterfactual_frozen_end_epoch=source_epoch + extended_limit,
        rejected_candidate_point=dict(
            logged={k: v for k, v in reject.items() if isinstance(v, (str, int, float, bool))},
            original_backend=original, counterfactual_backend=extended,
            original_latest_publication=latest,
            counterfactual_frozen_source_at_publication_clock=frozen_at_publication),
        newly_covered_window=dict(source_relative_start_exclusive_s=original_limit,
            source_relative_end_inclusive_s=extended_limit, nominal_scale=1., local_step_s=.01,
            original=scan_summary(original_samples), counterfactual=scan_summary(extended_samples)),
        scope=('Exact recorded candidate, sphere geometry, source points and velocities. '
               'Only the explicitly labeled slow CV time domain is counterfactual. '
               'The extra interval is an assumed constant-velocity continuation, not '
               'originally certified prediction data. No full ESDF, optimizer, latest '
               'gate, physical-contact or execution replay is performed. '
               'Global latest/MPPI prediction limit is not changed.'))


def narrative(report):
    point = report['rejected_candidate_point']
    original, extended = point['original_backend'], point['counterfactual_backend']
    latest = point['original_latest_publication']
    scan = report['newly_covered_window']['counterfactual']
    first, minimum = scan['first_collision'], scan['minimum']
    lines = ['旧候选的交付时域覆盖核对', '',
        '本次只核对一个问题：延长慢层自身的 CV 检查时域，能否看到上一轮已经漏出窗口的风险。',
        '输入仍是上一轮 B 组 q%d 的原始候选、原始碰撞球和原始预测点，未重新优化轨迹。' % report['request_id'], '',
        '原输入哈希：%s。两个原始日志文件计算前后的 SHA-256 一致。' % report['original_input_hash'],
        '冻结源 seq %d 有 %d 个点；最新验收源 seq %d 有 %d 个点。机器人仍为 %d 个碰撞球。' % (
            report['frozen_source_sequence'], report['frozen_source_point_count'],
            report['latest_source_sequence'], report['latest_source_point_count'], report['sphere_count']), '',
        '在旧验收拒绝的候选位置（轨迹本地时间 %.6f 秒）：' % original['local_t_s'],
        '按慢层原来的请求时钟，该点查询时刻距冻结源 %.6f 秒。原上限为 %.1f 秒，因此动态查询无效，只剩原静态/自身检查。' % (
            original['seconds_after_source'], report['original_prediction_limit_s']),
        '明确假设沿原 CV 继续外推至 %.1f 秒后，同一候选、同一冻结源的模型间隙为 %.2f cm，%s。' % (
            report['counterfactual_slow_limit_s'], 100 * extended['minimum_model_gap_m'],
            '存在球点模型相交' if extended['model_collision'] else '未见球点模型相交'),
        '原最新验收源、原验收时钟复算的间隙为 %.2f cm，也存在模型相交。两种时钟不可混为一次完全相同的查询。' % (
            100 * latest['minimum_model_gap_m']), '',
        '只扫描新增 %.1f～%.1f 秒源时域，间隔 0.01 秒：共 %d 个采样点，其中 %d 个出现模型相交。' % (
            report['original_prediction_limit_s'], report['counterfactual_slow_limit_s'],
            scan['sample_count'], scan['collision_sample_count'])]
    if first is not None:
        lines.append('新增窗口首个模型相交：轨迹本地时间 %.6f 秒，距冻结源 %.3f 秒，间隙 %.2f cm。' % (
            first['local_t_s'], first['seconds_after_source'], 100 * first['minimum_model_gap_m']))
    if minimum is not None:
        lines.append('新增窗口最小模型间隙 %.2f cm。' % (100 * minimum['minimum_model_gap_m']))
    lines += ['',
        '可以得出的结论：拟增加的慢层时域能够把这条旧候选的风险纳入检查，值得进行在线 A/B 验证。',
        '尚不能得出的结论：延长后一定能更快交付、一定能优化出可行轨迹，或预测 4.5 秒已经有实测准确性保证。',
        '4.5 秒是本轮待验证的恒速外推假设，不是把原 3 秒记录改称已认证有效；最新验收和 MPPI 的全局 3 秒上限没有改变。',
        '本复算不含完整 ESDF、轮约束、自碰撞或求解器重跑。负间隙是预测模型相交，不等于实验发生了物理碰撞。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, default=DEFAULT_TRIAL)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--request-id', type=int, default=2)
    parser.add_argument('--slow-limit', type=float, default=4.5)
    args = parser.parse_args()
    report = audit(args.trial, args.request_id, args.slow_limit)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / '旧候选覆盖核对.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (args.out / '旧候选覆盖核对.txt').write_text(narrative(report))
    print(json.dumps(dict(output=str(args.out.resolve()),
        original_input_hash=report['original_input_hash'],
        original_query_valid=report['rejected_candidate_point']['original_backend']['query_valid'],
        counterfactual_query_valid=report['rejected_candidate_point']['counterfactual_backend']['query_valid'],
        counterfactual_model_gap_m=report['rejected_candidate_point']['counterfactual_backend']['minimum_model_gap_m'],
        newly_covered_samples=report['newly_covered_window']['counterfactual']['sample_count'],
        newly_covered_collision_samples=report['newly_covered_window']['counterfactual']['collision_sample_count']),
        ensure_ascii=False))


if __name__ == '__main__':
    main()

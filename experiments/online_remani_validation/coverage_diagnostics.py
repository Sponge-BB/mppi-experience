#!/usr/bin/env python3
"""Summarize one recorded head-on A/B, including timing alignment evidence.

Reads complete online runs; does not execute or replay a controller. Dynamic
coverage is reported from the actual backend scope, not the nominal parameter.
"""
import argparse
from collections import Counter
import copy
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from report_frontend_rejoin import keyframes
from report_static_agility import gif, show
from report_time_indexed_collision import OUTCOMES, TERMINATIONS, inspect
from replay_temporal_collision import Candidate, digest, seconds, summarize as summarize_checks
from replay_temporal_collision import temporal_check, temporal_gap, time_samples


def compact(check):
    return {k: v for k, v in check.items() if k != 'samples'}


def phase_check(candidate, snapshot, overlay, now, phase, scale, step):
    remaining = overlay['maximum_prediction_s']-max(0., now-seconds(snapshot))
    end = min(candidate.duration, phase+max(0., remaining)*scale)
    samples = []
    for local_t in time_samples(end, step, phase):
        value = temporal_gap(candidate, local_t, now+(local_t-phase)/scale,
                             snapshot, overlay, 'native_cv_uncertainty')
        value['branch'] = 'trajectory'
        samples.append(value)
    return summarize_checks(samples, mapping='projected_phase_at_validation_now',
        execution_scale=scale, phase_local_t_s=phase, valid_wall_horizon_s=remaining,
        insufficient_horizon=remaining <= 1e-3)


def frozen_clock_check(candidate, snapshot, overlay, epoch, scale, model):
    """One fixed candidate and frozen source; compare clock/radius individually."""
    samples = []
    for local_t in time_samples(candidate.duration, .05):
        query = epoch+local_t/scale
        if query > seconds(snapshot)+overlay['maximum_prediction_s']+1e-6:
            break
        value = temporal_gap(candidate, local_t, query, snapshot, overlay, model)
        value['branch'] = 'trajectory'
        samples.append(value)
    return summarize_checks(samples, mapping='fixed_epoch_plus_local_t_over_scale',
        candidate_epoch=epoch, execution_scale=scale, model=model)


def frozen_pairs(candidate, plan, boundary, scales):
    overlay = copy.deepcopy(plan['overlay'])
    snapshot = overlay['source_snapshot']
    if not snapshot or not overlay['source_exact']:
        return []
    age = max(0., overlay['refresh_nsec']/1e9-seconds(snapshot))
    overlay['horizon_s'] = max(0., overlay['maximum_prediction_s']-age)
    epochs = [('requested_epoch', plan['requested_epoch'])]
    if boundary and abs(boundary['measurement_epoch']-plan['requested_epoch']) > 1e-8:
        epochs.append(('measurement_epoch', boundary['measurement_epoch']))
    result = []
    for label, epoch in epochs:
        for scale in scales:
            native = frozen_clock_check(candidate, snapshot, overlay, epoch, scale,
                'native_cv_uncertainty')
            native['clock_source'] = label
            result.append(compact(native))
            if label == 'requested_epoch':
                padded = frozen_clock_check(candidate, snapshot, overlay, epoch, scale,
                    'backend_guide_radius')
                padded['clock_source'] = label
                result.append(compact(padded))
    return result


def candidate_pairs(captures, timeline):
    plans = {r['plan_ordinal']: r for r in captures if r['stage'] == 'planning'}
    boundaries = {r['request_id']: r for r in timeline if r['event'] == 'coverage_alignment_boundary'}
    phases = [r for r in timeline if r['event'] == 'candidate_phase_alignment']
    pairs = []
    for acceptance in [r for r in captures if r['stage'] == 'acceptance']:
        plan = plans[acceptance['plan_ordinal']]
        candidate = Candidate(acceptance['candidate'], plan)
        request, now = acceptance['request_id'], acceptance['validation_now_nsec']/1e9
        matching = [r for r in phases if r['request_id'] == request and abs(r['validation_now']-now) < 1e-6]
        if len(matching) != 1:
            pairs.append(dict(request_id=request, skipped=True, reason='exact_phase_hook_missing_or_ambiguous'))
            continue
        phase = matching[0]
        overlay, latest = acceptance['acceptance_overlay'], acceptance['latest_snapshot']
        scales = sorted(set((acceptance['resume_scale'], max(.1, .5*acceptance['resume_scale']))))
        checks = []
        for scale in scales:
            original = temporal_check(candidate, latest, overlay, now,
                acceptance['candidate']['start_epoch'], scale, acceptance['check_step_s'],
                'native_cv_uncertainty', 'original_publication_restart_zero')
            projected = phase_check(candidate, latest, overlay, now,
                phase['diagnostic_projected_phase_local_t'], scale, acceptance['check_step_s'])
            checks.extend([compact(original), compact(projected)])
        frozen = plan['overlay']['source_snapshot']
        frozen_checks = frozen_pairs(candidate, plan, boundaries.get(request), scales)
        pairs.append(dict(request_id=request, plan_ordinal=acceptance['plan_ordinal'],
            candidate_geometry_sha256=candidate.geometry_hash(), latest_snapshot_sha256=digest(latest),
            frozen_snapshot_sha256=digest(frozen) if frozen else None,
            new_phase_mapping=phase['new_phase_mapping'], phase_hook=phase,
            same_latest_snapshot_restart_vs_projected=checks,
            same_frozen_snapshot_clock_and_radius=frozen_checks,
            scope='Dynamic model counterfactual for one unchanged polynomial and sphere body; no static/self/wheel or actual MPPI execution replay'))
    return pairs


def native_candidate_pairs(captures, timeline):
    plans = {r['plan_ordinal']: r for r in captures if r['stage'] == 'planning'}
    boundaries = {r['request_id']: r for r in timeline if r['event'] == 'coverage_alignment_boundary'}
    result = []
    for final in [r for r in captures if r['stage'] == 'final_candidate']:
        plan = plans[final['plan_ordinal']]
        candidate = Candidate(final['candidate'], plan)
        result.append(dict(request_id=final['request_id'], plan_ordinal=final['plan_ordinal'],
            native_success=final['native_success'], native_reason=final['native_reason'],
            candidate_geometry_sha256=candidate.geometry_hash(),
            native_gate_first_local_t_s=.05,
            checks=frozen_pairs(candidate, plan, boundaries.get(final['request_id']), (.5, 1.)),
            scope='Unchanged final native candidate; dynamic model counterfactual only. Includes t=0 separately; native gate starts at t=.05s.'))
    return result


def load_trial(source, label):
    row, trial = inspect(source, label)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    row['alignment_events'] = [r for r in timeline
        if r['event'].startswith('coverage_') or r['event'] in
            ('candidate_phase_alignment', 'candidate_measured_pose_check')]
    row['coverage_scopes'] = [r for r in timeline if r['event'] == 'time_indexed_collision_scope']
    row['coverage_summaries'] = [r for r in timeline if r['event'] == 'time_indexed_collision_summary']
    row['native_capture_counts'] = dict(Counter(r['stage'] for r in captures))
    row['same_candidate_pairs'] = candidate_pairs(captures, timeline)
    row['native_candidate_pairs'] = native_candidate_pairs(captures, timeline)
    refresh = [r for r in row['remani']['requests'] if r['request_id'] > 1]
    commands = trial.reader.jsonl('state_mppi_cycles.jsonl')
    delivery = dict(first_refresh_to_new_reference_kernel_wall_ms=None,
        first_refresh_to_new_reference_validated_command_wall_ms=None,
        initial_reference_revision=1,
        definition='One-goal SIL: earliest refresh request_begin.monotonic to new revision >1 first kernel / first validated command_publish_after.monotonic. ROS consumed-reference metric is reported separately.')
    if refresh:
        began = min(r['start_mono'] for r in refresh)
        new_kernels = [r for r in commands if r.get('guide_revision', 0) is not None
            and r.get('guide_revision', 0) > 1 and r.get('online_step_invoked')
            and r['online_step_entry_monotonic'] >= began]
        valid = [r for r in commands if r.get('guide_revision', 0) is not None
            and r.get('guide_revision', 0) > 1 and r.get('validated_command_published')
            and r['command_publish_after_monotonic'] >= began]
        if new_kernels:
            first = min(new_kernels, key=lambda r: r['online_step_entry_monotonic'])
            delivery['first_refresh_to_new_reference_kernel_wall_ms'] = 1000*(first['online_step_entry_monotonic']-began)
            delivery['first_new_kernel_cycle_id'] = first['cycle_id']
        if valid:
            first = min(valid, key=lambda r: r['command_publish_after_monotonic'])
            delivery['first_refresh_to_new_reference_validated_command_wall_ms'] = 1000*(first['command_publish_after_monotonic']-began)
            delivery['first_new_validated_command_cycle_id'] = first['cycle_id']
    row['control_delivery'] = delivery
    row['coverage_acceptances'] = []
    for acceptance in [r for r in captures if r['stage'] == 'acceptance']:
        request = acceptance['request_id']
        scopes = [r for r in row['coverage_scopes'] if r['request_id'] == request]
        scope = scopes[-1] if scopes else None
        snapshot = acceptance['latest_snapshot']
        now = acceptance['validation_now_nsec'] / 1e9
        maximum_prediction = acceptance['acceptance_overlay']['maximum_prediction_s']
        end = snapshot['stamp_nsec'] / 1e9 + maximum_prediction
        row['coverage_acceptances'].append(dict(request_id=request,
            plan_ordinal=acceptance['plan_ordinal'], validation_epoch=now,
            candidate_epoch=acceptance['candidate']['start_epoch'],
            candidate_age_s=now-acceptance['candidate']['start_epoch'],
            latest_source_sequence=snapshot['sequence'],
            latest_source_stamp=snapshot['stamp_nsec']/1e9,
            latest_valid_end_epoch=end, latest_remaining_horizon_s=end-now,
            backend=scope,
            backend_to_latest_end_difference_s=(end-scope['indexed_end_epoch']) if scope else None))
    row['rejection_coverage'] = []
    for rejection in row['latest_rejections']:
        scopes = [r for r in row['coverage_scopes'] if r['request_id'] == rejection['request_id']]
        scope = scopes[-1] if scopes else None
        row['rejection_coverage'].append(dict(request_id=rejection['request_id'],
            branch=rejection['branch'], local_t=rejection['local_t'],
            execution_scale=rejection['execution_scale'], query_epoch=rejection['query_stamp'],
            backend_indexed_end_epoch=scope['indexed_end_epoch'] if scope else None,
            seconds_beyond_backend_window=(rejection['query_stamp']-scope['indexed_end_epoch']) if scope else None,
            planning_source_sequence=scope['source_sequence'] if scope else None,
            latest_source_sequence=rejection['snapshot_sequence']))
    trial.verify_unchanged()
    return row, trial


def narrative(rows, notes):
    lines = ['预测覆盖范围与轨迹起点时间对齐：迎面对照', '',
        '本轮同一次在线运行同时记录：规划预测范围、起点时间、发布时候选相位、最终采用、MPPI失败和完整循环耗时。',
        'A和B的实际运行条件以各自manifest.json为准；所有完成和失败结果都保留。', '']
    lines += ['B同时对齐预测范围、测量起点时间和发布相位，并将连续动态查询半径与原生CV验收统一，去掉离散扫掠的额外采样补偿；物理球及底盘地图分辨率余量未缩小。',
        '因此在线A/B检验的是这一组一致性调整，不能把所有变化都归因于预测窗延长。下方同候选复算分别比较时间映射和额外半径。', '']
    lines += list(notes) + [''] if notes else []
    for row in rows:
        d = row['delivery']
        lines += [row['label'],
            '结果：%s；实走%s米，运动%s秒，非目标低速停留%s秒。' % (
                '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination']),
                show(row['actual_path_length_m']), show(row['motion_duration_s']),
                show(row['non_goal_low_speed']['total_s'])),
            '首次重规划请求到MPPI采用%s毫秒；成功交付那次请求耗时%s毫秒。' % (
                show(d['first_refresh_start_to_consumed_reference_ms']),
                show(d['first_adopted_request_wall_ms'])),
            '墙钟：首次请求到新参考首个MPPI计算%s毫秒，到新参考首条有效控制发布完成%s毫秒。' % (
                show(row['control_delivery']['first_refresh_to_new_reference_kernel_wall_ms']),
                show(row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms'])),
            '最小物理球采样间隙%s厘米；%s采样接触。' % (
                show(100*row['minimum_physical_sample_gap_m']),
                '发生' if row['physical_sample_collision'] else '未发现'),
            'MPPI完整循环P95/最大%s/%s毫秒，超过200毫秒%d次，命令发布最大间隔%s毫秒。' % (
                show(row['callback']['p95_ms']), show(row['callback']['max_ms']),
                row['callback']['over_200_ms'], show(row['maximum_command_publication_gap_ms'])),
            '控制结果：'+json.dumps(row['reason_counts'], ensure_ascii=False),
            '优化预算停止%d次。' % len(row['budget_stops'])]
        for request in row['remani']['requests']:
            lines.append('请求%d：%s，墙钟%s毫秒；后端原因%s；发布拒绝%s。' % (
                request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                show(request.get('request_wall_ms')), [c.get('reason') for c in request.get('optimizer_calls', [])],
                request.get('acceptance_rejections', [])))
        for scope in row['coverage_scopes']:
            lines.append('请求%d：后端按时查询%s，源seq%d，有效绝对时间[%s,%s]秒，相对请求起点还覆盖%s秒。' % (
                scope['request_id'], '启用' if scope['active'] else '未启用', scope['source_sequence'],
                show(scope['indexed_begin_epoch'], 6), show(scope['indexed_end_epoch'], 6),
                show(scope['indexed_end_epoch']-scope['requested_epoch'])))
        for summary in row['coverage_summaries']:
            lines.append('请求%d：按时距离查询%d次，扫掠回退%d次；有效时间分支%s，源时刻前分支%s，未知未来分支%s。' % (
                summary['request_id'], summary['indexed_queries'], summary['swept_fallback_queries'],
                summary.get('valid_branches', '未记录'), summary.get('before_source_branches', '未记录'),
                summary.get('unknown_future_branches', '未记录')))
        for acceptance in row['coverage_acceptances']:
            lines.append('请求%d发布验收：最新源seq%d，可用未来%s秒，窗口末端比本次规划源晚%s秒；候选时间戳到验收已过%s秒。' % (
                acceptance['request_id'], acceptance['latest_source_sequence'],
                show(acceptance['latest_remaining_horizon_s']),
                show(acceptance['backend_to_latest_end_difference_s']), show(acceptance['candidate_age_s'])))
        for rejection in row['rejection_coverage']:
            lines.append('请求%d发布拒绝：%s，候选t=%s秒、比例%s；风险查询比后端预测窗口末端晚%s秒（负值表示仍在窗内）。' % (
                rejection['request_id'], rejection['branch'], show(rejection['local_t']),
                show(rejection['execution_scale']), show(rejection['seconds_beyond_backend_window'])))
        for event in row['alignment_events']:
            # Keep hook fields verbatim; this prevents an absent or renamed
            # measurement from being silently replaced with a different clock.
            if event['event'] == 'candidate_phase_alignment':
                lines.append('请求%d相位匹配：候选t=%s秒，诊断投影t=%s秒，位置残差%s厘米，朝向残差%s弧度；方法%s。' % (
                    event['request_id'], show(event.get('phase_local_t')),
                    show(event.get('diagnostic_projected_phase_local_t')),
                    show(100*event['position_residual_m']) if event.get('position_residual_m') is not None else '未记录',
                    show(event.get('yaw_residual_rad')), event.get('phase_method', '未记录')))
            elif event['event'] == 'coverage_alignment_boundary':
                lines.append('请求%d起点：实测时刻%s秒，请求起点时刻%s秒，相差%s毫秒；位置(%s,%s)。' % (
                    event['request_id'], show(event['measurement_epoch'], 6), show(event['requested_epoch'], 6),
                    show(1000*(event['requested_epoch']-event['measurement_epoch'])),
                    show(event['boundary_x']), show(event['boundary_y'])))
            elif event['event'] == 'candidate_measured_pose_check':
                lines.append('请求%d发布时实际位置检查：预测查询%s，模型相交%s。' % (
                    event['request_id'], event['query_valid'], event['collision']))
        for pair in row['same_candidate_pairs']:
            if pair.get('skipped'):
                continue
            lines.append('请求%d同一候选、同一最新预测复算（只比较发布相位）：' % pair['request_id'])
            for check in pair['same_latest_snapshot_restart_vs_projected']:
                minimum = check.get('minimum')
                lines.append('  %s，速度比例%s：%s，最小模型间隙%s厘米。' % (
                    '从候选开头检查' if check['mapping'] == 'original_publication_restart_zero' else '从实测位置投影相位检查',
                    show(check['execution_scale']), '存在模型碰撞' if check.get('first_collision') else '未见模型碰撞',
                    show(100*minimum['minimum_model_gap_m']) if minimum else '无有效点对'))
            lines.append('请求%d同一候选、同一冻结预测复算（分别比较起点时刻和额外半径）：' % pair['request_id'])
            for check in pair['same_frozen_snapshot_clock_and_radius']:
                minimum = check.get('minimum')
                lines.append('  %s，%s，速度比例%s：%s，最小模型间隙%s厘米，时域外样本%d个。' % (
                    '请求起点时刻' if check['clock_source'] == 'requested_epoch' else '实测起点时刻',
                    '连续原生CV半径' if check['model'] == 'native_cv_uncertainty' else '叠加原离散扫掠补偿',
                    show(check['execution_scale']), '存在模型碰撞' if check.get('first_collision') else '未见模型碰撞',
                    show(100*minimum['minimum_model_gap_m']) if minimum else '无有效点对',
                    check['invalid_sample_count']))
        for pair in row['native_candidate_pairs']:
            if pair['native_success']:
                continue
            lines.append('请求%d原生最终候选未通过（%s），同冻结输入复算：' % (
                pair['request_id'], pair['native_reason']))
            for check in pair['checks']:
                first = check.get('first_collision')
                lines.append('  %s，%s，速度比例%s：%s%s。' % (
                    '请求起点时刻' if check['clock_source'] == 'requested_epoch' else '实测起点时刻',
                    '连续原生CV半径' if check['model'] == 'native_cv_uncertainty' else '叠加原离散扫掠补偿',
                    show(check['execution_scale']), '模型相交' if first else '未见模型相交',
                    '，首次位于候选t=%s秒' % show(first['local_t_s']) if first else ''))
        lines += ['']
    lines += ['解释与边界',
        '“窗口末端”是这次冻结预测真正覆盖到的绝对时刻。最新快照更晚时，它的3秒末端也会更晚；参数都写3秒并不意味着绝对覆盖范围完全相同。',
        '未知未来分支表示超出已记录预测可用范围，不能解释成已确认无障碍。',
        '完整循环统计含失败和发布后记录，排除休眠；使用实际采用记录统计交付。',
        '模型检查间隙与物理球采样间隙不同。少量在线对照用于定位本场景问题，不代表总体成功率。',
        '同候选复算包括t=0以观察固定起点；原生最终硬检查从t=0.05秒开始，不能将t=0单独风险直接当作实际首拒。',
        '本轮实际发布映射以candidate_phase_alignment的new_phase_mapping和time_mapping为准；不根据继承日志中的旧说明推断执行行为。',
        'GIF中的绿色为记录到的获准局部展开，不是全部MPPI搜索样本。',
        '所有结果来自固定机械臂、完整REMANI+底盘MPPI的ROS数值闭环。']
    return '\n'.join(lines)+'\n'


def brief(rows, notes):
    lines = ['本轮实验结果', '',
        '同一迎面场景跑A/B两组；每组同时记录求解、验收、轨迹采用、MPPI控制和运动结果。',
        'A是上一轮已启用时间化查询的原型：1.2秒内按时刻查询，窗外回退扫掠层；不是原生REMANI。',
        'B把动态规划查询覆盖到可用CV窗口，统一连续CV半径，将起点时间与实测状态对应，并从发布时的匹配相位检查候选。',
        '物理碰撞球没有缩小。在线对照检验这组调整，同候选复算用于分别判断时间映射和额外半径的影响。', '']
    lines += list(notes)+[''] if notes else []
    for row in rows:
        d = row['delivery']
        command_ms = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        lines += [row['label'],
            '结果：%s；%s物理球采样接触，最小间隙%s厘米。' % (
                '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination']),
                '发生' if row['physical_sample_collision'] else '未发现', show(100*row['minimum_physical_sample_gap_m'])),
            '首次重规划到新参考首条有效控制发布（墙钟）：%s；成功交付那次计算：%s。' % (
                show(command_ms/1000)+'秒' if command_ms is not None else '没有交付记录',
                show(d['first_adopted_request_wall_ms']/1000)+'秒'
                if d['first_adopted_request_wall_ms'] is not None else '无'),
            '按仿真时钟记录的首次请求到参考采用%s秒；这与上面的有效控制发布终点不同。' % (
                show(d['first_refresh_start_to_consumed_reference_ms']/1000)
                if d['first_refresh_start_to_consumed_reference_ms'] is not None else '无'),
            'MPPI完整循环P95/最大%s/%s毫秒；没有可行控制%d次，局部验收失败%d次。' % (
                show(row['callback']['p95_ms']), show(row['callback']['max_ms']),
                row['reason_counts'].get('no_feasible_candidate', 0),
                row['reason_counts'].get('local_validation_failed', 0))]
        for request in row['remani']['requests']:
            if request['request_id'] <= 1:
                continue
            lines.append('重规划请求%d：%s，耗时%s秒。' % (
                request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                show(request['request_wall_ms']/1000)))
        lines += ['']
    lines += ['详细的窗口、起点时间、相位残差及同候选复算见“覆盖与交接诊断.txt”和summary.json。',
        '这些是固定机械臂的完整REMANI+MPPI ROS数值闭环，尚不是实车验证；少量对照用于定位本场景问题。']
    return '\n'.join(lines)+'\n'


def chart(rows, output):
    image = Image.new('RGB', (1250, 170+160*len(rows)), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    draw.text((20, 15), '预测范围与起点时间对齐：同场景在线对照', font=title, fill='#333333')
    draw.text((20, 52), '墙钟计时至新参考首条有效控制发布；单次计算耗时取最终被采用的那次请求。', font=small, fill='#666666')
    for i, row in enumerate(rows):
        top = 90+160*i
        color = '#2874a6' if i == 0 else '#c47723'
        draw.line((20, top-10, 1230, top-10), fill='#dddddd')
        draw.text((20, top), row['label'], font=text, fill=color)
        d = row['delivery']
        draw.text((20, top+33), '首次请求→首条有效控制 %s ms；交付那次计算 %s ms' % (
            show(row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms'], 1),
            show(d['first_adopted_request_wall_ms'], 1)), font=text, fill='#444444')
        draw.text((20, top+67), '重规划：'+'；'.join('%s %d次' % (OUTCOMES.get(k, k), v)
            for k, v in row['replanning_outcomes'].items()), font=small, fill='#555555')
        draw.text((20, top+95), '结果 %s；最小物理采样间隙 %.2f cm；低速停留 %.2f s' % (
            '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination']),
            100*row['minimum_physical_sample_gap_m'], row['non_goal_low_speed']['total_s']), font=small, fill='#555555')
        draw.text((20, top+123), 'MPPI完整循环 P95/最大 %s/%s ms；无可行候选 %d次；局部验收失败 %d次' % (
            show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1),
            row['reason_counts'].get('no_feasible_candidate', 0), row['reason_counts'].get('local_validation_failed', 0)),
            font=small, fill='#555555')
    draw.text((20, image.height-35), '固定臂 ROS数值闭环；完整保留失败结果；少量在线对照用于定位问题。', font=small, fill='#777777')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--changed', type=Path, required=True)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--baseline-label', default='A：上一轮时间化原型（1.2秒＋扫掠回退）')
    parser.add_argument('--changed-label', default='B：完整CV窗口与测量时间对齐')
    parser.add_argument('--note', action='append', default=[])
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    output = args.out or args.baseline.parent/'report'
    output.mkdir(parents=True, exist_ok=True)
    selected = [load_trial(args.baseline, args.baseline_label), load_trial(args.changed, args.changed_label)]
    rows, trials = zip(*selected)
    chart(rows, output/'交付与运动对照.png')
    animation = output/'迎面动态对照.gif'
    gif_rows = [dict(row, termination=TERMINATIONS.get(row['termination'], row['termination'])) for row in rows]
    artifacts = gif(gif_rows, trials, animation, args.max_frames)
    keyframes(animation, output/'迎面动态关键帧.png')
    (output/'实验结果.txt').write_text(brief(rows, args.note))
    (output/'覆盖与交接诊断.txt').write_text(narrative(rows, args.note))
    (output/'summary.json').write_text(json.dumps(dict(rows=rows, notes=args.note, artifacts=artifacts,
        scope='Recorded fixed-arm ROS SIL A/B; no controller replay'), ensure_ascii=False, indent=2)+'\n')
    print('COVERAGE_ALIGNMENT_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

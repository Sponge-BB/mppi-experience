#!/usr/bin/env python3
"""Report one normal online pair comparing post-feasible native-check schedules.

Read-only analysis after timed trials stop. Pending accepted optimizer states
are never certified or counted as delivered references. Only recorded native
checks, latest admission, consumed references and validated commands count.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from replay_temporal_collision import digest
from report_fast_refresh_feedback import animation
from report_feasible_quality import load_online as load_quality_online
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window
from report_long_goal_crossing import settings
from report_prefix_handoff import normalized_geometry
from report_static_agility import low_speed, show
from report_static_width_diagnosis import sample_curve
from report_time_indexed_collision import OUTCOMES, TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/deferred_quality_20261101_01'
LABELS = ('A：现有复核节奏', 'B：末尾集中复核')


def unique(records):
    return records[0] if len(records) == 1 else None


def initial_reference(trial, row):
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    finals = [r for r in captures if r.get('stage') == 'final_candidate'
              and r.get('request_id') == 1 and r.get('native_success') is True]
    chosen = unique(finals)
    parts = chosen['candidate']['parts'] if chosen else None
    references = [r for r in trial.refs if r.get('revision') == 1]
    ref = unique(references)
    error = None
    if chosen and ref:
        values = sample_curve(dict(parts=parts), ref['times'])
        error = float(np.max(np.linalg.norm(values-np.asarray(ref['reference'])[:, :2], axis=1)))
    q1 = row['quality']['initial_planning_checkpoints']
    return dict(native_success_count=len(finals), consumed_revision_one_count=len(references),
        normalized_polynomial_sha256=digest(normalized_geometry(parts)) if parts is not None else None,
        consumed_reference_sha256=ref.get('sha256') if ref else None,
        native_polynomial_to_consumed_reference_max_xy_error_m=error,
        first_planning_polish_values=[r['polish_enabled'] for r in q1],
        first_planning_later_improvements=[r.get('improvements') for r in q1],
        normalized_polynomial_parts=normalized_geometry(parts) if parts is not None else None,
        scope='All q1 native-success final records must bind uniquely to revision one. Polynomial hash excludes epoch and capture clocks; consumed message hashes contain timing and need not match across runs.')


def initial_pair_contract(rows):
    a, b = [r['initial_reference'] for r in rows]
    available = all(r['normalized_polynomial_sha256'] is not None for r in (a, b))
    same = a['normalized_polynomial_sha256'] == b['normalized_polynomial_sha256'] if available else None
    initial_mode = all(r['first_planning_polish_values']
                       and all(value is False for value in r['first_planning_polish_values']) for r in (a, b))
    return dict(unique_native_initial_polynomials_available=available,
        initial_native_polynomials_exactly_equal=same,
        both_initial_plans_first_feasible_exit=initial_mode,
        initial_reference_controlled=bool(same is True and initial_mode),
        polynomial_sha256=[r['normalized_polynomial_sha256'] for r in (a, b)],
        interpretation=('首次规划完整多项式完全相同，且两组均在首个可行解返回。'
                        if same is True and initial_mode else
                        '首次规划完整多项式未能确认相同，起始参考对照不成立；全程A/B差异只能描述，不能归因复核节奏。'),
        online_input_scope='Identical initial polynomials do not freeze later trigger clocks, CV maps, measured poses or MPPI seed histories. Refresh starts and each same-solve first-safe-to-selected change are recorded separately.')


def post_adoption_motion(trial, begin, seconds=2.):
    end = min(begin+seconds, trial.end)
    _, goal, _ = settings(trial.manifest)
    stop = low_speed(trial.state, begin, end, goal)
    reverse_s, path_m, elapsed, min_v = 0., 0., 0., None
    for a, b in zip(trial.state, trial.state[1:]):
        dt = float(b['t']-a['t'])
        overlap = max(0., min(end, float(b['t']))-max(begin, float(a['t'])))
        if dt <= 0. or overlap <= 0.:
            continue
        elapsed += overlap
        reverse_s += overlap if float(a['v']) < -.02 else 0.
        path_m += math.hypot(float(b['x']-a['x']), float(b['y']-a['y']))*overlap/dt
        min_v = min(min_v, float(a['v'])) if min_v is not None else float(a['v'])
    return dict(begin_elapsed_s=begin, end_elapsed_s=end, requested_window_s=seconds,
        observed_window_s=elapsed, non_goal_near_stop=stop,
        reverse_translation_duration_s=reverse_s, minimum_measured_v_mps=min_v,
        sampled_polyline_path_length_m=path_m,
        interpretation='Actual state after the first validated command using this reference. Near-stop uses existing |v|<.02, |w|<.05, goal distance>.1m. Reverse means measured v<-.02m/s; it is observed reverse motion, not proof that the new reference caused a correction.')


def refresh_analysis(row, trial):
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    solver = trial.reader.jsonl('solver_trace.jsonl')
    commands = trial.reader.jsonl('state_mppi_cycles.jsonl')
    raw_requests = {r['request_id']: r for r in timeline if r.get('event') == 'request_begin'}
    deferred = [r for r in captures if r.get('stage') == 'deferred_quality']
    calls = []
    for checkpoint in row['quality']['checkpoints']:
        request_id = checkpoint['request_id']
        if request_id <= 1:
            continue
        matching = [r for r in deferred if r.get('request_id') == request_id
                    and r.get('plan_ordinal') == checkpoint['plan_ordinal']]
        record = unique(matching)
        if len(matching) != 1:
            raise ValueError('Missing or ambiguous deferred-quality record for request %s' % request_id)
        request = raw_requests[request_id]
        summary = unique([r for r in row['remani']['requests'] if r['request_id'] == request_id])
        summaries = [r for r in timeline if r.get('event') == 'early_delivery_summary'
                     and r.get('request_id') == request_id]
        internal = unique(summaries)
        opt_metrics = [r for r in solver if r.get('event') == 'end' and r.get('stage') == 'optimizer']
        expected_end = request.get('mono')
        opt_ends = [r for r in timeline if r.get('event') == 'stage_end'
                    and r.get('stage') == 'optimizer' and r.get('request_id') == request_id]
        resource = None
        if len(opt_ends) == 1 and opt_metrics:
            expected_end = opt_ends[0]['mono']
            nearest = min(opt_metrics, key=lambda r: abs(r['mono']-expected_end))
            resource = nearest if abs(nearest['mono']-expected_end) < .03 else None
        publications = [r for r in timeline if r.get('event') == 'reference_publish'
                        and r.get('request_id') == request_id]
        references = [r for r in trial.refs if any(abs(r['stamp']-p['guide_start']) < 1e-7 for p in publications)]
        revisions = {r['revision'] for r in references}
        valid = [r for r in commands if r.get('guide_revision') in revisions
                 and r.get('validated_command_published') is True
                 and r['command_publish_after_ros_time']-1000. <= trial.end]
        first_valid = min(valid, key=lambda r: r['command_publish_after_monotonic']) if valid else None
        first_mono = first_valid['command_publish_after_monotonic'] if first_valid else None
        publish = min(publications, key=lambda r: r['mono']) if publications else None
        matches = checkpoint['consumed_reference_matches']
        consumed_curve_matches = bool(references and matches and all(
            r['maximum_xy_polynomial_error_m'] < 1e-7 for r in matches))
        adopted = bool(checkpoint['safe_checkpoint_available'] and consumed_curve_matches and first_valid
                       and any(r['native_success'] is True for r in checkpoint['selected_matches_final_native_candidates']))
        phases = [r for r in row['prefix_handoff']['phase_alignment_events'] if r['request_id'] == request_id]
        checks = [r for r in timeline if r.get('event') == 'early_delivery_check'
                  and r.get('request_id') == request_id]
        native = [r for r in checks if r.get('native_checked') == 1]
        first_iteration = checkpoint.get('first_feasible_iteration')
        later = [r for r in native if first_iteration is not None and r['accepted_iteration'] > first_iteration]
        solver_calls = summary.get('optimizer_calls', []) if summary else []
        calls.append(dict(request_id=request_id, plan_ordinal=checkpoint['plan_ordinal'],
            refresh_start=dict(x=request['start_x'], y=request['start_y'], yaw=request['start_yaw'],
                sim_elapsed_s=request['sim']-1000., monotonic=request['mono']),
            quality_checkpoint=checkpoint, deferred_record=record,
            post_first_accepted_updates=record.get('post_first_accepted_count'),
            deferred_enabled=record.get('deferred_enabled'), pending_available=record.get('pending_available'),
            pending_cost=record.get('pending_cost') if record.get('pending_available') else None,
            pending_feasibility_at_cache='unverified',
            pending_later_native_certified_and_promoted=bool(record.get('deferred_check_native_pass')
                and record.get('promoted')),
            final_deferred_check_attempted=record.get('deferred_check_attempted'),
            final_deferred_check_reason=record.get('deferred_check_reason'),
            final_deferred_native_reason=record.get('deferred_check_native_reason'),
            final_deferred_collision_type=record.get('deferred_check_collision_type'),
            final_deferred_rejection_time_s=record.get('deferred_check_rejection_time'),
            final_deferred_native_pass=record.get('deferred_check_native_pass'),
            final_deferred_candidate_promoted=record.get('promoted'),
            returned_first_feasible=record.get('fallback_to_first'),
            native_check_count=len(native), native_wall_ms=sum(r['native_gate_ms'] for r in native),
            post_first_native_check_count=len(later), post_first_native_wall_ms=sum(r['native_gate_ms'] for r in later),
            request_wall_ms=summary.get('request_wall_ms') if summary else None,
            optimizer_total_wall_ms=sum(r['wall_ms'] for r in solver_calls) if solver_calls else None,
            internal_solver_wall_ms=internal.get('elapsed_ms') if internal else None,
            lbfgs_wall_ms=resource.get('lbfgs_wall_ms') if resource else None,
            native_checks_are_inside_internal_clock=True,
            final_native_check_time_scope='Optimizer total includes final native check after the internal solve clock. It is not added to the internal 1.2s budget claim.',
            native_final_success=any(r.get('success') is True for r in solver_calls),
            latest_rejection_count=len(checkpoint['native_to_latest_rejection']),
            reference_publications=len(publications), consumed_revisions=sorted(revisions),
            first_validated_command_cycle_id=first_valid['cycle_id'] if first_valid else None,
            first_validated_command_monotonic=first_mono,
            first_validated_command_elapsed_s=first_valid['command_publish_after_ros_time']-1000. if first_valid else None,
            request_to_validated_command_wall_s=first_mono-request['mono'] if first_mono is not None else None,
            publish_to_validated_command_wall_s=first_mono-publish['mono'] if first_mono is not None and publish else None,
            selected_curve_matches_consumed=consumed_curve_matches,
            selected_curve_actually_adopted=adopted,
            adopted_reference_geometry=checkpoint['selected_geometry'] if adopted else None,
            phase_alignment_events=phases,
            post_adoption_motion=post_adoption_motion(trial, first_valid['command_publish_after_ros_time']-1000.)
                if adopted else None,
            scope='Pending cost is descriptive until native promotion. Native-certified selected, final native success, latest publish, consumed curve and validated command are bound separately.'))
    first_adopted = next((r for r in calls if r['selected_curve_actually_adopted']), None)
    refresh_requests = [r for r in raw_requests.values() if r['request_id'] > 1]
    first_request = min(refresh_requests, key=lambda r: r['mono']) if refresh_requests else None
    return dict(calls=calls, first_actually_adopted=first_adopted,
        first_refresh_request_id=first_request['request_id'] if first_request else None,
        first_refresh_request_to_first_new_validated_command_wall_s=(
            first_adopted['first_validated_command_monotonic']-first_request['mono']
            if first_adopted and first_request else None),
        first_request_wait_semantics='Earliest refresh request to the first actually adopted refresh command; includes rejected requests and retry delay.',
        pending_proposals=sum(r.get('pending_available') is True for r in calls),
        deferred_check_attempts=sum(r.get('final_deferred_check_attempted') is True for r in calls),
        deferred_promotions=sum(r.get('final_deferred_candidate_promoted') is True for r in calls),
        deferred_fallbacks=sum(r.get('returned_first_feasible') is True for r in calls),
        selection='First chronological actually adopted refresh, not the best-looking cost or successful example.')


def load_online(source, label):
    row, trial = load_quality_online(source, label)
    row['initial_reference'] = initial_reference(trial, row)
    row['deferred_quality'] = refresh_analysis(row, trial)
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def conclusions(data):
    rows = data['rows']
    notes = [data['initial_pair_contract']['interpretation']]
    for row in rows:
        calls = row['deferred_quality']['calls']
        notes.append(row['label']+'：'+('到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination'])))+
            '，'+('发现' if row['physical_sample_collision'] else '未发现')+'物理球采样接触；'+
            '实走%s米、起步到结束%s秒。' % (show(row['actual_path_length_m'], 3), show(row['motion_duration_s'], 3)))
        notes.append('  首次避障反馈→首条实际有效新控制%s秒；首个重规划请求%d→首条实际新控%s秒，包含失败和重试。' % (
            show(row['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'], 3),
            row['deferred_quality']['first_refresh_request_id'],
            show(row['deferred_quality']['first_refresh_request_to_first_new_validated_command_wall_s'], 3)))
        notes.append('  重规划%d次；首解后完整复核%d次、共%s毫秒；实际采用%s条新参考。' % (
            len(calls), sum(c['post_first_native_check_count'] for c in calls),
            show(sum(c['post_first_native_wall_ms'] for c in calls), 1),
            sum(c['selected_curve_actually_adopted'] for c in calls)))
        adopted = row['deferred_quality']['first_actually_adopted']
        if adopted:
            q = adopted['quality_checkpoint']
            first, chosen = q['first_feasible_geometry'], q['selected_geometry']
            notes.append('  首条实际采用新参考来自请求%d：同次首解→选中，曲线运动时长%s→%s秒，长度%s→%s米，最大|y|%s→%s米。' % (
                adopted['request_id'], show(first['duration_s'], 3), show(chosen['duration_s'], 3),
                show(first['full_curve_path_length_m'], 3), show(chosen['full_curve_path_length_m'], 3),
                show(first['maximum_absolute_world_y_m'], 3), show(chosen['maximum_absolute_world_y_m'], 3)))
            notes.append('  辅助：成功交付的该请求自身→实际有效新控制%s秒；请求/优化器完整/内部求解%s/%s/%s毫秒。' % (
                show(adopted['request_to_validated_command_wall_s'], 3),
                show(adopted['request_wall_ms'], 1), show(adopted['optimizer_total_wall_ms'], 1),
                show(adopted['internal_solver_wall_ms'], 1)))
            motion = adopted['post_adoption_motion']
            notes.append('  采用后实测%s秒内：非目标近停%s秒、倒退%s秒、移动%s米。' % (
                show(motion['observed_window_s'], 3), show(motion['non_goal_near_stop']['total_s'], 3),
                show(motion['reverse_translation_duration_s'], 3), show(motion['sampled_polyline_path_length_m'], 3)))
        else:
            notes.append('  没有新候选同时核实通过最终验收、参考采用及有效控制；不计算已交付轨迹质量收益。')
        if row['callback']['max_ms'] > 200.:
            notes.append('  MPPI完整循环最大%s毫秒，超过200毫秒；不能说完整循环全部在时限内。' % show(row['callback']['max_ms'], 1))
    b = rows[1]['deferred_quality']
    notes.append('B末尾集中复核尝试%d次，提升%d次，退回首解%d次；未验收的pending候选不计可行改善。' % (
        b['deferred_check_attempts'], b['deferred_promotions'], b['deferred_fallbacks']))
    notes.append('相同首条参考只排除了首次规划开关差异；在线重规划时刻、起点和感知仍可能不同。本次单场景各一次，用于验证机制，不估计总体成功率。')
    return notes


def narrative(data):
    lines = ['首解后集中复核：实验结果', '',
        '当前关键任务：用相同求解预算减少重复验收占用，观察已交付轨迹质量和MPPI接续是否改善。',
        'A采用现有继续优化及复核节奏；B保存首个可行解，后续记录成本更低的真实已接受候选，在原1.2秒内预留约150毫秒集中复核一次。不通过或时间不足时保留首解。',
        '两组首次空旷规划均在首个可行解返回；重规划都启用MPPI短前缀和快层反馈，使用同一代价、约束及最终/最新动态验收。', '', '主要结果']+data['conclusions']
    for row in data['rows']:
        chain = row['delivery_chain_counts']
        lines += ['', row['label'],
            '请求→原生通过→最新发布→实际采用：%d→%d→%d→%d；最新动态拒绝%d次。' % (
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed'], len(row['latest_rejections'])),
            'MPPI完整循环P95/最大%s/%s毫秒；最小物理球采样间隙%s厘米。' % (
                show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1),
                show(100*row['minimum_physical_sample_gap_m'], 3))]
        for call in row['deferred_quality']['calls']:
            q = call['quality_checkpoint']
            start = call['refresh_start']
            lines.append('请求%d：起点(%s,%s)米、朝向%s度；原生最终通过%s，发布%d次，实际采用版本%s。' % (
                call['request_id'], show(start['x'], 3), show(start['y'], 3), show(math.degrees(start['yaw']), 2),
                call['native_final_success'], call['reference_publications'], call['consumed_revisions']))
            lines.append('  请求/优化器完整/内部求解/LBFGS：%s/%s/%s/%s毫秒；内部原生检查%d次、%s毫秒，其中首解后%d次、%s毫秒。' % (
                show(call['request_wall_ms'], 1), show(call['optimizer_total_wall_ms'], 1),
                show(call['internal_solver_wall_ms'], 1), show(call['lbfgs_wall_ms'], 1),
                call['native_check_count'], show(call['native_wall_ms'], 1),
                call['post_first_native_check_count'], show(call['post_first_native_wall_ms'], 1)))
            lines.append('  首解后接受更新%s次；末次复核%s，原生通过%s，提升%s，退回首解%s。' % (
                call['post_first_accepted_updates'], call['final_deferred_check_reason'],
                call['final_deferred_native_pass'], call['final_deferred_candidate_promoted'], call['returned_first_feasible']))
            if call['final_deferred_check_attempted'] and not call['final_deferred_candidate_promoted']:
                lines.append('  末检原生原因%s，碰撞类型%s，拒绝所在曲线时刻%s秒。' % (
                    call['final_deferred_native_reason'], call['final_deferred_collision_type'],
                    show(call['final_deferred_rejection_time_s'], 3)))
            for rejection in q['native_to_latest_rejection']:
                lines.append('  最新动态拒绝：%s；前看%s秒处碰撞，类型%s，候选年龄%s秒。' % (
                    rejection.get('branch'), show(rejection.get('wall_ahead'), 3),
                    rejection.get('collision_type'), show(rejection.get('candidate_age'), 3)))
            if q['first_feasible_geometry'] and q['selected_geometry']:
                first, chosen = q['first_feasible_geometry'], q['selected_geometry']
                lines.append('  同次首次可行→最终选择：原目标%s→%s；曲线运动时长%s→%s秒，长度%s→%s米，最大|y|%s→%s米。' % (
                    show(q['first_cost'], 3), show(q['best_cost'], 3),
                    show(first['duration_s'], 3), show(chosen['duration_s'], 3),
                    show(first['full_curve_path_length_m'], 3), show(chosen['full_curve_path_length_m'], 3),
                    show(first['maximum_absolute_world_y_m'], 3), show(chosen['maximum_absolute_world_y_m'], 3)))
            if call['pending_available']:
                lines.append('  pending原目标%s，只是待检查候选；是否原生提升以末次检查和最终选择为准。' % show(call['pending_cost'], 3))
            for phase in call['phase_alignment_events']:
                lines.append('  发布关联位置残差%s厘米、朝向%s度；常速检查时刻偏移%+.3f秒。' % (
                    show(100*phase['position_residual_m'], 2), show(math.degrees(phase['yaw_residual_rad']), 2),
                    phase['nominal_epoch_offset_s']))
            if call['selected_curve_actually_adopted']:
                motion = call['post_adoption_motion']
                lines.append('  请求→有效控制%s秒，发布→有效控制%s秒；采用后%s秒近停%s秒、倒退%s秒。' % (
                    show(call['request_to_validated_command_wall_s'], 3),
                    show(call['publish_to_validated_command_wall_s'], 3), show(motion['observed_window_s'], 3),
                    show(motion['non_goal_near_stop']['total_s'], 3), show(motion['reverse_translation_duration_s'], 3)))
        execution = row['execution_evidence']
        lines.append('有效控制与获准展开＋制动唯一匹配%d/%d条；非零且未经有效发布%d条。' % (
            execution['successful_exact_selected_and_brake_matches'], execution['validated_command_count'],
            execution['nonzero_without_validated_publication_count']))
    lines += ['', '统计口径',
        '1.2秒是内部求解预算；集中复核计入该预算，优化器后面的最终检查和下游最新验收仍单独耗时。请求总时长包含前端、后端及发布流程，不能混同计算预算。',
        '原目标下降只说明记录目标更低；路线长宽、运动时长和真正采用分别核实。pending未验收轨迹不计可行收益。',
        '位置/朝向残差是现有发布关联读数；采用后2秒近停和倒退是实际状态现象，单次实验不把它们直接归因某个代价项。',
        '近停沿用|v|<0.02米/秒、|角速度|<0.05弧度/秒、距目标>0.1米；倒退为v<-0.02米/秒。',
        '完整REMANI＋底盘MPPI，固定臂ROS数值闭环；非Gazebo、非实车。GIF画真实参考、获准MPPI展开、CV及67球投影，本次未记录搜索候选。', '']
    return '\n'.join(lines)


def chart(data, output):
    image = Image.new('RGB', (1180, 680), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    rows = data['rows']
    draw.text((20, 14), '首解后集中复核：交付与实际接续', font=title, fill='#333333')
    draw.text((20, 51), data['initial_pair_contract']['interpretation'], font=small, fill='#555555')
    calls = [r['deferred_quality']['first_actually_adopted'] for r in rows]
    metrics = [
        ('采用该请求的首解后复核 / ms', [c['post_first_native_wall_ms'] if c else None for c in calls]),
        ('首次反馈→首条有效新控制 / s', [
            r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows]),
        ('同次首解→选中时长缩短 / %', [100*(
            c['quality_checkpoint']['first_feasible_geometry']['duration_s']-
            c['adopted_reference_geometry']['duration_s'])/
            c['quality_checkpoint']['first_feasible_geometry']['duration_s'] if c else None for c in calls])]
    for index, (name, values) in enumerate(metrics):
        left, right = 55+index*390, 325+index*390
        upper = max([float(v) for v in values if v is not None]+[.1])*1.25
        draw.text((left-28, 98), name, font=small, fill='#333333')
        if index == 2:
            for group, (call, value) in enumerate(zip(calls, values)):
                y = 158+group*69
                if call is None:
                    label = 'AB'[group]+'：未实际采用'
                else:
                    first_t = call['quality_checkpoint']['first_feasible_geometry']['duration_s']
                    chosen_t = call['adopted_reference_geometry']['duration_s']
                    label = '%s：%s→%s s（%+.2f%%）' % (
                        'AB'[group], show(first_t, 2), show(chosen_t, 2), value)
                draw.text((left-27, y), label, font=small, fill=colors[group])
            continue
        for fraction in (0., .5, 1.):
            x = left+fraction*(right-left)
            draw.line((x, 134, x, 283), fill='#e5e5e5')
            draw.text((x-8, 290), show(upper*fraction, 2), font=small, fill='#777777')
        for group, value in enumerate(values):
            y = 151+group*69
            draw.text((left-27, y+7), 'AB'[group], font=text, fill=colors[group])
            if value is None:
                draw.text((left+5, y+8), '未实际采用', font=small, fill='#555555')
            else:
                x = left+float(value)/upper*(right-left)
                if value > 0.:
                    draw.rectangle((left, y, x, y+32), fill=colors[group])
                draw.text((x+5, y+7), show(value, 3), font=small, fill='#444444')
    draw.line((20, 329, 1160, 329), fill='#dddddd')
    for group, (row, call) in enumerate(zip(rows, calls)):
        y = 348+group*115
        chain = row['delivery_chain_counts']
        chain_text = '→'.join(str(chain[k]) for k in ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed'))
        draw.text((20, y), 'AB'[group]+'　请求→原生→发布→采用：'+chain_text+
            '；实走 %s m / %s s' % (show(row['actual_path_length_m'], 3), show(row['motion_duration_s'], 3)),
            font=text, fill=colors[group])
        if call:
            first = call['quality_checkpoint']['first_feasible_geometry']
            chosen = call['adopted_reference_geometry']
            draw.text((20, y+30), '同次首解→选中：运动时长 %s→%s s，长度 %s→%s m，最大|y| %s→%s m。' % (
                show(first['duration_s'], 3), show(chosen['duration_s'], 3),
                show(first['full_curve_path_length_m'], 3), show(chosen['full_curve_path_length_m'], 3),
                show(first['maximum_absolute_world_y_m'], 3), show(chosen['maximum_absolute_world_y_m'], 3)),
                font=small, fill='#555555')
            motion = call['post_adoption_motion']
            draw.text((20, y+57), '采用后 %s s：近停 %s s、倒退 %s s；请求/完整优化器/内部 %s/%s/%s ms。' % (
                show(motion['observed_window_s'], 2), show(motion['non_goal_near_stop']['total_s'], 3),
                show(motion['reverse_translation_duration_s'], 3), show(call['request_wall_ms'], 1),
                show(call['optimizer_total_wall_ms'], 1), show(call['internal_solver_wall_ms'], 1)),
                font=small, fill='#555555')
    draw.text((20, 596), 'B末次复核尝试/提升/退回首解：%d/%d/%d；待验收候选不计改善。' % (
        rows[1]['deferred_quality']['deferred_check_attempts'], rows[1]['deferred_quality']['deferred_promotions'],
        rows[1]['deferred_quality']['deferred_fallbacks']), font=small, fill='#666666')
    draw.text((20, 623), '内部求解预算1.2s；最终及最新验收另外计时。每组一次在线闭环，重规划输入并非完全相同。',
        font=small, fill='#666666')
    draw.text((20, 650), '第二栏包含首次失败及重试；第一、三栏对应首条采用请求，时长缩短以该次首解为基准。',
        font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    parser.add_argument('--skip-gif', action='store_true')
    args = parser.parse_args()
    pairs = [load_online(args.root/'headon_A', LABELS[0]), load_online(args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='deferred_quality_report_v1', rows=list(rows),
        initial_pair_contract=initial_pair_contract(rows),
        same_native_binaries=rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native'],
        same_mppi_library=rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi'],
        scope='One normal unpaused fixed-arm ROS numerical SIL pair. Same initial first-feasible policy, refresh ongoing native checks versus one budget-reserved final native check. Original final/latest gates retained.')
    data['conclusions'] = conclusions(data)
    print('DEFERRED_QUALITY_CORE', json.dumps(dict(conclusions=data['conclusions']), ensure_ascii=False), flush=True)
    output = args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    if args.skip_gif:
        previous = json.loads((output/'summary.json').read_text())
        artifact = dict(previous['artifacts']['headon'])
        if sha(output/'迎面对照.gif') != artifact['sha256'] or not (output/'迎面关键帧.png').is_file():
            raise ValueError('Existing GIF/keyframes missing or changed.')
        artifact['reused_without_rerender'] = True
        data['artifacts'] = dict(headon=artifact)
    else:
        data['artifacts'] = dict(headon=animation(rows, trials, output/'迎面对照.gif', args.max_frames))
        keyframes(output/'迎面对照.gif', output/'迎面关键帧.png')
    chart(data, output/'对照数据.png')
    (output/'实验结果.txt').write_text(narrative(data))
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('DEFERRED_QUALITY_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

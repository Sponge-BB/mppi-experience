#!/usr/bin/env python3
"""Report a completed online slow-delivery CV-window A/B pair.

Run only after timed ROS trials stop. Source domains come from the actual
slow_delivery_window events; the overlay capture still describes the live 3 s
source and is deliberately not substituted for the B planning-copy domain.
"""
import argparse
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
import yaml

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_deferred_quality import initial_pair_contract, load_online as load_deferred_online
from report_fast_refresh_feedback import animation
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window
from report_static_agility import show
from report_time_indexed_collision import TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE / 'results/delivery_window_20261101_01'
LABELS = ('A：慢层 3s；最新/MPPI 3s', 'B：慢层 4.5s；最新/MPPI 3s')
WINDOW_FIELDS = ('original_source_sequence', 'original_source_stamp',
    'original_source_refresh_epoch', 'original_max_prediction_s', 'original_end_epoch',
    'planning_max_prediction_s', 'planning_end_epoch', 'configured_delivery_allowance_s',
    'delivery_allowance_s', 'requested_epoch', 'optimizer_entry_epoch')


def one(rows):
    return rows[0] if len(rows) == 1 else None


def global_prediction_contract(trial):
    path = trial.reader.locate('parameters.yaml', required=True)
    with trial.reader.open(path) as stream:
        parameters = yaml.safe_load(stream)
    key = '/remani_planner_node/dynamic_point_flow/max_prediction_time'
    if key not in parameters:
        raise ValueError('missing actual planner prediction limit in parameters.yaml')
    planner_limit = float(parameters[key])
    shared = trial.reader.json('shared_collision_config.json', required=True)
    shared_limit = float(shared['max_prediction'])
    return dict(planner_parameter_key=key, latest_planner_max_prediction_s=planner_limit,
        mppi_shared_max_prediction_s=shared_limit,
        both_global_limits_are_three_seconds=(planner_limit == shared_limit == 3.),
        scope='Actual per-trial parameter dump and shared execution configuration, not the original deployment parameter template.')


def coverage_analysis(row, trial):
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    events = [r for r in timeline if r.get('event') == 'slow_delivery_window']
    if not events:
        raise ValueError('missing actual slow_delivery_window events')
    acceptances = [r for r in captures if r.get('stage') == 'acceptance']
    plans = {r['plan_ordinal']: r for r in captures if r.get('stage') == 'planning'}
    phases = [r for r in timeline if r.get('event') == 'candidate_phase_alignment']
    rejections = [r for r in timeline if r.get('event') == 'latest_prediction_reject_detail']
    contract = global_prediction_contract(trial)
    global_limit = contract['latest_planner_max_prediction_s']
    calls = row['deferred_quality']['calls']
    records = []
    for event in events:
        for field in WINDOW_FIELDS:
            if field not in event:
                raise ValueError('slow_delivery_window missing field: ' + field)
        request_id = event['request_id']
        matching_plans = [p for p in plans.values() if p.get('request_id') == request_id
            and p['overlay']['source_snapshot']['sequence'] == event['original_source_sequence']
            and abs(p['overlay']['source_snapshot']['stamp_nsec'] / 1e9 - event['original_source_stamp']) < 1e-7]
        plan = one(matching_plans)
        admissions = [a for a in acceptances if a.get('request_id') == request_id
                      and plan is not None and a.get('plan_ordinal') == plan['plan_ordinal']]
        linked_calls = [c for c in calls if c['request_id'] == request_id
                        and plan is not None and c['plan_ordinal'] == plan['plan_ordinal']]
        call = one(linked_calls)
        admission_records = []
        for admission in admissions:
            snapshot = admission['latest_snapshot']
            latest_stamp = snapshot['stamp_nsec'] / 1e9
            latest_end = latest_stamp + global_limit
            now = admission['validation_now_nsec'] / 1e9
            aligned = [p for p in phases if p.get('request_id') == request_id
                       and abs(p['validation_now'] - now) < 1e-7]
            phase = one(aligned)
            related_rejections = [r for r in rejections if r.get('request_id') == request_id
                and r['snapshot_sequence'] == snapshot['sequence']
                and abs(r['sim'] - now) < 1e-7]
            rejection_details = []
            for reject in related_rejections:
                # The slow native hard gate checks nominal scale only. The
                # original latest event keeps its actual scale and branch.
                backend_query = event['requested_epoch'] + reject['local_t']
                source_age = backend_query - event['original_source_stamp']
                rejection_details.append(dict(
                    logged={k: v for k, v in reject.items() if isinstance(v, (str, int, float, bool))},
                    actual_latest_source_sequence=snapshot['sequence'],
                    actual_latest_source_stamp=latest_stamp,
                    actual_latest_query_age_s=reject['query_stamp'] - latest_stamp,
                    inside_actual_latest_window=(-1e-6 <= reject['query_stamp'] - latest_stamp <= global_limit + 1e-6),
                    nominal_backend_query_epoch=backend_query,
                    nominal_backend_seconds_after_frozen_source=source_age,
                    inside_original_slow_window=(-1e-6 <= source_age <= event['original_max_prediction_s'] + 1e-6),
                    inside_actual_planning_window=(-1e-6 <= source_age <= event['planning_max_prediction_s'] + 1e-6),
                    scope='Clock/domain arithmetic at the actual rejection point; this does not recompute its collision geometry.'))
            phase_end = None
            if phase is not None and phase.get('new_phase_mapping'):
                scale = float(admission['resume_scale'])
                local_end = min(admission['candidate']['duration_s'],
                    phase['phase_local_t'] + max(0., latest_end - now) * scale)
                # Translate the latest nominal trajectory endpoint into the
                # slow nominal clock. The raw source endpoint difference below
                # is useful, but it omits this recorded phase association.
                required_backend_end = event['requested_epoch'] + local_end
                phase_end = dict(execution_scale=scale, latest_local_end_s=local_end,
                    nominal_backend_required_end_epoch=required_backend_end,
                    planning_margin_s=event['planning_end_epoch'] - required_backend_end,
                    original_margin_s=event['original_end_epoch'] - required_backend_end,
                    phase_local_t_s=phase['phase_local_t'],
                    nominal_epoch_offset_s=phase.get('nominal_epoch_offset_s'),
                    scope='Actual latest nominal trajectory endpoint mapped to the slow nominal clock; candidate end caps the interval. No new collision or phase rejection threshold.')
            admission_records.append(dict(plan_ordinal=admission['plan_ordinal'],
                latest_snapshot_sequence=snapshot['sequence'], latest_snapshot_valid=snapshot['valid'],
                latest_snapshot_stamp=latest_stamp, latest_snapshot_point_count=len(snapshot['points']),
                global_latest_max_prediction_s=global_limit, latest_end_epoch=latest_end,
                validation_now=now, latest_valid_remaining_s=latest_end - now,
                planning_end_minus_latest_end_s=event['planning_end_epoch'] - latest_end,
                original_end_minus_latest_end_s=event['original_end_epoch'] - latest_end,
                phase_alignment=phase, phase_corrected_nominal_coverage=phase_end,
                rejections=rejection_details,
                scope='Actual admission snapshot, not a guessed elapsed duration or the old overlay prediction limit.'))
        records.append(dict(request_id=request_id, plan_ordinal=plan['plan_ordinal'] if plan else None,
            planning_record_match_count=len(matching_plans), event=event,
            admissions=admission_records,
            native_final_success=call['native_final_success'] if call else None,
            reference_publications=call['reference_publications'] if call else None,
            consumed_revisions=call['consumed_revisions'] if call else None,
            actually_adopted=call['selected_curve_actually_adopted'] if call else None,
            domain_fields_from='actual slow_delivery_window event',
            overlay_limit_used_as_planning_limit=False))
    refresh = sorted([r for r in records if r['request_id'] > 1], key=lambda r: r['event']['mono'])
    first_refresh = refresh[0] if refresh else None
    first_admission = next((r for r in refresh if r['admissions']), None)
    first_adopted = next((r for r in refresh if r['actually_adopted']), None)
    return dict(global_contract=contract, events=records, refresh_events=refresh,
        first_refresh_window=first_refresh, first_refresh_with_latest_admission=first_admission,
        first_actually_adopted_window=first_adopted,
        actual_extension_active_request_ids=[r['request_id'] for r in refresh if r['event'].get('enabled') is True],
        initial_plan_extension_active=any(r['request_id'] == 1 and r['event'].get('enabled') for r in records),
        actual_refresh_allowances_s=sorted(set(r['event']['delivery_allowance_s'] for r in refresh)),
        all_actual_sources_match_capture=all(r['planning_record_match_count'] == 1 for r in records),
        scope='Saved source states/epochs stay fixed within a solve. B adds an explicit slow-copy CV continuation; global/latest/MPPI source limits are separately read from actual trial configuration.')


def load_online(source, label):
    row, trial = load_deferred_online(source, label)
    row['delivery_window'] = coverage_analysis(row, trial)
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def replay_summary(root):
    path = root / 'replay/旧候选覆盖核对.json'
    if not path.is_file():
        return None
    report = json.loads(path.read_text())
    point = report['rejected_candidate_point']
    scan = report['newly_covered_window']['counterfactual']
    return dict(path=str(path.resolve()), sha256=sha(path),
        original_input_hash=report['original_input_hash'],
        original_files_unchanged=report['original_files_unchanged'],
        original_backend_query_valid=point['original_backend']['query_valid'],
        counterfactual_backend_query_valid=point['counterfactual_backend']['query_valid'],
        counterfactual_backend_minimum_model_gap_m=point['counterfactual_backend']['minimum_model_gap_m'],
        original_latest_minimum_model_gap_m=point['original_latest_publication']['minimum_model_gap_m'],
        newly_covered_samples=scan['sample_count'], newly_covered_collision_samples=scan['collision_sample_count'],
        first_collision=scan['first_collision'],
        scope=report['scope'])


def first_admission(row):
    window = row['delivery_window']['first_refresh_with_latest_admission']
    return (window, window['admissions'][0]) if window else (None, None)


def conclusions(data):
    rows = data['rows']
    notes = [data['initial_pair_contract']['interpretation']]
    limits = [r['delivery_window']['global_contract']['both_global_limits_are_three_seconds'] for r in rows]
    notes.append('两组最新验收和 MPPI 的实际全局预测上限%s保持 3 秒。' % ('均' if all(limits) else '未能全部确认'))
    for row in rows:
        window = row['delivery_window']
        notes.append(row['label'] + '：' + ('到达目标' if row['reached_goal'] else
            TERMINATIONS.get(row['termination'], str(row['termination']))) + '，' +
            ('发现' if row['physical_sample_collision'] else '未发现') + '物理球采样接触。')
        notes.append('  首次避障反馈→首条实际有效新控制 %s 秒，包含前面的失败和重试；实走 %s 米、起步到结束 %s 秒。' % (
            show(row['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'], 3),
            show(row['actual_path_length_m'], 3), show(row['motion_duration_s'], 3)))
        record, admission = first_admission(row)
        if admission:
            notes.append('  第一次进入最新验收的重规划为 q%d：规划窗口相对最新窗口的末端余量 %+.3f 秒，原 3 秒窗口则为 %+.3f 秒。' % (
                record['request_id'], admission['planning_end_minus_latest_end_s'],
                admission['original_end_minus_latest_end_s']))
            corrected = admission['phase_corrected_nominal_coverage']
            if corrected:
                notes.append('  再按真实候选相位换算到慢层时钟，覆盖余量 %+.3f 秒；这是检查时域的覆盖情况，不等于候选已通过碰撞检查。' % corrected['planning_margin_s'])
        first_refresh = window['first_refresh_window']
        if first_refresh and not first_refresh['admissions']:
            notes.append('  最早 q%d 尚未进入最新验收，原生最终通过为 %s；其覆盖余量不补造。' % (
                first_refresh['request_id'], first_refresh['native_final_success']))
        chain = row['delivery_chain_counts']
        notes.append('  请求→原生通过→发布→采用：%d→%d→%d→%d，最新动态拒绝 %d 次。' % (
            chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
            chain['refresh_reference_publications'], chain['refresh_references_consumed'], len(row['latest_rejections'])))
        adopted = row['deferred_quality']['first_actually_adopted']
        if adopted:
            q = adopted['quality_checkpoint']
            first, selected = q['first_feasible_geometry'], q['selected_geometry']
            notes.append('  首条真正采用的候选来自 q%d，同次首解→选中：运动时长 %s→%s 秒，长度 %s→%s 米，最大 |y| %s→%s 米。' % (
                adopted['request_id'], show(first['duration_s'], 3), show(selected['duration_s'], 3),
                show(first['full_curve_path_length_m'], 3), show(selected['full_curve_path_length_m'], 3),
                show(first['maximum_absolute_world_y_m'], 3), show(selected['maximum_absolute_world_y_m'], 3)))
        else:
            notes.append('  尚无候选核实为最终通过并实际产生新控制，不能计作交付改善。')
    a_wait, b_wait = [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows]
    if a_wait is not None and b_wait is not None:
        notes.append('本次首次实际交付，B 比 A %s %.3f 秒。' % (
            '缩短' if b_wait < a_wait else '增加', abs(a_wait - b_wait)))
    b = rows[1]
    notes.append('B 实际启用延长窗口的请求：%s；首次规划是否误启用：%s。' % (
        b['delivery_window']['actual_extension_active_request_ids'], b['delivery_window']['initial_plan_extension_active']))
    if b['latest_rejections']:
        notes.append('B 仍有最新动态拒绝，需逐条区分“时域仍未覆盖”和“已覆盖但轨迹仍存在预测碰撞”，不能直接宣布交付缺口解决。')
    elif b['deferred_quality']['first_actually_adopted']:
        notes.append('B 本次交付未出现最新动态拒绝；结合实际窗口和采用链，可判断本场景机制是否生效，不能外推总体成功率。')
    notes.append('首次参考相同不意味着重规划输入完全相同：机器人位置、触发时刻和预测源均按各自在线记录统计。')
    return notes


def narrative(data):
    lines = ['慢层覆盖交付后近期执行时段：实验结果', '',
        '当前关键任务：减少“慢层检查时可行，交付时却因预测碰撞被拒”的轨迹，推动 REMANI 与 MPPI 的实际接续。',
        'A 慢层仍用冻结源 3 秒；B 仅后续重规划把冻结副本按 CV 继续外推 1.5 秒，慢层软代价和原生硬检查共同使用 4.5 秒窗口。',
        '两组均保存首解并在原 1.2 秒预算内末尾集中复核；首次空旷规划策略相同。', '',
        '主要结果'] + data['conclusions']
    replay = data.get('old_candidate_replay')
    if replay:
        lines += ['', '同旧输入核查',
            '旧 q2 同一候选、同冻结源：原 3 秒查询无效；明确假设外推 4.5 秒后间隙 %.2f cm。新增时段 %d 个采样点中 %d 个模型相交。' % (
                100 * replay['counterfactual_backend_minimum_model_gap_m'],
                replay['newly_covered_samples'], replay['newly_covered_collision_samples']),
            '该核查说明延长窗口能看到旧候选风险；在线实验用于检验能否真正优化并交付新轨迹。']
    for row in data['rows']:
        lines += ['', row['label'],
            'MPPI 完整循环 P95 / 最大 %s / %s 毫秒；最小物理球采样间隙 %s cm。' % (
                show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1),
                show(100 * row['minimum_physical_sample_gap_m'], 3)),
            '原生失败原因计数：%s。' % json.dumps(row['failure_reason_counts'], ensure_ascii=False)]
        for record in row['delivery_window']['refresh_events']:
            event = record['event']
            lines.append('q%d：冻结源 seq%d，原时域 %.1f 秒→实际慢层 %.1f 秒；原生最终通过 %s，发布 %s，实际采用 %s。' % (
                record['request_id'], event['original_source_sequence'], event['original_max_prediction_s'],
                event['planning_max_prediction_s'], record['native_final_success'],
                record['reference_publications'], record['actually_adopted']))
            lines.append('  原源 stamp %.9f，refresh %.9f，请求启用 %.9f，优化入口 %.9f；原末端 %.9f，实际慢层末端 %.9f。' % (
                event['original_source_stamp'], event['original_source_refresh_epoch'], event['requested_epoch'],
                event['optimizer_entry_epoch'], event['original_end_epoch'], event['planning_end_epoch']))
            for admission in record['admissions']:
                lines.append('  最新源 seq%d / stamp %.9f / 末端 %.9f；实际慢层末端余量 %+.3f 秒，原窗口余量 %+.3f 秒。' % (
                    admission['latest_snapshot_sequence'], admission['latest_snapshot_stamp'],
                    admission['latest_end_epoch'], admission['planning_end_minus_latest_end_s'],
                    admission['original_end_minus_latest_end_s']))
                corrected = admission['phase_corrected_nominal_coverage']
                if corrected:
                    lines.append('  按实际相位校正，常速近期执行段覆盖余量 %+.3f 秒；位置残差 %s cm，朝向残差 %s 度。' % (
                        corrected['planning_margin_s'],
                        show(100 * admission['phase_alignment']['position_residual_m'], 2),
                        show(math.degrees(admission['phase_alignment']['yaw_residual_rad']), 2)))
                for reject in admission['rejections']:
                    logged = reject['logged']
                    lines.append('  最新拒绝 %s：轨迹本地 %.6f 秒，前看 %.3f 秒；该点在原慢层窗口内 %s，在本组实际慢层窗口内 %s。' % (
                        logged['branch'], logged['local_t'], logged['wall_ahead'],
                        reject['inside_original_slow_window'], reject['inside_actual_planning_window']))
        for call in row['deferred_quality']['calls']:
            q = call['quality_checkpoint']
            lines.append('q%d：请求 / 完整优化器 / 内部求解 %s / %s / %s ms；末次候选提升 %s，回退首解 %s。' % (
                call['request_id'], show(call['request_wall_ms'], 1), show(call['optimizer_total_wall_ms'], 1),
                show(call['internal_solver_wall_ms'], 1), call['final_deferred_candidate_promoted'], call['returned_first_feasible']))
            if q['first_feasible_geometry'] and q['selected_geometry']:
                first, chosen = q['first_feasible_geometry'], q['selected_geometry']
                lines.append('  同次首解→选中：成本 %s→%s，时长 %s→%s 秒，长度 %s→%s 米；实际采用 %s。' % (
                    show(q['first_cost'], 3), show(q['best_cost'], 3), show(first['duration_s'], 3), show(chosen['duration_s'], 3),
                    show(first['full_curve_path_length_m'], 3), show(chosen['full_curve_path_length_m'], 3),
                    call['selected_curve_actually_adopted']))
            if call['selected_curve_actually_adopted']:
                motion = call['post_adoption_motion']
                lines.append('  采用后 %.3f 秒实测：近停 %s 秒、倒退 %s 秒、移动 %s 米。' % (
                    motion['observed_window_s'], show(motion['non_goal_near_stop']['total_s'], 3),
                    show(motion['reverse_translation_duration_s'], 3), show(motion['sampled_polyline_path_length_m'], 3)))
        execution = row['execution_evidence']
        lines.append('有效控制与获准展开及制动唯一匹配 %d/%d 条；非零但未经有效发布 %d 条。' % (
            execution['successful_exact_selected_and_brake_matches'], execution['validated_command_count'],
            execution['nonzero_without_validated_publication_count']))
    lines += ['', '口径',
        '4.5 秒是慢层冻结副本的恒速外推假设，原记录只配置 3 秒；它不代表新增区间已经获得实测预测准确性保证。',
        '窗口端点余量按实际 slow_delivery_window 事件和验收时最新源计算；相位校正再将最新近期执行端点换算到慢层时钟。余量为正表示时间覆盖，不等于无碰撞。',
        '1.2 秒是内部求解预算；最终原生检查和下游最新验收另计时间。首条反馈到实际新控制包括失败请求及重试，单次成功请求耗时只作辅助。',
        '原生可行、最新发布、MPPI 收到参考和真正产生有效控制逐层核实；未发布、未采用的选中候选不算交付收益。',
        '完整 REMANI＋底盘 MPPI、固定机械臂的 ROS 数值闭环；非 Gazebo、非实车。每组一次迎面场景，用于验证机制。',
        'GIF 使用真实 REMANI 参考、获准 MPPI 展开、CV 和 67 球投影；本轮未记录 MPPI 搜索候选。', '']
    return '\n'.join(lines)


def chart(data, output):
    image = Image.new('RGB', (1180, 690), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    rows = data['rows']
    draw.text((20, 14), '交付窗口覆盖：能否减少候选交付时失效', font=title, fill='#333333')
    draw.text((20, 51), data['initial_pair_contract']['interpretation'], font=small, fill='#555555')
    admissions = [first_admission(row) for row in rows]
    metrics = [
        ('首轮最新验收：窗口末端余量 / s', [a['planning_end_minus_latest_end_s'] if a else None for _, a in admissions]),
        ('首次反馈→首条有效新控制 / s', [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows]),
        ('起步到结束：实际运动时间 / s', [r['motion_duration_s'] for r in rows])]
    for column, (name, values) in enumerate(metrics):
        left, right = 55 + column * 390, 325 + column * 390
        draw.text((left - 28, 98), name, font=small, fill='#333333')
        finite = [float(v) for v in values if v is not None]
        lower = min([0.] + finite) * 1.25
        upper = max([.1] + finite) * 1.25
        def coord(value):
            return left + (value - lower) / (upper - lower) * (right - left)
        zero = coord(0.)
        for value in (lower, .5 * (lower + upper), upper):
            x = coord(value)
            draw.line((x, 136, x, 280), fill='#e5e5e5')
            draw.text((x - 8, 289), show(value, 2), font=small, fill='#777777')
        draw.line((zero, 134, zero, 282), fill='#999999')
        for group, value in enumerate(values):
            y = 151 + group * 69
            draw.text((left - 27, y + 7), 'AB'[group], font=text, fill=colors[group])
            if value is None:
                draw.text((left + 5, y + 8), '未进入验收' if column == 0 else '未交付', font=small, fill='#555555')
            else:
                x = coord(float(value))
                draw.rectangle((min(zero, x), y, max(zero, x), y + 32), fill=colors[group])
                label_x = max(left + 3, min(right - 55, x + 5))
                draw.text((label_x, y + 7), show(value, 3), font=small, fill='#333333')
    draw.line((20, 329, 1160, 329), fill='#dddddd')
    for group, row in enumerate(rows):
        y = 345 + group * 119
        chain = row['delivery_chain_counts']
        numbers = '→'.join(str(chain[k]) for k in ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed'))
        draw.text((20, y), 'AB'[group] + '　请求→原生→发布→采用：' + numbers +
            '；最新拒绝 %d 次；实走 %s m' % (len(row['latest_rejections']), show(row['actual_path_length_m'], 3)),
            font=text, fill=colors[group])
        record, admission = admissions[group]
        if admission:
            corrected = admission['phase_corrected_nominal_coverage']
            draw.text((20, y + 30), '首轮最新验收 q%d：原窗口余量 %+.3f s，实际余量 %+.3f s；相位校正后 %s s。' % (
                record['request_id'], admission['original_end_minus_latest_end_s'],
                admission['planning_end_minus_latest_end_s'],
                show(corrected['planning_margin_s'], 3) if corrected else '未关联'),
                font=small, fill='#555555')
        adopted = row['deferred_quality']['first_actually_adopted']
        if adopted:
            first = adopted['quality_checkpoint']['first_feasible_geometry']
            selected = adopted['adopted_reference_geometry']
            motion = adopted['post_adoption_motion']
            draw.text((20, y + 58), '首条采用 q%d：同次时长 %s→%s s；接续后近停 %s s、倒退 %s s。' % (
                adopted['request_id'], show(first['duration_s'], 2), show(selected['duration_s'], 2),
                show(motion['non_goal_near_stop']['total_s'], 3), show(motion['reverse_translation_duration_s'], 3)),
                font=small, fill='#555555')
        draw.text((20, y + 84), 'MPPI 完整循环 P95 / 最大 %s / %s ms；%s物理球采样接触。' % (
            show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1),
            '发现' if row['physical_sample_collision'] else '未发现'), font=small, fill='#555555')
    draw.text((20, 599), 'A 慢层 3s；B 仅重规划慢层 4.5s（恒速外推假设）；两组全局最新验收与 MPPI 仍为 3s。',
        font=small, fill='#666666')
    draw.text((20, 627), '第一栏为首次进入最新验收的请求，可能已含重试；第二栏计入首次失败和重试。',
        font=small, fill='#666666')
    draw.text((20, 655), '每组一次真实在线闭环，重规划输入并非完全相同。未发布、未采用的候选不计交付收益。',
        font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    parser.add_argument('--skip-gif', action='store_true')
    args = parser.parse_args()
    pairs = [load_online(args.root / 'headon_A', LABELS[0]), load_online(args.root / 'headon_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='delivery_window_report_v1', rows=list(rows),
        initial_pair_contract=initial_pair_contract(rows), old_candidate_replay=replay_summary(args.root),
        same_native_binaries=rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native'],
        same_mppi_library=rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi'],
        scope='One unpaused online numerical fixed-arm ROS pair. Both use deferred quality checks; only B refresh extends the slow frozen-copy CV domain. Actual source clocks, native/latest gates, reference consumption and validated commands are retained.')
    data['conclusions'] = conclusions(data)
    print('DELIVERY_WINDOW_CORE', json.dumps(dict(conclusions=data['conclusions']), ensure_ascii=False), flush=True)
    output = args.root / 'report'
    output.mkdir(parents=True, exist_ok=True)
    if args.skip_gif:
        previous = json.loads((output / 'summary.json').read_text())
        artifact = dict(previous['artifacts']['headon'])
        if sha(output / '迎面对照.gif') != artifact['sha256'] or not (output / '迎面关键帧.png').is_file():
            raise ValueError('Existing GIF/keyframes missing or changed.')
        artifact['reused_without_rerender'] = True
        data['artifacts'] = dict(headon=artifact)
    else:
        data['artifacts'] = dict(headon=animation(rows, trials, output / '迎面对照.gif', args.max_frames))
        data['artifacts']['headon']['cv_visualization_semantics'] = (
            'Existing recorded execution-side CV visualization; latest and MPPI remain at 3 s. '
            'The slow-copy 4.5 s continuation is not fabricated as an observed CV line.')
        keyframes(output / '迎面对照.gif', output / '迎面关键帧.png')
    chart(data, output / '对照数据.png')
    text = narrative(data)
    (output / '实验结果.txt').write_text(text)
    (output / '本轮结论.txt').write_text('\n'.join(data['conclusions']) + '\n')
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    (output / 'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print('DELIVERY_WINDOW_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

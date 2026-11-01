#!/usr/bin/env python3
"""Read completed slow-reference admission A/B and the retained MPPI gates.

No ROS, solver or controller is run. Same-polynomial counterfactuals cover only
the frozen dynamic branch; complete native acceptance is taken from the actual
online record, never inferred from the Python dynamic check.
"""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_candidate_collision_repair import dynamic_check
from report_frontend_rejoin import keyframes
from report_static_agility import low_speed, show
from report_time_indexed_collision import TERMINATIONS
from report_window_continuity import animation, online
from replay_temporal_collision import Candidate, digest


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/half_speed_contract_20261101_01'
LABELS = ('A：常速＋半速验收', 'B：常速验收')


def delay(row):
    value = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
    return value/1000. if value is not None else None


def command_key(record, cycle=False):
    """Exact compact publication fields, not nearest-time state matching."""
    command = (record.get('command_v'), record.get('command_w')) if cycle else record.get('command', [])
    if len(command) != 2:
        return None
    # local_execution writes execution.t from ``published`` (the validation /
    # command-header stamp), while command_ros_time is the later send clock.
    # The explicitly recorded header stamp is the matching field; no nearest
    # publication-time substitution is permitted.
    return (record['command_header_ros_time']-1000. if cycle else record['t'],
            record.get('guide_revision'), record.get('reason'), *command)


def execution_evidence(trial, row):
    begin, end = row['metric_window_start_t'], row['end_t']
    commands = [c for c in trial.reader.jsonl('state_mppi_cycles.jsonl')
                if begin <= c['command_ros_time']-1000. <= end]
    executions = trial.reader.jsonl('execution_rollouts.jsonl')
    by_key, cycle_keys = defaultdict(list), Counter(command_key(c, True) for c in commands)
    for execution in executions:
        by_key[command_key(execution)].append(execution)
    validated = [c for c in commands if c.get('validated_command_published') is True]
    matched, unresolved, inconsistent = [], [], []
    for cycle in validated:
        key = command_key(cycle, True)
        records = by_key.get(key, [])
        if len(records) != 1 or cycle_keys[key] != 1:
            unresolved.append(dict(cycle_id=cycle['cycle_id'], matching_execution_count=len(records),
                                   matching_cycle_count=cycle_keys[key]))
            continue
        execution = records[0]
        report = execution.get('validation', [])
        passed = (cycle.get('validation_invoked') is True and cycle.get('validation_code') == 0
                  and cycle.get('reason') == 'ok' and bool(execution.get('rollout'))
                  and len(report) == 6 and report[0] == 1 and report[5] == 1
                  and report[3] == len(execution['rollout']))
        item = dict(cycle_id=cycle['cycle_id'], guide_revision=cycle.get('guide_revision'),
            elapsed_s=execution['t'], command=execution['command'],
            selected_and_brake_approved=passed,
            selected_model_minimum_gap_m=report[1] if len(report) == 6 else None,
            brake_model_minimum_gap_m=report[2] if len(report) == 6 else None,
            checked_horizon_s=execution['rollout'][-1][0] if execution.get('rollout') else None,
            command_lease_s=execution.get('lease_s'))
        (matched if passed else inconsistent).append(item)
    nonzero = [c for c in commands if abs(c.get('command_v', 0.))+abs(c.get('command_w', 0.)) > 1e-9]
    zeros = [c for c in commands if abs(c.get('command_v', 0.))+abs(c.get('command_w', 0.)) <= 1e-9]
    return dict(cycle_count=len(commands), validated_command_count=len(validated),
        nonzero_command_count=len(nonzero), zero_command_count=len(zeros),
        nonzero_without_validated_publication_count=sum(c.get('validated_command_published') is not True for c in nonzero),
        validation_code_counts=dict(Counter(str(c.get('validation_code')) for c in commands)),
        validation_attempt_code_counts=dict(Counter(str(a.get('code')) for c in commands for a in c.get('validation_attempts', []))),
        zero_reason_counts=dict(Counter(c.get('reason') for c in zeros)),
        successful_exact_selected_and_brake_matches=len(matched),
        exact_match_unresolved=unresolved, matched_but_not_approved=inconsistent,
        all_validated_publications_have_matching_selected_and_brake_approval=(
            len(validated) > 0 and len(matched) == len(validated)),
        matching_fields='exact command_header_ros_time minus 1000 equals execution.t, guide revision, reason, v, w; unique records only. command_ros_time is the later publication clock, not execution.t.',
        minimum_selected_model_gap_m=min((r['selected_model_minimum_gap_m'] for r in matched), default=None),
        minimum_brake_model_gap_m=min((r['brake_model_minimum_gap_m'] for r in matched), default=None),
        checked_horizon_s=sorted(set(r['checked_horizon_s'] for r in matched)),
        command_leases_s=sorted(set(r['command_lease_s'] for r in matched)),
        matched_cycles=matched,
        brake_check_scope='First action for the command lease, then brake until measured-model |v|<.01, |w|<.03 is held for .10s (loop capped at 2s and CV validity); approval does not check remaining prediction time after stopping or guarantee that an approaching obstacle will not later reach the stopped robot.',
        scope='Actual compact publication records plus retained selected/brake validation. No exact state replay or future execution commitment is inferred.')


def load_online(source, label):
    row, trial = online(source, label)
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    plans = {r['plan_ordinal']: r for r in captures if r.get('stage') == 'planning'}
    publications = [r for r in timeline if r.get('event') == 'reference_publish']
    pairs = []
    for final in captures:
        if final.get('stage') != 'final_candidate' or final.get('request_id', 0) <= 1:
            continue
        plan = plans[final['plan_ordinal']]
        candidate = Candidate(final['candidate'], plan)
        checks = [dynamic_check(candidate, plan, scale) for scale in (1., .5)]
        source = plan['overlay'].get('source_snapshot')
        final_source = final.get('final_native_overlay', {}).get('source_snapshot')
        same_source = digest(source) == digest(final_source) if source is not None and final_source is not None else None
        full, half = checks
        full_pass = bool(full.get('available') and full.get('valid_sample_count', 0) > 0
                         and full.get('first_collision') is None)
        half_hit = bool(half.get('available') and half.get('first_collision') is not None)
        publish = [r for r in publications if r['request_id'] == final['request_id']]
        consumed = [r for r in trial.refs if any(abs(r['stamp']-p['guide_start']) < 1e-7 for p in publish)]
        pairs.append(dict(request_id=final['request_id'], plan_ordinal=final['plan_ordinal'],
            recorded_native_pass=final.get('native_success'), native_reason=final.get('native_reason'),
            geometry_sha256=candidate.geometry_hash(), same_frozen_native_source=same_source,
            nominal_dynamic_pass=full_pass, half_dynamic_collision=half_hit,
            old_half_branch_would_reject=(half_hit if same_source is True else None),
            native_pass_but_old_half_branch_would_reject=(final.get('native_success') is True
                and same_source is True and half_hit),
            reference_publications=len(publish), consumed_revisions=[r['revision'] for r in consumed],
            checks=checks,
            scope='Same final polynomial and frozen CV, dynamic branch only. Other native constraints are evidenced by recorded native_pass, not replayed here. Latest CV and actual MPPI execution are separate checks.'))
    row['final_candidate_counterfactuals'] = pairs
    row['slow_contract_events'] = [r for r in timeline if r.get('event') == 'slow_reference_latest_contract']
    row['repair_execution_events'] = [r for r in timeline if r.get('event') in
        ('early_collision_repair_yield', 'candidate_collision_repair_begin')]
    row['execution_evidence'] = execution_evidence(trial, row)
    observed = trial.result.get('native_realtime_evidence', {}).get('first_observed_moving_t')
    triggers = [r for r in timeline if r.get('event') == 'refresh_trigger']
    requests = [r for r in timeline if r.get('event') == 'request_begin' and r.get('request_id', 0) > 1]
    refresh_publications = [r for r in publications if r.get('request_id', 0) > 1]
    new_valid = [r for r in trial.reader.jsonl('state_mppi_cycles.jsonl')
                 if (r.get('guide_revision') or 0) > 1 and r.get('validated_command_published')]
    first_trigger = min(triggers, key=lambda r: r['mono']) if triggers else None
    first_request = min(requests, key=lambda r: r['mono']) if requests else None
    first_publish = min(refresh_publications, key=lambda r: r['mono']) if refresh_publications else None
    first_valid = min(new_valid, key=lambda r: r['command_publish_after_monotonic']) if new_valid else None
    row['stage_timeline'] = dict(first_observed_moving_elapsed_s=observed,
        first_blocked_trigger_elapsed_s=first_trigger['sim']-1000. if first_trigger else None,
        first_refresh_request_elapsed_s=first_request['sim']-1000. if first_request else None,
        first_refresh_reference_publish_elapsed_s=first_publish['sim']-1000. if first_publish else None,
        first_new_validated_command_elapsed_s=first_valid['command_publish_after_ros_time']-1000. if first_valid else None,
        observation_to_first_request_elapsed_s=first_request['sim']-1000.-observed
            if first_request and observed is not None else None,
        request_to_reference_publish_wall_s=first_publish['mono']-first_request['mono']
            if first_publish and first_request else None,
        reference_publish_to_new_validated_command_wall_s=first_valid['command_publish_after_monotonic']-first_publish['mono']
            if first_valid and first_publish else None,
        trigger_rule='Refresh when the current guide future 2-second geometric check is blocked. Observation of motion alone does not request immediate REMANI refresh; all preceding elapsed time is not labeled system latency.')
    row['contract_settings'] = trial.manifest.get('half_speed_contract', {})
    row['native_binaries_sha256'] = trial.manifest.get('native_binaries_sha256')
    row['mppi_library_sha256'] = trial.manifest.get('mppi_library_sha256')
    validator = Path(trial.manifest['build'])/'source/online_api.cpp'
    row['execution_validator_source'] = dict(path=str(validator),
        sha256=sha(validator) if validator.is_file() else None,
        validation_fields=['valid', 'selected_min_gap', 'brake_min_gap', 'rows', 'restoration', 'brake_valid'])
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def history_summary(path):
    if not path.is_file():
        return dict(available=False, path=str(path))
    data = json.loads(path.read_text())
    trials = []
    for trial in data.get('trials', []):
        failures = trial['failed_candidates']
        trials.append(dict(source=trial['source'], actual_contract=trial['actual_contract'],
            failure_count=len(failures),
            nominal_pass_half_fail_count=sum(r['nominal_dynamic_pass'] and not r['half_dynamic_pass'] for r in failures),
            unexecuted_failure_count=sum(r['failed_candidate_was_executed'] is False for r in failures),
            consumed_reference_phase_evidence=[{k: v for k, v in r.items() if k != 'projection_rows'}
                for r in trial['consumed_reference_phase_evidence']]))
    return dict(available=True, path=str(path.resolve()), sha256=sha(path), trials=trials,
        scope='Historical failed candidates were not executed; actual motion belongs to consumed references. Dynamic-only retrospective evidence is not complete native safety acceptance.')


def add_common_motion_window(rows, trials):
    """Clip to equal elapsed time since actual motion; retain both full outcomes."""
    from report_long_goal_crossing import settings
    duration = min(t.end-t.anchor for t in trials)
    for row, trial in zip(rows, trials):
        start, end = trial.anchor, trial.anchor+duration
        _, goal, _ = settings(trial.manifest)
        length = 0.
        for a, b in zip(trial.state, trial.state[1:]):
            dt = float(b['t']-a['t'])
            overlap = max(0., min(end, float(b['t']))-max(start, float(a['t'])))
            if dt > 0. and overlap > 0.:
                length += math.hypot(float(b['x']-a['x']), float(b['y']-a['y']))*overlap/dt
        row['common_motion_window'] = dict(start_elapsed_s=start, end_elapsed_s=end,
            duration_s=duration, non_goal_low_speed=low_speed(trial.state, start, end, goal),
            sampled_polyline_path_length_m=length,
            definition='Same duration from each actual motion anchor, ending at the earlier trial termination; left-sample measured low speed and clipped recorded state polyline.')


def conclusions(data):
    rows = data['rows']
    a, b = rows
    hits = [r for r in b['final_candidate_counterfactuals'] if r['native_pass_but_old_half_branch_would_reject']]
    notes = []
    if hits:
        notes.append('B有%d条候选已通过本轮完整慢层验收，但同候选按旧半速动态分支会被拒绝；半速要求确实增加了候选验收门槛。' % len(hits))
    else:
        notes.append('本轮未记录到“完整慢层验收已通过、仅旧半速分支会拒绝”的B候选，尚未直接检验到这一放行机制。')
    da, db = delay(a), delay(b)
    if da is not None and db is not None:
        notes.append('首次请求到有效新控制：A %.3f秒，B %.3f秒，差值%+.3f秒；起算点是规划请求，不是感知到障碍。' % (da, db, db-da))
    else:
        notes.append('首次请求到有效新控制：A %s，B %s；起算点是规划请求，不是感知到障碍。' % tuple('%.3f秒' % v if v is not None else '未交付' for v in (da, db)))
    if b['physical_sample_collision']:
        notes.append('B发生物理球采样接触；交付门槛调整尚不能作为可采用方案。')
    elif b['reached_goal']:
        notes.append('B本次到达目标，未发现物理球采样接触；这只支持本场景可运行，不代表总体安全或避障算法更强。')
    else:
        notes.append('B未到达目标，结束原因为'+TERMINATIONS.get(b['termination'], str(b['termination']))+'。')
    if not b['execution_evidence']['all_validated_publications_have_matching_selected_and_brake_approval']:
        notes.append('B的有效控制与实际展开/制动验收记录未能全部唯一核对，不能声称执行验证证据完整。')
    if a['physical_sample_collision']:
        notes.append('A因物理球采样接触提前结束；全程路径和停留时长不能直接与B横比，下方另列共同运动时间窗。')
    notes.append('A/B首次触发时刻与绕行方向不同，在线输入并不相同；A碰撞、B到达不能全部归因于半速开关。')
    if any(r['online_pause_audit']['diagnostic_pause_detected'] for r in rows):
        notes.append('检测到诊断暂停，在线性能比较受干扰。')
    notes.append('本轮检验快慢层对参考的验收要求是否匹配；放宽参考入口不等于完成双时间尺度耦合优化。')
    return notes


def narrative(data):
    lines = ['慢层半速要求与MPPI实际执行：实验结果', '',
        '核心问题：MPPI反复重新选择局部动作，慢层却要求候选整段半速执行也安全；这个附加要求是否造成无必要的交付拒绝？',
        'A保留慢层常速＋半速硬验收；B取消慢层半速硬拒绝。两组保留常速动态、原生静态/自碰撞/轮约束、最新CV当前位置与常速检查，以及MPPI实际展开和制动检查。',
        '动态软目标仍含常速和半速固定窗积分；两组提前修复和加权修复均关闭。完整固定臂REMANI＋MPPI ROS数值闭环，迎面场景各一次，不暂停。', '',
        '结论']+data['conclusions']+['', '正常在线结果']
    for row in data['rows']:
        chain, execution = row['delivery_chain_counts'], row['execution_evidence']
        status = '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination']))
        lines += [row['label']+'：'+status+'；首次请求→有效新控制'+(show(delay(row), 3)+'秒。' if delay(row) is not None else '未交付。'),
            '请求→原生验收通过→最新CV发布→实际采用：%d→%d→%d→%d；最新CV拒绝%d次。' % (
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed'], len(row['latest_rejections'])),
            '各自结束前：实走%s米，最小物理采样间隙%s厘米，%s接触；非目标低速%s秒；MPPI整循环P95/最大%s/%s毫秒。' % (
                show(row['actual_path_length_m'], 3), show(100*row['minimum_physical_sample_gap_m'], 2),
                '发现' if row['physical_sample_collision'] else '未发现', show(row['non_goal_low_speed']['total_s'], 3),
                show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1)),
            '有效命令%d条，其中%d条唯一匹配实际展开＋制动批准；未匹配%d条，不符合%d条；非零但未经有效发布%d条。' % (
                execution['validated_command_count'], execution['successful_exact_selected_and_brake_matches'],
                len(execution['exact_match_unresolved']), len(execution['matched_but_not_approved']),
                execution['nonzero_without_validated_publication_count']),
            '零命令%d/%d条，原因%s。' % (execution['zero_command_count'], execution['cycle_count'],
                json.dumps(execution['zero_reason_counts'], ensure_ascii=False))]
        common = row['common_motion_window']
        lines.append('共同起步后%s秒内：非目标低速%s秒，实走采样折线%s米。' % (
            show(common['duration_s'], 3), show(common['non_goal_low_speed']['total_s'], 3),
            show(common['sampled_polyline_path_length_m'], 3)))
        stages = row['stage_timeline']
        lines.append('仿真时刻：移动观测%s → 受阻触发%s → 规划请求%s → 发布新参考%s → 首条有效新控制%s秒。' % tuple(
            show(stages[k], 3) for k in ('first_observed_moving_elapsed_s', 'first_blocked_trigger_elapsed_s',
                'first_refresh_request_elapsed_s', 'first_refresh_reference_publish_elapsed_s',
                'first_new_validated_command_elapsed_s')))
        if stages['request_to_reference_publish_wall_s'] is not None:
            lines.append('请求到发布参考%s秒墙钟；发布到首条有效新控制%s秒墙钟。' % (
                show(stages['request_to_reference_publish_wall_s'], 3),
                show(stages['reference_publish_to_new_validated_command_wall_s'], 3)))
        for item in row['final_candidate_counterfactuals']:
            if not item['recorded_native_pass']:
                continue
            checks = item['checks']
            gaps = [show(100*c['minimum']['minimum_model_gap_m'], 2)+'厘米' if c.get('minimum') else '无有效样本' for c in checks]
            lines.append('请求%d实际完整慢层验收通过：同冻结CV常速/半速最小模型间隙%s/%s；旧半速分支%s；实际采用版本%s。' % (
                item['request_id'], *gaps, '会拒绝' if item['old_half_branch_would_reject'] is True else
                '未发现拒绝' if item['old_half_branch_would_reject'] is False else '未确认', item['consumed_revisions']))
        adoption = row.get('first_adoption')
        if adoption and adoption.get('adoption_error'):
            error, after = adoption['adoption_error'], adoption['following_2s']
            lines.append('首次采用位置/朝向偏差%s厘米/%s度；随后实际记录%s秒内，低速%s秒、倒车%s秒。' % (
                show(100*error['position_m'], 2), show(error['yaw_abs_deg'], 2), show(after['duration_s']),
                show(after['measured_non_goal_low_speed']['total_s']), show(after['measured_reverse_duration_s'])))
        else:
            lines.append('未关联到新参考采用，采用偏差与随后2秒指标留空。')
        lines.append('优化/硬验收失败原因%s；修复触发%d次；诊断暂停%d条。' % (
            json.dumps(row['failure_reason_counts'], ensure_ascii=False), len(row['repair_execution_events']),
            len(row['online_pause_audit']['explicit_diagnostic_pause_events'])))
    history = data['history']
    if history['available']:
        lines += ['', '历史核对：%d条失败候选常速动态通过、半速失败；%d条确认未执行。实际MPPI反馈比例随运动变化，没有固定半速执行承诺。失败候选附近的旧参考运动不能证明该失败候选安全。' % (
            sum(r['nominal_pass_half_fail_count'] for r in history['trials']),
            sum(r['unexecuted_failure_count'] for r in history['trials']))]
    if data.get('trigger_execution_audit'):
        lines += ['', 'A碰撞链核对：障碍开始运动后，旧参考未来2秒检查多次仍判可走；判受阻才触发重规划。新轨迹失败后，MPPI先验收失败、再无可行动作，机器人接近停住时被迎面障碍物碰到。具体时刻与原始记录见“A触发与执行核对.json”。',
            'B首次触发比A早约0.555秒、原MPPI绕行侧也不同，不能把两组最终成败全部归因于验收开关。']
    lines += ['', '解释范围：常速动态复算通过不等于完整原生验收通过；完整验收以本次实际记录为准。模型间隙与物理球采样间隙分开；超出CV窗的未来未知。',
        '重规划触发看旧参考未来2秒几何检查是否受阻，看到移动障碍本身不立即触发；观测到请求前的经过时间不能一概视为系统延迟。',
        '制动批准检查候选首条指令持续一个控制有效期（本轮0.25秒），再制动到停稳0.1秒；没有检查停稳后剩余预测时间，因此仍可能被继续接近的障碍物撞到。',
        '同种子在线A/B输入时序可能不同；只有同候选反事实能直接定位额外拒绝来源。GIF保留67球投影、真实参考、获准展开和保存的搜索子集。',
        '数值仿真不是Gazebo/实车；生产部署未替换。', '']
    return '\n'.join(lines)


def chart(data, output):
    rows = data['rows']
    image = Image.new('RGB', (1180, 590), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 14), '慢层参考验收：半速要求是否阻碍交付', font=title, fill='#333333')
    draw.text((20, 51), 'A：常速＋半速硬验收　B：常速硬验收；两组保留实际MPPI展开与制动检查。', font=small, fill='#666666')
    metrics = [('首次请求 → 有效新控制 / s', [delay(r) for r in rows], 3),
        ('慢层未通过请求 / 次', [r['delivery_chain_counts']['refresh_requests']-
            r['delivery_chain_counts']['requests_with_native_optimizer_pass'] for r in rows], 0),
        ('共同时间窗非目标低速 / s', [r['common_motion_window']['non_goal_low_speed']['total_s'] for r in rows], 3)]
    for index, (name, values, digits) in enumerate(metrics):
        left, right = 55+index*390, 327+index*390
        upper = max([float(v) for v in values if v is not None]+[1.])*1.25
        draw.text((left-28, 103), name, font=small, fill='#333333')
        for fraction in (0., .5, 1.):
            x = left+fraction*(right-left)
            draw.line((x, 143, x, 299), fill='#e5e5e5')
            draw.text((x-8, 307), show(upper*fraction, 1), font=small, fill='#777777')
        for group, value in enumerate(values):
            y = 160+group*74
            draw.text((left-27, y+7), 'AB'[group], font=text, fill=colors[group])
            if value is None:
                draw.text((left+5, y+8), '未交付', font=small, fill='#555555')
            else:
                x = left+float(value)/upper*(right-left)
                if value > 0:
                    draw.rectangle((left, y, x, y+35), fill=colors[group])
                draw.text((x+5, y+8), show(value, digits), font=small, fill='#444444')
    draw.line((20, 352, 1160, 352), fill='#dddddd')
    for group, row in enumerate(rows):
        top = 371+group*82
        c, e = row['delivery_chain_counts'], row['execution_evidence']
        chain = '→'.join(str(c[k]) for k in ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed'))
        draw.text((20, top), 'AB'[group]+'　请求→原生→发布→采用：'+chain, font=text, fill=colors[group])
        draw.text((20, top+32), '最小物理采样间隙 %s cm；%s接触；展开＋制动唯一核对 %d/%d 条有效控制。' % (
            show(100*row['minimum_physical_sample_gap_m'], 2), '发现' if row['physical_sample_collision'] else '未发现',
            e['successful_exact_selected_and_brake_matches'], e['validated_command_count']), font=small, fill='#555555')
    draw.text((20, 537), '停留柱仅比较各自起步后的共同 %.3f s；全程终止结果均保留。' % rows[0]['common_motion_window']['duration_s'], font=small, fill='#666666')
    draw.text((20, 563), '单次不暂停ROS数值闭环；参考入口要求调整，不代表双时间尺度协同已经完成。', font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--changed', type=Path)
    parser.add_argument('--history', type=Path)
    parser.add_argument('--max-frames', type=int, default=70)
    args = parser.parse_args()
    pairs = [load_online(args.baseline or args.root/'headon_A', LABELS[0]),
             load_online(args.changed or args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='half_speed_contract_report_v1', rows=list(rows),
        history=history_summary(args.history or args.root/'replay/历史核对.json'),
        same_native_binaries=(rows[0]['native_binaries_sha256'] == rows[1]['native_binaries_sha256']
            if all(r['native_binaries_sha256'] for r in rows) else None),
        same_mppi_library=(rows[0]['mppi_library_sha256'] == rows[1]['mppi_library_sha256']
            if all(r['mppi_library_sha256'] for r in rows) else None),
        scope='One normal unpaused fixed-arm REMANI+MPPI numerical ROS SIL A/B. Full native admission from recorded execution, dynamic-only same-candidate counterfactual, actual selected/brake publication evidence. No hardware.')
    audit = args.root/'report/A触发与执行核对.json'
    data['trigger_execution_audit'] = (dict(path=str(audit.resolve()), sha256=sha(audit),
        records=json.loads(audit.read_text())) if audit.is_file() else None)
    data['conclusions'] = conclusions(data)
    output = args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    data['artifacts'] = dict(headon=animation(rows, trials, output/'迎面对照.gif', args.max_frames))
    keyframes(output/'迎面对照.gif', output/'迎面关键帧.png')
    chart(data, output/'对照数据.png')
    (output/'实验结果.txt').write_text(narrative(data))
    for trial in trials:
        trial.verify_unchanged()
    data['report_script_sha256'] = sha(Path(__file__))
    (output/'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('HALF_SPEED_CONTRACT_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

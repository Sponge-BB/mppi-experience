#!/usr/bin/env python3
"""Read completed off/on fast-avoidance notification runs; no control replay.

The experiment changes the slow refresh trigger only. A sent hint, a slow
request, a published reference, and an actually validated new control are
different events. Reference collision counterfactuals are not recomputed here.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_delivery_feedback import load_trial
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window, execution_evidence
from report_static_agility import show
from report_time_indexed_collision import OUTCOMES, TERMINATIONS
from report_window_continuity import animation


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/fast_refresh_feedback_20261101_01'
LABELS = ('A：原重规划触发', 'B：增加快层绕行反馈')


def earliest(rows, key):
    return min(rows, key=lambda r: r[key]) if rows else None


def span(later, earlier):
    return later-earlier if later is not None and earlier is not None else None


def analyze_hints(trial, timeline, commands):
    cycle_index = defaultdict(list)
    for cycle in commands:
        cycle_index[(cycle.get('command_header_ros_time'), cycle.get('goal_generation'),
                     cycle.get('guide_revision'))].append(cycle)
    hints = []
    for diagnostic in trial.diagnostics:
        value = diagnostic.get('fast_refresh_feedback')
        if not value:
            continue
        hint = dict(value)
        cycles = cycle_index[(hint['stamp'], hint['generation'], hint['revision'])]
        cycle = cycles[0] if len(cycles) == 1 else None
        hint.update(exact_cycle_match_count=len(cycles),
            cycle_id=cycle.get('cycle_id') if cycle else None,
            exact_actual_validated_publication=(cycle.get('validated_command_published') is True
                and cycle.get('validation_invoked') is True and cycle.get('validation_code') == 0)
                if cycle else None,
            command=[cycle['command_v'], cycle['command_w']] if cycle else None,
            actual_command_publish_after_monotonic=cycle.get('command_publish_after_monotonic') if cycle else None,
            actual_command_publish_after_ros_time=cycle.get('command_publish_after_ros_time') if cycle else None)
        hints.append(hint)
    active = [h for h in hints if h['active'] and h['validated']]
    episodes = defaultdict(list)
    for hint in active:
        episodes[(hint['generation'], hint['episode'])].append(hint)
    episode_records = []
    for (generation, episode), rows in sorted(episodes.items()):
        first = earliest(rows, 'publication_monotonic')
        episode_records.append(dict(generation=generation, episode=episode, first=first,
            active_validated_hint_count=len(rows), revisions=sorted(set(h['revision'] for h in rows))))
    triggers = [r for r in timeline if r.get('event') == 'refresh_trigger']
    linked = []
    for trigger in triggers:
        matching = [e for e in episode_records if e['episode'] == trigger.get('episode')
                    and e['first']['publication_monotonic'] <= trigger['mono']]
        episode = matching[0] if len(matching) == 1 else None
        first = episode['first'] if episode else None
        linked.append(dict(trigger=trigger, unique_episode_match=len(matching) == 1,
            first_episode_hint=first,
            first_episode_hint_send_to_trigger_wall_s=span(trigger['mono'], first['publication_monotonic']) if first else None,
            first_episode_hint_send_to_trigger_sim_s=span(trigger['sim'], first['publication_ros_time']) if first else None,
            scope='Episode onset to trigger, not exact transport delay or identity of the latest eligible packet. The FSM validates packet identity and freshness separately.'))
    consumed = Counter(r.get('episode') for r in triggers if r.get('fast_feedback_trigger') is True)
    cooperative = [r for r in timeline if r.get('event') == 'cooperative_check']
    return dict(hints=hints, hint_count=len(hints), active_validated_hints=active,
        active_validated_hint_count=len(active), episode_count=len(episode_records), episodes=episode_records,
        first_actual_approved_avoidance_hint=earliest(active, 'publication_monotonic'),
        active_hints_without_exact_validated_cycle=sum(h['exact_actual_validated_publication'] is not True for h in active),
        trigger_links=linked, triggers=triggers,
        feedback_trigger_count=sum(r.get('fast_feedback_trigger') is True for r in triggers),
        feedback_triggers_with_old_guide_unblocked=sum(r.get('fast_feedback_trigger') is True
            and r.get('blocked') is False for r in triggers),
        feedback_consumptions_by_episode=dict(consumed),
        maximum_feedback_consumptions_per_episode=max(consumed.values(), default=0),
        trigger_source_counts=dict(Counter(r.get('source') for r in triggers)),
        cooperative_check_count=len(cooperative),
        eligible_but_disabled_check_count=sum(r.get('fast_feedback_eligible') is True
            and r.get('feedback_enabled') is False for r in cooperative),
        first_cooperative_guide_blocked=earliest([r for r in cooperative if r.get('blocked') is True], 'mono'),
        scope='First approved avoidance is the existing kernel dynamic intervention mode with a validated published command; it is not a new yaw/position threshold or an execution commitment.')


def load_online(source, label):
    # This loader retains the previous delivery/adoption and loop accounting,
    # without repeating the previous experiment's candidate half-speed replay.
    row, trial = load_trial(source, label)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    commands = trial.reader.jsonl('state_mppi_cycles.jsonl')
    requests = [r for r in row['remani']['requests'] if r.get('request_id', 0) > 1]
    publications = [r for r in timeline if r.get('event') == 'reference_publish'
                    and r.get('request_id', 0) > 1 and r['sim'] <= 1000.+trial.end+1e-6]
    references = [r for r in trial.refs if r.get('revision', 0) > 1 and r['t'] <= trial.end]
    passed = [r for r in requests if any(c.get('success') is True for c in r.get('optimizer_calls', []))]
    row['delivery_chain_counts'] = dict(refresh_requests=len(requests),
        requests_with_native_optimizer_pass=len(passed),
        refresh_reference_publications=len(publications), refresh_references_consumed=len(references),
        native_pass_request_ids=[r['request_id'] for r in passed],
        publication_request_ids=[r['request_id'] for r in publications],
        consumed_revisions=[r['revision'] for r in references])
    row['failure_reason_counts'] = dict(Counter(c.get('reason', 'unrecorded')
        for request in requests for c in request.get('optimizer_calls', []) if c.get('success') is not True))
    row['execution_evidence'] = execution_evidence(trial, row)
    row['fast_refresh'] = analyze_hints(trial, timeline, commands)
    first_hint = row['fast_refresh']['first_actual_approved_avoidance_hint']
    first_trigger = earliest(row['fast_refresh']['triggers'], 'mono')
    raw_requests = [r for r in timeline if r.get('event') == 'request_begin' and r.get('request_id', 0) > 1]
    first_request = earliest(raw_requests, 'mono')
    first_publish = earliest(publications, 'mono')
    valid = [c for c in commands if (c.get('guide_revision') or 0) > 1
             and c.get('validated_command_published')]
    first_valid = earliest(valid, 'command_publish_after_monotonic')
    first_observed = trial.result.get('native_realtime_evidence', {}).get('first_observed_moving_t')
    hint_mono = first_hint['publication_monotonic'] if first_hint else None
    row['stage_timeline'] = dict(first_observed_moving_elapsed_s=first_observed,
        first_approved_avoidance_hint_elapsed_s=first_hint['publication_ros_time']-1000. if first_hint else None,
        first_approved_avoidance_command_elapsed_s=first_hint['actual_command_publish_after_ros_time']-1000.
            if first_hint and first_hint['actual_command_publish_after_ros_time'] is not None else None,
        first_trigger_elapsed_s=first_trigger['sim']-1000. if first_trigger else None,
        first_trigger_source=first_trigger.get('source') if first_trigger else None,
        first_trigger_old_guide_blocked=first_trigger.get('blocked') if first_trigger else None,
        first_refresh_request_elapsed_s=first_request['sim']-1000. if first_request else None,
        first_refresh_reference_publish_elapsed_s=first_publish['sim']-1000. if first_publish else None,
        first_new_validated_command_elapsed_s=first_valid['command_publish_after_ros_time']-1000. if first_valid else None,
        hint_send_to_trigger_wall_s=span(first_trigger['mono'] if first_trigger else None, hint_mono),
        hint_send_to_first_request_wall_s=span(first_request['mono'] if first_request else None, hint_mono),
        hint_send_to_first_new_validated_command_wall_s=span(first_valid['command_publish_after_monotonic'] if first_valid else None, hint_mono),
        trigger_to_request_wall_s=span(first_request['mono'] if first_request else None, first_trigger['mono'] if first_trigger else None),
        request_to_reference_publish_wall_s=span(first_publish['mono'] if first_publish else None, first_request['mono'] if first_request else None),
        reference_publish_to_new_validated_command_wall_s=span(first_valid['command_publish_after_monotonic'] if first_valid else None, first_publish['mono'] if first_publish else None),
        request_to_new_validated_command_wall_s=(row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']/1000.
            if row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms'] is not None else None),
        hint_clock_semantics='publication_monotonic/publication_ros_time are sampled immediately before hint publish; command header stamp is the earlier validation clock. These are not network-arrival measurements.')
    events = trial.reader.json('events.json', [])
    if isinstance(events, dict):
        events = events.get('events', events.get('rows', []))
    pauses = [r for r in timeline if r.get('event') in ('candidate_exit_audit_begin', 'candidate_exit_audit_end')]
    row['online_pause_audit'] = dict(explicit_diagnostic_pause_events=pauses,
        plant_loop_gap_events=[r for r in events if r.get('kind') == 'plant_loop_gap'])
    row['repair_execution_events'] = [r for r in timeline if r.get('event') in
        ('early_collision_repair_yield', 'candidate_collision_repair_begin')]
    row['settings'] = dict(fast_refresh_feedback=trial.manifest.get('fast_refresh_feedback'),
        half_speed_contract=trial.manifest.get('half_speed_contract'))
    row['binary_identity'] = dict(native=trial.manifest.get('native_binaries_sha256'),
        mppi=trial.manifest.get('mppi_library_sha256'))
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def conclusions(rows):
    a, b = rows
    ba = b['fast_refresh']
    notes = []
    if ba['feedback_trigger_count']:
        notes.append('B实际由快层反馈触发%d次，其中%d次旧参考尚未判受阻；新增触发链确实被用到。' % (
            ba['feedback_trigger_count'], ba['feedback_triggers_with_old_guide_unblocked']))
    else:
        notes.append('B未出现快层反馈触发，本轮不能判断新增触发机制效果。')
    waits = [r['stage_timeline']['hint_send_to_trigger_wall_s'] for r in rows]
    notes.append('首次获准绕行反馈发送→重规划触发：A %s，B %s。' % tuple(
        show(v, 3)+'秒' if v is not None else '未关联到' for v in waits))
    deliveries = [r['stage_timeline']['request_to_new_validated_command_wall_s'] for r in rows]
    notes.append('首次规划请求→新参考有效控制：A %s，B %s；这段时间不含此前感知和触发等待。' % tuple(
        show(v, 3)+'秒' if v is not None else '未交付' for v in deliveries))
    for row in rows:
        notes.append(row['label']+'：'+('到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination'])))+
            '，'+('检测到' if row['physical_sample_collision'] else '未发现')+'物理球采样接触。')
    if ba['maximum_feedback_consumptions_per_episode'] > 1:
        notes.append('同一绕行episode出现重复反馈触发，需检查去重；当前不能认为触发控制正确。')
    if not b['execution_evidence']['all_validated_publications_have_matching_selected_and_brake_approval']:
        notes.append('B有效控制与实际展开/制动验收没有全部唯一关联，执行验证证据不完整。')
    notes.append('本轮只检验快层已经介入后能否及时通知慢层；异步A/B输入可能不同，不能据最终成败单独证明因果或双时间尺度优化已完成。')
    return notes


def narrative(data):
    rows = data['rows']
    lines = ['快层绕行反馈触发慢层：实验结果', '',
        '核心问题：MPPI已获准介入动态避障，REMANI仍等旧参考判受阻才重规划，造成快慢层响应脱节。',
        'A沿用原触发；B把已通过实际展开/制动检查的动态介入episode反馈给REMANI，每个episode只触发一次。沿用原轮询、冷却和内核滞回，没有加新的偏差/TTC阈值。',
        '两组都采用慢层常速硬验收，常速＋半速固定窗软代价；提前/加权修复关闭。MPPI算法、原生67球、静态/自碰撞/轮约束及实际展开/制动检查保持一致。', '',
        '结论']+data['conclusions']+['', '正常在线迎面对照（各一次、不暂停）']
    for row in rows:
        stages, fast, chain = row['stage_timeline'], row['fast_refresh'], row['delivery_chain_counts']
        lines += ['', row['label'],
            '仿真时刻：移动观测%s → 首次获准绕行控制%s（反馈发送%s）→ 重规划触发%s → 请求%s → 新参考发布%s → 有效新控制%s秒。' % tuple(
                show(stages[k], 3) for k in ('first_observed_moving_elapsed_s',
                    'first_approved_avoidance_command_elapsed_s', 'first_approved_avoidance_hint_elapsed_s',
                    'first_trigger_elapsed_s', 'first_refresh_request_elapsed_s',
                    'first_refresh_reference_publish_elapsed_s', 'first_new_validated_command_elapsed_s')),
            '墙钟：反馈发送→触发%s秒，触发→请求%s秒，请求→发布%s秒，发布→有效新控制%s秒；反馈发送→有效新控制共%s秒。' % tuple(
                show(stages[k], 3) for k in ('hint_send_to_trigger_wall_s', 'trigger_to_request_wall_s',
                    'request_to_reference_publish_wall_s', 'reference_publish_to_new_validated_command_wall_s',
                    'hint_send_to_first_new_validated_command_wall_s')),
            '首次触发来源%s，旧参考受阻=%s。获准介入episode%d个、反馈触发%d次，同episode最多触发%d次；所有触发来源%s。' % (
                stages['first_trigger_source'], stages['first_trigger_old_guide_blocked'], fast['episode_count'],
                fast['feedback_trigger_count'], fast['maximum_feedback_consumptions_per_episode'],
                json.dumps(fast['trigger_source_counts'], ensure_ascii=False)),
            '请求→原生验收通过→最新CV发布→MPPI采用：%d→%d→%d→%d；最新CV拒绝%d次；优化/硬验收失败%s。' % (
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed'],
                len(row['latest_rejections']), json.dumps(row['failure_reason_counts'], ensure_ascii=False)),
            '各自结束前：最小物理采样间隙%s厘米，实走%s米，非目标低速%s秒；MPPI完整循环P95/最大%s/%s毫秒。' % (
                show(100*row['minimum_physical_sample_gap_m'], 3), show(row['actual_path_length_m'], 3),
                show(row['non_goal_low_speed']['total_s'], 3), show(row['callback']['p95_ms'], 1), show(row['callback']['max_ms'], 1))]
        common, execution = row['common_motion_window'], row['execution_evidence']
        lines += ['共同起步后%s秒：非目标低速%s秒、实走采样折线%s米。有效控制展开＋制动唯一核对%d/%d条，非零但未经有效发布%d条。' % (
            show(common['duration_s'], 3), show(common['non_goal_low_speed']['total_s'], 3),
            show(common['sampled_polyline_path_length_m'], 3), execution['successful_exact_selected_and_brake_matches'],
            execution['validated_command_count'], execution['nonzero_without_validated_publication_count'])]
        for request in row['remani']['requests']:
            if request.get('request_id', 0) <= 1:
                continue
            lines.append('请求%d：%s，%s毫秒；原因%s；发布拒绝%s。' % (
                request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                show(request.get('request_wall_ms'), 1), [c.get('reason') for c in request.get('optimizer_calls', [])],
                request.get('acceptance_rejections', [])))
        adoption = row.get('first_adoption')
        if adoption and adoption.get('adoption_error'):
            error, following = adoption['adoption_error'], adoption['following_2s']
            lines.append('首次采用偏差%s厘米/%s度；采用后记录%s秒内低速%s秒、倒车%s秒。' % (
                show(100*error['position_m'], 2), show(error['yaw_abs_deg'], 2), show(following['duration_s']),
                show(following['measured_non_goal_low_speed']['total_s']), show(following['measured_reverse_duration_s'])))
    lines += ['', '解释范围：获准绕行指既有MPPI动态介入模式下的有效控制，不用角速度或横向偏差另设判据；反馈本身不授予控制权限。观测到障碍与参考将来冲突不同，不能把此前全部经过时间称作系统延迟。',
        '展开/制动批准检查候选首条命令持续有效期（0.25秒）再制动至停稳0.1秒，没有保证停稳后剩余预测时间持续安全。模型间隙和物理球采样接触分开，后者仍是离散采样。',
        '早终止场景的全程停留/路程不能直接横比；另给共同时间窗。固定臂、差速底盘完整REMANI＋MPPI ROS数值闭环，非Gazebo/实车；生产部署未替换。', '']
    return '\n'.join(lines)


def chart(data, output):
    rows = data['rows']
    image = Image.new('RGB', (1180, 590), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 14), '快层绕行反馈：是否更早触发并交付慢层参考', font=title, fill='#333333')
    draw.text((20, 51), 'A：原触发　B：增加已获准介入反馈；两组均为慢层常速验收，实际控制检查保持一致。', font=small, fill='#666666')
    metrics = [('介入反馈发送 → 触发 / s', [r['stage_timeline']['hint_send_to_trigger_wall_s'] for r in rows]),
        ('首次请求 → 有效新控制 / s', [r['stage_timeline']['request_to_new_validated_command_wall_s'] for r in rows]),
        ('共同窗非目标低速 / s', [r['common_motion_window']['non_goal_low_speed']['total_s'] for r in rows])]
    for index, (name, values) in enumerate(metrics):
        left, right = 55+index*390, 327+index*390
        upper = max([float(v) for v in values if v is not None]+[.1])*1.25
        draw.text((left-28, 103), name, font=small, fill='#333333')
        for fraction in (0., .5, 1.):
            x = left+fraction*(right-left)
            draw.line((x, 143, x, 299), fill='#e5e5e5')
            draw.text((x-8, 307), show(upper*fraction, 2), font=small, fill='#777777')
        for group, value in enumerate(values):
            y = 160+group*74
            draw.text((left-27, y+7), 'AB'[group], font=text, fill=colors[group])
            if value is None:
                draw.text((left+5, y+8), '未触发/未交付', font=small, fill='#555555')
            else:
                x = left+float(value)/upper*(right-left)
                if value > 0:
                    draw.rectangle((left, y, x, y+35), fill=colors[group])
                draw.text((x+5, y+8), show(value, 3), font=small, fill='#444444')
    draw.line((20, 352, 1160, 352), fill='#dddddd')
    for group, row in enumerate(rows):
        y = 371+group*82
        chain, fast = row['delivery_chain_counts'], row['fast_refresh']
        chain_text = '→'.join(str(chain[k]) for k in ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed'))
        draw.text((20, y), 'AB'[group]+'　请求→原生→发布→采用：'+chain_text+
            '；反馈触发 %d 次（旧参考尚通 %d 次）' % (fast['feedback_trigger_count'], fast['feedback_triggers_with_old_guide_unblocked']), font=text, fill=colors[group])
        draw.text((20, y+32), '最小物理采样间隙 %s cm；%s接触；完整循环P95 %s ms。' % (
            show(100*row['minimum_physical_sample_gap_m'], 3), '发现' if row['physical_sample_collision'] else '未发现',
            show(row['callback']['p95_ms'], 1)), font=small, fill='#555555')
    draw.text((20, 537), '停留柱比较各自起步后的共同 %.3f s；第一柱从实际获准介入后的反馈发送起算。' % rows[0]['common_motion_window']['duration_s'], font=small, fill='#666666')
    draw.text((20, 563), '单次不暂停ROS数值闭环；保留全部失败，不据异步A/B最终成败单独归因。', font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    args = parser.parse_args()
    pairs = [load_online(args.root/'headon_A', LABELS[0]), load_online(args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='fast_refresh_feedback_report_v1', rows=list(rows), conclusions=conclusions(rows),
        same_native_binaries=(rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native']),
        same_mppi_library=(rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi']),
        both_slow_nominal_only=all(r['settings']['half_speed_contract']['nominal_only'] is True for r in rows),
        scope='One unpaused fixed-arm REMANI+MPPI numerical ROS SIL pair; slow refresh hint off/on only, no optimizer/controller replay or historical performance substitution.')
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
    print('FAST_REFRESH_FEEDBACK_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

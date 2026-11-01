#!/usr/bin/env python3
"""Read one existing-prefix off/on pair, including complete initializer curves.

No ROS, optimization, control replay, new collision policy or time mapping.
Prefix use is an observed event, never inferred from its enabling flag. Its
continuation A* search head is not mistaken for the complete trajectory head.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from replay_temporal_collision import digest
from report_fast_refresh_feedback import animation, load_online as load_feedback_online
from report_frontend_rejoin import keyframes
from report_half_speed_contract import add_common_motion_window
from report_static_agility import crossing_y, show
from report_static_width_diagnosis import geometry, sample_curve
from report_time_indexed_collision import OUTCOMES, TERMINATIONS


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/prefix_handoff_20261101_01'
LABELS = ('A：原搜索初始化', 'B：尝试加入MPPI前缀')
REASONS = dict(used='实际使用', stale_or_missing='预测过期或缺失', start_mismatch='预测起点与真实状态不符',
    short_prefix='可用前缀太短', gear_or_low_speed='换向或速度太低',
    prefix_fit_singularity='拟合出现奇异或过低速度', prefix_fit_collision='拟合前缀碰撞',
    continuation_failed_or_gear_changed='末端续接搜索失败或换挡不符')


def normalized_geometry(parts):
    return [dict(gear=part['gear'], pieces=[dict(T=p['T'], coeffs=p['coeffs'])
                                          for p in part['pieces']]) for part in parts]


def sign(value):
    return 0 if abs(value) < 1e-9 else (1 if value > 0. else -1)


def side_offset(point, head):
    return -math.sin(head['yaw'])*(point[0]-head['x'])+math.cos(head['yaw'])*(point[1]-head['y'])


def side_equal(first, second):
    return sign(first) == sign(second) if sign(first) and sign(second) else None


def curve_summary(record, head, target, prefix_s=None):
    if record is None:
        return None
    complete = all(record.get(k, len(record['parts'])) == len(record['parts'])
                   for k in ('minco_count', 'gear_count', 'parts_recorded'))
    result = dict(stage=record['stage'], success=record.get('success'),
        capture_mono=record['capture_mono'], capture_sim=record['capture_sim'],
        resource_replan_id=record['resource_replan_id'], complete=complete,
        full_polynomial_parts=record['parts'])
    if not complete or not record['parts']:
        result['analysis_unavailable_reason'] = 'missing_or_incomplete_internal_curve_capture'
        return result
    durations = [p['T'] for part in record['parts'] for p in part['pieces']]
    duration = float(sum(durations))
    points = sample_curve(record)
    requested = sorted(set([0., .5, 1., 1.1, 2., 3.]+([prefix_s] if prefix_s is not None else [])))
    times = [t for t in requested if t <= duration]
    samples = sample_curve(record, times)
    target_offset = side_offset([target['x'], target['y']], head) if target else None
    offsets = np.asarray([side_offset(point, head) for point in points])
    result.update(geometry_sha256=digest(normalized_geometry(record['parts'])), duration_s=duration,
        pieces=len(durations), start_xy=points[0].tolist(), end_xy=points[-1].tolist(),
        head_position_error_m=math.hypot(points[0, 0]-head['x'], points[0, 1]-head['y']),
        full_curve_path_length_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
        full_curve_side_offset_range_m=[float(offsets.min()), float(offsets.max())],
        world_centerline_y_at_x3_m=crossing_y(points, 3.),
        sampled_nodes=[dict(local_t_s=t, xy=point.tolist(), world_y_m=float(point[1]),
            head_frame_side_offset_m=side_offset(point, head),
            same_side_as_frozen_soft_target=side_equal(side_offset(point, head), target_offset)
                if target_offset is not None else None) for t, point in zip(times, samples)],
        scope='Complete recorded polynomial sampled across every piece at <=.01s. Local samples describe geometry, not a promised rendezvous time. x=3 is a fixed world cross-section in this straight centerline encounter.')
    return result


def prefix_analysis(row, trial):
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    trace = sorted(trial.reader.jsonl('static_width_trace.jsonl'), key=lambda r: r['capture_mono'])
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    native_finals = [r for r in captures if r.get('stage') == 'final_candidate']
    prefix_events = [r for r in timeline if r.get('event') == 'collab_prefix' and r.get('request_id', 0) > 1]
    initializer_events = [r for r in timeline if r.get('event') == 'collab_initializer' and r.get('request_id', 0) > 1]
    requests = {r['request_id']: r for r in timeline if r.get('event') == 'request_begin'}
    publications = [r for r in timeline if r.get('event') == 'reference_publish']
    attempts = []
    for initial in [r for r in trace if r.get('stage') == 'minco_before_optimizer' and r.get('request_id', 0) > 1]:
        request_id = initial['request_id']
        request = requests[request_id]
        head = dict(x=request['start_x'], y=request['start_y'], yaw=request['start_yaw'])
        prefix = [r for r in prefix_events if r['request_id'] == request_id and r['mono'] <= initial['capture_mono']]
        prefix = max(prefix, key=lambda r: r['mono']) if prefix else None
        used = prefix.get('used') is True if prefix else False
        searches = [r for r in trace if r.get('stage') == 'native_search_raw'
            and r['request_id'] == request_id and r['capture_mono'] <= initial['capture_mono']]
        search = searches[-1] if searches else None
        finals = [r for r in trace if r.get('stage') == 'optimizer_return' and r['request_id'] == request_id
            and r['resource_replan_id'] == initial['resource_replan_id'] and r['capture_mono'] >= initial['capture_mono']]
        final = finals[0] if len(finals) == 1 else None
        feedback = [f for f in row['feedback']['scopes'] if f['scope']['request_id'] == request_id and f.get('target')]
        feedback = feedback[0] if len(feedback) == 1 else None
        target = feedback['target'].get('predicted') if feedback else None
        prefix_s = prefix.get('prefix_s') if used else None
        initial_summary = curve_summary(initial, head, target, prefix_s)
        final_summary = curve_summary(final, head, target, prefix_s)
        if final_summary and final_summary.get('geometry_sha256'):
            matching_native = [r for r in native_finals if r['request_id'] == request_id
                and digest(normalized_geometry(r['candidate']['parts'])) == final_summary['geometry_sha256']]
            final_summary.update(matching_native_final_candidate_count=len(matching_native),
                actual_candidate_geometry_evidenced=(final.get('success') is True or len(matching_native) == 1),
                matching_native_verdicts=[dict(native_success=r.get('native_success'), native_reason=r.get('native_reason'),
                    plan_ordinal=r['plan_ordinal']) for r in matching_native])
            if not final_summary['actual_candidate_geometry_evidenced']:
                final_summary['interpretation'] = 'Failed optimizer storage without matching reconstructed final candidate; retained for inspection, not used to claim final candidate side or feasibility.'
        pub = [r for r in publications if r['request_id'] == request_id]
        refs = [r for r in trial.refs if any(abs(r['stamp']-p['guide_start']) < 1e-7 for p in pub)]
        request_summary = next((r for r in row['remani']['requests'] if r['request_id'] == request_id), {})
        search_points = geometry(search) if search else np.empty((0, 2))
        attempts.append(dict(request_id=request_id, resource_replan_id=initial['resource_replan_id'],
            request_head=head, prefix_event=prefix, prefix_actually_used=used,
            prefix_record_source='actual collab_prefix event, not COLLAB_PREFIX flag',
            search=dict(record=search, start_xy=search_points[0].tolist() if len(search_points) else None,
                head_semantics='native continuation from the prefix endpoint; not full trajectory initial state' if used else 'ordinary native search head',
                world_centerline_y_at_x3_m=crossing_y(search_points, 3.) if len(search_points) else []),
            frozen_soft_target=target,
            target_head_frame_side_offset_m=side_offset([target['x'], target['y']], head) if target else None,
            target_scope=feedback['scope'] if feedback else None,
            initial=initial_summary, optimizer_return=final_summary,
            native_outcome=request_summary.get('outcome_class'), consumed_revisions=[r['revision'] for r in refs],
            latest_rejections=[r for r in row['latest_rejections'] if r['request_id'] == request_id],
            semantics='Full MINCO initializer retains the measured request head. Prefix source geometry and the later frozen coupling target are distinct records; a 1s prefix or early same-side point does not establish whole-curve same-side avoidance or committed execution.'))
    phases = [r for r in timeline if r.get('event') == 'candidate_phase_alignment' and r.get('request_id', 0) > 1]
    return dict(prefix_events=prefix_events, initializer_events=initializer_events,
        attempts=attempts, prefix_attempt_count=len(prefix_events),
        prefix_used_count=sum(r.get('used') is True for r in prefix_events),
        fallback_reason_counts=dict(Counter(r.get('reason') for r in prefix_events if r.get('used') is not True)),
        successful_use_reasons=dict(Counter(r.get('reason') for r in prefix_events if r.get('used') is True)),
        phase_alignment_events=phases,
        actual_world_centerline_y_at_x3_m=row.get('actual_crossing_y_at_x3_m'),
        scope='Recorded initializer and optimizer polynomials, plus continuation-only raw search when prefix is used. Analytic geometry inspection, no solver replay or collision-boundary change.')


def load_online(source, label):
    row, trial = load_feedback_online(source, label)
    row['prefix_handoff'] = prefix_analysis(row, trial)
    row['prefix_settings'] = trial.manifest.get('prefix_handoff', trial.manifest.get('coupling_experiment', {}))
    row['source_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def conclusions(rows):
    a, b = rows
    notes = ['B实际尝试前缀%d次、使用%d次；未使用原因%s。启用开关不等于实际用上。' % (
        b['prefix_handoff']['prefix_attempt_count'], b['prefix_handoff']['prefix_used_count'],
        json.dumps(b['prefix_handoff']['fallback_reason_counts'], ensure_ascii=False))]
    totals = [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows]
    notes.append('首次获准介入反馈→真正有效新控制：A %s，B %s。' % tuple(
        show(value, 3)+'秒' if value is not None else '未交付' for value in totals))
    if all(value is not None for value in totals):
        notes.append('本次端到端交付差值B−A为%+.3f秒；相同种子在线输入仍可能不同，不据此估计总体收益。' % (totals[1]-totals[0]))
    notes.append('最新动态发布验收拒绝：A %d次，B %d次。' % tuple(len(r['latest_rejections']) for r in rows))
    for row in rows:
        notes.append(row['label']+'：'+('到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], str(row['termination'])))+
            '，'+('检测到' if row['physical_sample_collision'] else '未发现')+'物理球采样接触。')
    notes.append('初始化短前缀跟随、优化后绕行同侧、最新检查通过、实际交付是四件事；本轮不把前缀作为必须执行的承诺，也未解决所有双时间尺度协同问题。')
    return notes


def curve_text(summary):
    if not summary or not summary.get('complete') or not summary.get('sampled_nodes'):
        return '没有完整曲线记录'
    side = {round(r['local_t_s'], 6): r['head_frame_side_offset_m'] for r in summary['sampled_nodes']}
    samples = '、'.join('%g秒%+.2f厘米' % (t, 100*side[t]) for t in (1., 2., 3.) if t in side)
    across = summary['world_centerline_y_at_x3_m']
    return '完整曲线%d段；头部偏差%s厘米；head坐标系侧移%s；x=3米截面y=%s米' % (
        summary['pieces'], show(100*summary['head_position_error_m'], 3), samples,
        '/'.join('%+.3f' % y for y in across) if across else '未穿过')


def narrative(data):
    lines = ['MPPI短前缀作为REMANI初始化：实验结果', '',
        '核心问题：快层已经绕行，慢层新候选却可能选择另一侧，或按原时刻安全、到发布时已不安全。先检验现有短前缀能否改善完整初值及最终交付。',
        '同一已编译版本，A关闭COLLAB_PREFIX，B打开；两组都启用快层反馈、慢层常速硬验收及原1.1秒软交付目标。没有新增代价、时序映射或约束，也没有重编。',
        '前缀最多1秒、同档位，压成3点拟合2段，再从末端A*续接并合并完整初值；真实当前head保留。前缀只是优化建议，不要求机器人照着执行。', '', '结论']+data['conclusions']
    for row in data['rows']:
        p, chain, stages = row['prefix_handoff'], row['delivery_chain_counts'], row['stage_timeline']
        lines += ['', row['label'],
            '请求→原生通过→最新发布→实际采用：%d→%d→%d→%d；获准介入→触发%s秒，请求→有效新控制%s秒。' % (
                chain['refresh_requests'], chain['requests_with_native_optimizer_pass'],
                chain['refresh_reference_publications'], chain['refresh_references_consumed'],
                show(stages['hint_send_to_trigger_wall_s'], 3), show(stages['request_to_new_validated_command_wall_s'], 3)),
            '最小物理采样间隙%s厘米；MPPI完整循环P95/最大%s/%s毫秒；共同%s秒内非目标低速%s秒。' % (
                show(100*row['minimum_physical_sample_gap_m'], 3), show(row['callback']['p95_ms'], 1),
                show(row['callback']['max_ms'], 1), show(row['common_motion_window']['duration_s'], 3),
                show(row['common_motion_window']['non_goal_low_speed']['total_s'], 3))]
        for attempt in p['attempts']:
            prefix = attempt['prefix_event']
            lines.append('请求%d：前缀%s；实际采用新参考%s；请求结果%s。' % (
                attempt['request_id'], REASONS.get(prefix.get('reason'), prefix.get('reason')) if prefix else '未调用',
                attempt['consumed_revisions'], OUTCOMES.get(attempt['native_outcome'], attempt['native_outcome'])))
            if attempt['prefix_actually_used']:
                lines.append('  前缀%s秒、%s点；native_search_raw起点%s是前缀末端续接起点，不能当整条轨迹起点。' % (
                    show(prefix.get('prefix_s'), 3), prefix.get('prefix_points'), attempt['search']['start_xy']))
            if attempt['frozen_soft_target']:
                target = attempt['frozen_soft_target']
                lines.append('  冻结软反馈目标world y=%+.3f米，head坐标侧移%+.2f厘米（不是固定时刻的强制接轨点）。' % (
                    target['y'], 100*attempt['target_head_frame_side_offset_m']))
            lines.append('  完整初值：'+curve_text(attempt['initial'])+'。')
            final = attempt['optimizer_return']
            if final and final.get('actual_candidate_geometry_evidenced'):
                lines.append('  优化后候选：'+curve_text(final)+'；原生返回成功=%s。' % final['success'])
            else:
                lines.append('  优化返回的内部曲线未能唯一核实为最终候选，不拿它判断最终绕行侧或安全。')
        for phase in p['phase_alignment_events']:
            lines.append('请求%d发布时刻关联：位置残差%s厘米、朝向%s度，常速检查时刻偏移%+.3f秒；残差只是读数，没有设拒绝门限。' % (
                phase['request_id'], show(100*phase['position_residual_m'], 2),
                show(math.degrees(phase['yaw_residual_rad']), 2), phase['nominal_epoch_offset_s']))
        for request in row['remani']['requests']:
            if request.get('request_id', 0) > 1:
                lines.append('请求%d：%s，%s毫秒；后端%s；发布拒绝%s。' % (
                    request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                    show(request.get('request_wall_ms'), 1), [c.get('reason') for c in request.get('optimizer_calls', [])],
                    request.get('acceptance_rejections', [])))
        execution = row['execution_evidence']
        lines.append('有效控制展开＋制动唯一核对%d/%d条；非零但未经有效发布%d条。' % (
            execution['successful_exact_selected_and_brake_matches'], execution['validated_command_count'],
            execution['nonzero_without_validated_publication_count']))
    lines += ['', '解释范围：侧移按每次请求真实head的朝向定义，正为左、负为右；另列本迎面中心线场景x=3米截面的world y，避免仅凭前1秒微小偏移就说整条绕行同侧。全曲线数据与冻结目标都保留在JSON和原始记录中。',
        '同侧不等于同一执行时刻；最新动态拒绝不等于预算或轮约束失败。原生通过、最新发布、快层采用、有效新控制分开统计。',
        'GIF保留真实参考、获准MPPI展开和67球投影；本轮未记录搜索采样，不补画搜索。固定臂差速底盘ROS数值闭环，非Gazebo/实车；部署未替换。', '']
    return '\n'.join(lines)


def chart(data, output):
    rows = data['rows']
    image = Image.new('RGB', (1180, 590), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 14), 'MPPI短前缀初始化：能否改善慢层真正交付', font=title, fill='#333333')
    draw.text((20, 51), 'A：原初始化　B：尝试前缀；同一二进制、相同快层反馈与慢层常速验收。', font=small, fill='#666666')
    metrics = [('介入反馈 → 有效新控制 / s', [r['stage_timeline']['hint_send_to_first_new_validated_command_wall_s'] for r in rows], 3),
        ('最新动态验收拒绝 / 次', [len(r['latest_rejections']) for r in rows], 0),
        ('实际使用前缀 / 次', [r['prefix_handoff']['prefix_used_count'] for r in rows], 0)]
    for index, (name, values, digits) in enumerate(metrics):
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
                draw.text((left+5, y+8), '未交付', font=small, fill='#555555')
            else:
                x = left+float(value)/upper*(right-left)
                if value > 0.:
                    draw.rectangle((left, y, x, y+35), fill=colors[group])
                draw.text((x+5, y+8), show(value, digits), font=small, fill='#444444')
    draw.line((20, 352, 1160, 352), fill='#dddddd')
    for group, row in enumerate(rows):
        y, chain, prefix = 371+group*82, row['delivery_chain_counts'], row['prefix_handoff']
        chain_text = '→'.join(str(chain[k]) for k in ('refresh_requests', 'requests_with_native_optimizer_pass',
            'refresh_reference_publications', 'refresh_references_consumed'))
        draw.text((20, y), 'AB'[group]+'　请求→原生→发布→采用：'+chain_text+
            '；前缀实际使用/尝试 %d/%d' % (prefix['prefix_used_count'], prefix['prefix_attempt_count']), font=text, fill=colors[group])
        draw.text((20, y+32), '最小物理采样间隙 %s cm；%s接触；完整循环P95 %s ms。' % (
            show(100*row['minimum_physical_sample_gap_m'], 3), '发现' if row['physical_sample_collision'] else '未发现',
            show(row['callback']['p95_ms'], 1)), font=small, fill='#555555')
    draw.text((20, 537), '前缀只是优化初值；初始化早期同侧不等于优化后整条绕行同侧，更不等于承诺按时执行。', font=small, fill='#666666')
    draw.text((20, 563), '单次不暂停ROS数值闭环；失败与回退保留，不据异步A/B最终成败单独归因。', font=small, fill='#666666')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--max-frames', type=int, default=70)
    args = parser.parse_args()
    pairs = [load_online(args.root/'headon_A', LABELS[0]), load_online(args.root/'headon_B', LABELS[1])]
    rows, trials = zip(*pairs)
    add_common_motion_window(rows, trials)
    data = dict(schema='prefix_handoff_report_v1', rows=list(rows), conclusions=conclusions(rows),
        same_native_binaries=rows[0]['binary_identity']['native'] == rows[1]['binary_identity']['native'],
        same_mppi_library=rows[0]['binary_identity']['mppi'] == rows[1]['binary_identity']['mppi'],
        both_slow_nominal_only=all(r['settings']['half_speed_contract']['nominal_only'] is True for r in rows),
        scope='One unpaused numerical ROS SIL pair; existing compiled prefix off/on only. No new cost, time mapping, hard constraints or physical deployment. Full initializer vs continuation-only search distinguished.')
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
    print('PREFIX_HANDOFF_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

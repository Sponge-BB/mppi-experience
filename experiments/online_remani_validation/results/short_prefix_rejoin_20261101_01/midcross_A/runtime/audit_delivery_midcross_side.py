#!/usr/bin/env python3
"""Read-only targeted audit of the completed midcross B route-side change."""
import gzip
import json
import math
from pathlib import Path

import numpy as np

from render_coupled_objective import sha
from report_static_width_diagnosis import sample_curve


HERE = Path(__file__).resolve().parent
ROOT = HERE / 'results/delivery_midcross_20261101_01'
TRIAL = ROOT / 'midcross_B'


def rows(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def geometry(record):
    parts = record.get('parts', record.get('candidate', {}).get('parts'))
    points = sample_curve(dict(parts=parts))
    crossings = []
    for first, second in zip(points, points[1:]):
        if (first[0] - 3.) * (second[0] - 3.) <= 0. and first[0] != second[0]:
            crossings.append(float(first[1] + (3. - first[0]) *
                (second[1] - first[1]) / (second[0] - first[0])))
    return dict(stage=record['stage'], request_id=record['request_id'],
        start_xy=points[0].tolist(), end_xy=points[-1].tolist(),
        minimum_x_m=float(np.min(points[:, 0])), maximum_x_m=float(np.max(points[:, 0])),
        minimum_y_m=float(np.min(points[:, 1])), maximum_y_m=float(np.max(points[:, 1])),
        geometric_backtrack_from_start_x_m=max(0., float(points[0, 0] - np.min(points[:, 0]))),
        y_at_all_x3_crossings_m=crossings,
        gears=[part['gear'] for part in parts], duration_s=sum(piece['T'] for part in parts for piece in part['pieces']),
        sampled_path_length_m=float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))),
        scope='Recorded full polynomial sampled at <=0.01s; backtrack is geometric negative-world-x motion, not necessarily reverse gear.')


def prefix_check(frozen, request, native_plan):
    # This uses saved request-time feedback, not a newly captured prefix-entry
    # packet. The actual prefix event is the authoritative used/reason result.
    data = frozen['frozen_rows']
    age = frozen['request_time'] - frozen['seed_stamp']
    begin = 0
    while begin + 1 < len(data) and data[begin][0] < max(0., age):
        begin += 1
    first = data[begin]
    direction = native_plan['gears'][0]
    kept, previous, stopped = [], -1., None
    for index in range(begin, len(data)):
        source = data[index]
        local = source[0] - first[0]
        if local > 1. + 1e-6:
            stopped = dict(reason='one_second_limit', source_row=source)
            break
        if abs(source[4]) < .015 or source[4] * direction <= 0.:
            stopped = dict(reason='low_speed' if abs(source[4]) < .015 else 'gear_change', source_row=source)
            break
        if local - previous < .18 and index != begin:
            continue
        kept.append(dict(local_t_s=local, source_row=source))
        previous = local
    duration = kept[-1]['local_t_s'] if kept else None
    distance = math.hypot(kept[-1]['source_row'][1] - request['start_x'],
        kept[-1]['source_row'][2] - request['start_y']) if kept else None
    angle_change = math.atan2(math.sin(kept[-1]['source_row'][3] - first[3]),
        math.cos(kept[-1]['source_row'][3] - first[3])) if kept else None
    first_reverse = next((r for r in data if r[0] >= max(0., age) and r[4] * direction < -.01), None)
    delivery_target = max(data[0][0], max(0., age)) + 1.1
    return dict(seed_stamp=frozen['seed_stamp'], request_time=frozen['request_time'], seed_age_s=age,
        full_recorded_rows=len(data), full_recorded_prediction_end_s=data[-1][0],
        prefix_selected_source_begin_s=first[0], retained_rows=kept, retained_row_count=len(kept),
        retained_duration_s=duration, last_retained_distance_from_request_start_m=distance,
        retained_yaw_change_rad=angle_change, first_truncation=stopped,
        fails_row_count=len(kept) < 3, fails_minimum_duration=(duration is None or duration < .4),
        fails_minimum_displacement=(distance is None or distance < .06),
        first_reverse_row=first_reverse,
        same_gear_feedback_end_s=first_reverse[0] if first_reverse else data[-1][0],
        configured_handover_delay_s=1.1, handover_target_source_local_s=delivery_target,
        handover_target_beyond_first_reverse=(delivery_target >= first_reverse[0] - 1e-6) if first_reverse else False,
        scope='Arithmetic reproduction of existing prefix rules on saved request feedback. No exact prefix-entry packet capture is claimed. Handover same-gear clipping is distinct from the full safety-checked rollout horizon.')


def command_key(cycle):
    return (cycle['command_header_ros_time'] - 1000., cycle['guide_revision'], cycle['reason'],
            cycle['command_v'], cycle['command_w'])


def command_evidence(cycle, executions):
    key = command_key(cycle)
    matches = [r for r in executions if (r['t'], r['guide_revision'], r['reason'], *r['command']) == key]
    record = dict(cycle_id=cycle['cycle_id'], revision=cycle['guide_revision'],
        actual_command_elapsed_s=cycle['command_publish_after_ros_time'] - 1000.,
        command_v=cycle['command_v'], command_w=cycle['command_w'], exact_execution_matches=len(matches))
    if len(matches) == 1 and matches[0]['rollout']:
        execution = matches[0]
        rollout = np.asarray(execution['rollout'])
        record.update(selected_rollout_duration_s=float(rollout[-1, 0]),
            selected_rollout_start_xy=rollout[0, 1:3].tolist(), selected_rollout_end_xy=rollout[-1, 1:3].tolist(),
            selected_rollout_delta_xy=(rollout[-1, 1:3] - rollout[0, 1:3]).tolist(),
            selected_and_brake_validation=execution['validation'])
    return record


def main():
    names = ('dual_timeline.jsonl', 'temporal_collision.jsonl', 'static_width_trace.jsonl',
             'consumed_references.jsonl', 'state_mppi_cycles.jsonl', 'execution_rollouts.jsonl.gz', 'state.csv')
    input_hashes = {name: sha(TRIAL / name) for name in names}
    timeline, captures, initial = [rows(TRIAL / name) for name in names[:3]]
    consumed, cycles, executions = [rows(TRIAL / name) for name in names[3:6]]
    state = np.genfromtxt(TRIAL / 'state.csv', names=True, delimiter=',')
    finals = {r['request_id']: r for r in captures if r['stage'] == 'final_candidate'}
    plans = {r['request_id']: r for r in captures if r['stage'] == 'planning'}
    initial = {r['request_id']: r for r in initial if r['stage'] == 'minco_before_optimizer'}
    publishes = [r for r in timeline if r.get('event') == 'reference_publish']
    adopted_curves = []
    for request_id in (3, 5):
        final = finals[request_id]
        publication = next(r for r in publishes if r['request_id'] == request_id)
        reference = next(r for r in consumed if abs(r['stamp'] - publication['guide_start']) < 1e-7)
        polynomial = sample_curve(dict(parts=final['candidate']['parts']), reference['times'])
        error = float(np.max(np.linalg.norm(polynomial - np.asarray(reference['reference'])[:, :2], axis=1)))
        adopted_curves.append(dict(request_id=request_id, revision=reference['revision'],
            reference_sha256=reference['sha256'], native_success=final['native_success'],
            consumed_maximum_polynomial_error_m=error,
            initial_geometry=geometry(initial[request_id]), returned_geometry=geometry(final)))
    request5 = next(r for r in timeline if r.get('event') == 'request_begin' and r.get('request_id') == 5)
    frozen5 = next(r for r in timeline if r.get('event') == 'handoff_feedback_input' and r.get('request_id') == 5)
    feedback = prefix_check(frozen5, request5, plans[5])
    scope5 = next(r for r in timeline if r.get('event') == 'coupling_scope' and r.get('request_id') == 5)
    prefix_events = [r for r in timeline if r.get('event') == 'collab_prefix' and r.get('request_id') in (3, 4, 5)]
    trigger4 = next(r for r in timeline if r.get('event') == 'refresh_trigger' and r.get('source') == 'guide_blocked')
    request_events = [r for r in timeline if r.get('event') in ('request_begin', 'request_end') and r.get('request_id') in (4, 5)]
    valid = [r for r in cycles if r.get('validated_command_published') is True and r.get('validation_code') == 0]
    rev2 = [r for r in valid if r['guide_revision'] == 2]
    rev3 = [r for r in valid if r['guide_revision'] == 3]
    snapshots = [rev2[0], rev2[-1], rev3[0], min(rev3, key=lambda r: abs(r['command_publish_after_ros_time'] - 1034.))]
    selected_commands = [command_evidence(r, executions) for r in snapshots]
    begin = rev3[0]['command_publish_after_ros_time'] - 1000.
    loop = state[(state['t'] >= begin) & (state['x'] < 3.)]
    # Limit to the first crossing after revision 3, excluding later samples if
    # the robot were to return behind the crossing plane again.
    after = state[state['t'] >= begin]
    first_cross = np.flatnonzero(after['x'] >= 3.)
    if len(first_cross):
        loop = after[:first_cross[0] + 1]
    observed_loop = dict(begin_elapsed_s=begin, end_elapsed_s=float(loop['t'][-1]),
        minimum_x_m=float(np.min(loop['x'])), maximum_y_m=float(np.max(loop['y'])),
        minimum_y_m=float(np.min(loop['y'])),
        net_unwrapped_heading_change_rad=float(np.unwrap(loop['yaw'])[-1] - np.unwrap(loop['yaw'])[0]))
    report = dict(schema='delivery_midcross_route_side_audit_v1', trial=str(TRIAL.resolve()),
        source_sha256=input_hashes, script_sha256=sha(Path(__file__)),
        adopted_references=adopted_curves, guide_blocked_trigger=trigger4,
        q4_q5_request_events=request_events, q4_native_final_success=finals[4]['native_success'],
        q4_native_final_reason=finals[4]['native_reason'], prefix_events=prefix_events,
        q5_saved_feedback_prefix_rule_check=feedback, q5_actual_coupling_scope=scope5,
        command_samples=selected_commands, actual_revision_three_loop=observed_loop,
        scope='One existing online B run, recorded reference geometry and actual control/state only. No new solver, controller or collision replay; source files remain unchanged.')
    if any(sha(TRIAL / name) != digest for name, digest in input_hashes.items()):
        raise RuntimeError('an original input changed during analysis')
    upper, lower = [r['returned_geometry'] for r in adopted_curves]
    init5 = adopted_curves[1]['initial_geometry']
    text = '\n'.join([
        '中段横穿 B 组：绕行侧与绕圈核查', '',
        '结论：存在“先选 +y 侧，后改到 −y 侧”的路线变化，但这圈主要已经写进 REMANI 新参考里，不能归因成 MPPI 不跟踪一条直接下绕的轨迹。', '',
        '1. q3 实际发布并采用为参考 rev2，x=3 米处 y=%.3f 米，选择 +y 绕行；机器人随后也向 +y 走。' % upper['y_at_all_x3_crossings_m'][0],
        '2. 约 %.3f 秒时，原参考在当前预测下被判受阻，模型间隙 %.2f cm，触发 q4。q4 最终因 %s 被拒；旧参考继续存在，随后重试 q5。' % (
            trigger4['sim'] - 1000., 100 * trigger4['nominal_gap'], finals[4]['native_reason']),
        '3. q5 实际发布并采用为 rev3，x=3 米处 y=%.3f 米，后段最低 y=%.3f 米，已换到 −y 侧；但开头并不直接向下。' % (
            lower['y_at_all_x3_crossings_m'][0], lower['minimum_y_m']),
        '   q5 优化前 MINCO 初值已从 x=%.3f/y=%.3f 的起点，先抬到 y=%.3f，并回到 x=%.3f，后面才接下绕；优化后放大为最高 y=%.3f、最小 x=%.3f。' % (
            init5['start_xy'][0], init5['start_xy'][1], init5['maximum_y_m'], init5['minimum_x_m'],
            lower['maximum_y_m'], lower['minimum_x_m']),
        '   因此回头形状在送入优化器之前已经存在，后端进一步放大。现有记录能定位到初值构造链，尚不能仅凭 MINCO 曲线再细分是原始 A* 节点还是节点转多项式时引入。',
        '4. 两条发布参考与实际消费曲线最大误差分别为 %.3g 和 %.3g 米，均确认为同一条曲线。q5 新参考采用后，实际先到最高 y=%.3f、最小 x=%.3f，与参考里的圈基本一致。' % (
            adopted_curves[0]['consumed_maximum_polynomial_error_m'], adopted_curves[1]['consumed_maximum_polynomial_error_m'],
            observed_loop['maximum_y_m'], observed_loop['minimum_x_m']),
        '   q5 换参考前最后一条有效控制为 v=%.3f、角速度=%.3f；新参考第一条有效控制改为 v=%.3f、角速度=%.3f。其获准展开先向上、向后侧走，后续再转向下，支持 MPPI 在执行这条带圈参考。' % (
            rev2[-1]['command_v'], rev2[-1]['command_w'], rev3[0]['command_v'], rev3[0]['command_w']), '',
        '已有 MPPI 反馈为什么没挡住这次回头',
        'q5 前缀功能已启用，但实际事件是 used=false / short_prefix；不能把下一步写成“再增加一个 MPPI 前缀”。',
        '按保存的 q5 预测包和请求时刻核对：原包有 %d 行、持续 %.1f 秒；前缀规则遇到源时间 %.1f 秒的低速 %.5f 米/秒就截断，稀疏保留后只有 %d 行、%.2f 秒、末点离起点约 %.2f cm。行数、时长和位移都不合格，不只是 6 cm 门槛。' % (
            feedback['full_recorded_rows'], feedback['full_recorded_prediction_end_s'], feedback['first_truncation']['source_row'][0],
            feedback['first_truncation']['source_row'][4], feedback['retained_row_count'], feedback['retained_duration_s'],
            100 * feedback['last_retained_distance_from_request_start_m']),
        '固定 1.1 秒交接目标对应源时间 %.3f 秒，而预测在 %.1f 秒已开始反向；同档位反馈区间因此截到该反向点。实际 coupling_scope 为 valid=false / delivery_outside_checked_interval，所以 q5 的交接软项并未生效。' % (
            feedback['handover_target_source_local_s'], feedback['same_gear_feedback_end_s']),
        '这不是原包只有半秒预测或安全验证时域只有半秒；原包持续 2.8 秒，是同档位规则在换向处截断。以上是保存包的条件核对，不冒充另有精确前缀入口快照。', '',
        '对下一步的指向',
        '优先处理低平移、制动或换向时，现有前缀和交接反馈一起失效后，前端怎样构造和连接候选；检查已出现上绕姿态时，为何下绕候选先接出一大圈。',
        '保留当前双层框架，先针对这一退化分支做小改；不需要重新增加控制器、继续延长预测，也不宜直接放松前缀门槛。',
        '本次能确认路线换侧、反馈退化与带圈初值同时存在，且实际执行与带圈参考一致；尚不能从单次异步 A/B 断定所有额外耗时都由其中单一因素造成。', ''])
    output = ROOT / 'report'
    (output / '绕行侧核查.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    (output / '绕行侧核查.txt').write_text(text)
    print(json.dumps(dict(output=str(output), q3_y_at_x3=upper['y_at_all_x3_crossings_m'],
        q5_y_at_x3=lower['y_at_all_x3_crossings_m'], q5_initial_min_x=init5['minimum_x_m'],
        q5_final_min_x=lower['minimum_x_m'], prefix_retained_rows=feedback['retained_row_count'],
        prefix_duration_s=feedback['retained_duration_s'], prefix_displacement_m=feedback['last_retained_distance_from_request_start_m']), ensure_ascii=False))


if __name__ == '__main__':
    main()

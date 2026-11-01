#!/usr/bin/env python3
"""Read one finished head-on A/B; report soft MPPI feedback and actual delivery.

No controller/optimizer is replayed.  Frozen targets use the rollout source
clock, including its age at request time.  A spatial soft match is never
reported as a fixed-time rendezvous or a committed MPPI prefix.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from progress import project
from render_dual_timescale import FONT
from report_frontend_rejoin import keyframes
from report_static_agility import gif, low_speed, show
from report_time_indexed_collision import OUTCOMES, TERMINATIONS, inspect


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/delivery_feedback_20261031_01'
ROS_ORIGIN = 1000.  # Recorded numerical-SIL epoch; state.csv uses elapsed time.


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def measured(state, t):
    """Interpolate only between recorded state rows; never extrapolate."""
    if not len(state) or t < float(state['t'][0]) or t > float(state['t'][-1]):
        return None
    right = int(np.searchsorted(state['t'], t, side='right'))
    right = min(max(1, right), len(state)-1)
    a, b = state[right-1], state[right]
    dt = float(b['t']-a['t'])
    u = (t-float(a['t']))/dt if dt > 0 else 0.
    out = {k: float(a[k]+u*(b[k]-a[k])) for k in ('x', 'y', 'v', 'w')}
    out['yaw'] = float(a['yaw'])+u*angle(float(b['yaw']-a['yaw']))
    out.update(t=t, interpolation_bracket_s=[float(a['t']), float(b['t'])])
    return out


def interpolate_rollout(rows, t):
    if len(rows) < 2 or t < rows[0][0] or t > rows[-1][0]:
        return None
    right = int(np.searchsorted([r[0] for r in rows], t, side='left'))
    right = min(max(1, right), len(rows)-1)
    a, b = rows[right-1], rows[right]
    u = (t-a[0])/(b[0]-a[0])
    return dict(x=a[1]+u*(b[1]-a[1]), y=a[2]+u*(b[2]-a[2]),
        yaw=a[3]+u*angle(b[3]-a[3]), v=a[4]+u*(b[4]-a[4]),
        w=a[5]+u*(b[5]-a[5]), source_local_t_s=t)


def errors(reference, actual):
    if reference is None or actual is None:
        return None
    return dict(position_m=math.hypot(reference['x']-actual['x'], reference['y']-actual['y']),
        yaw_rad=angle(reference['yaw']-actual['yaw']),
        yaw_abs_deg=abs(math.degrees(angle(reference['yaw']-actual['yaw']))),
        signed_speed_mps=reference['v']-actual['v'],
        velocity_vector_mps=math.hypot(reference['v']*math.cos(reference['yaw'])-actual['v']*math.cos(actual['yaw']),
            reference['v']*math.sin(reference['yaw'])-actual['v']*math.sin(actual['yaw'])))


def curve_state(candidate, local_t):
    pieces = [(part['gear'], piece['T'], piece['coeffs'])
              for part in candidate['parts'] for piece in part['pieces']]
    t = float(local_t)
    for index, (gear, duration, coeffs) in enumerate(pieces):
        if t <= duration+1e-9 or index == len(pieces)-1:
            t = min(max(0., t), duration)
            p = [float(np.polyval(row, t)) for row in coeffs[:2]]
            v = [float(np.polyval(np.polyder(row), t)) for row in coeffs[:2]]
            return dict(x=p[0], y=p[1], yaw=math.atan2(gear*v[1], gear*v[0]),
                        v=gear*math.hypot(*v), local_t_s=local_t, gear=gear)
        t -= duration
    raise ValueError('captured candidate has no polynomial pieces')


def delivery(commands, requests):
    refresh = [r for r in requests if r['request_id'] > 1]
    result = dict(first_refresh_to_new_reference_kernel_wall_ms=None,
        first_refresh_to_new_reference_validated_command_wall_ms=None,
        definition='Earliest request>1 request_begin.monotonic to new revision>1 kernel entry / validated command publication after.monotonic.')
    if not refresh:
        return result
    began = min(r['start_mono'] for r in refresh)
    for mode, event, flag in (
            ('kernel', 'online_step_entry_monotonic', 'online_step_invoked'),
            ('validated_command', 'command_publish_after_monotonic', 'validated_command_published')):
        selected = [c for c in commands if (c.get('guide_revision') or 0) > 1
                    and c.get(flag) and c.get(event) is not None and c[event] >= began]
        if selected:
            first = min(selected, key=lambda c: c[event])
            result['first_refresh_to_new_reference_'+mode+'_wall_ms'] = 1000*(first[event]-began)
            result['first_new_'+mode+'_cycle'] = first
    return result


def feedback(timeline, captures, state):
    scopes = [r for r in timeline if r['event'] == 'coupling_scope' and r['request_id'] > 1]
    inputs = [r for r in timeline if r['event'] == 'handoff_feedback_input' and r['request_id'] > 1]
    costs = [r for r in timeline if r['event'] == 'coupling_cost' and r['request_id'] > 1]
    details = []
    for scope in scopes:
        inputs_here = [r for r in inputs if r['scope_id'] == scope['scope_id']]
        cost_here = [r for r in costs if r['scope_id'] == scope['scope_id']]
        item = dict(scope=scope, costs=cost_here,
            cost_or_gradient_nonzero=any(abs(r.get('delivery_cost', 0.)) > 1e-12
                or abs(r.get('gradient_norm', 0.)) > 1e-12 for r in cost_here),
            initial=next((r for r in cost_here if r.get('phase') == 'initial'), None),
            final=next((r for r in reversed(cost_here) if r.get('phase') == 'final'), None),
            frozen_input_count=len(inputs_here))
        if len(inputs_here) != 1:
            item['target_unavailable_reason'] = 'missing_or_ambiguous_frozen_input'
            details.append(item)
            continue
        frozen = inputs_here[0]
        rows = frozen['frozen_rows']
        item['frozen_input'] = frozen
        if len(rows) < 2:
            item['target_unavailable_reason'] = 'short_frozen_rollout'
            details.append(item)
            continue
        age = frozen['request_time']-frozen['seed_stamp']
        begin = max(rows[0][0], max(0., age))
        target_local = begin+scope['delivery_delay_s']
        # The bridge compares on this source-relative clock, not t=delay from
        # the source and not t=request_time+delay when the source is in future.
        target_epoch = frozen['seed_stamp']+target_local
        target = interpolate_rollout(rows, target_local)
        plan = next((r for r in captures if r['stage'] == 'planning'
            and r['request_id'] == scope['request_id']
            and str(r.get('input_hash')) == str(scope.get('input_hash'))), None)
        gear = plan['gears'][0] if plan and plan.get('gears') else None
        checked_end = rows[-1][0]
        if gear is not None:
            for r in rows:
                if r[0] >= begin and r[4]*gear < -.01:
                    checked_end = r[0]
                    break
        actual_target = measured(state, target_epoch-ROS_ORIGIN)
        actual_request = measured(state, frozen['request_time']-ROS_ORIGIN)
        origin = interpolate_rollout(rows, begin)
        item['target'] = dict(seed_age_at_request_s=age, checked_begin_s=rows[0][0],
            checked_end_after_same_gear_clip_s=checked_end, native_initial_gear=gear,
            source_relative_begin_s=begin, target_source_relative_t_s=target_local,
            target_absolute_ros_s=target_epoch, target_elapsed_s=target_epoch-ROS_ORIGIN,
            target_ahead_of_request_s=target_epoch-frozen['request_time'],
            inside_same_gear_checked_interval=target_local < checked_end-1e-6,
            predicted=target, actual_at_target_time=actual_target,
            prediction_error_at_target_time=errors(target, actual_target),
            predicted_request_origin=origin, actual_at_request_time=actual_request,
            request_origin_error=errors(origin, actual_request),
            scope='Frozen checked rollout versus later actual measured state; prediction is not an execution commitment.')
        details.append(item)
    return dict(scopes=details, scope_count=len(scopes),
        valid_scope_count=sum(r.get('valid') is True for r in scopes),
        reasons=dict(Counter(r.get('reason') for r in scopes)),
        nonzero_cost_or_gradient_scope_count=sum(r['cost_or_gradient_nonzero'] for r in details),
        valid_with_nonzero_count=sum(r['scope'].get('valid') is True and r['cost_or_gradient_nonzero'] for r in details),
        missing_frozen_input_count=sum(r['frozen_input_count'] != 1 for r in details),
        definition='Refresh optimizers only; configured weight does not establish an active objective.')


def scalar_time(state, field, start, end):
    selected = state[(state['t'] >= start) & (state['t'] <= end)]
    values = np.asarray(selected[field], dtype=float)
    return dict(min=float(np.min(values)) if len(values) else None,
                max=float(np.max(values)) if len(values) else None,
                count=len(values))


def following_window(trial, commands, ref):
    start, end = float(ref['t']), min(trial.end, float(ref['t'])+2.)
    _, goal, _ = __import__('report_long_goal_crossing').settings(trial.manifest)
    selected = [c for c in commands if start <= c['command_ros_time']-ROS_ORIGIN <= end]
    before = [c for c in commands if c['command_ros_time']-ROS_ORIGIN < start]
    around = before[-1:]+selected
    dv = [abs(b['command_v']-a['command_v']) for a, b in zip(around, around[1:])]
    dw = [abs(b['command_w']-a['command_w']) for a, b in zip(around, around[1:])]
    reverse_s = 0.
    for a, b in zip(trial.state, trial.state[1:]):
        lo, hi = max(start, float(a['t'])), min(end, float(b['t']))
        if hi > lo and a['v'] < -.02:
            reverse_s += hi-lo
    return dict(start_elapsed_s=start, end_elapsed_s=end, duration_s=end-start,
        measured_speed_mps=scalar_time(trial.state, 'v', start, end),
        measured_yaw_rate_radps=scalar_time(trial.state, 'w', start, end),
        measured_non_goal_low_speed=low_speed(trial.state, start, end, goal),
        measured_reverse_duration_s=reverse_s,
        first_command_after_adoption=selected[0] if selected else None,
        last_command_before_adoption=before[-1] if before else None,
        maximum_command_delta_v_mps=max(dv) if dv else None,
        maximum_command_delta_w_radps=max(dw) if dw else None,
        command_reason_counts=dict(Counter(c['reason'] for c in selected)),
        validated_commands=sum(c['validated_command_published'] for c in selected),
        command_count=len(selected),
        definition='First 2 measured seconds after actual reference consumption; |v|<.02 and |w|<.05 defines low speed, v<-.02 defines reverse.')


def adoption_diagnostics(row, trial, commands, timeline, captures):
    plans = {r['plan_ordinal']: r for r in captures if r['stage'] == 'planning'}
    references = trial.reader.jsonl('consumed_references.jsonl')
    publications = [r for r in timeline if r['event'] == 'reference_publish']
    phases = [r for r in timeline if r['event'] == 'candidate_phase_alignment']
    result = []
    for ref in references:
        if ref.get('revision', 0) <= 1 or ref['t'] > trial.end:
            continue
        pub = [r for r in publications if abs(r['guide_start']-ref['stamp']) < 1e-7]
        if len(pub) != 1:
            result.append(dict(revision=ref['revision'], reason='publication_not_unique'))
            continue
        request_id = pub[0]['request_id']
        ac = [r for r in captures if r['stage'] == 'acceptance' and r['request_id'] == request_id
              and abs(r['candidate']['start_epoch']-ref['stamp']) < 1e-7]
        actual = measured(trial.state, ref['t'])
        reference, times = np.asarray(ref['reference'], dtype=float), np.asarray(ref['times'], dtype=float)
        item = dict(request_id=request_id, revision=ref['revision'], epoch=ref.get('epoch'),
            consumption_elapsed_s=ref['t'], consumption_monotonic=ref.get('cycle_start_monotonic'),
            reference_sha256=ref.get('sha256'), measured_at_consumption=actual,
            acceptance_phase_hook=next((r for r in reversed(phases) if r['request_id'] == request_id), None))
        if actual is not None and len(ac) == 1:
            phase, scale, polyline_residual = project(
                np.asarray([actual[k] for k in ('x', 'y', 'yaw', 'v')]), reference, times, 0.)
            candidate = ac[0]['candidate']
            nominal = curve_state(candidate, phase)
            # Reference time is a geometric association; no equality with the
            # actual wall clock or the 1.1-second target is assumed here.
            item.update(candidate_plan_ordinal=ac[0]['plan_ordinal'],
                candidate_initial_gear=plans[ac[0]['plan_ordinal']]['gears'][0],
                projected_phase_local_t_s=phase, projected_consumer_scale=scale,
                reference_polyline_position_residual_m=polyline_residual,
                candidate_at_projected_phase=nominal,
                adoption_error=errors(nominal, actual),
                candidate_start_epoch=candidate['start_epoch'],
                candidate_age_at_consumption_s=ROS_ORIGIN+ref['t']-candidate['start_epoch'])
        else:
            item['projection_unavailable_reason'] = 'actual_or_unique_acceptance_missing'
        item['following_2s'] = following_window(trial, commands, ref)
        result.append(item)
    return result


def load_trial(source, label):
    row, trial = inspect(source, label)
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    captures = trial.reader.jsonl('temporal_collision.jsonl')
    commands = trial.reader.jsonl('state_mppi_cycles.jsonl')
    row['control_delivery'] = delivery(commands, row['remani']['requests'])
    row['feedback'] = feedback(timeline, captures, trial.state)
    row['adoption_diagnostics'] = adoption_diagnostics(row, trial, commands, timeline, captures)
    row['feedback_settings'] = {k: v for k, v in trial.manifest.items()
        if k in ('coupling_experiment', 'delivery_feedback', 'coverage_alignment', 'delivery_feedback_experiment')}
    row['first_adoption'] = next((r for r in row['adoption_diagnostics'] if r.get('adoption_error')), None)
    row['first_feedback_scope'] = next((r for r in row['feedback']['scopes']
        if r.get('target') and r['scope']['request_id'] == row['delivery'].get('first_adopted_request_id')), None)
    for scope in row['feedback']['scopes']:
        target = scope.get('target')
        if target:
            prior_adoptions = [r for r in row['adoption_diagnostics']
                if r.get('consumption_elapsed_s') is not None
                and scope['frozen_input']['request_time']-ROS_ORIGIN <= r['consumption_elapsed_s'] <= target['target_elapsed_s']]
            target['new_reference_adopted_before_target_time'] = bool(prior_adoptions)
            target['prior_adoption_elapsed_s'] = [r['consumption_elapsed_s'] for r in prior_adoptions]
    row['input_sha256'] = trial.reader.hashes
    trial.verify_unchanged()
    return row, trial


def f(value, digits=2):
    return show(value, digits)


def brief(rows):
    lines = ['MPPI交付状态软反馈：迎面场景A/B实验结果', '',
        'A：上一轮已修正预测覆盖与候选相位的版本，反馈权重0。',
        'B：同一版本，只启用交付状态软反馈，权重1；执行空间项权重仍为0，预测目标取请求后约1.1秒。',
        '每组只跑一次，同一次记录求解、失败、交付、预测误差、交接和运动情况。',
        '反馈让新轨迹前段在空间上靠近冻结MPPI预测的位置、朝向和速度；它没有约束必须在1.1秒恰好到达该状态。', '']
    for row in rows:
        fb, ad = row['feedback'], row['first_adoption']
        cmd = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        lines += [row['label'],
            '结果：%s；%s物理球采样接触，最小间隙%s厘米；实走%s米。' % (
                '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination']),
                '发生' if row['physical_sample_collision'] else '未发现',
                f(100*row['minimum_physical_sample_gap_m']), f(row['actual_path_length_m'], 3)),
            '首次重规划请求到新参考首条有效控制：%s；成功交付那次请求：%s。' % (
                f(cmd/1000, 3)+'秒' if cmd is not None else '未交付',
                f(row['delivery']['first_adopted_request_wall_ms']/1000, 3)+'秒'
                if row['delivery']['first_adopted_request_wall_ms'] is not None else '未交付'),
            '反馈：重规划优化器%d次，反馈作用域启用%d次，有效且出现非零代价或梯度%d次。原因%s。' % (
                fb['scope_count'], fb['valid_scope_count'], fb['valid_with_nonzero_count'],
                json.dumps(fb['reasons'], ensure_ascii=False)),
            'MPPI完整循环P95/最大%s/%s毫秒；超过200毫秒%d次。' % (
                f(row['callback']['p95_ms'], 1), f(row['callback']['max_ms'], 1), row['callback']['over_200_ms'])]
        if fb['reasons'].get('zero_weights'):
            lines.append('权重为0时反馈作用域不启用；这不代表冻结预测输入无效。')
        for request in row['remani']['requests']:
            if request['request_id'] <= 1:
                continue
            request_ms = request.get('request_wall_ms')
            lines.append('重规划请求%d：%s，耗时%s秒；优化/硬检查原因%s，发布拒绝%s。' % (
                request['request_id'], OUTCOMES.get(request.get('outcome_class'), request.get('outcome_class')),
                f(request_ms/1000, 3) if request_ms is not None else '未结束',
                [c.get('reason') for c in request.get('optimizer_calls', [])], request.get('acceptance_rejections', [])))
        for scope in fb['scopes']:
            target = scope.get('target')
            e = target.get('prediction_error_at_target_time') if target else None
            if e:
                lines.append('请求%d冻结目标：来源龄%s秒，目标在请求后%s秒；与目标时刻实际状态差%s厘米、%s度、速度差%s米/秒。' % (
                    scope['scope']['request_id'], f(target['seed_age_at_request_s'], 3),
                    f(target['target_ahead_of_request_s'], 3), f(100*e['position_m']),
                    f(e['yaw_abs_deg']), f(e['signed_speed_mps'], 3)))
        first_fb = row['first_feedback_scope']
        if first_fb:
            target = first_fb['target']
            e = target['prediction_error_at_target_time']
            if e:
                lines.append('交付请求冻结预测：来源已过去%s秒；约1.1秒后目标与当时实际位置差%s厘米、朝向差%s度、速度差%s米/秒。' % (
                    f(target['seed_age_at_request_s'], 3), f(100*e['position_m']),
                    f(e['yaw_abs_deg']), f(e['signed_speed_mps'], 3)))
                if target['new_reference_adopted_before_target_time']:
                    lines.append('该目标时刻已在新参考采用之后，实际运动受新参考影响；上述误差包含策略改变，不能全部当成旧预测失效。')
        if ad:
            e, after = ad['adoption_error'], ad['following_2s']
            lines += ['实际采用新参考时：位置差%s厘米、朝向差%s度、参考速度减实测速度%s米/秒。' % (
                f(100*e['position_m']), f(e['yaw_abs_deg']), f(e['signed_speed_mps'], 3)),
                '采用后2秒：实测速度范围[%s,%s]米/秒；低速停留%s秒，倒退%s秒；相邻控制最大变化Δv=%s米/秒、Δω=%s弧度/秒。' % (
                    f(after['measured_speed_mps']['min'], 3), f(after['measured_speed_mps']['max'], 3),
                    f(after['measured_non_goal_low_speed']['total_s'], 3), f(after['measured_reverse_duration_s'], 3),
                    f(after['maximum_command_delta_v_mps'], 3), f(after['maximum_command_delta_w_radps'], 3))]
            before_cmd, after_cmd = after['last_command_before_adoption'], after['first_command_after_adoption']
            if before_cmd and after_cmd:
                lines.append('采用前末条→采用后首条控制：v=%+.3f→%+.3f米/秒，ω=%+.3f→%+.3f弧度/秒；没有停顿不代表方向衔接完全平滑。' % (
                    before_cmd['command_v'], after_cmd['command_v'], before_cmd['command_w'], after_cmd['command_w']))
        else:
            lines.append('没有实际采用新参考；交接残差和采用后2秒指标无记录，不能按0统计。')
        lines.append('')
    a, b = rows
    ae = a['first_adoption']['adoption_error'] if a['first_adoption'] else None
    be = b['first_adoption']['adoption_error'] if b['first_adoption'] else None
    lines += ['怎么看这轮结果',
        '先确认B反馈是否有效进入优化，再比较交接残差、运动和交付；不能只看权重已经开启。']
    lines += ['本轮没有证明软反馈解决了稳定交付：B仍在第三次重规划才成功。无反馈版本上一轮通过、本轮却失败，也说明一次通过不足以判断稳定。']
    lines += ['B前两次失败已进一步核对：同一最终候选在常速动态检查中有正间隙，但按半速执行会与预测障碍相交，因此被拒；这两次不是预算超时或最终轮速不合格。详见“失败原因核对.txt”。']
    if b['feedback']['valid_with_nonzero_count'] == 0:
        lines.append('本轮B未证实有效的非零反馈，不能根据闭环差异评价软反馈效果。')
    elif ae and be:
        lines.append('B相对A：采用时位置差变化%s厘米，朝向绝对差变化%s度，速度绝对差变化%s米/秒（负值表示减小）。' % (
            f(100*(be['position_m']-ae['position_m'])), f(be['yaw_abs_deg']-ae['yaw_abs_deg']),
            f(abs(be['signed_speed_mps'])-abs(ae['signed_speed_mps']), 3)))
    elif be and not ae:
        lines.append('本次B交付了新参考，A没有交付，因此不能给出A/B交接残差改善百分比。B仍需结合前几次失败判断交付稳定性。')
    elif not be:
        lines.append('本次B没有实际采用新参考，尚不能证明该反馈改善了在线交接。')
    lines += ['朝向残差本身不等于已发生运动故障；应连同采用后2秒的速度、停顿和控制变化判断。',
        '1.1秒是预先选定的预测目标时间；实际求解可快可慢。空间软匹配没有强制两者相等，目标与真正交付时状态仍可能不一致。',
        '相同场景和随机种子不等于相同在线输入；异步ROS消息、调度与控制会让两组进入重规划时的状态不同。',
        '本轮只有同场景A/B各一次，运行状态和求解耗时可受调度影响，不据此宣称总体成功率或彻底解决协同问题。',
        '固定机械臂、完整REMANI与底盘MPPI的ROS数值闭环，非Gazebo或实车。',
        'GIF绿色是获准局部展开，不是全部MPPI搜索；完整球投影保留。详细数据在summary.json。']
    return '\n'.join(lines)+'\n'


def chart(rows, trials, output):
    image = Image.new('RGB', (1250, 820), 'white')
    draw = ImageDraw.Draw(image)
    title, text, small = [ImageFont.truetype(FONT, n) for n in (23, 18, 15)]
    colors = ('#2874a6', '#c47723')
    draw.text((20, 12), '交付状态软反馈：交付是否及时，新轨迹是否好接', font=title, fill='#333333')
    draw.text((20, 48), '同一迎面场景，A/B各一次；下方按“实际采用新参考”对齐，查看后续2秒。', font=small, fill='#666666')
    for index, row in enumerate(rows):
        top = 86+145*index
        draw.line((20, top-8, 1230, top-8), fill='#dddddd')
        draw.text((20, top), row['label'], font=text, fill=colors[index])
        ms = row['control_delivery']['first_refresh_to_new_reference_validated_command_wall_ms']
        draw.text((20, top+29), '首次请求→有效控制 %s；实走 %.3f m；最小物理间隙 %.2f cm' % (
            f(ms/1000, 3)+' s' if ms is not None else '无交付', row['actual_path_length_m'],
            100*row['minimum_physical_sample_gap_m']), font=text, fill='#444444')
        ad = row['first_adoption']
        e = ad['adoption_error'] if ad else None
        draw.text((20, top+62), '采用时残差：'+('位置 %.2f cm，朝向 %.2f°，速度 %+.3f m/s' % (
            100*e['position_m'], e['yaw_abs_deg'], e['signed_speed_mps']) if e else '无可用记录'),
            font=small, fill='#555555')
        draw.text((20, top+89), '有效非零反馈 %d/%d 次；MPPI完整循环P95/最大 %s/%s ms；%s' % (
            row['feedback']['valid_with_nonzero_count'], row['feedback']['scope_count'],
            f(row['callback']['p95_ms'], 1), f(row['callback']['max_ms'], 1),
            '到达目标' if row['reached_goal'] else TERMINATIONS.get(row['termination'], row['termination'])),
            font=small, fill='#555555')
    for axis, (field, name) in enumerate((('v', '实测线速度 / m/s'), ('w', '实测角速度 / rad/s'))):
        left, right = 80+axis*610, 590+axis*610
        top, bottom = 430, 725
        values = []
        for row, trial in zip(rows, trials):
            if row['first_adoption']:
                s = row['first_adoption']['consumption_elapsed_s']
                values += [float(v) for v in trial.state[field][(trial.state['t'] >= s) & (trial.state['t'] <= s+2.)]]
        limit = max([.1]+[abs(v) for v in values])*1.12
        def xy(t, value):
            return left+t/2.*(right-left), (top+bottom)/2-value/limit*(bottom-top)/2
        draw.text((left, top-33), name, font=text, fill='#333333')
        for value in (-limit, 0., limit):
            yy = xy(0., value)[1]
            draw.line((left, yy, right, yy), fill='#e5e5e5')
            draw.text((left-53, yy-8), '%.2f' % value, font=small, fill='#777777')
        for index, (row, trial) in enumerate(zip(rows, trials)):
            if not row['first_adoption']:
                continue
            start = row['first_adoption']['consumption_elapsed_s']
            selected = trial.state[(trial.state['t'] >= start) & (trial.state['t'] <= min(trial.end, start+2.))]
            points = [xy(float(r['t'])-start, float(r[field])) for r in selected]
            if len(points) > 1:
                draw.line(points, fill=colors[index], width=3)
        for tick in range(5):
            xx = xy(tick*.5, 0.)[0]
            draw.text((xx-12, bottom+8), '%.1f' % (tick*.5), font=small, fill='#777777')
        draw.rectangle((left, top, right, bottom), outline='#cccccc')
    draw.text((20, 766), '横轴：实际采用后 / s；A未交付，故只有B曲线。反馈为空间软匹配，没有承诺固定时刻精确接合。', font=small, fill='#555555')
    draw.text((20, 796), '固定臂ROS数值闭环；完整保留失败，单次对照不代表总体成功率。', font=small, fill='#777777')
    image.save(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--changed', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--max-frames', type=int, default=100)
    args = parser.parse_args()
    selected = [load_trial(args.baseline or args.root/'headon_A', 'A：当前版本，反馈关闭'),
                load_trial(args.changed or args.root/'headon_B', 'B：增加1.1秒交付状态软反馈')]
    rows, trials = zip(*selected)
    output = args.out or args.root/'report'
    output.mkdir(parents=True, exist_ok=True)
    chart(rows, trials, output/'交付与衔接对照.png')
    animation = output/'迎面动态对照.gif'
    gif_rows = [dict(row, termination=TERMINATIONS.get(row['termination'], row['termination'])) for row in rows]
    artifacts = gif(gif_rows, trials, animation, args.max_frames)
    keyframes(animation, output/'迎面动态关键帧.png')
    (output/'实验结果.txt').write_text(brief(rows), encoding='utf8')
    (output/'summary.json').write_text(json.dumps(dict(rows=rows, artifacts=artifacts,
        scope='One recorded fixed-arm numerical ROS SIL head-on A/B. No controller replay.',
        time_conventions=dict(state_csv_origin_ros_s=ROS_ORIGIN,
            frozen_target='seed_stamp + max(first_source_local_t, max(0, request_time-seed_stamp)) + delivery_delay',
            adoption='Geometric initial previous=0 projection of measured pose; candidate polynomial evaluated at that associated phase.',
            wall_delivery='First refresh monotonic request begin to new-reference validated command publication after.')),
        ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    for trial in trials:
        trial.verify_unchanged()
    print('DELIVERY_FEEDBACK_REPORT', output, flush=True)


if __name__ == '__main__':
    main()

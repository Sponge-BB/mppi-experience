#!/usr/bin/env python3
"""Read the completed two-run execution-timing comparison; never rerun control."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from render_native_collision_3d import Trial as NativeTrial, draw_panel, truth_report
from report_native_dynamic_fast import deadline_zero_duration, delivery_metrics, timing_by_phase, show


GROUPS = ('current', 'matched')
LABELS = {'current': '当前：50 / 75 ms', 'matched': '匹配：200 / 250 ms'}
TIMING = {'current': (.05, .075), 'matched': (.2, .25)}


class Trial(NativeTrial):
    def __init__(self, source, group):
        super().__init__(source, 'native3d')
        timing = self.manifest.get('execution_timing_pair', {})
        if group not in GROUPS or timing.get('group') != group:
            raise ValueError('execution_timing_pair group does not match panel')
        deadline, lease = TIMING[group]
        if not math.isclose(timing.get('callback_deadline_s', -1), deadline) or not math.isclose(timing.get('command_lease_s', -1), lease):
            raise ValueError('recorded execution timing does not match group')
        if self.manifest.get('native_dynamic_fast', {}).get('enabled') is not True or self.manifest.get('native_realtime', {}).get('demand') is not True:
            raise ValueError('both groups require fast and demand enabled')
        if self.status.get('result_written') is not True:
            raise ValueError('finish trial before reporting')
        self.group, self.title = group, LABELS[group]


def validated_actions(rows):
    """A validated proposal is not a published or physically executed action."""
    checked = [r for r in rows if r.get('reference_points', 0) > 0 and r.get('validation_code') == 0]
    known = [r for r in checked if r.get('validated_proposal_v') is not None and r.get('validated_proposal_w') is not None]
    nonzero = [r for r in known if abs(r['validated_proposal_v'])+abs(r['validated_proposal_w']) > 1e-9]
    issued = [r for r in nonzero if r.get('authorized') and r.get('reason') == 'ok' and
              abs(r.get('applied_v', 0))+abs(r.get('applied_w', 0)) > 1e-9]
    return dict(validated_cycles=len(checked), proposal_recorded_cycles=len(known),
        proposal_missing_cycles=len(checked)-len(known), validated_nonzero_proposals=len(nonzero),
        nonzero_proposals_published=len(issued),
        validated_nonzero_publication_ratio=len(issued)/len(nonzero) if nonzero else None,
        basis='validated selected proposal before deadline clearing versus published nonzero command',
        physical_execution_is_separate_measured_state_metric=True)


def measured_stops(state, start, end):
    """Integrate measured low-speed intervals, clipped to the obstacle window."""
    if 'w' not in (state.dtype.names or ()):
        return dict(available=False, total_s=None, longest_s=None)
    total, longest, streak = 0., 0., 0.
    intervals = []
    streak_start = None
    for index in range(len(state)-1):
        row = state[index]
        lo, hi = max(start, float(row['t'])), min(end, float(state[index+1]['t']))
        if hi <= lo:
            continue
        stopped = abs(row['v']) < .01 and abs(row['w']) < .03
        if stopped:
            if streak_start is None:
                streak_start = lo
            streak += hi-lo
            total += hi-lo
            longest = max(longest, streak)
        elif streak_start is not None:
            intervals.append(dict(start_s=streak_start, end_s=lo, duration_s=streak))
            streak_start, streak = None, 0.
    if streak_start is not None:
        intervals.append(dict(start_s=streak_start, end_s=min(end, float(state['t'][-1])), duration_s=streak))
    return dict(available=True, window_start_s=start, window_end_s=end,
        total_s=total, longest_s=longest, intervals=intervals,
        definition='measured |v| < 0.01 m/s and |w| < 0.03 rad/s; left-sample intervals',
        zero_command_is_not_measured_stop=True)


def maneuver_events(trial, actor_time):
    """Describe actual sideways travel, without asserting its control cause."""
    if actor_time is None:
        return dict(first_actual_sideways_t=None, first_refresh_consumed_t=None,
                    sideways_before_refresh=None, sideways_minus_refresh_s=None,
                    sideways_after_actor_s=None)
    before = trial.state[trial.state['t'] <= actor_time]
    baseline_y = float(before[-1]['y']) if len(before) else float(trial.state[0]['y'])
    rows = trial.state[(trial.state['t'] >= actor_time) & (trial.state['t'] <= trial.end)]
    sideways = rows[np.abs(rows['y']-baseline_y) > .05]
    first_side = float(sideways[0]['t']) if len(sideways) else None
    initial = trial.refs[0] if trial.refs else None
    refreshed = [r for r in trial.refs[1:] if r['t'] >= actor_time and
                 (r.get('sha256') != initial.get('sha256') if initial and initial.get('sha256') else
                  r.get('revision') != initial.get('revision'))]
    refresh = min((float(r['t']) for r in refreshed), default=None)
    return dict(actor_first_truth_t=actor_time, actor_baseline_y_m=baseline_y,
        first_actual_sideways_t=first_side, first_refresh_consumed_t=refresh,
        sideways_after_actor_s=first_side-actor_time if first_side is not None else None,
        sideways_minus_refresh_s=first_side-refresh if first_side is not None and refresh is not None else None,
        sideways_before_refresh=first_side < refresh if first_side is not None and refresh is not None else None,
        observation_definition='first measured |y - y_at_actor_appearance| > 0.05 m; not a new controller threshold or causal MPPI attribution')


def summarize_trial(source, group):
    trial = Trial(source, group)
    rows = [r for r in trial.diagnostics if r.get('reference_points', 0) > 0]
    evidence = trial.result.get('native_realtime_evidence', {})
    actor = evidence.get('first_truth_actor_t')
    start = float(actor) if actor is not None else trial.anchor
    result = truth_report(trial)
    monitor_gap = (trial.result.get('native3d_truth_monitor') or {}).get('min_physical_sample_gap')
    result['minimum_recorded_3d_sample_gap_m'] = min(
        [result['native3d_independent_min_physical_sample_gap_m']] +
        ([float(monitor_gap)] if monitor_gap is not None else []))
    result.update(group=group, label=LABELS[group], execution_timing=trial.manifest['execution_timing_pair'],
        phase_timing=timing_by_phase(rows), validated_actions=validated_actions(rows),
        measured_stop=measured_stops(trial.state, start, trial.end),
        maneuver_events=maneuver_events(trial, actor), deadline_zero=deadline_zero_duration(trial),
        deadline_miss_cycles=sum(r.get('reason') == 'deadline_miss' for r in rows),
        reason_counts=dict(Counter(r.get('reason') for r in rows)),
        remani_delivery=delivery_metrics(trial), first_truth_actor_t=actor)
    trial.verify_unchanged()
    result['input_sha256'] = trial.reader.hashes
    return result


def render_pair(trials, out, max_frames=120):
    duration = max(t.end-t.anchor for t in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, int(math.ceil(duration/.3))+1)))
    frame_ms = max(20, 10*round(1000*float(ages[1]-ages[0])/2./10))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 482), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '迎面对照　%.1f s　起步对齐' % age, font=font, fill='#333333')
        draw.text((16, 34), '橙：REMANI参考　蓝：实际机器人投影　灰：真实候选子集　绿：获准展开', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 64), trial.title, font=font, fill='#333333')
            row = draw_panel(image, trial, t, (index*580+16, 98, (index+1)*580-16, 404))
            draw = ImageDraw.Draw(image)
            draw.text((index*580+16, 414), '%s　v=%+.2f m/s' % (trial.status_at(t), row['v']), font=small, fill='#555555')
        draw.text((16, 452), '只改变执行时序整套配置；固定臂差速ROS数值闭环，非Gazebo/实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'comparison.gif', save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(500, frame_ms)], loop=0, disposal=2, optimize=True)
    for trial in trials:
        trial.verify_unchanged()


def draw_chart(rows, out):
    image = Image.new('RGB', (1100, 600), 'white')
    draw = ImageDraw.Draw(image)
    font, small = ImageFont.truetype(FONT, 22), ImageFont.truetype(FONT, 17)
    draw.text((24, 12), '迎面场景：执行时序对照', font=font, fill='#333333')
    measures = [('完整回调 P95 / ms', lambda r: (r['phase_timing']['overall']['callback_ms'] or {}).get('p95')),
                ('通过检查的非零动作下发率 / %', lambda r: 100*r['validated_actions']['validated_nonzero_publication_ratio'] if r['validated_actions']['validated_nonzero_publication_ratio'] is not None else None),
                ('障碍出现后最长实际停车 / s', lambda r: r['measured_stop']['longest_s']),
                ('障碍出现至实际侧移5cm / s', lambda r: r['maneuver_events']['sideways_after_actor_s'])]
    for k, (title, fn) in enumerate(measures):
        x, y = 25+(k % 2)*550, 75+(k // 2)*225
        draw.text((x, y), title, font=small, fill='#333333')
        values = [fn(row) for row in rows]
        maximum = max([v for v in values if v is not None]+[1.])
        for i, (row, value) in enumerate(zip(rows, values)):
            yy = y+46+i*65
            draw.text((x, yy), '当前' if row['group'] == 'current' else '匹配', font=small, fill='#333333')
            width = 330*value/maximum if value is not None else 0.
            if width > 0:
                draw.rectangle((x+55, yy+2, x+55+width, yy+22), fill=('#999999' if i == 0 else '#2874a6'))
            draw.text((x+62+width, yy), show(value), font=small, fill='#333333')
    draw.text((25, 535), '每组一次，不能估计总体成功率。动作下发不等于物理运动，停车另用实测速度计算。', font=small, fill='#555555')
    draw.text((25, 564), '灰：50 / 75 ms　蓝：200 / 250 ms。侧移5cm只用于观察，不是新增控制阈值。', font=small, fill='#555555')
    image.save(out)


def narrative(result):
    text = ['本轮做了什么',
        '在同一个迎面场景重新跑两次在线闭环。当前组用50毫秒截止期限、75毫秒指令有效期；匹配组用200和250毫秒。',
        '保持时间检查、候选验证预留时间等随执行周期一起匹配；不是只放宽一个超时数字。搜索、碰撞球、速度和REMANI不改。',
        '目的是区分：到底是不会避让，还是算出了动作却因迟到没执行。', '', '实际结果']
    for row in result['trials']:
        timing = row['phase_timing']['overall']['callback_ms'] or {}
        actions, events, stopped = row['validated_actions'], row['maneuver_events'], row['measured_stop']
        ratio = actions['validated_nonzero_publication_ratio']
        text += [row['label'],
            '结束原因：%s；三维采样碰撞：%s；记录最小净距：%s米。' %
            (row['termination'], '有' if row['native3d_physical_sample_collision'] else '无', show(row['minimum_recorded_3d_sample_gap_m'], 4)),
            '完整回调P50/P95：%s/%s毫秒；超期%d次。' % (show(timing.get('p50')), show(timing.get('p95')), row['deadline_miss_cycles']),
            '通过检查的非零动作%d次，实际下发%d次，下发比例%s。这里的“下发”不等于小车已经动了。' %
            (actions['validated_nonzero_proposals'], actions['nonzero_proposals_published'], '无记录' if ratio is None else '%.1f%%' % (100*ratio)),
            '障碍出现后，实际累计停车%s秒，最长连续停车%s秒。' % (show(stopped['total_s']), show(stopped['longest_s'])),
            '障碍出现后首次实际侧移超过5厘米：%s秒；第一条新REMANI参考被使用：实验时间%s秒。' %
            (show(events['sideways_after_actor_s']), show(events['first_refresh_consumed_t'])),
            '侧移相对新参考时刻：%s秒（负数表示先侧移；无事件则不作判断）。' % show(events['sideways_minus_refresh_s']), '']
    text += ['怎样看这组结果',
        '动作下发率看的是通过检查后有没有被清零；实际停车看的是里程计速度，不拿零指令时间冒充停车时间。',
        '首次侧移按相对障碍出现时的位置横向变化超过5厘米统计，只是观察口径，不是新加控制阈值，也不能仅凭它断言是哪一层造成的。',
        '本轮每组只跑一次，用来定位执行时序问题，不给总体成功率结论。',
        '这是完整算法节点参与的ROS数值闭环，不是Gazebo或实车。200/250毫秒是本次定位配置，不是最终实车参数。']
    return '\n'.join(text)+'\n'


def report_batch(batch, out, render=True):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('finish both timed trials before reporting')
    plan = json.loads((batch/'plan.json').read_text())
    paths = [batch/('dynamic_head_on_center_%s_seed%d' % (g, plan['seed'])) for g in GROUPS]
    trials = [Trial(path, group) for path, group in zip(paths, GROUPS)]
    if len({t.manifest['scene'] for t in trials}) != 1 or len({t.parameters['seed'] for t in trials}) != 1:
        raise ValueError('paired scene/seed differs')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('body geometry differs')
    out.mkdir(parents=True, exist_ok=False)
    rows = [summarize_trial(path, group) for path, group in zip(paths, GROUPS)]
    result = dict(plan=plan, trials=rows, changed_factor='execution timing configuration as a set, not isolated one-parameter test',
                  scope='new fixed-arm ROS numerical closed loop, not Gazebo/hardware', statistical_success_rate_claim=False)
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验总结.txt').write_text(narrative(result))
    draw_chart(rows, out/'comparison.png')
    if render:
        render_pair(trials, out)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    report_batch(args.batch, args.out)


if __name__ == '__main__':
    main()

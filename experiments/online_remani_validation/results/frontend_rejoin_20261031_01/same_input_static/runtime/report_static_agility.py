#!/usr/bin/env python3
"""Read two completed SIL runs; report measured agility without controller replay."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_long_goal_crossing import panel, summarize as read_trial
from report_sim_crop_bypass import precontact_pairs


def show(value, digits=3):
    return '无记录' if value is None else '%.*f' % (digits, value)


def timing(rows, key):
    values = [float(row[key]) for row in rows if row.get(key) is not None
              and math.isfinite(float(row[key]))]
    return dict(count=len(values), p95_ms=float(np.percentile(values, 95)) if values else None,
                max_ms=max(values) if values else None)


def sustained_turn(times, rates, start, end, duration=.2):
    """First same-sign |w| > .2 held for >= .2 s, using left-sample intervals."""
    beginning, side = None, 0
    for index in range(len(times)-1):
        lo, hi = max(start, float(times[index])), min(end, float(times[index+1]))
        if hi <= lo:
            continue
        sign = int(np.sign(rates[index])) if abs(rates[index]) > .2 else 0
        if sign == 0:
            beginning, side = None, 0
        elif sign != side or beginning is None:
            beginning, side = lo, sign
        if beginning is not None and hi-beginning >= duration:
            return dict(t=beginning, confirmed_at_t=hi, sign=side,
                        direction='左转' if side > 0 else '右转')
    return None


def low_speed(state, start, end, goal):
    intervals, beginning, total = [], None, 0.
    for index in range(len(state)-1):
        row = state[index]
        lo, hi = max(start, float(row['t'])), min(end, float(state[index+1]['t']))
        if hi <= lo:
            continue
        low = (abs(row['v']) < .02 and abs(row['w']) < .05
               and math.hypot(row['x']-goal[0], row['y']-goal[1]) > .1)
        if low:
            if beginning is None:
                beginning = lo
            total += hi-lo
        elif beginning is not None:
            intervals.append(dict(start_t=beginning, end_t=lo, duration_s=lo-beginning))
            beginning = None
    if beginning is not None:
        intervals.append(dict(start_t=beginning, end_t=end, duration_s=end-beginning))
    return dict(total_s=total, longest_s=max([r['duration_s'] for r in intervals]+[0.]),
                intervals=intervals, definition='Measured |v|<.02 m/s, |w|<.05 rad/s, goal distance>.1 m.')


def crossing_y(path, x=3.):
    """Actual sampled/polyline crossings of x=3; no nearest-point extrapolation."""
    path = np.asarray(path, dtype=float)
    values = []
    if path.ndim != 2 or path.shape[1] < 2:
        return values
    for a, b in zip(path, path[1:]):
        if abs(b[0]-a[0]) > 1e-12 and min(a[0], b[0]) <= x <= max(a[0], b[0]):
            values.append(float(a[1]+(x-a[0])/(b[0]-a[0])*(b[1]-a[1])))
    return values


def summarize(source, label):
    original, trial = read_trial(source)
    cycles = trial.reader.jsonl('state_mppi_cycles.jsonl')
    pairs, alignment = precontact_pairs(trial.diagnostics, cycles, trial.end)
    actor = original['first_truth_actor_t']
    start = float(actor) if actor is not None else trial.anchor
    active = [pair for pair in pairs if pair[2] >= start]
    diagnostic = [d for d, _, _ in active if d is not None and d.get('reference_points', 0) > 0]
    state = trial.state
    actual = state[(state['t'] >= trial.anchor) & (state['t'] <= trial.end)]
    # This is a speed-threshold event, not a claim that an obstacle caused braking.
    first_brake, has_fast_sample = None, False
    for row in state:
        if row['t'] > trial.end:
            break
        if row['v'] >= .22:
            has_fast_sample = True
        if row['t'] >= start and has_fast_sample and row['v'] < .22:
            first_brake = float(row['t'])
            break
    turn = sustained_turn(state['t'], state['w'], start, trial.end)
    command_turn = sustained_turn(np.array([p[2] for p in pairs]),
        np.array([p[1]['command_w'] for p in pairs]), start, trial.end)
    prior = [r for r in trial.refs if r['t'] <= start]
    previous_hash = prior[-1].get('sha256') if prior else None
    references = []
    for row in trial.refs:
        if row['t'] < start or row['t'] > trial.end or row.get('sha256') == previous_hash:
            continue
        previous_hash = row.get('sha256')
        references.append(dict(t=float(row['t']), after_actor_s=float(row['t'])-start,
            epoch=row.get('epoch'), revision=row.get('revision'), sha256=previous_hash,
            crossing_y_at_x3_m=crossing_y(row['reference'])))
    near = actual[np.abs(actual['x']-3.) <= .3]
    result = dict(trial=str(trial.source), label=label, scene=trial.manifest['scene'],
        reached_goal=original['reached_goal'], termination=original['termination'],
        interface_audit_valid=original['interface_audit_valid'],
        minimum_physical_sample_gap_m=original['minimum_3d_physical_sample_gap_m'],
        physical_sample_collision=original['physical_sample_contact'],
        actual_path_length_m=float(np.sum(np.hypot(np.diff(actual['x']), np.diff(actual['y'])))),
        motion_duration_s=trial.end-trial.anchor, motion_anchor_t=trial.anchor, end_t=trial.end,
        first_truth_actor_t=actor, metric_window_start_t=start,
        first_measured_speed_below_022_t=first_brake, first_measured_sustained_turn=turn,
        first_published_sustained_turn=command_turn,
        brake_to_measured_turn_s=None if first_brake is None or turn is None else turn['t']-first_brake,
        brake_to_published_turn_s=None if first_brake is None or command_turn is None else command_turn['t']-first_brake,
        non_goal_low_speed=low_speed(state, start, trial.end, original['experiment_settings']['goal']),
        kernel=timing(diagnostic, 'kernel_ms'), callback=timing(diagnostic, 'callback_ms'),
        reason_counts=dict(Counter(c['reason'] for _, c, _ in active)),
        diagnostic_alignment=alignment, reference_adoptions=references,
        actual_crossing_y_at_x3_m=crossing_y(np.column_stack((actual['x'], actual['y']))),
        actual_y_range_near_x3_m=[float(min(near['y'])), float(max(near['y']))] if len(near) else None,
        side_convention='World +x forward; +y left, -y right. x=3 crossings are geometric, not a shared passing-side decision.',
        remani=original['remani'], input_sha256=trial.reader.hashes,
        scope='Fixed-arm differential-drive REMANI+MPPI ROS numerical SIL, not Gazebo/hardware.')
    trial.verify_unchanged()
    return result, trial


def chart(rows, trials, output):
    image = Image.new('RGB', (1120, 710), 'white')
    draw = ImageDraw.Draw(image)
    title, font = [ImageFont.truetype(FONT, size) for size in (21, 15)]
    colors = ['#2874a6', '#d67d23']
    draw.text((20, 10), '实际速度与转向：按障碍出现时刻对齐', font=title, fill='#333333')
    for index, row in enumerate(rows):
        draw.text((25+index*520, 42), row['label'], font=font, fill=colors[index])
    duration = max(trial.end-row['metric_window_start_t'] for row, trial in zip(rows, trials))
    for axis, (field, name, threshold) in enumerate((('v', '实测线速度 / m/s', .22), ('w', '实测角速度 / rad/s', .2))):
        left, top, right, bottom = 82, 110+axis*270, 1085, 302+axis*270
        all_values = [np.abs(trial.state[field][trial.state['t'] >= row['metric_window_start_t']])
                      for row, trial in zip(rows, trials)]
        maximum = max([threshold]+[float(np.max(v)) for v in all_values if len(v)])*1.12
        def xy(t, value):
            return left+t/duration*(right-left), (top+bottom)/2-value/maximum*(bottom-top)/2
        draw.text((20, top-28), name, font=font, fill='#333333')
        for value in (-maximum, 0., maximum):
            yy = xy(0., value)[1]
            draw.line([(left, yy), (right, yy)], fill='#e7e7e7')
            draw.text((22, yy-8), '%.2f' % value, font=font, fill='#777777')
        yy = xy(0., threshold)[1]
        draw.line([(left, yy), (right, yy)], fill='#c5c5c5')
        draw.text((right-130, yy-21), '阈值 %.2f' % threshold, font=font, fill='#777777')
        if axis:
            yy = xy(0., -threshold)[1]
            draw.line([(left, yy), (right, yy)], fill='#c5c5c5')
        for index, (row, trial) in enumerate(zip(rows, trials)):
            start = row['metric_window_start_t']
            selected = trial.state[(trial.state['t'] >= start) & (trial.state['t'] <= trial.end)]
            points = [xy(float(s['t'])-start, float(s[field])) for s in selected]
            if len(points) > 1:
                draw.line(points, fill=colors[index], width=2)
            for event in row['reference_adoptions']:
                xx = xy(event['t']-start, 0.)[0]
                draw.line([(xx, top), (xx, bottom)], fill=colors[index], width=1)
        for tick in range(6):
            value = duration*tick/5
            xx = xy(value, 0.)[0]
            draw.text((xx-16, bottom+8), '%.1f' % value, font=font, fill='#777777')
        draw.rectangle((left, top, right, bottom), outline='#cccccc')
    draw.text((20, 620), '横轴：障碍出现后 / s；细竖线：该组实际采用新 REMANI 参考。曲线均来自实测记录。', font=font, fill='#555555')
    draw.text((20, 650), '明显转向：同方向 |ω|>0.2 rad/s 持续至少0.2秒；绿线展开另见GIF，不是完整搜索过程。', font=font, fill='#555555')
    draw.text((20, 680), '固定机械臂 ROS 数值闭环，非Gazebo/实车；少量重复用于检查现象，不宣称总体成功率。', font=font, fill='#777777')
    image.save(output)


def gif(rows, trials, output, max_frames):
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.4)+1)))
    title, small = [ImageFont.truetype(FONT, size) for size in (19, 14)]
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 510), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 8), '避障对照：起步后 %.1f s' % age, font=title, fill='#333333')
        draw.text((18, 38), '蓝：实际机器人及轨迹　橙：REMANI参考　绿：获准局部展开　紫：CV预测', font=small, fill='#555555')
        for index, (row, trial) in enumerate(zip(rows, trials)):
            left, t = index*580+18, min(trial.anchor+float(age), trial.end)
            draw.text((left, 69), row['label'], font=title, fill='#333333')
            state = panel(image, trial, t, (left, 104, (index+1)*580-18, 410))
            draw = ImageDraw.Draw(image)
            if t >= trial.end-1e-9:
                status = '到达目标' if row['reached_goal'] else '结束：'+row['termination']
            else:
                status = trial.status_at(t)
                status = '控制有效' if status == 'ok' else status
            draw.text((left, 422), '%s　v=%+.2f　ω=%+.2f' % (status, state['v'], state['w']), font=small, fill='#555555')
        draw.text((18, 461), '实录回放、2倍播放。显示机器人碰撞球投影；未记录完整搜索，不生成示意搜索线。', font=small, fill='#777777')
        draw.text((18, 484), '完整REMANI＋底盘MPPI ROS数值闭环，固定机械臂；不是Gazebo/实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    durations = [max(30, 10*round(float(dt)*500./10.)) for dt in np.diff(ages)]+[1000]
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=durations,
                   loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), playback_speedup=2., sha256=sha(output), actual_records_only=True,
                full_search_recorded=False, green_semantics='Execution-authorized local rollout, not full MPPI search.')


def narrative(rows):
    lines = ['静态绕行响应对照', '只读完整在线实录，没有重新运行控制器。固定机械臂ROS数值闭环，不是实车。', '']
    for row in rows:
        turn = row['first_measured_sustained_turn'] or {}
        command_turn = row['first_published_sustained_turn'] or {}
        lines += [row['label']+'：'+('到达目标。' if row['reached_goal'] else '未到达目标：'+row['termination']),
            '最小物理球采样间隙%s米，%s采样碰撞；实走%s米，用时%s秒。' % (
                show(row['minimum_physical_sample_gap_m']), '发生' if row['physical_sample_collision'] else '未发现',
                show(row['actual_path_length_m']), show(row['motion_duration_s'])),
            '实测速度首次降到0.22米/秒以下：%s秒；持续明显转向：%s秒（%s）。' % (
                show(row['first_measured_speed_below_022_t']), show(turn.get('t')), turn.get('direction', '无')),
            '降速到实测持续转向%s秒；到发布持续转向控制%s秒。负值表示先转向、后降速。' % (
                show(row['brake_to_measured_turn_s']), show(row['brake_to_published_turn_s'])),
            '发布持续转向控制时刻：%s秒；不是首次发布MPPI控制，之前仍在持续控制。' % show(command_turn.get('t')),
            '非目标低速累计%s秒，最长%s秒。' % (
                show(row['non_goal_low_speed']['total_s']), show(row['non_goal_low_speed']['longest_s'])),
            'MPPI内核P95/最大：%s/%s毫秒；整回调P95/最大：%s/%s毫秒。' % (
                show(row['kernel']['p95_ms']), show(row['kernel']['max_ms']),
                show(row['callback']['p95_ms']), show(row['callback']['max_ms'])),
            '控制结果计数：'+json.dumps(row['reason_counts'], ensure_ascii=False),
            '实际经过x=3米处的y坐标：'+str(row['actual_crossing_y_at_x3_m'])]
        for ref in row['reference_adoptions']:
            lines.append('新参考采用于%s秒（障碍出现后%s秒），参考在x=3米处的y坐标：%s。' % (
                show(ref['t']), show(ref['after_actor_s']), str(ref['crossing_y_at_x3_m'])))
        lines.append('')
    lines += ['读图说明：朝+x前进时，+y为左、-y为右。所有事件时刻为实验相对时间；GIF各自起步对齐，速度图按障碍出现对齐。',
        '明显转向要求同方向角速度超过0.2弧度/秒并持续至少0.2秒，排除短暂抖动。',
        '降速指标只是记录中速度跨过0.22米/秒，不单凭这个指标认定减速的原因。',
        '低速定义为实测|v|<0.02米/秒、|ω|<0.05弧度/秒，且距目标超过0.1米。',
        '绿色是实际获准的局部轨迹展开，不是全部采样搜索。物理球采样没有接触不等于连续网格安全证明。',
        '单次同种子在线对照不能保证执行时序完全相同；本报告不自动宣布优于V12或实车可用。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--treatment', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-label', default='原启用条件')
    parser.add_argument('--treatment-label', default='提前比较侧绕方案')
    parser.add_argument('--max-frames', type=int, default=110)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    if args.max_frames < 2:
        parser.error('--max-frames must be >=2')
    pairs = [summarize(source, label) for source, label in
             ((args.baseline, args.baseline_label), (args.treatment, args.treatment_label))]
    rows, trials = map(list, zip(*pairs))
    if trials[0].manifest['scenario'] != trials[1].manifest['scenario']:
        raise ValueError('Compare the same recorded obstacle scenario')
    if trials[0].parameters.get('seed') != trials[1].parameters.get('seed'):
        raise ValueError('Compare the same declared seed')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('Physical collision geometry differs between groups')
    output = args.output.resolve()
    if any(output == trial.source or trial.source in output.parents for trial in trials):
        raise ValueError('Write reports outside the recorded trial directories')
    output.mkdir(parents=True, exist_ok=False)
    chart(rows, trials, output/'速度与转向.png')
    render = None if args.no_gif else gif(rows, trials, output/'避障对照.gif', args.max_frames)
    for trial in trials:
        trial.verify_unchanged()
    (output/'summary.json').write_text(json.dumps(dict(trials=rows, rendering=render), ensure_ascii=False, indent=2)+'\n')
    (output/'实验结果.txt').write_text(narrative(rows))
    print(json.dumps(dict(output=str(output), trials=[dict(label=r['label'], goal=r['reached_goal'],
        brake_to_turn_s=r['brake_to_measured_turn_s'], min_gap_m=r['minimum_physical_sample_gap_m']) for r in rows]), ensure_ascii=False))


if __name__ == '__main__':
    main()

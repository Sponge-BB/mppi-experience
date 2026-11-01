#!/usr/bin/env python3
"""Compare completed SIL records with and without the per-cycle stack watchdog.

Read-only with respect to measured trials; no ROS, controller, or replay runs.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from report_long_goal_crossing import panel, summarize
from report_execution_timing import measured_stops
from report_native_dynamic_fast import show
from report_sim_crop_bypass import precontact_pairs, window_metrics


HERE = Path(__file__).resolve().parent
BEFORE = HERE / 'results/long_goal_crossing_20261029_01/mid_cross_seed11'
AFTER = HERE / 'results/watchdog_off_20261029_01/mid_cross_seed11'
OUTPUT = HERE / 'results/watchdog_off_report_20261029_01'
LABELS = ('原每轮抓栈开启', '关闭每轮抓栈')
COLORS = ('#a87953', '#2874a6')


def span_stats(records, key):
    values = [float(r[key]) for r in records]
    return dict(count=len(values), maximum_s=max(values) if values else None,
        p50_s=float(np.percentile(values, 50)) if values else None,
        p95_s=float(np.percentile(values, 95)) if values else None,
        over_250ms_count=sum(v > .25 for v in values),
        largest_examples=sorted(records, key=lambda r: r[key], reverse=True)[:8])


def cycle_spans(cycles, start, end):
    """Clip by observed publication; never append an unobserved tail gap.

    cycle_start occurs after watchdog setup, so postpublication-to-next-start
    also covers setup before the next cycle's accounting starts. We measure
    ordinary Twist publication, not the later local-execution envelope publish.
    """
    rows = [r for r in cycles if start <= float(r['command_publish_after_ros_time'])-1000. < end]
    active, gaps = [], []
    for row in rows:
        active.append(dict(cycle=row['cycle_id'],
            publish_t=float(row['command_publish_after_ros_time'])-1000.,
            duration_s=float(row['command_publish_after_monotonic'])-float(row['cycle_start_monotonic']),
            reason=row['reason']))
    for previous, current in zip(rows, rows[1:]):
        gaps.append(dict(cycle=previous['cycle_id'], next_cycle=current['cycle_id'],
            previous_publish_t=float(previous['command_publish_after_ros_time'])-1000.,
            next_publish_t=float(current['command_publish_after_ros_time'])-1000.,
            duration_s=float(current['cycle_start_monotonic'])-float(previous['command_publish_after_monotonic']),
            next_reason=current['reason']))
    return dict(window_start_t=start, window_end_t=end,
        start_to_publish=span_stats(active, 'duration_s'),
        previous_publish_to_next_start=span_stats(gaps, 'duration_s'),
        semantics='wall monotonic spans, publications inside [start,end); gaps include pre-timing watchdog setup; no unobserved final tail; ordinary Twist publication is not full actuator-envelope publication')


def additional_metrics(result, trial, label):
    cycles = trial.reader.jsonl('state_mppi_cycles.jsonl')
    pairs, _ = precontact_pairs(trial.diagnostics, cycles, trial.end)
    navigation_end = result['measured_stop_navigation']['window_end_s']
    valid = trial.audit.get('valid_experiment') is True and not trial.status.get('required_process_failed')
    native = trial.reader.jsonl('dual_timeline.jsonl')
    last_native = max((float(r['sim'])-1000. for r in native if r.get('sim') is not None), default=trial.end)
    # Both runs cover 44 s after actual motion starts. Neither has arrived or
    # crashed in that common window. Keep full-run values separately above.
    end = min(float(trial.anchor)+44., navigation_end, last_native if not valid else trial.end)
    actor = result['measured_stop_navigation']['window_start_s']
    active = [p for p in pairs if actor <= p[2] < end]
    diagnostic = [d for d, _, _ in active if d is not None and d.get('reference_points', 0) > 0]
    result.update(label=label,
        diagnostic_observation_window=dict(start_t=trial.anchor, end_t=end,
            requested_motion_window_s=44., duration_s=end-trial.anchor,
            last_native_event_t=last_native,
            interpretation='common first 44 s after actual motion; excludes shutdown tail and goal settling; full-run data remain separately preserved'),
        experiment_valid=valid, required_process_failed=trial.status.get('required_process_failed', False),
        measured_stop_diagnostic=measured_stops(trial.state, actor, end),
        navigation_cycles=window_metrics(active),
        navigation_cycle_spans=cycle_spans(cycles, float(trial.anchor), end),
        kernel_over_200ms_cycles=sum(d.get('kernel_ms', 0.) > 200. for d in diagnostic),
        callback_over_200ms_cycles=sum(d.get('callback_ms', 0.) > 200. for d in diagnostic),
        timing_count_scope='obstacle appearance until common 44 s motion cutoff; diagnostic with reference; computation counts are distinct from safe-action deadline clearing')
    trial.verify_unchanged()
    result['input_sha256'] = dict(trial.reader.hashes)
    return result


def check_conditions(trials):
    a, b = trials
    for key in ('scenario', 'long_goal_crossing_experiment', 'execution_timing_pair'):
        if a.manifest.get(key) != b.manifest.get(key):
            raise ValueError('Recorded conditions differ: '+key)
    if a.parameters.get('seed') != b.parameters.get('seed'):
        raise ValueError('Recorded seeds differ')
    if not np.array_equal(a.body3d, b.body3d):
        raise ValueError('Recorded robot spheres differ')
    if a.reader.json('limits.json', {}, required=True) != b.reader.json('limits.json', {}, required=True):
        raise ValueError('Recorded controller limits differ')
    return dict(same_scene=True, same_seed=True, same_collision_spheres=True,
        same_recorded_limits=True, same_long_goal_settings=True,
        scope='recorded condition equality, not identical evolving feedback inputs')


def chart(rows, out):
    """Compact values plus bars; no shared scale between unlike quantities."""
    image = Image.new('RGB', (1160, 1030), 'white')
    draw = ImageDraw.Draw(image)
    font, small, title = (ImageFont.truetype(FONT, size) for size in (18, 14, 23))
    draw.text((22, 12), '关闭每轮抓栈前后：起步后0～44秒诊断记录（新组最终异常中断）', font=title, fill='#333333')
    for i in range(2):
        draw.rectangle((25+i*320, 55, 42+i*320, 71), fill=COLORS[i])
        draw.text((50+i*320, 52), LABELS[i], font=font, fill='#444444')

    def phase(row, metric, pct):
        return ((row['navigation_cycles']['phase_timing']['intervention'].get(metric) or {}).get(pct))

    specs = [
        ('累计实际停车 / s', lambda r: [r['measured_stop_diagnostic']['total_s']], ('',)),
        ('最长连续实际停车 / s', lambda r: [r['measured_stop_diagnostic']['longest_s']], ('',)),
        ('指令发布后→下一轮起点：最大空档 / s', lambda r: [r['navigation_cycle_spans']['previous_publish_to_next_start']['maximum_s']], ('',)),
        ('本轮起点→指令发布：最大耗时 / ms', lambda r: [1000*r['navigation_cycle_spans']['start_to_publish']['maximum_s']], ('',)),
        ('避障介入：MPPI内核 / ms', lambda r: [phase(r, 'kernel_ms', p) for p in ('p50', 'p95')], ('P50', 'P95')),
        ('避障介入：求解＋验收 / ms', lambda r: [phase(r, 'callback_ms', p) for p in ('p50', 'p95')], ('P50', 'P95')),
        ('安全非零动作仅因超期被清零 / 次', lambda r: [r['navigation_cycles']['deadline']['validated_nonzero_proposals_cleared']], ('',)),
        ('发布后空档超过250 ms / 次', lambda r: [r['navigation_cycle_spans']['previous_publish_to_next_start']['over_250ms_count']], ('',)),
    ]
    for index, (heading, getter, names) in enumerate(specs):
        left, top = 22+(index % 2)*574, 102+(index // 2)*211
        draw.text((left, top), heading, font=font, fill='#333333')
        values = [getter(row) for row in rows]
        maximum = max([float(v) for group in values for v in group if v is not None]+[1e-6])
        for group, group_values in enumerate(values):
            for j, value in enumerate(group_values):
                y = top+37+group*65+j*26
                draw.text((left, y-1), names[j], font=small, fill='#666666')
                if value is not None:
                    bar = 370*float(value)/maximum
                    if bar > 0:
                        draw.rectangle((left+42, y+1, left+42+bar, y+13), fill=COLORS[group])
                draw.text((left+427, y-3), show(value, 2), font=small, fill=COLORS[group])
        draw.line((left, top+192, left+545, top+192), fill='#eeeeee')
    draw.text((22, 965), '主要指标均截止各自起步后44秒；停车/超期从障碍出现开始，循环空档从起步开始。全程原始数据另保留。', font=small, fill='#666666')
    draw.text((22, 992), '新组随后REMANI崩溃，不是有效的完整导航对照；这里只展示崩溃前诊断记录，不宣称成功率或唯一因果。', font=small, fill='#666666')
    image.save(out/'comparison.png')


def render(trials, rows, out, max_frames=120):
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.3)+1)))
    frame_ms = max(20, 10*round(1000*float(ages[1]-ages[0])/2./10))
    font, small = ImageFont.truetype(FONT, 19), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1220, 565), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 9), '远目标·中段横穿　%.1f s（各自起步对齐）' % age, font=font, fill='#333333')
        draw.text((18, 37), '蓝：整机投影及实走轨迹　橙：REMANI参考　灰：实录搜索子集　绿：获准展开　紫：CV预测', font=small, fill='#555555')
        for i, trial in enumerate(trials):
            t = min(float(trial.anchor)+float(age), trial.end)
            draw.text((i*610+18, 72), LABELS[i], font=font, fill=COLORS[i])
            row = panel(image, trial, t, (i*610+16, 105, (i+1)*610-16, 439))
            draw = ImageDraw.Draw(image)
            ended = age >= trial.end-trial.anchor
            status = (('异常结束，画面冻结' if not rows[i]['experiment_valid'] else '已结束，画面冻结')
                      if ended else trial.status_at(t))
            draw.text((i*610+18, 454), '%s　v=%+.2f m/s' % (status, row['v']), font=small, fill='#444444')
        draw.text((18, 510), '固定臂差速ROS数值闭环，非Gazebo/实车；只回放实际记录，不生成未记录的候选。', font=small, fill='#777777')
        draw.text((18, 538), '新组最终REMANI异常退出：不是完整有效对照。二维完整球模型投影不代替三维接触判据。', font=small, fill='#a34d41')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'comparison.gif', save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(800, frame_ms)], loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), playback_speedup=2.,
        alignment='each actual motion anchor; freeze at recorded termination',
        actual_robot_projection=True, actual_remani_reference=True,
        actual_recorded_mppi_search_subset=True, no_controller_reexecution=True)


def narrative(result):
    lines = ['先说本次状态',
        '新组REMANI进程以exit -11异常退出，ROS随之关闭；没有完成有效的完整导航实验。',
        '下面只对比双方起步后0～44秒的实录诊断窗口，均在崩溃和目标驻停之前；不能把本轮当作完整性能/成功率结果。',
        '关闭抓栈后仍出现秒级控制空档；轻量阶段记录主要把长等待落在状态记录区，不能归为MPPI一直在求解。',
        '这是阶段范围定位，还没有系统调用级证据证明状态写入是唯一原因。', '', '本轮做了什么',
        '只关闭每轮重新启动的超时抓栈监测，保留轻量阶段时间记录。',
        '目标6米、障碍在路径中段横穿、轮速上限、3秒预测上限、200/250毫秒设置不变。',
        '完整REMANI＋MPPI、固定机械臂、差速底盘ROS数值闭环；不是Gazebo或实车。', '']
    for row in result['trials']:
        stop = row['measured_stop_diagnostic']
        spans = row['navigation_cycle_spans']
        timing = row['navigation_cycles']['phase_timing']['intervention']
        kernel, callback = timing['kernel_ms'] or {}, timing['callback_ms'] or {}
        deadline = row['navigation_cycles']['deadline']
        lines.extend([row['label'],
            '原始全程结束：%s；到达目标：%s；三维采样接触：%s；完整实验有效：%s。' %
                (row['termination'], '是' if row['reached_goal'] else '否', '有' if row['physical_sample_contact'] else '无', '是' if row['experiment_valid'] else '否'),
            '最小三维物理采样间隙%s米。' % show(row['minimum_3d_physical_sample_gap_m'], 4),
            '共同诊断窗口内、障碍出现后：累计停车%s秒，最长连续停车%s秒。' % (show(stop['total_s']), show(stop['longest_s'])),
            '发布后到下一轮开始的最大空档%s毫秒；超过250毫秒%d次。这个间隙包含下一轮计时开始前的等待。' %
                (show(1000*spans['previous_publish_to_next_start']['maximum_s']), spans['previous_publish_to_next_start']['over_250ms_count']),
            '本轮计时开始到发布指令：最大%s毫秒；超过250毫秒%d次。' %
                (show(1000*spans['start_to_publish']['maximum_s']), spans['start_to_publish']['over_250ms_count']),
            '避障介入时，MPPI内核P50/P95：%s/%s毫秒；求解＋验收P50/P95：%s/%s毫秒。' %
                (show(kernel.get('p50')), show(kernel.get('p95')), show(callback.get('p50')), show(callback.get('p95'))),
            '已有参考的记录中，内核超过200毫秒%d轮，求解＋验收超过200毫秒%d轮。' %
                (row['kernel_over_200ms_cycles'], row['callback_over_200ms_cycles']),
            '超期标签%d次；其中通过安全检查的非零动作仅因超期被清零%d次。不能把这两个数混为一谈。' %
                (deadline['label_count'], deadline['validated_nonzero_proposals_cleared']),
            '全程原始记录中，障碍出现后使用新REMANI参考%d条；该项不是44秒窗口统计。' % row['remani']['new_references_after_actor'], ''])
    lines += ['怎么看结果',
        '本轮先看关闭抓栈后，发布后空档与空旷停车有没有减少；再单独看MPPI计算超过200毫秒的情况。',
        '循环起点到发布用墙钟单调时间；MPPI求解＋验收耗时不包含所有两轮之间的操作。',
        '实际停车按实测|v|<0.01 m/s且|ω|<0.03 rad/s统计，不把零指令等同于机器人已停。',
        '主要对比指标只统计共同44秒窗口；JSON同时保留原始全程摘要，包括新组异常退出。GIF回放完整实录，结束即冻结，不补画之后的运动。',
        '旧组是之前的单次记录，新组是本次同设置重新闭环且最终崩溃，两次输入会随运动分叉。不能由这两次给出总体成功率，也不能证明所有卡顿只有一个原因。',
        'comparison.gif为起步对齐的实际记录，comparison.png为关键指标；report.json保留详细数字、时间窗口与输入校验。', '']
    return '\n'.join(lines)


def report(before=BEFORE, after=AFTER, out=OUTPUT, make_gif=True):
    rows, trials = [], []
    for path, label in zip((before, after), LABELS):
        row, trial = summarize(path)
        rows.append(additional_metrics(row, trial, label))
        trials.append(trial)
    conditions = check_conditions(trials)
    out = Path(out).resolve()
    if any(out == t.source or t.source in out.parents for t in trials):
        raise ValueError('Report must remain outside measured trial directories')
    out.mkdir(parents=True, exist_ok=True)
    result = dict(trials=rows, recorded_condition_checks=conditions,
        changed_factor='per-cycle stack-dump watchdog disabled; lightweight phase timestamps retained',
        scope='complete REMANI+MPPI fixed-arm differential-drive ROS numerical SIL; not Gazebo/hardware',
        historical_single_trial_comparison=True, statistical_success_rate_claim=False)
    chart(rows, out)
    if make_gif:
        result['visualization'] = render(trials, rows, out)
    for row, trial in zip(rows, trials):
        trial.verify_unchanged()
        row['input_sha256'] = dict(trial.reader.hashes)
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验对比.txt').write_text(narrative(result))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', type=Path, default=BEFORE)
    parser.add_argument('--after', type=Path, default=AFTER)
    parser.add_argument('--out', type=Path, default=OUTPUT)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    result = report(args.before, args.after, args.out, not args.no_gif)
    print(json.dumps(dict(output=str(args.out.resolve()),
        termination=[r['termination'] for r in result['trials']]), ensure_ascii=False))


if __name__ == '__main__':
    main()

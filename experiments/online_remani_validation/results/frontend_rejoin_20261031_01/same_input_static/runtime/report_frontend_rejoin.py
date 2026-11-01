#!/usr/bin/env python3
"""Report completed front-end rejoin experiments without replaying controllers.

The same-input search study and online closed loops stay separate evidence.
Static adopted trajectories use the existing publication/polynomial audit.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_long_goal_crossing import panel
from report_static_agility import chart, gif, narrative as detail_text, show, summarize
from report_static_width_diagnosis import STAGES, draw_geometry, inspect_trial, metrics


def read_replay(path):
    if path is None:
        return None
    if path.suffix == '.jsonl':
        records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        captures = [r for r in records if r.get('kind') == 'capture']
        complete = [r for r in records if r.get('kind') == 'complete']
        if len(captures) != 1 or len(complete) != 1:
            raise ValueError('same-input capture/completion missing or ambiguous')
        variants = []
        for row in records:
            if row.get('kind') != 'variant':
                continue
            points = np.asarray([p[:2] for part in row['output']['parts'] for p in part['points']], dtype=float)
            variants.append(dict(variant=row['variant'], status=row['status'],
                steer_penalty=row['steer_penalty'], steer_change_penalty=row['steer_change_penalty'],
                yaw_resolution=row['yaw_resolution'], frozen_stable=row['frozen_stable'],
                wrapper_wall_ms=row['wrapper_wall_ms'], expanded_nodes=row['expanded_nodes'],
                geometry=metrics(points, 3.) if len(points) else None))
        data = dict(capture=captures[0], completion=complete[0]['result'], variants=variants,
            raw_records=records,
            limitation='ROS simulation time was paused; ROS-time native search budget does not advance. Geometry diagnostic only, not proof of online search timing.')
    else:
        data = json.loads(path.read_text())
    return dict(path=str(path.resolve()), sha256=sha(path), data=data)


def first_adopted(row):
    return next((a for a in row.get('width_diagnosis', {}).get('attempts', [])
                 if a['actually_consumed'] and a['consumed_t'] > row['metric_window_start_t']), None)


def delivery_metrics(row):
    """Keep request wall time distinct from end-to-end reference availability."""
    requests = row['remani']['requests']
    adopted = first_adopted(row)
    request = next((r for r in requests if adopted and r['request_id'] == adopted['request_id']), None)
    refresh = [r for r in requests if r.get('request_id', 0) > 1
               and r.get('start_sim_s', -1) >= row['metric_window_start_t']]
    result = dict(first_adopted_request_id=adopted['request_id'] if adopted else None,
                  first_adopted_request_wall_ms=request.get('request_wall_ms') if request else None,
                  request_start_to_consumed_reference_ms=None,
                  first_refresh_start_to_consumed_reference_ms=None)
    if request and adopted:
        result['request_start_to_consumed_reference_ms'] = 1000.*(adopted['consumed_t']-request['start_sim_s'])
        if refresh:
            result['first_refresh_start_to_consumed_reference_ms'] = 1000.*(
                adopted['consumed_t']-min(r['start_sim_s'] for r in refresh))
    return result


def single_gif(row, trial, output, max_frames):
    duration = trial.end-trial.anchor
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.4)+1)))
    title, small = [ImageFont.truetype(FONT, size) for size in (20, 15)]
    frames = []
    for age in ages:
        image = Image.new('RGB', (1050, 610), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 9), '迎面障碍回归：起步后 %.1f s' % age, font=title, fill='#333333')
        draw.text((18, 42), '蓝：机器人/实走轨迹　橙：REMANI参考　绿：获准局部展开　紫：CV预测', font=small, fill='#555555')
        t = min(trial.anchor+float(age), trial.end)
        state = panel(image, trial, t, (18, 85, 1032, 505))
        draw = ImageDraw.Draw(image)
        status = ('到达目标' if row['reached_goal'] else row['termination']) if t >= trial.end-1e-9 else trial.status_at(t)
        status = '控制有效' if status == 'ok' else status
        draw.text((18, 520), '%s　v=%+.2f m/s　ω=%+.2f rad/s' % (status, state['v'], state['w']), font=small, fill='#555555')
        draw.text((18, 555), '实录回放，2倍播放；机器人为完整固定臂碰撞球投影；绿色不是全部搜索采样。', font=small, fill='#777777')
        draw.text((18, 582), '完整REMANI＋底盘MPPI，ROS数值闭环，非Gazebo/实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(output, save_all=True, append_images=frames[1:],
        duration=[max(30, 10*round(float(dt)*500./10.)) for dt in np.diff(ages)]+[1000],
        loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), sha256=sha(output), actual_records_only=True,
                full_search_recorded=False, playback_speedup=2.)


def narrative(rows, replay, replay_notes, decision_notes):
    lines = ['前端绕行回归：同输入排查与完整在线验证', '',
             '一、这轮要解决什么',
             '保持提前避障能力，避免REMANI的初始路线沿侧向走得太远、绕过障碍后迟迟不回归。',
             '同输入排查用于定位前端选择；完整在线闭环用于判断实际控制有没有改善，两者不混为同一种证据。', '',
             '二、同输入排查']
    if replay:
        lines += ['原始数据：'+replay['path']]
        data = replay['data']
        for variant in data.get('variants', []):
            geometry = variant.get('geometry')
            lines.append('%s：转向惩罚%s、转向变化惩罚%s、朝向分辨率%s弧度；输出状态%s，路线最宽%s米、全段长%s米。' % (
                variant['variant'], show(variant['steer_penalty']), show(variant['steer_change_penalty']),
                show(variant['yaw_resolution']), variant['status'],
                show(geometry['max_abs_y_m'] if geometry else None),
                show(geometry['path_length_m'] if geometry else None)))
        if data.get('completion'):
            lines += ['输入冻结检查：'+json.dumps(data['completion'], ensure_ascii=False)]
        if data.get('limitation'):
            lines += ['这组排查暂停了仿真时钟，所以原搜索按ROS时间计算的预算没有推进。只能用它比较路线选择，不能把它当作在线耗时合格的证明；随后闭环恢复正常时钟和原预算。']
    lines += replay_notes or ['本报告没有附加同输入结论；请以原始排查记录为准。']
    lines += ['', '三、完整在线结果']
    for row in rows:
        lines += ['', row['label']+'：'+('到达目标。' if row['reached_goal'] else '未到达目标：'+row['termination']),
            '实走%s米，用时%s秒；最小物理球采样间隙%s米，%s采样碰撞。' % (
                show(row['actual_path_length_m']), show(row['motion_duration_s']),
                show(row['minimum_physical_sample_gap_m']), '发生' if row['physical_sample_collision'] else '未发现'),
            '降速到实际持续转向%s秒；非目标停顿累计%s秒。' % (
                show(row['brake_to_measured_turn_s']), show(row['non_goal_low_speed']['total_s'])),
            'MPPI内核P95/最大%s/%s毫秒；整回调P95/最大%s/%s毫秒。' % (
                show(row['kernel']['p95_ms']), show(row['kernel']['max_ms']),
                show(row['callback']['p95_ms']), show(row['callback']['max_ms']))]
        attempt = first_adopted(row)
        if attempt:
            values = [attempt['stages'][stage]['max_abs_y_m'] if attempt['stages'][stage] else None for stage in STAGES]
            lines += ['首次采用的新轨迹（请求%d）：搜索路点最宽%s米 → 优化前%s米 → 交付后%s米。' % (
                attempt['request_id'], *(show(v) for v in values))]
            delivery = row['delivery']
            lines += ['该请求本身计算%s毫秒；从请求开始到MPPI采用%s毫秒；从本轮第一次重规划开始到采用%s毫秒。' % (
                show(delivery['first_adopted_request_wall_ms']),
                show(delivery['request_start_to_consumed_reference_ms']),
                show(delivery['first_refresh_start_to_consumed_reference_ms']))]
        lines += ['重规划结果分布：'+json.dumps(row['remani']['refresh_outcomes'], ensure_ascii=False),
                  '控制结果计数：'+json.dumps(row['reason_counts'], ensure_ascii=False)]
        for attempt in row.get('width_diagnosis', {}).get('attempts', []):
            if not attempt['actually_consumed']:
                lines.append('未交付候选：请求%d，优化%s，拒收记录%s。' % (
                    attempt['request_id'], '成功' if attempt['optimizer_success'] else '失败', attempt['rejection_reasons']))
    lines += ['', '四、结论与是否保留']
    lines += decision_notes or ['尚未填写方案取舍；以上为实际测量结果，不自动判定改进成功。']
    lines += ['', '五、看图和读数说明',
        '两组静态都保留提前绕行与局部范围修正，只对比本轮前端调整。',
        '最宽是相对原直线y=0的最大横向距离。只把实际发布并被MPPI采用的候选计入交付主图；失败尝试保留记录。',
        'GIF蓝色是机器人和实走轨迹，橙色是实际采用的REMANI参考，绿色是获准的局部展开；机器人显示碰撞球投影，不是点机器人。',
        '持续转向要求同方向角速度超过0.2弧度/秒至少0.2秒；降速指线速度低于0.22米/秒。负延迟表示先转向后降速。',
        '停顿定义为|线速度|<0.02米/秒且|角速度|<0.05弧度/秒、距离目标>0.1米。',
        '请求本身计算耗时是墙钟时间；请求开始到采用按实验时钟记录，包含下游发布/采用等待。二者不是轨迹运动时长。',
        '本轮为固定机械臂、差速底盘、完整REMANI+MPPI的ROS数值闭环，不是Gazebo/实车；没有修改实车部署文件。',
        '有限次数用于检验这次问题和回归，不能据此给出总体成功率；未发现采样碰撞不等于连续时间安全证明。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--treatment', type=Path, required=True)
    parser.add_argument('--headon', type=Path)
    parser.add_argument('--headon-baseline', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--baseline-label', default='提前绕行：原前端')
    parser.add_argument('--treatment-label', default='提前绕行：调整前端')
    parser.add_argument('--replay-summary', type=Path)
    parser.add_argument('--replay-note', action='append', default=[])
    parser.add_argument('--decision-note', action='append', default=[])
    parser.add_argument('--max-frames', type=int, default=100)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    values = [inspect_trial(args.baseline, args.baseline_label), inspect_trial(args.treatment, args.treatment_label)]
    rows, trials, pictures = map(list, zip(*values))
    if trials[0].manifest['scenario'] != trials[1].manifest['scenario']:
        raise ValueError('static scenarios differ')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('static collision geometry differs')
    extra = []
    if args.headon_baseline:
        extra.append(summarize(args.headon_baseline, '迎面：原前端'))
    if args.headon:
        extra.append(summarize(args.headon, '迎面：调整前端'))
    for row, trial in extra:
        rows.append(row)
        trials.append(trial)
    for row in rows:
        row['delivery'] = delivery_metrics(row)
    output = args.output.resolve()
    if any(output == t.source or t.source in output.parents for t in trials):
        raise ValueError('report must be outside source trials')
    output.mkdir(parents=True, exist_ok=True)
    geometry_available = all(pictures)
    if geometry_available:
        draw_geometry(rows[:2], trials[:2], pictures, output/'前端调整三阶段对照.png')
    chart(rows[:2], trials[:2], output/'静态速度与转向.png')
    renderings = {}
    if not args.no_gif:
        renderings['static'] = gif(rows[:2], trials[:2], output/'静态前端调整对照.gif', args.max_frames)
        if len(extra) == 2:
            if extra[0][1].manifest['scenario'] != extra[1][1].manifest['scenario']:
                raise ValueError('headon scenarios differ')
            renderings['headon'] = gif(rows[2:], trials[2:], output/'迎面回归对照.gif', args.max_frames)
        elif len(extra) == 1:
            renderings['headon'] = single_gif(rows[-1], trials[-1], output/'迎面回归.gif', args.max_frames)
    for trial in trials:
        trial.verify_unchanged()
    replay = read_replay(args.replay_summary)
    summary = dict(trials=rows, renderings=renderings, same_input_study=replay,
        adopted_geometry_comparison_available=geometry_available,
        replay_notes=args.replay_note, decision_notes=args.decision_note,
        scope='Completed numerical ROS SIL runs; same-input front-end replay is separate evidence.')
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    (output/'实验结果.txt').write_text(narrative(rows, replay, args.replay_note, args.decision_note))
    (output/'详细数据.txt').write_text(detail_text(rows))
    print(json.dumps(dict(output=str(output), rows=[dict(label=r['label'], reached_goal=r['reached_goal'],
        actual_path_length_m=r['actual_path_length_m'], delivery=r['delivery']) for r in rows]), ensure_ascii=False))


if __name__ == '__main__':
    main()

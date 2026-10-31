#!/usr/bin/env python3
"""Read one finished long-goal crossing SIL run; never rerun a controller.

The map, goal and actor motion come from the recorded manifest, not the older
3 m / x=4.5 m renderer defaults. All plotted trajectories are recorded data.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT, latest
from render_native_collision_3d import Trial
from report_execution_timing import measured_stops
from report_native_dynamic_fast import show
from report_sim_crop_bypass import precontact_pairs, window_metrics


HERE = Path(__file__).resolve().parent
SOURCE = HERE / 'results/long_goal_crossing_20261029_01/mid_cross_seed11'
OUTPUT = HERE / 'results/long_goal_crossing_report_20261029_01'


def settings(manifest):
    value = manifest['long_goal_crossing_experiment']
    goal = np.asarray(value['goal'], dtype=float)
    bounds = np.asarray(value['bounds'], dtype=float)
    if (goal.shape != (2,) or bounds.shape != (4,) or
            not np.isfinite(np.r_[goal, bounds]).all() or
            not bounds[0] < goal[0] < bounds[1] or
            not bounds[2] < goal[1] < bounds[3]):
        raise ValueError('invalid recorded goal/world bounds')
    return value, goal, bounds


def scene_actors(manifest, result, t):
    """Recorded prescribed scene truth, used only for drawing/evaluation."""
    scene = manifest['scenario']
    anchor = result.get('motion_anchor') if manifest.get('scenario_start') == 'motion' else result.get('goal_sent')
    if anchor is None:
        return []
    age = float(t)-float(anchor)-float(scene['appearance_s'])
    if age < 0:
        return []
    return [(float(a['x'])+age*float(a['vx']),
             float(a['y'])+age*float(a['vy']), float(a['r']), a['id'],
             float(a['vx']), float(a['vy'])) for a in scene['actors']]


def sampled_gap(pose, body, resolution, actors, bounds):
    """Same finite walls/cylinders and physical sphere semantics as SIL truth.

    The world extents are explicitly supplied; no old x=4.5 m wall is retained.
    This is a discrete sampling audit, not full-mesh/continuous collision proof.
    """
    xmin, xmax, ymin, ymax = bounds
    c, s = np.cos(pose[2]), np.sin(pose[2])
    centers = body[:, :3].copy()
    centers[:, :2] = centers[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]
    radii = body[:, 3]-(body[:, 6] == 0)*resolution
    x, y, z = centers.T
    dz = np.maximum(np.maximum(.15-z, z-1.41), 0.)
    dx = np.maximum(np.maximum(xmin-x, x-xmax), 0.)
    dy = np.maximum(np.maximum(ymin-y, y-ymax), 0.)
    gaps = [np.sqrt((y-side)**2+dx**2+dz**2)-radii for side in (ymin, ymax)]
    gaps += [np.sqrt((x-side)**2+dy**2+dz**2)-radii for side in (xmin, xmax)]
    for ox, oy, radius, *_ in actors:
        radial = np.hypot(x-ox, y-oy)-radius
        vertical = np.abs(z-(.15+1.4)/2)-(1.4-.15)/2
        distance = (np.hypot(np.maximum(radial, 0.), np.maximum(vertical, 0.))+
                    np.minimum(np.maximum(radial, vertical), 0.))
        gaps.append(distance-radii)
    return float(np.min(gaps))


def measured_rows(trial, start=None):
    start = trial.anchor if start is None else start
    return trial.state[(trial.state['t'] >= start) & (trial.state['t'] <= trial.end)]


def summarize(source):
    trial = Trial(source, 'native3d')
    config, goal, bounds = settings(trial.manifest)
    if not trial.status.get('result_written'):
        raise ValueError('finish the timed trial before reporting')
    cycles = trial.reader.jsonl('state_mppi_cycles.jsonl')
    pairs, alignment = precontact_pairs(trial.diagnostics, cycles, trial.end)
    sent = float(trial.result.get('goal_sent', trial.anchor))
    after_goal = [p for p in pairs if p[2] >= sent]
    actor_t = trial.result.get('native_realtime_evidence', {}).get('first_truth_actor_t')
    actor_start = float(actor_t) if actor_t is not None else trial.anchor
    actor_pairs = [p for p in pairs if p[2] >= actor_start]
    physical = [sampled_gap(np.array([r['x'], r['y'], r['yaw']]), trial.body3d,
                           trial.resolution, scene_actors(trial.manifest, trial.result, float(r['t'])), bounds)
                for r in measured_rows(trial, float(trial.state['t'][0]))]
    monitor = trial.result.get('native3d_truth_monitor', {})
    minimum = min(physical+[float(monitor['min_physical_sample_gap'])]
                  if monitor.get('min_physical_sample_gap') is not None else physical)
    analysis = trial.reader.json('analysis/summary.json', {})
    requests = [r for r in analysis.get('requests', []) if r.get('start_sim_s', math.inf) < trial.end]
    refresh = [r for r in requests if r.get('request_id', 0) > 1]
    # A completed request is counted only if its completion was observed before
    # termination. A started-but-unfinished request remains listed separately.
    completed = [r for r in requests if r.get('end_sim_s') is not None and r['end_sim_s'] <= trial.end]
    accepted = [r for r in completed if r.get('success') is True]
    refs = [r for r in trial.refs if r['t'] < trial.end]
    prior = [r for r in refs if r['t'] <= actor_start]
    initial = prior[-1] if prior else None
    new_refs = [r for r in refs if r['t'] > actor_start and
                (initial is None or r.get('sha256') != initial.get('sha256'))]
    gaps = [float(b[1]['cycle_start_monotonic'])-float(a[1]['command_publish_after_monotonic'])
            for a, b in zip(after_goal, after_goal[1:])]
    limits = trial.reader.json('limits.json', {}, required=True)
    result = dict(trial=str(trial.source), scene=trial.manifest['scene'],
        seed=trial.parameters.get('seed'), scope='fixed-arm differential-drive REMANI+MPPI ROS numerical SIL; not Gazebo/hardware',
        experiment_settings=config, scene_definition=trial.manifest['scenario'], recorded_limits=limits,
        termination=trial.result['reason'], reached_goal=trial.result['reason'] == 'goal',
        physical_sample_contact=bool(minimum < 0 or monitor.get('sample_collision')),
        minimum_3d_physical_sample_gap_m=minimum,
        independent_state_sample_minimum_gap_m=min(physical), online_substep_truth_monitor=monitor,
        truth_scope='native physical sphere samples versus recorded finite walls/cylinders; no 2D projection contact assertion',
        start_t=trial.anchor, end_t=trial.end, observed_duration_s=trial.end-trial.anchor,
        goal_sent_t=sent, first_truth_actor_t=actor_t,
        final_goal_distance_m=float(np.linalg.norm(np.array([trial.state[-1]['x'], trial.state[-1]['y']])-goal)),
        after_goal=window_metrics(after_goal), actor_window=window_metrics(actor_pairs),
        measured_stop=measured_stops(trial.state, actor_start, trial.end),
        largest_postpublish_to_next_start_gap_s=max(gaps) if gaps else None,
        diagnostic_alignment=alignment,
        remani=dict(total_requests_started=len(requests), refresh_requests_started=len(refresh),
            completed_requests=len(completed), accepted_completed_requests=len(accepted),
            refresh_outcomes=dict(Counter(r.get('outcome_class', 'unknown') for r in refresh)),
            optimizer_calls=sum(len(r.get('optimizer_calls', [])) for r in requests),
            consumed_references=len(refs), new_references_after_actor=len(new_refs),
            first_new_reference_after_actor_t=min((r['t'] for r in new_refs), default=None),
            requests=requests), interface_audit_valid=trial.audit.get('valid_experiment'),
        statistical_success_rate_claim=False)
    trial.verify_unchanged()
    result['input_sha256'] = dict(trial.reader.hashes)
    return result, trial


def world_transform(bounds, rect):
    xmin, xmax, ymin, ymax = bounds
    left, top, right, bottom = rect
    scale = min((right-left-24)/(xmax-xmin), (bottom-top-24)/(ymax-ymin))
    cx, cy = .5*(xmin+xmax), .5*(ymin+ymax)
    def xy(x, y):
        return .5*(left+right)+(x-cx)*scale, .5*(top+bottom)-(y-cy)*scale
    return xy, scale


def prediction(draw, xy, trial, t):
    """Draw CV only from a fresh recorded detector input, not actor truth."""
    row = latest(trial.diagnostics, t)
    if row is None or not 0 <= t-float(row['t']) < .35:
        return 0
    age = float(row.get('observed_obstacle_age', math.inf))+t-float(row['t'])
    validity = float(trial.manifest['long_goal_crossing_experiment'].get('prediction_s',
                        trial.manifest.get('local_prediction_s', 0.)))
    if not 0 <= age < validity-.01:
        return 0
    horizon = validity-age-.01
    count = 0
    for x, y, vx, vy, radius, moving in row.get('observed_obstacles', []):
        if not moving:
            continue
        points = [xy(x+(age+a)*vx, y+(age+a)*vy) for a in np.linspace(0., horizon, 11)]
        for i in range(len(points)-1):
            if i % 2 == 0:
                draw.line(points[i:i+2], fill='#8761a8', width=2)
        end = points[-1]
        angle = math.atan2(end[1]-points[0][1], end[0]-points[0][0])
        draw.line([(end[0]-6*math.cos(angle-.5), end[1]-6*math.sin(angle-.5)), end,
                   (end[0]-6*math.cos(angle+.5), end[1]-6*math.sin(angle+.5))], fill='#8761a8', width=2)
        count += 1
    return count


def panel(image, trial, t, rect):
    _, goal, bounds = settings(trial.manifest)
    xy, scale = world_transform(bounds, rect)
    layer = Image.new('RGB', image.size, 'white')
    draw = ImageDraw.Draw(layer)
    draw.rectangle([xy(bounds[0], bounds[3]), xy(bounds[1], bounds[2])], outline='#cccccc', width=2)
    gx, gy = xy(*goal)
    draw.line([(gx-6, gy), (gx+6, gy)], fill='#555555', width=2)
    draw.line([(gx, gy-6), (gx, gy+6)], fill='#555555', width=2)
    for ox, oy, radius, *_ in scene_actors(trial.manifest, trial.result, t):
        px, py = xy(ox, oy)
        r = radius*scale
        draw.ellipse((px-r, py-r, px+r, py+r), fill='#999999', outline='#666666')
    reference = trial.reference_at(t)
    if reference is not None and len(reference) > 1:
        draw.line([xy(*p[:2]) for p in reference], fill='#d78224', width=3)
    row, index = trial.state_at(t)
    if index:
        draw.line([xy(x, y) for x, y in zip(trial.state['x'][:index+1], trial.state['y'][:index+1])], fill='#2874a6', width=3)
    search = trial.search_at(t)
    if search is not None:
        rows = np.asarray(search['rows'])
        if rows.ndim == 2 and len(rows):
            for ident in np.unique(rows[:, 0]):
                path = rows[rows[:, 0] == ident]
                if len(path) > 1:
                    draw.line([xy(x, y) for x, y in path[:, 2:4]], fill='#b5b5b5', width=1)
    selected = trial.selected_at(t)
    if selected is not None:
        draw.line([xy(*p[:2]) for p in selected], fill='#258446', width=3)
    prediction(draw, xy, trial, t)
    c, s = math.cos(row['yaw']), math.sin(row['yaw'])
    # Project every recorded native physical sphere, including the fixed arm.
    # Duplicate heights share an outline; no point-robot fallback is allowed.
    disks = set()
    for bx, by, bz, radius, _, _, kind in trial.body3d:
        physical_radius = radius-(trial.resolution if kind == 0 else 0.)
        disks.add((bx, by, physical_radius))
    for bx, by, radius in sorted(disks):
        px, py = xy(row['x']+c*bx-s*by, row['y']+s*bx+c*by)
        r = radius*scale
        draw.ellipse((px-r, py-r, px+r, py+r), fill='#e5edf4', outline='#9db0c0', width=1)
    length, width = trial.geometry['base_length']/2, trial.geometry['base_width']/2
    corners = [(-length, -width), (length, -width), (length, width), (-length, width), (-length, -width)]
    draw.line([xy(row['x']+c*x-s*y, row['y']+s*x+c*y) for x, y in corners], fill='#2874a6', width=2)
    draw.line([xy(row['x'], row['y']), xy(row['x']+.27*c, row['y']+.27*s)], fill='#164a73', width=3)
    image.paste(layer.crop(rect), (rect[0], rect[1]))
    return row


def render(trial, out, max_frames=180):
    if max_frames < 2:
        raise ValueError('at least two GIF frames required')
    duration = trial.end-trial.anchor
    ages = np.linspace(0., duration, min(max_frames, max(2, math.ceil(duration/.25)+1)))
    frame_ms = max(20, 10*round(1000*float(ages[1]-ages[0])/2./10))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1040, 615), 'white')
        draw = ImageDraw.Draw(image)
        t = min(trial.anchor+float(age), trial.end)
        draw.text((20, 9), '远目标·中段穿越　%.1f s（起步后）' % age, font=font, fill='#333333')
        draw.text((20, 39), '蓝：机器人及实走轨迹　橙：REMANI参考　灰：真实搜索子集　绿：获准展开　紫虚线：CV预测', font=small, fill='#555555')
        row = panel(image, trial, t, (20, 78, 1020, 535))
        draw = ImageDraw.Draw(image)
        draw.text((20, 548), '%s　v=%+.2f m/s　ω=%+.2f rad/s' % (trial.status_at(t), row['v'], row['w']), font=small, fill='#444444')
        draw.text((20, 582), '固定臂差速 ROS 数值闭环；非Gazebo/实车。只回放实录；二维显示投影不代替三维碰撞判据。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'crossing.gif', save_all=True, append_images=frames[1:],
                   duration=[frame_ms]*(len(frames)-1)+[max(800, frame_ms)], loop=0, disposal=2, optimize=True)
    trial.verify_unchanged()
    return dict(frames=len(frames), speedup=2., actual_robot_projection=True,
        actual_remani_reference=True, actual_recorded_mppi_search_subset=True,
        cv_from_recorded_detector_inputs=True, no_controller_reexecution=True,
        end_t=trial.end)


def chart(trial, result, out):
    """One concise time-series figure; all curves stop at measured termination."""
    image = Image.new('RGB', (1060, 760), 'white')
    draw = ImageDraw.Draw(image)
    font, small = ImageFont.truetype(FONT, 21), ImageFont.truetype(FONT, 14)
    draw.text((24, 12), '远目标·中段穿越：实际速度与规划耗时', font=font, fill='#333333')
    start, end = result['goal_sent_t'], trial.end
    actor = result['first_truth_actor_t']
    rows = measured_rows(trial, start)
    diagnostic = [r for r in trial.diagnostics if start <= r['t'] < end]
    requests = [r for r in result['remani']['requests'] if r.get('end_sim_s') is not None and start <= r['end_sim_s'] <= end]
    specs = [
        ('实测底盘线速度 / m/s', [(np.array(rows['t']), np.array(rows['v']), '#2874a6', '实际速度')],
         max(.05, float(max(np.abs(rows['v']))) if len(rows) else .05), True),
        ('MPPI求解及验收 / ms', [(np.array([r['t'] for r in diagnostic]), np.array([r.get('callback_ms', np.nan) for r in diagnostic]), '#2874a6', '求解+验收'),
                               (np.array([r['t'] for r in diagnostic]), np.array([r.get('kernel_ms', np.nan) for r in diagnostic]), '#8fa8b6', '内核')],
         max(210., max([r.get('callback_ms', 0.) for r in diagnostic]+[0.])), False),
        ('REMANI整次规划请求 / ms（按请求结束时刻）', [(np.array([r['end_sim_s'] for r in requests]), np.array([r.get('request_wall_ms', np.nan) for r in requests]), '#d78224', '请求墙钟耗时')],
         max(50., max([r.get('request_wall_ms', 0.) for r in requests]+[0.])), False)]
    for index, (title, curves, ymax, signed) in enumerate(specs):
        left, top, right, bottom = 88, 98+index*205, 1015, 240+index*205
        ymin = -ymax*1.08 if signed else 0.
        ymax *= 1.08
        def xy(t, v):
            return left+(t-start)/max(end-start, .001)*(right-left), bottom-(v-ymin)/(ymax-ymin)*(bottom-top)
        draw.text((24, top-27), title, font=small, fill='#333333')
        for j in range(4):
            value = ymin+(ymax-ymin)*j/3
            yy = xy(start, value)[1]
            draw.line([(left, yy), (right, yy)], fill='#eeeeee')
            draw.text((19, yy-8), '%.2f' % value if signed else '%.0f' % value, font=small, fill='#777777')
        draw.rectangle((left, top, right, bottom), outline='#cccccc')
        for j in range(5):
            t = start+(end-start)*j/4
            xx = xy(t, 0)[0]
            draw.text((xx-14, bottom+5), '%.1f' % (t-start), font=small, fill='#777777')
        if actor is not None and start <= actor <= end:
            xx = xy(actor, 0)[0]
            draw.line([(xx, top), (xx, bottom)], fill='#b5b5b5', width=1)
            if index == 0:
                draw.text((xx+4, top+3), '障碍出现', font=small, fill='#777777')
        if index == 1:
            deadline = 1000*trial.manifest.get('execution_timing_pair', {}).get('callback_deadline_s', .2)
            if deadline <= ymax:
                yy = xy(start, deadline)[1]
                draw.line([(left, yy), (right, yy)], fill='#b2615a', width=1)
                draw.text((right-105, yy-19), '截止 %.0f ms' % deadline, font=small, fill='#b2615a')
        for times, values, color, label in curves:
            points = [xy(t, v) for t, v in zip(times, values) if math.isfinite(v)]
            if len(points) > 1 and index != 2:
                draw.line(points, fill=color, width=2)
            for px, py in points if index == 2 else []:
                draw.line([(px, bottom), (px, py)], fill=color, width=2)
                draw.ellipse((px-3, py-3, px+3, py+3), fill=color)
        draw.text((right-250, bottom+26), '发目标后时间 / s', font=small, fill='#777777')
    draw.text((24, 726), '蓝：实测速度 / MPPI求解及验收　浅蓝：MPPI内核　橙：REMANI整次请求；均截止实际结束时刻。', font=small, fill='#666666')
    image.save(out/'timeseries.png')


def narrative(result):
    scene, limits = result['scene_definition'], result['recorded_limits']
    actor = scene['actors'][0]
    timing = result['after_goal']['phase_timing']['overall']['callback_ms'] or {}
    stopped, remani = result['measured_stop'], result['remani']
    deadline = result['actor_window']['deadline']
    return '\n'.join([
        '本轮设置',
        '固定机械臂、差速底盘，完整REMANI＋MPPI在线数值闭环；不是Gazebo或实车。',
        '目标点%s；地图边界%s米。' % (result['experiment_settings']['goal'], result['experiment_settings']['bounds']),
        '底盘线/角速度上限%s m/s、%s rad/s；单轮线速度上限%s m/s。实际修改明细看report.json的experiment_settings。' %
            (show(limits.get('vmax'), 3), show(limits.get('wmax'), 3), show(limits.get('wheel_v'), 3)),
        'CV和MPPI预测时间设置%s秒，预测距离随实际速度变化，不是固定长度。' %
            show(result['experiment_settings'].get('prediction_s', limits.get('max_prediction'))),
        '障碍初始位置(%s,%s)米，速度(%s,%s) m/s；起步后%s秒出现，沿路径中段穿越。' %
            (show(actor['x']), show(actor['y']), show(actor['vx']), show(actor['vy']), show(scene['appearance_s'])),
        '', '本次实测',
        '结束原因：%s；到达目标：%s；三维物理采样接触：%s。' %
            (result['termination'], '是' if result['reached_goal'] else '否', '有' if result['physical_sample_contact'] else '无'),
        '最小三维物理采样间隙%s米；结束时距目标%s米。负间隙表示采样外形已相交。' %
            (show(result['minimum_3d_physical_sample_gap_m'], 4), show(result['final_goal_distance_m'])),
        '障碍出现后，实际累计停车%s秒，最长连续停车%s秒。停车按实测线/角速度统计，不把零指令等同于停车。' %
            (show(stopped['total_s']), show(stopped['longest_s'])),
        '发目标后MPPI求解及验收段P50/P95：%s/%s毫秒；这是该段耗时，不是整个两轮间隔。' %
            (show(timing.get('p50')), show(timing.get('p95'))),
        '障碍出现后超期标签%d次，其中通过检查的非零动作仅因超期被清零%d次。' %
            (deadline['label_count'], deadline['validated_nonzero_proposals_cleared']),
        'REMANI共启动%d次规划请求，其中重规划%d次；已完成并接受%d次；共使用参考%d条，障碍出现后新参考%d条。' %
            (remani['total_requests_started'], remani['refresh_requests_started'], remani['accepted_completed_requests'], remani['consumed_references'], remani['new_references_after_actor']),
        '重规划结果分布：%s。' % json.dumps(remani['refresh_outcomes'], ensure_ascii=False),
        '', '文件说明',
        'crossing.gif：实录回放，显示完整机器人投影、实际REMANI参考、记录的MPPI搜索子集、获准展开与记录CV预测。',
        'timeseries.png：实际速度、MPPI求解及验收耗时、REMANI整次规划请求耗时。',
        'report.json：上述数字、详细设置、输入文件校验值及请求记录。',
        '这是一次场景观察，不据此宣称总体成功率，也不在本报告提前归因。二维绘图投影不是三维碰撞判据。', ''])


def report(source=SOURCE, out=OUTPUT, make_gif=True):
    result, trial = summarize(source)
    out = Path(out).resolve()
    if out == trial.source or trial.source in out.parents:
        raise ValueError('report output must stay outside measured trial directory')
    out.mkdir(parents=True, exist_ok=True)
    if make_gif:
        result['visualization'] = render(trial, out)
    chart(trial, result, out)
    trial.verify_unchanged()
    result['input_sha256'] = dict(trial.reader.hashes)
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验总结.txt').write_text(narrative(result))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial', type=Path, default=SOURCE)
    parser.add_argument('--out', type=Path, default=OUTPUT)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    result = report(args.trial, args.out, not args.no_gif)
    print(json.dumps(dict(output=str(args.out.resolve()), termination=result['termination'],
                         reached_goal=result['reached_goal'], physical_sample_contact=result['physical_sample_contact']), ensure_ascii=False))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Locate static detour width in recorded search/initialization/optimization.

Read completed SIL runs only. Match an optimizer result to an actually consumed
reference through the native publication request and start stamp. No controller
replay, invented search points, or failed-candidate-as-delivered comparisons.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from report_long_goal_crossing import settings, world_transform
from report_static_agility import crossing_y, gif, show, summarize


STAGES = ('native_search_raw', 'minco_before_optimizer', 'optimizer_return')
NAMES = ('搜索输出路点', '优化前初始曲线', '优化后交付曲线')
COLORS = ('#78909c', '#397eaf', '#d78128')


def sample_curve(row, times=None):
    counts = [row[k] for k in ('minco_count', 'gear_count', 'parts_recorded') if k in row]
    if counts and len(set(counts+[len(row['parts'])])) != 1:
        raise ValueError('incomplete MINCO/gear capture')
    pieces = [p for part in row['parts'] for p in part['pieces']]
    if not pieces:
        return np.empty((0, 2))
    durations = np.array([p['T'] for p in pieces], dtype=float)
    ends = np.cumsum(durations)
    starts = np.r_[0., ends[:-1]]
    if times is None:
        times = np.concatenate([start+np.linspace(0., duration,
            max(2, int(math.ceil(duration/.01))+1))
            for start, duration in zip(starts, durations)])
    times = np.asarray(times, dtype=float)
    if np.any(times < -1e-7) or np.any(times > ends[-1]+1e-7):
        raise ValueError('reference times outside captured polynomial duration')
    ids = np.minimum(np.searchsorted(ends, times, side='right'), len(pieces)-1)
    positions = np.empty((len(times), 2))
    for index, piece in enumerate(pieces):
        if piece['basis'] != 'descending':
            raise ValueError('unexpected polynomial basis')
        mask = ids == index
        coeffs = np.asarray(piece['coeffs'], dtype=float)
        local = times[mask]-starts[index]
        positions[mask] = np.array([np.polyval(coeffs[axis], local) for axis in (0, 1)]).T
    if not np.isfinite(positions).all():
        raise ValueError('nonfinite captured curve')
    return positions


def geometry(row):
    if row['stage'] == 'native_search_raw':
        points = [p[:2] for part in row['parts'] for p in part['points']]
        return np.asarray(points, dtype=float).reshape((-1, 2))
    return sample_curve(row)


def suffix_at_x(path, x):
    """Keep path from its first geometric crossing, without extrapolation."""
    for index, (a, b) in enumerate(zip(path, path[1:])):
        if abs(b[0]-a[0]) > 1e-12 and min(a[0], b[0]) <= x <= max(a[0], b[0]):
            y = a[1]+(x-a[0])/(b[0]-a[0])*(b[1]-a[1])
            return np.vstack(([x, y], path[index+1:]))
    return np.empty((0, 2))


def metrics(path, obstacle_x):
    if not len(path):
        return None
    index = int(np.argmax(np.abs(path[:, 1])))
    suffix = suffix_at_x(path, obstacle_x)
    length = lambda a: float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum())
    return dict(points=len(path), max_abs_y_m=float(abs(path[index, 1])),
        peak_x_m=float(path[index, 0]), peak_y_m=float(path[index, 1]),
        path_length_m=length(path), start_xy=path[0].tolist(), end_xy=path[-1].tolist(),
        y_at_obstacle_x_m=crossing_y(path, obstacle_x),
        obstacle_to_goal_length_m=length(suffix) if len(suffix) else None)


def inspect_trial(source, label):
    result, trial = summarize(source, label)
    trace = sorted(trial.reader.jsonl('static_width_trace.jsonl'), key=lambda r: r['capture_mono'])
    if not trace or any(r['schema'] != 'static_width_v1' for r in trace):
        raise ValueError('missing/unsupported width trace')
    timeline = trial.reader.jsonl('dual_timeline.jsonl')
    pubs = [r for r in timeline if r['event'] == 'reference_publish']
    bindings = []
    for pub in pubs:
        matches = [r for r in trial.refs if abs(r['stamp']-pub['guide_start']) < 1e-7]
        if len(matches) > 1:
            raise ValueError('ambiguous consumed reference stamp')
        if matches:
            bindings.append((pub, matches[0]))
    obstacle_x = float(trial.manifest['scenario']['actors'][0]['x'])
    attempts, pictures = [], []
    for final in [r for r in trace if r['stage'] == 'optimizer_return']:
        request = final['request_id']
        initials = [r for r in trace if r['request_id'] == request and
            r['resource_replan_id'] == final['resource_replan_id'] and
            r['stage'] == 'minco_before_optimizer' and r['capture_mono'] < final['capture_mono']]
        if not initials:
            raise ValueError('optimizer return lacks paired initial curve')
        initial = initials[-1]
        searches = [r for r in trace if r['request_id'] == request and
            r['stage'] == 'native_search_raw' and r['capture_mono'] < initial['capture_mono']]
        search = searches[-1] if searches else None
        # A failed solve may leave stale or partially updated internal curves.
        # Preserve its outcome but never interpret that storage as a candidate.
        paths = [geometry(r) if r else np.empty((0, 2))
                 for r in (search, initial, final if final['success'] else None)]
        matched = [(p, ref) for p, ref in bindings if p['request_id'] == request and
            p['mono'] >= final['capture_mono'] and final['success']]
        # Only the last successful result before that publication can supply it.
        matched = [(p, ref) for p, ref in matched if not any(
            r['request_id'] == request and r['stage'] == 'optimizer_return' and r['success'] and
            final['capture_mono'] < r['capture_mono'] <= p['mono'] for r in trace)]
        if len(matched) > 1:
            raise ValueError('multiple publications for a captured optimizer result')
        state = next((r for r in timeline if r['event'] == 'handoff_state' and
            r['request_id'] == request and r['stage'] == 'before_solve'), None)
        row = dict(request_id=request, resource_replan_id=final['resource_replan_id'],
            optimizer_success=final['success'], actually_consumed=bool(matched),
            capture_sim=final['capture_sim'], search_status=search['status'] if search else None,
            stages={name: metrics(path, obstacle_x) for name, path in zip(STAGES, paths)},
            start_state=None if state is None else {k: state[k] for k in (
                'applied_x', 'applied_y', 'applied_yaw', 'applied_body_vx', 'applied_w',
                'processed_vx', 'processed_vy')},
            rejection_reasons=[r.get('reason') for r in timeline if r['request_id'] == request and
                r['event'] == 'acceptance_reject'])
        row['planned_curve_duration_s'] = {r['stage']: float(sum(
            p['T'] for part in r['parts'] for p in part['pieces']))
            for r in (initial, final) if r['stage'] != 'optimizer_return' or r['success']}
        request_summary = next((r for r in result['remani']['requests'] if r['request_id'] == request), {})
        row['native_request_wall_ms'] = request_summary.get('request_wall_ms')
        if search and search['parts']:
            first = search['parts'][0]
            points = np.asarray(first['points'], dtype=float)
            yaws = np.asarray(first['yaw'], dtype=float)
            if len(points) == len(yaws) and len(points):
                count = 1
                while count < len(yaws) and abs(math.atan2(math.sin(yaws[count]-yaws[0]),
                        math.cos(yaws[count]-yaws[0]))) < 1e-8:
                    count += 1
                row['search_initial_constant_heading'] = dict(points=count, yaw_rad=float(yaws[0]),
                    start_xy=points[0, :2].tolist(), end_xy=points[count-1, :2].tolist(),
                    polyline_length_m=float(np.linalg.norm(np.diff(points[:count, :2], axis=0), axis=1).sum()))
        if matched:
            pub, reference = matched[0]
            predicted = sample_curve(final, reference['times'])
            recorded = np.asarray(reference['reference'], dtype=float)[:, :2]
            mismatch = float(np.max(np.linalg.norm(predicted-recorded, axis=1)))
            if mismatch > 1e-6:
                raise ValueError('captured final polynomial differs from consumed reference: %.9g m' % mismatch)
            row.update(reference_stamp=reference['stamp'], consumed_t=reference['t'],
                reference_sha256=reference['sha256'], polynomial_reference_max_error_m=mismatch,
                published_sim=pub['sim'])
            if reference['t'] > result['metric_window_start_t']:
                pictures.append((row, paths))
        attempts.append(row)
    result['width_diagnosis'] = dict(attempts=attempts,
        adopted_refresh_request_ids=[row['request_id'] for row, _ in pictures],
        obstacle_x_m=obstacle_x, curve_sample_step_upper_s=.01,
        association='request_id + resource_replan_id; reference_publish.guide_start == consumed_reference.stamp; polynomial checked at every consumed reference time')
    result['input_sha256'] = dict(trial.reader.hashes)
    trial.verify_unchanged()
    return result, trial, pictures


def draw_geometry(rows, trials, pictures, destination):
    chosen = [next(iter(p), None) for p in pictures]
    if any(p is None for p in chosen):
        raise ValueError('each panel requires an actually adopted refresh')
    image = Image.new('RGB', (1160, 620), 'white')
    draw = ImageDraw.Draw(image)
    title, normal, small = [ImageFont.truetype(FONT, size) for size in (21, 17, 14)]
    draw.text((20, 12), '绕行在哪里变宽：同一次实际交付请求的三阶段', font=title, fill='#333333')
    for index, (name, color) in enumerate(zip(NAMES, COLORS)):
        x = 22+index*330
        draw.line(((x, 57), (x+34, 57)), fill=color, width=3)
        draw.text((x+44, 46), name, font=normal, fill=color)
    for index, (result, trial, (row, paths)) in enumerate(zip(rows, trials, chosen)):
        left, right = index*580+18, (index+1)*580-18
        draw.text((left, 86), result['label']+'：实际采用请求 '+str(row['request_id']), font=normal, fill='#333333')
        _, goal, bounds = settings(trial.manifest)
        xy, scale = world_transform(bounds, (left, 120, right, 410))
        draw.rectangle([xy(bounds[0], bounds[3]), xy(bounds[1], bounds[2])], outline='#d0d0d0')
        for actor in trial.manifest['scenario']['actors']:
            x, y = xy(actor['x'], actor['y'])
            radius = actor['r']*scale
            draw.ellipse((x-radius, y-radius, x+radius, y+radius), fill='#cccccc', outline='#999999')
        draw.line([xy(0, 0), xy(*goal)], fill='#e6e6e6', width=1)
        for path, color in zip(paths, COLORS):
            if len(path) > 1:
                draw.line([xy(*p) for p in path], fill=color, width=2)
        for point in paths[0]:
            x, y = xy(*point)
            draw.ellipse((x-3, y-3, x+3, y+3), fill=COLORS[0])
        start = paths[1][0]
        x, y = xy(*start)
        draw.ellipse((x-4, y-4, x+4, y+4), fill='#333333')
        for line, (name, color, stage) in enumerate(zip(NAMES, COLORS, STAGES)):
            metric = row['stages'][stage]
            text = name+'：无记录' if metric is None else '%s：最宽 %.2f m，发生于 x=%.2f m' % (
                name, metric['max_abs_y_m'], metric['peak_x_m'])
            draw.text((left, 430+line*26), text, font=small, fill=color)
        state = row['start_state']
        draw.text((left, 516), '起点 (%.2f, %.2f)，速度 %.2f m/s，朝向 %+.1f°' % (
            state['applied_x'], state['applied_y'], state['applied_body_vx'],
            math.degrees(state['applied_yaw'])), font=small, fill='#555555')
    draw.text((20, 558), '最宽：相对原直线 y=0 的最大横向距离；曲线均为实际记录，不是重新求解或示意图。', font=small, fill='#666666')
    draw.text((20, 586), '每组仅画障碍出现后首次实际采用的新参考；失败候选另列在TXT/JSON，不混入交付指标。', font=small, fill='#666666')
    image.save(destination)


def narrative(rows):
    lines = ['静态绕行变宽：搜索、初始曲线、优化后曲线逐段定位', '', '一、先说这次定位结果']
    selected = [next((a for a in row['width_diagnosis']['attempts']
        if a['actually_consumed'] and a['consumed_t'] > row['metric_window_start_t']), None) for row in rows]
    for row, attempt in zip(rows, selected):
        if attempt is None:
            lines.append(row['label']+'没有实际采用的新参考，不能做三阶段交付比较。')
            continue
        values = [attempt['stages'][stage]['max_abs_y_m'] for stage in STAGES]
        lines.append('%s：搜索输出路点最宽%.3f米 → 优化前曲线%.3f米 → 实际交付曲线%.3f米。' %
            (row['label'], values[0], values[1], values[2]))
    enabled = selected[-1]
    if enabled and enabled['stages'][STAGES[0]]:
        a, b, c = [enabled['stages'][stage]['max_abs_y_m'] for stage in STAGES]
        if abs(b-a) < .03 and c < b:
            lines += ['本次打开提前绕行后，宽绕在前端搜索输出时已经存在。MINCO生成初始曲线几乎没有进一步扩大，后端优化反而略微收窄。',
                '因此，这次不能把“绕远”归因于后端把一条原本很窄的路线优化坏了。',
                '早停交付保留了这条较宽的可行路线，但本轮没有关闭早停的同输入对照，不能证明早停就是宽绕的根因。']
        first_run = enabled.get('search_initial_constant_heading')
        if first_run and first_run['points'] > 1:
            lines += ['更具体地看，打开组搜索输出的前%d个路点朝向同为%+.2f度；从(%.3f, %.3f)沿向外斜线走了%.3f米，到(%.3f, %.3f)才结束这段同朝向路点。' % (
                first_run['points'], math.degrees(first_run['yaw_rad']), *first_run['start_xy'],
                first_run['polyline_length_m'], *first_run['end_xy']),
                '也就是说，经过障碍物所在的x=3米位置后，搜索路线还在继续向外绕；不是后端优化才出现这个走势。']
    lines += ['需要继续区分的是：前端为什么选择这条宽路线。两组重规划起点的位置、朝向不同，本轮不能把原因只归给其中某一项。',
        '补充纠正：当前搜索几何主要使用位置、朝向和挡位，两组都是前进挡。实测速度大小主要影响时间分配，不能直接说“速度较快把A*路径甩宽了”。',
        '本次只是查明宽绕首先出现在哪一步，没有修改控制算法来消除宽绕。', '',
        '二、这次做了什么',
        '重新运行两组完整REMANI+MPPI在线数值闭环：关闭与打开“提前绕行＋局部范围修正”。',
        '只增加搜索结果、优化前曲线、优化后曲线的记录；未修改避障代价、轮速约束或安全距离。',
        '不是Gazebo或实车。两组是新跑的闭环，不是历史GIF的确定性重放。', '',
        '三、实际记录到什么']
    for row in rows:
        lines += ['', row['label']+'：'+('到达目标。' if row['reached_goal'] else '未到达目标。'),
            '实走%.3f米，用时%.3f秒；物理球采样最小间隙%.3f米。' % (
                row['actual_path_length_m'], row['motion_duration_s'], row['minimum_physical_sample_gap_m'])]
        for attempt in row['width_diagnosis']['attempts']:
            lines.append('请求%d：优化%s，%s。' % (attempt['request_id'],
                '成功' if attempt['optimizer_success'] else '失败',
                '实际被MPPI采用' if attempt['actually_consumed'] else '没有作为实际采用的新参考'))
            for name, stage in zip(NAMES, STAGES):
                m = attempt['stages'][stage]
                if m:
                    lines.append('  %s：最大横向距离%.3f米（在x=%.3f米处）；全段长%.3f米，障碍位置之后到目标长%s米。' % (
                        name.replace('交付', '') if not attempt['actually_consumed'] else name,
                        m['max_abs_y_m'], m['peak_x_m'], m['path_length_m'], show(m['obstacle_to_goal_length_m'])))
            if attempt['rejection_reasons']:
                lines.append('  后续拒收原因：'+str(attempt['rejection_reasons']))
            if attempt['actually_consumed']:
                lines.append('  已按发布时间戳关联实际采用参考，并逐点评估多项式核对；最大误差%.3g米。' %
                    attempt['polynomial_reference_max_error_m'])
                lines.append('  本次REMANI请求计算耗时%s毫秒；规划的剩余轨迹时长%s秒。后者不是计算耗时，也不等于实际机器人到达用时。' % (
                    show(attempt['native_request_wall_ms']), show(attempt['planned_curve_duration_s'].get('optimizer_return'))))
    lines += ['', '四、结论边界',
        '同一请求的三条曲线能定位“在哪一步变宽”；两组起始状态不同，不能把两组差值全归因于某一个参数。',
        '搜索输出路点已经过原生getTrajsWithTime/evaluatePos与SampleMani处理，不是A*原始节点或全部扩展树；初始曲线是实际送入后端前的MINCO曲线。',
        '成功优化不等于交付，交付不等于最短。只有匹配实际采用参考的候选才用于主图。失败返回时的内部曲线可能是残留缓存，本报告不把它当成本次最终候选。',
        '具体原因要结合三阶段变化判断，不能仅凭最终GIF认定是障碍膨胀或轮速约束。', '']
    lines += ['五、下一步只做什么',
        '固定同一个重规划起点和同一张地图，检查前端能否在绕过障碍后更早回到目标方向。',
        '代码里有转向和转向变化的惩罚，可能偏好少转弯，但本轮没有做同起点的单项对照，不能宣布它就是唯一根因。',
        '先查前端的路线选择，不继续增加优化预算，也不靠放宽安全距离来掩盖宽绕。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--treatment', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--no-gif', action='store_true')
    args = parser.parse_args()
    values = [inspect_trial(args.baseline, '关闭提前绕行与范围修正'),
              inspect_trial(args.treatment, '提前绕行＋局部范围修正')]
    rows, trials, pictures = map(list, zip(*values))
    if trials[0].manifest['scenario'] != trials[1].manifest['scenario']:
        raise ValueError('different scenes')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('different collision geometry')
    output = args.output.resolve()
    if any(output == t.source or t.source in output.parents for t in trials):
        raise ValueError('report must be outside source trials')
    output.mkdir(parents=True, exist_ok=True)
    draw_geometry(rows, trials, pictures, output/'三阶段轨迹定位.png')
    rendering = None if args.no_gif else gif(rows, trials, output/'静态闭环对照.gif', 100)
    for trial in trials:
        trial.verify_unchanged()
    (output/'summary.json').write_text(json.dumps(dict(trials=rows, rendering=rendering), ensure_ascii=False, indent=2)+'\n')
    (output/'实验结果.txt').write_text(narrative(rows))
    print(json.dumps(dict(output=str(output), adopted_refreshes=[r['width_diagnosis']['adopted_refresh_request_ids'] for r in rows]), ensure_ascii=False))


if __name__ == '__main__':
    main()

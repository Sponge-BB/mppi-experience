#!/usr/bin/env python3
"""Render frozen rejected-candidate diagnostics; never rerun a controller."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.patches import Circle
import numpy as np
from PIL import Image
import yaml

from render_dual_timescale import FONT

REASONS = {'ok': '通过', 'planning_collision': '碰撞',
    'left_wheel_speed': '左轮速度超限', 'right_wheel_speed': '右轮速度超限',
    'left_wheel_acceleration': '左轮加速度超限',
    'right_wheel_acceleration': '右轮加速度超限'}
COLORS = ('#888888', '#d7922f', '#246da0')
LABELS = ('初始曲线', '中间候选', '最后候选')


def load(path):
    return json.loads(Path(path).read_text())


def finite(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)


def shown(x, places=3):
    return '%.*f' % (places, x) if finite(x) else '未记录'


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(stage):
    return stage['audit']


def samples(stage, key='samples'):
    return audit(stage).get(key, [])


def array(rows, key):
    return np.asarray([r.get(key, np.nan) if r.get(key) is not None else np.nan for r in rows], dtype=float)


def summary(stage):
    return audit(stage)['summary']


def wheel_ratio(stage, rows_key='samples'):
    rows = samples(stage, rows_key)
    limits = summary(stage)
    speed_limit = limits['omega_hard_limit_with_tolerance']
    alpha_limit = limits['alpha_hard_limit_with_tolerance']
    ratios = [np.abs(array(rows, 'wheel_'+side+'_'+kind))/limit
              for side in ('left', 'right')
              for kind, limit in (('omega', speed_limit), ('alpha', alpha_limit))]
    return float(np.nanmax(ratios)) if rows else None


def selected(stages):
    detailed = [s for s in stages if samples(s)]
    if len(detailed) < 2:
        raise ValueError('Need initial and final detailed native-grid samples')
    if len(detailed) <= 3:
        return detailed
    return [detailed[0], detailed[len(detailed)//2], detailed[-1]]


def projection(ax, body, row, color, alpha=.15):
    yaw = row['yaw']
    rot = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    points = np.asarray(body)
    centers = points[:, :2]@rot.T + np.asarray([row['x'], row['y']])
    for center, ball in zip(centers, points):
        ax.add_patch(Circle(center, ball[3], fc=color, ec=color, lw=.25, alpha=alpha))


def plot_paths(case, body, out, obstacle_xy=None, static_slice=None):
    fig, ax = plt.subplots(figsize=(9, 4.6), constrained_layout=True)
    if static_slice is not None:
        xs, ys = static_slice
        ax.scatter(xs, ys, s=3, c='#bdbdbd', alpha=.55,
                   label='原生ESDF障碍切片（z≈0.25 m）', rasterized=True)
    if obstacle_xy is not None and len(obstacle_xy):
        ax.scatter(obstacle_xy[:, 0], obstacle_xy[:, 1], s=5, c='#c9c4cd', alpha=.55,
                   label='冻结的动态预测占用（投影）', rasterized=True)
    stages = selected(case['stages'])
    for i, stage in enumerate(stages):
        rows = samples(stage)
        color = COLORS[0 if i == 0 else 2 if i == len(stages)-1 else 1]
        label = LABELS[0 if i == 0 else 2 if i == len(stages)-1 else 1]
        ax.plot(array(rows, 'x'), array(rows, 'y'), color=color, lw=1.7, label=label)
        if i == len(stages)-1:
            for index in np.linspace(0, len(rows)-1, 4, dtype=int):
                projection(ax, body, rows[index], color)
    final = stages[-1]
    t = final.get('native_reject_time')
    if finite(t):
        rows = samples(final)
        row = min(rows, key=lambda r: abs(r['t']-t))
        projection(ax, body, row, '#b5372c', .10)
        ax.scatter([row['x']], [row['y']], marker='x', s=65, color='#b5372c', zorder=5,
                   label='原生验收首次拒绝位置')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x / m')
    ax.set_ylabel('y / m')
    ax.set_title(case['title']+'：初始曲线到最后候选')
    ax.grid(alpha=.18)
    ax.legend(loc='best', fontsize=9)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def plot_wheels(case, out):
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True, constrained_layout=True)
    stages = selected(case['stages'])
    final = stages[-1]
    limits = summary(final)
    for row, (kind, limit_key, unit, text) in enumerate((
            ('omega', 'omega_hard_limit_with_tolerance', 'rad/s', '轮速'),
            ('alpha', 'alpha_hard_limit_with_tolerance', 'rad/s²', '轮加速度'))):
        limit = limits[limit_key]
        for col, (side, name) in enumerate((('left', '左'), ('right', '右'))):
            ax = axes[row, col]
            for i, stage in enumerate(stages):
                index = 0 if i == 0 else 2 if i == len(stages)-1 else 1
                data = samples(stage)
                ax.plot(array(data, 't'), array(data, 'wheel_'+side+'_'+kind),
                        color=COLORS[index], lw=1.05, alpha=.9, label=LABELS[index])
            data = samples(final, 'optimizer_samples')+samples(final, 'feasibility_optimizer_samples')
            if data:
                ax.scatter(array(data, 't'), array(data, 'wheel_'+side+'_'+kind),
                           s=7, color='#151515', alpha=.5, label='最后候选的优化采样点', zorder=5)
            for sign in (-1, 1):
                ax.axhline(sign*limit, ls='--', color='#b5372c', lw=.9,
                           label='验收上限（含5%容差）' if sign == 1 else None)
            ax.axvspan(0, .05, color='#dddddd', alpha=.4)
            ax.set_title(name+text)
            ax.set_ylabel(unit)
            ax.grid(alpha=.16)
            magnitude = max(np.nanmax(np.abs(array(samples(stage), 'wheel_'+side+'_'+kind)))
                            for stage in stages)
            if magnitude > 20*limit:
                ax.set_yscale('symlog', linthresh=limit)
                ax.set_ylabel(unit+'（对称对数）')
            if row == 1:
                ax.set_xlabel('轨迹相对时间 / s')
    axes[0, 0].legend(fontsize=8, loc='best')
    fig.suptitle(case['title']+'：灰色起始0.05秒不属于原生验收区间', fontsize=12)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def plot_trend(case, out):
    stages = case['stages']
    xs = [s.get('iteration', i) for i, s in enumerate(stages)]
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True, constrained_layout=True)
    for key, label in (('min_base_hard_gap_m', '底盘'), ('min_arm_hard_gap_m', '机械臂')):
        values = [summary(s).get(key) for s in stages]
        axes[0].plot(xs, values, marker='.', ms=4, lw=1.2, label=label)
    axes[0].axhline(0, color='#b5372c', ls='--', lw=.9)
    axes[0].set_ylabel('原生采样最小间隙 / m')
    axes[0].legend(fontsize=9)
    # Summaries include maxima even when complete rows are omitted.
    ratios = []
    for stage in stages:
        s = summary(stage)
        ratios.append(max(s.get('max_'+side+'_'+kind, 0)/s[limit]
                          for side in ('left', 'right')
                          for kind, limit in (('omega', 'omega_hard_limit_with_tolerance'),
                                              ('alpha', 'alpha_hard_limit_with_tolerance'))))
    axes[1].plot(xs, ratios, marker='.', ms=4, color='#246da0', lw=1.2)
    axes[1].axhline(1, color='#b5372c', ls='--', lw=.9)
    axes[1].set_ylabel('最大轮约束值 / 验收上限')
    axes[1].set_xlabel('真正接受的优化步（0为初始曲线）')
    for ax in axes:
        ax.grid(alpha=.18)
    fig.suptitle(case['title']+'：候选可行性变化', fontsize=12)
    fig.savefig(out, dpi=170)
    plt.close(fig)


def map_geometry(folder, case):
    meta_path = folder/case['map_metadata']
    meta = load(meta_path)
    with gzip.open(meta['overlay_path'], 'rb') as stream:
        overlay = np.frombuffer(stream.read(), dtype=np.float32).reshape((-1, 4))
    with gzip.open(meta['esdf_path'], 'rb') as stream:
        values = np.frombuffer(stream.read(), dtype=np.float64)
    shape = tuple(int(n) for n in meta['map_voxel_num'])
    if len(overlay) != meta['overlay_count'] or values.size != math.prod(shape):
        raise ValueError('Native map snapshot shape mismatch')
    origin, resolution = np.asarray(meta['map_origin']), meta['resolution']
    z = min(max(int((.25-origin[2])/resolution), 0), shape[2]-1)
    mask = values.reshape(shape)[:, :, z] <= .001
    ix, iy = np.where(mask)
    static_slice = ((ix+.5)*resolution+origin[0], (iy+.5)*resolution+origin[1])
    # Projection here represents the recorded KD cloud, not a guessed person.
    # Collapse duplicate x,y pixels only for a lighter illustration.
    xy = np.unique(np.round(overlay[:, :2], 3), axis=0)
    return xy, static_slice, meta


def poly_samples(stage, dt=.025):
    """Positions from recorded native ascending-power polynomial coefficients."""
    result = []
    at = 0.
    for part in stage['parts']:
        coefficients = np.asarray(part['b_rows_ascending_power'], dtype=float)
        for index, duration in enumerate(part['times']):
            c = coefficients[index*8:(index+1)*8]
            if c.shape[0] != 8:
                raise ValueError('A native polynomial piece needs eight rows')
            for t in np.linspace(0., duration, max(2, math.ceil(duration/dt)+1)):
                p = np.polynomial.polynomial.polyval(t, c)
                v = np.polynomial.polynomial.polyval(t, (np.arange(1, 8)[:, None]*c[1:]))
                yaw = math.atan2(part['gear']*v[1], part['gear']*v[0])
                result.append(dict(t=at+t, x=p[0], y=p[1], yaw=yaw))
            at += duration
    return result


def optimization_gif(case, body, obstacle_xy, static_slice, out):
    """Each frame is an actual accepted polynomial, never simulated motion."""
    stages = case['stages']
    initial = poly_samples(stages[0])
    frames = []
    all_xy = np.asarray([[r['x'], r['y']] for s in stages for r in poly_samples(s, .08)])
    lo, hi = all_xy.min(axis=0)-.7, all_xy.max(axis=0)+.7
    for stage in stages:
        rows = poly_samples(stage)
        fig, ax = plt.subplots(figsize=(8, 4.5), constrained_layout=True)
        xs, ys = static_slice
        ax.scatter(xs, ys, s=3, c='#cccccc', alpha=.4, rasterized=True)
        if len(obstacle_xy):
            ax.scatter(obstacle_xy[:, 0], obstacle_xy[:, 1], s=5, c='#b8adc1', alpha=.5)
        ax.plot([r['x'] for r in initial], [r['y'] for r in initial],
                color='#aaaaaa', ls='--', lw=1, label='初始曲线')
        ax.plot([r['x'] for r in rows], [r['y'] for r in rows],
                color='#246da0', lw=1.7, label='本步候选')
        for index in np.linspace(0, len(rows)-1, 4, dtype=int):
            projection(ax, body, rows[index], '#246da0')
        t = stage.get('native_reject_time')
        if stage['native_IsTrajSafe_pass'] is False and finite(t):
            row = min(rows, key=lambda r: abs(r['t']-t))
            ax.scatter([row['x']], [row['y']], c='#b5372c', marker='x', s=55, label='首次拒绝位置')
        verdict = '通过' if stage['native_IsTrajSafe_pass'] else REASONS.get(stage['native_reason'], stage['native_reason'])
        ax.set_title('%s：接受步%s，原生验收%s\n优化曲线变化演示；障碍输入冻结，非机器人运动' %
                     (case['title'], stage['iteration'], verdict), fontsize=11)
        ax.set(xlim=(lo[0], hi[0]), ylim=(lo[1], hi[1]), xlabel='x / m', ylabel='y / m')
        ax.set_aspect('equal', adjustable='box')
        ax.legend(fontsize=8, loc='best')
        ax.grid(alpha=.12)
        fig.canvas.draw()
        image = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
        plt.close(fig)
    frames[0].save(out, save_all=True, append_images=frames[1:],
                   duration=[350]*(len(frames)-1)+[1200], loop=0, disposal=2, optimize=True)


def coverage(case):
    """Compare the actual cost grid with native acceptance, without re-solving."""
    final = case['stages'][-1]
    native, cost = summary(final), audit(final)['optimizer_summary']
    dense = audit(final)['feasibility_optimizer_summary']
    ratio = lambda s: max(s['max_'+side+'_'+kind]/s[limit]
        for side in ('left', 'right')
        for kind, limit in (('omega', 'omega_hard_limit_with_tolerance'),
                           ('alpha', 'alpha_hard_limit_with_tolerance')))
    earlier = [s['iteration'] for s in case['stages'][:-1] if s['native_IsTrajSafe_pass']]
    return dict(earlier_native_pass_iterations=earlier,
        native_max_wheel_ratio=ratio(native), cost_grid_max_wheel_ratio=max(ratio(cost), ratio(dense)),
        coarse_cost_grid_max_wheel_ratio=ratio(cost), dense_cost_grid_max_wheel_ratio=ratio(dense),
        native_min_base_gap_m=native['min_base_hard_gap_m'],
        cost_grid_min_base_gap_m=cost['min_base_hard_gap_m'],
        native_min_arm_gap_m=native['min_arm_hard_gap_m'],
        cost_grid_min_arm_gap_m=cost['min_arm_hard_gap_m'],
        cost_grid_note='collision summary uses coarse K grid; wheel summary combines coarse and actual dense feasibility times')


def obstacle_cost_lower_bounds(case, weight):
    """One worst sphere per quadrature point is a lower bound on all spheres.

    The rows are audited on the final frozen map, whereas recorded costs were
    produced online. A lower bound above the recorded total exposes a context
    mismatch; it does not alone identify which map changed.
    """
    k = case['params']['constrain_points_per_piece']
    checks = []
    for stage in selected(case['stages']):
        rows = samples(stage, 'optimizer_samples')
        cursor, bound = 0, 0.
        for part in stage['parts']:
            for duration in part['times']:
                for j in range(k+1):
                    gap = rows[cursor]['base_soft_gap']
                    cursor += 1
                    bound += weight*max(0., -gap)**3*duration/k*(.5 if j in (0, k) else 1.)
        if cursor != len(rows):
            raise ValueError('Recorded coarse grid and native piece quadrature mismatch')
        recorded = stage['cost_components']['base_obstacle']
        checks.append(dict(iteration=stage['iteration'], final_map_single_sphere_lower_bound=bound,
            recorded_online_all_sphere_cost=recorded,
            lower_bound_exceeds_recorded=bound > recorded+max(1e-6, abs(recorded)*1e-6)))
    return checks


def case_text(case, checks):
    stages = case['stages']
    a, b = summary(stages[0]), summary(stages[-1])
    initial, final = stages[0], stages[-1]
    lines = [case['title'],
        '请求%s；轨迹输入编号%s；最后首次拒绝：%s，轨迹相对时间%s秒。' %
        (case['request_id'], case['input_hash'], REASONS.get(final['native_reason'], final['native_reason']),
         shown(final.get('native_reject_time'))),
        '0.6秒预算内真正接受%s步；诊断在求解结束、模拟器暂停后执行，诊断时间不计在线性能。' % (len(stages)-1),
        '初始→最后：底盘最小硬间隙%s→%s米；机械臂%s→%s米。' %
        tuple(shown(v, 4) for v in (a['min_base_hard_gap_m'], b['min_base_hard_gap_m'],
                                    a['min_arm_hard_gap_m'], b['min_arm_hard_gap_m'])),
        '轮速上限（含5%%容差）%s rad/s；轮加速度上限%s rad/s²。' %
        (shown(b['omega_hard_limit_with_tolerance']), shown(b['alpha_hard_limit_with_tolerance'])),
        '最大轮约束值/验收上限：初始%s，最后%s；大于1表示超限。' %
        (shown(max(a['max_'+side+'_'+kind]/a[limit]
             for side in ('left', 'right') for kind, limit in (('omega', 'omega_hard_limit_with_tolerance'),
                                                               ('alpha', 'alpha_hard_limit_with_tolerance')))),
         shown(checks['native_max_wheel_ratio'])),
        '最后候选在优化采样点的最大轮约束比例%s，在原生密检查点为%s。' %
        (shown(checks['cost_grid_max_wheel_ratio']), shown(checks['native_max_wheel_ratio']))]
    if checks['earlier_native_pass_iterations']:
        lines.append('把中间候选放在最终冻结环境重建检查，接受步%s通过；最后候选未通过。' %
                     checks['earlier_native_pass_iterations'])
        lines.append('这不等于它形成时已在线合格；若求解期间环境变化，不能直接说“在线合格候选被丢了”。')
    else:
        lines.append('相对最终冻结环境，逐步检查没有找到通过原生验收的中间候选。')
    if checks['cost_grid_max_wheel_ratio'] > 1:
        lines.append('轮约束违规在优化实际采样处已经存在，至少这一部分不能归因于密检查发现了采样间漏点。')
    if final['native_reason'] == 'planning_collision':
        coarse = audit(final)['optimizer_summary']
        lines.append('最后候选：原生密检查的底盘最小间隙%s米；碰撞代价粗采样处为%s米。' %
                     (shown(b['min_base_hard_gap_m'], 4), shown(coarse['min_base_hard_gap_m'], 4)))
        if coarse['min_base_hard_gap_m'] < 0:
            lines.append('底盘违规在碰撞代价实际采样处已经存在，不能仅用“检查采样不一样”来解释。')
        elif b['min_base_hard_gap_m'] < 0:
            lines.append('底盘硬违规出现在碰撞代价粗采样之间；粗采样处仍可能有软惩罚，不能把漏点认定为唯一原因。')
    if all(s.get('cost_components_recorded') for s in (initial, final)):
        ca, cb = initial['cost_components'], final['cost_components']
        lines.append('代价变化：总值%s→%s；底盘避障%s→%s；底盘运动约束%s→%s。' %
                     tuple(shown(v, 2) for v in (initial['cost'], final['cost'],
                           ca['base_obstacle'], cb['base_obstacle'],
                           ca['base_feasibility'], cb['base_feasibility'])))
    if any(r['lower_bound_exceeds_recorded'] for r in checks['obstacle_cost_consistency']):
        lines.append('发现代价与事后地图不一致的线索：按最终冻结地图、每个粗采样点只算最差一个球，得出的避障代价下界已超过在线记录的全部球代价。')
        for row in checks['obstacle_cost_consistency']:
            if row['lower_bound_exceeds_recorded']:
                lines.append('  接受步%s：冻结地图单球下界%s，在线记录全部球代价%s。' %
                             (row['iteration'], shown(row['final_map_single_sphere_lower_bound'], 2),
                              shown(row['recorded_online_all_sphere_cost'], 2)))
        lines.append('这提示求解时与事后验收的环境或计算上下文不同；需核查动态KD更新，不能直接归因于避障权重太小。')
    history = case.get('dynamic_map_history')
    if history:
        lines.append('求解期间：%s次完整评估中，%s次评估前后动态查询快照世代改变；整个求解首尾快照相同=%s。' %
                     (history['evaluate_complete_count'], history['evaluate_overlay_changed_count'],
                      history['solver_entry_exit_key_equal']))
        lines.append('求解期间固定动态查询快照=%s；事后逐候选诊断使用同一个固定快照。' %
                     history['solver_freeze_requested'])
    if initial['native_IsTrajSafe_pass'] is False:
        lines.append('初始曲线相对最终冻结环境不合格；若求解期间地图变过，不能据此认定A*生成时已经碰撞。')
    lines += ['代码核实：避障是软代价；底盘软惩罚阈值为球半径+%s米，硬验收为球半径+%s米。' %
              (shown(case['params']['base_safe_margin']), shown(case['params']['map_resolution'])),
              '软轮约束也比含5%容差的硬验收更早惩罚；“有软惩罚”不保证有限时间内违规全部消失。',
              '注意：原生验收从0.05秒开始；起始时刻单独画出，不能把0秒的现象当成原生拒绝原因。',
              '路径图中的灰紫色点是同一冻结的预测点集合；GIF展示优化曲线变化，不表示机器人在运动。', '']
    return lines


def report(batch, out, make_gifs=True):
    batch, out = Path(batch).resolve(), Path(out).resolve()
    complete = load(batch/'complete.json')
    plan = load(batch/'plan.json')
    if complete.get('complete') is not True:
        raise ValueError('Finish the predeclared capture batch before rendering')
    out.mkdir(parents=True, exist_ok=True)
    font = FontProperties(fname=FONT)
    plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False,
                         'font.size': 10, 'savefig.facecolor': 'white'})
    entries = []
    text = ['候选轨迹为什么仍不合格：两次定向诊断',
        '场景为6米目标、中段横穿、67球、seed11。两次分别捕获首次碰撞失败和首次轮约束失败。',
        '求解结束后冻结环境，再重建检查初始曲线、每个真正接受的中间候选和最后候选；使用原生验收规则。',
        '中间候选相对最终冻结地图的结果，不等于它形成当时的在线验收结果；需另查求解期间地图是否更新。',
        '这是后端失败原因诊断，不是新一轮避障效果或速度对比。', '']
    for job in plan['trials']:
        run = batch/job['directory']
        status = load(run/'run_status.json')
        path = run/'candidate_audit/case.json'
        if not path.exists():
            text += ['目标%s：本次未捕获，保留这次运行；没有换输入补样本。' % job['target'], '']
            entries.append(dict(target=job['target'], captured=False, status=status))
            continue
        case = load(path)
        case['title'] = '碰撞失败候选' if job['target'] == 'collision' else '轮约束失败候选'
        if case['target'] != job['target'] or case['diagnostic_candidates_published'] is not False:
            raise ValueError('Capture target/publication contract mismatch')
        if not all(case['frozen_checks'].values()):
            raise ValueError('Frozen input checks failed; cannot attribute cause across stages')
        checks = coverage(case)
        body_record = load(run/'native_collision_3d_body.json')
        body = np.asarray(body_record['body'], dtype=float)
        if body.shape[0] != 67:
            raise ValueError('Expected actual 67-sphere model')
        # Export columns store native hard radius. Draw the physical proxy
        # spheres: only the base hard radius includes one map-resolution margin.
        body[body[:, 6] == 0, 3] -= float(body_record['header'][11])
        parameter_path = run/'parameters.yaml'
        parameter_key = '/remani_planner_node/optimization/weight_obstacle'
        weight = float(yaml.safe_load(parameter_path.read_text())[parameter_key])
        base_weights = np.unique(body[body[:, 6] == 0, 5])
        if len(base_weights) != 1 or float(base_weights[0]) != weight:
            raise ValueError('Exported model and actual native obstacle weight disagree')
        checks['obstacle_weight'] = dict(value=weight, parameter_key=parameter_key,
                                        source=str(parameter_path), source_sha256=file_sha(parameter_path))
        checks['obstacle_cost_consistency'] = obstacle_cost_lower_bounds(case, weight)
        xy, static_slice, meta = map_geometry(path.parent, case)
        case_out = out/job['target']
        case_out.mkdir(exist_ok=True)
        plot_paths(case, body, case_out/'初始与最终轨迹.png', xy, static_slice)
        plot_wheels(case, case_out/'左右轮约束.png')
        plot_trend(case, case_out/'接受步骤与违规变化.png')
        if make_gifs:
            optimization_gif(case, body, xy, static_slice, case_out/'候选优化过程.gif')
        lines = case_text(case, checks)
        text += lines
        (case_out/'这条候选的问题.txt').write_text('\n'.join(lines)+'\n')
        entries.append(dict(target=job['target'], captured=True,
            request_id=case['request_id'], initial_native_pass=case['stages'][0]['native_IsTrajSafe_pass'],
            final_native_pass=case['stages'][-1]['native_IsTrajSafe_pass'],
            native_reason=case['stages'][-1]['native_reason'], coverage=checks,
            frozen_checks=case['frozen_checks'], source_sha256={str(path):file_sha(path)},
            map_metadata=meta))
    result = dict(schema='candidate_audit_report_v1', source=str(batch), cases=entries,
                  navigation_metrics_valid=False, diagnostic_candidates_published=False)
    (out/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    (out/'实验结论.txt').write_text('\n'.join(text)+'\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path,
        default=Path(__file__).resolve().parent/'results/candidate_audit_20261030_01')
    parser.add_argument('--out', type=Path,
        default=Path(__file__).resolve().parent/'results/candidate_audit_report_20261030_01')
    parser.add_argument('--no-gifs', action='store_true')
    args = parser.parse_args()
    report(args.batch, args.out, not args.no_gifs)


if __name__ == '__main__':
    main()

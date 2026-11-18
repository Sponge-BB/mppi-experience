#!/usr/bin/env python3
"""Offline report of recorded native / MPPI / frozen-reference arm-box trials.

Reads recorded states only. No optimizer, ROS client or motion command is run.
The 67-sphere geometry is reconstructed from each trial's frozen parameters;
analytic AABB clearance is an independent sampled-state audit, not a mesh or
continuous collision certificate.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from active_arm_model import PiperArmModel


LABELS = {'native': 'REMANI', 'mppi': 'REMANI + MPPI',
          'fast_only': 'MPPI · 初始参考冻结'}
COLORS = {'native': '#356d9b', 'mppi': '#dc833b', 'fast_only': '#477e64'}
REQUIRED = ['t', 'x', 'y', 'yaw', 'v', 'w'] + ['q%d' % n for n in range(1, 7)]


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path, default=None):
    return json.loads(path.read_text()) if path.is_file() else default


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    return value


def sphere_box_gaps(spheres, center, size):
    """Signed distance to a solid axis-aligned box minus physical sphere r."""
    delta = np.abs(np.asarray(spheres)[..., :3] - center) - np.asarray(size) / 2.
    signed = (np.linalg.norm(np.maximum(delta, 0.), axis=-1)
              + np.minimum(np.max(delta, axis=-1), 0.))
    return signed - np.asarray(spheres)[..., 3]


def at(times, time):
    return max(0, min(len(times) - 1, int(np.searchsorted(times, time, side='right') - 1)))


def load_trial(path):
    path = Path(path)
    result = read_json(path / 'result.json')
    config = read_json(path / 'active_arm_config.json')
    scene = read_json(path / 'scenario.json')
    if not all(isinstance(item, dict) for item in (result, config, scene)):
        raise ValueError('Missing result/config/scenario: ' + str(path))
    if scene.get('schema') != 'arm_box_scene_v1':
        raise ValueError('Unknown scene schema: ' + str(path))
    with (path / 'state.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    if not rows or not set(REQUIRED).issubset(rows[0]):
        raise ValueError('Missing measured base/joint states: ' + str(path))
    state = {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}
    if not all(np.isfinite(state[key]).all() for key in REQUIRED):
        raise ValueError('Nonfinite measured states: ' + str(path))
    if np.any(np.diff(state['t']) < 0):
        raise ValueError('Backwards state time: ' + str(path))
    model = PiperArmModel.from_parameters(path / 'parameters.yaml',
                         **({'urdf_path': config['urdf_path']} if config.get('urdf_path') else {}))
    q = np.column_stack([state['q%d' % n] for n in range(1, 7)])
    base = np.column_stack([state[key] for key in ('x', 'y', 'yaw')])
    body = model.body_spheres(q, base)
    if body.shape[1:] != (67, 4):
        raise ValueError('Expected frozen 47 + 20 sphere model')
    runtime = read_json(path / 'box_runtime.json', {})
    origin = result.get('motion_anchor')
    if origin is None:
        origin = result.get('goal_sent')
    if origin is None:
        origin = float(state['t'][0])
    appearance = runtime.get('appearance_time')
    if appearance is None and result.get('motion_anchor') is not None and runtime:
        appearance = float(result['motion_anchor']) + float(scene['appearance_s'])
    box = dict(scene['box'])
    box.update(runtime.get('box', {}))
    center = runtime.get('center', runtime.get('box_center', box.get('center')))
    if center is None and appearance is not None:
        raise ValueError('An appeared box needs its recorded world center')
    center = np.asarray(center if center is not None else [0., 0., 0.], dtype=float)
    size = np.asarray(box['size'], dtype=float)
    if center.shape != (3,) or size.shape != (3,) or not np.isfinite(np.r_[center, size]).all() or np.any(size <= 0):
        raise ValueError('Invalid recorded box geometry')
    visible = np.zeros(len(q), dtype=bool) if appearance is None else state['t'] >= float(appearance)
    with (path/'obstacles.csv').open() as stream:
        obstacle_rows=list(csv.DictReader(stream))
    centers=np.tile(center,(len(q),1))
    if obstacle_rows:
        ot=np.array([float(row['t']) for row in obstacle_rows])
        oc=np.array([[float(row[k]) for k in ('cx','cy','cz')] for row in obstacle_rows])
        ids=np.maximum(0,np.searchsorted(ot,state['t'],side='right')-1)
        centers=oc[ids]
        if np.max(np.abs(ot[ids[visible]]-state['t'][visible]))>1e-7:
            raise ValueError('Obstacle states are not synchronized with robot samples')
    gaps = sphere_box_gaps(body, centers[:,None,:], size)
    arm_gap, base_gap = gaps[:, 47:].min(axis=1), gaps[:, :47].min(axis=1)
    arm_gap[~visible] = np.nan
    base_gap[~visible] = np.nan
    trajectories = read_json(path / 'trajectories.json', [])
    refs, seen = [], set()
    for row in sorted(trajectories, key=lambda item: item.get('t', 0.)):
        if row.get('action') != 1 or row.get('id') != 1:
            continue
        # A native bundle's first block, deduplicated by its canonical bytes.
        key = row.get('canonical_message_sha256', row.get('message_sha256'))
        if key is None:
            key = json.dumps({k: row.get(k) for k in ('stamp', 'polynomials', 'points')}, sort_keys=True)
        if key in seen:
            continue
        seen.add(key)
        refs.append(float(row['t']))
    reference_count = np.searchsorted(refs, state['t'], side='right')
    cmd_names = ['cmd_dq%d' % n for n in range(1, 7)]
    cmd_norm = np.linalg.norm(np.column_stack([state[k] for k in cmd_names]), axis=1) if all(k in state for k in cmd_names) else None
    cycles_path = path / 'active_arm_cycles.jsonl'
    cycles = []
    if cycles_path.is_file():
        with cycles_path.open() as stream:
            cycles = [json.loads(line) for line in stream if line.strip()]
    approved_arm = sum(row.get('reason') == 'ok' and isinstance(row.get('command'), list)
                       and len(row['command']) == 8 and np.linalg.norm(row['command'][2:]) > 1e-5 for row in cycles)
    path_length = float(np.linalg.norm(np.diff(base[:, :2], axis=0), axis=1).sum())
    after_refs = sum(t >= float(appearance) for t in refs) if appearance is not None else 0
    finite_gap = lambda gap: float(np.nanmin(gap)) if np.isfinite(gap).any() else None
    metrics = dict(group=path.name, label=LABELS.get(path.name, path.name),
        reason=str(result.get('reason', 'unknown')), goal_reached=bool(result.get('goal_reached', result.get('reason') == 'goal')),
        collision_physical=result.get('collision_physical'), recorded_samples=len(q),
        motion_anchor=result.get('motion_anchor'), plot_origin_s=float(origin),
        duration_from_origin_s=float(state['t'][-1] - origin), box_appeared=appearance is not None,
        box_appearance_s=None if appearance is None else float(appearance - origin),
        box_center=center.tolist(), box_size=size.tolist(), recorded_front_gap_m=runtime.get('front_gap_m'),
        actual_base_path_length_m=path_length, maximum_lateral_offset_m=float(np.max(np.abs(base[:, 1] - base[0, 1]))),
        maximum_heading_change_rad=float(np.max(np.abs(np.unwrap(base[:, 2]) - base[0, 2]))),
        maximum_joint_change_rad=float(np.max(np.abs(q - q[0]))),
        arm_joint_path_l2_rad=float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()),
        minimum_arm_box_sampled_gap_m=finite_gap(arm_gap), minimum_base_box_sampled_gap_m=finite_gap(base_gap),
        unique_native_reference_bundles=len(refs), unique_native_reference_bundles_after_box=after_refs,
        reference_count_scope='received action=1,id=1 bundles deduplicated by canonical message; not optimizer invocations',
        nonzero_arm_command_state_samples=None if cmd_norm is None else int(np.count_nonzero(cmd_norm > 1e-5)),
        approved_mppi_nonzero_arm_cycles=approved_arm if cycles_path.is_file() else None,
        native_planner_freeze_requested=config.get('slow_planner_frozen_after_initial_reference', config.get('freeze_slow_replanning', config.get('freeze_reference'))),
        sampling_scope='all saved states; analytic physical sphere versus solid AABB; no wall/self/continuous proof')
    names = ['state.csv', 'result.json', 'scenario.json', 'active_arm_config.json', 'parameters.yaml',
             'box_runtime.json', 'obstacles.csv', 'trajectories.json', 'events.json', 'active_arm_cycles.jsonl']
    metrics['source_sha256'] = {name: sha(path / name) for name in names if (path / name).is_file()}
    return dict(path=path, result=result, config=config, scene=scene, runtime=runtime, state=state,
                q=q, base=base, body=body, origin=float(origin), t=state['t'] - origin,
                appearance=None if appearance is None else float(appearance - origin),
                center=center, centers=centers, size=size, arm_gap=arm_gap, base_gap=base_gap,
                refs=np.asarray(refs) - origin, reference_count=reference_count,
                cmd_norm=cmd_norm, metrics=metrics)


def result_label(trial):
    meta = trial['metrics']
    if meta['collision_physical']:
        return '记录报告碰撞 · ' + meta['reason']
    return ('到达目标' if meta['goal_reached'] else '未到目标 · ' + meta['reason'])


def plot_comparison(trials, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 3, figsize=(14., 8.2), constrained_layout=True)
    for trial in trials:
        name = trial['path'].name
        color, label, t = COLORS.get(name, '#666666'), LABELS.get(name, name), trial['t']
        axes[0, 0].plot(trial['base'][:, 0], trial['base'][:, 1], color=color, label=label)
        axes[0, 0].plot(*trial['base'][-1, :2], 'o', color=color, ms=4)
        axes[0, 1].plot(t, trial['state']['v'], color=color, label=label)
        axes[0, 2].plot(t, np.rad2deg(np.max(np.abs(trial['q'] - trial['q'][0]), axis=1)), color=color, label=label)
        axes[1, 0].plot(t, trial['arm_gap'], color=color, label=label + ' 臂')
        axes[1, 0].plot(t, trial['base_gap'], '--', color=color, label=label + ' 底盘')
        axes[1, 1].step(t, trial['reference_count'], where='post', color=color, label=label)
        if trial['cmd_norm'] is not None:
            axes[1, 2].plot(t, trial['cmd_norm'], color=color, label=label)
        if trial['appearance'] is not None:
            for ax in axes.flat[1:]:
                ax.axvline(trial['appearance'], color=color, ls=':', alpha=.45, lw=.8)
    titles = [('实际底盘轨迹', 'x / m', 'y / m'), ('实际底盘速度', '运动对齐时间 / s', 'v / (m/s)'),
              ('相对初始姿态的最大关节变化', '运动对齐时间 / s', '角度 / °'),
              ('障碍出现后的物理球—长方体间隙', '运动对齐时间 / s', '间隙 / m'),
              ('收到的唯一原生参考束累计数', '运动对齐时间 / s', '束数（非优化次数）'),
              ('下发机械臂速度向量范数', '运动对齐时间 / s', '||dq_cmd|| / (rad/s)')]
    for ax, (title, xlabel, ylabel) in zip(axes.flat, titles):
        ax.set(title=title, xlabel=xlabel, ylabel=ylabel)
        ax.grid(alpha=.18)
        ax.legend(fontsize=8)
    axes[0, 0].set_aspect('equal', adjustable='datalim')
    axes[1, 0].axhline(0., color='#b34848', lw=1.)
    fig.suptitle('迎面移动障碍：三组数值闭环对照\n'
                 '点线：各轮实测障碍出现时刻；各组保留实际终止点，未预设优劣', fontsize=13)
    fig.savefig(out / 'comparison.png', dpi=135)
    plt.close(fig)


def make_gif(trials, out, max_frames):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from PIL import Image
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, len(trials), figsize=(4.4*len(trials), 6.4), dpi=100, squeeze=False)
    fig.subplots_adjust(left=.055, right=.99, top=.87, bottom=.09, wspace=.19, hspace=.36)
    xmin = min(float(t['body'][..., 0].min()) - .13 for t in trials)
    xmax = max(max(float(t['body'][..., 0].max())+.13, float(t['scene']['goal'][0])+.2, float(t['center'][0]+t['size'][0]/2)+.15) for t in trials)
    ymin = min(-.6, *(float(t['body'][..., 1].min()) - .15 for t in trials))
    ymax = max(.6, *(float(t['body'][..., 1].max()) + .15 for t in trials))
    zmax = max(1.5, *(max(float(t['body'][..., 2].max())+.15,float(t['center'][2]+t['size'][2]/2)+.1) for t in trials))
    groups = []
    for col, trial in enumerate(trials):
        side, top = axes[:, col]
        side.set(xlim=(xmin, xmax), ylim=(0., zmax), xlabel='x / m', ylabel='z / m')
        top.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel='x / m', ylabel='y / m')
        for ax in (side, top):
            ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.14); ax.tick_params(labelsize=8)
        side.set_title(trial['metrics']['label'], fontsize=10)
        balls = []
        for sphere in range(67):
            pair = []
            for ax in (side, top):
                patch = Circle((0., 0.), .01, color='#e58b36' if sphere >= 47 else '#438ac0', alpha=.72 if sphere >= 47 else .42, lw=.3)
                ax.add_patch(patch); pair.append(patch)
            balls.append(pair)
        c, s = trial['center'], trial['size']
        rects = [Rectangle((c[0] - s[0]/2., c[2] - s[2]/2.), s[0], s[2], color='#d95d62', alpha=.65),
                 Rectangle((c[0] - s[0]/2., c[1] - s[1]/2.), s[0], s[1], color='#d95d62', alpha=.35)]
        side.add_patch(rects[0]); top.add_patch(rects[1])
        trail, = top.plot([], [], color=COLORS[trial['path'].name], lw=1.8)
        goal = trial['scene']['goal']; top.plot(*goal, '+', color='#426a55', ms=9)
        note = fig.text((col+.08)/len(trials), .035, '', fontsize=9, va='bottom')
        groups.append((balls, rects, trail, note))
    end = max(max(0., float(t['t'][-1])) for t in trials)
    times = np.linspace(0., end, min(max_frames, max(2, int(end/.18) + 1)))
    frames = []
    for elapsed in times:
        fig.suptitle('迎面障碍 · 仿真记录回放 %.2f s · 上侧视 / 下俯视 · 1倍速\n'
                     '蓝：固定结构；橙：机械臂；红：迎面移动长方体；曲线：实际底盘轨迹' % elapsed, fontsize=11)
        for trial, (balls, rects, trail, note) in zip(trials, groups):
            k = at(trial['t'], elapsed)
            for sphere, pair in zip(trial['body'][k], balls):
                pair[0].center = sphere[[0, 2]]; pair[1].center = sphere[[0, 1]]
                pair[0].radius = pair[1].radius = sphere[3]
            visible = trial['appearance'] is not None and trial['t'][k] >= trial['appearance']
            center, size = trial['centers'][k], trial['size']
            rects[0].set_xy((center[0]-size[0]/2,center[2]-size[2]/2))
            rects[1].set_xy((center[0]-size[0]/2,center[1]-size[1]/2))
            for rect in rects:
                rect.set_visible(visible)
            trail.set_data(trial['base'][:k+1, 0], trial['base'][:k+1, 1])
            ended = elapsed >= trial['t'][-1] - 1e-8
            note.set_text('v=%.2f m/s；参考束=%d\n%s' %
                (trial['state']['v'][k], trial['reference_count'][k], result_label(trial) if ended else '实测 t=%.2f s' % trial['t'][k]))
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()).convert('P', palette=Image.ADAPTIVE, colors=96))
    plt.close(fig)
    duration = max(20, int(round(1000. * end / max(1, len(times)-1) / 1.)))
    frames[0].save(out / 'comparison.gif', save_all=True, append_images=frames[1:], duration=duration,
                   loop=0, optimize=False, disposal=2)
    for frame in frames:
        frame.close()
    return dict(frames=len(times), width=int(440*len(trials)), playback_speed=1, ended_panels_frozen=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--max-frames', type=int, default=180)
    args = parser.parse_args()
    if not 2 <= args.max_frames <= 240:
        parser.error('--max-frames must be between 2 and 240')
    case = args.case.resolve()
    trials = [load_trial(case / name) for name in LABELS if (case / name / 'state.csv').is_file()]
    if not trials:
        raise ValueError('No completed state.csv trials under ' + str(case))
    out = case / 'report'; out.mkdir(exist_ok=True)
    plot_comparison(trials, out)
    animation = make_gif(trials, out, args.max_frames)
    for trial in trials:
        single=out/trial['path'].name;single.mkdir(exist_ok=True)
        make_gif([trial],single,args.max_frames)
    summary = dict(schema='headon_threeway_report_v1', trials=[t['metrics'] for t in trials], gif=animation,
        missing_groups=[name for name in LABELS if not (case / name / 'state.csv').is_file()],
        scope='Recorded ROS numerical closed-loop states; no invented trajectories or candidate traces',
        reporter_sha256=sha(Path(__file__)), hardware_validation=False)
    (out / 'summary.json').write_text(json.dumps(safe(summary), ensure_ascii=False, indent=2) + '\n')
    # Read-only report generation must never mutate any trial source.
    for trial in trials:
        for name, expected in trial['metrics']['source_sha256'].items():
            if sha(trial['path'] / name) != expected:
                raise RuntimeError('Trial source changed during report: ' + str(trial['path'] / name))
    print(out)


if __name__ == '__main__':
    main()

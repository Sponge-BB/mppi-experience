#!/usr/bin/env python3
"""Render actual free-base standby records; never synthesize control motion."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
from PIL import Image

from active_arm_model import PiperArmModel
from standby_arm_scene import actors_3d_at, cylinder_gaps, scene_definition, validate_scene


HERE = Path(__file__).resolve().parent
DEFAULT = HERE/'results/standby_arm_influence_20261105_01'
GROUPS = ('native_piper', 'current_unmodified', 'current_standby_extension')
LABELS = dict(native_piper='原生 REMANI Piper 适配基线',
              current_unmodified='当前项目 · 原样待机',
              current_standby_extension='当前项目 · 新增待机保护扩展')
NATIVE_COMMIT = 'f138ec82dce5c58be2e7028548ed317efe541c4f'
COLORS = ('#777777', '#3d72a4', '#d67a2c', '#4c9666')
plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False,
                     'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [safe(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (np.bool_, np.integer)):
        return value.item()
    return value


def trial_status(path):
    """Admission is based on process completion, never on having a CSV alone."""
    path = Path(path); problems = []
    status_path = path/'run_status.json'
    status = None
    if (path/'interruption_status.json').exists():
        problems.append('interruption_status.json exists')
    if not status_path.is_file():
        problems.append('missing run_status.json')
    else:
        try:
            status = json.loads(status_path.read_text())
            for name, expected in (('result_written', True), ('required_process_failed', False),
                                   ('graph_audit_written', True)):
                if status.get(name) is not expected:
                    problems.append('run_status.%s is not %s' % (name, expected))
        except (ValueError, OSError, AttributeError) as exc:
            problems.append('invalid run_status.json: '+str(exc))
    for name in ('config.json', 'result.json', 'state.csv', 'graph_audit.json'):
        if not (path/name).is_file():
            problems.append('missing '+name)
    return dict(trial=str(path), complete=not problems, reasons=problems, run_status=status)


def load_trial(path):
    path = Path(path)
    status = trial_status(path)
    if not status['complete']:
        raise ValueError('Incomplete/interrupted trial: '+str(status))
    config = json.loads((path/'config.json').read_text())
    result = json.loads((path/'result.json').read_text())
    with (path/'state.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    required = ['t', 'x', 'y', 'yaw', 'v', 'w']
    required += ['q%d' % j for j in range(1, 7)] + ['dq%d' % j for j in range(1, 7)]
    if not rows or not all(key in rows[0] for key in required):
        raise ValueError('Missing recorded state columns: '+str(path))
    state = {key: np.asarray([float(row[key]) for row in rows]) for key in rows[0]}
    if (not all(np.isfinite(state[key]).all() for key in required)
            or np.any(np.diff(state['t']) < 0)):
        raise ValueError('Invalid measured state or reversed time')
    scene = config.get('scene_definition', config.get('scene'))
    if not isinstance(scene, dict):
        scenario = path/'scenario.json'
        if not scenario.exists():
            raise ValueError('Missing frozen standby scene')
        scene = json.loads(scenario.read_text())
    validate_scene(scene)
    origin = result.get('experiment_start', config.get('experiment_start', 0.))
    times = state['t'] - float(origin)
    base = np.column_stack([state[k] for k in ('x', 'y', 'yaw')])
    q = np.column_stack([state['q%d' % j] for j in range(1, 7)])
    dq = np.column_stack([state['dq%d' % j] for j in range(1, 7)])
    parameters = config.get('parameters_path', config.get('parameters'))
    if parameters:
        parameters = Path(parameters)
        if not parameters.is_absolute():
            parameters = path/parameters
    else:
        parameters = path/'parameters.yaml'
    fallback = not parameters.is_file()
    if fallback:
        parameters = HERE/'results/active_arm_20261104_01/v2_B_active_seed23/parameters.yaml'
    model = PiperArmModel.from_parameters(parameters)
    bodies = model.body_spheres(q, base)
    gaps = np.asarray([float(np.min(cylinder_gaps(body, actors_3d_at(float(t), scene))))
                       for t, body in zip(times, bodies)])
    group = config.get('mode', config.get('group', config.get('variant', config.get('tag', path.name))))
    if group not in GROUPS:
        group = next((g for g in GROUPS if str(group).startswith(g)), str(group))
    seed = config.get('seed')
    label = LABELS.get(group, group) + (' · seed '+str(seed) if seed is not None else '')
    yaw = np.unwrap(base[:, 2]); yaw_delta = yaw-yaw[0]
    displacement = np.linalg.norm(base[:, :2]-base[0, :2], axis=1)
    path_length = np.cumsum(np.r_[0., np.linalg.norm(np.diff(base[:, :2], axis=0), axis=1)])
    collisions = np.flatnonzero(gaps < 0.)
    collision = bool(result.get('collision_physical', result.get('collision', False)) or len(collisions))
    violation = result.get('first_violation')
    violation = violation if isinstance(violation, dict) else {}
    audited_collision_time = violation.get('t') if collision else None
    audited_collision_source = 'result.json:first_violation.t' if audited_collision_time is not None else None
    if audited_collision_time is None:
        audited_collision_time = result.get('collision_time_s', result.get('first_collision_s'))
        if audited_collision_time is not None:
            audited_collision_source = 'result.json:collision_time_s/first_collision_s'
    # The independent integrator can detect contact after the final CSV tick.
    # Its saved state is an actual snapshot, and is used only for the picture;
    # CSV-derived motion statistics remain unchanged.
    terminal_snapshot = None
    audited_state = np.asarray(violation.get('state', []), dtype=float)
    if (audited_collision_time is not None and audited_state.shape == (17,)
            and np.isfinite(audited_state).all()):
        terminal_snapshot = dict(t=float(audited_collision_time), base=audited_state[:3],
            q=audited_state[5:11], body=model.body_spheres(audited_state[5:11], audited_state[:3]),
            external_gap=violation.get('external_gap_m'))
    manifest_path = path/'manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {}
    native_commit = manifest.get('native_baseline_commit')
    if group == 'native_piper' and native_commit != NATIVE_COMMIT:
        raise ValueError('Native Piper baseline commit is not the declared f138ec82 baseline')
    cycles = []
    if (path/'cycles.jsonl').exists():
        for line in (path/'cycles.jsonl').read_text().splitlines():
            if line.strip():
                cycles.append(json.loads(line))
    latency = []
    for row in cycles:
        for name in ('solve_ms', 'callback_ms', 'cycle_ms', 'compute_ms', 'elapsed_ms'):
            if name in row:
                value = float(row[name])
                if np.isfinite(value):
                    latency.append(value)
                break
    reason = result.get('reason', result.get('termination', 'not_recorded'))
    audit_gap = result.get('min_external_gap_m', result.get('minimum_clearance_m'))
    metadata = dict(trial=str(path), group=group, seed=seed, label=label,
        termination=reason, collision_physical=collision,
        first_recorded_collision_s=float(times[collisions[0]]) if len(collisions) else None,
        first_audited_collision_s=audited_collision_time,
        collision_time_source=audited_collision_source,
        first_collision_time_s=audited_collision_time if audited_collision_time is not None else
            float(times[collisions[0]]) if len(collisions) else None,
        first_recorded_time_s=float(times[0]), last_recorded_time_s=float(times[-1]),
        actual_max_base_displacement_m=float(np.max(displacement)),
        actual_base_path_length_m=float(path_length[-1]),
        actual_max_base_angle_deg=float(np.rad2deg(np.max(np.abs(yaw_delta)))),
        actual_cumulative_base_angle_deg=float(np.rad2deg(np.sum(np.abs(np.diff(yaw))))),
        actual_final_base_position_error_m=float(displacement[-1]),
        actual_max_base_displacement_mm=float(np.max(displacement)*1000.),
        actual_base_path_length_mm=float(path_length[-1]*1000.),
        actual_final_base_position_error_mm=float(displacement[-1]*1000.),
        actual_final_base_yaw_error_deg=float(np.rad2deg(abs(yaw_delta[-1]))),
        maximum_measured_base_speed_m_s=float(np.max(np.abs(state['v']))),
        maximum_measured_base_yaw_rate_rad_s=float(np.max(np.abs(state['w']))),
        maximum_joint_displacement_deg=float(np.rad2deg(np.max(np.abs(q-q[0])))),
        final_joint_position_error_deg=float(np.rad2deg(np.max(np.abs(q[-1]-q[0])))),
        measured_min_external_gap_m=float(np.min(gaps)), independently_audited_min_external_gap_m=audit_gap,
        joint_limit_violation=result.get('joint_limit_violation'),
        cycle_count=len(cycles), callback_p95_ms=float(np.percentile(latency, 95)) if latency else None,
        callback_max_ms=float(np.max(latency)) if latency else None,
        avoidance_completed=bool(reason == 'completed' and not collision and
                                 not result.get('joint_limit_violation', False)),
        accepted_control_cycles=sum(row.get('reason', row.get('status')) == 'ok' for row in cycles),
        exponential_mean_cycles=sum(row.get('info', {}).get('selection') == 'exponential_mean'
                                    for row in cycles),
        model_parameters=str(parameters), model_parameters_sha256=sha(parameters),
        model_parameters_fallback=fallback,
        completion=status, native_baseline_commit=native_commit,
        terminal_display_snapshot_source='result.json:first_violation.state' if terminal_snapshot else None,
        statistics_source='complete recorded state.csv; base origin is first measured base pose',
        source_sha256={name: sha(path/name) for name in
            ('config.json', 'result.json', 'state.csv', 'obstacles.csv', 'cycles.jsonl',
             'run_status.json', 'graph_audit.json', 'manifest.json') if (path/name).is_file()})
    return dict(path=path, config=config, result=result, scene=scene, model=model,
                state=state, t=times, q=q, dq=dq, base=base, bodies=bodies, gaps=gaps,
                displacement=displacement, path_length=path_length, yaw_delta=yaw_delta,
                metadata=metadata, cycles=cycles, terminal_snapshot=terminal_snapshot)


def data_plot(trials, path):
    fig, axes = plt.subplots(2, 3, figsize=(16.5, 9.), constrained_layout=True)
    for i, trial in enumerate(trials):
        c = COLORS[i % len(COLORS)]; m = trial['metadata']; label = m['label']; t = trial['t']
        axes[0, 0].plot(t, trial['displacement']*1000., color=c, label=label)
        axes[0, 1].plot(t, trial['path_length']*1000., color=c, label=label)
        axes[0, 2].plot(t, np.rad2deg(trial['yaw_delta']), color=c, label=label)
        axes[1, 0].plot(t, np.rad2deg(trial['q'][:, 1]), color=c, label=label+' q2')
        axes[1, 0].plot(t, np.rad2deg(trial['q'][:, 4]), '--', color=c, label=label+' q5')
        gap = trial['gaps'].copy(); gap[~np.isfinite(gap)] = np.nan
        axes[1, 1].plot(t, gap*100., color=c, label=label)
        if m['collision_physical']:
            at = m['first_collision_time_s']
            if at is not None:
                terminal = trial['terminal_snapshot']
                contact_gap = terminal.get('external_gap') if terminal else None
                shown_gap = float(contact_gap)*100. if contact_gap is not None else np.interp(at, t, gap*100.)
                axes[1, 1].plot(at, shown_gap, 'x', color='#c0392b', markersize=9)
        axes[1, 2].text(.01, .98-i*.95/max(1, len(trials)),
            '%s\n%s；最大位移 %.3f mm，路程 %.3f mm\n最大转角 %.3f°；返回误差 %.3f mm' %
            (label, '碰撞' if m['collision_physical'] else '未检出碰撞',
             m['actual_max_base_displacement_mm'], m['actual_base_path_length_mm'],
             m['actual_max_base_angle_deg'], m['actual_final_base_position_error_mm']),
            transform=axes[1, 2].transAxes, va='top', color=c, fontsize=9)
    axes[0, 0].set(title='待机位置受影响多大：实际底盘位移', ylabel='离起始位置 / mm')
    axes[0, 1].set(title='实际底盘累计路程', ylabel='mm')
    axes[0, 2].set(title='实际底盘转角', ylabel='相对初始航向 / °')
    axes[1, 0].set(title='机械臂实际关节变化', ylabel='关节角 / °')
    axes[1, 1].set(title='球模型实际外部间隙', ylabel='cm')
    axes[1, 1].axhline(0., color='#c0392b', linewidth=1.)
    axes[1, 2].axis('off')
    for ax in axes.flat:
        if ax.axison:
            ax.set_xlabel('独立实验时钟 / s'); ax.grid(alpha=.18)
    axes[0, 0].legend(fontsize=8); axes[1, 0].legend(fontsize=7)
    fig.suptitle('待机伸展机械臂遇移动障碍：底盘与机械臂均可动\n'
        '场景经离线验证存在纯机械臂避让通道；碰撞组底盘位移为零不能视为更优。'
        '新增待机避让是单独扩展，不代表现有项目原样行为。', fontsize=12)
    fig.savefig(path, dpi=125); plt.close(fig)


def select_animation_trials(trials):
    selected = []
    for group in GROUPS:
        choices = [t for t in trials if t['metadata']['group'] == group]
        if choices:
            choices.sort(key=lambda t: (t['metadata']['seed'] is None,
                                        str(t['metadata']['seed']), t['path'].name))
            selected.append(choices[0])
    return selected or trials[:3]


def make_figure(trials):
    fig, axes = plt.subplots(2, len(trials), figsize=(5.9*len(trials), 8.4),
                             dpi=92, squeeze=False)
    fig.subplots_adjust(left=.04, right=.985, top=.85, bottom=.14, wspace=.20, hspace=.31)
    artists = []
    for i, trial in enumerate(trials):
        top, front = axes[:, i]
        top.set(xlim=(-.68, .78), ylim=(-1.18, .82), xlabel='x / m', ylabel='y / m')
        front.set(xlim=(-1.18, .82), ylim=(0., 1.45), xlabel='y / m', ylabel='z / m')
        for ax in (top, front):
            ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.13)
        top.plot(*trial['base'][0, :2], '+', color='#111111', markersize=13)
        path_line, = top.plot([], [], color='#128375', linewidth=2.)
        base_tip, = top.plot([], [], color='#128375', linewidth=2.)
        spheres = []
        for index in range(67):
            arm = index >= 47; color = '#d47730' if arm else '#557a95'
            pair = [Circle((0., 0.), .01, facecolor=color, edgecolor=color,
                            alpha=.70 if arm else .18, linewidth=.6) for _ in range(2)]
            top.add_patch(pair[0]); front.add_patch(pair[1]); spheres.append(pair)
        actors = []
        for a in trial['scene']['actors']:
            circle = Circle((0., 0.), a['r'], color='#c8443f', alpha=.62)
            rectangle = Rectangle((0., a['z_min']), a['r']*2., a['z_max']-a['z_min'],
                                  color='#c8443f', alpha=.62)
            top.add_patch(circle); front.add_patch(rectangle); actors.append((circle, rectangle))
        status = front.text(.0, -.28, '', transform=front.transAxes, va='top', fontsize=9)
        artists.append((path_line, base_tip, spheres, actors, status))
    return fig, axes, artists


def draw_time(fig, axes, artists, trials, elapsed):
    fig.suptitle('待机避让实测回放｜独立实验时钟 %.2f s\n'
        '两类项目原样待机行为 + 新增待机避让扩展；底盘均可自由运动\n'
        '蓝：固定结构；橙：机械臂；红：移动障碍；绿：实际底盘轨迹；上俯视，下正视 y-z' % elapsed,
        fontsize=12)
    for i, trial in enumerate(trials):
        idx = int(np.clip(np.searchsorted(trial['t'], elapsed, side='right')-1, 0, len(trial['t'])-1))
        path_line, tip, spheres, actor_artists, status = artists[i]
        base = trial['base'][idx]; body = trial['bodies'][idx]; m = trial['metadata']
        shown_time = float(trial['t'][idx]); q = trial['q'][idx]
        gap = float(trial['gaps'][idx])
        displacement = float(trial['displacement'][idx]); length = float(trial['path_length'][idx])
        angle = float(np.rad2deg(trial['yaw_delta'][idx]))
        snapshot = trial['terminal_snapshot']
        # Contact can fall between CSV ticks. Display only the independent
        # integrator's real saved contact pose, never an interpolated motion.
        contact = snapshot is not None and elapsed >= snapshot['t']-1e-8
        if contact:
            shown_time = snapshot['t']; base = snapshot['base']; body = snapshot['body']; q = snapshot['q']
            gap = float(snapshot['external_gap']) if snapshot['external_gap'] is not None else gap
            displacement = float(np.linalg.norm(base[:2]-trial['base'][0, :2]))
            length += float(np.linalg.norm(base[:2]-trial['base'][-1, :2]))
            angle = float(np.rad2deg(math.atan2(math.sin(base[2]-trial['base'][0, 2]),
                                                math.cos(base[2]-trial['base'][0, 2]))))
        path_line.set_data(trial['base'][:idx+1, 0], trial['base'][:idx+1, 1])
        tip.set_data([base[0], base[0]+.20*np.cos(base[2])], [base[1], base[1]+.20*np.sin(base[2])])
        for sphere, pair in zip(body, spheres):
            pair[0].center = (sphere[0], sphere[1]); pair[1].center = (sphere[1], sphere[2])
            pair[0].radius = pair[1].radius = sphere[3]
        # If the recorded experiment ended, both its robot and actors freeze.
        actors = actors_3d_at(shown_time, trial['scene'])
        for j, (circle, rectangle) in enumerate(actor_artists):
            visible = j < len(actors); circle.set_visible(visible); rectangle.set_visible(visible)
            if visible:
                a = actors[j]; circle.center = (a['x'], a['y']); rectangle.set_x(a['y']-a['r'])
        end = snapshot['t'] if snapshot else float(trial['t'][-1])
        ended = elapsed >= end-1e-8
        collided = bool(gap < 0. or contact)
        state_label = ('%.3f s 接触，画面冻结（积分审计实测）' % shown_time if collided else
                       '运行中' if not ended else '完整运行结束，画面冻结' if m['avoidance_completed'] else
                       '记录已结束，画面冻结')
        q = np.rad2deg(q)
        status.set_text('%s\n底盘位移 %.3f mm；转角 %.3f°；累计路程 %.3f mm\nq2 %.1f°，q5 %.1f°；外部间隙 %s cm' %
            (state_label, displacement*1000., angle, length*1000.,
             q[1], q[4], '未出现障碍' if not np.isfinite(gap) else '%.2f' % (gap*100.)))
        status.set_color('#b42c28' if collided else '#333333')
        axes[0, i].set_title(m['label'] + ('\n画面冻结' if ended else ''), fontsize=10)


def animation_and_keyframe(trials, directory, frames=97):
    selected = select_animation_trials(trials)
    fig, axes, artists = make_figure(selected)
    end = max(float(t['terminal_snapshot']['t'] if t['terminal_snapshot'] else t['t'][-1]) for t in selected)
    times = np.linspace(0., end, min(frames, max(2, int(end/.20)+1)))
    images = []
    for time in times:
        draw_time(fig, axes, artists, selected, float(time))
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
                      .convert('P', palette=Image.ADAPTIVE))
    duration_ms = max(20, int(round(end/(len(images)-1)/2.*1000.)))
    images[0].save(directory/'待机避让_对照预览.gif', save_all=True, append_images=images[1:],
                   duration=duration_ms, loop=0, optimize=False, disposal=2)
    # Exact audited contact is the shared time for the still image.
    collisions = [t['metadata']['first_collision_time_s'] for t in selected
                  if t['metadata']['first_collision_time_s'] is not None]
    keytime = min(collisions) if collisions else min(10., end)
    draw_time(fig, axes, artists, selected, keytime)
    fig.savefig(directory/'待机避让_关键现象.png', dpi=150)
    plt.close(fig)
    for im in images:
        im.close()
    return dict(selected_trial_names=[t['path'].name for t in selected], frames=len(images),
                keyframe_time_s=keytime, playback_speed=2,
                source='actual measured base and joint positions; no invented candidate or motion',
                ended_panels_frozen=True,
                contact_display='actual integrator first_violation.state when contact follows final CSV tick')


def sideview_closeup(trials, path):
    """Project recorded arm spheres in x-z, joining samples only on each link."""
    selected = select_animation_trials(trials)
    contacts = [t['metadata']['first_collision_time_s'] for t in selected
                if t['metadata']['first_collision_time_s'] is not None]
    elapsed = max(contacts)+.01 if contacts else min(10., max(t['t'][-1] for t in selected))
    fig, axes = plt.subplots(1, len(selected), figsize=(5.3*len(selected), 6.6), squeeze=False)
    fig.subplots_adjust(left=.05, right=.985, bottom=.21, top=.81, wspace=.20)
    fig.suptitle('机械臂主动降臂避障｜x-z 侧视近景｜共同实验时钟 %.3f s\n'
        '灰色虚线：初始姿态；橙色：实际姿态；红色：移动障碍投影\n'
        '只连接同一连杆上的原生碰撞球采样中心；没有补画控制动作或跨连杆造型' % elapsed,
        fontsize=12)
    entries = []
    for ax, trial in zip(axes[0], selected):
        model = trial['model']; m = trial['metadata']
        idx = int(np.clip(np.searchsorted(trial['t'], elapsed, side='right')-1, 0, len(trial['t'])-1))
        base = trial['base'][idx]; q = trial['q'][idx]; shown_time = float(trial['t'][idx])
        snapshot = trial['terminal_snapshot']
        contact = snapshot is not None and elapsed >= snapshot['t']-1e-8
        if contact:
            base = snapshot['base']; q = snapshot['q']; shown_time = snapshot['t']
        initial = model.arm_spheres(trial['q'][0], trial['base'][0])
        actual = model.arm_spheres(q, base)
        for body in trial['bodies'][idx, :len(model.fixed_spheres)]:
            ax.add_patch(Circle((body[0], body[2]), body[3], color='#557a95', alpha=.08))
        for a in actors_3d_at(shown_time, trial['scene']):
            ax.add_patch(Rectangle((a['x']-a['r'], a['z_min']), a['r']*2., a['z_max']-a['z_min'],
                                   facecolor='#c8443f', edgecolor='#b12d29', alpha=.45))
        for body, color, style, alpha in ((initial, '#777777', '--', .65),
                                          (actual, '#d47730', '-', .95)):
            for sphere in body:
                ax.add_patch(Circle((sphere[0], sphere[2]), sphere[3], facecolor='none',
                                    edgecolor=color, linestyle=style, alpha=alpha, linewidth=1.))
            for link in np.unique(model.link_ids):
                samples = body[model.link_ids == link]
                ax.plot(samples[:, 0], samples[:, 2], style, color=color, linewidth=2.8, alpha=alpha)
        ax.set(xlim=(-.16, .59), ylim=(.32, 1.36), xlabel='x / m', ylabel='z / m')
        ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.16)
        ax.set_title(m['label'].replace(' · seed ', '\nseed '), fontsize=10)
        displacement = float(np.linalg.norm(base[:2]-trial['base'][0, :2]))*1000.
        angles = np.rad2deg(q); initial_angles = np.rad2deg(trial['q'][0])
        ax.text(0., -.17, '%s\nq2：%.1f° → %.1f°；q5：%.1f° → %.1f°\n底盘实际位移 %.3f mm' %
            ('接触后冻结于 %.3f s' % shown_time if contact else '实际记录姿态 %.3f s' % shown_time,
             initial_angles[1], angles[1], initial_angles[4], angles[4], displacement),
            transform=ax.transAxes, va='top', fontsize=10,
            color='#b42c28' if contact else '#333333')
        entries.append(dict(trial=trial['path'].name, displayed_record_time_s=shown_time,
                            contact_snapshot=contact, actual_base_displacement_mm=displacement))
    fig.text(.5, .015, '侧视投影用于观察折臂；碰撞由完整三维球—有限圆柱距离判定。底盘未固定。',
             ha='center', fontsize=10, color='#555555')
    fig.savefig(path, dpi=150); plt.close(fig)
    return dict(common_time_s=elapsed, panels=entries,
                geometry='native arm collision-sphere centers, connected within each original link only')


def report(root, frames=97, names=None):
    root = Path(root).resolve()
    candidates = [root/name for name in names] if names else sorted(p.parent for p in root.glob('*/config.json'))
    if names and any(p.parent != root for p in candidates):
        raise ValueError('--trials accepts direct trial directory names only')
    admissions = [trial_status(p) for p in candidates]
    excluded = [s for s in admissions if not s['complete']]
    if names and excluded:
        raise ValueError('Explicitly selected trial is incomplete: '+str(excluded))
    trials = [load_trial(p) for p, status in zip(candidates, admissions) if status['complete']]
    if not trials:
        raise ValueError('No completed standby trials under '+str(root))
    trials.sort(key=lambda t: (GROUPS.index(t['metadata']['group']) if t['metadata']['group'] in GROUPS else 99,
                               str(t['metadata']['seed']), t['path'].name))
    directory = root/'report'; directory.mkdir(exist_ok=True)
    data_plot(trials, directory/'待机避让_数据对照.png')
    closeup = sideview_closeup(trials, directory/'机械臂避障_侧视近景.png')
    movie = animation_and_keyframe(trials, directory, frames)
    summary = dict(schema='standby_comparison_report_v2', trials=[t['metadata'] for t in trials],
        excluded_trials=excluded,
        animation=movie, sideview_closeup=closeup,
        statistical_claim='limited seeds and one scene; no general success-rate claim',
        scope='native DH sphere model numerical experiment; no mesh or real hardware claim',
        interpretation=['base is free for every actual trial',
            'offline arm-only witness proves scene availability, not observed controller performance',
            'original native and current standby behavior are distinct from the new standby extension',
            'extension is standby protection only; no REMANI slow-layer optimization or handoff is exercised',
            'extension consumes a separate ideal moving-object world cloud; A/B retain native map/tracker perception',
            'comparison measures complete system standby capability, not MPPI superiority under identical perception',
            'zero base movement during collision is not a successful low-disturbance outcome',
            'actual base displacement/path/angle/return error come from complete state.csv'])
    (directory/'实验结果.json').write_text(json.dumps(safe(summary), ensure_ascii=False, indent=2)+'\n')
    lines = ['待机状态：只需移动机械臂即可避障的场景中，底盘实际受多大影响', '',
        '共同条件：无导航目标，机械臂初始伸展，移动高位圆柱；底盘和机械臂均可动。',
        '离线预检提供纯机械臂可行通道；预检姿态时间表不参与实际控制。',
        '原生 REMANI、当前项目原样待机、新增待机避让扩展分列；不能将新扩展写成当前项目原有能力。',
        '新增扩展只测试待机保护，没有执行 REMANI 慢层重规划或快慢层交接。',
        '新增扩展读取单独的理想移动物体世界坐标点云；A/B 保留各自原生感知链。',
        '因此本次是整套系统的待机能力对照，不能据此单独归因于 MPPI 算法优越性。',
        '碰撞组即使底盘位移为零，也不能作为避让效果更好的证据。', '']
    for trial in trials:
        m = trial['metadata']
        lines += [m['label'],
            '  终止 %s；物理球模型接触 %s；记录结束 %.3f s。' %
                (m['termination'], m['collision_physical'], m['last_recorded_time_s']),
            '  独立积分审计的首次接触时间 %s s（CSV 末帧可能早于接触）。' % m['first_audited_collision_s'],
            '  完整无接触且无关节违规 %s；实际底盘：最大位移 %.4f mm；累计路程 %.4f mm。' %
                (m['avoidance_completed'], m['actual_max_base_displacement_mm'], m['actual_base_path_length_mm']),
            '  实际底盘最大转角 %.4f°；累计转角 %.4f°。' %
                (
                 m['actual_max_base_angle_deg'], m['actual_cumulative_base_angle_deg']),
            '  最终相对初始误差：位置 %.4f mm，航向 %.4f°。' %
                (m['actual_final_base_position_error_mm'], m['actual_final_base_yaw_error_deg']),
            '  实际最大单关节变化 %.3f°；最终关节返回误差 %.3f°。' %
                (m['maximum_joint_displacement_deg'], m['final_joint_position_error_deg']),
            '  实测姿态采样的最小外部间隙 %.3f cm；独立审计间隙 %s m。' %
                (m['measured_min_external_gap_m']*100., m['independently_audited_min_external_gap_m']),
            '  关节限制违规 %s；日志条目数 %d；求解/回调 P95/最大 %s/%s ms。' %
                (m['joint_limit_violation'], m['cycle_count'], m['callback_p95_ms'], m['callback_max_ms']), '']
    lines += ['解释边界：本图是原生 DH 球模型的数值实测回放，非 URDF 网格或实车。',
        '每组底盘统计使用完整 state.csv；“最大位移”和“累计路程”分别衡量离待机点偏离程度及总扰动。',
        '只能在避障成功组中讨论底盘扰动；碰撞/提前结束组的零位移不是更优结果。',
        'GIF 选取每类首个种子，其他种子都保留在数据图与JSON；结束组画面冻结，不补画后续控制。',
        '图示间隙从记录姿态重算，独立积分审计可能给出更小间隙；二者分列。']
    lines += ['原生组日志条目为底盘/关节命令消息；当前两组为快层回调记录，不能据此比较 MPPI 计算周期。']
    if excluded:
        lines += ['', '未纳入统计的未完成/中断案例：']
        lines += ['  %s：%s' % (Path(item['trial']).name, '；'.join(item['reasons'])) for item in excluded]
    (directory/'实验结果.txt').write_text('\n'.join(lines)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT)
    parser.add_argument('--gif-max-frames', type=int, default=97)
    parser.add_argument('--trials', nargs='+', help='Only these completed direct trial directory names')
    args = parser.parse_args()
    if not 2 <= args.gif_max_frames <= 160:
        parser.error('GIF frames must lie in [2, 160]')
    summary = report(args.out, args.gif_max_frames, args.trials)
    print('STANDBY_REPORT_COMPLETE', args.out/'report', len(summary['trials']), 'trials')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Read measured active-arm SIL records and render an honest A/B report.

No ROS, optimization, actuation, shutdown, or candidate reconstruction occurs.
Ground-truth actors are used only for the scene picture and independent audit.
"""
import argparse
import csv
import hashlib
import importlib.util
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
from active_arm_scene import actors_3d_at, cylinder_gaps, validate_scene


HERE = Path(__file__).resolve().parent
DEFAULT = HERE / 'results/active_arm_20261104_01'
COLORS = ('#276b98', '#d47730', '#458d62', '#8b62a5')
plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False,
                     'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def first_number(mapping, keys):
    for key in keys:
        value = finite(mapping.get(key))
        if value is not None:
            return value, key
    return None, None


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (np.integer, np.bool_)):
        return value.item()
    return value


def load_jsonl(path):
    if not path.exists():
        return [], ['missing ' + path.name]
    rows, errors = [], []
    for lineno, line in enumerate(path.read_text().splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('cycle is not an object')
            rows.append(row)
        except (ValueError, TypeError) as exc:
            errors.append('%s:%d: %s' % (path.name, lineno, exc))
    return rows, errors


def load_scene(trial, config):
    options = [config.get('scene_definition'), config.get('scene')]
    if (trial / 'scenario.json').exists():
        options.append(json.loads((trial / 'scenario.json').read_text()))
    for scene in options:
        if isinstance(scene, dict) and scene.get('schema') == 'active_arm_scene_v1':
            validate_scene(scene)
            return scene
    raise ValueError('No saved active_arm_scene_v1 definition; refusing to invent scene truth')


def load_model(trial, config):
    """Prefer the trial-frozen model implementation and parameters."""
    source = trial / 'runtime/active_arm_model.py'
    cls = PiperArmModel
    if source.exists():
        spec = importlib.util.spec_from_file_location('report_model_' + trial.name, source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        cls = module.PiperArmModel
    else:
        source = HERE / 'active_arm_model.py'
    parameters = trial / 'parameters.yaml'
    if not parameters.exists():
        raise ValueError('Missing trial parameters.yaml; exact reported sphere model cannot be established')
    kwargs = {'urdf_path': config['urdf_path']} if config.get('urdf_path') else {}
    return cls.from_parameters(parameters, **kwargs), dict(source=str(source), source_sha256=sha(source),
                                                parameters=str(parameters), parameters_sha256=sha(parameters))


def load_trial(trial):
    trial = Path(trial)
    config = json.loads((trial / 'active_arm_config.json').read_text())
    result = json.loads((trial / 'result.json').read_text())
    with (trial / 'state.csv').open() as stream:
        raw = list(csv.DictReader(stream))
    required = ['t', 'x', 'y', 'yaw', 'v', 'w'] + ['q%d' % i for i in range(1, 7)] + ['dq%d' % i for i in range(1, 7)]
    if not raw or any(key not in raw[0] for key in required):
        raise ValueError('Missing measured base/q/dq state columns or empty state.csv')
    state = {key: np.asarray([float(row[key]) for row in raw]) for key in required}
    for key in ('clearance', 'configuration_gap', 'whole_body_command_fresh'):
        if key in raw[0]:
            state[key] = np.asarray([float(row[key]) for row in raw])
    if not all(np.all(np.isfinite(state[k])) for k in required) or np.any(np.diff(state['t']) < 0):
        raise ValueError('Nonfinite states or backwards measured time')
    scene = load_scene(trial, config)
    model, model_provenance = load_model(trial, config)
    q = np.column_stack([state['q%d' % i] for i in range(1, 7)])
    dq = np.column_stack([state['dq%d' % i] for i in range(1, 7)])
    base = np.column_stack([state[k] for k in ('x', 'y', 'yaw')])
    bodies = model.body_spheres(q, base)
    if bodies.shape[1:] != (67, 4):
        raise ValueError('Recorded model is not the declared 67-sphere model')
    motion_anchor = finite(result.get('motion_anchor'))
    goal_time = finite(result.get('goal_sent'))
    origin = motion_anchor if motion_anchor is not None else goal_time if goal_time is not None else float(state['t'][0])
    origin_kind = 'motion_anchor' if motion_anchor is not None else 'goal_sent' if goal_time is not None else 'first_recorded_state'
    relative = state['t'] - origin
    mask = relative >= -1e-9
    if not np.any(mask):
        mask = np.ones(len(relative), dtype=bool)
    # No motion anchor means these motion-triggered actors never appeared.
    actor_gaps = np.asarray([float(np.min(cylinder_gaps(body,
                    actors_3d_at(t-motion_anchor, scene) if motion_anchor is not None else [])))
                    for body, t in zip(bodies, state['t'])])
    observed_gap, observed_key = first_number(result,
        ('min_external_gap_m', 'external_min_gap', 'min_clearance', 'min_physical_sample_gap_m'))
    if observed_gap is None and 'clearance' in state:
        values = state['clearance'][np.isfinite(state['clearance'])]
        if len(values):
            observed_gap, observed_key = float(np.min(values)), 'state.csv:clearance'
    configuration_gap, configuration_key = first_number(result, ('min_configuration_gap_m',))
    if configuration_gap is None and 'configuration_gap' in state:
        values = state['configuration_gap'][np.isfinite(state['configuration_gap'])]
        if len(values):
            configuration_gap, configuration_key = float(np.min(values)), 'state.csv:configuration_gap'
    cycles, read_errors = load_jsonl(trial / 'active_arm_cycles.jsonl')
    timings, timing_keys = [], set()
    for row in cycles:
        value, key = first_number(row, ('callback_ms', 'total_callback_ms', 'cycle_ms', 'elapsed_ms', 'compute_ms'))
        if value is not None and value >= 0:
            timings.append(value); timing_keys.add(key)
    cycle_reasons = {}
    for row in cycles:
        reason = str(row.get('reason', row.get('status', 'not_recorded')))
        cycle_reasons[reason] = cycle_reasons.get(reason, 0) + 1
    approved_arm_commands = sum(row.get('reason') == 'ok'
        and isinstance(row.get('command'), list) and len(row['command']) == 8
        and np.linalg.norm(np.asarray(row['command'][2:], dtype=float)) > 1e-5 for row in cycles)
    approved_arm_rollouts = sum(row.get('reason') == 'ok' and finite(row.get('arm_motion')) is not None
                               and float(row['arm_motion']) > 1e-5 for row in cycles)
    # Plot alignment must not discard motion from whole-record statistics.
    path_length = float(np.sum(np.linalg.norm(np.diff(base[:, :2], axis=0), axis=1)))
    arm_total = float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))
    arm_max = float(np.max(np.abs(q - np.asarray(scene['q_start']))))
    reason = str(result.get('reason', result.get('termination', 'unknown')))
    reached = bool(result.get('goal_reached', result.get('reached_goal', reason == 'goal')))
    collision = result.get('collision_physical', result.get('collision', None))
    enabled = config.get('active_arm_enabled', config.get('enabled', config.get('active_arm', None)))
    group = config.get('group', config.get('comparison_group', trial.name))
    label = str(config.get('label', group))
    label = {'A': 'A 固定臂', 'A_fixed': 'A 固定臂',
             'B': 'B 主动臂', 'B_active': 'B 主动臂'}.get(label, label)
    seed = config.get('seed')
    if seed is not None:
        label += ' · seed ' + str(seed)
    deadline_s = finite(config.get('deadline_s'))
    over_deadline = sum(value > deadline_s*1000. for value in timings) if deadline_s is not None else None
    metadata = dict(trial=str(trial), label=label, group=group, seed=seed, active_arm_enabled=enabled,
        termination=reason, goal_reached=reached, collision_physical=collision,
        recorded_samples=len(q), motion_anchor_s=motion_anchor, plot_origin_s=origin,
        plot_origin_kind=origin_kind, observed_motion_duration_s=float(state['t'][-1]-origin),
        actual_base_path_length_m=path_length, maximum_absolute_base_y_m=float(np.max(np.abs(base[:, 1]))),
        aggregate_scope='complete state.csv, including all samples before the plot alignment origin',
        record_start_s=float(state['t'][0]), record_end_s=float(state['t'][-1]),
        recorded_duration_s=float(state['t'][-1]-state['t'][0]),
        arm_max_joint_displacement_rad=arm_max, arm_path_length_l2_rad=arm_total,
        max_abs_measured_joint_speed_rad_s=np.max(np.abs(dq), axis=0).tolist(),
        minimum_physical_gap_m=observed_gap, minimum_physical_gap_source=observed_key,
        minimum_configuration_gap_m=configuration_gap, minimum_configuration_gap_source=configuration_key,
        joint_limit_violation=result.get('joint_limit_violation'),
        maximum_audit_integration_step_s=result.get('maximum_integration_step_s'),
        audit_substeps=result.get('audit_substeps'),
        recorded_pose_actor_min_gap_m=finite(np.min(actor_gaps)),
        recorded_pose_actor_gap_scope='recomputed finite cylinder only at recorded measured poses; excludes walls and between-record substeps',
        callback_count=len(timings), callback_p95_ms=float(np.percentile(timings, 95)) if timings else None,
        callback_max_ms=float(np.max(timings)) if timings else None,
        callback_over_deadline_count=over_deadline,
        callback_over_deadline_fraction=over_deadline/len(timings) if timings and over_deadline is not None else None,
        callback_fields=sorted(timing_keys), cycle_reason_counts=cycle_reasons,
        approved_nonzero_arm_command_cycles=approved_arm_commands,
        approved_nonzero_predicted_arm_motion_cycles=approved_arm_rollouts,
        cycle_min_gap_scope='cycle min_gap combines external/self/base-arm/ground; never used as physical external gap',
        controller_horizon=config.get('horizon'), controller_random_samples=config.get('random_samples'),
        controller_deadline_s=deadline_s, controller_rate_hz=config.get('rate_hz'),
        raw_cycle_count=len(cycles), read_errors=read_errors, model=model_provenance,
        source_sha256={name: sha(trial/name) for name in ('active_arm_config.json', 'state.csv', 'result.json', 'active_arm_cycles.jsonl') if (trial/name).exists()})
    return dict(metadata=metadata, config=config, result=result, scene=scene, model=model,
                state=state, q=q, base=base, bodies=bodies, relative_t=relative,
                mask=mask, actor_gaps=actor_gaps, cycles=cycles)


def display(value, digits=3):
    return '未记录' if value is None else ('%.' + str(digits) + 'f') % value


def plot_data(trials, path, comparison_note):
    fig, axes = plt.subplots(2, 3, figsize=(16, 9.8), constrained_layout=True)
    names = [t['metadata']['label'] for t in trials]
    for i, trial in enumerate(trials):
        color = COLORS[i % len(COLORS)]; meta = trial['metadata']; mask = trial['mask']
        t = trial['relative_t'][mask]; q = np.rad2deg(trial['q'][mask])
        axes[0, 0].plot(trial['base'][:, 0], trial['base'][:, 1], color=color, label=names[i])
        axes[0, 0].plot(*trial['base'][-1, :2], 'o', color=color)
        axes[0, 1].plot(t, q[:, 1], color=color, label=names[i]+' q2')
        axes[0, 1].plot(t, q[:, 4], '--', color=color, label=names[i]+' q5')
        gaps = trial['state'].get('clearance', trial['actor_gaps'])[mask]
        valid = np.isfinite(gaps)
        axes[0, 2].plot(t[valid], gaps[valid], color=color, label=names[i])
        axes[1, 0].bar(i-.16, meta['actual_base_path_length_m'], width=.30, color=color)
        axes[1, 0].bar(i+.16, meta['maximum_absolute_base_y_m'], width=.30, color=color, alpha=.4)
        if meta['callback_p95_ms'] is not None:
            axes[1, 1].bar(i-.16, meta['callback_p95_ms'], width=.30, color=color)
            axes[1, 1].bar(i+.16, meta['callback_max_ms'], width=.30, color=color, alpha=.4)
    axes[0, 0].set(title='全程实测底盘轨迹（终止点保留）', xlabel='x / m', ylabel='y / m')
    axes[0, 0].set_aspect('equal', adjustable='datalim'); axes[0, 0].legend(fontsize=8)
    axes[0, 1].set(title='实测肩关节 q2 与腕关节 q5', xlabel='记录对齐后 / s', ylabel='关节角 / °')
    axes[0, 1].legend(fontsize=7, ncol=2)
    axes[0, 2].axhline(0., color='#b94c4c', linewidth=.8)
    axes[0, 2].set(title='物理球模型外部间隙', xlabel='记录对齐后 / s', ylabel='间隙 / m')
    axes[0, 2].legend(fontsize=8)
    tick_names = [name.replace(' · ', '\n') for name in names]
    axes[1, 0].set(title='全程实走路程 / 最大横向偏移', ylabel='m', xticks=range(len(names)), xticklabels=tick_names)
    axes[1, 0].text(.02, .97, '实色：路程；浅色：最大 |y|', transform=axes[1, 0].transAxes, va='top', fontsize=9)
    axes[1, 1].set(title='实测 callback P95 / 最大值', ylabel='ms', xticks=range(len(names)), xticklabels=tick_names)
    axes[1, 1].text(.02, .97, '实色：P95；浅色：最大值', transform=axes[1, 1].transAxes, va='top', fontsize=9)
    deadlines = sorted({t['metadata']['controller_deadline_s'] for t in trials
                        if t['metadata']['controller_deadline_s'] is not None})
    for deadline in deadlines:
        axes[1, 1].axhline(deadline*1000., color='#b94c4c', linestyle='--', linewidth=.9,
                          label='配置期限 %g ms' % (deadline*1000.))
    if deadlines:
        axes[1, 1].legend(fontsize=8, loc='upper right')
    for ax in axes[1, :2]:
        ax.set_ylim(0., ax.get_ylim()[1]*1.12)
        ax.tick_params(axis='x', labelsize=8)
    axes[1, 2].axis('off')
    y = .98
    for i, trial in enumerate(trials):
        m = trial['metadata']
        lines = [m['label'] + '：' + ('到达目标' if m['goal_reached'] else m['termination']),
                 '对齐至结束 %s s；最小物理间隙 %s m' % (display(m['observed_motion_duration_s']), display(m['minimum_physical_gap_m'])),
                 '最大关节变化 %s°；callback 最大 %s ms' % (display(math.degrees(m['arm_max_joint_displacement_rad']), 1), display(m['callback_max_ms'], 1))]
        axes[1, 2].text(0., y, '\n'.join(lines), va='top', color=COLORS[i % len(COLORS)], fontsize=9)
        y -= .96/len(trials)
    for ax in axes.flat:
        if ax.axison:
            ax.grid(alpha=.2)
    fig.suptitle('主动机械臂避障：同场景数值闭环实录\n' + comparison_note + '\n'
                 '全程统计保留起步前记录；时间曲线按运动触发时刻对齐。原生 DH 球模型，尚非网格/实车验证。', fontsize=12)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def make_gif(trials, path, max_frames=120):
    if not 2 <= max_frames <= 240:
        raise ValueError('GIF frame limit must be between 2 and 240')
    count = len(trials)
    fig, axes = plt.subplots(2, count, figsize=(5.4*count, 6.4), squeeze=False, dpi=85)
    fig.subplots_adjust(left=.055, right=.985, top=.86, bottom=.11, wspace=.14, hspace=.32)
    # Preserve common axes while keeping actual excursions and arm spheres visible.
    lower = np.min([np.min(t['bodies'][:, :, :3]-t['bodies'][:, :, 3:4], axis=(0, 1)) for t in trials], axis=0)
    upper = np.max([np.max(t['bodies'][:, :, :3]+t['bodies'][:, :, 3:4], axis=(0, 1)) for t in trials], axis=0)
    xlim = (min(-.45, lower[0]-.05), max(3.55, upper[0]+.05))
    ylim = (min(-.95, lower[1]-.05), max(.95, upper[1]+.05))
    zlim = (min(0., lower[2]-.05), max(1.55, upper[2]+.05))
    artists = []
    for i, trial in enumerate(trials):
        scene = trial['scene']; top, side = axes[:, i]
        top.set(xlim=xlim, ylim=ylim, xlabel='x / m', ylabel='y / m')
        side.set(xlim=xlim, ylim=zlim, xlabel='x / m', ylabel='z / m')
        for ax in (top, side):
            ax.set_aspect('equal', adjustable='box'); ax.grid(alpha=.12)
        trail, = top.plot([], [], color=COLORS[i % len(COLORS)], linewidth=1.6)
        top.plot(scene['goal'][0], scene['goal'][1], '+', color='#333333', markersize=10)
        spheres = []
        for index in range(67):
            color = '#d47730' if index >= 47 else '#427ba1'
            pair = [Circle((0., 0.), .01, facecolor=color, edgecolor=color, alpha=.22, linewidth=.5) for _ in range(2)]
            top.add_patch(pair[0]); side.add_patch(pair[1]); spheres.append(pair)
        actors = []
        for actor in scene['actors']:
            circle = Circle((actor['x'], actor['y']), actor['r'], color='#777777', alpha=.45)
            rect = Rectangle((actor['x']-actor['r'], actor['z_min']), 2*actor['r'], actor['z_max']-actor['z_min'], color='#777777', alpha=.45)
            top.add_patch(circle); side.add_patch(rect); actors.append((circle, rect))
        text = side.text(.01, -.20, '', transform=side.transAxes, va='top', fontsize=8)
        artists.append((trail, spheres, actors, text))
    end = max(max(0., float(t['relative_t'][-1])) for t in trials)
    frame_count = min(max_frames, max(2, int(math.ceil(end/.25))+1))
    times = np.linspace(0., end, frame_count)
    frames = []
    for elapsed in times:
        fig.suptitle('记录对齐后 %.2f s｜俯视 + 侧视：实测关节驱动的 67 球\n蓝：固定结构 / 实走底盘；橙：机械臂；灰：已出现障碍（评估真值）' % elapsed, fontsize=11)
        for i, trial in enumerate(trials):
            trail, spheres, actor_artists, info = artists[i]
            index = int(np.clip(np.searchsorted(trial['relative_t'], elapsed, side='right')-1, 0, len(trial['q'])-1))
            body = trial['bodies'][index]
            trail.set_data(trial['base'][:index+1, 0], trial['base'][:index+1, 1])
            for row, pair in zip(body, spheres):
                pair[0].center = (row[0], row[1]); pair[1].center = (row[0], row[2])
                pair[0].radius = pair[1].radius = row[3]
            anchor = trial['metadata']['motion_anchor_s']
            actors = actors_3d_at(trial['state']['t'][index]-anchor, trial['scene']) if anchor is not None else []
            for j, (circle, rect) in enumerate(actor_artists):
                visible = j < len(actors); circle.set_visible(visible); rect.set_visible(visible)
                if visible:
                    actor = actors[j]; circle.center = (actor['x'], actor['y']); rect.set_x(actor['x']-actor['r'])
            ended = elapsed > trial['relative_t'][-1]+1e-6
            axes[0, i].set_title(trial['metadata']['label'] + ('｜已结束，画面冻结' if ended else ''), fontsize=11)
            q = np.rad2deg(trial['q'][index]); s = trial['state']
            info.set_text('v=%.2f m/s；q2=%.1f°，q5=%.1f°\n%s' %
                          (s['v'][index], q[1], q[4], trial['metadata']['termination'] if ended else '实测状态；未补画候选搜索'))
        fig.canvas.draw()
        frame = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()).convert('P', palette=Image.ADAPTIVE)
        frames.append(frame)
    plt.close(fig)
    # Fixed two-times playback; source time is always visible in the header.
    duration_ms = max(20, int(round((end/max(1, frame_count-1))/2.*1000.)))
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=duration_ms,
                   loop=0, optimize=False, disposal=2)
    for frame in frames:
        frame.close()
    return dict(frames=frame_count, duration_per_frame_ms=duration_ms, nominal_playback_speed=2,
                views='top and side', source='measured base pose and joint positions',
                actual_candidates_drawn=False, ended_panels_frozen=True)


def report(out, selected=None, max_frames=120):
    out = Path(out).resolve()
    if not out.is_dir():
        raise ValueError('Result directory does not exist: ' + str(out))
    comparison_plan = None
    plan_path = out/'comparison_plan_v2.json'
    if plan_path.exists():
        plan = json.loads(plan_path.read_text())
        planned_names = [job['tag'] for job in plan.get('jobs', [])]
        if not selected and planned_names:
            selected = planned_names
        comparison_plan = dict(path=str(plan_path), sha256=sha(plan_path),
                               planned_trial_names=planned_names,
                               selected_matches_plan=set(selected or []) == set(planned_names),
                               complete_at_report_time=plan.get('complete'))
    paths = [out/name for name in selected] if selected else sorted(p.parent for p in out.glob('*/active_arm_config.json'))
    if not paths:
        raise ValueError('No trial active_arm_config.json found under ' + str(out))
    trials, incomplete = [], []
    for path in paths:
        try:
            trials.append(load_trial(path))
        except (ValueError, OSError, KeyError, TypeError) as exc:
            incomplete.append(dict(trial=str(path), unavailable_reason=str(exc)))
    if not trials:
        raise ValueError('No usable measured trial; retained failures: ' + json.dumps(incomplete, ensure_ascii=False))
    selected_paths = {path.resolve() for path in paths}
    other_records = []
    # Explicit formal selection must not hide earlier interrupted or failed runs.
    for config_path in sorted(out.glob('*/active_arm_config.json')):
        if config_path.parent.resolve() in selected_paths:
            continue
        path = config_path.parent
        entry = dict(trial=str(path), included_in_selected_comparison=False,
                     source_sha256={'active_arm_config.json': sha(config_path)})
        try:
            config = json.loads(config_path.read_text())
            result_path = path/'result.json'
            result = json.loads(result_path.read_text())
            entry.update(group=config.get('group'), seed=config.get('seed'),
                         termination=result.get('reason', 'unknown'),
                         goal_reached=result.get('goal_reached'),
                         collision_physical=result.get('collision_physical'),
                         joint_limit_violation=result.get('joint_limit_violation'),
                         elapsed_s=result.get('elapsed'),
                         minimum_physical_gap_m=result.get('min_external_gap_m'))
            entry['source_sha256']['result.json'] = sha(result_path)
        except (ValueError, OSError, TypeError) as exc:
            entry['unavailable_reason'] = str(exc)
        other_records.append(entry)
    seed_groups = {}
    for trial in trials:
        m = trial['metadata']
        if m['seed'] is not None:
            seed_groups.setdefault(str(m['seed']), set()).add(m['active_arm_enabled'])
    paired_seeds = sorted(seed for seed, groups in seed_groups.items() if True in groups and False in groups)
    comparison_note = ('本图 %d 轮，%d 个种子形成 A/B 对照；样本不足以推断总体成功率。'
                       '开发记录另列保留。' % (len(trials), len(paired_seeds)))
    directory = out/'report'; directory.mkdir(exist_ok=True)
    plot_data(trials, directory/'对照数据.png', comparison_note)
    gif = make_gif(trials, directory/'主动收臂对照.gif', max_frames=max_frames)
    summary = dict(schema='active_arm_report_v2', trials=[t['metadata'] for t in trials],
        comparison_plan=comparison_plan,
        comparison_trial_count=len(trials), paired_seed_count=len(paired_seeds), paired_seeds=paired_seeds,
        retained_other_records=other_records, unavailable_trials=incomplete, gif=gif,
        scope='native-DH 67-sphere numerical ROS SIL; not precise URDF mesh, Gazebo, hardware or continuous collision proof',
        limitations=['limited asynchronous paired seeds do not establish population success rate',
                     'large joint displacement is measured; small or smooth arm adjustment is not established',
                     'callback duration may exceed configured deadline; no hard real-time guarantee',
                     'whole-record path and joint-motion aggregates include pre-alignment samples',
                     'physical external gaps and self/base-arm/ground validity are different measurements',
                     'no unrecorded candidate search has been drawn',
                     'failure and timeout are retained; termination times are not all successful traversal times'])
    (directory/'实验结果.json').write_text(json.dumps(json_safe(summary), ensure_ascii=False, indent=2)+'\n')
    lines = ['主动机械臂避障：数值闭环记录', '',
             comparison_note,
             '统一条件：原生 DH 球采样模型（47 个固定球＋20 个臂球），机械臂位置随实测关节变化。',
             '不是 URDF 网格精确碰撞、Gazebo 或实车验证；离散采样无接触不等于连续安全证明。',
             '路程、最大横移和关节累计路长使用完整 state.csv；时间曲线/GIF 按 motion_anchor 对齐，未将此前记录从全程统计中丢弃。', '']
    if comparison_plan:
        lines += ['比较版本来源：comparison_plan_v2.json；本次选择与该计划完全一致：%s。' % comparison_plan['selected_matches_plan'],
                  'v1、development 和 smoke 原始试验均保留在结果目录，并在本报告末尾列出；最终比较版本以 v2 计划为准。', '']
    for trial in trials:
        m = trial['metadata']
        lines += [m['label']+'：'+('到达目标' if m['goal_reached'] else m['termination']),
                  '  全程实走 %s m，最大 |y| %s m，对齐至结束 %s s。' % tuple(display(m[k]) for k in ('actual_base_path_length_m', 'maximum_absolute_base_y_m', 'observed_motion_duration_s')),
                  '  时间对齐来源 %s，原始时刻 %s s。' % (m['plot_origin_kind'], display(m['plot_origin_s'])),
                  '  最小物理外部间隙 %s m（来源 %s）；物理采样接触 %s。' % (display(m['minimum_physical_gap_m']), m['minimum_physical_gap_source'], m['collision_physical']),
                  '  自碰/底盘臂/地面综合最小间隙 %s m；关节约束违规 %s。' % (display(m['minimum_configuration_gap_m']), m['joint_limit_violation']),
                  '  最大单关节变化 %s°，关节空间累计 L2 路长 %s rad。' % (display(math.degrees(m['arm_max_joint_displacement_rad']), 2), display(m['arm_path_length_l2_rad'])),
                  '  已记录周期 P95/最大 %s/%s ms，共 %d 条，实际计时键 %s。' % (display(m['callback_p95_ms'], 1), display(m['callback_max_ms'], 1), m['callback_count'], m['callback_fields']),
                  '  callback 超过配置期限 %s ms 的周期 %s/%d；配置期限不等于硬实时保证。' % (display(m['controller_deadline_s']*1000. if m['controller_deadline_s'] is not None else None, 1), m['callback_over_deadline_count'], m['callback_count']),
                  '  获准非零关节命令 %d 周期；获准预测含臂运动 %d 周期。' % (m['approved_nonzero_arm_command_cycles'], m['approved_nonzero_predicted_arm_motion_cycles']),
                  '  实际配置 horizon=%s，random_samples=%s，deadline_s=%s，rate_hz=%s。' % (m['controller_horizon'],m['controller_random_samples'],m['controller_deadline_s'],m['controller_rate_hz']),
                  '  周期状态 %s；记录读取问题 %s。' % (json.dumps(m['cycle_reason_counts'], ensure_ascii=False), m['read_errors']), '']
    if incomplete:
        lines += ['无法形成完整曲线的试验（保留，不隐藏）：'] + ['  '+p['trial']+'：'+p['unavailable_reason'] for p in incomplete] + ['']
    if other_records:
        lines += ['此前开发/冒烟及其他未入本次对照的记录（原文件全部保留；不混入正式对照）：']
        for record in other_records:
            lines += ['  %s：终止 %s，到达目标 %s，物理采样接触 %s，最小外部间隙 %s m%s。' %
                      (Path(record['trial']).name, record.get('termination', '未记录'),
                       record.get('goal_reached'), record.get('collision_physical'),
                       display(finite(record.get('minimum_physical_gap_m'))),
                       '；读取问题 '+record['unavailable_reason'] if record.get('unavailable_reason') else '')]
        lines += ['']
    lines += ['解释边界：',
              '%d 个种子的 %d 对异步 A/B 仍不能直接给出总体成功率。固定臂基线也有几何绕行通道；不得把预检通道当作在线成功。' % (len(seed_groups), len(paired_seeds)),
              '主动臂包含较大的实际关节调整，应结合每组最大单关节变化评估，不能描述为小幅或已证明平滑。',
              '实测 callback 最大值可能超过配置的 200 ms 期限；报告逐组列出超期数量，不能宣称硬实时保证。',
              '只启用主动臂开关不等于实际选中了臂运动，应结合 measured q 与周期日志判断。',
              'cycles 的 min_gap 混合外障/自碰/地面，未拿它充当独立外障最小物理间隙。',
              '失败/超时的终止时刻不是成功通行时间，不能按更早结束宣称更快。',
              'GIF 从实测底盘与关节重算球心；只展示已出现障碍，结束面板冻结，没有补画搜索或未来动作。',
              '有起步记录时按起步时刻对齐；没有起步的失败样本按目标或首条记录对齐，见每组时间来源。',
              'JSON 保存源记录 SHA256、模型来源、计时键和全部终止状态。']
    (directory/'实验结果.txt').write_text('\n'.join(lines)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT)
    parser.add_argument('--trials', nargs='+', help='Explicit child trial directory names; all discovered trials by default')
    parser.add_argument('--gif-max-frames', type=int, default=120)
    args = parser.parse_args()
    try:
        result = report(args.out, args.trials, args.gif_max_frames)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, 'ACTIVE_ARM_REPORT_ERROR: '+str(exc)+'\n')
    print('ACTIVE_ARM_REPORT_COMPLETE', args.out/'report', len(result['trials']), 'trials')


if __name__ == '__main__':
    main()

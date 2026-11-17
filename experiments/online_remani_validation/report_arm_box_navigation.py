#!/usr/bin/env python3
"""Offline report of recorded native / MPPI / frozen-reference arm-box trials.

Reads recorded states only. No optimizer, ROS client or motion command is run.
The 67-sphere geometry is reconstructed from each trial's frozen parameters;
analytic AABB clearance is an independent sampled-state audit, not a mesh or
continuous collision certificate.
"""
import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

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
    gaps = sphere_box_gaps(body, center, size)
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
             'box_runtime.json', 'trajectories.json', 'events.json', 'active_arm_cycles.jsonl']
    metrics['source_sha256'] = {name: sha(path / name) for name in names if (path / name).is_file()}
    return dict(path=path, result=result, config=config, scene=scene, runtime=runtime, state=state,
                q=q, base=base, body=body, origin=float(origin), t=state['t'] - origin,
                appearance=None if appearance is None else float(appearance - origin),
                center=center, size=size, arm_gap=arm_gap, base_gap=base_gap,
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
    fig.suptitle('行驶中机械臂前方突然出现长方体：数值闭环记录\n'
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
    fig, axes = plt.subplots(2, len(trials), figsize=(9.9, 6.2), dpi=100, squeeze=False)
    fig.subplots_adjust(left=.055, right=.99, top=.87, bottom=.09, wspace=.19, hspace=.36)
    xmin = min(float(t['body'][..., 0].min()) - .13 for t in trials)
    xmax = max(float(t['body'][..., 0].max()) + .13 for t in trials)
    ymin = min(-.6, *(float(t['body'][..., 1].min()) - .15 for t in trials))
    ymax = max(.6, *(float(t['body'][..., 1].max()) + .15 for t in trials))
    zmax = max(1.5, *(float(t['body'][..., 2].max()) + .15 for t in trials))
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
        note = top.text(.01, -.30, '', transform=top.transAxes, fontsize=8, va='top')
        groups.append((balls, rects, trail, note))
    end = max(max(0., float(t['t'][-1])) for t in trials)
    times = np.linspace(0., end, min(max_frames, max(2, int(end/.3) + 1)))
    frames = []
    for elapsed in times:
        fig.suptitle('实测状态回放 %.2f s · 上侧视 / 下俯视 · 2 倍速\n'
                     '蓝：固定结构；橙：机械臂；红：高处长方体；曲线：实际底盘轨迹' % elapsed, fontsize=11)
        for trial, (balls, rects, trail, note) in zip(trials, groups):
            k = at(trial['t'], elapsed)
            for sphere, pair in zip(trial['body'][k], balls):
                pair[0].center = sphere[[0, 2]]; pair[1].center = sphere[[0, 1]]
                pair[0].radius = pair[1].radius = sphere[3]
            visible = trial['appearance'] is not None and trial['t'][k] >= trial['appearance']
            for rect in rects:
                rect.set_visible(visible)
            trail.set_data(trial['base'][:k+1, 0], trial['base'][:k+1, 1])
            ended = elapsed > trial['t'][-1] + .001
            note.set_text('v=%.2f m/s；参考束=%d\n%s' %
                (trial['state']['v'][k], trial['reference_count'][k], result_label(trial) if ended else '实测 t=%.2f s' % trial['t'][k]))
        fig.canvas.draw()
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()).convert('P', palette=Image.ADAPTIVE, colors=96))
    plt.close(fig)
    duration = max(20, int(round(1000. * end / max(1, len(times)-1) / 2.)))
    frames[0].save(out / 'comparison.gif', save_all=True, append_images=frames[1:], duration=duration,
                   loop=0, optimize=False, disposal=2)
    for frame in frames:
        frame.close()
    return dict(frames=len(times), width=990, playback_speed=2, ended_panels_frozen=True)


def packed(array):
    return base64.b64encode(np.asarray(array, dtype='<f4').tobytes()).decode('ascii')


def make_html(trials, out):
    data = []
    for trial in trials:
        candidates = np.flatnonzero(trial['t'] >= -1e-9)
        if not len(candidates):
            candidates = np.arange(len(trial['t']))
        stride = max(1, int(math.ceil(len(candidates)/600.)))
        ids = np.unique(np.r_[candidates[::stride], candidates[-1]])
        # Preserve the nearest recorded states on both sides of appearance.
        if trial['appearance'] is not None:
            k = at(trial['t'], trial['appearance'])
            ids = np.unique(np.r_[ids, k, min(k+1, len(trial['t'])-1)])
        data.append(dict(label=trial['metrics']['label'], status=result_label(trial), meta=trial['metrics'],
             appearance=trial['appearance'], center=trial['center'], size=trial['size'], goal=trial['scene']['goal'],
             t=packed(trial['t'][ids]), body=packed(trial['body'][ids]), q=packed(trial['q'][ids]),
             base=packed(trial['base'][ids]), v=packed(trial['state']['v'][ids]), refs=packed(trial['reference_count'][ids])))
    html = HTML.replace('__DATA__', json.dumps(safe(data), ensure_ascii=False, separators=(',', ':')).replace('</', '<\\/'))
    (out / 'comparison.html').write_text(html)


HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>机械臂前方突现长方体 · 仿真对照</title><style>
*{box-sizing:border-box}body{margin:0;background:#edf2f5;color:#213448;font:14px system-ui,sans-serif}header{background:white;padding:20px 24px}h1{font-size:24px;margin:0 0 8px}p{color:#607181;line-height:1.65;margin:6px 0}.controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:14px 24px;background:white;position:sticky;top:0;z-index:2;border-bottom:1px solid #d8e2e9}button,select{padding:8px 12px;border:1px solid #c3d2dd;border-radius:7px;background:white;font:inherit;color:#244356;cursor:pointer}button.active,#play{background:#17627a;color:white}input{flex:1;min-width:160px;accent-color:#17627a}.views{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px;padding:18px}.panel{background:white;border:1px solid #dce5eb;border-radius:12px;overflow:hidden}h2{font-size:18px;margin:16px 16px 5px}.status{margin:0 16px 10px;color:#567183;font-size:12px}.stats{padding:0 16px 12px;color:#526a7c;font-size:12px;line-height:1.8}.live{display:flex;justify-content:space-between;padding:12px 15px;border-top:1px solid #e6ecf0;font-variant-numeric:tabular-nums;font-size:12px}canvas{display:block;width:100%;height:410px;background:#fafcfd}.note,details{padding:0 24px 20px;color:#5f7282;font-size:12px;line-height:1.8}pre{white-space:pre-wrap;overflow-wrap:anywhere}#time{font-variant-numeric:tabular-nums;min-width:100px}@media(max-width:1000px){.views{grid-template-columns:1fr}canvas{height:420px}} </style>
<header><h1>行驶中，机械臂前方 1 m 突然出现长方体</h1><p>同一场景三组数值闭环：REMANI、REMANI＋MPPI、保留初始参考而冻结慢层的 MPPI。按各轮实际起步时刻对齐，保留失败与终止结果。</p></header>
<div class="controls"><button id="play">▶ 播放</button><button id="reset">回到起点</button><span id="time"></span><input id="slider" type="range" min="0" value="0" step=".02"><select id="speed"><option value=".5">0.5×</option><option value="1">1×</option><option selected value="2">2×</option><option value="4">4×</option></select><button class="view active" data-view="iso">斜视</button><button class="view" data-view="side">侧视</button><button class="view" data-view="top">俯视</button><button id="appear">障碍出现时刻</button></div><div class="views" id="panels"></div>
<div class="note">蓝色：47 个固定结构球；橙色：20 个机械臂球；红色：真实场景长方体；地面曲线：已执行底盘轨迹。所有姿态来自 state.csv，按冻结参数执行 native DH 正运动学重建，画面为球模型数值仿真。回放选择当前时刻之前的最新显示帧，不生成候选或后续运动；结束后冻结。参考束为收到并去重的原生消息，不等同优化调用次数。三维几何属于原生球模型，不代表 URDF 网格精确接触或实车测试。</div><details><summary>实际结果及源文件 SHA256</summary><pre id="sources"></pre></details><script type="application/json" id="data">__DATA__</script><script>
'use strict';const $=id=>document.getElementById(id);function unpack(s){const b=atob(s),a=new Uint8Array(b.length);for(let i=0;i<b.length;i++)a[i]=b.charCodeAt(i);return new Float32Array(a.buffer)}const raw=JSON.parse($('data').textContent),trials=raw.map(r=>({...r,t:unpack(r.t),body:unpack(r.body),base:unpack(r.base),q:unpack(r.q),v:unpack(r.v),refs:unpack(r.refs)}));let time=0,playing=false,last=0,view='iso';const end=Math.max(...trials.map(t=>t.t[t.t.length-1]));$('slider').max=end;$('sources').textContent=JSON.stringify(raw.map(t=>t.meta),null,2);
const fmt=(x,n=1)=>x===null?'未出现/未记录':Number(x).toFixed(n);trials.forEach((t,i)=>{$('panels').insertAdjacentHTML('beforeend',`<section class="panel"><h2>${t.label}</h2><div class="status">${t.status}</div><div class="stats">全程最大横移 ${fmt(t.meta.maximum_lateral_offset_m*100)} cm · 走过 ${fmt(t.meta.actual_base_path_length_m,3)} m<br>最大关节变化 ${fmt(t.meta.maximum_joint_change_rad*180/Math.PI)}° · 障碍后新参考 ${t.meta.unique_native_reference_bundles_after_box} 束</div><canvas id="c${i}"></canvas><div class="live"><span id="lv${i}"></span><span id="lr${i}"></span></div></section>`);t.canvas=$('c'+i)});
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];let all=[];for(const t of trials){for(let k=0;k<t.t.length;k++)for(let s=0;s<67;s++){let j=k*268+s*4;all.push([t.body[j],t.body[j+1],t.body[j+2]])}if(t.appearance!==null)for(let x of [-1,1])for(let y of [-1,1])for(let z of [-1,1])all.push(t.center.map((c,i)=>c+[x,y,z][i]*t.size[i]/2));all.push([t.goal[0],t.goal[1],0])}const lo=[0,1,2].map(a=>all.reduce((v,p)=>Math.min(v,p[a]),Infinity)-.15),hi=[0,1,2].map(a=>all.reduce((v,p)=>Math.max(v,p[a]),-Infinity)+.15);lo[2]=0;all=null;
function camera(w,h){let az=view==='iso'?-.36:0,el=view==='iso'?.60:view==='top'?Math.PI/2:0,R=[Math.cos(az),Math.sin(az),0],U=[-Math.sin(az)*Math.sin(el),Math.cos(az)*Math.sin(el),Math.cos(el)],D=[Math.sin(az)*Math.cos(el),-Math.cos(az)*Math.cos(el),Math.sin(el)],p=[];for(let x of [lo[0],hi[0]])for(let y of [lo[1],hi[1]])for(let z of [lo[2],hi[2]])p.push([dot([x,y,z],R),dot([x,y,z],U)]);let xl=Math.min(...p.map(a=>a[0])),xh=Math.max(...p.map(a=>a[0])),yl=Math.min(...p.map(a=>a[1])),yh=Math.max(...p.map(a=>a[1])),scale=Math.min((w-40)/(xh-xl),(h-60)/(yh-yl));return{scale,p:v=>[w/2+(dot(v,R)-(xl+xh)/2)*scale,h/2-(dot(v,U)-(yl+yh)/2)*scale],d:v=>dot(v,D)}}
function indexAt(a,s){let l=0,r=a.length;while(l<r){let k=(l+r)>>1;if(a[k]<=s)l=k+1;else r=k}return Math.max(0,l-1)}
function draw(t,i){let c=t.canvas,w=c.clientWidth,h=c.clientHeight,dpr=Math.min(devicePixelRatio||1,2);if(c.width!==Math.round(w*dpr)||c.height!==Math.round(h*dpr)){c.width=Math.round(w*dpr);c.height=Math.round(h*dpr)}let g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,w,h);let cam=camera(w,h),p=cam.p,k=indexAt(t.t,time),objects=[];function line(a,b,color,width=1,dash=[]){g.beginPath();g.setLineDash(dash);g.strokeStyle=color;g.lineWidth=width;g.moveTo(...p(a));g.lineTo(...p(b));g.stroke();g.setLineDash([])}for(let x=Math.floor(lo[0]*2)/2;x<=hi[0];x+=.5)line([x,lo[1],0],[x,hi[1],0],'#e0e8ee');for(let y=Math.floor(lo[1]*2)/2;y<=hi[1];y+=.5)line([lo[0],y,0],[hi[0],y,0],'#e0e8ee');for(let n=1;n<=k;n++)line([t.base[(n-1)*3],t.base[(n-1)*3+1],.008],[t.base[n*3],t.base[n*3+1],.008],'#2f6c8b',2);let goal=p([...t.goal,0]);g.strokeStyle='#418768';g.lineWidth=2;g.beginPath();g.arc(...goal,5,0,Math.PI*2);g.stroke();
for(let s=0;s<67;s++){let j=k*268+s*4,pos=[t.body[j],t.body[j+1],t.body[j+2]];objects.push({sphere:true,pos,r:t.body[j+3],arm:s>=47,depth:cam.d(pos)})}
let visible=t.appearance!==null&&t.t[k]>=t.appearance;if(visible){let corners=[];for(let x of [-1,1])for(let y of [-1,1])for(let z of [-1,1])corners.push(t.center.map((c,i)=>c+[x,y,z][i]*t.size[i]/2));for(let ids of [[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]){let poly=ids.map(j=>corners[j]),center=[0,1,2].map(a=>poly.reduce((v,q)=>v+q[a],0)/4);objects.push({poly,depth:cam.d(center)})}}
objects.sort((a,b)=>a.depth-b.depth);for(let o of objects){if(o.sphere){let xy=p(o.pos),r=Math.max(.6,o.r*cam.scale),grad=g.createRadialGradient(xy[0]-r*.3,xy[1]-r*.4,.02*r,xy[0],xy[1],r);grad.addColorStop(0,o.arm?'#ffd59c':'#99c7e6');grad.addColorStop(1,o.arm?'#cb7428':'#367daa');g.fillStyle=grad;g.beginPath();g.arc(...xy,r,0,Math.PI*2);g.fill();g.strokeStyle='#294b5d33';g.lineWidth=.5;g.stroke()}else{g.beginPath();o.poly.forEach((v,j)=>j?g.lineTo(...p(v)):g.moveTo(...p(v)));g.closePath();g.fillStyle='#d8545ab5';g.fill();g.strokeStyle='#ad424b';g.lineWidth=.7;g.stroke()}}g.font='12px system-ui';g.fillStyle='#647f90';g.fillText(view==='top'?'俯视 x–y':view==='side'?'侧视 x–z':'三维正交斜视',13,23);g.fillText(visible?'长方体已出现':t.appearance===null?'本轮未触发障碍':'障碍将在 '+t.appearance.toFixed(2)+' s 出现',13,h-14);if(time>t.t[t.t.length-1]+.02){g.fillStyle='#815f46';g.fillText('本轮已结束 · 姿态冻结',13,43)}$('lv'+i).textContent='帧 '+t.t[k].toFixed(2)+' s · v='+t.v[k].toFixed(2)+' m/s';$('lr'+i).textContent='y='+((t.base[k*3+1]-t.base[1])*100).toFixed(1)+' cm · 参考 '+Math.round(t.refs[k])+' 束'}
function render(){trials.forEach(draw);$('time').textContent=time.toFixed(2)+' / '+end.toFixed(2)+' s';$('slider').value=time}function setPlaying(v){playing=v;$('play').textContent=v?'❚❚ 暂停':'▶ 播放'}$('play').onclick=()=>{if(time>=end)time=0;setPlaying(!playing);render()};$('reset').onclick=()=>{time=0;setPlaying(false);render()};$('slider').oninput=e=>{time=Number(e.target.value);setPlaying(false);render()};$('appear').onclick=()=>{let times=trials.filter(t=>t.appearance!==null).map(t=>t.appearance);time=times.length?Math.min(...times):0;setPlaying(false);render()};document.querySelectorAll('.view').forEach(b=>b.onclick=()=>{view=b.dataset.view;document.querySelectorAll('.view').forEach(x=>x.classList.toggle('active',x===b));render()});window.addEventListener('resize',render);function tick(now){let dt=last?Math.min((now-last)/1000,.2):0;last=now;if(playing){time=Math.min(end,time+dt*Number($('speed').value));if(time>=end)setPlaying(false);render()}requestAnimationFrame(tick)}render();requestAnimationFrame(tick);window.REPLAY={trials,setTime(s){time=Math.max(0,Math.min(end,s));setPlaying(false);render()}};
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--max-frames', type=int, default=105)
    args = parser.parse_args()
    if not 2 <= args.max_frames <= 110:
        parser.error('--max-frames must be between 2 and 110')
    case = args.case.resolve()
    trials = [load_trial(case / name) for name in LABELS if (case / name / 'state.csv').is_file()]
    if not trials:
        raise ValueError('No completed state.csv trials under ' + str(case))
    out = case / 'report'; out.mkdir(exist_ok=True)
    plot_comparison(trials, out)
    animation = make_gif(trials, out, args.max_frames)
    make_html(trials, out)
    summary = dict(schema='arm_box_report_v1', trials=[t['metrics'] for t in trials], gif=animation,
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

#!/usr/bin/env python3
"""Multi-actor V7 SIL. Never starts ROS or publishes hardware commands."""
import argparse
import ctypes as ct
import hashlib
import json
import math
import shutil
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from test_mppi_v2 import Controller, DP, ptr

HERE = Path(__file__).resolve().parent
# All actors appear at t=2 s, follow independent CV tracks, and never disappear.
# Entries: x/y at appearance, vx/vy, physical radius, dynamic flag.
A = [1.55, -1.65, 0., .26, .23, 1.]
B = [2.40, 1.90, 0., -.20, .23, 1.]
C = [2.75, -2.30, 0., .17, .20, 1.]
D = [4.30, .30, -.18, 0., .20, 1.]
CASES = {'01_single': [A], '02_two_crossing': [A, B],
         '03_three_sequential': [A, B, C], '04_four_dense': [A, B, C, D]}
NAMES = {'01_single': '单障碍对照', '02_two_crossing': '双障碍交叉',
         '03_three_sequential': '三障碍连续交会', '04_four_dense': '四障碍密集交会'}


def obstacles(case, t):
    if t < 2.: return np.empty((0, 6))
    a = np.array(CASES[case], dtype=float)
    a[:, :2] += (t - 2.) * a[:, 2:4]
    return a


def clearance(s, obs, body):
    c, sn = np.cos(s[2]), np.sin(s[2])
    centers = body[:, :2] @ np.array([[c, sn], [-sn, c]]) + s[:2]
    gap = float(np.min(2. - np.abs(centers[:, 1]) - body[:, 2]))
    if len(obs):
        d = np.linalg.norm(centers[:, None, :] - obs[None, :, :2], axis=2)
        gap = min(gap, float(np.min(d - body[:, None, 2] - obs[None, :, 4])))
    return gap


def plant(c, s, u, case, t, body):
    """Same lagged plant as Controller.plant; audit geometry every 5 ms."""
    u = u.copy(); h = c['track'] / 2
    u /= max(1, abs(u[0])/c['vmax'], abs(u[1])/c['wmax'],
             abs(u[0]-h*u[1])/c['wheel_v'], abs(u[0]+h*u[1])/c['wheel_v'])
    gap = clearance(s, obstacles(case, t), body)
    for j in range(10):
        a = (u - s[3:]) / .12
        a /= max(1, abs(a[0])/c['accel'], abs(a[1])/c['angular_accel'],
                 abs(a[0]-h*a[1])/c['wheel_a'], abs(a[0]+h*a[1])/c['wheel_a'])
        new = s[3:] + .005*a; v, w = (s[3:] + new)/2
        theta = s[2] + .0025*w
        s[:2] += .005*v*np.array([np.cos(theta), np.sin(theta)])
        s[2] = math.atan2(math.sin(s[2]+.005*w), math.cos(s[2]+.005*w)); s[3:] = new
        gap = min(gap, clearance(s, obstacles(case, t+(j+1)*.005), body))
    return gap


def search_snapshot(ctrl, t):
    n = ctrl.lib.online_search_trace(ctrl.handle, None, 0)
    assert 0 <= n < 20000, n
    buf = np.zeros((n, 8))
    assert ctrl.lib.online_search_trace(ctrl.handle, ptr(buf), n) == n
    candidates = []
    for ident in np.unique(buf[:, 0]):
        rows = buf[buf[:, 0] == ident]
        candidates.append({'id': int(ident), 'cost': float(rows[0, 4]),
                           'gap': float(rows[0, 5]), 'feasible': bool(rows[0, 6]),
                           'selected': bool(rows[0, 7]), 'points': rows[:, 1:4].tolist()})
    return {'t': t, 'candidates': candidates}


def trial(build, fixture, case, seed, capture=True):
    ctrl = Controller(build, fixture, seed)
    body = np.atleast_2d(np.loadtxt(fixture/'geometry.csv', delimiter=','))
    lib = ctrl.lib
    lib.online_set_classified_static.argtypes = [ct.c_void_p, DP, ct.c_int, ct.c_int]
    lib.online_decision_debug.argtypes = [ct.c_void_p, DP]
    lib.online_avoidance_debug.argtypes = [ct.c_void_p, DP]
    has_trace = hasattr(lib, 'online_search_trace')
    if has_trace:
        lib.online_search_enable.argtypes = [ct.c_void_p, ct.c_int]
        lib.online_search_trace.argtypes = [ct.c_void_p, DP, ct.c_int]
        lib.online_search_trace.restype = ct.c_int
    xx, yy = np.meshgrid(-7.5+.05*np.arange(300), -5.+.05*np.arange(200))
    static = np.ascontiguousarray(np.maximum(0., 2.-abs(yy)-np.sqrt(2)*.05))
    x = np.linspace(0, 3, 301)
    ref = np.ascontiguousarray(np.column_stack((x, x*0, x*0, np.minimum(.18, .8*(3-x)))))
    state = np.zeros(5); trace = []; searches = []; export_ms = []; reached = False; termination = 'timeout'
    try:
        for k in range(900):
            t = k*.05; obs = obstacles(case, t)
            field = 2.-abs(yy)
            for o in obs: field = np.minimum(field, np.hypot(xx-o[0], yy-o[1])-o[4])
            ctrl.field = np.ascontiguousarray(np.maximum(0., field-np.sqrt(2)*.05))
            lib.online_set_classified_static(ctrl.handle, ptr(static), 300, 200)
            record = capture and k % 4 == 0 and has_trace
            if has_trace: lib.online_search_enable(ctrl.handle, int(record))
            measured = obs.copy(); measured[:, :2] -= .04*measured[:, 2:4]
            before = state.copy(); command = ctrl.step(state, ref, measured)
            decision = np.zeros(12); safety = np.zeros(8)
            lib.online_decision_debug(ctrl.handle, ptr(decision)); lib.online_avoidance_debug(ctrl.handle, ptr(safety))
            if record:
                start = time.perf_counter(); searches.append(search_snapshot(ctrl, t))
                export_ms.append(1000*(time.perf_counter()-start))
            gap = plant(ctrl.c, state, command, case, t, body)
            trace.append({'t': t, 'state': before.tolist(), 'next_state': state.tolist(),
                          'command': command.tolist(), 'obstacles': obs.tolist(), 'gap': gap,
                          'decision': decision.tolist(), 'unverified_brake': bool(safety[7]),
                          'kernel_ms': ctrl.ms[-1]})
            if gap <= 0: termination = 'collision'; break
            if np.linalg.norm(state[:2]-ref[-1, :2]) < .05 and abs(state[2]) < .1 and abs(state[3]) < .05:
                reached = True; termination = 'reached'; break
    finally: ctrl.close()
    a = np.array([row['next_state'] for row in trace]); live = (np.arange(len(a))*.05 > 1)&(np.linalg.norm(a[:, :2]-[3, 0], axis=1) > .15)
    gaps = np.array([row['gap'] for row in trace])
    m = {'reached': reached, 'termination': termination, 'duration_s': len(trace)*.05,
         'path_m': float(np.linalg.norm(np.diff(np.vstack((np.zeros((1, 2)), a[:, :2])), axis=0), axis=1).sum()),
         'min_gap_m': float(gaps.min()), 'margin_violation_cycles': int(np.sum(gaps < .1-1e-6)),
         'low_speed_s': float(np.sum(live&(abs(a[:, 3]) < .02))*.05),
         'reverse_m': float(np.maximum(0, -a[:, 3]).sum()*.05),
         'max_lateral_m': float(abs(a[:, 1]).max()),
         'unverified_brake_cycles': sum(row['unverified_brake'] for row in trace),
         'kernel_p95_ms': float(np.percentile(ctrl.ms, 95)), 'kernel_max_ms': max(ctrl.ms),
         'kernel_over_50ms': sum(ms > 50 for ms in ctrl.ms),
         'trace_export_p95_ms': float(np.percentile(export_ms, 95)) if export_ms else 0.}
    return {'case': case, 'seed': seed, 'build': str(build.resolve()), 'body': body.tolist(),
            'metrics': m, 'trace': trace, 'searches': searches}


def frame(r, index, search=True):
    row = r['trace'][index]; t = row['t']; s = row['state']
    im = Image.new('RGB', (800, 640), 'white'); d = ImageDraw.Draw(im)
    font = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 19)
    small = ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 15)
    def xy(x, y): return (80+(x+.6)*135, 350-y*110)
    # Draw world geometry on a clipped layer so off-screen persistent actors
    # cannot overwrite captions. Dynamic tracks are never removed from input.
    layer = Image.new('RGB', im.size, 'white'); g = ImageDraw.Draw(layer)
    for y in (-2, 2): g.line([xy(-.5, y), xy(4.1, y)], fill='#dddddd', width=2)
    g.line([xy(0, 0), xy(3, 0)], fill='#b0b0b0', width=2)
    if search:
        snapshots = [v for v in r['searches'] if v['t'] <= t+1e-8]
        snap = snapshots[-1] if snapshots else {'candidates': []}
        for selected in (False, True):
            for c in snap['candidates']:
                if c['selected'] != selected: continue
                pts = [xy(p[1], p[2]) for p in c['points']]
                if len(pts) > 1:
                    color = '#23934c' if selected else '#b9cddd' if c['feasible'] else '#e3c4be'
                    g.line(pts, fill=color, width=4 if selected else 1)
    for j, o in enumerate(row['obstacles']):
        cx, cy = xy(o[0], o[1]); rx = 135*o[4]; ry = 110*o[4]
        g.ellipse((cx-rx, cy-ry, cx+rx, cy+ry), fill='#b5b5b5', outline='#777777')
        end = xy(o[0]+1.5*o[2], o[1]+1.5*o[3]); g.line([(cx, cy), end], fill='#666666', width=2)
        angle = math.atan2(end[1]-cy, end[0]-cx)
        g.line([(end[0]-8*math.cos(angle-.5), end[1]-8*math.sin(angle-.5)), end,
                (end[0]-8*math.cos(angle+.5), end[1]-8*math.sin(angle+.5))], fill='#666666', width=2)
        g.text((cx-5, cy-11), str(j+1), fill='white', font=small)
    history = [xy(v['state'][0], v['state'][1]) for v in r['trace'][:index+1:2]]
    if len(history) > 1: g.line(history, fill='#16689a', width=3)
    for bx, by, rad in r['body']:
        px = s[0]+math.cos(s[2])*bx-math.sin(s[2])*by; py = s[1]+math.sin(s[2])*bx+math.cos(s[2])*by
        cx, cy = xy(px, py); g.ellipse((cx-135*rad, cy-110*rad, cx+135*rad, cy+110*rad), outline='#16689a', width=2)
    g.line([xy(s[0], s[1]), xy(s[0]+.3*math.cos(s[2]), s[1]+.3*math.sin(s[2]))], fill='#16689a', width=3)
    cx, cy = xy(3, 0); g.ellipse((cx-4, cy-4, cx+4, cy+4), fill='#333333')
    im.paste(layer.crop((30, 110, 770, 578)), (30, 110)); d = ImageDraw.Draw(im)
    d.rectangle((30, 110, 770, 578), outline='#dddddd')
    d.text((30, 12), f'V7 | {NAMES[r["case"]]} | t={t:.1f}s | seed={r["seed"]}', fill='#333333', font=font)
    d.text((30, 43), '蓝：已执行轨迹   绿：选中预测   灰：障碍物与 CV 方向', fill='#555555', font=small)
    d.text((30, 68), '浅蓝：可行候选   浅红：不可行候选（V7 外层实际评估）' if search else '原参考保持不变；障碍物出画面后仍持续被观测', fill='#666666', font=small)
    mode = ['跟踪', '调速', '绕行', '回接', '未验证制动'][int(row['decision'][0])]
    note = f'{mode}  v={s[3]:.2f}m/s  区间最小净距={row["gap"]:.3f}m'
    if index == len(r['trace'])-1: note += '  '+r['metrics']['termination']
    d.text((30, 587), note, fill='#333333', font=small)
    d.text((30, 611), '固定参考 / 理想分层 / 非 Gazebo、非实车 / 2 倍速', fill='#777777', font=small)
    return im


def render_case(dest, records):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    r = min(records, key=lambda v: v['seed'])
    ids = list(range(0, len(r['trace']), 4))
    if ids[-1] != len(r['trace'])-1: ids.append(len(r['trace'])-1)
    for show, name in ((True, 'search.gif'), (False, 'motion.gif')):
        frames = [frame(r, i, show) for i in ids]
        frames[0].save(dest/'gif'/name, save_all=True, append_images=frames[1:], duration=100, loop=0, disposal=2)
    # Fixed rule: densest search snapshot; not handpicked to hide failures.
    snap = max(r['searches'], key=lambda v: len(v['candidates']))
    idx = min(len(r['trace'])-1, round(snap['t']/.05))
    frame(r, idx).save(dest/'plots'/'search_snapshot.png')
    fig, ax = plt.subplots(2, 2, figsize=(10, 7))
    for record in records:
        rows = record['trace']; t = np.array([v['t'] for v in rows]); s = np.array([v['state'] for v in rows])
        label = f'seed {record["seed"]}'
        ax[0, 0].plot(s[:, 0], s[:, 1], label=label)
        ax[0, 1].plot(t, s[:, 3], label=label)
        ax[1, 0].plot(t, [v['gap'] for v in rows], label=label)
        ax[1, 1].plot(t, [v['kernel_ms'] for v in rows], label=label, alpha=.65)
    ax[0, 0].plot([0, 3], [0, 0], '--', color='.7'); ax[0, 0].set(title='Executed paths', xlabel='x / m', ylabel='y / m', aspect='equal')
    ax[0, 1].set(title='Actual body speed', xlabel='time / s', ylabel='m/s')
    ax[1, 0].axhline(.1, ls='--', color='.6'); ax[1, 0].set(title='Body clearance (5 ms audit)', xlabel='time / s', ylabel='m')
    ax[1, 1].axhline(50, ls='--', color='.6'); ax[1, 1].set(title='Kernel time, not full ROS latency', xlabel='time / s', ylabel='ms')
    for a in ax.flat: a.legend(); a.grid(alpha=.15)
    fig.suptitle(r['case']+' | fixed reference / ideal CV input'); fig.tight_layout()
    fig.savefig(dest/'plots'/'metrics.png', dpi=140); plt.close(fig)


def main():
    p = argparse.ArgumentParser(); p.add_argument('--build', type=Path, required=True)
    p.add_argument('--original', type=Path, required=True); p.add_argument('--fixture', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True); p.add_argument('--seeds', type=int, nargs='+', default=[1, 2, 3])
    a = p.parse_args(); a.out.mkdir(parents=True, exist_ok=False)
    (a.out/'source_snapshot').mkdir(); (a.out/'fixture').mkdir()
    for name in ('suite_multi_dynamic.py', 'mppi_v7.hpp', 'build_mppi_v2.py', 'test_mppi_v2.py'):
        shutil.copy2(HERE/name, a.out/'source_snapshot'/name)
    for name in ('geometry.csv', 'limits.json'): shutil.copy2(a.fixture/name, a.out/'fixture'/name)
    manifest = {'scope': 'fixed-reference local SIL, ideal CV and classification; no native replanner/ROS/robot',
                'cases': CASES, 'appearance_s': 2., 'persistent': True, 'seeds': a.seeds,
                'build': json.loads((a.build/'build_manifest.json').read_text()),
                'original_build': str(a.original.resolve()),
                'library_sha256': hashlib.sha256((a.build/'build/libonline_mppi.so').read_bytes()).hexdigest()}
    manifest['source_snapshot_sha256'] = {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in (a.out/'source_snapshot').iterdir()}
    (a.out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    summaries = {}; equivalence = {}
    for case in CASES:
        dest = a.out/case
        for sub in ('data', 'gif', 'plots'): (dest/sub).mkdir(parents=True)
        records = []
        for seed in a.seeds:
            r = trial(a.build, a.fixture, case, seed)
            (dest/'data'/f'seed{seed}.json').write_text(json.dumps(r, separators=(',', ':'))+'\n')
            records.append(r); print(case, seed, r['metrics'], flush=True)
        # Full closed-loop command/state equivalence to uninstrumented V7.
        original = trial(a.original, a.fixture, case, a.seeds[0], capture=False)
        (dest/'data'/'original_v7_seed1.json').write_text(json.dumps(original, separators=(',', ':'))+'\n')
        def vector(r): return np.array([v['state']+v['next_state']+v['command'] for v in r['trace']])
        before, after = vector(original), vector(records[0])
        same = before.shape == after.shape and np.array_equal(before, after)
        equivalence[case] = {'identical_commands_and_states': same, 'original_metrics': original['metrics']}
        (a.out/'instrumentation_equivalence.json').write_text(json.dumps(equivalence, indent=2)+'\n')
        assert same, 'Trace instrumentation changed controller behavior: '+case
        render_case(dest, records)
        summaries[case] = [dict(seed=r['seed'], **r['metrics']) for r in records]
        (dest/'summary.json').write_text(json.dumps(summaries[case], indent=2)+'\n')
        (a.out/'summary.json').write_text(json.dumps(summaries, indent=2)+'\n')
    native = manifest['build']['production_sources_sha256']
    changed = [f for f, h in native.items() if hashlib.sha256((HERE.parents[1]/f).read_bytes()).hexdigest() != h]
    assert not changed, changed
    audit = {'production_files_unchanged': len(native), 'instrumentation_equivalent': all(v['identical_commands_and_states'] for v in equivalence.values()),
             'trials': sum(len(v) for v in summaries.values()), 'gifs': len(list(a.out.glob('*/gif/*.gif')))}
    (a.out/'audit.json').write_text(json.dumps(audit, indent=2)+'\n'); print('AUDIT', audit, flush=True)


if __name__ == '__main__': main()

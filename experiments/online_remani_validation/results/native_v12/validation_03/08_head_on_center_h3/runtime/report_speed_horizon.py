#!/usr/bin/env python3
"""Plots and classified GIFs from all predeclared speed/horizon trials."""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from suite_speed_horizon import CASES, audit_record
from suite_v8 import sha, HERE

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'


def panel(r, index, search=False):
    row = r['trace'][index]; terminal = index == len(r['trace'])-1
    state = row['next_state'] if terminal else row['state']
    t = row['t'] + (.05 if terminal else 0.)
    obs = np.array(row['obstacles']).reshape(-1, 6).copy()
    if terminal: obs[:, :2] += .05*obs[:, 2:4]
    h = .1*int(np.floor((r['config']['validity_s']-.05)/.1))
    image = Image.new('RGB', (540, 440), 'white'); d = ImageDraw.Draw(image)
    font = ImageFont.truetype(FONT, 18); small = ImageFont.truetype(FONT, 14)
    name = {'base': '原速', 'fast': '提速', 'capacity': '仅提高上限'}[r['config']['profile']]
    d.text((16, 10), f'{name} | 轨迹展开 {h:.1f}s | t={t:.1f}s', font=font, fill='#333333')
    d.text((16, 39), '蓝：实际运动   灰：障碍物及对应时域CV预测', font=small, fill='#666666')
    layer = Image.new('RGB', image.size, 'white'); g = ImageDraw.Draw(layer)
    def xy(x, y): return (64+84*x, 228-84*y)
    for y in (-2, 2): g.line([xy(-.5, y), xy(5.2, y)], fill='#dddddd')
    g.line([xy(0, 0), xy(3, 0)], fill='#bbbbbb')
    if search:
        snaps = [s for s in r['searches'] if s['t'] <= row['t']+1e-8]
        if snaps:
            for c in sorted(snaps[-1]['candidates'], key=lambda c: c['selected']):
                points = [xy(p[1], p[2]) for p in c['points']]
                if len(points) > 1:
                    g.line(points, fill='#21964b' if c['selected'] else '#bacddd' if c['feasible'] else '#e5caca', width=3 if c['selected'] else 1)
    for o in obs:
        center = xy(*o[:2]); end = xy(*(o[:2]+h*o[2:4])); rad = 84*o[4]
        g.line([center, end], fill='#888888', width=2)
        angle = math.atan2(end[1]-center[1], end[0]-center[0])
        g.line([(end[0]-7*math.cos(angle-.5), end[1]-7*math.sin(angle-.5)), end,
                (end[0]-7*math.cos(angle+.5), end[1]-7*math.sin(angle+.5))], fill='#888888', width=2)
        g.ellipse((center[0]-rad, center[1]-rad, center[0]+rad, center[1]+rad), fill='#b1b1b1')
    points = [xy(*row['state'][:2]) for row in r['trace'][:index+1:2]] + [xy(*state[:2])]
    if len(points) > 1: g.line(points, fill='#1b789e', width=3)
    for bx, by, radius in r['body']:
        x = state[0]+math.cos(state[2])*bx-math.sin(state[2])*by
        y = state[1]+math.sin(state[2])*bx+math.cos(state[2])*by
        cx, cy = xy(x, y); rad = 84*radius
        g.ellipse((cx-rad, cy-rad, cx+rad, cy+rad), outline='#1b789e', width=2)
    g.line([xy(*state[:2]), xy(state[0]+.3*math.cos(state[2]), state[1]+.3*math.sin(state[2]))], fill='#1b789e', width=2)
    gx, gy = xy(3, 0); g.ellipse((gx-3, gy-3, gx+3, gy+3), fill='#333333')
    image.paste(layer.crop((15, 70, 525, 390)), (15, 70)); d = ImageDraw.Draw(image)
    d.rectangle((15, 70, 525, 390), outline='#dddddd')
    note = f'v={state[3]:.2f}m/s | 净距={row["gap"]:.3f}m'
    if terminal: note += ' | '+r['metrics']['termination']
    d.text((16, 395), note, font=small, fill='#444444')
    d.text((16, 419), '固定参考数值闭环；非实车；seed 1；2倍速', font=small, fill='#777777')
    return image


def animation(path, records, search=False):
    # Resume expensive derived rendering without re-running a control trial.
    # A partial/unreadable GIF is regenerated, never silently accepted.
    if path.exists():
        try:
            with Image.open(path) as im:
                assert im.n_frames > 1
                im.seek(im.n_frames-1); im.load()
            return
        except (OSError, AssertionError, EOFError):
            pass
    count = max(len(r['trace']) for r in records)
    ids = list(range(0, count, 4))
    if ids[-1] != count-1: ids.append(count-1)
    images = []
    for k in ids:
        canvas = Image.new('RGB', (540*len(records), 440), 'white')
        for n, r in enumerate(records): canvas.paste(panel(r, min(k, len(r['trace'])-1), search), (540*n, 0))
        # Keep palette frames, not all RGB frames, to bound GIF encoder memory.
        images.append(canvas.convert('P', palette=Image.ADAPTIVE, colors=128))
    images[0].save(path, save_all=True, append_images=images[1:], duration=100, loop=0, disposal=2)


def report(out):
    manifest = json.loads((out/'manifest.json').read_text())
    assert (out/'audit.json').exists(), 'Do not present an incomplete experiment as final'
    assert sha(Path(manifest['build'])/'build/libonline_mppi.so') == manifest['build_manifest']['mppi_library_sha256']
    for name, h in manifest['snapshots'].items(): assert sha(out/'source_snapshot'/name) == h
    for name, h in manifest['build_manifest']['production_sources_sha256'].items(): assert sha(HERE.parents[1]/name) == h
    summary = {}; raw_hashes = {}; audited = 0; failure_diagnostics = []
    (out/'comparison').mkdir(exist_ok=True)
    lines = ['# 速度与预测时域对照', '',
             '39次固定参考数值闭环；理想CV，非完整REMANI、Gazebo或实车。全部结果保留，GIF固定seed 1。', '',
             '原速：车体上限0.30、轮缘上限0.2296、参考速度0.18m/s；提速：0.35、0.32、0.24m/s。',
             '仅提高上限：使用提速上限，参考仍0.18m/s。加速度、角速度0.8rad/s、车体和0.1m余量不变。',
             'h2/h3/h4为CV有效期2/3/4秒，实际候选展开1.9/2.9/3.9秒，观测年龄40ms。', '',
             '|场景|配置|到达|碰撞|平均倒退/m|平均低速/s|平均终止/s|最小净距/m|内核P95均值/ms|>50ms周期|',
             '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for case in CASES:
        records = {}
        for key, cfg in manifest['configs'].items():
            if case not in cfg['cases']: continue
            assert json.loads((out/key/'fixture/limits.json').read_text()) == cfg['limits']
            assert sha(out/key/'fixture/geometry.csv') == sha(HERE/'results/mppi_v8/validation_03/fixture/geometry.csv')
            ms = []
            for seed in manifest['seeds']:
                path = out/key/case/'data'/f'seed{seed}.json'
                r = json.loads(path.read_text()); assert r['config'] == cfg
                assert r['case'] == case and r['seed'] == seed
                audit_record(r, cfg['limits'], cfg['validity_s']); audited += 1
                raw_hashes[str(path.relative_to(out))] = sha(path)
                states = np.array([x['next_state'] for x in r['trace']])
                gaps = np.array([x['gap'] for x in r['trace']])
                assert min(gaps) == r['metrics']['min_gap_m']
                assert int(np.sum(gaps < .1-1e-6)) == r['metrics']['margin_violation_cycles']
                assert bool(np.any(gaps <= 0)) == (r['metrics']['termination'] == 'collision')
                assert abs(float(np.maximum(0, -states[:,3]).sum()*.05)-r['metrics']['reverse_m']) < 1e-12
                measured = dict(r['metrics'])
                measured['peak_abs_speed_mps'] = float(abs(states[:, 3]).max())
                measured['all_low_speed_s'] = float(np.sum((np.arange(len(states))*.05>1)&(abs(states[:, 3])<.02))*.05)
                ms.append(measured)
                if r['metrics']['termination'] != 'reached' or r['metrics']['margin_violation_cycles']:
                    first = next((x for x in r['trace'] if x['unverified_brake']), None)
                    end = r['trace'][-1]
                    failure_diagnostics.append(dict(config=key, case=case, seed=seed,
                        termination=r['metrics']['termination'], min_gap_m=r['metrics']['min_gap_m'],
                        first_unverified=None if first is None else {k:first[k] for k in ('t','state','gap','recovery')},
                        final_state=end['next_state'], goal_distance_m=float(np.linalg.norm(states[-1,:2]-[3,0])),
                        all_low_speed_s=measured['all_low_speed_s'],
                        legacy_low_speed_excludes_goal_radius_m=.15))
                if seed == 1:
                    # Hundreds of thousands of Python point lists otherwise
                    # dominate memory when comparing all seven configurations.
                    for snap in r['searches']:
                        for c in snap['candidates']: c['points'] = np.asarray(c['points'], dtype=float)
                    records[key] = r
            mean = lambda k: float(np.mean([m[k] for m in ms]))
            v = dict(reached=sum(m['reached'] for m in ms), trials=len(ms),
                     collisions=sum(m['termination']=='collision' for m in ms),
                     reverse_m=mean('reverse_m'), low_speed_s=mean('low_speed_s'), duration_s=mean('duration_s'),
                     all_low_speed_s=mean('all_low_speed_s'), peak_abs_speed_mps=max(m['peak_abs_speed_mps'] for m in ms),
                     path_m=mean('path_m'), min_gap_m=min(m['min_gap_m'] for m in ms),
                     margin_violation_cycles=sum(m['margin_violation_cycles'] for m in ms),
                     unverified_brake_cycles=sum(m['unverified_brake_cycles'] for m in ms),
                     kernel_p95_ms=mean('kernel_p95_ms'), kernel_max_ms=max(m['kernel_max_ms'] for m in ms),
                     kernel_over_50ms=sum(m['kernel_over_50ms'] for m in ms),
                     export_p95_ms=ms[0]['trace_export_p95_ms'])
            summary.setdefault(case, {})[key] = v
            lines.append(f'|{case}|{key}|{v["reached"]}/3|{v["collisions"]}|{v["reverse_m"]:.3f}|{v["low_speed_s"]:.2f}|{v["duration_s"]:.2f}|{v["min_gap_m"]:.7f}|{v["kernel_p95_ms"]:.2f}|{v["kernel_over_50ms"]}|')
            dest = out/key/case; (dest/'gif').mkdir(exist_ok=True); (dest/'plots').mkdir(exist_ok=True)
            animation(dest/'gif/motion.gif', [records[key]])
            animation(dest/'gif/search.gif', [records[key]], search=True)
            snaps = records[key]['searches']
            if snaps:
                snap = max(snaps, key=lambda s: len(s['candidates']))
                panel(records[key], min(round(snap['t']/.05), len(records[key]['trace'])-1), True).save(dest/'plots/search.png')
            (dest/'README.md').write_text(f'# {key} / {case}\n\n[运动GIF](gif/motion.gif) · [候选GIF](gif/search.gif)\n\n实际候选：浅蓝可行、浅红不可行、绿色选中；灰箭头对应本组展开时域。\n')
            print('RENDER', case, key, flush=True)
        dest = out/'comparison'/case; dest.mkdir(exist_ok=True)
        for profile in ('base', 'fast'):
            animation(dest/f'{profile}_horizons.gif', [records[f'{profile}_h{h}'] for h in (2, 3, 4)])
        animation(dest/'speed_h3.gif', [records['base_h3'], records['fast_h3']])
        if 'capacity_h2' in records:
            animation(dest/'speed_capacity_h2.gif', [records['base_h2'], records['capacity_h2'], records['fast_h2']])
        fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), constrained_layout=True)
        for profile, color in (('base', '#247b9e'), ('fast', '#c17435')):
            for ax, metric, title, unit in zip(axes, ('reverse_m', 'all_low_speed_s', 'kernel_p95_ms'),
                    ('Reverse distance', 'Low-speed incl. goal area', 'Kernel time (P95)'), ('m', 's', 'ms')):
                values = [summary[case][f'{profile}_h{h}'][metric] for h in (2, 3, 4)]
                label = profile
                if summary[case][f'{profile}_h2']['reached'] < 3:
                    label += f' (1.9s: {summary[case][f"{profile}_h2"]["reached"]}/3 reached)'
                ax.plot([1.9, 2.9, 3.9], values, 'o-', label=label, color=color)
                ax.set(xlabel='rollout horizon / s', ylabel=unit, title=title, xticks=[1.9, 2.9, 3.9]); ax.grid(alpha=.2)
        axes[2].axhline(50, ls='--', color='#999999'); axes[0].legend()
        fig.suptitle(case+' | mean over 3 seeds | see table for failures', fontsize=11)
        fig.savefig(dest/'metrics.png', dpi=150); plt.close(fig)
    lines += ['', '## 动画与数据', '',
              '- [单迎面：原速下延长时域](comparison/05_head_on_only/base_horizons.gif)',
              '- [单迎面：提速后延长时域](comparison/05_head_on_only/fast_horizons.gif)',
              '- [单迎面：提高上限和提高巡航速度的区别](comparison/05_head_on_only/speed_capacity_h2.gif)',
              '- [四障碍：原速下延长时域](comparison/04_four_dense/base_horizons.gif)',
              '- [四障碍：提速后延长时域](comparison/04_four_dense/fast_horizons.gif)',
              '- [单迎面指标图](comparison/05_head_on_only/metrics.png) · [四障碍指标图](comparison/04_four_dense/metrics.png)', '',
              '每个配置/场景独立包含data、gif、plots。原始数据包括所有失败，终止时间不能把碰撞解释为更快到达。',
              '低速定义、净距采样及到达判据保持原V8；净距不足0.1m与实际碰撞分开统计。', '',
              '## 边界', '',
              '模型仍为差速约束，未启用麦克纳姆横移；无CBF-QP、无新增不确定性膨胀。',
              '候选原语速度与0.8秒阶段切换、代价系数保持不变；不是重新调优后的最快方案。',
              '底层采样器沿用按时域减少样本的预算策略；外层候选数量不变。局部栅格提案风险前瞻仍1.5秒，但完整候选检查已延长。',
              '更快巡航改变相遇时刻，不可将一组通过完全归因于脱困能力提高；capacity组用于初步区分。',
              '参考速度不是实际速度硬上限：capacity虽然参考不变，实际行驶也可能变快，不能称为同速度同相遇状态实验。',
              '表中低速沿用V8口径，排除起步1秒和目标0.15m范围；目标附近卡住可能不计入。指标图改用包含目标附近的低速时间（仍排除起步1秒），aggregates.json另列all_low_speed_s，failure_diagnostics.json保留失败末态。',
              '计时为内核，不含候选导出、点云和ROS链路；没有把实测计算时延施加给运动模型。',
              '因此即使不碰撞，也不等于满足20Hz完整实车控制。增速参数未做硬件能力标定，禁止直接套用到实车。',
              '审计检查记录一致性和执行约束，不把碰撞、余量不足或计算超时掩盖为通过。', '']
    (out/'aggregates.json').write_text(json.dumps(summary, indent=2)+'\n')
    (out/'failure_diagnostics.json').write_text(json.dumps(failure_diagnostics, indent=2)+'\n')
    (out/'README.md').write_text('\n'.join(lines)+'\n')
    gifs = list(out.rglob('*.gif'))
    for path in gifs:
        with Image.open(path) as im: assert im.n_frames > 1; im.seek(im.n_frames-1); im.load()
    (out/'report_audit.json').write_text(json.dumps(dict(trials=audited, readable_gifs=len(gifs),
           record_sha256=raw_hashes, report_script_sha256=sha(Path(__file__))), indent=2)+'\n')
    print('REPORT COMPLETE', out, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('out', type=Path); a = p.parse_args(); report(a.out)

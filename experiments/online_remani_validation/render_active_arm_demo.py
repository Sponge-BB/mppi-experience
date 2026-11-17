#!/usr/bin/env python3
"""Large A/B playback made only from the saved v2 measured states."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np
from PIL import Image

from report_active_arm import load_trial, actors_3d_at

ROOT = Path(__file__).resolve().parent / 'results/active_arm_20261104_01'
OUT = ROOT / 'report'
BLUE = '#286f9d'
ORANGE = '#e17d26'
RED = '#c84545'
plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False,
                     'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})


def index_at(trial, elapsed):
    return int(np.clip(np.searchsorted(trial['relative_t'], elapsed, side='right')-1,
                       0, len(trial['q'])-1))


def build(trials):
    fig, axes = plt.subplots(2, 2, figsize=(13.4, 9.4), dpi=80,
                             gridspec_kw={'height_ratios': [1.45, .95]})
    fig.subplots_adjust(left=.065, right=.985, top=.82, bottom=.12, wspace=.16, hspace=.42)
    groups = []
    for col, trial in enumerate(trials):
        side, top = axes[:, col]
        side.set(ylim=(-.02, 1.42), xlabel='行进方向 x / m', ylabel='高度 z / m')
        top.set(ylim=(-.40, .68), xlabel='行进方向 x / m', ylabel='横移 y / m')
        top.axhline(0., color='#999999', ls='--', lw=1)
        top.text(.01, .03, '虚线：原直行方向；实线：已走过的底盘轨迹',
                 transform=top.transAxes, fontsize=9, color='#555555')
        for ax in (side, top):
            ax.set_aspect('equal', adjustable='box')
            ax.grid(alpha=.15)
        side.axhline(0., color='#555555', lw=1)
        arm_datum = side.axhline(1.05, color=RED, ls=':', lw=.9, alpha=.5)
        balls = []
        for j in range(67):
            color = ORANGE if j >= 47 else BLUE
            pair = []
            for ax in (side, top):
                patch = Circle((0., 0.), .01, facecolor=color,
                               edgecolor='#a55015' if j >= 47 else '#245879',
                               alpha=.92 if j >= 47 else .48, lw=.5,
                               zorder=4 if j >= 47 else 2)
                ax.add_patch(patch)
                pair.append(patch)
            balls.append(pair)
        skeleton = []
        for link in range(6):
            skeleton.append([ax.plot([], [], color='#803e12', lw=1.2, zorder=5)[0]
                             for ax in (side, top)])
        obstacles = []
        for actor in trial['scene']['actors']:
            rect = Rectangle((actor['x']-actor['r'], actor['z_min']),
                             2*actor['r'], actor['z_max']-actor['z_min'],
                             color=RED, alpha=.65, zorder=6)
            circle = Circle((actor['x'], actor['y']), actor['r'], color=RED,
                            alpha=.45, zorder=1)
            side.add_patch(rect); top.add_patch(circle)
            obstacles.append((rect, circle))
        trail = top.plot([], [], color=BLUE if col == 0 else ORANGE, lw=2.3, zorder=3)[0]
        stats = side.text(.02, .02, '', transform=side.transAxes, fontsize=10,
                          va='bottom', zorder=9,
                          bbox=dict(facecolor='white', alpha=.9, edgecolor='none', pad=4))
        displacement = top.text(.98, .98, '', transform=top.transAxes, fontsize=12,
                                ha='right', va='top', color=BLUE if col == 0 else '#a95719',
                                bbox=dict(facecolor='white', alpha=.9, edgecolor='none', pad=3))
        groups.append((balls, skeleton, obstacles, trail, stats, displacement, arm_datum))
    note = fig.text(.5, .045, '', ha='center', va='center', fontsize=11)
    return fig, axes, groups, note


def update(fig, axes, groups, note, trials, indices, title, subtitle):
    center = float(np.mean([trial['base'][i, 0] for trial, i in zip(trials, indices)]))
    window = (center-.65, center+1.15)
    for col, (trial, i, group) in enumerate(zip(trials, indices, groups)):
        balls, skeleton, obstacles, trail, stats, displacement, datum = group
        body = trial['bodies'][i]
        for row, pair in zip(body, balls):
            pair[0].center = (row[0], row[2]); pair[1].center = (row[0], row[1])
            pair[0].radius = pair[1].radius = row[3]
        for link, lines in enumerate(skeleton):
            a, b = trial['model']._point_offsets[link:link+2]
            arm = body[47+a:47+b]
            lines[0].set_data(arm[:, 0], arm[:, 2])
            lines[1].set_data(arm[:, 0], arm[:, 1])
        anchor = trial['metadata']['motion_anchor_s']
        elapsed = float(trial['relative_t'][i])
        actors = actors_3d_at(trial['state']['t'][i]-anchor, trial['scene'])
        for j, (rect, circle) in enumerate(obstacles):
            visible = j < len(actors)
            rect.set_visible(visible); circle.set_visible(visible)
            if visible:
                actor = actors[j]
                rect.set_x(actor['x']-actor['r']); rect.set_y(actor['z_min'])
                circle.center = (actor['x'], actor['y'])
        datum.set_visible(bool(actors))
        trail.set_data(trial['base'][:i+1, 0], trial['base'][:i+1, 1])
        q2, q5 = np.rad2deg(trial['q'][i, [1, 4]])
        arm_top = np.max(body[47:, 2]+body[47:, 3])*100.
        stats.set_text('机械臂最高点 %.1f cm\n肩关节 %.1f°；腕关节 %.1f°' % (arm_top, q2, q5))
        displacement.set_text('底盘横移 %.1f cm' % round(float(trial['base'][i, 1])*100., 1)
                              if abs(float(trial['base'][i, 1])) >= .0005 else '底盘横移 0.0 cm')
        for ax in axes[:, col]:
            ax.set_xlim(window)
        axes[0, col].set_title(('固定机械臂：依靠底盘绕行' if col == 0 else '主动机械臂：折臂降低高度')+
                               '\n侧视｜实测 t = %.2f s' % elapsed,
                               fontsize=13, color=BLUE if col == 0 else '#a95719', pad=10)
        axes[1, col].set_title('俯视｜红色为高位障碍的平面投影', fontsize=11)
    fig.suptitle(title+'\n'+subtitle, y=.97, fontsize=15)
    note.set_text('蓝：固定结构　橙：机械臂　红：高位障碍（底面 105 cm）\n'
                  '同一场景、同一种子 23；仅回放实测底盘/关节。图中为碰撞球模型，非实车录像。')
    fig.canvas.draw()


def main():
    OUT.mkdir(exist_ok=True)
    tags = ['v2_A_fixed_seed23', 'v2_B_active_seed23']
    trials = [load_trial(ROOT/tag) for tag in tags]
    fig, axes, groups, note = build(trials)
    indices = [int(np.argmin(abs(t['base'][:, 0]-1.6))) for t in trials]
    update(fig, axes, groups, note, trials, indices,
           '同样行进到 x ≈ 1.60 m：机械臂收拢后，底盘可以少绕行',
           '两幅关键帧按相同行进位置对齐；各自实测时刻标在图内')
    fig.savefig(OUT/'机械臂避障_关键现象.png', dpi=120)
    keyframes = []
    for trial, index in zip(trials, indices):
        keyframes.append(dict(trial=trial['metadata']['trial'], csv_data_row=index+1,
                              record_sha256=trial['metadata']['source_sha256'],
                              model_provenance=trial['metadata']['model'],
                              elapsed=float(trial['relative_t'][index]),
                              base=trial['base'][index].tolist(), q=trial['q'][index].tolist(),
                              arm_highest_surface_m=float(np.max(trial['bodies'][index, 47:, 2]+
                                                               trial['bodies'][index, 47:, 3]))))
    frames = []
    end = max(float(t['relative_t'][-1]) for t in trials)
    count = int(np.ceil(end/.24))+1
    for elapsed in np.linspace(0., end, count):
        indices = [index_at(t, elapsed) for t in trials]
        update(fig, axes, groups, note, trials, indices,
               '看侧视的橙色机械臂下降，再看俯视的底盘侧移',
               '同一运动时钟 %.2f s｜2 倍速｜镜头沿行进方向跟随' % elapsed)
        frames.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())
                      .convert('P', palette=Image.ADAPTIVE, colors=96, dither=Image.NONE))
    duration = int(round(end/(count-1)*500.))
    frames[0].save(OUT/'机械臂避障_放大对照.gif', save_all=True, append_images=frames[1:],
                   duration=duration, loop=0, optimize=True, disposal=2)
    for frame in frames:
        frame.close()
    plt.close(fig)
    (OUT/'机械臂避障_可视化来源.json').write_text(json.dumps(dict(
        trials=tags, keyframes=keyframes, playback_frames=count,
        playback_duration_per_frame_ms=duration, playback_speed=2,
        data_source='saved measured state.csv via trial-frozen native DH sphere model',
        camera='shared moving x window, identical scale in both panels',
        keyframe_alignment='matched base x, not matched time',
        movie_alignment='same motion-anchor-relative time; ended trajectory freezes'),
        ensure_ascii=False, indent=2)+'\n')
    print('DEMO_READY', OUT/'机械臂避障_关键现象.png', OUT/'机械臂避障_放大对照.gif')


if __name__ == '__main__':
    main()

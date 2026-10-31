#!/usr/bin/env python3
"""Concise plots/report of the frozen native backend root-cause replay."""
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
import numpy as np
from PIL import Image, ImageChops, ImageDraw, ImageFilter, ImageFont

from audit_delivery_iterates import loads_preserving_cpp_zero
from backend_base_feasibility_breakdown import state_values
from rootcause_components import HERE, TRIAL, Probe, attest, context
from render_dual_timescale import FONT

ROOT = HERE/'results/root_cause_20261030_01'


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def summary_row(row, source):
    stages = row['stages']; passing = [s for s in stages if s['native_pass']]
    last = stages[-1]; setup = row.get('setup_s', 0.)
    return dict(label=row['label'], source=str(source), result=row['result'], force_stop=row['force_stop'],
        solver_elapsed_s=row['elapsed_s'], setup_s=setup, total_setup_plus_solver_s=setup+row['elapsed_s'],
        first_native_pass_solver_s=passing[0]['elapsed_s'] if passing else None,
        first_native_pass_setup_plus_solver_s=setup+passing[0]['elapsed_s'] if passing else None,
        first_native_pass_iteration=passing[0]['iteration'] if passing else None,
        final_native_pass=last['native_pass'], final_native_reason=last['native_reason'],
        final_cost=last['cost'], final_components=last['components'],
        final_max_wheel_ratio=float(max(np.asarray(last['wheel_max'])/[4.305,4.305,4.4625,4.4625])),
        complete_evaluations=row['evaluations'], accepted_steps=row['accepted_steps'])


def comparison_figure(rows, checkpoint, destination):
    plt.rcParams['font.family'] = FontProperties(fname=FONT).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.1), constrained_layout=True)
    labels = ['原初值0.6s', '原初值1.2s', '原初值3s', '首尾加时0.6s', '首尾加时1.2s']
    names = ['original_initial_060', 'original_initial_120', 'original_initial_300',
             'endpoint_initial_060', 'endpoint_initial_120']
    if 'original_initial_diagonal_120' in rows:
        names.append('original_initial_diagonal_120'); labels.append('原初值+缩放1.2s')
    selected = [rows[n] for n in names]
    values = [r['final_max_wheel_ratio'] for r in selected]
    colors = ['#4e7c9d' if r['final_native_pass'] else '#999999' for r in selected]
    axes[0].bar(range(len(values)), values, color=colors, width=.65)
    for i, (v, row) in enumerate(zip(values, selected)):
        word = '完整通过' if row['final_native_pass'] else ('碰撞拒收' if row['final_native_reason']=='planning_collision' else '轮约束拒收')
        axes[0].text(i, v+.03, '%.2f\n%s' % (v, word), ha='center', va='bottom', fontsize=8)
    axes[0].axhline(1, color='#b34d3f', ls='--', lw=1)
    axes[0].set_xticks(range(len(values))); axes[0].set_xticklabels(labels, rotation=18, fontsize=9)
    axes[0].set_ylabel('最终最坏轮约束 / 验收上限')
    axes[0].set_ylim(0, max(values)*1.28)
    axes[0].set_title('同一请求：预算、初始化和变量缩放的影响', fontsize=11)
    axes[0].grid(axis='y', alpha=.15)
    groups = ['first_point_xy', 'later_point_xy', 'time', 'gear_xy', 'gear_angle']
    group_labels = ['首段内点', '其余内点', '时间变量', '换向位置', '换向角度']
    for offset, channel, name, color in [(-.25, 'smooth', '平滑', '#999999'),
            (0., 'wheel', '底盘可行性', '#4e7c9d'), (.25, 'base_obstacle_residual', '障碍物', '#ba8b4e')]:
        v = [max(1e-7, checkpoint['stats'][channel]['groups'][g]) for g in groups]
        axes[1].bar(np.arange(len(groups))+offset, v, width=.23, color=color, label=name)
    axes[1].set_yscale('log'); axes[1].set_ylim(1e-2, 1e6)
    axes[1].set_xticks(range(len(groups))); axes[1].set_xticklabels(group_labels, fontsize=9)
    axes[1].set_ylabel('原始变量坐标中的梯度范数（对数轴）')
    axes[1].set_title('3s失败候选：不同变量的数值尺度差异很大', fontsize=11)
    axes[1].legend(fontsize=8, ncol=3); axes[1].grid(axis='y', alpha=.15)
    cold=rows.get('original_initial_diagonal_120')
    subtitle = ('缩放组首次合格候选：准备 %.3fs + 优化 %.3fs = %.3fs' %
        (cold['setup_s'],cold['first_native_pass_solver_s'],cold['first_native_pass_setup_plus_solver_s'])) if cold else ''
    fig.suptitle('当前工程同一冻结请求；原初值不是原生项目\n'+subtitle, fontsize=11)
    fig.savefig(destination, dpi=170); plt.close(fig)


def curve(value, ctx, n=240):
    duration = float(sum(sum(t) for t in value['times_parts']))
    return np.asarray([at_pose(value, ctx, t) for t in np.linspace(0, duration, n)])


def at_pose(value, ctx, t):
    local = float(t)
    for block, (coeff, times, gear) in enumerate(zip(value['b_parts'], value['times_parts'], ctx.gears)):
        for piece, duration in enumerate(times):
            if local <= duration or (block+1 == len(ctx.gears) and piece+1 == len(times)):
                c = coeff[piece*8:(piece+1)*8]
                p = np.polynomial.polynomial.polyval(local, c)
                v = state_values(c, [local])[0][0]
                return [float(p[0]), float(p[1]), math.atan2(gear*v[1], gear*v[0])]
            local -= duration
    raise ValueError('candidate time')


def candidates_gif(raw, destination):
    old = raw['original_initial_300']['stages'][-1]
    cold_start = 'original_initial_diagonal_120' in raw
    new = raw.get('original_initial_diagonal_120',raw.get('collision_checkpoint_diagonal'))
    if new is None:
        return None
    run = next(loads_preserving_cpp_zero(l) for l in (TRIAL/'candidate_audit/endpoint_time_replay.jsonl').open()
               if '"label":"original_060"' in l)
    ctx = context(run, json.loads((TRIAL/'effective_real_params.json').read_text()))
    probe = Probe(attest()['library']); old_value = probe.evaluate(np.asarray(old['x']), ctx)
    body = json.loads((TRIAL/'native_collision_3d_body.json').read_text())
    body = np.asarray(body['body']); body[body[:, 6] == 0, 3] -= .05
    meta = json.loads((TRIAL/'candidate_audit/map.json').read_text())
    shape = meta['map_voxel_num']; resolution = meta['resolution']; origin = np.asarray(meta['map_origin'])
    esdf = np.frombuffer(gzip.open(meta['esdf_path'], 'rb').read(), dtype=np.float64).reshape(shape)
    heights = (np.arange(shape[2])+.5)*resolution+origin[2]
    visible_height = (heights >= .10) & (heights <= 1.35)
    # Project the collision-relevant observed volume, excluding virtual ceiling.
    occupied = np.argwhere(np.any(esdf[:, :, visible_height] <= 0.,axis=2))
    occupied = (occupied+.5)*resolution+origin[:2]
    overlay = np.frombuffer(gzip.open(meta['overlay_path'], 'rb').read(), dtype=np.float32).reshape(-1, 4)
    overlay_xy = np.unique(np.round(overlay[:, :2], 2), axis=0)
    font = ImageFont.truetype(FONT, 18); small = ImageFont.truetype(FONT, 15)
    xmin, xmax, ymin, ymax = 1.65, 6.45, -1.45, 1.00
    old_path = curve(old_value, ctx)
    frames=[]
    first_pass = next((i for i,s in enumerate(new['stages']) if s['native_pass']),7)
    indices = sorted(set([0, 3, 7, first_pass, len(new['stages'])-1]))
    for index in indices:
        stage = new['stages'][index]; value = probe.evaluate(np.asarray(stage['x']), ctx)
        frame = Image.new('RGB', (1100, 450), 'white'); draw = ImageDraw.Draw(frame)
        draw.text((24, 8), '冻结候选优化示意：曲线没有在机器人上执行', font=font, fill='#222222')
        for panel, candidate, path, title in [(0, old_value, old_path, '原始初始化：3秒后仍碰撞'),
                (1, value, curve(value, ctx), ('原初值+变量缩放：接受步 %d / %s' if cold_start else '失败候选重启：接受步 %d / %s') %
                 (stage['iteration'], '完整通过' if stage['native_pass'] else '未通过'))]:
            left = 25+panel*550; top=82; width=500; height=270
            def pixel(xy):
                return (int(left+(xy[0]-xmin)/(xmax-xmin)*width),
                        int(top+height-(xy[1]-ymin)/(ymax-ymin)*height))
            draw.text((left, 43), title, font=small, fill='#333333')
            draw.rectangle((left, top, left+width, top+height), outline='#bbbbbb')
            for p in occupied:
                if xmin < p[0] < xmax and ymin < p[1] < ymax:
                    x,y=pixel(p); draw.rectangle((x-2,y-2,x+2,y+2), fill='#bdbdbd')
            for p in overlay_xy:
                if xmin < p[0] < xmax and ymin < p[1] < ymax:
                    x,y=pixel(p);draw.point((x,y), fill='#8cb4b0')
            draw.line([pixel(p) for p in path], fill='#315f83', width=3)
            pose = at_pose(candidate, ctx, 6.37)
            rot = np.array([[math.cos(pose[2]),-math.sin(pose[2])],[math.sin(pose[2]),math.cos(pose[2])]])
            centers = body[:, :2]@rot.T+pose[:2]
            mask = Image.new('L', frame.size, 0); md=ImageDraw.Draw(mask)
            for center, radius in zip(centers, body[:, 3]):
                x,y=pixel(center);rx=radius/(xmax-xmin)*width;ry=radius/(ymax-ymin)*height
                md.ellipse((x-rx,y-ry,x+rx,y+ry),fill=255)
            frame.paste('#d9e7f1', mask=mask)
            boundary = ImageChops.subtract(mask, mask.filter(ImageFilter.MinFilter(3)))
            frame.paste('#315f83', mask=boundary)
            draw=ImageDraw.Draw(frame)
            if panel == 0:
                x,y=pixel(centers[12]);draw.ellipse((x-3,y-3,x+3,y+3),fill='#b34d3f')
            draw.text((left, 364), '投影：候选 t=6.37s 的67球外形', font=small, fill='#555555')
        message = ('右图从原初值求解；准备 %.3fs + 首次合格优化 %.3fs，均另计。' %
            (new.get('setup_s',0),new['stages'][first_pass]['elapsed_s'])) if cold_start else '右图从既有失败候选重启；缩放准备约1.08s另计。'
        draw.text((25, 386), '灰色：ESDF≤0体素在z=0.10～1.35m的投影；青色：动态层；红点：违规球中心。',font=small,fill='#666666')
        draw.text((25, 412), message+'不是导航回放。', font=small, fill='#555555')
        frames.append(frame)
    frames[-1].save(destination.with_name('候选最终对照.png'))
    frames[0].save(destination, save_all=True, append_images=frames[1:], loop=0,
                   duration=[700]*(len(frames)-1)+[1500], optimize=False, disposal=2)
    return dict(path=str(destination), frames=len(frames), motion_execution=False,
                interpretation='accepted-candidate geometry changes at a fixed trajectory time, not navigation playback')


def main():
    out=ROOT/'report';out.mkdir(exist_ok=True)
    raw={};sources={}; records=[]
    for folder in sorted(ROOT.glob('native_run_*')):
        path=folder/'results.jsonl'
        if not path.exists():continue
        for row in read_rows(path):
            if row['kind']=='solve':
                raw[row['label']]=row; records.append(summary_row(row,path))
        sources[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    rows={r['label']:r for r in records}
    checkpoint=json.loads((ROOT/'checkpoint_components.json').read_text())
    collision=json.loads((ROOT/'collision_residual_analysis.json').read_text())
    comparison_figure(rows,checkpoint,out/'核心排查.png')
    gif=candidates_gif(raw,out/'冻结候选对照.gif')
    summary=dict(schema='rootcause_native_summary_v1', source_hashes=sources, solves=records,
        collision_residual=collision, gif=gif, online_delivery_measured=False,
        physical_constraints_relaxed=False, production_code_modified=False,
        single_frozen_request_only=True)
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    lines=['本次排查结论（同一条冻结输入，真实原生后端回放）',
        '所有original标签都指当前工程的原始初始化，不是原生REMANI项目。全部使用同一4.1/4.25轮约束、67球和同一地图。', '',
        '直白地说：前面短段的变量过于敏感，求解器只能采取很小的步子，后面绕障的部分因此修得很慢。较短预算和只看总成本变化的停止条件，又让尚未合格的候选提前结束。只改变计算坐标尺度就能让相同请求通过，说明不能把原因单独归为轮约束太紧。', '',
        '1. 这条请求不是根本无解。保持相同地图、边界、碰撞模型和轮约束，已经找到完整原生验收通过的曲线。',
        '2. 0.6秒预算确实会截断修复过程，但它不是全部原因。首尾加时初值给1.2秒时，约0.709秒出现第一条合格候选，约0.886秒正常结束。原始初值给3秒，约2.759秒正常结束，轮约束虽通过，底盘碰撞仍未消掉。',
        '3. 剩余碰撞已经具体定位。最深处底盘球距硬验收边界差约1.32厘米；优化器自己的204个K16节点中有15个节点已碰撞，地图梯度也有方向。距离来自融合ESDF层，不能直接称为物理静态障碍。独立重算障碍代价与原生成本一致，因此采样漏检或地图梯度缺失不能解释这次失败。',
        '4. 数值尺度不合适是当前确认的重要问题。失败候选首段内点的底盘可行性梯度约23万，后面内点的障碍梯度约1801；不同变量的敏感度差异很大，优化目标下降不代表碰撞和轮约束都被修好。细步长差分支持现有解析梯度，没有证实梯度公式错误。',
        '5. 普通的目标变化停止条件可以在梯度仍很大、硬验收尚未通过时结束。原始3秒组正常停止时完整梯度范数约24.6万；收紧停止条件和从同点普通重启都没有马上消掉碰撞。',
        '6. 同一失败候选重启时，普通坐标求解仍未通过，固定对角缩放后能通过。进一步从原始未加时初值直接求解，变量缩放组也完整通过；约束、外形和代价权重均未放宽。变量缩放只改变求解坐标，不改变待求的物理轨迹目标。', '']
    descriptions={
        'endpoint_initial_060':'首尾加时初值，预算0.6秒',
        'endpoint_initial_120':'首尾加时初值，预算1.2秒',
        'endpoint_last_restart_060':'从首尾加时的失败候选普通重启',
        'original_initial_060':'当前原始初值，预算0.6秒',
        'original_initial_120':'当前原始初值，预算1.2秒',
        'original_initial_300':'当前原始初值，预算3秒',
        'original_collision_restart_060':'从碰撞候选普通重启',
        'original_delta1e6_300':'当前原始初值，只收紧停止条件，预算3秒',
        'collision_checkpoint_unscaled':'从碰撞候选重启，不做变量缩放',
        'collision_checkpoint_diagonal':'从同一碰撞候选重启，做变量缩放',
        'original_initial_diagonal_120':'从当前原始初值直接求解，做变量缩放'}
    reasons={'ok':'完整通过','planning_collision':'碰撞拒收','left_wheel_speed':'左轮速度拒收',
        'right_wheel_speed':'右轮速度拒收','left_wheel_acceleration':'左轮加速度拒收','right_wheel_acceleration':'右轮加速度拒收'}
    for r in records:
        first = '无' if r['first_native_pass_solver_s'] is None else '%.3fs' % r['first_native_pass_solver_s']
        lines.append('%s：准备%.3fs；求解%.3fs；首次原生通过（从求解开始）%s；最终%s。' %
            (descriptions.get(r['label'],r['label']),r['setup_s'],r['solver_elapsed_s'],first,reasons.get(r['final_native_reason'],r['final_native_reason'])))
    cold=rows.get('original_initial_diagonal_120')
    if cold:
        lines += ['', '原初值直接缩放组：准备 %.6fs + 首次合格候选的优化 %.6fs = %.6fs。' %
            (cold['setup_s'],cold['first_native_pass_solver_s'],cold['first_native_pass_setup_plus_solver_s']),
            '这个合计仍不是在线交付耗时：没有包含在线触发、A*搜索、初始化、最终验收和发布链路。']
    lines += ['', '结果边界：',
        '失败候选重启组的0.188秒从已有3秒失败曲线开始计算；准备成本另计。原初值直接求解组的准备成本也已单列，不能只报0.333秒。',
        '这是一次固定输入的根因实验，不是在线导航、Gazebo或实车实验，也不能据此统计所有场景的成功率。',
        '后续需要降低尺度估计的准备成本，并把有效的修改放回在线链路测触发到交付的全过程。',
        '全部失败记录保留。图中GIF是候选曲线变化示意，67球投影取固定候选时刻，没有发出机器人控制。']
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')
    print(out)


if __name__=='__main__':
    main()

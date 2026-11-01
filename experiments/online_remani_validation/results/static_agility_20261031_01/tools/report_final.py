#!/usr/bin/env python3
"""Post-run summary of the static-agility candidate; no controller execution."""
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve()
ROOT = HERE.parents[1]
EXPERIMENT = HERE.parents[3]
sys.path.insert(0, str(EXPERIMENT))

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_coupled_objective import sha
from render_dual_timescale import FONT
from report_long_goal_crossing import panel
from report_static_agility import chart, gif, narrative, show, summarize


def single_gif(row, trial, output):
    duration = trial.end-trial.anchor
    ages = np.linspace(0., duration, min(110, max(2, math.ceil(duration/.4)+1)))
    title, small = [ImageFont.truetype(FONT, size) for size in (20, 15)]
    frames = []
    for age in ages:
        image = Image.new('RGB', (1050, 610), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((18, 9), '迎面障碍回归：起步后 %.1f s' % age, font=title, fill='#333333')
        draw.text((18, 42), '蓝：实走轨迹/机器人　橙：REMANI参考　绿：获准局部展开　紫：CV预测', font=small, fill='#555555')
        t = min(trial.anchor+float(age), trial.end)
        state = panel(image, trial, t, (18, 85, 1032, 505))
        draw = ImageDraw.Draw(image)
        status = ('到达目标' if row['reached_goal'] else row['termination']) if t >= trial.end-1e-9 else trial.status_at(t)
        if status == 'ok':
            status = '控制有效'
        draw.text((18, 520), '%s　v=%+.2f m/s　ω=%+.2f rad/s' % (status, state['v'], state['w']), font=small, fill='#555555')
        draw.text((18, 555), '实录回放，2倍播放；显示完整固定臂碰撞球投影。绿色不是全部搜索采样。', font=small, fill='#777777')
        draw.text((18, 582), '完整REMANI＋底盘MPPI，ROS数值闭环，非Gazebo/实车；结束后不续画未来。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(output, save_all=True, append_images=frames[1:],
        duration=[max(30, 10*round(float(dt)*500./10.)) for dt in np.diff(ages)]+[1000],
        loop=0, disposal=2, optimize=True)
    return dict(frames=len(frames), sha256=sha(output), actual_records_only=True,
                full_search_recorded=False, playback_speedup=2.)


def short_text(rows):
    old, new, repeat, headon = rows
    lines = ['本轮结果：提前侧绕，并修正局部引导的地图范围检查', '',
        '这轮不是放宽碰撞距离，也没有提高轮速。仍是67个碰撞球、原来的轮速/加速度和最新状态安全验收。',
        '主要改动：原参考将受阻时，更早启用已有的、经过检查的侧绕分支；检查完整局部绕行连接，并至少覆盖一个展开时域可行进的路程，不再用未执行的全局尾段否掉局部引导。',
        '没有加入MPPI向REMANI传递绕行侧的机制，不能把方向偶然一致写成已解决两层协同。', '',
        '核心对照：',
        '原条件：降速到实测持续转向%s秒，到发布持续转向控制%s秒。' % (
            show(old['brake_to_measured_turn_s']), show(old['brake_to_published_turn_s'])),
        '修正后第1次：降速到实测持续转向%s秒，到发布持续转向控制%s秒。' % (
            show(new['brake_to_measured_turn_s']), show(new['brake_to_published_turn_s'])),
        '修正后第2次：降速到实测持续转向%s秒，到发布持续转向控制%s秒。' % (
            show(repeat['brake_to_measured_turn_s']), show(repeat['brake_to_published_turn_s'])),
        '负值代表先开始持续转向、后降到0.22米/秒以下。这里不是等待首次MPPI控制，减速时也一直在控制。', '',
        '结果表（转向阈值：同方向|角速度|>0.2弧度/秒，持续至少0.2秒）：']
    for row in rows:
        lines.append('%s：%s，采样碰撞%s；路长%s米，运动%s秒；最小物理采样间隙%s米；非目标低速累计%s秒、最长%s秒。' % (
            row['label'], '到达目标' if row['reached_goal'] else row['termination'],
            '有' if row['physical_sample_collision'] else '未发现', show(row['actual_path_length_m']),
            show(row['motion_duration_s']), show(row['minimum_physical_sample_gap_m']),
            show(row['non_goal_low_speed']['total_s']), show(row['non_goal_low_speed']['longest_s'])))
        lines.append('  内核P95/最大%s/%s毫秒；整回调P95/最大%s/%s毫秒；控制结果%s。' % (
            show(row['kernel']['p95_ms']), show(row['kernel']['max_ms']), show(row['callback']['p95_ms']),
            show(row['callback']['max_ms']), json.dumps(row['reason_counts'], ensure_ascii=False)))
        turn = row['first_measured_sustained_turn'] or {}
        first_ref = row['reference_adoptions'][0] if row['reference_adoptions'] else None
        lines.append('  首次持续转向%s；实际经过x=3处y=%s；首次新参考在x=3处y=%s。' % (
            turn.get('direction', '无'), ', '.join(show(v) for v in row['actual_crossing_y_at_x3_m']) or '未经过',
            ', '.join(show(v) for v in first_ref['crossing_y_at_x3_m']) if first_ref else '无'))
    lines += ['', '保留的问题：',
        '1. 本轮静态实际绕行更宽，路程增加约0.35至0.39米，到达目标也没有更快；不能只看转向提前就宣布全面优于V12。',
        '2. 第2次静态记录出现2次snapshot_binding_failed：增量快照换了局部裁剪范围，但缓存还没匹配新裁剪，因此这两次没有进入MPPI计算；完整快照恢复后继续控制。约62毫秒下发零指令，期间实测速度最低约0.220米/秒，没有停住。相关传输代码与父版本相同；完整包具体丢在哪个环节记录不足，本轮不扩大修复范围。',
        '3. 迎面只做1次修正后回归，没有配套迎面旧条件对照，不能据此宣称动态性能提高多少。',
        '4. 这是固定机械臂、完整REMANI+MPPI的ROS数值闭环，不是Gazebo/实车，也不是与历史V12同输入同配置的直接对照。',
        '图中蓝线为实走轨迹，橙线为真实采用的REMANI参考，绿线为获准局部展开；本轮没有记录完整搜索，不补画示意搜索线。', '']
    return '\n'.join(lines)


def main():
    jobs = [('static_old_repeat01', '原条件'), ('static_final_repeat01', '提前绕行＋局部范围修正'),
            ('static_final_repeat02', '修正后静态第2次'), ('headon_final_repeat01', '修正后迎面回归')]
    pairs = [summarize(ROOT/name, label) for name, label in jobs]
    rows, trials = map(list, zip(*pairs))
    out = ROOT/'final_report'
    out.mkdir(exist_ok=False)
    chart(rows[:2], trials[:2], out/'静态速度与转向.png')
    for row in rows:
        print(json.dumps({key: row[key] for key in ('label', 'reached_goal', 'actual_path_length_m', 'motion_duration_s',
            'minimum_physical_sample_gap_m', 'brake_to_measured_turn_s', 'brake_to_published_turn_s',
            'kernel', 'callback', 'reason_counts', 'actual_crossing_y_at_x3_m', 'reference_adoptions')}, ensure_ascii=False), flush=True)
    renderings = dict(static=gif(rows[:2], trials[:2], out/'静态旧新对照.gif', 110),
                      headon=single_gif(rows[3], trials[3], out/'迎面回归.gif'))
    for trial in trials:
        trial.verify_unchanged()
    (out/'summary.json').write_text(json.dumps(dict(trials=rows, renderings=renderings,
        comparison='Current old-condition versus final static-guidance fix; NOT a direct historical V12 comparison.'), ensure_ascii=False, indent=2)+'\n')
    (out/'详细数据.txt').write_text(narrative(rows))
    (out/'实验结论.txt').write_text(short_text(rows))
    print(str(out), flush=True)


if __name__ == '__main__':
    main()

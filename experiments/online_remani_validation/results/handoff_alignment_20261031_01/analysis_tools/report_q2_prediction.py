#!/usr/bin/env python3
"""Generate a compact audit report from the recorded rejected candidate."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

from replay_q2_prediction import audit


def main():
    result = audit()
    out = Path(__file__).resolve().parent / 'q2_prediction_report'
    out.mkdir(exist_ok=True)
    (out / '同输入时间映射结果.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.size': 10, 'axes.unicode_minus': False})
    fig, ax = plt.subplots(figsize=(8.3, 4.9), constrained_layout=True)
    checks = result['checks']
    x = np.arange(3)
    half = [checks[i]['minimum']['gap_m'] * 100 for i in [0, 2, 4]]
    full = [checks[i]['minimum']['gap_m'] * 100 for i in [1, 3, 5]]
    for shift, vals, color, label in [(-.18, half, '#3676a7', '0.5倍速度'),
                                     (.18, full, '#d49442', '1倍速度（反事实）')]:
        bars = ax.bar(x + shift, vals, .34, color=color, label=label)
        for bar, value in zip(bars, vals):
            ax.annotate(f'{value:+.2f}', (bar.get_x() + bar.get_width()/2, value),
                        xytext=(0, 4 if value >= 0 else -4), textcoords='offset points',
                        ha='center', va='bottom' if value >= 0 else 'top')
    ax.axhline(0, color='#444444', lw=.9)
    ax.set_xticks(x)
    ax.set_xticklabels(['从候选开头开始\n原验收起点', '按候选时间戳推进\n反事实',
                        '按当前位置投影\n反事实'], fontproperties=font)
    ax.set_ylabel('预测时域内最小检查间隙（厘米）', fontproperties=font)
    ax.set_title('同一条被拒绝的候选：改变时间假设会改变碰撞判断', fontproperties=font)
    ax.set_ylim(-9.3, 6.4)
    for side in ['top', 'right']:
        ax.spines[side].set_visible(False)
    ax.legend(prop=font, loc='lower right', frameon=False)
    fig.text(.5, -.015, '零以下表示球与预测点云重叠。候选未执行；投影不是实际轨迹进度，也不是取消半速检查的依据。',
             ha='center', fontproperties=font, fontsize=9)
    fig.savefig(out / '同输入时间映射对照.png', dpi=180, bbox_inches='tight')
    plt.close(fig)
    rows = ['q2 最新预测拒绝：同输入复算', '',
            '这次只检查上一轮同一条被拒绝的轨迹，没有重新运行机器人。',
            '使用原记录的完整98个动态点、CV速度、候选多项式和67个原生碰撞球。',
            '候选没有被发布执行。以下全速、时间戳推进和位置投影均为反事实。', '',
            '复现结果：原检查从轨迹开头开始，要求半速执行也安全。',
            '它在轨迹的0.8秒位置，与1.6秒之后的预测障碍物发生检查重叠；间隙为-1.33毫米。',
            '复算首次失败的时间、位置、朝向和底盘碰撞类型均与原记录一致。',
            '当时轨迹时间戳已过去0.249秒，机器人相对候选起点已经移动12.56厘米。', '',
            '同一输入，整个预测时域最小检查间隙：',
            '原验收在首次碰撞处停止；这里离线继续检查到时域末端，才能得到以下最小值。',
            '时间起点                       半速         全速（反事实）']
    for label, a, b in zip(['候选开头（原检查）', '按候选时间戳推进', '按当前位置投影'], half, full):
        rows.append(f'{label:<21} {a:+.2f}厘米       {b:+.2f}厘米')
    rows += ['', '说明：',
             '1. 仅修正过期时间仍然不能让半速检查通过，不能归因为一个时间戳错误。',
             '2. 全速检查通过，说明验收所假设的速度会影响结果；不代表MPPI一定这样走。',
             '3. 当前位置投影得到的0.548秒只是几何投影，不是机器人执行这条候选的真实进度。',
             '4. 这里的间隙是原生球与CV预测点云的检查间隙，不是机器人实测安全距离。',
             '5. 未保存当时完整ESDF和冻结扫掠点云，所以本次无法精确复现q3原生规划碰撞。', '',
             '检查规则和来源：',
             '底盘47球、机械臂20球；球位置和半径来自native_collision_3d_body.json。',
             '底盘硬半径=模型球半径+地图分辨率0.05米，机械臂使用原生球半径。',
             '位置、速度和加速度预测不确定性均为0；CV最大预测时间为3秒。',
             '候选局部时间检查步长为0.05秒，与原验收函数一致。',
             '最小间隙<0判碰撞；没有另加投影或不确定性余量。',
             '参数来源：原parameters.yaml、原生导出代码及validatePredictedDynamicTrajectory。', '',
             f"原优化器输入hash：{result['source_identity']['optimizer_input_hash']}",
             f"原候选hash：{result['source_identity']['native_candidate_hash']}",
             '完整输入文件、原代码SHA256与采样明细保存在同目录JSON。']
    (out / '复算说明.txt').write_text('\n'.join(rows) + '\n')
    print(json.dumps({'output': str(out), 'first_failure_gap_m': result['original_logged_failure']['gap_m'],
                      'candidate_hash': result['source_identity']['native_candidate_hash']}, ensure_ascii=False))


if __name__ == '__main__':
    main()

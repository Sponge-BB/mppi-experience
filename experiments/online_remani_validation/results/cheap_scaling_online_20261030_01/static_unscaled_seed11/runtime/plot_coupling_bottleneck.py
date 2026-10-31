#!/usr/bin/env python3
"""Post-run summary of measured delivery and pre-optimizer target rejection."""
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np


GROUPS = ('native', 'engineering', 'execution', 'handover', 'coupled')
LABELS = ('串联', '投影补足', '近端项', '交接项', '两项耦合')
SCENES = ('static', 'dynamic_four', 'dynamic_head_on_center')
SCENE_LABELS = ('静态障碍', '四动态障碍', '迎面单障碍')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    source = args.summary.resolve()
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    data = json.loads(source.read_text())
    if data.get('provisional') or data.get('all_integrity_valid') is not True:
        raise ValueError('Requires a completed, integrity-valid report')
    rows = data['trials']
    if len(rows) != 15 or len({r['seed'] for r in rows}) != 1:
        raise ValueError('This figure is for the single-seed 3 x 5 pilot, not pooled experiments')
    index = {(r['scene'], r['group']): r for r in rows}
    if len(index) != 15 or set(index) != {(s, g) for s in SCENES for g in GROUPS}:
        raise ValueError('Expected exactly one retained trial per scene/group')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', size=10)
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.1), gridspec_kw={'width_ratios': [1.65, 1.]})
    table_values, table_colors = [], []
    for scene in SCENES:
        values, colors = [], []
        for group in GROUPS:
            row = index[scene, group]
            status = row['delivery_status']
            if status == 'delivered':
                value = row['blocked_to_consume_s']
                if not isinstance(value, (float, int)) or not np.isfinite(value):
                    raise ValueError('Delivered trial missing measured delay')
                values.append('%.2f' % value)
                colors.append('#edf3f7')
            elif status == 'no_delivery':
                values.append('未交付')
                colors.append('#faece9')
            elif status == 'not_triggered':
                values.append('未触发')
                colors.append('#eeeeee')
            else:
                raise ValueError('Unknown delivery status')
        table_values.append(values)
        table_colors.append(colors)
    axes[0].axis('off')
    axes[0].set_title('受阻 → MPPI采用新参考 / 秒', fontproperties=font, pad=18)
    table = axes[0].table(cellText=table_values, rowLabels=SCENE_LABELS, colLabels=LABELS,
                          cellColours=table_colors, cellLoc='center', loc='center',
                          colWidths=[.19] * 5)
    table.scale(1., 2.1)
    for cell in table.get_celld().values():
        cell.set_edgecolor('white')
        cell.get_text().set_fontproperties(font)
    x = np.arange(5)
    rejected = [index['dynamic_four', g]['exact_target_warning_requests'] for g in GROUPS]
    axes[1].bar(x, rejected, color='#587f99', width=.65)
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(LABELS, fontproperties=font, rotation=20)
    axes[1].set_title('四障碍：优化前的目标拒用 / 次', fontproperties=font, pad=18)
    axes[1].set_ylim(0, max(rejected) + 3)
    axes[1].set_yticks([0, 5, 10, 15])
    for i, count in enumerate(rejected):
        axes[1].text(i, count + .35, str(count), ha='center', fontsize=10)
    for name in ('top', 'right'):
        axes[1].spines[name].set_visible(False)
    axes[1].grid(axis='y', alpha=.15)
    fig.suptitle('耦合机制已接通，但本轮未证明避障/交付性能提升', fontproperties=font, fontsize=12, y=.98)
    fig.text(.5, .035, '固定机械臂 · 在线数值仿真（非实车） · 每格仅1次 · 未交付不记为0 · 串联也包含MPPI',
             ha='center', fontproperties=font, fontsize=9)
    fig.tight_layout(rect=[.065, .10, 1., .93])
    target = out / '验证要点.png'
    fig.savefig(target, dpi=160)
    plt.close(fig)
    if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
        raise RuntimeError('Source report changed while plotting')
    audit = dict(source=str(source), source_sha256=digest, source_unchanged=True,
                 trials=15, single_seed=True, visualization_only=True,
                 delay='first recorded blocked to first subsequent adopted reference; missing not imputed',
                 rejection='requests with exact source ROS target-selection warning, not all failures',
                 comparison='coupled includes engineering projection supplement; isolate item effect against engineering',
                 output_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    (out / 'plot_audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2) + '\n')
    print(target)


if __name__ == '__main__':
    main()

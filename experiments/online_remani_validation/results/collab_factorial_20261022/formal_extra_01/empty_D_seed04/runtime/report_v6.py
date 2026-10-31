#!/usr/bin/env python3
"""Rebuild reports/GIFs from saved data, never rerun or select a best seed."""
import argparse
import json
from pathlib import Path
import numpy as np
from suite_v6 import metrics,plots,gif

def main():
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--render',action='store_true');a=p.parse_args()
    all_rows={};text=['V5 / V6 局部动态避障对照','',
      '范围：固定机械臂底盘、固定旧参考、理想合成观测的数值闭环。不是完整 REMANI、Gazebo 或实车。',
      'V5 是上一实验版，不是原始 REMANI Tracker。若 separate_dynamic=true，V6 使用理想测量点归属分层；这是额外输入假设。',
      '各场景 GIF 均取最小种子号，不挑最好的一次。数据和失败记录全部保留。',
      '低速：|实际线速度|<0.02m/s，排除前1秒和目标0.15m内；不等于零角速度。',
      '净距：车体组合圆到障碍/墙的几何间距，不包含额外安全余量。kernel耗时不是完整回调时延。','']
    links=['# 实验导航','',
      '每个实验：`data/` 原始记录；`gif/` 独立动画；`plots/` 简洁曲线；`summary.json` 全部种子指标。',
      '范围和假设见 [结果汇总.txt](结果汇总.txt)。','']
    for case in sorted(a.out.iterdir()):
        if not (case/'data').is_dir():continue
        records=[(f,json.loads(f.read_text())) for f in sorted((case/'data').glob('*.json'))]
        rows=[{'version':f.stem.split('_')[0],'seed':r['seed'],**metrics(r)} for f,r in records]
        all_rows[case.name]=rows;(case/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
        first=[]
        for version in ('v5','v6'):
            matches=[r for f,r in records if f.stem.startswith(version+'_')]
            if matches:first.append(min(matches,key=lambda r:r['seed']))
        if a.render and len(first)==2:
            (case/'plots').mkdir(exist_ok=True);(case/'gif').mkdir(exist_ok=True)
            plots(case/'plots',first);gif(case/'gif'/'comparison.gif',first,['旧版 V5','新版 V6'])
            for r,label in zip(first,('v5','v6')):gif(case/'gif'/f'{label}.gif',[r],[label.upper()])
        text.append(case.name)
        for version in ('v5','v6'):
            rs=[r for r in rows if r['version']==version]
            if not rs:continue
            text.append(f"  {version}: 到达 {sum(r['reached'] for r in rs)}/{len(rs)}; "
              f"平均通行 {np.mean([r['duration_s'] for r in rs]):.2f}s; "
              f"平均低速 {np.mean([r['low_speed_away_from_goal_s'] for r in rs]):.2f}s; "
              f"平均路径 {np.mean([r['path_length_m'] for r in rs]):.2f}m; "
              f"最小净距 {min(r['min_clearance_m'] for r in rs):.3f}m; "
              f"未验证制动周期 {sum(r['unverified_brake_cycles'] for r in rs)}; "
              f"内核超50ms {sum(r['kernel_over_50ms'] for r in rs)}")
        text.append('')
        links.append(f"- {case.name}: [对比 GIF]({case.name}/gif/comparison.gif) · [V5]({case.name}/gif/v5.gif) · [V6]({case.name}/gif/v6.gif) · [曲线]({case.name}/plots/comparison.png)")
    (a.out/'summary.json').write_text(json.dumps(all_rows,indent=2)+'\n')
    (a.out/'结果汇总.txt').write_text('\n'.join(text)+'\n');(a.out/'README.md').write_text('\n'.join(links)+'\n')
    print('\n'.join(text))

if __name__=='__main__':main()

#!/usr/bin/env python3
"""Audit and summarize completed multi-obstacle trials without rerunning them."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image
from suite_multi_dynamic import NAMES, HERE, clearance, actor_pair_gaps


def report(out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    manifest = json.loads((out/'manifest.json').read_text())
    cases = manifest['cases']
    def observed(case, t):
        if t < manifest['appearance_s']: return np.empty((0, 6))
        a = np.array(cases[case]); a[:, :2] += (t-manifest['appearance_s'])*a[:, 2:4]; return a
    pairs = {case: actor_pair_gaps(actors) for case, actors in cases.items()}
    pair_overlaps = sum(v['gap'] < 0 for rows in pairs.values() for v in rows)
    equivalents = json.loads((out/'instrumentation_equivalence.json').read_text())
    assert all(v['identical_commands_and_states'] for v in equivalents.values())
    for name, digest in manifest['source_snapshot_sha256'].items():
        assert hashlib.sha256((out/'source_snapshot'/name).read_bytes()).hexdigest() == digest
    native = manifest['build']['production_sources_sha256']
    assert all(hashlib.sha256((HERE.parents[1]/name).read_bytes()).hexdigest() == digest for name, digest in native.items())
    rows = []; details = {}; trials = 0; selected = 0; collisions = 0
    lines = ['# V7 多动态障碍物实验', '',
             '范围：固定机械臂几何、固定直线参考、理想静动态分层及 CV 输入的局部数值闭环。',
             '**不是完整 REMANI 在线重规划、Gazebo 或实车；没有加入 CBF-QP。**', '',
             '## 设置', '',
             '- 每组 3 个种子，GIF 固定用 seed 1，不挑选最好一次。单障碍组是本轮对照，不是无 MPPI 基线。',
             '- 全部障碍物从 t=2s 出现并持续作匀速运动，出画面后也不从控制器输入中删除。',
             '- 参考从 (0,0) 到 (3,0)，走廊 y∈[-2,2]m；目标参考速度上限 0.18m/s。',
             '- 控制周期 50ms，CV 有效期 2s，观测年龄 40ms；车体圆组合及固定 0.1m 余量不变。',
             '- 实際车体净距在滞后执行模型的每个 5ms 子步检查；这仍不是连续时间安全证明。',
             '- 各场景只新增目标，前面已有目标的初始位置和速度完全相同；具体配置见 manifest.json。', '',
             '## 结果', '',
             '| 场景 | 到达 | 平均耗时/s | 路径/m | 低速/s | 倒车/m | 最小净距/m | 内核P95/ms |',
             '|---|---:|---:|---:|---:|---:|---:|---:|']
    for case in cases:
        records = [json.loads(f.read_text()) for f in sorted((out/case/'data').glob('seed*.json'))]
        assert len(records) == len(manifest['seeds'])
        metrics = [r['metrics'] for r in records]; trials += len(records)
        for r in records:
            body = np.array(r['body']); trace = r['trace']
            for row in trace:
                assert np.allclose(np.array(row['obstacles']).reshape(-1, 6), observed(case, row['t']))
                assert np.isfinite(row['state']+row['command']).all()
                # Dense minimum must include each interval's final endpoint.
                assert row['gap'] <= clearance(np.array(row['next_state']), observed(case, row['t']+.05), body)+1e-9
            for snap in r['searches']:
                chosen = [v for v in snap['candidates'] if v['selected']]
                assert len(chosen) <= 1
                for v in snap['candidates']:
                    assert np.isfinite(np.array(v['points'])).all()
                    assert v['feasible'] == (v['gap'] >= 0.)
                assert all(v['feasible'] for v in chosen); selected += len(chosen)
            collisions += int(r['metrics']['termination'] == 'collision')
        mean = lambda k: float(np.mean([m[k] for m in metrics]))
        r = {'case': case, 'count': len(cases[case]), 'reached': sum(m['reached'] for m in metrics),
             'trials': len(metrics), 'duration_s': mean('duration_s'), 'path_m': mean('path_m'),
             'low_speed_s': mean('low_speed_s'), 'reverse_m': mean('reverse_m'),
             'min_gap_m': min(m['min_gap_m'] for m in metrics), 'kernel_p95_ms': mean('kernel_p95_ms'),
             'margin_violation_cycles': sum(m['margin_violation_cycles'] for m in metrics),
             'unverified_brake_cycles': sum(m['unverified_brake_cycles'] for m in metrics),
             'kernel_over_50ms': sum(m['kernel_over_50ms'] for m in metrics)}
        rows.append(r)
        lines.append(f'| {NAMES[case]} | {r["reached"]}/{r["trials"]} | {r["duration_s"]:.2f} | {r["path_m"]:.2f} | {r["low_speed_s"]:.2f} | {r["reverse_m"]:.3f} | {r["min_gap_m"]:.4f} | {r["kernel_p95_ms"]:.2f} |')
        nominal = records[0]; ts = nominal['trace']
        active = [row for row in ts if row['obstacles']]
        reasons = {str(k): sum(int(row['decision'][0]) == k for row in active)*.05 for k in range(5)}
        details[case] = {'seed1_mode_duration_s': reasons,
                         'search_max_candidates': max(len(s['candidates']) for s in nominal['searches']),
                         'first_unverified_brake_s': next((v['t'] for v in ts if v['unverified_brake']), None),
                         'min_gap_time_s': min(ts, key=lambda v: v['gap'])['t'],
                         'min_predicted_feasible_count': min((sum(v['feasible'] and not v['selected'] for v in s['candidates']) for s in nominal['searches'] if s['candidates']), default=0)}
        (out/case/'README.md').write_text(f'# {NAMES[case]}\n\n'
            '[全景搜索 GIF](gif/search.gif) · [局部搜索放大 GIF](gif/search_zoom.gif) · [纯运动 GIF](gif/motion.gif)\n\n'
            '[搜索截图](plots/search_snapshot.png) · [局部放大图](plots/search_zoom.png) · [全部种子曲线](plots/metrics.png)\n\n'
            'data/seed*.json：逐周期状态、全部障碍物、净距、指令、候选及决策。\n'
            'data/original_v7_seed1.json：未加入记录接口的旧 V7，用于验证新增可视化不改变运动。\n')
    lines += ['', '低速：|v|<0.02m/s，排除起步1秒及距目标0.15m内；净距为全部重复中的最小值，其余表列为均值。',
              '超时/碰撞试验若有，会保留在原始数据及平均值中，不能将这些耗时视为成功通行时间。', '',
              '## GIF 导航', '']
    for case in cases:
        lines.append(f'- {NAMES[case]}：[全景]({case}/gif/search.gif) · [搜索放大]({case}/gif/search_zoom.gif) · [纯运动]({case}/gif/motion.gif) · [数据与曲线]({case}/README.md)')
    lines += ['', '## 搜索图怎么看', '',
              '- 蓝色实线：已经执行的中心轨迹；蓝色圆组合：实际采用的车体碰撞模型。',
              '- 绿色：本周期选中的未来中心预测轨迹；浅蓝：可行候选；浅红：碰撞检查未通过的候选。',
              '- 灰色圆：障碍物当前位置；箭头：1.5秒 CV 运动方向，不是机器人绕行方向。',
              '- 可行性同时检查未来时刻障碍物的位置，不能仅凭候选与障碍物当前圆形的平面相交判断碰撞。',
              '- 候选来自 V7 外层实际比较的跟踪/调速/转向/随机扰动/加权均值，不是绘图伪造的射线。',
              '- 不包含内层 V5 的全部随机采样和局部栅格搜索展开；内层输出只作为外层候选出现。',
              '- 每200ms导出一次，监测/回接分支候选很少属正常；无动态目标时不绘制 V7 动态候选。',
              '- 导出时按代价评估相同的两个50ms子步重建预测，所有曲线只是当前短时域，不是承诺执行的整条路径。', '',
              '## 边界与审计', '',
              f'- {trials}次正式试验，另4次原 V7 等价性复跑；4个场景的 seed 1 指令及状态逐项完全一致。',
              f'- {len(native)}个生产源码文件哈希未变；没有连接 ROS master 或发布实车指令。',
              f'- 物理碰撞试验数：{collisions}；低于0.1m余量周期数：{sum(r["margin_violation_cycles"] for r in rows)}；未验证制动周期数：{sum(r["unverified_brake_cycles"] for r in rows)}。',
              f'- 内核超过50ms周期数：{sum(r["kernel_over_50ms"] for r in rows)}；导出耗时单独记录，离线渲染不计入控制耗时。',
              '- 仿真不会因内核超时而按墙钟延迟执行；所以时延指标是诊断，不是20Hz端到端实时性认证。',
              '- 没有雷达遮挡、错分目标、速度估计错误、定位漂移及真实底盘模型误差，不能把无碰撞等同实车安全。',
              '- 未加入障碍物相互避让模型：每个障碍独立遵循预设 CV，代表几何压力测试而非行人行为仿真。',
              f'- 解析检查0～45秒内障碍物两两间距：重叠对数（按场景计）={pair_overlaps}；详见 actor_pair_audit.json。',
              '- 仅3个种子，不构成统计充分的成功率证明；本轮没有优化控制策略或缩小碰撞余量。', '',
              '## 文件与回退', '',
              '- 新构建：`builds/mppi_v7_search_01`；原构建：`builds/mppi_v7_05`，原样保留。',
              '- `source_snapshot/`、`fixture/`、`manifest.json` 固定实验脚本、参数和源码哈希。',
              '- `instrumentation_equivalence.json`、`audit.json`、`independent_audit.json` 保存检查结果。',
              '- `multi_dynamic_01` 是首次 JSON 序列化失败的目录，无正式结果，保留而不混入统计。', '']
    (out/'README.md').write_text('\n'.join(lines)+'\n')
    (out/'结果汇总.txt').write_text('\n'.join(lines)+'\n')
    (out/'aggregate.json').write_text(json.dumps(rows, indent=2)+'\n')
    (out/'decision_analysis.json').write_text(json.dumps(details, indent=2)+'\n')
    (out/'actor_pair_audit.json').write_text(json.dumps(pairs, indent=2)+'\n')
    fig, axs = plt.subplots(1, 3, figsize=(10, 3.3))
    counts = [r['count'] for r in rows]
    for ax, key, title in zip(axs, ('duration_s', 'low_speed_s', 'min_gap_m'), ('Time to termination / s', 'Mean low-speed time / s', 'Worst body clearance / m')):
        vals = [r[key] for r in rows]; ax.bar(counts, vals, color='#4589a8', width=.55)
        for x, v in zip(counts, vals): ax.text(x, v, f'{v:.3f}' if key == 'min_gap_m' else f'{v:.2f}', ha='center', va='bottom', fontsize=9)
        ax.set(title=title, xlabel='Moving obstacles', xticks=counts, ylim=(min(0, min(vals)*1.3), max(vals)*1.2))
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)
        if key == 'min_gap_m': ax.axhline(.1, color='.5', ls='--', lw=1)
    fig.suptitle('V7 SIL | 3 seeds | reached: '+', '.join(f'{r["count"]} obs {r["reached"]}/3' for r in rows), fontsize=11)
    fig.tight_layout(); fig.savefig(out/'summary.png', dpi=150); plt.close(fig)
    gifs = list(out.glob('*/gif/*.gif'))
    for gif in gifs:
        with Image.open(gif) as im:
            assert im.n_frames > 1; im.seek(im.n_frames-1); im.load()
    result = {'passed': True, 'trials': trials, 'readable_gifs': len(gifs),
              'checked_selected_rollouts': selected, 'production_files_unchanged': len(native),
              'note': 'data/instrumentation audit, NOT a safety certification'}
    (out/'independent_audit.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'aggregate': rows, 'audit': result, 'decisions': details}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('out', type=Path); a = p.parse_args(); report(a.out)

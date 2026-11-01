#!/usr/bin/env python3
"""Concise posterior explanation of one fixed-input backend investigation."""
import argparse
import json
from pathlib import Path
import numpy as np

from rootcause_components import HERE, sha


def load_rows(path):
    return [json.loads(line) for line in path.open()]


def main(output):
    source = HERE/'results/rootcause_components_20261030_02/summary.json'
    report = json.loads(source.read_text())
    native_paths = [HERE/'results/root_cause_20261030_01'/name/'results.jsonl' for name in ('native_run_01', 'native_run_02')]
    native = [load_rows(path) for path in native_paths]
    checks = {r['label']:r for r in native[0] if r['kind'] == 'check'}
    agreements = []
    for row in report['stages']:
        label = row['run']+('_initial' if row['stage'] == 'initial' else '_last')
        expected = np.asarray(checks[label]['gradient'], float)
        rebuilt = np.asarray(row['component_gradients']['known_full_local'], float)
        error = float(np.max(abs(expected-rebuilt)/np.maximum(1., np.maximum(abs(expected), abs(rebuilt)))))
        cost = sum(row['component_costs'][k] for k in ('smooth', 'time', 'wheel'))+row['native_recorded_components']['self_collision']
        agreements.append(dict(label=label, full_gradient_max_normalized_error=error,
            full_cost_absolute_error=abs(cost-checks[label]['cost']), native_gradient_sha256=checks[label]['gradient_hash']))
    solves = []
    for path, rows in zip(native_paths, native):
        for row in rows:
            if row['kind'] != 'solve': continue
            stages = row['stages']
            lss = [int(s['line_search']) for s in stages[1:]]
            steps = [float(np.linalg.norm(np.asarray(b['x'])-np.asarray(a['x']))) for a,b in zip(stages, stages[1:])]
            passes = [s for s in stages if s['native_pass']]
            solves.append(dict(label=row['label'], source=str(path), elapsed_s=row['elapsed_s'],
                complete_evaluations=row['evaluations'], accepted_steps=row['accepted_steps'],
                mean_evaluation_ms=1000*row['elapsed_s']/row['evaluations'],
                accepted_line_search_counts=lss, max_line_search=max(lss) if lss else 0,
                accepted_steps_requiring_retries=sum(v > 1 for v in lss),
                line_search_trial_overhead=max(0, row['evaluations']-1-row['accepted_steps']),
                first_full_native_pass_s=passes[0]['elapsed_s'] if passes else None,
                first_full_native_pass_iteration=passes[0]['iteration'] if passes else None,
                final_native_pass=stages[-1]['native_pass'], final_reason=stages[-1]['native_reason'],
                final_wheel_ratios=(np.asarray(stages[-1]['wheel_max'])/[4.305,4.305,4.4625,4.4625]).tolist(),
                final_cost=stages[-1]['cost'], accepted_step_x_norms=steps,
                very_small_steps_under_1e_minus4=sum(v < 1e-4 for v in steps)))
    nearby = checks['endpoint_last_small_descent']
    item = next(s for s in report['stages'] if s['run'] == 'endpoint150_060' and s['stage'] == 'accepted')
    final = dict(schema='rootcause_components_explanation_v1', original_component_summary=str(source),
        original_component_summary_sha256=sha(source), native_input_hashes={str(path):sha(path) for path in native_paths},
        full_native_gradient_agreement=agreements, solves=solves,
        nearby_full_native_pass=nearby['native_pass'], nearby_full_native_cost=nearby['cost'],
        final_peak_sensitivity=item['top_left_alpha_coordinates'],
        coarse_fd_warning='3e-6 and 8e-7 steps are too coarse for the sensitive spatial coordinates; independent finer-step investigation converges. They are not evidence of a native gradient bug.',
        scope='One captured request only. Full native replay is frozen ROS numerical evaluation, not online delivery, Gazebo or hardware. No aggregate success-rate conclusion.',
        source_sha256=sha(__file__))
    output.mkdir(parents=True, exist_ok=False)
    (output/'summary.json').write_text(json.dumps(final, indent=2)+'\n')
    lines = [
        '这次排查的是同一条失败请求，不能代表所有场景的总体通过率。',
        '',
        '1. 当前候选不是根本无解，也不是只能靠放宽轮子限制才能通过。',
        '原来0.6秒结束的首尾加时候选，左轮加速度峰值4.712，验收线4.4625。',
        '保持边界、限速和地图，沿当前真实目标的下降方向只挪动一个很小的步长，完整原生轮约束和3D碰撞检查都通过。',
        '完整原生成本868.911→831.140，说明这个位置还有既降低目标又满足验收的方向。',
        '',
        '2. 首尾加时解决了一部分初始化问题，但0.6秒仍会在修复完成前截断。',
        '同一首尾加时输入给1.2秒预算，第一次完整原生通过出现在约0.709秒，约0.886秒正常结束。',
        '原始初始化给3秒预算，约2.759秒正常结束，轮约束通过，但仍因底盘碰撞被拒。',
        '所以只增加预算不保证交付；初始化影响最后进入什么样的解。',
        '',
        '3. 剩余轮加速度对最开始的内点和换向点位置很敏感。',
        '在这条候选上，个别位置变量改变1微米，左轮加速度峰值的局部变化可达约0.01～0.02 rad/s²。',
        '优化器需要同时协调位置和时间，而不只是把所有运动时间拉长。',
        '',
        '4. 不能再把“加时”理解成必然减小轮加速度。',
        '固定起终点速度/加速度后重新生成曲线，全片运动时间再拉长3%，轮加速度峰值反而4.712→4.814；低速惩罚192→514。',
        '这是曲线形状也被重新改变，不是把同一条曲线慢放。',
        '',
        '5. 没有确认梯度公式出错。',
        '现有MINCO分项探针与完整原生回放的成本、梯度可以复原一致。',
        '粗扰动曾出现百分比误差，进一步缩小扰动后收敛到原生解析梯度；不能把粗步长误差当程序错误。',
        '',
        '6. 本输入的固定机械臂仍产生一项随总运动时间累加的自碰撞软惩罚，约2.385/秒。',
        '它把时间方向的压力从1提高到约3.385，但这个量是否是交付失败主因，需要结合主实验的去除对照判断。',
        '',
        '每次真实目标评估约15毫秒，0.6秒能做约40次评估。下面列出同输入求解的线搜索负担：']
    for row in solves:
        lines.append('%s：%.1f毫秒；%s次评估，%s个接受步；%s个接受步需要重试，单步最多%s次线搜索；最终%s。' % (
            row['label'],1000*row['elapsed_s'],row['complete_evaluations'],row['accepted_steps'],
            row['accepted_steps_requiring_retries'],row['max_line_search'],row['final_reason']))
    lines += ['', '这些是冻结同输入的后端求解结果，不是完整在线轨迹交付耗时，也没有在实车发送控制。']
    (output/'分项排查说明.txt').write_text('\n'.join(lines)+'\n')
    print(output)
    print(agreements)
    for row in solves: print(row['label'],row['max_line_search'],row['accepted_steps_requiring_retries'],row['line_search_trial_overhead'],row['first_full_native_pass_s'])


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    main(args.out.resolve())

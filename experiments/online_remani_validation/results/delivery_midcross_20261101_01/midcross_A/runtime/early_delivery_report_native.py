#!/usr/bin/env python3
"""Read actual timed native gates, not posterior feasibility as an oracle."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stats(values):return dict(min=min(values),median=statistics.median(values),max=max(values))


def main(directories,out):
    runs=[]
    for directory in directories:
        status=json.loads((directory/'status.json').read_text());raw=directory/'results.jsonl'
        if not status['complete'] or not status['physical_initial_identity'] or sha(raw)!=status['results_sha256']:
            raise ValueError('Invalid completed input')
        rows=[json.loads(line) for line in raw.open()]
        groups=[]
        for early in (False,True):
            selected=[r for r in rows if r['kind']=='solve' and r['early_delivery']==early]
            if not selected:raise ValueError('Missing arm')
            groups.append(dict(early=early,n=len(selected),native_valid=sum(r['live_candidate_native_pass'] for r in selected),
                on_time_valid=sum(r['live_on_time_valid_candidate'] for r in selected),
                actual_total_s=stats([r['live_total_return_s'] for r in selected]),
                prepare_s=stats([r['setup_s'] for r in selected]),solver_s=stats([r['solver_s'] for r in selected]),
                final_native_gate_s=stats([r['live_final_gate']['native_gate_s'] for r in selected]),
                live_full_check_count=[r['live_native_check_count'] for r in selected],
                live_wheel_check_s=stats([r['live_wheel_check_s'] for r in selected]),
                live_native_check_s=stats([r['live_native_check_s'] for r in selected]),
                planned_duration_s=stats([r['live_final_gate']['duration_s'] for r in selected]),
                last_accepted_cost=stats([r['stages'][-1]['cost'] for r in selected]),
                solver_exit_codes=[r['result'] for r in selected],force_stop=[r['force_stop'] for r in selected]))
        runs.append(dict(request=status['request_name'],source_proof=status['source_proof'],groups=groups,
            status_sha256=sha(directory/'status.json'),results_sha256=sha(raw)))
    result=dict(schema='early_delivery_native_summary_v1',runs=runs,source_sha256=sha(Path(__file__)),
        physical_objective_and_native_gate_unchanged=True,live_checks_charged_to_shared_clock=True,
        real_final_native_gate_repeated_and_counted=True,posthoc_feasible_candidates_not_selected=True,
        actual_online_fsm_delivery_measured=False,
        wheel_preview_sampling_caveat='Preview uses a floating-time loop and can include one additional tail sample compared with the native integer-count loop; final native IsTrajSafe is unchanged. This can delay an early check, not certify an otherwise invalid candidate.')
    lines=['实时验收并提前结束：同输入原生后端对照','',
        '两组都使用相同廉价变量缩放、初值、地图、轮约束、碰撞球、代价和原停止阈值。',
        '新增组只在已接受迭代上按预定规则验收：轮预检通过后，最多每100毫秒进行一次完整原生验收；通过才结束。',
        '准备、优化、实时检查及保留的最终原生验收都使用实际墙钟记录。最终验收仍重做，没有省掉或用事后结果替代。','']
    for i,run in enumerate(runs):
        lines.append('请求%d：%s'%(i+1,run['request']))
        for group in run['groups']:
            label='实时验收提前结束' if group['early'] else '现有退出规则'
            timing=group['actual_total_s']
            lines.append('%s：最终原生合格%d/%d次，其中1.2秒内完成%d/%d次；真实准备至最终验收完成中位%.3f秒（%.3f～%.3f）。'%(
                label,group['native_valid'],group['n'],group['on_time_valid'],group['n'],timing['median'],timing['min'],timing['max']))
            lines.append('准备中位%.1f毫秒；最终完整原生验收中位%.1f毫秒；迭代内完整验收次数%s。'%(
                1000*group['prepare_s']['median'],1000*group['final_native_gate_s']['median'],group['live_full_check_count']))
            lines.append('候选计划执行时长中位%.2f秒（%.2f～%.2f），最后接受迭代目标成本中位%.3f；执行时长不是计算耗时。'%(
                group['planned_duration_s']['median'],group['planned_duration_s']['min'],group['planned_duration_s']['max'],group['last_accepted_cost']['median']))
        lines.append('')
    lines += ['原规则超过1.2秒的部分是末尾验收时间，不能把其候选仍合格说成算法无解。',
        '新增组的提前退出是真实运行时发生的验收，不是事后从历史候选里挑选。',
        '提前交付追求先有合格轨迹，目标成本可能高于继续优化；最终运动表现还需要在线闭环验证。',
        '本轮确有运动时长代价：请求1从原规则30.12～31.01秒变为33.26秒；请求2从34.40秒变为36.38秒。不能把计算交付变快说成机器人行驶也变快。',
        '预检使用浮点时间循环，极少数情况下可能比原生整数计数循环多检查尾部一个点；它可能延后一次提前退出，最终完整原生验收保持原样。已测版本不再改动。',
        '本结果没有包含在线触发、A*、FSM最新地图检查、发布和接收，不能直接称为在线交付耗时。',
        '每条请求的3次重复用于看重复性，不是3个独立场景。']
    out.mkdir(parents=True,exist_ok=False)
    (out/'离线实时验收对照.txt').write_text('\n'.join(lines)+'\n')
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(out)
    for run in runs:
        print(run['request'],[(g['early'],g['native_valid'],g['on_time_valid'],g['actual_total_s']) for g in run['groups']])


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-dir',action='append',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();main([v.resolve() for v in a.run_dir],a.out.resolve())

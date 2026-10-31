#!/usr/bin/env python3
"""Small posterior report for the two-arm frozen native scaling experiment."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np

from render_dual_timescale import FONT

HERE=Path(__file__).resolve().parent
ROOT=HERE/'results/cheap_scaling_20261030_01'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def describe(values):
    return None if not values else dict(min=min(values),median=statistics.median(values),max=max(values))


def parse_run(directory):
    status_path=directory/'status.json'
    status=json.loads(status_path.read_text())
    if not status.get('complete') or not (status.get('initial_native_hash_identity') or status.get('initial_native_physical_identity')):
        raise ValueError('Incomplete or unequal-initial-state run: '+str(directory))
    raw=directory/'results.jsonl'
    if sha(raw)!=status['results_sha256']:
        raise ValueError('Completed native output hash changed')
    rows=[json.loads(line) for line in raw.open()]
    check=next(row for row in rows if row['kind']=='check')
    identity={}
    if status.get('signed_zero_identity_proof'):
        proof_path=Path(status['signed_zero_identity_proof'])
        if sha(proof_path)!=status['signed_zero_identity_proof_sha256']:raise ValueError('Identity proof changed')
        proof=json.loads(proof_path.read_text())
        if not proof['passed'] or proof['results_sha256']!=sha(raw):raise ValueError('Invalid identity proof')
        identity={row['label']:row for row in proof['checks']}
    arms=[]
    for row in rows:
        if row['kind']!='solve':continue
        equivalent=identity.get(row['label'],{})
        x_same=row['initial_native_x_hash']==check['x_hash'] or (
            equivalent.get('numeric_x_equal') and equivalent.get('differences_only_signed_zero') and
            equivalent.get('observed_x_hash')==row['initial_native_x_hash'] and equivalent.get('source_x_hash')==check['x_hash'])
        if not x_same or row['initial_native_gradient_hash']!=check['gradient_hash'] or row['initial_native_cost']!=check['cost']:
            raise ValueError('Physical initialization mismatch')
        timeout_selected=row['existing_timeout_policy_selected']
        if timeout_selected:
            selected=row['existing_timeout_policy_audit']
            selected_time=row['prepare_solve_timeout_candidate_reconstruct_gate_s']
            scope='existing timeout policy candidate, reconstructed offline'
        else:
            selected=row['returned_audit']
            selected_time=row['prepare_solve_reconstruct_gate_s']
            scope='normal returned x reconstructed offline, not actual online publication'
        timeout_without_usable=(row['force_stop']==3 and not timeout_selected)
        terminal_solver_failure=row['result']<0 and not timeout_selected
        passed=bool(selected['native_pass'] and not timeout_without_usable and not row['setup_exhausted'] and not terminal_solver_failure and row['force_stop']!=2)
        reason='terminal_solver_failure' if terminal_solver_failure else selected['native_reason']
        historical=[stage for stage in row['stages'] if stage['native_pass']]
        first=historical[0]['prepare_plus_solver_elapsed_s'] if historical else None
        arms.append(dict(label=row['label'],cheap=row['cheap_scaling'],repeat=row['repeat'],
            order_index=row['order_index'],setup_s=row['setup_s'],solver_s=row['solver_s'],
            budget_elapsed_s=row['budget_elapsed_s'],budget_s=row['total_budget_s'],
            budget_overrun_s=max(0.,row['budget_elapsed_s']-row['total_budget_s']),
            selected_candidate_native_pass=passed,selected_candidate_scope=scope,
            selected_candidate_reason=reason,selected_candidate_total_s=selected_time,
            selected_candidate_planned_duration_s=selected['duration_s'],
            selected_reconstruction_s=selected['trajectory_reconstruction_s'],selected_native_gate_s=selected['native_gate_s'],
            returned_native_pass=row['returned_audit']['native_pass'],
            latest_accepted_native_pass=None if row['latest_accepted_audit'] is None else row['latest_accepted_audit']['native_pass'],
            returned_equals_latest_accepted=row['returned_equals_latest_accepted'],timeout_policy_selected=timeout_selected,
            posthoc_first_feasible_prepare_plus_solver_s=first,
            earlier_feasible_but_final_rejected=first is not None and not passed,
            native_solver_evaluations=row['native_solver_evaluations'],setup_evaluations=row['setup_evaluations'],
            accepted_steps=row['accepted_steps'],solver_result=row['result'],force_stop=row['force_stop'],terminal_solver_failure=terminal_solver_failure,
            initial_native_cost=row['initial_native_cost'],initial_native_gradient_hash=row['initial_native_gradient_hash']))
    grouped=[]
    for cheap in (False,True):
        group=[row for row in arms if row['cheap']==cheap]
        if not group:raise ValueError('Missing arm')
        representative=sorted(group,key=lambda row:row['selected_candidate_total_s'])[(len(group)-1)//2]
        grouped.append(dict(cheap=cheap,n=len(group),valid=sum(row['selected_candidate_native_pass'] for row in group),
            median_total_trial_components=dict(setup_s=representative['setup_s'],solver_s=representative['solver_s'],
                reconstruction_and_gate_s=representative['selected_reconstruction_s']+representative['selected_native_gate_s']),
            setup_s=describe([row['setup_s'] for row in group]),solver_s=describe([row['solver_s'] for row in group]),
            selected_total_s=describe([row['selected_candidate_total_s'] for row in group]),
            selected_planned_duration_s=describe([row['selected_candidate_planned_duration_s'] for row in group]),
            selected_reconstruction_and_gate_s=describe([row['selected_reconstruction_s']+row['selected_native_gate_s'] for row in group]),
            posthoc_first_feasible_s=describe([row['posthoc_first_feasible_prepare_plus_solver_s'] for row in group if row['posthoc_first_feasible_prepare_plus_solver_s'] is not None]),
            native_solver_evaluations=describe([row['native_solver_evaluations'] for row in group]),
            failures=[row['selected_candidate_reason'] for row in group if not row['selected_candidate_native_pass']],
            earlier_feasible_final_rejected=sum(row['earlier_feasible_but_final_rejected'] for row in group),
            max_budget_overrun_s=max(row['budget_overrun_s'] for row in group)))
    return dict(directory=str(directory),request_name=status['request_name'],
        independent_request=status['source_proof']['independent_request'],source_proof=status['source_proof'],
        input_sha256=sha(directory/'input.json'),results_sha256=sha(raw),status_sha256=sha(status_path),
        initial_native_gradient_hash=check['gradient_hash'],groups=grouped,arms=arms,
        actual_online_publication_measured=False)


def plot(runs,target):
    font=FontProperties(fname=FONT)
    plt.rcParams['font.family']=font.get_name()
    fig,axes=plt.subplots(1,2,figsize=(10,4.4),constrained_layout=True)
    labels=[];setup=[];solve=[];gate=[];valid=[];counts=[]
    for index,run in enumerate(runs):
        tag=('请求%d'%(index+1)) if run['independent_request'] else '请求1检查点'
        for group in run['groups']:
            labels.append(tag+'\n'+('廉价缩放' if group['cheap'] else '原坐标'))
            representative=group['median_total_trial_components']
            setup.append(representative['setup_s']);solve.append(representative['solver_s'])
            gate.append(representative['reconstruction_and_gate_s'])
            valid.append(group['valid']);counts.append(group['n'])
    x=np.arange(len(labels));width=.65
    axes[0].bar(x,setup,width,color='#b6b6b6',label='尺度准备')
    axes[0].bar(x,solve,width,bottom=setup,color='#397db0',label='原生优化')
    bottom=np.asarray(setup)+solve
    axes[0].bar(x,gate,width,bottom=bottom,color='#e3a455',label='重建与原生验收')
    tops=bottom+gate
    for xx,top in zip(x,tops):axes[0].text(xx,top,'%.3f秒'%top,ha='center',va='bottom',fontsize=8)
    axes[0].axhline(1.2,color='#b53b32',lw=1,ls='--',label='准备＋优化预算1.2秒')
    axes[0].set_ylim(0,max(1.4,max(tops)*1.2));axes[0].set_ylabel('总耗时中位试次的分解 / 秒')
    axes[0].set_title('准备成本是否真正降下来');axes[0].legend(fontsize=8)
    fractions=np.asarray(valid)/counts
    axes[1].bar(x,fractions,width,color=['#397db0' if i%2 else '#999999' for i in x])
    for xx,value,n,frac in zip(x,valid,counts,fractions):axes[1].text(xx,frac+.025,'%d/%d'%(value,n),ha='center',fontsize=10)
    axes[1].set_ylim(0,1.18);axes[1].set_ylabel('最终所选候选通过比例');axes[1].set_title('同输入重复的最终验收')
    for ax in axes:
        ax.set_xticks(x);ax.set_xticklabels(labels,fontsize=8);ax.grid(axis='y',alpha=.15)
    fig.suptitle('冻结请求原生后端：物理目标、约束和停止条件一致；非在线交付',fontsize=11)
    fig.savefig(target,dpi=160);plt.close(fig)


def main(directories,out):
    runs=[parse_run(path.resolve()) for path in directories]
    result=dict(schema='cheap_scaling_offline_report_v1',runs=runs,
        independent_request_count=sum(r['independent_request'] for r in runs),
        source_sha256=sha(__file__),actual_online_publication_measured=False,
        posthoc_first_feasible_is_not_delivery=True,
        timing_scope='scale preparation + solver share 1.2 seconds; reconstruct and native gate measured separately; diagnostic motion/iterate audits excluded',
        unchanged='initial x, fixed map, effective PVAJ, wheel/3D geometry, native objective and delta=1e-3')
    out.mkdir(parents=True,exist_ok=False)
    plot(runs,out/'廉价缩放对照.png')
    lines=['廉价变量缩放：冻结输入对照','',
        '每条请求的两组均从相同初值开始，物理目标、地图、碰撞球、轮约束和停止阈值相同。',
        '尺度准备和优化共用1.2秒预算，实际原生验收耗时另计。',
        '这是冻结请求的原生后端回放，不是在线轨迹发布；重复3次不是3条独立场景。','']
    for index,run in enumerate(runs):
        lines += [('独立请求%d：'%(index+1) if run['independent_request'] else '同请求碰撞检查点：')+run['request_name']]
        for group in run['groups']:
            name='廉价缩放' if group['cheap'] else '原坐标'
            lines.append('%s：最终候选完整原生通过%d/%d次；准备中位%.1f毫秒，优化中位%.1f毫秒；准备＋优化＋重建验收中位%.3f秒。' % (
                name,group['valid'],group['n'],1000*group['setup_s']['median'],1000*group['solver_s']['median'],group['selected_total_s']['median']))
            if group['posthoc_first_feasible_s']:
                stat=group['posthoc_first_feasible_s'];lines.append('事后审计首个合格迭代：准备＋当时优化累计中位%.3f秒，范围%.3f～%.3f秒；当时未执行验收或发布。' % (stat['median'],stat['min'],stat['max']))
            if group['failures']:lines.append('失败原因：'+', '.join(group['failures']))
            lines.append('候选计划执行时长中位%.2f秒；这是规划出来的运动时长，不是本次计算耗时或实际行驶时间。'%group['selected_planned_duration_s']['median'])
            if group['earlier_feasible_final_rejected']:lines.append('有%d次曾存在事后合格迭代，但最终候选被拒；这是候选选择环节的证据，需要单独处理。'%group['earlier_feasible_final_rejected'])
        lines += ['']
    lines += ['超时组按已有“最后有效接受候选”规则另行验收；普通退出按返回变量重建验收。',
        '返回变量、最后接受变量分别保留，不能把事后找到的历史合格候选当作当时已经交付。',
        '只有在线闭环才能验证触发到轨迹接收的总延迟，以及动态环境中的有效性。']
    (out/'离线对照说明.txt').write_text('\n'.join(lines)+'\n')
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(out)
    for run in runs:
        print(run['request_name'],[(g['cheap'],g['valid'],g['n'],g['setup_s']['median'],g['selected_total_s']['median']) for g in run['groups']])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,action='append',required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();main(args.run_dir,args.out.resolve())

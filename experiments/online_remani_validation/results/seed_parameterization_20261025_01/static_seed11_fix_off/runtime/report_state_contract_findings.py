#!/usr/bin/env python3
"""Small post-run supplement: exact handoff residuals and safety-model units."""
import argparse
import json
import math
from pathlib import Path
import numpy as np

from report_domain_recovery import read_records, digest, assert_plot_quiet
from report_state_contract import source_stamp, identity_key, state_matches_identity
from dynamic_scenarios import actors_at, body_clearance


def analyze(root):
    report=json.loads((root/'report/summary.json').read_text())
    assert_plot_quiet(report)
    trials=[]
    for trial in report['trials']:
        directory=Path(trial['source'])
        raw_cycles,_=read_records(directory,'state_mppi_cycles.jsonl')
        by_cycle={row['cycle_id']:row for row in raw_cycles}
        if len(by_cycle)!=len(raw_cycles):raise ValueError('Duplicate cycle identity')
        body=np.loadtxt(directory/'geometry.csv',delimiter=',')
        native,_=read_records(directory,'dual_timeline.jsonl')
        failures,_=read_records(directory,'failure_trace.jsonl')
        requests=[]
        for request in trial['native_chain']['requests']:
            stages=request['stages'];selected=(stages['selected_boundary']['record'] or {})
            solver=(stages['solver_return']['record'] or {})
            publication=(stages['reference_publish']['record'] or {})
            stamp=source_stamp(request['raw_odom_identity'])
            metrics=request['metrics'];first=metrics['first_kernel_cycle_id']
            residual=None
            if first is not None:
                cycle=by_cycle[first]
                analyzed=[r for r in trial['python_chain']['rows'] if r['cycle_id']==first]
                if (len(analyzed)!=1 or analyzed[0]['solve']['exact_message_observed'] is not True or
                    analyzed[0]['reference']['complete_exact_reference_chain'] is not True or
                    state_matches_identity(cycle['solve_state'],cycle['solve_odom_identity']) is not True):
                    raise ValueError('No exact copied-state/reference proof')
                p=publication['planner_p'];state=cycle['solve_state']
                dyaw=state[2]-publication['planner_yaw']
                residual=dict(cycle_id=first,solve_odom_identity=cycle['solve_odom_identity'],
                    reference_initial_position=p[:2],actual_copied_state=state,
                    translation_m=math.hypot(p[0]-state[0],p[1]-state[1]),
                    yaw_rad=math.atan2(math.sin(dyaw),math.cos(dyaw)))
            snapshot=request.get('backend_snapshot') or {}
            field=snapshot.get('independent_raw_static')
            world=None
            if field is not None:
                pose=[selected['raw_odom_x'],selected['raw_odom_y'],selected['raw_odom_yaw'],
                      selected['raw_odom_body_vx'],0.]
                # This is same-pose analytic comparison, NOT nearest state.csv.
                elapsed=stamp-1000.-trial['outcome']['motion_anchor']
                world=body_clearance(pose,body,actors_at(trial['scene'],elapsed))
            backend=stages['backend_fixed']['record'] or {}
            matched=[r for r in failures if str(r.get('input_hash'))==backend.get('input_hash')]
            accepts=[r for r in native if r.get('request_id')==request['request_id'] and
                     r.get('event')=='stage_end' and r.get('stage')=='live_acceptance']
            requests.append(dict(request_id=request['request_id'],raw_identity=request['raw_odom_identity'],
                selected_age_s=metrics['selected_raw_odom_age_s'],
                solver_return_age_s=solver['capture_sim']-stamp if solver and stamp is not None else None,
                publish_age_s=publication['capture_sim']-stamp if publication and stamp is not None else None,
                publish_lateness_s=metrics['publish_lateness_s'],
                live_acceptance_wall_ms=accepts[0]['wall_ms'] if len(accepts)==1 else None,
                first_kernel_handoff=residual,signed_speed=metrics['signed_speed'],gears=metrics['gears'],
                frozen_field_static=field,same_pose_analytic_physical_m=world,
                failure_reason=matched[0].get('reason') if len(matched)==1 else None,
                failure_input_hash=backend.get('input_hash')))
        trials.append(dict(trial=trial['trial'],signed_low_speed=trial['signed_low_speed'],
            result=trial['outcome'],requests=requests))
    build=Path(report['trials'][0]['build'])
    contract_sources={str(build/'source'/name):digest(build/'source'/name)
        for name in ('online_kernel.hpp','online_api.cpp','coupling_bridge.hpp')}
    return dict(scope='static diagnostic; not same-input performance comparison or hardware validation',
        source_report_sha256=digest(root/'report/summary.json'),reporter_sha256=digest(Path(__file__)),
        trials=trials,limitations=report['limitations'],
        contract_sources_sha256=contract_sources,
        safety_contract_note='Geometry/field/margin match, but static margin-restoration admissibility differs: fast rollout may monotonically restore positive physical clearance below full margin; slow fixed-boundary candidate rejects any margin gap including t=0. Do not silently relax either safety gate.',
        model_note='Analytic world physical distance and conservative gridded-field distance are different quantities; 0.10m gate is applied to the latter.')


def text_report(data):
    lines=['起点—交付一致性实验：核心结论','',
        '两组静态SIL均到达，未记录投影碰撞，安全门、预算、轮速与MPPI库均未改。',
        '两组闭环输入不同，不能用6次/4次请求差异证明方向策略提升了速度。','']
    for trial in data['trials']:
        name='ON' if trial['signed_low_speed'] else 'OFF'
        lines.append(name+'：')
        for row in trial['requests']:
            v=row['signed_speed'];h=row['first_kernel_handoff']
            lines.append('  请求%d：raw v=%+.6f，processed=%+.6f，MINCO=%s；失败原因=%s。'%(
                row['request_id'],v['raw_body_vx'],v['processed'],str(v['minco_head']),row['failure_reason']))
            if h and row['request_id']>1:
                lines.append('    发布时原始起点已过去%.3fs；相对首次实际MPPI输入位置差%.2fcm、朝向差%+.2f度；原生验收耗时%.1fms。'%(
                    row['publish_age_s'],100*h['translation_m'],math.degrees(h['yaw_rad']),row['live_acceptance_wall_ms']))
            field=row['frozen_field_static'];world=row['same_pose_analytic_physical_m']
            if field and field.get('gap',0.)<0.:
                lines.append('    固定起点：保守场物理净距%.2fcm，小于10cm要求；同一姿态解析世界净距%.2fcm，不是已经发生物理碰撞。'%(
                    100*field['physical'],100*world))
        lines.append('')
    lines.extend(['已确认：旧分支可把低速倒退改成向前初速度；本批ON未在非零低速区间发起请求，尚不能声称修复已线上贯穿。',
        '已确认：选起点时odom较新，但同步求解和验收期间快层继续移动，新参考交付时起点已经落后。',
        '已确认：部分固定起点在保守场内先天不满足10cm；只优化内部变量无法让这个固定点合格。',
        '关键协同缺口：统一了几何、距离场和10cm参数，但没有统一可行性/恢复规则。',
        '  快层：仅无动态目标、物理净距仍为正时，允许从余量不足区单调退出；执行rollout末尾必须恢复完整余量，并独立检验正物理净距刹车备份。',
        '  慢层：固定起点也必须立即满足10cm，t=0不合格就无法交付。因此快层可以恢复，不等于慢层当前固定边界可行。',
        '  此结论来自冻结源码和同一场的端点复核，不等于已证明它是每次超时的唯一原因；本轮没有放宽任何安全门。',
        '场的保守扣减含sqrt(2)*0.05≈7.07cm及栅格化误差；不能将场内距离直接当解析世界距离。',
        '本批失败是预算超时和一次前端无路径，没有轮速不合格拒收；不据此否定历史轮速问题。',
        '下一步：先做同一低速倒退输入的确定性贯穿验证，再检验交付时刻与可行起点契约；不继续盲调罚权或简单增加预算。',
        '',data['scope'],'实车src/devel未改；全部记录ROS进程已退出、11441关闭。本轮未安排关机。'])
    return '\n'.join(lines)+'\n'


def plot(data,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(10.6,3.5))
    reverse=[r for t in data['trials'] if not t['signed_low_speed'] for r in t['requests']
        if -.05<r['signed_speed']['raw_body_vx']<-.0001]
    if reverse:
        row=reverse[0];v=row['signed_speed']
        axes[0].bar(['Measured','Processed','MINCO'],[v['raw_body_vx'],v['processed'],v['minco_head']],color=['#377eb8','#e78b37','#e78b37'])
        axes[0].axhline(0,color='.4',lw=.7);axes[0].set_ylabel('Signed velocity (m/s)')
        axes[0].set_title('Captured reverse mismatch')
    deliveries=[]
    for trial in data['trials']:
        values=[r for r in trial['requests'] if r['first_kernel_handoff'] and r['request_id']>1]
        if len(values)==1:deliveries.append(('ON' if trial['signed_low_speed'] else 'OFF',values[0]))
    if deliveries:
        axes[1].bar([name for name,_ in deliveries],[100*r['first_kernel_handoff']['translation_m'] for _,r in deliveries],color='#59a89c')
        for index,(_,row) in enumerate(deliveries):
            axes[1].text(index,100*row['first_kernel_handoff']['translation_m']+.3,'age %.2fs'%row['publish_age_s'],ha='center',fontsize=9)
        axes[1].set_ylim(0,12);axes[1].set_ylabel('Initial anchor to actual MPPI input (cm)')
        axes[1].set_title('Handoff residual')
    margins=[]
    for trial in data['trials']:
        rows=[r for r in trial['requests'] if r['frozen_field_static'] and r['frozen_field_static']['gap']<0.]
        if rows:margins.append(('ON' if trial['signed_low_speed'] else 'OFF',rows[0]))
    for index,(name,row) in enumerate(margins):
        axes[2].bar(index-.17,100*row['frozen_field_static']['physical'],.32,color='#e78b37',label='Grid field' if index==0 else None)
        axes[2].bar(index+.17,100*row['same_pose_analytic_physical_m'],.32,color='#377eb8',label='Analytic world' if index==0 else None)
    axes[2].axhline(10,color='black',lw=.8,ls='--',label='10cm field gate')
    axes[2].set_xticks(range(len(margins)));axes[2].set_xticklabels([n for n,_ in margins])
    axes[2].set_ylabel('Same-pose physical clearance (cm)');axes[2].set_title('Two distance definitions')
    axes[2].legend(fontsize=8)
    for ax in axes:ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Static diagnostics — different closed-loop requests, not a performance claim',fontsize=10)
    fig.tight_layout(rect=(0,0,1,.93));fig.savefig(out/'key_findings.png',dpi=150);plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();root=args.root.resolve();out=args.out.resolve()
    if out.exists() or out==root or out in root.parents:parser.error('Use a new report directory')
    data=analyze(root);out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(data,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    (out/'核心结论.txt').write_text(text_report(data))
    plot(data,out);print(out)

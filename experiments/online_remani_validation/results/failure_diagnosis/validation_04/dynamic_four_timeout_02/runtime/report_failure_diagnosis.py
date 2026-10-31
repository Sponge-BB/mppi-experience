#!/usr/bin/env python3
"""Audit and plot causal diagnostics; never count paused runs as navigation trials."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import numpy as np
from report import plt
from report_resource_comparison import lines

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
REASONS={'budget_timeout':'预算超时','left_wheel_speed':'左轮速拒绝','right_wheel_speed':'右轮速拒绝',
    'left_wheel_acceleration':'左轮加速度拒绝','right_wheel_acceleration':'右轮加速度拒绝',
    'planning_collision':'碰撞拒绝','ok':'验收通过','nonfinite':'非有限值'}


def audit_case(path):
    trace=lines(path/'failure_trace.jsonl');snapshot=[r for r in trace if r['kind']=='snapshot']
    finish=[r for r in trace if r['kind']=='snapshot_end'];rows=[r for r in trace if r['kind']=='solve' and r['label']!='normal']
    if len(snapshot)!=1 or len(finish)!=1 or not finish[0]['stable']:raise RuntimeError('invalid snapshot '+str(path))
    if len(rows)!=12:raise RuntimeError('incomplete paired matrix '+str(path)+': '+str(len(rows)))
    base=[r for r in rows if r['wheel_limit']==3.28]
    checks=dict(input_fingerprint_identical=len({r['input_hash'] for r in rows})==1,
        initial_objective_identical=len({r['first_cost'] for r in base})==1,
        initial_gradient_identical=len({r['first_gradient_hash'] for r in base})==1,
        ros_time_identical=len({r['sim'] for r in rows}|{r['sim_end'] for r in rows})==1,
        map_generations_identical=len({(r['raw_seq'],r['esdf_seq']) for r in rows})==1,
        balanced_budgets=collections.Counter(r['budget_s'] for r in rows if r['label'].startswith('budget_'))=={.6:2,1.5:2,3.:2},
        profile_off_controls=sum(not r['profile_enabled'] for r in rows)==4,
        all_inputs_finite=all(np.isfinite(r['first_cost']) and np.isfinite(r['wall_ms']) for r in rows),
        thread_time_consistent=all(0<=r['thread_cpu_ms']<=r['wall_ms']+2 for r in rows))
    if not all(checks.values()):raise RuntimeError('request mismatch '+str(path)+' '+str(checks))
    status=json.loads((path/'diagnosis_status.json').read_text());publishers=json.loads((path/'publisher_audit.json').read_text())
    checks['exclusive_sim_authority']=publishers.get('/sim/cmd_vel')==['/base_mppi'] and not publishers.get('/cmd_vel')
    checks['processes_ok']=status['diagnostic_complete'] and not status['process_failure']
    manifest=json.loads((path/'manifest.json').read_text());build=Path(manifest['build'])
    checks['native_binaries_match']=all(hashlib.sha256((build/'build'/name).read_bytes()).hexdigest()==expected
        for name,expected in manifest['native_binaries_sha256'].items())
    checks['wheel_resolves_present']=sum(r['wheel_limit']==3.5 and r['budget_s']==.6 for r in rows)==2
    if not all(checks.values()):raise RuntimeError('runtime audit failed')
    return dict(path=str(path),snapshot=snapshot[0],solves=rows,
        rechecks=[r for r in trace if r['kind']=='recheck'],audit=checks)


def old_timeline(source):
    result=[]
    for path in sorted(source.glob('*_*_0[12]')):
        if not (path/'manifest.json').exists():continue
        manifest=json.loads((path/'manifest.json').read_text())
        if manifest['scene']!='static':continue
        state=np.atleast_1d(np.genfromtxt(path/'state.csv',names=True,delimiter=','))
        obs=np.atleast_1d(np.genfromtxt(path/'obstacles.csv',names=True,delimiter=','))
        if not len(obs):continue
        appearance=float(obs['t'][0]);trajs=json.loads((path/'trajectories.json').read_text())
        pubs=[r for r in trajs if r['action']==1 and r['id']==1 and r['t']>=appearance]
        if not pubs:continue
        published=pubs[0]['t'];trace=lines(path/'solver_trace.jsonl');begins={r['id']:r for r in trace if r['event']=='begin'}
        requests=[];optimizer_ms=0.
        for row in trace:
            if row['event']!='end' or row['label']!='normal':continue
            begin=begins[row['id']]
            if begin['sim']-1000<appearance or row['sim']-1000>published+.1:continue
            if row['stage']=='replan':requests.append(row)
            elif row['stage']=='optimizer':optimizer_ms+=row['wall_ms']
        request_ms=sum(r['wall_ms'] for r in requests);latency=(published-appearance)*1000
        selected=state['t']>=published;dt=np.diff(state['t'],prepend=state['t'][0])
        native_actuation=manifest['resource_group']!='coupled'
        stopped=(abs(state['cmd_v'])<1e-6)&(abs(state['cmd_w'])<1e-6)&(state['time_scale']==0)&selected
        result.append(dict(trial=path.name,group=manifest['resource_group'],appearance_s=appearance,
            first_new_reference_s=published,delivery_latency_ms=latency,request_count=len(requests),
            request_wall_ms=request_ms,optimizer_wall_ms=optimizer_ms,
            outside_request_gap_ms=max(0.,latency-request_ms),
            post_delivery_zero_command_and_clock_s=float(dt[stopped].sum()) if native_actuation else None,
            stop_scope='native Tracker owns actuation' if native_actuation else 'MPPI owns actuation; native clock is advisory'))
    return result


def render(out):
    cases=[audit_case(p) for p in sorted(out.iterdir()) if p.is_dir() and (p/'diagnosis_status.json').exists()]
    if not cases:raise RuntimeError('no diagnostic solver cases')
    tracking=[]
    for p in sorted(out.glob('tracking_*')):
        if (p/'tracking_comparison.json').exists():
            latch=json.loads((p/'latch_cases.json').read_text())
            if not latch['passed']:raise RuntimeError('freeze mechanism checks failed')
            comparison=json.loads((p/'tracking_comparison.json').read_text())
            for v in comparison['variants']:
                csv=np.genfromtxt(p/(v['variant']+'.csv'),names=True,delimiter=',')
                if abs(float(csv['ey'][-1])-v['final_lateral_m'])>1e-12:raise RuntimeError('tracking column mismatch')
            tracking.append(dict(path=str(p),latch=latch,comparison=comparison))
    plan=json.loads((out/'experiment_plan.json').read_text());build=Path(plan['build'])
    bm=json.loads((build/'build_manifest.json').read_text())
    unchanged={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==expected for name,expected in bm['production_sources_sha256'].items()}
    if not all(unchanged.values()):raise RuntimeError('production sources changed')
    timeline=old_timeline(HERE/'results/resource_contention/validation_02')
    audit=dict(passed=True,solver_snapshots=len(cases),paired_resolves=sum(len(c['solves']) for c in cases),
        frozen_clock_tests=len(tracking)*5,fixed_reference_runs=sum(len(t['comparison']['variants']) for t in tracking),
        production_sources_verified=len(unchanged),production_sources_unchanged=all(unchanged.values()),
        case_audits={Path(c['path']).name:c['audit'] for c in cases})
    data=dict(scope='isolated solver requests and held-reference counterfactuals; not Gazebo, hardware or statistical navigation evaluation',
        cases=cases,tracking=tracking,previous_normal_online_timeline=timeline,audit=audit)
    (out/'diagnosis_summary.json').write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
    (out/'diagnosis_audit.json').write_text(json.dumps(audit,indent=2)+'\n')

    # One panel per immutable request; show every observation, not a spurious
    # P95 based on two repeats. Unprofiled controls quantify timing perturbation.
    fig,axes=plt.subplots(2,2,figsize=(11,7),squeeze=False)
    colors={'budget_timeout':'#bd5d50','ok':'#39886f'}
    for ax,case in zip(axes.flat,cases):
        for row in case['solves']:
            if row['wheel_limit']!=3.28:continue
            budget=row['budget_s'];profile=row['profile_enabled']
            ax.scatter(budget+(.025 if profile else -.025),row['lbfgs_wall_ms'],
                c=colors.get(row['reason'],'#c9953c') if profile else 'white',
                edgecolors=colors.get(row['reason'],'#c9953c'),s=45,marker='o' if profile else 's',zorder=3)
        ax.plot([.6,3],[600,3000],ls='--',color='#999999',lw=1)
        name=Path(case['path']).name
        ax.set_title(('静态超时请求' if 'static' in name else '四动态超时请求')+' '+name.rsplit('_',1)[-1])
        ax.set_xticks([.6,1.5,3]);ax.set_xlabel('优化预算 / s');ax.set_ylabel('L-BFGS墙钟耗时 / ms');ax.grid(alpha=.2)
    fig.suptitle('同输入预算对照：红=超时，黄=验收拒绝，绿=通过；空心方块=关闭分项计时',fontsize=11)
    fig.tight_layout();fig.savefig(out/'01_budget_comparison.png',dpi=160);plt.close(fig)

    fig,ax=plt.subplots(figsize=(10,4));names=[];parts=[];edt=[]
    for case in cases:
        rs=[r for r in case['solves'] if r['label'].startswith('budget_') and r['budget_s']==3.]
        profile={k:np.mean([r['profile'][k]['ms'] for r in rs]) for k in rs[0]['profile']}
        total=profile['cost_callback'];leaf=[profile['obstacle_cost'],profile['base_feasibility'],profile['joint_feasibility']]
        parts.append([*leaf,max(0.,total-sum(leaf))]);edt.append(profile['edt_query']/max(profile['obstacle_cost'],1e-9)*100)
        names.append(('静态' if 'static' in case['path'] else '四动态')+' '+Path(case['path']).name.rsplit('_',1)[-1])
    values=np.array(parts);values=values/values.sum(axis=1,keepdims=True)*100;left=np.zeros(len(names))
    for i,(label,color) in enumerate(zip(['障碍/自碰撞代价','底盘可行性','关节可行性','其它代价与梯度'],['#447e9a','#c9953c','#829a6c','#c1c8cd'])):
        ax.barh(names,values[:,i],left=left,label=label,color=color);left+=values[:,i]
    ax.set_xlim(0,100);ax.set_xlabel('代价回调内部耗时占比 / %（嵌套计时不重复相加）');ax.legend(loc='upper center',bbox_to_anchor=(.5,-.18),ncol=4,fontsize=9)
    ax.set_title('计算热点分解；障碍代价包含底盘、机械臂环境碰撞与自碰撞')
    fig.tight_layout();fig.savefig(out/'02_cost_breakdown.png',dpi=160,bbox_inches='tight');plt.close(fig)

    fig,ax=plt.subplots(figsize=(8,4));labels=[];values=[]
    for case in cases:
        for limit in (3.28,3.5):
            rs=[r for r in case['rechecks'] if r['wheel_limit']==limit]
            if not rs:continue
            labels.append(('静态' if 'static' in case['path'] else '动态')+Path(case['path']).name.rsplit('_',1)[-1]+' / '+str(limit))
            values.append(float(np.mean([r['success'] for r in rs])))
    ax.bar(labels,values,color=['#39886f' if v==1 else '#bd5d50' for v in values]);ax.set_ylim(0,1.25)
    ax.set_yticks([0,1]);ax.set_yticklabels(['拒绝','通过']);ax.set_xlabel('静态请求编号 / 仅改变验收轮速上限(rad/s)')
    ax.set_title('同一候选重新验收：放宽轮速是否足够？')
    fig.tight_layout();fig.savefig(out/'03_wheel_recheck.png',dpi=160);plt.close(fig)

    if tracking:
        fig,axes=plt.subplots(2,len(tracking),figsize=(6*len(tracking),6),squeeze=False)
        labels={'current':'当前执行模型','response_003':'响应滞后降至0.03s','tracker_angular_accel_2':'Tracker角加速度增至2','plant_wheel_speed_plus15pct':'仅底盘轮速增加15%',
                'consistent_yaw_feedforward':'仅替换时序一致的转向前馈'}
        for col,entry in enumerate(tracking):
            path=Path(entry['path'])
            for variant in entry['comparison']['variants']:
                state=np.genfromtxt(path/(variant['variant']+'.csv'),names=True,delimiter=',')
                axes[0,col].plot(state['t'],abs(state['ey'])*100,label=labels[variant['variant']],lw=1.4)
                axes[1,col].plot(state['t'],state['virtual_time'],lw=1.4)
            axes[0,col].axhline(5,color='#999999',ls='--',lw=1);axes[0,col].set_ylabel('横向误差绝对值 / cm')
            axes[0,col].set_title('固定原生参考 '+str(col+1));axes[0,col].legend(fontsize=8)
            axes[1,col].set_ylabel('参考轨迹推进 / s');axes[1,col].set_xlabel('固定参考实验时间 / s')
            for ax2 in axes[:,col]:ax2.grid(alpha=.2)
        fig.tight_layout();fig.savefig(out/'04_fixed_reference_tracking.png',dpi=160);plt.close(fig)

    text=['预算、轮速验收与冻结恢复：定向诊断结果','2026-10-16，Asia/Shanghai','',
        '范围：当前工程冻结V12，固定机械臂。隔离ROS数值仿真，不是Gazebo或实车。',
        '求解诊断暂停本次自己的MPPI/模拟器，固定同一新采集请求和地图，不能作为导航性能评估。',
        '两个静态、两个四动态预算超时快照，均做预算配对及轮速敏感性测试；另有关闭分项计时的对照。',
        '优化预算不含搜索/等待/失败重试；最终轨迹验收失败不等于优化超时。','',
        '一、审计',json.dumps(audit,ensure_ascii=False,indent=2),'','二、相同请求的预算和轮速对照']
    for case in cases:
        text+=['',Path(case['path']).name,'输入指纹：'+case['snapshot']['input_hash']]
        for row in case['solves']:
            text.append('%s：预算%.2fs，轮速%.2f；优化%.2fms，线程CPU %.2fms；完成代价计算%d次；%s；最终有效=%s；分项计时=%s'%(
                row['label'],row['budget_s'],row['wheel_limit'],row['lbfgs_wall_ms'],row['thread_cpu_ms'],row['iterations'],
                REASONS.get(row['reason'],row['reason']),row['success'],row['profile_enabled']))
        for row in case['rechecks']:
            text.append('同一候选只改变验收上限%.2f：%s，首个失败%s，值%.6f，轨迹相对时间%.3fs'%(
                row['wheel_limit'],'通过' if row['success'] else '拒绝',REASONS.get(row['reason'],row['reason']),row['value'],row['t']))
        r=next(r for r in case['solves'] if r['label'].startswith('budget_') and r['budget_s']==3.)
        p=r['profile'];den=max(p['cost_callback']['ms'],1e-9)
        text.append('3秒预算样本分项：障碍/自碰撞 %.2f%%；底盘可行性 %.2f%%；关节可行性 %.2f%%；ESDF查询占障碍函数 %.2f%%。'%(
            p['obstacle_cost']['ms']/den*100,p['base_feasibility']['ms']/den*100,p['joint_feasibility']['ms']/den*100,
            p['edt_query']['ms']/max(p['obstacle_cost']['ms'],1e-9)*100))
        text.append('注意：ESDF是障碍函数的子项，不能与障碍函数耗时相加。')
    text+=['','三、实际原生控制时钟与Tracker的冻结测试']
    for entry in tracking:
        text+=['',Path(entry['path']).name,'来源：'+entry['comparison']['source']]
        for case in entry['latch']['cases']:
            text.append('%s：先前冻结=%s，横误差%.3fm，当前冻结=%s；输出v=%.6f、w=%.6f'%(
                case['case'],case['prior_hold'],case['ey'],case['hold'],case['v'],case['w']))
        text.append('固定参考闭环：同一原生多项式、同一起始状态、无障碍和在线重规划；不是原历史传输时序重放。')
        for v in entry['comparison']['variants']:
            text.append('%s：首次冻结%s s，累计冻结%.2fs，末端横残差%.2fcm，参考推进%.3f/%.3fs，底盘速度限幅占比%.3f'%(
                v['variant'],v['first_freeze_s'],v['hold_seconds'],v['final_lateral_m']*100,
                v['final_virtual_time_s'],v['reference_duration_s'],v['command_clip_fraction']))
    text+=['','四、既有正常在线静态运行的时间分解（没有诊断暂停）']
    for row in timeline:
        text.append('%s：障碍出现到发布新参考%.3fs；规划请求%d次，累计请求%.3fs，其中优化%.3fs；请求之外间隙%.3fs；发布后零指令/冻结累计=%s s。'%(
            row['trial'],row['delivery_latency_ms']/1000,row['request_count'],row['request_wall_ms']/1000,
            row['optimizer_wall_ms']/1000,row['outside_request_gap_ms']/1000,row['post_delivery_zero_command_and_clock_s']))
    text+=['请求之外间隙包含触发、ESDF等待、冷却/重试和通信，未进一步拆分的部分不能全归为某一种等待。',
        'MPPI执行组原生时钟是参考观察器，不能把其冻结直接计作底盘停车。','',
        '五、结论边界',
        '轮速上限调整只在实验优化器/验收中生效，未改实车real_speed.yaml，不证明3.5rad/s已通过硬件标定。',
        '收敛后的最大轮速按5ms采样统计，不是连续时间数学最大值；原生验收采样和5%容差保持不变。',
        '同快照重复用于机制诊断，不能把两次重求解当作独立导航成功率样本。',
        '0.6/1.5/3秒预算均可能出现验收失败；是否有效必须逐条看success和reason，不能只看是否超时。',
        '原生产源码98项哈希检查不变；没有修改底盘模型、部署算法或硬件参数。']
    (out/'实验结果与分析.txt').write_text('\n'.join(text)+'\n',encoding='utf-8')
    print(json.dumps(audit,ensure_ascii=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('out',type=Path);a=p.parse_args();render(a.out.resolve())

#!/usr/bin/env python3
"""Report measured CPU, solver deadlines and paired scheduling diagnostics."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from report import plt

GROUPS=('remani','background','coupled')
LABELS={'remani':'REMANI','background':'REMANI＋后台 MPPI','coupled':'REMANI＋接入 MPPI'}
COLORS={'remani':'#697680','background':'#d09a36','coupled':'#2785a7'}
SCENES={'empty':'空旷','static':'突现静态障碍','dynamic_four':'四动态障碍'}


def lines(path):
    if not path.exists():return []
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def p95(values):return float(np.percentile(values,95)) if values else None
def mean(values):return float(np.mean(values)) if values else None


def trial(path):
    manifest=json.loads((path/'manifest.json').read_text());result=json.loads((path/'result.json').read_text())
    traces=lines(path/'solver_trace.jsonl');samples=lines(path/'cpu_samples.jsonl')
    ends=[r for r in traces if r['event']=='end' and r['label']=='normal']
    replans=[r for r in ends if r['stage']=='replan'];opts=[r for r in ends if r['stage']=='optimizer']
    begins=[r for r in traces if r['event']=='begin']
    offset=np.median([r['mono']-(r['sim']-1000) for r in begins]) if begins else samples[0]['mono']
    start=offset+result['goal_sent'];finish=offset+result['elapsed']
    samples=[s for s in samples if start<=s['mono']<=finish]
    cpu=collections.defaultdict(list);rss=collections.defaultdict(list)
    series=[]
    for s in samples:
        values={}
        for p in s['processes']:
            name=p['name']
            if name in ('base_mppi','background_mppi'):name='mppi'
            if p['cpu_pct'] is not None:cpu[name].append(p['cpu_pct']);values[name]=p['cpu_pct']
            rss[name].append(p['rss_mb'])
        cores={c['core']:c['busy_pct'] for c in s['cores']}
        if 'cpu' in cores:cpu['host_total_normalized'].append(cores['cpu'])
        cpu['busiest_core'].append(max((v for k,v in cores.items() if k!='cpu'),default=0))
        series.append(dict(t=s['mono']-start,**values))
    if manifest['resource_group']=='background':diagnostics=lines(path/'shadow_diagnostics.jsonl')
    elif (path/'mppi_diagnostics.json').exists():diagnostics=json.loads((path/'mppi_diagnostics.json').read_text())
    else:diagnostics=[]
    computed=[d for d in diagnostics if d.get('kernel_ms',0)>0 and d.get('code')==0]
    status=json.loads((path/'run_status.json').read_text())
    log=(path/'launch_stdout.log').read_text(errors='replace')
    state=np.atleast_1d(np.genfromtxt(path/'state.csv',delimiter=',',names=True))
    request_begins={r['id']:r for r in begins if r['stage']=='replan'}
    movement=[];start_gap=[]
    for r in replans:
        begin=request_begins[r['id']];t0=begin['sim']-1000;t1=r['sim']-1000
        p0=np.array([np.interp(t0,state['t'],state['x']),np.interp(t0,state['t'],state['y'])])
        p1=np.array([np.interp(t1,state['t'],state['x']),np.interp(t1,state['t'],state['y'])])
        movement.append(float(np.linalg.norm(p1-p0)))
        if r['success']:start_gap.append(float(np.linalg.norm(p1-[r['start_x'],r['start_y']])))
    row=dict(path=str(path),scene=manifest['scene'],group=manifest['resource_group'],reason=result['reason'],
        collision=result['collision_conservative_projection'],elapsed_s=result['elapsed']-result['goal_sent'],
        replans=len(replans),optimizer_calls=len(opts),optimizer_timeouts=sum(r.get('force_stop')==3 for r in opts),
        replan_successes=sum(r['success'] for r in replans),
        requests_without_optimizer=len(replans)-len(opts),
        invalid_seed_rejections=log.count('[SampleMani][InvalidSeed]'),
        start_not_free_rejections=sum('KinoAstar: start' in s and 'not free' in s for s in log.splitlines()),
        goal_not_free_rejections=sum('KinoAstar: goal' in s and 'not free' in s for s in log.splitlines()),
        final_constraint_rejections=log.count('is not feasible at relative time'),
        during_replan_motion_max_m=max(movement,default=0),
        successful_reference_start_gap_max_m=max(start_gap,default=0),
        replan_wall_p95_ms=p95([r['wall_ms'] for r in replans]),
        optimizer_wall_mean_ms=mean([r['wall_ms'] for r in opts]),
        optimizer_wall_p95_ms=p95([r['wall_ms'] for r in opts]),
        optimizer_thread_cpu_mean_ms=mean([r['thread_cpu_ms'] for r in opts]),
        optimizer_thread_cpu_p95_ms=p95([r['thread_cpu_ms'] for r in opts]),
        optimizer_runqueue_wait_mean_ms=mean([r['runqueue_wait_ms'] for r in opts]),
        optimizer_budget_counts=dict(collections.Counter(str(r['budget_s']) for r in opts)),
        native_cpu_mean_pct=mean(cpu['remani_planner_node']),native_cpu_p95_pct=p95(cpu['remani_planner_node']),
        mppi_cpu_mean_pct=mean(cpu['mppi']),mppi_cpu_p95_pct=p95(cpu['mppi']),
        host_cpu_mean_pct=mean(cpu['host_total_normalized']),busiest_core_p95_pct=p95(cpu['busiest_core']),
        native_rss_max_mb=max(rss['remani_planner_node'],default=0),mppi_rss_max_mb=max(rss['mppi'],default=0),
        mppi_computed_cycles=len(computed),mppi_kernel_p95_ms=p95([d['kernel_ms'] for d in computed]),
        mppi_cycle_p95_ms=p95([d['callback_ms'] for d in computed]),
        mppi_deadlines=sum(d['reason']=='deadline_miss' for d in diagnostics),
        native_process_failed=status['required_process_failed'],
        monitor_sample_mean_ms=mean([s['sample_ms'] for s in samples]),
        monitor_cpu_mean_ms=mean([s['sampler_cpu_ms'] for s in samples if 'sampler_cpu_ms' in s]))
    return row,series,opts,replans


def report(out):
    rows=[];data={};optimizers=[];replans=[]
    for path in sorted(out.iterdir()):
        if path.name=='paired_static_diagnostic' or not (path/'result.json').exists():continue
        row,series,opts,requests=trial(path);rows.append(row);data[path.name]=(row,series)
        optimizers.extend(dict(r,trial=path.name,scene=row['scene'],group=row['group']) for r in opts)
        replans.extend(dict(r,trial=path.name,scene=row['scene'],group=row['group']) for r in requests)
    isolation=[]
    for row in rows:
        if row['group']!='background':continue
        path=Path(row['path']);audit=json.loads((path/'publisher_audit.json').read_text())
        if audit['background_nonisolated_publishers']:raise RuntimeError('background contaminated graph')
        a=path.with_name(path.name.replace('_background_','_remani_'))
        identical=yaml.safe_load((path/'parameters.yaml').read_text())==yaml.safe_load((a/'parameters.yaml').read_text())
        if not identical:raise RuntimeError('native/background parameter mismatch')
        if row['mppi_computed_cycles']<20:raise RuntimeError('background was not actually computing')
        isolation.append({'background':path.name,'same_native_parameters':identical,'outputs_isolated':True,
                          'computed_cycles':row['mppi_computed_cycles']})
    pair_path=out/'paired_static_diagnostic';paired=[]
    if pair_path.exists():
        paired=[r for r in lines(pair_path/'solver_trace.jsonl') if r['event']=='end' and r['stage']=='optimizer' and r['label'].startswith('paired_')]
        if paired and len({r['input_hash'] for r in paired})!=1:raise RuntimeError('paired request changed')
    summary={'trials':rows,'paired_optimizer':paired,'isolation_audit':isolation,
             'reporter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             'cpu_convention':'100 percent = one logical CPU; native process includes map threads',
             'paired_scope':'same optimizer arrays, serialized planner callback; MPPI SIGSTOP/SIGCONT; static scene; diagnostic excluded from online performance'}
    (out/'resource_summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    (out/'optimizer_calls.json').write_text(json.dumps(optimizers,indent=2)+'\n')
    (out/'replan_calls.json').write_text(json.dumps(replans,indent=2)+'\n')
    with (out/'resource_metrics.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    scenes=list(dict.fromkeys(r['scene'] for r in rows))
    fig,axes=plt.subplots(2,len(scenes),figsize=(4.2*len(scenes),5.8),squeeze=False)
    for col,scene in enumerate(scenes):
        for group in GROUPS:
            selected=[v for name,v in data.items() if v[0]['scene']==scene and v[0]['group']==group and name.endswith('_01')]
            if not selected:continue
            row,series=selected[0]
            for ax,key in zip(axes[:,col],['remani_planner_node','mppi']):
                if key=='mppi' and group=='remani':continue
                ts=[s['t'] for s in series if key in s];ys=[s[key] for s in series if key in s]
                if ys:
                    # One-second smoothing is only for display. Raw 100ms data is retained.
                    smoothed=[np.mean(ys[max(0,i-4):i+1]) for i in range(len(ys))]
                    ax.plot(ts,smoothed,color=COLORS[group],label=LABELS[group],lw=1.5)
        axes[0,col].set_title(SCENES.get(scene,scene));axes[1,col].set_xlabel('导航目标下发后 / s')
        for ax in axes[:,col]:
            ax.grid(alpha=.15);ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False);ax.set_ylim(bottom=0)
    axes[0,0].set_ylabel('REMANI 进程 CPU / %');axes[1,0].set_ylabel('MPPI 进程 CPU / %')
    handles,labels=axes[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',ncol=3,frameon=False)
    fig.text(.5,.012,'100%＝占满一个逻辑核；曲线为第1次试验的1秒平滑值，完整原始采样均保留',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.035,1,.92]);fig.savefig(out/'01_cpu_timeline.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(12,4.2));x=np.arange(len(scenes));width=.24
    for index,group in enumerate(GROUPS):
        for ax,key in zip(axes,['native_cpu_mean_pct','mppi_cpu_mean_pct','optimizer_wall_mean_ms']):
            vals=[mean([r[key] for r in rows if r['scene']==s and r['group']==group and r[key] is not None]) for s in scenes]
            vals=[0 if v is None else v for v in vals]
            ax.bar(x+(index-1)*width,vals,width,color=COLORS[group],label=LABELS[group])
    for ax,title in zip(axes,['REMANI 平均 CPU / %','MPPI 平均 CPU / %','每次优化平均墙钟耗时 / ms']):
        ax.set_title(title,fontsize=11);ax.set_xticks(x);ax.set_xticklabels([SCENES.get(s,s) for s in scenes],fontsize=9)
        ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True);ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
    fig.legend(*axes[0].get_legend_handles_labels(),loc='upper center',ncol=3,frameon=False)
    fig.text(.5,.012,'在线闭环的规划请求可能不同；这些柱形用于描述负载，单独不能证明 CPU 竞争的因果关系',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.045,1,.9]);fig.savefig(out/'02_resource_comparison.png',dpi=160);plt.close(fig)
    if paired:
        fig,axes=plt.subplots(1,2,figsize=(9.5,4.3))
        for ax,budget in zip(axes,[.6,1.5]):
            for i,label in enumerate(['paired_mppi_paused','paired_mppi_active']):
                subset=[r for r in paired if r['label']==label and abs(r['budget_s']-budget)<1e-8]
                if not subset:continue
                for offset,key,color in [(-.16,'wall_ms','#2785a7'),(.16,'thread_cpu_ms','#8ba47b')]:
                    values=[r[key] for r in subset]
                    ax.bar(i+offset,np.mean(values),.28,color=color,label='墙钟时间' if offset<0 else '线程 CPU 时间')
                    ax.scatter(np.full(len(values),i+offset),values,s=15,color='#333333',zorder=3)
            ax.axhline(budget*1000,color='#aa6666',ls='--',lw=1)
            ax.set_title(f'相同求解请求，预算 {budget:.1f} s');ax.set_xticks([0,1]);ax.set_xticklabels(['MPPI 暂停计算','MPPI 同时计算'])
            ax.set_ylabel('耗时 / ms');ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True)
            ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
        handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles[:2],labels[:2],loc='upper center',ncol=2,frameon=False)
        fig.tight_layout(rect=[0,0,1,.9]);fig.savefig(out/'03_paired_solver.png',dpi=160);plt.close(fig)
    text=['REMANI / 后台 MPPI / 接入 MPPI 计算资源实验','日期：2026-10-15（Asia/Shanghai）','',
          '范围：固定机械臂、原生 ROS 在线数值仿真；三组使用同一份带计时探针的 V12 原生二进制、同一 MPPI 库和感知配置。',
          'A：REMANI + 原控制器；B：A + 只观察的 V12 MPPI（全部输出隔离）；C：V12 协同执行。',
          'B 的计算包含同一 MPPI 内核与发布前候选/制动复核；原生取消后保留旧参考供后台计算，运动和规划反馈仍由原控制器负责。',
          '正式试验CPU每200ms采样（具体见各manifest），100%代表一个逻辑CPU；REMANI进程CPU包含地图线程。墙钟和CPU时间采用实际时钟，不用ROS模拟时间。','',
          '逐次在线结果：']
    for r in rows:
        text.append(f"{r['scene']} / {LABELS[r['group']]} / {Path(r['path']).name.rsplit('_',1)[-1]}："
                    f"{r['reason']}，REMANI CPU均值 {r['native_cpu_mean_pct']:.1f}%，"
                    f"MPPI CPU均值 {r['mppi_cpu_mean_pct'] or 0:.1f}%，"
                    f"优化调用 {r['optimizer_calls']}，超时 {r['optimizer_timeouts']}，"
                    f"优化墙钟均值 {r['optimizer_wall_mean_ms'] or 0:.1f}ms，"
                    f"线程CPU均值 {r['optimizer_thread_cpu_mean_ms'] or 0:.1f}ms，"
                    f"运行队列等待均值 {r['optimizer_runqueue_wait_mean_ms'] or 0:.2f}ms；"
                    f"空种子{r['invalid_seed_rejections']}、起点拒绝{r['start_not_free_rejections']}、"
                    f"最终约束拒绝{r['final_constraint_rejections']}，"
                    f"重规划期间最大移动{r['during_replan_motion_max_m']:.3f}m。")
    text.extend(['','相同请求的独立诊断：'])
    for budget in [.6,1.5]:
        for label in ['paired_mppi_paused','paired_mppi_active']:
            subset=[r for r in paired if r['label']==label and abs(r['budget_s']-budget)<1e-8]
            if subset:text.append(f"预算{budget:.1f}s / {label}：n={len(subset)}，"
                f"墙钟均值{mean([r['wall_ms'] for r in subset]):.2f}ms，"
                f"线程CPU均值{mean([r['thread_cpu_ms'] for r in subset]):.2f}ms，"
                f"调度等待均值{mean([r['runqueue_wait_ms'] for r in subset]):.2f}ms，"
                f"成功{sum(r['success'] for r in subset)}，超时{sum(r.get('force_stop')==3 for r in subset)}。")
    text.extend(['','解释边界：',
        '1. A/B参数完全一致并核对了背景发布隔离；但闭环微小时序差异会改变后续请求，不能只比较任务总用时判断CPU竞争。',
        '2. 配对诊断在同一次静态地图、同一序列化规划回调内，重新初始化相同优化数组，按暂停/运行/运行/暂停顺序切换本次MPPI进程；输入指纹必须一致。',
        '3. 配对诊断的额外重复求解会改变在线运动时序，单独归档，不纳入三组在线性能。',
        '4. 空种子、起点碰撞与优化超时分别统计；任务运行超时与优化器预算超时不是一个指标。',
        '5. 程序运行的CPU负载和调度等待可以证明资源压力；层间状态、参考接续和不同规划入口还需结合每次请求与轨迹日志解释。',
        '','图：01_cpu_timeline.png、02_resource_comparison.png、03_paired_solver.png。',
        '原始数据：各试验cpu_samples.jsonl、solver_trace.jsonl、publisher_audit.json、shadow_diagnostics.jsonl/执行日志。'])
    (out/'实验结果与分析.txt').write_text('\n'.join(text)+'\n')
    print(out/'实验结果与分析.txt')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);report(p.parse_args().out.resolve())

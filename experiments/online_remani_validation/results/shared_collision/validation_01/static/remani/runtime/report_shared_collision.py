#!/usr/bin/env python3
"""Measured path/deviation and honest safety/cost attribution; concise images."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from compare_native_v9 import load
from compare_shared_panels import compare
from progress import project
from dynamic_scenarios import actors_at,body_clearance


def distance(state,reference):
    points=reference[:-1,:2];edges=reference[1:,:2]-points
    length=np.einsum('ij,ij->i',edges,edges)
    f=np.clip(np.einsum('ij,ij->i',state[:2]-points,edges)/np.maximum(1e-15,length),0,1)
    return float(np.linalg.norm(points+f[:,None]*edges-state[:2],axis=1).min())


def measured_anchor(result):
    """A missing motion anchor is not evidence that the robot ever moved."""
    for key in ('motion_anchor','goal_sent'):
        if result.get(key) is not None:return float(result[key])
    return 0.


def active_reference(references,events,t):
    ref=next((v for v in reversed(references) if v['t']<=t),None)
    event=next((v for v in reversed(events) if v['t']<=t),None)
    if ref is None or event is None:return None
    if event['event'] in ('cancel','invalid') or event['revision']!=ref['revision']:return None
    return ref


def trial_metrics(path,write=True):
    a=load(path);s=a['state'];r=a['result'];diag=a['diag'];anchor=measured_anchor(r)
    if not len(s) or not all(np.isfinite(s[n]).all() for n in ('t','x','y','yaw','v','w')):
        raise ValueError('missing/nonfinite measured base states')
    if np.any(np.diff(s['t'])<0):raise ValueError('measured state times are not ordered')
    dt=np.diff(s['t']);away=np.hypot(s['x']-3.,s['y'])>.35
    stopped=(abs(s['v'])<.02)&(abs(s['w'])<.06)&away&(s['t']>=anchor)
    longest=0.;streak=0.
    for duration,low in zip(dt,stopped[:-1]):
        streak=streak+duration if low else 0.;longest=max(longest,streak)
    errors=[];ordered=[];post=[];post_ordered=[];safe=[];times=[];revisions=[];previous=0.;last_epoch=None
    for row in s:
        ref=active_reference(a['refs'],a['events'],row['t'])
        if ref is None:continue
        ref_array=np.asarray(ref['reference']);state=np.array([row[n] for n in ('x','y','yaw','v','w')])
        if len(ref_array)<2:continue
        if ref['epoch']!=last_epoch:previous=0.;last_epoch=ref['epoch']
        previous,_,continuous_error=project(state,ref_array,np.asarray(ref['times']),previous)
        error=distance(state,ref_array);errors.append(error);ordered.append(continuous_error)
        times.append(float(row['t']-anchor));revisions.append(ref['epoch'])
        if ref['epoch']>1 and row['t']>=ref['t']+.5 and np.linalg.norm(state[:2]-[3.,0.])>.35:
            post.append(error)
            if np.isfinite(continuous_error):post_ordered.append(continuous_error)
            d=next((v for v in reversed(diag) if v['t']<=row['t']),None)
            if d and row['t']-d['t']<.15:
                decision=(d.get('local_phase') or {}).get('decision')
                if d.get('reason')=='ok' and d.get('code')==0 and decision and len(decision)>6 and decision[3]>.04 and decision[6]==0:safe.append(error)
    kernel=[d['kernel_ms'] for d in diag if d['code']==0 and d['t']>=anchor]
    callback=[d['callback_ms'] for d in diag if d['code']==0 and d['t']>=anchor]
    body=np.atleast_2d(np.loadtxt(path/'geometry.csv',delimiter=','))
    static_ref=[]
    if r['scene']=='static':
        for ref in a['refs']:
            if ref['epoch']<=1 or ref['t']<anchor+4.:continue
            static_ref.append(dict(epoch=ref['epoch'],t=ref['t']-anchor,
                min_physical_projection_clearance_m=min(body_clearance(np.array([*v[:3],v[3],0.]),body,
                    actors_at('static',ref['t']-anchor)) for v in ref['reference'])))
    producer=json.loads((path/'trajectories.json').read_text())
    appearance=4. if r['scene']=='static' else 2. if r['scene'].startswith('dynamic') else None
    fresh=[v['t']-anchor-appearance for v in producer if appearance is not None and v['action']==1 and v['id']==1 and v['t']>=anchor+appearance]
    status=json.loads((path/'run_status.json').read_text())
    result='process_failed' if status['required_process_failed'] else 'collision' if r['collision_conservative_projection'] else r['reason']
    ordered_finite=[v for v in ordered if np.isfinite(v)]
    values=dict(result=result,motion_to_end_s=r['elapsed']-anchor,min_clearance_m=r['min_clearance'],
        measured_motion_started=r.get('motion_anchor') is not None,
        metrics_time_origin='measured_motion' if r.get('motion_anchor') is not None else 'goal_or_launch_no_measured_motion',
        stop_definition='measured abs(v)<0.02 m/s AND abs(w)<0.06 rad/s, goal distance>0.35 m; recorded-state intervals',
        path_length_m=float(np.linalg.norm(np.diff(np.column_stack((s['x'],s['y'])),axis=0),axis=1).sum()),
        full_stop_away_goal_s=float(np.sum(dt*stopped[:-1])),longest_full_stop_s=float(longest),
        native_bundles=r['trajectory_bundles'],first_post_appearance_reference_s=fresh[0] if fresh else None,
        nearest_reference_p95_m=float(np.percentile(errors,95)) if errors else None,
        ordered_reference_p95_m=float(np.percentile(ordered_finite,95)) if ordered_finite else None,
        reference_error_samples=len(errors),ordered_reference_valid_samples=len(ordered_finite),
        after_new_reference_p95_m=float(np.percentile(post,95)) if post else None,
        after_new_reference_max_m=float(np.max(post)) if post else None,
        after_new_reference_samples=len(post),
        after_new_reference_ordered_p95_m=float(np.percentile(post_ordered,95)) if post_ordered else None,
        after_new_reference_ordered_valid_samples=len(post_ordered),
        safe_nominal_after_new_reference_p95_m=float(np.percentile(safe,95)) if safe else None,
        safe_nominal_after_new_reference_samples=len(safe),
        static_new_reference_clearance=static_ref,
        kernel_p95_ms=float(np.percentile(kernel,95)) if kernel else None,
        callback_p95_ms=float(np.percentile(callback,95)) if callback else None,
        reasons=dict(Counter(d['reason'] for d in diag)),required_process_failed=status['required_process_failed'])
    if write:
        (path/'reference_deviation_metrics.json').write_text(json.dumps(values,indent=2)+'\n')
        np.savez_compressed(path/'reference_deviation.npz',t=times,nearest=errors,ordered=ordered,epoch=revisions)
    return values


def summary_image(out,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    main=[r for r in rows if r['repeat']==1]
    scenes=list(dict.fromkeys(r['scene'] for r in main));variants=list(dict.fromkeys(r['variant'] for r in main))
    labels={'remani':'REMANI','v12':'V12','remani_shared':'REMANI shared','v12_shared':'V12 shared'}
    colors=['#8b8b8b','#cc8b38','#608aa1','#388561']
    fig,axes=plt.subplots(1,3,figsize=(12,3.6),constrained_layout=True)
    for ax,key,title in zip(axes,('full_stop_away_goal_s','after_new_reference_p95_m','min_clearance_m'),
                          ('Stopped away from goal (s)','Post-replan error P95 (m)','Minimum physical clearance (m)')):
        for i,variant in enumerate(variants):
            values=[next((r.get(key) for r in main if r['scene']==scene and r['variant']==variant),None) for scene in scenes]
            ax.bar(np.arange(len(scenes))+(i-1.5)*.18,[np.nan if v is None else v for v in values],width=.17,color=colors[i],label=labels[variant])
        names={'empty':'Empty','static':'Static','dynamic_single':'Crossing','dynamic_head_on_center':'Head-on','dynamic_four':'Four'}
        ax.set_xticks(range(len(scenes)));ax.set_xticklabels([names.get(scene,scene) for scene in scenes],rotation=20)
        ax.set_title(title,fontsize=10)
        for spine in ('top','right'):ax.spines[spine].set_visible(False)
        ax.grid(axis='y',alpha=.2)
        if key=='min_clearance_m':ax.axhline(0,color='#a55050',linewidth=.8)
    axes[0].legend(fontsize=8,frameon=False);fig.savefig(out/'comparison_summary.png',dpi=170);plt.close(fig)
    # Separate tracking plot; no overlapping per-candidate labels.
    fig,ax=plt.subplots(figsize=(8,3.2),constrained_layout=True)
    for variant,color in zip(variants,colors):
        f=out/'static'/variant/'reference_deviation.npz'
        if f.exists():
            data=np.load(f);ax.plot(data['t'],data['nearest'],color=color,label=labels[variant],linewidth=1.4)
    ax.set_xlabel('Time since measured motion (s)');ax.set_ylabel('Nearest reference error (m)')
    if ax.get_legend_handles_labels()[0]:ax.legend(frameon=False,ncol=2,fontsize=8)
    ax.grid(alpha=.2)
    for spine in ('top','right'):ax.spines[spine].set_visible(False)
    fig.savefig(out/'static_reference_error.png',dpi=170);plt.close(fig)


def report(out,render=True):
    out=Path(out);plan=json.loads((out/'suite_manifest.json').read_text())['specs'];rows=[];failures=[]
    def failure(stage,key,exc,row=None):
        message=str(exc)
        failures.append(dict(stage=stage,key=key,error=message))
        if row is not None:row[stage+'_error']=message
        print(stage.upper()+' ERROR',key,message,flush=True)
    for spec in plan:
        path=out/spec['key'];row=dict(spec,result='analysis_failed')
        if not (path/'result.json').exists():row['result']='incomplete';rows.append(row);continue
        # Keep the independently recorded trial outcome even if a derived
        # reference metric is unavailable. Missing audits never count as pass.
        try:
            result=json.loads((path/'result.json').read_text());status=json.loads((path/'run_status.json').read_text())
            row['result']='process_failed' if status['required_process_failed'] else 'collision' if result['collision_conservative_projection'] else result['reason']
            row.update(trial_metrics(path))
        except Exception as exc:failure('analysis',spec['key'],exc,row)
        if spec['variant'].startswith('v12'):
            row['reference_contract_pass']=False;row['execution_contract_pass']=False
            try:
                from audit_native_reference import audit as reference_audit
                reference_audit(path,render_images=False)
                row['reference_contract_pass']=json.loads((path/'reference_audit.json').read_text())['reference_contract_pass']
            except Exception as exc:failure('reference_audit',spec['key'],exc,row)
            try:
                from audit_execution import audit as execution_audit
                execution_audit(path)
                row['execution_contract_pass']=json.loads((path/'execution_audit.json').read_text())['execution_contract_pass']
            except Exception as exc:failure('execution_audit',spec['key'],exc,row)
            if render and spec['repeat']==1:
                target=path/'visualization'
                if not target.exists():
                    try:
                        from render_native_search import render as render_search
                        render_search(path,target)
                    except Exception as exc:failure('search_render',spec['key'],exc,row)
                elif not (target/'visualization_audit.json').is_file():
                    failure('search_render',spec['key'],'existing partial visualization; retained without overwrite',row)
        rows.append(row)
    if render:
        try:summary_image(out,rows)
        except Exception as exc:failure('summary_render','suite',exc)
        for scene in list(dict.fromkeys(s['scene'] for s in plan if s['repeat']==1)):
            for old,new,label_a,label_b,name in (
                ('remani','v12','REMANI（当前工程，无MPPI）','V12','original'),
                ('remani_shared','v12_shared','REMANI（统一碰撞，无MPPI）','V12（统一碰撞＋回接）','shared'),
                ('v12','v12_shared','V12 原版','V12 统一版','v12_change')):
                target=out/'comparison'/scene/name
                if not (out/scene/old/'result.json').exists() or not (out/scene/new/'result.json').exists():continue
                if target.exists():
                    if not (target/'comparison.json').is_file() or not (target/'comparison.gif').is_file():
                        failure('pair_render',scene+'/'+name,'existing partial comparison; retained without overwrite')
                    continue
                try:compare(out/scene/old,out/scene/new,target,label_a,label_b)
                except Exception as exc:failure('pair_render',scene+'/'+name,exc)
    (out/'summary.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n')
    (out/'report_status.json').write_text(json.dumps(dict(planned_trials=len(plan),reported_trials=len(rows),
        incomplete_trials=sum(row['result']=='incomplete' for row in rows),errors=failures,
        derived_report_complete=not failures and not any(row['result']=='incomplete' for row in rows)),indent=2,ensure_ascii=False)+'\n')
    lines=['# REMANI / V12：共用碰撞模型对照','',
        '固定臂、原生 REMANI 在线规划和重规划、真实检测器、滞后差速数值底盘。不是 Gazebo 或实车验收。',
        '四组区分局部执行器与原生碰撞几何变化：原 REMANI、原 V12、统一模型 REMANI、统一模型 V12。',
        '这里的 REMANI baseline 是当前工程使用原生规划器、控制器和 Tracker 并关闭 MPPI；不是未经修改的上游 REMANI-Planner。',
        '统一版共用同一投影、同一测量快照、0.10 m 硬余量及三次碰撞代价。原生优化仍使用冻结当前帧；局部控制按候选时刻做CV外推，时域策略没有被强行变成一样。',
        'V12 统一版是碰撞成本、暖启动清空、回接权重及终点模式的联合改进；本四组对照不能把全部差异单独归因于碰撞代价。',
        '局部参考改变后清空旧绕行暖启动；安全清空路线恢复参考约束，实际预测冲突仍可暂时绕行。','',
        '[对照摘要图](comparison_summary.png) · [静态参考误差](static_reference_error.png)','',
        '| 场景/版本 | 结果 | 实际静止/s | 新参考后误差P95/m | 最小物理净距/m |',
        '| --- | --- | ---: | ---: | ---: |']
    def number(value):return '—' if value is None else f'{value:.3f}'
    for row in rows:
        lines.append(f'| {row["key"]} | {row["result"]} | {number(row.get("full_stop_away_goal_s"))} | {number(row.get("after_new_reference_p95_m"))} | {number(row.get("min_clearance_m"))} |')
    lines+=['','## 每个场景的独立 GIF','']
    for scene in list(dict.fromkeys(s['scene'] for s in plan if s['repeat']==1)):
        artifacts=[('原版对照',f'comparison/{scene}/original/comparison.gif'),
            ('统一碰撞对照',f'comparison/{scene}/shared/comparison.gif'),
            ('V12改动前后',f'comparison/{scene}/v12_change/comparison.gif'),
            ('MPPI搜索',f'{scene}/v12_shared/visualization/search_zoom.gif')]
        links=[f'[{label}]({name})' if (out/name).is_file() else label+'未生成' for label,name in artifacts]
        lines.append('- '+scene+'：'+' · '.join(links))
    lines+=['','## 指标和限制','',
        '静止指标同时要求平移和角速度低，且距离目标>0.35m，避免把原地转向当作停车。',
        '新参考后误差剔除交付后0.5s和近终点阶段；额外记录无预测冲突时的误差。最近几何距离与连续分支距离均保留，不能把“贴住某条旧分支”当作按序跟踪。',
        '首个障碍出现后的新参考时差，不自动等同障碍触发重规划时延；需结合原生事件日志解释。',
        '全部失败、超时、投影碰撞和进程故障均保留；原生随机搜索未做配对种子，不作统计优势或任意场景安全声明。',
        '原生-only参考图来自实际发布多项式的回放，局部组来自实际接收事务；不伪造控制器接收确认。',
        '静态软碰撞代价作用区是0.12m，硬余量是0.10m。运动目标软代价另加 min(0.12m, 0.8s×目标速度)，且两层使用同一公式；该速度相关舒适余量不改变硬碰撞外形或引入CV不确定性膨胀。',
        '共同的0.02m软/硬边界差用于减少优化器在硬边界内收敛后被拒绝；动态软区通常大于0.12m，不能表述成所有目标恒定0.12m。',
        '控制、绘图和原始数据均按场景/版本独立归档；重绘不参与控制决策。','']
    if failures:
        lines+=['## 分析/绘图异常','',
            '异常已保留在 [report_status.json](report_status.json)，单项失败不会被伪装成成功或阻断其他报告。','']
        lines.extend('- '+item['stage']+' / '+item['key']+'：'+item['error'] for item in failures)
    (out/'README.md').write_text('\n'.join(lines)+'\n');print('REPORT',out,flush=True)
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--no-render',action='store_true')
    a=p.parse_args();report(a.out.resolve(),not a.no_render)

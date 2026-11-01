#!/usr/bin/env python3
"""Measured path/deviation and honest safety/cost attribution; concise images."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from compare_native_v9 import load,compare
from progress import project
from dynamic_scenarios import actors_at,body_clearance


def distance(state,reference):
    points=reference[:-1,:2];edges=reference[1:,:2]-points
    length=np.einsum('ij,ij->i',edges,edges)
    f=np.clip(np.einsum('ij,ij->i',state[:2]-points,edges)/np.maximum(1e-15,length),0,1)
    return float(np.linalg.norm(points+f[:,None]*edges-state[:2],axis=1).min())


def trial_metrics(path):
    a=load(path);s=a['state'];r=a['result'];diag=a['diag'];anchor=r['motion_anchor'] or r['goal_sent'] or 0.
    dt=np.diff(s['t']);away=np.hypot(s['x']-3.,s['y'])>.35
    stopped=(abs(s['v'])<.02)&(abs(s['w'])<.06)&away&(s['t']>=anchor)
    longest=0.;streak=0.
    for duration,low in zip(dt,stopped[:-1]):
        streak=streak+duration if low else 0.;longest=max(longest,streak)
    errors=[];ordered=[];post=[];safe=[];times=[];revisions=[];previous=0.;last_revision=None
    for row in s:
        ref=next((v for v in reversed(a['refs']) if v['t']<=row['t']),None)
        if ref is None:continue
        ref_array=np.asarray(ref['reference']);state=np.array([row[n] for n in ('x','y','yaw','v','w')])
        if len(ref_array)<2:continue
        if ref['revision']!=last_revision:previous=0.;last_revision=ref['revision']
        previous,_,continuous_error=project(state,ref_array,np.asarray(ref['times']),previous)
        error=distance(state,ref_array);errors.append(error);ordered.append(continuous_error)
        times.append(float(row['t']-anchor));revisions.append(ref['epoch'])
        if ref['epoch']>1 and row['t']>=ref['t']+.5 and np.linalg.norm(state[:2]-[3.,0.])>.35:
            post.append(error)
            d=next((v for v in reversed(diag) if v['t']<=row['t']),None)
            if d and row['t']-d['t']<.15:
                decision=(d.get('local_phase') or {}).get('decision')
                if decision and decision[3]>.04 and decision[6]==0:safe.append(error)
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
    values=dict(result=result,motion_to_end_s=r['elapsed']-anchor,min_clearance_m=r['min_clearance'],
        path_length_m=float(np.linalg.norm(np.diff(np.column_stack((s['x'],s['y'])),axis=0),axis=1).sum()),
        full_stop_away_goal_s=float(np.sum(dt*stopped[:-1])),longest_full_stop_s=float(longest),
        native_bundles=r['trajectory_bundles'],first_post_appearance_reference_s=fresh[0] if fresh else None,
        nearest_reference_p95_m=float(np.percentile(errors,95)) if errors else None,
        ordered_reference_p95_m=float(np.percentile([v for v in ordered if np.isfinite(v)],95)) if ordered else None,
        after_new_reference_p95_m=float(np.percentile(post,95)) if post else None,
        after_new_reference_max_m=float(np.max(post)) if post else None,
        safe_nominal_after_new_reference_p95_m=float(np.percentile(safe,95)) if safe else None,
        safe_nominal_after_new_reference_samples=len(safe),
        static_new_reference_clearance=static_ref,
        kernel_p95_ms=float(np.percentile(kernel,95)) if kernel else None,
        callback_p95_ms=float(np.percentile(callback,95)) if callback else None,
        reasons=dict(Counter(d['reason'] for d in diag)),required_process_failed=status['required_process_failed'])
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
        ax.set_xticks(range(len(scenes)));ax.set_xticklabels(['Empty','Static','Crossing','Head-on','Four'],rotation=20)
        ax.set_title(title,fontsize=10);ax.spines[['top','right']].set_visible(False);ax.grid(axis='y',alpha=.2)
        if key=='min_clearance_m':ax.axhline(0,color='#a55050',linewidth=.8)
    axes[0].legend(fontsize=8,frameon=False);fig.savefig(out/'comparison_summary.png',dpi=170);plt.close(fig)
    # Separate tracking plot; no overlapping per-candidate labels.
    fig,ax=plt.subplots(figsize=(8,3.2),constrained_layout=True)
    for variant,color in zip(variants,colors):
        f=out/'static'/variant/'reference_deviation.npz'
        if f.exists():
            data=np.load(f);ax.plot(data['t'],data['nearest'],color=color,label=labels[variant],linewidth=1.4)
    ax.set_xlabel('Time since measured motion (s)');ax.set_ylabel('Nearest reference error (m)')
    ax.legend(frameon=False,ncol=2,fontsize=8);ax.grid(alpha=.2);ax.spines[['top','right']].set_visible(False)
    fig.savefig(out/'static_reference_error.png',dpi=170);plt.close(fig)


def report(out,render=True):
    plan=json.loads((out/'suite_manifest.json').read_text())['specs'];rows=[]
    from audit_native_reference import audit as reference_audit
    from audit_execution import audit as execution_audit
    from render_native_search import render as render_search
    for spec in plan:
        path=out/spec['key'];row=dict(spec)
        if not (path/'result.json').exists():row['result']='incomplete';rows.append(row);continue
        try:
            row.update(trial_metrics(path))
            if spec['variant'].startswith('v12'):
                reference_audit(path,render_images=False);execution_audit(path)
                row['reference_contract_pass']=json.loads((path/'reference_audit.json').read_text())['reference_contract_pass']
                row['execution_contract_pass']=json.loads((path/'execution_audit.json').read_text())['execution_contract_pass']
                if render and spec['repeat']==1 and not (path/'visualization').exists():render_search(path,path/'visualization')
        except Exception as exc:row['analysis_error']=str(exc);print('ANALYSIS ERROR',spec['key'],str(exc),flush=True)
        rows.append(row)
    (out/'summary.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n')
    if render:
        summary_image(out,rows)
        for scene in list(dict.fromkeys(s['scene'] for s in plan if s['repeat']==1)):
            for old,new,label_a,label_b,name in (
                ('remani','v12','REMANI','V12','original'),
                ('remani_shared','v12_shared','REMANI（统一碰撞）','V12（统一碰撞＋回接）','shared'),
                ('v12','v12_shared','V12 原版','V12 统一版','v12_change')):
                target=out/'comparison'/scene/name
                if (out/scene/old/'result.json').exists() and (out/scene/new/'result.json').exists() and not target.exists():
                    compare(out/scene/old,out/scene/new,target,label_a,label_b)
    lines=['# REMANI / V12：共用碰撞模型对照','',
        '固定臂、原生 REMANI 在线规划和重规划、真实检测器、滞后差速数值底盘。不是 Gazebo 或实车验收。',
        '四组区分局部执行器与原生碰撞几何变化：原 REMANI、原 V12、统一模型 REMANI、统一模型 V12。',
        '统一版共用同一投影、同一测量快照、0.10 m 硬余量及三次碰撞代价。原生优化仍使用冻结当前帧；局部控制按候选时刻做CV外推，时域策略没有被强行变成一样。',
        '局部参考改变后清空旧绕行暖启动；安全清空路线恢复参考约束，实际预测冲突仍可暂时绕行。','',
        '[对照摘要图](comparison_summary.png) · [静态参考误差](static_reference_error.png)','',
        '| 场景/版本 | 结果 | 实际静止/s | 新参考后误差P95/m | 最小物理净距/m |',
        '| --- | --- | ---: | ---: | ---: |']
    def number(value):return '—' if value is None else f'{value:.3f}'
    for row in rows:
        lines.append(f'| {row["key"]} | {row["result"]} | {number(row.get("full_stop_away_goal_s"))} | {number(row.get("after_new_reference_p95_m"))} | {number(row.get("min_clearance_m"))} |')
    lines+=['','## 每个场景的独立 GIF','']
    for scene in list(dict.fromkeys(s['scene'] for s in plan if s['repeat']==1)):
        lines.append(f'- {scene}：[原版对照](comparison/{scene}/original/comparison.gif) · [统一碰撞对照](comparison/{scene}/shared/comparison.gif) · [V12改动前后](comparison/{scene}/v12_change/comparison.gif) · [MPPI搜索](../{out.name}/{scene}/v12_shared/visualization/search_zoom.gif)')
    lines+=['','## 指标和限制','',
        '静止指标同时要求平移和角速度低，且距离目标>0.35m，避免把原地转向当作停车。',
        '新参考后误差剔除交付后0.5s和近终点阶段；额外记录无预测冲突时的误差。最近几何距离与连续分支距离均保留，不能把“贴住某条旧分支”当作按序跟踪。',
        '首个障碍出现后的新参考时差，不自动等同障碍触发重规划时延；需结合原生事件日志解释。',
        '全部失败、超时、投影碰撞和进程故障均保留；原生随机搜索未做配对种子，不作统计优势或任意场景安全声明。',
        '原生-only参考图来自实际发布多项式的回放，局部组来自实际接收事务；不伪造控制器接收确认。',
        '0.12m是两层一致的软代价作用区，0.10m是硬余量，差值0.02m用于减少优化器在硬边界内收敛后被拒绝；并非额外的动态不确定性半径。',
        '控制、绘图和原始数据均按场景/版本独立归档；重绘不参与控制决策。','']
    (out/'README.md').write_text('\n'.join(lines)+'\n');print('REPORT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--no-render',action='store_true')
    a=p.parse_args();report(a.out.resolve(),not a.no_render)

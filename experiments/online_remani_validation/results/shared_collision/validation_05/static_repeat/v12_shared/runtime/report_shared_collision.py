#!/usr/bin/env python3
"""Measured path/deviation and honest safety/cost attribution; concise images."""
import argparse
from collections import Counter
import json
import os
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


def trial_metrics(path,write=True,analysis_out=None):
    """Compute from source records; optional independent output never edits them."""
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
    target=Path(analysis_out) if analysis_out is not None else path if write else None
    if target is not None:
        if analysis_out is not None:
            source_path=Path(path).resolve();target_path=target.resolve()
            if target_path==source_path or source_path in target_path.parents:
                raise ValueError('independent analysis output may not resolve into source records')
            target.mkdir(parents=True,exist_ok=True)
        (target/'reference_deviation_metrics.json').write_text(json.dumps(values,indent=2)+'\n')
        np.savez_compressed(target/'reference_deviation.npz',t=times,nearest=errors,ordered=ordered,epoch=revisions)
    return values


def outcome_matrix(rows,scenes,variants):
    """Outcome text is independent of low error/stop numbers and chart colors."""
    outcomes={(row['scene'],row['variant']):row.get('result') for row in rows if row['repeat']==1}
    names={'collision':'overlap','process_failed':'fault','analysis_failed':'n/a','incomplete':'missing'}
    return [[names.get(outcomes.get((scene,variant)),outcomes.get((scene,variant)) or 'missing')
             for scene in scenes] for variant in variants]


def summary_image(out,rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    main=[r for r in rows if r['repeat']==1]
    scenes=list(dict.fromkeys(r['scene'] for r in main));variants=list(dict.fromkeys(r['variant'] for r in main))
    labels={'remani':'REMANI','v12':'V12','remani_shared':'REMANI shared','v12_shared':'V12 shared'}
    colors=['#8b8b8b','#cc8b38','#608aa1','#388561']
    fig=plt.figure(figsize=(12,4.9),constrained_layout=True)
    grid=fig.add_gridspec(2,3,height_ratios=[3.2,1.15])
    axes=[fig.add_subplot(grid[0,i]) for i in range(3)]
    for ax,key,title in zip(axes,('full_stop_away_goal_s','after_new_reference_p95_m','min_clearance_m'),
                          ('Stopped away from goal (s)','Post-replan error P95 (m)','Minimum projected clearance (m)')):
        for i,variant in enumerate(variants):
            values=[next((r.get(key) for r in main if r['scene']==scene and r['variant']==variant),None) for scene in scenes]
            ax.bar(np.arange(len(scenes))+(i-1.5)*.18,[np.nan if v is None else v for v in values],width=.17,color=colors[i],label=labels[variant])
        names={'empty':'Empty','static':'Static','dynamic_single':'Crossing','dynamic_head_on_center':'Head-on','dynamic_four':'Four'}
        ax.set_xticks(range(len(scenes)));ax.set_xticklabels([names.get(scene,scene) for scene in scenes],rotation=20)
        ax.set_title(title,fontsize=10)
        for spine in ('top','right'):ax.spines[spine].set_visible(False)
        ax.grid(axis='y',alpha=.2)
        if key=='min_clearance_m':ax.axhline(0,color='#a55050',linewidth=.8)
    axes[0].legend(fontsize=8,frameon=False)
    status=fig.add_subplot(grid[1,:]);status.set_axis_off()
    status.set_title('Trial outcomes | overlap = conservative projected-body overlap (not certified physical collision)',fontsize=9,pad=7)
    cells=outcome_matrix(main,scenes,variants)
    table=status.table(cellText=cells,rowLabels=[labels[v] for v in variants],
        colLabels=[names.get(scene,scene) for scene in scenes],cellLoc='center',loc='center')
    table.auto_set_font_size(False);table.set_fontsize(8);table.scale(1.,1.05)
    shading={'goal':'#edf4ed','timeout':'#f7f0e3','overlap':'#f7e7e7','fault':'#eeeeee','n/a':'#eeeeee','missing':'#eeeeee'}
    for (i,j),cell in table.get_celld().items():
        cell.set_edgecolor('#dddddd');cell.set_linewidth(.5)
        if i>0 and j>=0:cell.set_facecolor(shading.get(cells[i-1][j],'white'))
    fig.savefig(out/'comparison_summary.png',dpi=170);plt.close(fig)
    # Separate tracking plot; no overlapping per-candidate labels.
    fig,ax=plt.subplots(figsize=(8,3.2),constrained_layout=True)
    for variant,color in zip(variants,colors):
        f=out/'reused_analysis/static'/variant/'reference_deviation.npz'
        if not f.exists():f=out/'static'/variant/'reference_deviation.npz'
        if f.exists():
            with np.load(f) as data:ax.plot(data['t'],data['nearest'],color=color,label=labels[variant],linewidth=1.4)
    ax.set_xlabel('Time since measured motion (s)');ax.set_ylabel('Nearest reference error (m)')
    if ax.get_legend_handles_labels()[0]:ax.legend(frameon=False,ncol=2,fontsize=8)
    ax.grid(alpha=.2)
    for spine in ('top','right'):ax.spines[spine].set_visible(False)
    fig.savefig(out/'static_reference_error.png',dpi=170);plt.close(fig)


def report(out,render=True):
    out=Path(out);suite=json.loads((out/'suite_manifest.json').read_text());plan=suite['specs'];rows=[];failures=[]
    def failure(stage,key,exc,row=None):
        message=str(exc)
        failures.append(dict(stage=stage,key=key,error=message))
        if row is not None:row[stage+'_error']=message
        print(stage.upper()+' ERROR',key,message,flush=True)
    for spec in plan:
        reused=spec.get('record_origin')=='reused_current_campaign_control'
        path=out/spec['key'];row=dict(spec,result='analysis_failed',shared_input_contract_pass=None,
            shared_input_status='not_checked',native_shared_reference_segments=None,
            local_shared_approved_records=None)
        if not (path/'result.json').exists():row['result']='incomplete';rows.append(row);continue
        # Keep the independently recorded trial outcome even if a derived
        # reference metric is unavailable. Missing audits never count as pass.
        try:
            result=json.loads((path/'result.json').read_text());status=json.loads((path/'run_status.json').read_text())
            row['result']='process_failed' if status['required_process_failed'] else 'collision' if result['collision_conservative_projection'] else result['reason']
            analysis_out=out/'reused_analysis'/spec['key'] if reused else None
            row.update(trial_metrics(path,write=not reused,analysis_out=analysis_out))
        except Exception as exc:failure('analysis',spec['key'],exc,row)
        try:
            manifest=json.loads((path/'manifest.json').read_text())
            if manifest.get('shared_collision',False):
                row['shared_input_contract_pass']=False;row['shared_input_status']='audit_failed'
                # This report runs only after run_status is committed. The
                # input audit independently checks that post-run interlock.
                from audit_shared_collision import audit as shared_input_audit
                shared=shared_input_audit(path,write=False) if reused else shared_input_audit(path)
                if reused:
                    target=out/'reused_analysis'/spec['key'];target.mkdir(parents=True,exist_ok=True)
                    (target/'shared_collision_audit.json').write_text(json.dumps(shared,indent=2)+'\n')
                native_count=int(shared.get('native_reference_segments',0))
                approved_count=int(shared.get('local_approved_records',0))
                local_required=bool(manifest.get('local_execution_owner',False)) or manifest.get('mode')!='remani'
                coverage=[]
                if not shared.get('validated_transports',0):coverage.append('no validated shared transport')
                if not native_count:coverage.append('no native shared reference publication')
                if local_required and not approved_count:coverage.append('no approved local shared execution')
                consistency=shared.get('passed') is True
                row.update(shared_input_recorded_checks_pass=consistency,
                    shared_input_contract_pass=consistency and not coverage,
                    shared_input_status='failed' if not consistency else 'insufficient_coverage' if coverage else 'checked_local_and_native_inputs' if local_required else 'checked_native_input_only',
                    shared_input_coverage_gaps=coverage,shared_input_warnings=shared.get('warnings',[]),
                    shared_input_errors=shared.get('errors',[]),
                    native_shared_reference_segments=native_count,local_shared_approved_records=approved_count,
                    native_shared_segments_over_max_age=shared.get('native_segments_over_max_age',0),
                    native_shared_input_age_s=shared.get('native_input_age_s'),
                    local_shared_input_age_s=shared.get('local_input_age_s'),
                    local_shared_input_verification='approved-record coverage present; consult audit errors, NOT independent execution safety verification' if local_required and approved_count else 'no approved local execution coverage' if local_required else 'not applicable; native-only, local execution is NOT verified',
                    shared_input_scope=shared.get('scope'))
            else:row['shared_input_status']='not_applicable'
        except Exception as exc:failure('shared_input_audit',spec['key'],exc,row)
        if spec['variant'].startswith('v12'):
            row['reference_contract_pass']=False;row['execution_contract_pass']=False
            try:
                if reused and not (path/'reference_audit.json').is_file():
                    row['reference_audit_coverage']='insufficient_coverage: no archived reference audit; reused source not modified'
                else:
                    if not reused:
                        from audit_native_reference import audit as reference_audit
                        reference_audit(path,render_images=False)
                    row['reference_contract_pass']=json.loads((path/'reference_audit.json').read_text())['reference_contract_pass']
                    if reused:row['reference_audit_coverage']='read existing current-campaign control audit; not rerun'
            except Exception as exc:failure('reference_audit',spec['key'],exc,row)
            try:
                if reused and not (path/'execution_audit.json').is_file():
                    row['execution_audit_coverage']='insufficient_coverage: no archived execution audit; reused source not modified'
                else:
                    if not reused:
                        from audit_execution import audit as execution_audit
                        execution_audit(path)
                    row['execution_contract_pass']=json.loads((path/'execution_audit.json').read_text())['execution_contract_pass']
                    if reused:row['execution_audit_coverage']='read existing current-campaign control audit; not rerun'
            except Exception as exc:failure('execution_audit',spec['key'],exc,row)
            if render and spec['repeat']==1:
                target=path/'visualization'
                if reused:
                    row['search_render_coverage']='read existing current-campaign visualization; not regenerated' if (target/'visualization_audit.json').is_file() else 'insufficient_coverage: reused visualization absent or partial; source not modified'
                elif not target.exists():
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
    if suite.get('campaign_type')=='followup_with_reused_controls':
        controls=Path(suite['controls_suite'])
        link=os.path.relpath(controls/'README.md',out)
        new_count=suite.get('new_online_trials',sum(s.get('record_origin')=='new_online_trial' for s in plan))
        reused_count=suite.get('reused_current_campaign_controls',sum(s.get('record_origin')=='reused_current_campaign_control' for s in plan))
        lines[2:2]=[
            f'本次为追加验证：{new_count}次新在线试验 + {reused_count}项同一当前研究批次已经完成的对照记录复用。不是24次全部重新运行，也不是从旧研究中选择历史成功案例。',
            f'复用对照来自[完整原始对照批次](<{link}>)；原始文件只读，新的派生分析保存在本目录 reused_analysis 中。每项 record_origin 和 original_record_path 均保留。',
            'MPPI采样种子固定但原生搜索与调度没有配对随机种子；复用记录不能被说成新运行或统计配对实验。','']
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
        '原生规划从一次新鲜快照启动并在完整优化期间冻结输入，交付参考时输入年龄可能超过0.35s。这是指导轨迹的冻结输入，不是当下执行批准；原生非优化安全查询使用最新快照，局部执行批准必须通过当前帧健康和≤0.35s新鲜度检查。',
        '继承的静态余量恢复存在明确例外：无预测障碍目标、已经进入0.10m余量带但物理净距仍为正时，可批准净距不减、最终恢复完整余量的退出轨迹；备用制动只要求保持正物理净距，不要求制动过程中退出余量带。不是允许物理碰撞，也不能宣称所有获准轨迹每一点始终≥0.10m。',
        '共享输入审计仅核对已记录的传输、模型、序号、健康和年龄，不重算碰撞或制动，也不构成安全证书。没有原生交付或局部获准执行时明确记为覆盖不足，不能把“无执行且无错误”说成执行验证通过；原生-only不验证局部执行。',
        '控制、绘图和原始数据均按场景/版本独立归档；重绘不参与控制决策。','']
    lines+=['## 共享输入审计覆盖','',
        '| 场景/版本 | 审计状态 | 输入契约 | 原生参考段 | 局部获准记录 |',
        '| --- | --- | --- | ---: | ---: |']
    for row in rows:
        if row['shared_input_status']=='not_applicable':continue
        check='通过（仅上述输入覆盖）' if row['shared_input_contract_pass'] else '未通过/覆盖不足'
        native='—' if row['native_shared_reference_segments'] is None else str(row['native_shared_reference_segments'])
        local='—' if row['local_shared_approved_records'] is None else str(row['local_shared_approved_records'])
        lines.append(f'| {row["key"]} | {row["shared_input_status"]} | {check} | {native} | {local} |')
    lines.append('')
    if suite.get('campaign_type')=='followup_with_reused_controls':
        lines+=['## 每项记录来源','',
            '| 场景/版本 | 来源 | 原始记录 |','| --- | --- | --- |']
        for spec in plan:
            original=Path(spec.get('original_record_path',out/spec['key']))
            relative=os.path.relpath(original/'result.json',out)
            lines.append(f'| {spec["key"]} | {spec.get("record_origin","unspecified")} | [原始结果](<{relative}>) |')
        lines.append('')
    if failures:
        lines+=['## 分析/绘图异常','',
            '异常已保留在 [report_status.json](report_status.json)，单项失败不会被伪装成成功或阻断其他报告。','']
        lines.extend('- '+item['stage']+' / '+item['key']+'：'+item['error'] for item in failures)
    (out/'README.md').write_text('\n'.join(lines)+'\n');print('REPORT',out,flush=True)
    return rows


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--no-render',action='store_true')
    a=p.parse_args();report(a.out.resolve(),not a.no_render)

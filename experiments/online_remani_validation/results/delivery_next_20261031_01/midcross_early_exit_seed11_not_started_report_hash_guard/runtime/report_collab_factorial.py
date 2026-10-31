#!/usr/bin/env python3
"""Read-only analysis of target x prefix trials; no ROS/control imports."""
import argparse
from collections import Counter, defaultdict
from contextlib import contextmanager, ExitStack
import csv
import gzip
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
TITLES={'A':'A 原V12逻辑','B':'B 仅前缀反馈','C':'C 仅滚动子目标','D':'D 两者结合'}


def read(path,default=None):
    path=Path(path)
    if not path.exists():
        if not path.with_name(path.name+'.gz').exists():return default
        with gzip.open(path.with_name(path.name+'.gz'),'rt') as stream:return json.load(stream)
    return json.loads(path.read_text())


def lines(path):
    path=Path(path)
    if path.exists():stream=path.open()
    elif path.with_name(path.name+'.gz').exists():stream=gzip.open(path.with_name(path.name+'.gz'),'rt')
    else:return []
    with stream:return [json.loads(line) for line in stream if line.strip()]


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


@contextmanager
def unpacked(trial):
    """Recreate only this trial's verified gzip duplicates, then release space."""
    created=[]
    try:
        for name,record in read(trial/'compression_audit.json',{}).items():
            path=trial/name;archive=trial/(name+'.gz')
            if path.exists():continue
            if sha(archive)!=record['gzip_sha256']:raise RuntimeError('archive changed: '+str(archive))
            with gzip.open(archive,'rb') as source,path.open('xb') as target:shutil.copyfileobj(source,target)
            created.append(path)
            if sha(path)!=record['original_sha256']:raise RuntimeError('decompressed hash mismatch')
        yield trial
    finally:
        for path in created:path.unlink()  # checked compressed original retained


def csv_write(path,rows):
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in rows:writer.writerow({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items()})


def measure(trial):
    manifest=read(trial/'manifest.json');factor=manifest['factorial']
    result=read(trial/'result.json');summary=read(trial/'analysis/summary.json')
    audit=read(trial/'trial_audit.json');native=lines(trial/'dual_timeline.jsonl')
    refreshes=[r for r in summary['requests'] if r['request_id']>1]
    outcomes=[]
    for request in refreshes:
        outcome=request['outcome_class'];ident=request['request_id']
        if outcome=='unclassified_failure':
            if request['end_mono'] is None:outcome='right_censored_request'
            elif any(r['event']=='collab_target' and r['request_id']==ident and not r['success'] for r in native):
                outcome='target_selection_reject'
            elif any(r['event']=='stage_end' and r['request_id']==ident and r.get('stage')=='search' and not r['success'] for r in native):
                outcome='search_failure'
        outcomes.append(outcome)
    prefix=[r for r in native if r['event']=='collab_prefix']
    initializers=[r for r in native if r['event']=='collab_initializer' and r['request_id']>1]
    solves=[s for r in refreshes for s in r['optimizer_calls']]
    lat=summary['latency'];account=summary.get('native_wall_accounting') or {};stops=summary['stop_analysis']
    blocked=lat['first_native_blocked_sim_s'];consumed=lat['first_refresh_consume_sim_s']
    navigation=[v for k,v in summary['stop_by_phase']['phases'].items() if k!='terminal_settling']
    state=np.atleast_1d(np.genfromtxt(trial/'state.csv',names=True,delimiter=','))
    limits=read(trial/'limits.json');wheel=np.column_stack((state['v']-.5*limits['track']*state['w'],state['v']+.5*limits['track']*state['w']))
    command_wheel=np.column_stack((state['cmd_v']-.5*limits['track']*state['cmd_w'],state['cmd_v']+.5*limits['track']*state['cmd_w']))
    wheel_max=float(np.abs(wheel).max());command_max=float(np.abs(command_wheel).max())
    dt=np.diff(state['t']);accel=np.abs(np.diff(wheel,axis=0)/dt[:,None])
    wheel_accel=float(accel[dt>1e-8].max()) if len(accel) else 0.
    measured_valid=wheel_max<=limits['wheel_v']+1e-6 and command_max<=limits['wheel_v']+1e-6 and wheel_accel<=limits['wheel_a']+1e-4
    return dict(path=str(trial),build=manifest['build'],group=factor['group'],seed=factor['seed'],scene=manifest['scene'],
        valid_experiment=audit['valid_experiment'],goal=result['reason']=='goal',outcome=result['reason'],
        collision=result['collision_conservative_projection'],min_clearance_m=result['min_clearance'],
        final_goal_distance_m=float(np.linalg.norm(np.array(result['final_state'][:2])-[3,0])),
        travel_time_s=result['elapsed']-(result.get('motion_anchor') or result['goal_sent']),
        path_length_m=float(np.hypot(np.diff(state['x']),np.diff(state['y'])).sum()),
        refresh_status='not_triggered' if blocked is None else 'no_delivery' if consumed is None else 'delivered',
        blocked_to_consume_s=consumed-blocked if consumed is not None and blocked is not None else None,
        blocked_to_end_lower_bound_s=summary['end_sim_s']-blocked if blocked is not None and consumed is None else None,
        request_wall_s=account.get('request_union_wall_s'),wait_wall_s=account.get('closed_explicit_wait_union_wall_s'),
        refresh_requests=len(refreshes),accepted_refresh_requests=sum(bool(r['success']) for r in refreshes),
        request_outcomes=dict(Counter(outcomes)),
        legacy_cached_outcomes=dict(Counter(r['outcome_class'] for r in refreshes)),
        optimizer_outcomes=dict(Counter(s.get('reason') for s in solves)),
        optimizer_budgets=dict(Counter(str(s['budget_s']) for s in solves)),
        optimizer_wall_ms=[s['wall_ms'] for s in solves],
        prefix_attempts=len(prefix),prefix_used=sum(bool(r['used']) for r in prefix),
        prefix_reasons=dict(Counter(r['reason'] for r in prefix)),
        initializer_pieces=[r['pieces'] for r in initializers],
        actual_nearly_stopped_s=sum(v['measured_nearly_zero_motion_s'] for v in navigation),
        command_zero_s=sum(sum(v['zero_command_duration_s'].values()) for v in navigation),
        deadline_miss_s=sum(v['reason_duration_s'].get('deadline_miss',0) for v in navigation),
        nearly_stopped_including_terminal_s=stops['measured_nearly_zero_motion_s'],
        max_actual_wheel_v=wheel_max,max_command_wheel_v=command_max,
        max_actual_wheel_a=wheel_accel,wheel_bounds_valid=measured_valid,
        fixed_arm_max_error=result['arm_max_error'],
        subgoal_continuation_checks=sum(r['event']=='collab_subgoal_continuation' for r in native))


def paired_measure(trial):
    summary=read(trial/'paired_summary.json');native=lines(trial/'dual_timeline.jsonl');solves=lines(trial/'failure_trace.jsonl')
    manifest=read(trial/'manifest.json');output=[];last_init=None;last_prefix=None
    plan=read(trial.parent/'experiment_plan.json',{})
    population='first_eligible_prefix' if plan.get('pair_eligible') else 'first_refresh'
    for row in native:
        if row['event']=='stage_begin' and row.get('stage')=='paired_request':last_init=None;last_prefix=None
        if row['event']=='collab_initializer':last_init=row
        if row['event']=='collab_prefix':last_prefix=row
        if row['event']!='collab_pair_result':continue
        failures=[s for s in solves if s.get('label')==row['label']]
        output.append(dict(path=str(trial),scene=manifest['scene'],seed=manifest['factorial']['seed'],
            population=population,donor_group=manifest['factorial'].get('group','unknown'),
            group=row['group'],label=row['label'],task_hash=row['task_hash'],target_hash=row['target_hash'],
            stable=row['stable'],target_ok=row['target_ok'],valid=row['valid'],wall_ms=row['wall_ms'],
            pieces=last_init['pieces'] if last_init else None,prefix_used=last_prefix['used'] if last_prefix else False,
            prefix_reason=last_prefix['reason'] if last_prefix else None,
            optimizer_reason=failures[-1]['reason'] if failures else 'not_entered',
            optimizer_input_hash=failures[-1]['input_hash'] if failures else None,
            first_cost=failures[-1]['first_cost'] if failures else None,
            first_gradient_hash=failures[-1]['first_gradient_hash'] if failures else None))
    errors=[];warnings=[]
    if not summary['paired_integrity'] or len(output)!=8:errors.append('snapshot count/stability')
    if len({r['task_hash'] for r in output})!=1:errors.append('paired task mismatch')
    if Counter(r['group'] for r in output)!=Counter(dict.fromkeys('ABCD',2)):errors.append('paired group count')
    if not all(r['stable'] for r in output):errors.append('snapshot changed')
    for groups in [('A','B'),('C','D')]:
        targets={r['target_hash'] for r in output if r['group'] in groups}
        if len(targets)>1:errors.append('target mismatch within '+str(groups))
    for group in 'ABCD':
        same=[r for r in output if r['group']==group and r['optimizer_input_hash'] is not None]
        if len(same)==2:
            if same[0]['optimizer_input_hash']!=same[1]['optimizer_input_hash']:
                warnings.append(group+': repeated initialization differs; task is paired, initialization is not bitwise deterministic')
            elif (same[0]['first_cost']!=same[1]['first_cost'] or same[0]['first_gradient_hash']!=same[1]['first_gradient_hash']):
                errors.append(group+': same optimizer input has changed first objective/gradient')
    return output,dict(path=str(trial),population=population,valid=not errors,errors=errors,warnings=warnings)


def common_inputs(rows):
    """Only treatment flags differ; verify actual geometry/limits/runtime."""
    buckets=defaultdict(list)
    for row in rows:buckets[(row['scene'],row['seed'])].append(Path(row['path']))
    checked=[];errors=[]
    for key,trials in buckets.items():
        baseline=trials[0]
        for name in ('limits.json','geometry.csv','parameters.yaml'):
            digests={sha(p/name) for p in trials}
            if len(digests)!=1:errors.append(str(key)+': '+name+' differs')
        libraries={read(p/'manifest.json')['mppi_library_sha256'] for p in trials}
        if len(libraries)!=1:errors.append(str(key)+': MPPI library differs')
        for name in ('mppi_node.py','local_execution.py','collab_seed_export.py','dynamic_scenarios.py'):
            if len({sha(p/'runtime'/name) for p in trials})!=1:errors.append(str(key)+': runtime '+name+' differs')
        checked.append(dict(scene=key[0],seed=key[1],trials=len(trials)))
    return dict(valid=not errors,errors=errors,matched_cells=checked,
                note='same settings/seed are not identical post-treatment closed-loop states')


def paired_contrasts(pairs):
    """Two repeated solves are one fixture, not two independent samples."""
    fixtures=defaultdict(list)
    for row in pairs:fixtures[row['path']].append(row)
    output=[]
    for path,data in fixtures.items():
        for original,treatment,claim in [('A','B','prefix_same_target'),('C','D','prefix_same_target'),
                                         ('A','C','rolling_changes_target'),('B','D','rolling_changes_target')]:
            left=[r for r in data if r['group']==original];right=[r for r in data if r['group']==treatment]
            output.append(dict(path=path,population=data[0]['population'],scene=data[0]['scene'],seed=data[0]['seed'],
                contrast=original+'->'+treatment,interpretation=claim,
                repeat_count_per_group=len(left),baseline_valid=sum(r['valid'] for r in left),
                treatment_valid=sum(r['valid'] for r in right),
                treatment_prefix_used=sum(r['prefix_used'] for r in right),
                baseline_target_ok=all(r['target_ok'] for r in left),treatment_target_ok=all(r['target_ok'] for r in right),
                mean_wall_change_ms=float(np.mean([r['wall_ms'] for r in right])-np.mean([r['wall_ms'] for r in left]))))
    return output


def static_reference_projection(rows):
    """Diagnose model/hand-over gaps, without changing either planner's cost.

    Uses only post-initial references and obstacles already present at adoption.
    This is sampled conservative 2D geometry, not proof of native 3D collision.
    """
    output=[]
    for row in rows:
        if row['scene']!='static':continue
        trial=Path(row['path']);body=np.atleast_2d(np.loadtxt(trial/'geometry.csv',delimiter=','))
        state=np.atleast_1d(np.genfromtxt(trial/'state.csv',names=True,delimiter=','))
        actors=np.atleast_1d(np.genfromtxt(trial/'obstacles.csv',names=True,delimiter=','))
        margin=read(trial/'limits.json')['margin']
        for ref in lines(trial/'consumed_references.jsonl'):
            if ref['revision']<2 or not len(actors) or ref['t']<actors['t'][0]:continue
            path=np.asarray(ref['reference']);c=np.cos(path[:,2,None]);s=np.sin(path[:,2,None])
            x=path[:,0,None]+c*body[None,:,0]-s*body[None,:,1]
            y=path[:,1,None]+s*body[None,:,0]+c*body[None,:,1]
            gap=np.minimum(2-abs(y)-body[None,:,2],np.minimum(x+1.5,4.5-x)-body[None,:,2])
            for actor_id in np.unique(actors['id']):
                actor=actors[actors['id']==actor_id][0]
                gap=np.minimum(gap,np.hypot(x-actor['x'],y-actor['y'])-body[None,:,2]-actor['r'])
            actual=np.array([np.interp(ref['t'],state['t'],state[k]) for k in ('x','y','yaw')])
            delta=float(abs(np.arctan2(np.sin(path[0,2]-actual[2]),np.cos(path[0,2]-actual[2]))))
            epoch_age=ref['t']+1000-ref['stamp']
            at_epoch=np.array([np.interp(epoch_age,ref['times'],path[:,k]) for k in (0,1)])
            output.append(dict(path=str(trial),group=row['group'],seed=row['seed'],revision=ref['revision'],
                adopted_t=ref['t'],minimum_sampled_projection_gap_m=float(np.nanmin(gap)),required_margin_m=margin,
                below_execution_margin=bool(np.nanmin(gap)<margin),
                sampled_poses_below_margin=int(np.sum(np.min(gap,axis=1)<margin)),
                sampled_poses=len(path),reference_start_to_actual_position_m=float(np.linalg.norm(path[0,:2]-actual[:2])),
                reference_start_to_actual_yaw_rad=delta,
                reference_epoch_age_at_adoption_s=epoch_age,
                reference_clock_pose_error_m=float(np.linalg.norm(at_epoch-actual[:2])),
                scope='sampled 2D execution model diagnostic, not native 3D collision or causality proof'))
    return output


def plots(rows,paired,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname=FONT,size=10);colors=['#6a7d8e','#629e82','#d4a75e','#8a7aaf']
    scenes=list(dict.fromkeys(r['scene'] for r in rows))
    latency_scenes=[s for s in scenes if any(r['scene']==s and r['refresh_status']!='not_triggered' for r in rows)] or scenes
    fig,axes=plt.subplots(1,len(latency_scenes),figsize=(3.7*len(latency_scenes),3.8),squeeze=False)
    for axis,scene in zip(axes[0],latency_scenes):
        for i,g in enumerate('ABCD'):
            data=[r for r in rows if r['scene']==scene and r['group']==g]
            valid=[r['blocked_to_consume_s'] for r in data if r['blocked_to_consume_s'] is not None]
            if valid:axis.scatter(np.full(len(valid),i)+np.linspace(-.08,.08,len(valid)),valid,c=colors[i],s=25)
            missing=sum(r['refresh_status']=='no_delivery' for r in data)
            axis.text(i,.98,'未交付%d'%missing,transform=axis.get_xaxis_transform(),ha='center',va='top',fontproperties=font,fontsize=8)
        axis.set_xticks(range(4));axis.set_xticklabels(list('ABCD'));axis.set_title(scene);axis.grid(axis='y',alpha=.2)
        axis.set_ylabel('参考受阻→采用新参考 / s',fontproperties=font)
    fig.suptitle('仅绘已交付时延；未交付不记成0，未触发另见CSV',fontproperties=font)
    fig.tight_layout(rect=[0,0,1,.91]);fig.savefig(out/'01_reference_delivery.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(11,3.8))
    for axis,key,title in zip(axes,['travel_time_s','path_length_m','actual_nearly_stopped_s'],['运动启动至结束 / s','实际路径长度 / m','实际近静止累计 / s']):
        for i,g in enumerate('ABCD'):
            data=[r for r in rows if r['group']==g]
            offsets=np.linspace(-.1,.1,len(data))
            for j,row in enumerate(data):axis.scatter(i+offsets[j],row[key],c=colors[i],s=22,marker='o' if row['goal'] else 'x')
        axis.set_xticks(range(4));axis.set_xticklabels(list('ABCD'));axis.set_title(title,fontproperties=font);axis.grid(axis='y',alpha=.2)
    fig.suptitle('圆点到达、叉号失败；停车不含终点正常稳定等候',fontproperties=font)
    fig.tight_layout(rect=[0,0,1,.91]);fig.savefig(out/'02_navigation.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,3.7))
    for axis,key,title in zip(axes,['goal','refresh_status'],['到达最终目标（不是子目标）','受阻后采用新REMANI参考']):
        grid=np.zeros((len(scenes),4))
        for y,scene in enumerate(scenes):
            for x,g in enumerate('ABCD'):
                data=[r for r in rows if r['scene']==scene and r['group']==g]
                count=sum(r['goal'] if key=='goal' else r['refresh_status']=='delivered' for r in data)
                denominator=len(data) if key=='goal' else sum(r['refresh_status']!='not_triggered' for r in data)
                grid[y,x]=count/denominator if denominator else np.nan
                axis.text(x,y,('%d/%d'%(count,denominator)) if denominator else '未触发',ha='center',va='center',fontproperties=font)
        axis.imshow(grid,cmap='YlGn',vmin=0,vmax=1);axis.set_xticks(range(4));axis.set_xticklabels(list('ABCD'))
        axis.set_yticks(range(len(scenes)));axis.set_yticklabels(scenes,fontsize=8);axis.set_title(title,fontproperties=font)
    fig.tight_layout();fig.savefig(out/'04_scene_outcomes.png',dpi=150);plt.close(fig)
    if paired:
        populations=list(dict.fromkeys(r['population'] for r in paired))
        fig,axes=plt.subplots(len(populations),2,figsize=(9,3.8*len(populations)),squeeze=False)
        for pair_axes,population in zip(axes,populations):
            title='首次刷新' if population=='first_refresh' else '首个可用前缀请求（另一个条件总体）'
            for i,g in enumerate('ABCD'):
                data=[r for r in paired if r['group']==g and r['population']==population]
                pair_axes[0].scatter(np.full(len(data),i)+np.linspace(-.1,.1,len(data)),[r['wall_ms'] for r in data],c=colors[i],s=24)
                pair_axes[1].bar(i,sum(r['valid'] for r in data)/len(data),color=colors[i])
                pair_axes[1].text(i,.02,'%d/%d'%(sum(r['valid'] for r in data),len(data)),ha='center')
            for axis in pair_axes:axis.set_xticks(range(4));axis.set_xticklabels(list('ABCD'));axis.grid(axis='y',alpha=.2)
            pair_axes[0].set_title(title+'：完整请求耗时',fontproperties=font);pair_axes[0].set_ylabel('耗时 / ms',fontproperties=font)
            pair_axes[1].set_title(title+'：原生验收通过率',fontproperties=font);pair_axes[1].set_ylim(0,1.05)
        fig.tight_layout();fig.savefig(out/'03_paired_solver.png',dpi=150);plt.close(fig)


def render_key(trials,out):
    from PIL import Image,ImageDraw,ImageFont
    from render_dual_timescale import Trial,bounds,draw_panel
    out.mkdir(parents=True,exist_ok=False)
    with ExitStack() as stack:
        for source in trials:stack.enter_context(unpacked(source))
        objects=[Trial(p,TITLES[read(p/'manifest.json')['factorial']['group']]) for p in trials]
        limits=bounds(objects);font=ImageFont.truetype(FONT,21);small=ImageFont.truetype(FONT,16)
        duration=max(t.end-t.anchor for t in objects);frames=[]
        for age in list(np.arange(0,duration,.2))+[duration]:
            canvas=Image.new('RGB',(1240,1000),'white');draw=ImageDraw.Draw(canvas)
            draw.text((18,10),'隔离数值仿真  |  各组从运动起点对齐  |  2倍播放',font=font,fill='#333333')
            draw.text((18,42),'橙：已采用REMANI参考   蓝：实际路线/完整投影   绿：本周期已检查预测   浅色：真实候选子集',font=small,fill='#555555')
            for i,trial in enumerate(objects):
                x=(i%2)*620;y=85+(i//2)*440;t=min(trial.anchor+age,trial.end)
                draw=ImageDraw.Draw(canvas);draw.text((x+18,y),trial.title+'  %.1fs'%age,font=font,fill='#333333')
                row,_=draw_panel(canvas,trial,t,(x+18,y+35,x+602,y+365),limits)
                draw=ImageDraw.Draw(canvas)
                draw.text((x+18,y+376),'v=%+.2f m/s  |  '%row['v']+trial.status_at(t),font=small,fill='#555555')
            draw.text((18,970),'固定机械臂；非Gazebo/实车。灰箭头为1秒运动示意，不是完整预测时域。',font=small,fill='#777777')
            frames.append(canvas.convert('P',palette=Image.ADAPTIVE,colors=128))
        durations=[100]*len(frames);durations[-1]=600
        frames[0].save(out/'overview.gif',save_all=True,append_images=frames[1:],duration=durations,loop=0,disposal=2)
        frames[-1].convert('RGB').save(out/'final_paths.png')
        for trial in objects:trial.verify_unchanged()
        audit=dict(no_future_reference=True,no_controller_reexecution=True,no_fabricated_rollouts=True,
            playback_multiplier=2,frames=len(frames),sources=[dict(path=str(t.source),hashes=t.hashes,body=t.body.tolist()) for t in objects])
        (out/'visualization_audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')


def report(batches,paired_batches,out,render=False):
    out.mkdir(parents=True,exist_ok=False);rows=[];pairs=[];pair_audits=[]
    for batch in batches:
        if not read(batch/'batch_complete.json',{}).get('complete'):raise RuntimeError('unfinished online batch')
        for trial in sorted(p.parent for p in batch.glob('*/trial_audit.json')):rows.append(measure(trial))
    for batch in paired_batches:
        if not read(batch/'batch_complete.json',{}).get('complete'):raise RuntimeError('unfinished paired batch')
        for trial in sorted(p.parent for p in batch.glob('*/paired_summary.json')):
            data,audit=paired_measure(trial);pairs+=data;pair_audits.append(audit)
    if not rows:raise RuntimeError('no formal online records')
    common=common_inputs(rows)
    contrasts=paired_contrasts(pairs)
    projection=static_reference_projection(rows)
    csv_write(out/'online_metrics.csv',rows);csv_write(out/'paired_metrics.csv',pairs)
    csv_write(out/'paired_contrasts.csv',contrasts)
    csv_write(out/'static_reference_projection.csv',projection)
    figures=[]
    plots(rows,pairs,out)
    if render:
        for scene in ['static','dynamic_head_on_center','dynamic_four']:
            cases=[r for r in rows if r['scene']==scene];seeds=sorted(set(r['seed'] for r in cases))
            selected=None
            for seed in seeds:
                found={r['group']:Path(r['path']) for r in cases if r['seed']==seed}
                if set(found)==set('ABCD'):selected=[found[g] for g in 'ABCD'];break
            if selected:
                destination=out/'figures'/scene;render_key(selected,destination)
                figures.append(str(destination.relative_to(out)/'overview.gif'))
    expected=read(Path(read(batches[0]/'experiment_plan.json')['build'])/'build_manifest.json')['production_sources_sha256']
    changed=[p for p,digest in expected.items() if sha(ROOT/p)!=digest]
    totals={}
    text=['REMANI—MPPI：目标选择×局部初值反馈实验','日期：2026-10-22，Asia/Shanghai','',
          '范围：原生ROS规划/检测器＋数值差速底盘；固定机械臂；不是Gazebo或实车。',
          '四组全部由MPPI执行；A是原V12协同逻辑，不是Tracker或原论文REMANI的单独基线。',
          'A原V12逻辑；B仅紧凑MPPI前缀反馈；C仅滚动子目标；D两者结合。',
          'C/D还包含子目标的双重可行性检查与提前续段；不是只改一个距离参数。',
          '两层碰撞代价未统一，原有异步交接未修复；不把本轮叫作已解决双时间尺度联合优化。',
          '试验时限从发布目标开始计48s；travel_time_s从实际启动运动起计，不含初始规划等候。',
          '正式结果不混入构建02失败冒烟；全部失败保留，未交付参考不填0秒。','', '一、正式在线结果']
    for group in 'ABCD':
        data=[r for r in rows if r['group']==group]
        if not data:continue
        lat=[r['blocked_to_consume_s'] for r in data if r['blocked_to_consume_s'] is not None]
        totals[group]=dict(trials=len(data),goals=sum(r['goal'] for r in data),collisions=sum(r['collision'] for r in data),
            refresh_status=dict(Counter(r['refresh_status'] for r in data)),
            median_delivered_latency_s=float(np.median(lat)) if lat else None,
            refresh_requests=sum(r['refresh_requests'] for r in data),
            accepted_refresh_requests=sum(r['accepted_refresh_requests'] for r in data),
            prefix_attempts=sum(r['prefix_attempts'] for r in data),prefix_used=sum(r['prefix_used'] for r in data))
        text.append(group+'：'+json.dumps(totals[group],ensure_ascii=False))
        for scene in sorted(set(r['scene'] for r in data)):
            subset=[r for r in data if r['scene']==scene]
            text.append('  '+scene+'：到达%d/%d；耗时%s；更新状态%s'%(sum(r['goal'] for r in subset),len(subset),
                [round(r['travel_time_s'],3) for r in subset],dict(Counter(r['refresh_status'] for r in subset))))
    text+=['','二、同输入配对结果（不是独立导航成功率）']
    for population in dict.fromkeys(r['population'] for r in pairs):
        text.append('快照总体：'+population+'；不同总体不可合并推断平均效果。')
        for group in 'ABCD':
            data=[r for r in pairs if r['group']==group and r['population']==population]
            if data:text.append('%s：有效%d/%d；实际使用前缀%d次；请求中位耗时%.2fms；优化结果%s；初值段数%s'%(group,
                sum(r['valid'] for r in data),len(data),sum(r['prefix_used'] for r in data),np.median([r['wall_ms'] for r in data]),
                dict(Counter(r['optimizer_reason'] for r in data)),[r['pieces'] for r in data]))
    text+=['','三、解释边界',
        '1. A/B及C/D同目标对照才用于判断初值反馈；C改变阶段任务，不能把它的提速全称优化器提速。',
        '2. 在线同种子不等于同状态；同输入求解另用暂停后的快照，暂停运行不计导航性能。',
        '3. 初始化转换、后续搜索和失败回退均计入请求耗时；实际多项式段数/优化预算在CSV保留。',
        '4. 0.6秒是协同刷新优化预算；短段结束后原有GEN_NEW入口可能使用1.5秒。预算分布需一起看，不能只报均值。',
        '5. 目前没有理论闭环安全或必达证明；数值无碰撞不能代替实车验收。',
        '6. 比较是否真的有帮助，要同时看首次有效交付、最终导航、停车和安全，不只看总到达率。',
        '7. 本轮不修改碰撞后端/学习调度/实车，不自动上传GitHub。','',
        '四、审计', '生产文件变更：'+str(changed),'配对审计：'+json.dumps(pair_audits,ensure_ascii=False),
        '在线接口审计全部有效：'+str(all(r['valid_experiment'] for r in rows)),
        '数值执行轮速/轮加速度边界全部通过：'+str(all(r['wheel_bounds_valid'] for r in rows)),
        '', '五、关键GIF（预先按最小种子选择，不挑最好成绩）',*figures]
    text+=['','六、静态新参考的二维执行模型复核',
        '只复核非初始参考及采用时已出现的固定障碍；参考按约40ms采样，非连续碰撞证明。',
        '新参考%d条，其中%d条某处净距小于MPPI要求的余量。'%
            (len(projection),sum(r['below_execution_margin'] for r in projection)),
        '该差异是两层几何可行性不一致的证据，不等于机器人实际碰撞或证明所有停滞由此导致。',
        '起点及按原参考时钟插值的位置与实际位置差异也记录在CSV；只是交接诊断，',
        'MPPI以实测路径投影推进，并不承诺按原时钟跟踪，不能单凭该差异判定交接故障。本轮不修改交接。']
    result=dict(scope=text[3],online=rows,paired=pairs,paired_contrasts=contrasts,static_reference_projection=projection,
        paired_audits=pair_audits,groups=totals,
        common_inputs_audit=common,
        production_files_verified=len(expected),production_changed=changed,gifs=figures,
        all_online_integrity=all(r['valid_experiment'] for r in rows),all_paired_integrity=all(r['valid'] for r in pair_audits),
        all_wheel_bounds=all(r['wheel_bounds_valid'] for r in rows))
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (out/'实验结果与分析.txt').write_text('\n'.join(text)+'\n')
    (out/'GIF_INDEX.md').write_text('# 关键GIF\n\n'+'\n'.join('- ['+p.split('/')[-2]+']('+p+')' for p in figures)+'\n')
    print('REPORTED',len(rows),'online',len(pairs),'paired solves',len(figures),'GIF',flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,action='append',required=True)
    p.add_argument('--paired-batch',type=Path,action='append',default=[]);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--render',action='store_true');a=p.parse_args();report(a.batch,a.paired_batch,a.out,a.render)

#!/usr/bin/env python3
"""Read actual delivery-next records; no controller invocation or rerun."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from render_native_collision_3d import Trial
from report_cheap_scaling_online import Reader, count_description, first, seconds
from report_cheap_scaling_online import summarize as common_summary
from report_long_goal_crossing import panel, settings
from report_sphere_scenarios import stats, shown

LABELS={False:'A 原退出方式',True:'B 合格候选提前退出'}
SCENES={'static':'静态挡路','midcross':'中段横穿'}
BUDGET_MS=1200.


def request_chain(timeline,solver):
    """Request IDs are source-native; check clocks precede deferred log emit."""
    request_ids=sorted({r['request_id'] for r in timeline if r.get('event')=='stage_begin'
        and r.get('stage')=='manager_replan' and 'request_id' in r})
    optimizer_ends=[r for r in solver if r.get('event')=='end' and r.get('stage')=='optimizer']
    requests=[]
    for request_id in request_ids:
        events=[r for r in timeline if r.get('request_id')==request_id]
        get=lambda name: first(events,lambda e:e.get('event')==name)
        stage=lambda name,kind: first(events,lambda e:e.get('event')==kind and e.get('stage')==name)
        begin=stage('manager_replan','stage_begin');optbegin=stage('optimizer','stage_begin')
        optend=stage('optimizer','stage_end');publish=get('reference_publish')
        acceptance=get('acceptance_pass');rejection=get('acceptance_reject')
        checks=[e for e in events if e.get('event')=='early_delivery_check']
        native_pass=[e for e in checks if bool(e.get('native_pass'))]
        chosen=first(checks,lambda e:bool(e.get('selected_for_exit')))
        final_pass=optend is not None and optend.get('success') is True
        # The native final check finishes before stage_end. In A/no-selected
        # cases this is an explicit upper-bound timestamp, not a fabricated
        # precise IsTrajSafe completion event.
        qualified=(chosen.get('check_end_mono_s') if chosen is not None else
                   optend['mono'] if final_pass else None)
        metrics=None
        if optend is not None and optimizer_ends:
            nearest=min(optimizer_ends,key=lambda e:abs(e['mono']-optend['mono']))
            if abs(nearest['mono']-optend['mono'])<.03:metrics=nearest
        details=[e for e in events if e.get('event')=='latest_prediction_reject_detail']
        requests.append(dict(request_id=request_id,begin_mono=begin['mono'] if begin else None,
            optimizer_begin_mono=optbegin['mono'] if optbegin else None,
            optimizer_end_mono=optend['mono'] if optend else None,
            backend_final_pass=final_pass,qualified_mono=qualified,
            qualified_clock_semantics='exact selected candidate native gate completion' if chosen else
                'upper bound: optimizer stage_end after final native gate' if final_pass else 'no final qualified candidate',
            first_live_native_pass_mono=min((e['check_end_mono_s'] for e in native_pass),default=None),
            latest_accept_mono=acceptance['mono'] if acceptance else None,
            latest_rejection=rejection,latest_reject_details=details,
            publish_mono=publish['mono'] if publish else None,
            publish_reference_age_sim_s=publish['sim']-publish['guide_start'] if publish and 'guide_start' in publish else None,
            early_checks=checks,early_summary=get('early_delivery_summary'),
            cheap_summary=get('cheap_scaling_summary'),optimizer_metrics=metrics))
    return requests


def enrich_episodes(episodes,requests):
    out=[]
    for original in episodes:
        episode=dict(original)
        if 'publish_mono' not in episode:out.append(episode);continue
        trigger=episode['trigger_mono'];end=episode['publish_mono']
        related=[r for r in requests if r['begin_mono'] is not None and trigger<=r['begin_mono']<=end]
        published=next((r for r in related if r['request_id']==episode.get('publish_request_id')),None)
        optbegins=[r['optimizer_begin_mono'] for r in related if r['optimizer_begin_mono'] is not None]
        qualified=[r['qualified_mono'] for r in related if r['qualified_mono'] is not None]
        episode.update(request_ids=[r['request_id'] for r in related],
            trigger_to_optimizer_begin_wall_s=min(optbegins)-trigger if optbegins else None,
            trigger_to_first_final_qualified_wall_s=min(qualified)-trigger if qualified else None,
            trigger_to_latest_accept_wall_s=published['latest_accept_mono']-trigger
                if published and published['latest_accept_mono'] is not None else None,
            published_request_qualified_clock_semantics=published.get('qualified_clock_semantics') if published else None,
            published_reference_age_sim_s=published.get('publish_reference_age_sim_s') if published else None)
        out.append(episode)
    return out


def summarize(path,job):
    # Both new groups deliberately have cheap scaling enabled. This reuses
    # only the prior read-only measurement parser, not its old conclusions.
    row=common_summary(path,dict(scene=job['scene'],enabled=True))
    reader=Reader(path);manifest=reader.read('manifest.json',default={})
    declaration=manifest.get('delivery_next_online',{})
    if declaration.get('early_exit') is not job['early_exit'] or declaration.get('common_cusp_phase_association') is not True:
        raise ValueError('Trial group/common cusp declaration differs from plan: '+str(path))
    timeline=reader.read('dual_timeline.jsonl',rows=True)
    solver=reader.read('solver_trace.jsonl',rows=True)
    phases=reader.read('cusp_phase_events.jsonl',rows=True)
    requests=request_chain(timeline,solver)
    opts=[r['optimizer_metrics'] for r in requests if r['optimizer_metrics']]
    totals=[r.get('wall_ms') for r in opts if r.get('wall_ms') is not None]
    budgeted=[]
    for request in requests:
        summary=request['early_summary'];metrics=request['optimizer_metrics']
        if summary and summary.get('elapsed_ms') is not None:budgeted.append(summary['elapsed_ms'])
        elif metrics and metrics.get('lbfgs_wall_ms') is not None:budgeted.append(metrics['lbfgs_wall_ms'])
    row.update(enabled=job['early_exit'],early_exit=job['early_exit'],requests=requests,
        delivery_episodes=enrich_episodes(row['delivery_episodes'],requests),
        budgeted_internal_ms=stats(budgeted),full_optimizer_wall_ms=stats(totals),
        budgeted_internal_overruns=sum(t>BUDGET_MS for t in budgeted),
        budgeted_internal_max_overrun_ms=max([max(0.,t-BUDGET_MS) for t in budgeted]+[0.]),
        full_optimizer_overruns=sum(t>BUDGET_MS for t in totals),
        full_optimizer_max_overrun_ms=max([max(0.,t-BUDGET_MS) for t in totals]+[0.]),
        early_gate_calls=sum(len(r['early_checks']) for r in requests),
        early_native_calls=sum(bool(c.get('native_checked')) for r in requests for c in r['early_checks']),
        early_gate_selected=sum(bool(r['early_summary'] and r['early_summary'].get('selected')) for r in requests),
        early_native_wall_ms=stats(c.get('native_gate_ms') for r in requests for c in r['early_checks'] if c.get('native_checked')),
        cusp_phase_transition_events=sum(e.get('measured_phase_transition') is True for e in phases),
        cusp_reference_events=sum(not e.get('measured_phase_transition',False) for e in phases),
        event_capture=reader.read('delivery_event_capture.json',default={}))
    row['input_hashes'].update(reader.hashes);reader.verify()
    return row


def first_episode(row):
    return next((e for e in row['delivery_episodes'] if e.get('publish_mono') is not None),{})


def chart(rows,path):
    image=Image.new('RGB',(1160,600),'white');draw=ImageDraw.Draw(image)
    title=ImageFont.truetype(FONT,21);font=ImageFont.truetype(FONT,16);small=ImageFont.truetype(FONT,14)
    draw.text((20,12),'可用轨迹交付：原退出方式 vs 合格候选提前退出',font=title,fill='#333333')
    draw.text((20,47),'两组共有：廉价缩放＋换向段关联修复；同67球、轮约束、1.2s内部预算。每组一次。',font=font,fill='#555555')
    colors={False:'#999999',True:'#4e7c9d'}
    metrics=[('首次重规划：触发→实际采用 / 秒',lambda r:first_episode(r).get('trigger_to_first_kernel_wall_s'),3),
             ('单次后端完整耗时中位数 / 毫秒',lambda r:r['full_optimizer_wall_ms']['p50'],1)]
    for index,(name,get,precision) in enumerate(metrics):
        left=20+index*575
        draw.text((left,94),name,font=font,fill='#333333')
        maximum=max([get(r) for r in rows if get(r) is not None]+[1e-6])
        for i,scene in enumerate(SCENES):
            y=145+i*135;draw.text((left,y),SCENES[scene],font=font,fill='#444444')
            for j,enabled in enumerate((False,True)):
                row=next((r for r in rows if r['scene']==scene and r['early_exit']==enabled),None)
                if row is None:continue
                value=get(row);width=270*value/maximum if value is not None else 0
                yy=y+30+j*30
                draw.text((left+5,yy-2),'B' if enabled else 'A',font=small,fill='#444444')
                if width:draw.rectangle((left+36,yy,left+36+width,yy+18),fill=colors[enabled])
                draw.text((left+43+width,yy-2),shown(value,precision),font=small,fill='#444444')
    y=435
    for row in rows:
        text='%s %s：%s；距目标%.2fm；完整后端超1.2s %s/%s次' % (SCENES[row['scene']],
            'B' if row['early_exit'] else 'A',row['termination_cn'],row['final_goal_distance_m'],
            row['full_optimizer_overruns'],row['optimizer_calls'])
        draw.text((20,y),text,font=small,fill='#555555');y+=27
    draw.text((20,561),'灰A、蓝B；没有采用事件不记0。后端最终验收及下游最新检查仍需耗时，不是硬实时承诺。',font=small,fill='#666666')
    image.save(path)


def gif_pair(left,right,path,scene,max_frames=64):
    trials=[Trial(left,'native3d'),Trial(right,'native3d')]
    for trial in trials:settings(trial.manifest);trial.native_validity=[]
    duration=max(t.end-t.anchor for t in trials)
    ages=np.linspace(0,duration,min(max_frames,max(2,math.ceil(duration/.5)+1)))
    frame_ms=max(30,10*round((ages[1]-ages[0])*500/10))
    title=ImageFont.truetype(FONT,19);small=ImageFont.truetype(FONT,14);frames=[]
    for age in ages:
        image=Image.new('RGB',(1160,480),'white');draw=ImageDraw.Draw(image)
        draw.text((18,9),'%s：起步后 %.1fs（各自实测起步对齐）' % (SCENES[scene],age),font=title,fill='#333333')
        draw.text((18,39),'蓝：机器人与实走轨迹　橙：REMANI参考　绿：获准局部轨迹　灰：障碍物　紫：CV预测',font=small,fill='#555555')
        for index,trial in enumerate(trials):
            t=min(trial.anchor+age,trial.end)
            draw.text((index*580+18,67),LABELS[bool(index)],font=title,fill='#333333')
            row=panel(image,trial,t,(index*580+18,103,(index+1)*580-18,408))
            draw=ImageDraw.Draw(image);status=trial.status_at(t)
            if t>=trial.end-1e-9:
                gap=float(np.hypot(row['x']-6.,row['y']))
                status='已到达目标' if trial.result['reason']=='goal' else '时间到，距目标%.2fm' % gap
            elif status=='ok':status='局部控制有效'
            draw.text((index*580+18,418),'%s　v=%+.2f m/s' % (status,row['v']),font=small,fill='#555555')
        draw.text((18,451),'实际记录回放，整机67球投影；固定机械臂ROS数值闭环，不是实车/Gazebo。',font=small,fill='#777777')
        frames.append(image.convert('P',palette=Image.ADAPTIVE,colors=96))
    frames[0].save(path,save_all=True,append_images=frames[1:],loop=0,disposal=2,optimize=True,
        duration=[frame_ms]*(len(frames)-1)+[900])
    for trial in trials:trial.verify_unchanged()


def report(batch,out,gifs=True):
    batch,out=Path(batch).resolve(),Path(out).resolve();plan=json.loads((batch/'plan.json').read_text())
    rows=[];missing=[]
    for job in plan['jobs']:
        path=batch/job['trial']
        if not (path/'run_status.json').exists():missing.append(job['trial']);continue
        rows.append(summarize(path,job))
    out.mkdir(parents=True,exist_ok=True);chart(rows,out/'轨迹交付对照.png')
    if gifs:
        for scene in SCENES:
            pair=[next((r for r in rows if r['scene']==scene and r['early_exit']==enabled),None) for enabled in (False,True)]
            if all(pair):gif_pair(batch/pair[0]['trial'],batch/pair[1]['trial'],out/(SCENES[scene]+'.gif'),scene)
    summary=dict(scope='actual numerical ROS closed loop; fixed arm; one run per group/scene',trials=rows,
        unmeasured_jobs=missing,plan_sha256=hashlib.sha256((batch/'plan.json').read_bytes()).hexdigest(),
        report_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        check_clock='explicit check_begin_mono_s/check_end_mono_s, not buffered event emit time',
        final_qualification_clock='exact selected live gate or upper bound at optimizer stage_end; per request labeled')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    lines=['轨迹稳定交付：下一步在线对照',
        '固定机械臂，完整REMANI＋MPPI，原始初始化、67球、轮速4.1、轮加速度4.25。ROS数值闭环，不是Gazebo或实车。',
        '两组都开廉价变量缩放、修复MPPI换向段关联。A保持原退出方式；B仅在已接受候选通过完整原生检查后提前退出。',
        '没有放松物理约束、碰撞外形或代价。B新检查计入1.2秒内部预算；A原本尾部验收另计。最终验收、FSM最新检查、发布和采用继续记录。',
        '1.2秒是软求解预算，不是保证整条链路1.2秒内完成。以下不把单次后端时间当成触发到可执行轨迹的时间。','']
    for scene in SCENES:
        lines.append(SCENES[scene])
        for enabled in (False,True):
            r=next((r for r in rows if r['scene']==scene and r['early_exit']==enabled),None)
            if r is None:lines.append(LABELS[enabled]+'：尚无完整记录。');continue
            e=first_episode(r)
            lines.extend([
                '%s：%s；接口核验%s；%s采样物理碰撞；后端通过%d/%d次。' % (LABELS[enabled],r['termination_cn'],
                    '通过' if r['interface_valid'] else '未通过','发生' if r['physical_sample_collision'] else '未发现',r['optimizer_successes'],r['optimizer_calls']),
                '实走%.2fm；终点(%.3f,%.3f)m、朝向%.1f度，距目标%.3fm；%s。' % (r['measured_travel_m'],*r['final_xy'],r['final_yaw_deg'],r['final_goal_distance_m'],
                    '结束前2秒基本静止' if r['stationary_last_two_seconds'] else '结束时仍在移动' if not r['stopped_at_end'] else '结束时已停住'),
                '首次重规划：触发→开始优化%s；→后端合格%s；→最新验收通过%s；→发布%s；→MPPI实际采用%s；→首次获准指令%s。' % (
                    seconds(e.get('trigger_to_optimizer_begin_wall_s')),seconds(e.get('trigger_to_first_final_qualified_wall_s')),
                    seconds(e.get('trigger_to_latest_accept_wall_s')),seconds(e.get('trigger_to_publish_wall_s')),seconds(e.get('trigger_to_first_kernel_wall_s')),
                    seconds(e.get('trigger_to_validated_command_wall_s'))),
                '单次后端完整耗时中位数%s毫秒；内部预算超时%d次、最多超%s毫秒；算上尾部验收后超1.2秒%d次、最多超%s毫秒。' % (
                    shown(r['full_optimizer_wall_ms']['p50']),r['budgeted_internal_overruns'],shown(r['budgeted_internal_max_overrun_ms']),
                    r['full_optimizer_overruns'],shown(r['full_optimizer_max_overrun_ms'])),
                '候选预检%d次，完整原生检查%d次，提前选中退出%d次。最新环境拒收：%s；发布%d次，结束时待交付事件%d个。' % (
                    r['early_gate_calls'],r['early_native_calls'],r['early_gate_selected'],count_description(r['latest_acceptance_rejections']),
                    r['reference_publications'],r['incomplete_delivery_episodes']),
                '实测换向段推进事件%d个；最后2秒控制状态：%s。' % (r['cusp_phase_transition_events'],count_description(r['final_two_seconds_control_reasons']))])
            for request in r['requests']:
                if request['latest_reject_details']:
                    lines.append('请求%d的最新预测拒收分支：%s。' % (request['request_id'],', '.join(str(d.get('branch')) for d in request['latest_reject_details'])))
        lines.append('')
    if missing:lines.append('未完成的原计划运行：'+', '.join(missing)+'；没有筛掉失败或替换种子。')
    lines.extend(['读数边界：A和未提前退出的B，后端合格时间取最终原生验收之后的优化器结束事件，是完成时刻的上界；提前选中的B用原生检查实际结束时刻。',
        '缓冲输出的检查日志不能用写出时刻算延迟；本报告使用检查内部的真实墙钟。首次事件缺失写“无记录”，不等于0。',
        '新在线在第一次实际换向或持续非目标停住时，最多保留一个完整绑定native3d输入，停止运行后才写盘；不保存每帧全图，也不声称记录了MPPI随机数全部历史。',
        '本轮只隔离提前交付的作用。两组共有换向关联修复，不能仅据本对照证明它单独改善多少；闭环分化后的请求也不能当作完全同输入。',
        '每组每场景仅一次，用来检查现象与因果链，不是总体成功率。GIF直接画实际记录，没有重新调用控制器替换失败画面。'])
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n');print(out)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--batch',type=Path,required=True)
    parser.add_argument('--out',type=Path);parser.add_argument('--no-gifs',action='store_true');args=parser.parse_args()
    report(args.batch,args.out or args.batch/'report',not args.no_gifs)

#!/usr/bin/env python3
"""Read completed cheap-scaling closed loops; never execute a controller."""
import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from render_native_collision_3d import Trial
from report_long_goal_crossing import panel, settings
from report_sphere_scenarios import stats, shown, request_metrics

LABELS={False:'原初值，不缩放',True:'原初值，廉价缩放'}
SCENES=dict(empty='空旷',static='静态挡路',midcross='中段横穿')


class Reader:
    def __init__(self,root):self.root=Path(root);self.hashes={}
    def read(self,name,rows=False,default=None):
        path=self.root/name
        compressed=False
        if not path.exists():path=path.with_name(path.name+'.gz');compressed=True
        if not path.exists():return [] if rows else default
        self.hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
        with (gzip.open(path,'rt') if compressed else path.open()) as stream:
            return [json.loads(line) for line in stream if line.strip()] if rows else json.load(stream)
    def verify(self):
        for path,digest in self.hashes.items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:raise ValueError('Report input changed: '+path)


def first(rows,predicate):
    return next((r for r in rows if predicate(r)),None)


def delivery_episodes(timeline,observer,commits,cycles):
    receipts={str(r['transaction_fnv64']):r for r in observer
              if r.get('event')=='reference_observer_receipt' and 'transaction_fnv64' in r}
    episodes=[];pending=None
    for event in sorted(timeline,key=lambda r:r['mono']):
        if event['event']=='refresh_trigger' and pending is None:
            pending=event
        if event['event']!='reference_publish' or pending is None:
            continue
        pub=event;receipt=receipts.get(str(pub.get('transaction_fnv64')))
        hashes=receipt.get('message_hashes') if receipt else None
        commit=first(commits,lambda r:hashes is not None and r.get('event')=='replace' and r.get('message_hashes')==hashes)
        related=[r for r in cycles if commit is not None and r.get('guide_epoch')==commit.get('epoch')
                 and r.get('guide_revision')==commit.get('revision')]
        use=first(related,lambda r:r.get('first_kernel_cycle_for_reference') and r.get('online_step_entry_monotonic') is not None)
        command=first(related,lambda r:r.get('first_validated_command_cycle_for_reference')
                      and r.get('command_publish_after_monotonic') is not None)
        trigger_mono=pending['mono']
        episodes.append(dict(trigger_mono=trigger_mono,trigger_sim=pending['sim'],publish_mono=pub['mono'],
            publish_request_id=pub.get('request_id'),transaction_fnv64=pub.get('transaction_fnv64'),
            trigger_to_publish_wall_s=pub['mono']-trigger_mono,
            commit_mono=commit.get('mono') if commit else None,
            trigger_to_commit_wall_s=commit['mono']-trigger_mono if commit and 'mono' in commit else None,
            first_kernel_mono=use['online_step_entry_monotonic'] if use else None,
            trigger_to_first_kernel_wall_s=use['online_step_entry_monotonic']-trigger_mono if use else None,
            first_validated_command_mono=command['command_publish_after_monotonic'] if command else None,
            trigger_to_validated_command_wall_s=command['command_publish_after_monotonic']-trigger_mono if command else None,
            matched_reference_revision=commit.get('revision') if commit else None,
            source='actual monotonic publish/commit/kernel/command events; observer read time is never substituted'))
        pending=None
    if pending is not None:
        episodes.append(dict(trigger_mono=pending['mono'],trigger_sim=pending['sim'],pending_at_end=True,
                             trigger_to_publish_wall_s=None,trigger_to_first_kernel_wall_s=None))
    return episodes


def summarize(path,job):
    read=Reader(path)
    result=read.read('result.json',default={});status=read.read('run_status.json',default={})
    audit=read.read('trial_audit.json',default={});analysis=read.read('analysis/summary.json',default={})
    timeline=read.read('dual_timeline.jsonl',rows=True)
    observer=read.read('observer_events.jsonl',rows=True)
    cycles=read.read('state_mppi_cycles.jsonl',rows=True)
    commits=read.read('reference_events.jsonl',rows=True)
    solver=read.read('solver_trace.jsonl',rows=True)
    diagnostics=read.read('mppi_diagnostics.json',default=[])
    opt=[r for r in solver if r.get('event')=='end' and r.get('stage')=='optimizer']
    episodes=delivery_episodes(timeline,observer,commits,cycles)
    measured=request_metrics(analysis,float(result.get('elapsed',math.inf)))
    monitor=result.get('native3d_truth_monitor',{})
    manifest=read.read('manifest.json',default={})
    if manifest.get('cheap_scaling_online',{}).get('enabled') is not job['enabled']:
        raise ValueError('Trial scaling switch differs from plan: '+str(path))
    read.verify()
    return dict(trial=path.name,scene=job['scene'],enabled=job['enabled'],
        termination=result.get('reason'),interface_valid=audit.get('valid_experiment') is True,
        runner_error=status.get('runner_error'),physical_sample_collision=monitor.get('sample_collision'),
        minimum_physical_sample_gap_m=monitor.get('min_physical_sample_gap'),
        motion_anchor=result.get('motion_anchor'),elapsed_sim_s=result.get('elapsed'),
        optimizer_calls=len(opt),optimizer_successes=sum(r.get('success') is True for r in opt),
        optimizer_failures=sum(r.get('success') is False for r in opt),
        optimizer_force_stop_counts=dict(Counter(str(r.get('force_stop')) for r in opt)),
        prepare_ms=stats(r.get('cheap_scaling_prepare_ms') for r in opt),
        prepare_plus_solver_ms=stats(r.get('lbfgs_wall_ms') for r in opt),
        entire_optimizer_entry_ms=stats(r.get('wall_ms') for r in opt),
        native_validation_ms=stats(r.get('native_validation_ms') for r in opt),
        mppi_callback_ms=stats(r.get('callback_ms') for r in diagnostics if r.get('reference_points',0)>0),
        mppi_deadline_misses=sum(r.get('reason')=='deadline_miss' for r in cycles),
        remani=measured,delivery_episodes=episodes,
        trigger_to_publish_wall_s=stats(r.get('trigger_to_publish_wall_s') for r in episodes),
        trigger_to_first_kernel_wall_s=stats(r.get('trigger_to_first_kernel_wall_s') for r in episodes),
        trigger_to_validated_command_wall_s=stats(r.get('trigger_to_validated_command_wall_s') for r in episodes),
        incomplete_delivery_episodes=sum(r.get('pending_at_end',False) for r in episodes),
        input_hashes=read.hashes)


def chart(rows,path):
    image=Image.new('RGB',(1160,570),'white');draw=ImageDraw.Draw(image)
    font=ImageFont.truetype(FONT,21);small=ImageFont.truetype(FONT,15)
    draw.text((20,12),'廉价变量缩放：完整REMANI＋MPPI闭环',font=font,fill='#333333')
    draw.text((20,44),'灰：不缩放　蓝：廉价缩放；同1.2s准备＋优化预算、同67球与轮约束；每组一次',font=small,fill='#555555')
    metrics=[('准备＋优化中位数 / ms',lambda r:r['prepare_plus_solver_ms']['p50']),
             ('首次重规划触发→MPPI实际采用 / s',lambda r:next((e.get('trigger_to_first_kernel_wall_s') for e in r['delivery_episodes'] if e.get('trigger_to_first_kernel_wall_s') is not None),None))]
    for index,(title,get) in enumerate(metrics):
        left=20+index*575;top=100
        draw.text((left,top),title,font=small,fill='#333333')
        values=[get(r) for r in rows];maximum=max([v for v in values if v is not None]+[1e-6])
        for i,scene in enumerate(SCENES):
            y=top+48+i*106
            draw.text((left,y),SCENES[scene],font=small,fill='#444444')
            for j,enabled in enumerate((False,True)):
                row=next((r for r in rows if r['scene']==scene and r['enabled']==enabled),None)
                if row is None:continue
                value=get(row);width=300*value/maximum if value is not None else 0
                draw.rectangle((left+100,y+j*29,left+100+width,y+j*29+18),fill='#4e7c9d' if enabled else '#999999')
                draw.text((left+107+width,y+j*29-2),shown(value,3 if index else 1),font=small,fill='#444444')
                if index==0:
                    draw.text((left+100,y+62+j*0), '最终结果见TXT；失败求解也保留统计',font=small,fill='#888888')
    draw.text((20,495),'空旷未触发重规划时，采用延迟显示“无记录”，不算0；预算用墙钟，GIF用各自仿真时间。',font=small,fill='#666666')
    draw.text((20,525),'ROS数值闭环，非实车/Gazebo；不同闭环输入会分化，不能将一次对照当总体成功率。',font=small,fill='#666666')
    image.save(path)


def gif_pair(left,right,path,scene,max_frames=64):
    trials=[Trial(left,'native3d'),Trial(right,'native3d')]
    for trial in trials:settings(trial.manifest);trial.native_validity=[]
    duration=max(t.end-t.anchor for t in trials)
    ages=np.linspace(0,duration,min(max_frames,max(2,math.ceil(duration/.5)+1)))
    frame_ms=max(30,10*round((ages[1]-ages[0])*500/10))
    title=ImageFont.truetype(FONT,19);small=ImageFont.truetype(FONT,14)
    frames=[]
    for age in ages:
        image=Image.new('RGB',(1160,480),'white');draw=ImageDraw.Draw(image)
        draw.text((18,9),'%s：起步后 %.1fs（各自实测起步对齐）' % (SCENES[scene],age),font=title,fill='#333333')
        draw.text((18,39),'蓝：机器人及实走轨迹　橙：REMANI参考　绿：获准局部轨迹　灰：障碍物　紫：CV预测',font=small,fill='#555555')
        for index,trial in enumerate(trials):
            t=min(trial.anchor+age,trial.end)
            draw.text((index*580+18,67),LABELS[bool(index)],font=title,fill='#333333')
            row=panel(image,trial,t,(index*580+18,103,(index+1)*580-18,408))
            draw=ImageDraw.Draw(image)
            draw.text((index*580+18,418),'%s　v=%+.2f m/s' % (trial.status_at(t),row['v']),font=small,fill='#555555')
        draw.text((18,451),'真实记录回放，整机67球投影；固定机械臂ROS数值闭环，不是实车/Gazebo。',font=small,fill='#777777')
        frames.append(image.convert('P',palette=Image.ADAPTIVE,colors=96))
    frames[0].save(path,save_all=True,append_images=frames[1:],loop=0,disposal=2,optimize=True,
        duration=[frame_ms]*(len(frames)-1)+[900])
    for trial in trials:trial.verify_unchanged()


def report(batch,out,gifs=True):
    batch=Path(batch).resolve();out=Path(out).resolve()
    plan=json.loads((batch/'plan.json').read_text())
    # The plan declares every measurement, including failed/unfinished jobs.
    rows=[];missing=[]
    for job in plan['jobs']:
        name='%s_%s_seed11' % (job['scene'],'cheap' if job['enabled'] else 'unscaled')
        path=batch/name
        if not (path/'run_status.json').exists():missing.append(name);continue
        rows.append(summarize(path,job))
    out.mkdir(parents=True,exist_ok=True)
    chart(rows,out/'在线交付对照.png')
    if gifs:
        for scene in ('static','midcross'):
            pair=[next((r for r in rows if r['scene']==scene and r['enabled']==e),None) for e in (False,True)]
            if all(pair):gif_pair(batch/pair[0]['trial'],batch/pair[1]['trial'],out/(SCENES[scene]+'.gif'),scene)
    result=dict(scope='fixed-arm native ROS SIL; single run per scene/group, not aggregate success rate',
        plan_sha256=hashlib.sha256((batch/'plan.json').read_bytes()).hexdigest(),trials=rows,unmeasured_jobs=missing,
        source_clock='monotonic timestamps for computation/delivery; simulation time for scene/GIF',
        reporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    lines=['廉价变量缩放在线对照',
        '完整REMANI规划与重规划、MPPI执行；固定机械臂，67球。数值ROS仿真，不是Gazebo或实车。',
        '两组都保留原始初始化和原轮约束，准备与优化合用1.2秒；比较只改变变量缩放开关。',
        '计算时间使用墙钟；触发到采用包含搜索、失败重试、验收、发布及MPPI接收，不能等同单次优化时间。','']
    for scene in SCENES:
        lines.append(SCENES[scene])
        for enabled in (False,True):
            r=next((r for r in rows if r['scene']==scene and r['enabled']==enabled),None)
            if r is None:lines.append(LABELS[enabled]+'：尚无完整运行记录。');continue
            episode=next((e for e in r['delivery_episodes'] if 'publish_mono' in e),{})
            lines += ['%s：终止=%s；记录有效=%s；采样物理碰撞=%s；后端成功%s/%s次。' %
                (LABELS[enabled],r['termination'],r['interface_valid'],r['physical_sample_collision'],r['optimizer_successes'],r['optimizer_calls']),
                '准备中位数%sms；准备＋优化中位数%sms；MPPI求解＋验收P95=%sms。' %
                (shown(r['prepare_ms']['p50']),shown(r['prepare_plus_solver_ms']['p50']),shown(r['mppi_callback_ms']['p95'])),
                '首次重规划触发到发布%ss，到MPPI实际采用%ss；结束时仍未交付的事件%s个。' %
                (shown(episode.get('trigger_to_publish_wall_s'),3),shown(episode.get('trigger_to_first_kernel_wall_s'),3),r['incomplete_delivery_episodes'])]
        lines.append('')
    if missing:lines.append('尚未完成的原计划运行：'+','.join(missing)+'。没有补种子或筛掉失败。')
    lines += ['“无记录”表示没有相应事件或匹配证据，不表示耗时0。每组只跑一次；闭环路线和之后的规划输入会分化。',
        'GIF来自实际记录：保留机器人投影、REMANI参考与获准MPPI局部轨迹。没有重新调用控制器生成画面。']
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')
    print(out)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,required=True)
    p.add_argument('--out',type=Path);p.add_argument('--no-gifs',action='store_true');a=p.parse_args()
    report(a.batch,a.out or a.batch/'report',not a.no_gifs)

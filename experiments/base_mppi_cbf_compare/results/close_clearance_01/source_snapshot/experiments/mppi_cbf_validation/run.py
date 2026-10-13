#!/usr/bin/env python3
"""Build/run a ROS-free screening experiment; never publishes robot commands."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
COLORS = {'mppi': '#2678b2', 'mppi_cbf': '#e87924'}


def dh(alpha, a, d, theta):
    c,s,ca,sa = math.cos(theta),math.sin(theta),math.cos(alpha),math.sin(alpha)
    return np.array([[c,-s*ca,s*sa,a*c],[s,c*ca,-c*sa,a*s],[0,sa,ca,d],[0,0,0,1]])


def geometry(out):
    """Conservative fixed-pose XY cover; NOT a verified full-body mesh model."""
    configs=WORKSPACE/'src/mm_remani_planner/plan_manage/config'
    profile=yaml.safe_load((configs/'piper_param.yaml').read_text())['mm']
    exp=yaml.safe_load((configs/'exp0_param.yaml').read_text())['fsm']
    q=np.deg2rad(exp['init_state'][2:])
    if len(q)!=6: raise ValueError('expected six arm joints')
    length,width=profile['mobile_base_length'],profile['mobile_base_width']
    radius=math.hypot(length/4,width/2)
    body=[[-length/4,0,radius],[length/4,0,radius]]
    hpi=math.pi/2
    params=[(-hpi,0,.123,0),(0,.28503,0,math.radians(-172.22)),
            (hpi,-.021984,0,math.radians(-102.78)),(-hpi,0,.25075,0),
            (hpi,0,0,0),(0,0,.211,0)]
    samples=[[(0,y,0) for y in (0,.06,.123)],[(x,0,0) for x in (0,-.07,-.14,-.21,-.285)],
             [(0,0,0),(.022,0,0)],[(0,y,0) for y in (0,.06,.12,.18,.25)],
             [(0,0,0)],[(0,0,z) for z in (0,-.07,-.14,-.211)]]
    mount=profile['base_mani_fixed_joint_xyz_ypr']
    if any(abs(v)>1e-12 for v in mount[3:]):
        raise ValueError('screen adapter requires current zero-rotation Piper mount')
    transform=np.eye(4);transform[:3,3]=mount[:3];points=[]
    for joint,par,local in zip(q,params,samples):
        a,b,c,d=par;transform=transform@dh(a,b,c,joint+d)
        points.extend([(transform@np.r_[p,1])[:3] for p in local])
    pts=np.asarray(points);center=.5*(pts[:,:2].min(axis=0)+pts[:,:2].max(axis=0))
    radius=np.linalg.norm(pts[:,:2]-center,axis=1).max()+profile['manipulator_thickness']+.10
    body.append([float(center[0]),float(center[1]),float(radius)])
    np.savetxt(out/'geometry.csv',np.asarray(body),delimiter=',')
    g={'body_disks_xy_r':body,'arm_q_deg':np.rad2deg(q).tolist(),
       'arm_collision_points_xyz':pts.tolist(),'base_length':length,'base_width':width,
       'tool_allowance_m':.10,'scope':'fixed-posture planar screen; not calibrated whole-body geometry'}
    (out/'geometry.json').write_text(json.dumps(g,indent=2)+'\n')
    return g


def load_trace(path):
    return np.atleast_1d(np.genfromtxt(path,delimiter=',',names=True))


def read_obstacles(out):
    objects={}
    with (out/'obstacles.csv').open() as f:
        for row in csv.DictReader(f):
            key=(row['scenario'],round(float(row['t']),2))
            objects.setdefault(key,[]).append(tuple(float(row[k]) for k in ('x','y','r')))
    return objects


def render(out):
    with (out/'runs.csv').open() as f:rows=list(csv.DictReader(f))
    if not rows:raise ValueError('no benchmark runs')
    names=list(dict.fromkeys(r['scenario'] for r in rows));aggregate=[]
    objects=read_obstacles(out)
    fig,axes=plt.subplots(math.ceil(len(names)/3),3,figsize=(15,4*math.ceil(len(names)/3)),squeeze=False)
    for name,ax in zip(names,axes.flat):
        ax.plot([0,3],[0,0],'k--',alpha=.35,label='Straight reference')
        xmin,xmax=-.4,3.3
        for mode,color in COLORS.items():
            group=[r for r in rows if r['scenario']==name and r['mode']==mode];traces=[]
            for r in group:
                d=load_trace(out/f"{name}__{mode}__{r['seed']}.csv");traces.append(d)
                xmin=min(xmin,float(d['x'].min())-.2);xmax=max(xmax,float(d['x'].max())+.2)
                ax.plot(d['x'],d['y'],color=color,alpha=.85 if r['seed']=='0' else .35,label=mode if r['seed']=='0' else None)
                ax.scatter(d['x'][-1],d['y'][-1],marker='x',s=40 if int(r['collision']) else 20,
                           color='red' if int(r['collision']) else color)
            lat=np.concatenate([d['total_ms'] for d in traces])
            aggregate.append({'scenario':name,'mode':mode,'runs':len(group),
                'reached':sum(int(r['reached']) for r in group),'collisions':sum(int(r['collision']) for r in group),
                'stop_s_mean':float(np.mean([float(r['stop_seconds']) for r in group])),
                'min_clearance_m':min(float(r['min_clearance']) for r in group),
                'total_ms_p95':float(np.percentile(lat,95)),'total_ms_p99':float(np.percentile(lat,99)),
                'total_ms_max':float(lat.max()),'qp_infeasible':sum(int(r['qp_infeasible']) for r in group),
                'unverified_cycles':sum(int(r['unverified']) for r in group),
                'deadline_misses':sum(int(r['deadline_misses']) for r in group),
                'cycles':sum(int(r['cycles']) for r in group)})
        for t in ([4] if name in ('empty','new_static','blocked','near_miss','late_intrusion') else [3,4.5,6,7.5]):
            for x,y,r in objects.get((name,t),[]):
                ax.add_patch(Circle((x,y),r,facecolor='#d24a43',edgecolor='#d24a43',alpha=.18))
                if name not in ('empty','new_static','blocked','near_miss','late_intrusion'):ax.text(x+.25,y,f'{t:g}s',fontsize=7)
        ax.scatter([0,3],[0,0],c=['black','#22964e'],s=40,zorder=5)
        ax.axhline(1.5,color='gray',lw=1);ax.axhline(-1.5,color='gray',lw=1)
        ax.set(title=name,xlabel='X [m]',ylabel='Y [m]',xlim=(xmin,xmax),ylim=(-1.65,1.65))
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=7,loc='lower left')
    for ax in list(axes.flat)[len(names):]:ax.axis('off')
    fig.suptitle('Synthetic closed-loop screening | all seeds | NOT real-robot results',fontsize=14)
    fig.tight_layout(rect=[0,0,1,.97]);fig.savefig(out/'trajectories.png',dpi=160);plt.close(fig)
    for name in names:
        fig,axes=plt.subplots(4,1,figsize=(11,10),sharex=True)
        for mode,color in COLORS.items():
            d=load_trace(out/f'{name}__{mode}__0.csv')
            for ax,key in zip(axes,['v','w','clearance','total_ms']):ax.plot(d['t'],d[key],color=color,label=mode,lw=1)
            if mode=='mppi_cbf':
                bad=d['infeasible']>0
                axes[0].scatter(d['t'][bad],d['v'][bad],marker='x',c='red',s=12,label='QP infeasible')
        for ax,label in zip(axes,['Actual v [m/s]','Actual w [rad/s]','Truth clearance [m]','Compute time [ms]']):
            ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper right')
        axes[2].axhline(0,color='red',ls='--',lw=1);axes[2].axhline(.08,color='gray',ls=':',lw=1)
        axes[3].axhline(50,color='red',ls='--',lw=1);axes[-1].set_xlabel('Simulation time [s]')
        fig.suptitle(f'{name} | seed 0 (not selected for success)')
        fig.tight_layout(rect=[0,0,1,.97]);fig.savefig(out/f'detail_{name}.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(17,5));x=np.arange(len(names));width=.36
    for i,(mode,color) in enumerate(COLORS.items()):
        values=[next(a for a in aggregate if a['scenario']==n and a['mode']==mode) for n in names]
        for ax,key in zip(axes,['stop_s_mean','total_ms_p95','min_clearance_m']):
            ax.bar(x+(i-.5)*width,[a[key] for a in values],width,label=mode,color=color)
    for ax,title in zip(axes,['Stopped time (mean) [s]','Local computation P95 [ms]','Minimum truth clearance [m]']):
        ax.set_title(title);ax.set_xticks(x);ax.set_xticklabels(names,rotation=40,ha='right',fontsize=8);ax.grid(axis='y',alpha=.2);ax.legend(fontsize=8)
    axes[1].axhline(50,color='red',ls='--');axes[2].axhline(0,color='red',ls='--')
    fig.suptitle('Measured CPU screening - NOT an A/B comparison against native REMANI')
    fig.tight_layout();fig.savefig(out/'summary.png',dpi=160);plt.close(fig)
    (out/'metrics.json').write_text(json.dumps(aggregate,indent=2)+'\n')
    g=json.loads((out/'geometry.json').read_text());fig,ax=plt.subplots(figsize=(6,5));l,w=g['base_length'],g['base_width']
    ax.plot([-.5*l,.5*l,.5*l,-.5*l,-.5*l],[-.5*w,-.5*w,.5*w,.5*w,-.5*w],'k-',label='Base rectangle')
    pts=np.asarray(g['arm_collision_points_xyz']);ax.scatter(pts[:,0],pts[:,1],s=18,label='Arm collision samples (XY)')
    for i,(cx,cy,r) in enumerate(g['body_disks_xy_r']):ax.add_patch(Circle((cx,cy),r,fill=False,color='#2678b2' if i<2 else '#e87924'))
    ax.set(xlim=(-.75,.75),ylim=(-.65,.65),xlabel='Body X [m]',ylabel='Body Y [m]',title='Fixed-posture screening collision cover')
    ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(fontsize=8);fig.tight_layout();fig.savefig(out/'geometry.png',dpi=160);plt.close(fig)
    (out/'render_manifest.json').write_text(json.dumps({
        'rendered_at':time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'plotter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'runs_csv_sha256':hashlib.sha256((out/'runs.csv').read_bytes()).hexdigest(),
        'scope':'plot rendering only; original benchmark manifest and traces unchanged'},indent=2)+'\n')
    return aggregate


def bag_diagnostics(path,out):
    """Original recording only; never substitute historical odom for new control."""
    import rosbag
    topics=['/dynamic_points','/cmd_vel','/planning/dynamic_point_flow_ttc',
            '/planning/safety_speed_limit','/planning/trajectory','/tracking_fault']
    series={t:[] for t in topics};ages=[]
    with rosbag.Bag(str(path),'r') as bag:
        start=bag.get_start_time()
        for topic,msg,stamp in bag.read_messages(topics=topics):
            t=stamp.to_sec()-start
            if topic=='/dynamic_points':value=msg.width*msg.height;ages.append(stamp.to_sec()-msg.header.stamp.to_sec())
            elif topic=='/cmd_vel':value=msg.linear.x
            elif topic=='/planning/trajectory':value=1
            else:value=float(msg.data)
            series[topic].append((t,value))
    with (out/'historical_bag_timeline.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['topic','record_time_s','value'])
        for topic,values in series.items():
            for t,v in values:writer.writerow([topic,t,v])
    fig,axes=plt.subplots(4,1,figsize=(12,9),sharex=True)
    for ax,topic in zip(axes,topics[:4]):
        data=np.asarray(series[topic]);ax.grid(alpha=.2);ax.set_ylabel(topic.rsplit('/',1)[-1],fontsize=9)
        if len(data):
            y=data[:,1].copy();y[~np.isfinite(y)]=np.nan
            if topic.endswith('ttc'):y[(y<0)|(y>10)]=np.nan
            ax.step(data[:,0],y,where='post',lw=1)
        for t,_ in series['/planning/trajectory']:ax.axvline(t,color='green',alpha=.15)
    axes[-1].set_xlabel('Historical bag record time [s]; green lines = trajectory messages')
    fig.suptitle('Historical REMANI recording ONLY - NOT new-controller replay')
    fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(out/'historical_bag.png',dpi=160);plt.close(fig)
    meta={'path':str(path.resolve()),'topic_counts':{k:len(v) for k,v in series.items()},
          'nonempty_dynamic_frames':sum(v>0 for _,v in series['/dynamic_points']),
          'scope':'historical timeline, not a new-controller rollout',
          'prediction_record_age_p95_s':float(np.percentile(ages,95)) if ages else None}
    (out/'historical_bag.json').write_text(json.dumps(meta,indent=2)+'\n');return meta


def report(out,metrics,bag_meta):
    collisions=sum(m['collisions'] for m in metrics);misses=sum(m['deadline_misses'] for m in metrics)
    failures=sum(m['qp_infeasible'] for m in metrics)
    text=['# MPPI / CBF-QP 独立可行性筛选报告','',
          '结果由实际运行 CSV 自动生成；没有发送控制指令，也未修改实车启动链路。',
          '固定机械臂姿态的二维投影闭环实验，不是全身实车验证，不是原生 REMANI 的公平 A/B。','',
          f'碰撞运行 {collisions} 次；计算超时周期 {misses} 次；QP 无解/验证失败周期 {failures} 次。',
          '出现碰撞、QP 无解或大量非必要停车时，不能宣称已实现安全实时避障。','',
          '## 图片','','![轨迹](trajectories.png)','','![统计](summary.png)','',
          '轨迹图包含全部种子；详情图固定为种子 0，不按成功结果选图。红色圆为标注时刻的障碍物外形，不是整个时域同时占据。','',
          '## 统计','','|场景|方法|到达/次数|碰撞|平均停车 s|最小净距 m|P95 ms|P99 ms|QP 失败周期|超时周期|',
          '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for m in metrics:
        text.append(f"|{m['scenario']}|{m['mode']}|{m['reached']}/{m['runs']}|{m['collisions']}|{m['stop_s_mean']:.2f}|{m['min_clearance_m']:.3f}|{m['total_ms_p95']:.2f}|{m['total_ms_p99']:.2f}|{m['qp_infeasible']}|{m['deadline_misses']}|")
    text+=['','## 口径与限制','',
        '- blocked 预期无法通过；停车不等于失败。其他场景 24 s 内未到达如实记录。',
        '- 停车时间：启动 1 s 后、远离目标时 |v|<0.025 m/s，含必要等待及原地转向，不等于误停车。',
        '- workspace_uncertainty 使用当前配置的 0.08 m、0.25 m/s、0.75 m/s² 包络；其他场景是较小的合成包络，不能外推到当前实车感知。',
        '- MPPI 为相关噪声、转向/制动候选的采样加权原型；CBF-QP 为数值线性化离散屏障，求解后再验证非线性约束。无形式化连续时间安全保证。',
        '- QP 无解、观测过期、计算超时都请求有加速度限制的制动；制动仍可能不安全，计入 unverified 与碰撞。',
        '- 独立真实障碍运动以 5 ms 步长检查碰撞，但几何仍为固定姿态投影外包络，未验证自碰撞、三维高度和运动关节。',
        '- 执行滞后 0.12 s、工具余量 0.10 m 是未标定的实验假设。计算耗时不含 ROS 感知、地图构建和驱动。期限违约是丢弃结果并制动的简化模型，非完整异步队列。',
        '- 尚未接入 ESDF、点云 KD-tree 和原生规划器。真实地图成本、原生 REMANI 对照、新控制器真实 bag 影子推演仍待验证。',
        '','## 原始文件','','runs.csv 为逐次结果，场景__方法__种子.csv 为逐周期数据，metrics.json 为统计，manifest.json 记录环境、参数与源码校验和。',
        '','## 下一步','','先通过有绕行空间的闭环场景、计算尾延迟和 CBF 可行性检查，再做真实地图影子节点、原生基线对照与受控低速实车实验。']
    if bag_meta:
        text+=['','## 历史录包单独呈现','',f"录包：{bag_meta['path']}；非空动态帧 {bag_meta['nonempty_dynamic_frames']}。",
               '没有把不同坐标系混用，也没有把历史位置当成新控制闭环。此图仅显示原记录时间线。','','![历史录包](historical_bag.png)']
    (out/'REPORT.md').write_text('\n'.join(text)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seeds',type=int,default=3);p.add_argument('--samples',type=int,default=256)
    p.add_argument('--scenarios',default='empty,near_miss,new_static,crossing,delayed_crossing,blocked,workspace_uncertainty,late_intrusion')
    p.add_argument('--out',type=Path);p.add_argument('--bag',type=Path,help='Historical timeline only, never ROS replay')
    p.add_argument('--render-only',type=Path);args=p.parse_args()
    if args.render_only:
        out=args.render_only.resolve();metrics=render(out)
        meta=json.loads((out/'historical_bag.json').read_text()) if (out/'historical_bag.json').exists() else None
        report(out,metrics,meta);print(out);return
    if args.seeds<1 or args.samples<8:p.error('seeds >= 1 and samples >= 8 required')
    expected={'tracker_max_v':.30,'tracker_max_w':.80,'tracker_max_linear_accel':.35,'tracker_max_angular_accel':1.}
    actual=yaml.safe_load((WORKSPACE/'src/mm_remani/config/real_speed.yaml').read_text())['real_base_tracker']
    for key,value in expected.items():
        if abs(actual[key]-value)>1e-9:raise ValueError(f'Workspace limit {key} changed: update and review prototype first')
    out=(args.out or HERE/'results'/time.strftime('%Y%m%d_%H%M%S')).resolve()
    out.mkdir(parents=True,exist_ok=False);geometry(out);binary=out/'benchmark'
    build=['g++','-std=c++17','-O3','-Wall','-Wextra','-Wpedantic',str(HERE/'benchmark.cpp'),'-o',str(binary)]
    subprocess.run(build,check=True);subprocess.run([str(binary),'--self-test'],check=True)
    command=[str(binary),str(out),str(out/'geometry.csv'),str(args.seeds),str(args.samples),args.scenarios]
    sources=[HERE/'benchmark.cpp',HERE/'run.py',WORKSPACE/'src/mm_remani_planner/plan_manage/config/piper_param.yaml',
             WORKSPACE/'src/mm_remani_planner/plan_manage/config/exp0_param.yaml',WORKSPACE/'src/mm_remani/config/real_speed.yaml']
    manifest={'started':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'platform':platform.platform(),'python':sys.version,
        'numpy':np.__version__,'matplotlib':matplotlib.__version__,'command':command,'build':build,'seeds':args.seeds,'samples':args.samples,
        'sources_sha256':{str(f.relative_to(WORKSPACE)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},
        'limits':{'vmax':.30,'wmax':.80,'acceleration':.35,'angular_acceleration':1.},
        'control_dt':.05,'rollout_dt':.1,'horizon_steps':18,'assumed_actuator_tau':.12,
        'scope':'synthetic closed-loop screen; no native REMANI A/B; no ROS or hardware writes'}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    subprocess.run(command,check=True);metrics=render(out)
    meta=bag_diagnostics(args.bag,out) if args.bag else None;report(out,metrics,meta)
    print(f"REPORT: {out/'REPORT.md'}",flush=True)


if __name__=='__main__':main()

#!/usr/bin/env python3
"""Empirical plots + event/latency audit of complete native ROS trials."""
import argparse
import collections
import csv
import hashlib
import json
import re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Circle

FONT='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
if Path(FONT).exists():
    font_manager.fontManager.ttflist.append(font_manager.FontEntry(fname=FONT,name='OnlineCJK'));plt.rcParams['font.family']='OnlineCJK'
plt.rcParams['axes.unicode_minus']=False
MODES={'remani':('完整 REMANI＋原 Tracker','#566573'),'mppi':('完整 REMANI＋MPPI','#277daf'),'mppi_cbf':('完整 REMANI＋MPPI＋CBF-QP','#d88727')}
SCENES={'empty':'空旷直行','static':'突现静态障碍','crossing':'移动障碍横穿'}


def load(path):return json.loads(path.read_text())


def metrics(p):
    r=load(p/'result.json');m=load(p/'manifest.json');a=np.atleast_1d(np.genfromtxt(p/'state.csv',delimiter=',',names=True));events=load(p/'events.json');trajs=load(p/'trajectories.json');d=load(p/'mppi_diagnostics.json')
    mask=a['t']>=r['goal_sent'];dt=np.diff(a['t'],prepend=a['t'][0]);messages=[e.get('message','') for e in events]
    logs=list((p/'roslogs').glob('*/remani_planner_node-*-stdout.log'));logs=[f for f in logs if f.parent.name!='latest'];log=logs[0].read_text(errors='replace') if logs else ''
    timing=[list(map(float,v)) for v in re.findall(r'total time: ([\d.]+), init: ([\d.]+), optimize: ([\d.]+)',log)]
    active=[z for z in d if z['t']>=r['goal_sent']];computed=[z['kernel_ms'] for z in active if z['kernel_ms']>0]
    obstacles=np.atleast_1d(np.genfromtxt(p/'obstacles.csv',delimiter=',',names=True));appearance=float(obstacles['t'][0]) if obstacles.size else None
    new=[t['t'] for t in trajs if t['action']==1 and t['id']==1 and appearance is not None and t['t']>=appearance]
    row={'scene':r['scene'],'mode':m['mode'],'reason':r['reason'],'duration_after_goal_s':r['elapsed']-r['goal_sent'],
         'trajectory_bundles':r['trajectory_bundles'],'replan_publications':max(0,r['trajectory_bundles']-1),'solver_successful_calls':len(timing),
         'min_clearance_m':r['min_clearance'],'collision_projection':r['collision_conservative_projection'],
         'low_speed_s':float(dt[mask&(abs(a['v'])<.02)].sum()),'native_zero_limit_s':float(dt[mask&(a['safety_limit']>=0)&(a['safety_limit']<=.001)].sum()),
         'perception_healthy_fraction':float(np.mean(a['perception_healthy'][mask])),
         'first_dynamic_points_s':float(a['t'][np.flatnonzero(mask&(a['dynamic_points']>0))[0]]-r['goal_sent']) if np.any(mask&(a['dynamic_points']>0)) else None,
         'obstacle_to_first_new_trajectory_s':new[0]-appearance if new else None,
         'late_candidate_rejections':sum('Rejecting late moving repair' in s for s in messages),
         'optimizer_budget_stops':sum('[Optimizer] Stop after' in s for s in messages),
         'legacy_controller_clock_freezes':sum('freeze reference progress' in s for s in messages),
         'kernel_p95_ms':float(np.percentile(computed,95)) if computed else None,
         'solver_successful_total_p95_ms':float(np.percentile(np.array(timing)[:,0],95)) if timing else None,
         'cbf_interventions':sum(z['cbf_intervention'] for z in active),'qp_failed':sum(z['qp_failed'] for z in active),
         'stop_unverified':sum(z['stop_unverified'] for z in active),'deadline_misses':sum(z['reason']=='deadline_miss' for z in active),
         'max_arm_error_rad':r['arm_max_error'],'progress_contract':m.get('progress_contract','native_virtual_clock'),'planner_variant':m.get('planner_variant','current'),
         'reasons':dict(collections.Counter(z['reason'] for z in active))}
    return row,a,events,trajs,d,obstacles


def render(out):
    trials={};rows=[]
    for p in sorted(out.iterdir()):
        if p.is_dir() and (p/'result.json').exists():
            data=metrics(p);row=data[0];trials[row['scene'],row['mode']]=(p,data);rows.append(row)
    (out/'metrics.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n')
    with (out/'metrics.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    names=[s for s in SCENES if any(key[0]==s for key in trials)]
    fig,axes=plt.subplots(3,len(names),figsize=(5*len(names),11),squeeze=False)
    for col,scene in enumerate(names):
        for row,mode in enumerate(MODES):
            ax=axes[row,col]
            if (scene,mode) not in trials:ax.axis('off');continue
            p,(m,a,e,tr,d,obs)=trials[scene,mode]
            for trajectory in tr:
                if trajectory['action']!=1 or not trajectory['points']:continue
                points=np.array(trajectory['points']);ax.plot(points[:,1],points[:,2],color='#709763',lw=1,ls='--',alpha=.5)
            if len(obs):
                indices=[0] if scene=='static' else np.linspace(0,len(obs)-1,5).astype(int)
                for i in indices:
                    o=obs[i];ax.add_patch(Circle((o['x'],o['y']),o['r'],color='#cc4444',alpha=.22))
            ax.plot(a['x'],a['y'],color=MODES[mode][1],lw=2,label='实测底盘运动')
            ax.plot([],[],'--',color='#709763',label='原生发布的多项式轨迹')
            ax.scatter([0,3],[0,0],c=['black','green'],marker='*',s=65);ax.scatter(a['x'][-1],a['y'][-1],c='red' if m['collision_projection'] else MODES[mode][1],marker='o' if m['reason']=='goal' else 's',s=45)
            ax.axhline(2,color='gray');ax.axhline(-2,color='gray');ax.set_aspect('equal',adjustable='box');ax.set_xlim(-.5,3.6);ax.set_ylim(-2.1,2.1);ax.grid(alpha=.2)
            status='到达' if m['reason']=='goal' else '超时未到达'
            ax.set_title(f'{SCENES[scene]}｜{MODES[mode][0]}\n{status}；新轨迹 {m["replan_publications"]} 次；低速 {m["low_speed_s"]:.1f}s',fontsize=10)
            ax.set_xlabel('X / m');ax.set_ylabel('Y / m');ax.legend(fontsize=7,loc='lower left')
    fig.suptitle('完整当前 REMANI 在线规划／重规划闭环：实际输出对照\n固定机械臂、合成点云＋数值底盘；非 Gazebo／实车；每组合单次探索实验',fontsize=13)
    fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'online_paths.png',dpi=150);plt.close(fig)
    dynamic=[s for s in names if s!='empty']
    if dynamic:
        fig,axes=plt.subplots(4,len(dynamic),figsize=(7*len(dynamic),12),squeeze=False)
        for col,scene in enumerate(dynamic):
            for mode,(label,color) in MODES.items():
                if (scene,mode) not in trials:continue
                p,(m,a,e,tr,d,obs)=trials[scene,mode];goal=load(p/'result.json')['goal_sent'];t=a['t']-goal;mask=t>=0
                axes[0,col].plot(t[mask],a['v'][mask],color=color,label=label,lw=1)
                axes[1,col].plot(t[mask],np.where(a['safety_limit'][mask]<0,.30,a['safety_limit'][mask]),color=color,label=label,lw=1)
                axes[2,col].step(t[mask],a['trajectory_count'][mask]-1,where='post',color=color,label=label)
                if d:
                    ts=np.array([z['t']-goal for z in d]);qp=np.array([z['qp_failed'] for z in d]);axes[3,col].plot(ts,np.cumsum(qp),color=color,label=label)
            for row,title in enumerate(['实际线速度 / m·s⁻¹','原生安全限速 / m·s⁻¹（不限速画为 0.30）','实际发布的新轨迹累计数','CBF 过滤失败累计次数（不是计算超时）']):
                ax=axes[row,col];ax.set_title(SCENES[scene]+'｜'+title,fontsize=10);ax.set_xlabel('目标发布后 / s');ax.set_xlim(left=0);ax.grid(alpha=.2);ax.legend(fontsize=7)
        fig.suptitle('停车、原生安全限速、重规划发布与 CBF 失败的时间对应关系',fontsize=13);fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(out/'online_timeline.png',dpi=150);plt.close(fig)
    lines=['# 完整原生 ROS 数值闭环：自动指标','', '仅统计此目录内已完成试验。底盘/传感器为数值模型，不是 Gazebo 或实车。低速含起步、避障停留与终点稳定等待。','', '|场景|模式|结果|用时/s|新轨迹数|零限速/s|低速/s|净距/m|QP失败|局部P95/ms|','|---|---|---|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append('|{}|{}|{}|{:.2f}|{}|{:.2f}|{:.2f}|{:.3f}|{}|{}|'.format(r['scene'],r['mode'],r['reason'],r['duration_after_goal_s'],r['replan_publications'],r['native_zero_limit_s'],r['low_speed_s'],r['min_clearance_m'],r['qp_failed'],'—' if r['kernel_p95_ms'] is None else round(r['kernel_p95_ms'],2)))
    lines+=['','净距为固定全身二维包络对真值几何的保守评估，非原生三维网格距离。控制器使用测量地图，真值只用于评价。','成功优化调用耗时不含失败调用、地图等待和发布前后校验，不能冒充端到端重规划耗时。','CBF 输出停止但 stop_unverified=true 时不构成安全保证。单次探索测试不可用于宣称成功率或实车安全性。']
    (out/'SUMMARY.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps(rows,indent=2,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);render(p.parse_args().out)

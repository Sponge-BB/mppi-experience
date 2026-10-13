#!/usr/bin/env python3
"""Time-aligned animation from recorded traces, no re-simulation or smoothing."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
from visualize import MODES, NAMES, PROFILES, read, trace


def animate(out,scene='crossing',profile='workspace',seed=0):
    rows=read(out/'runs.csv')
    selected={m:next(r for r in rows if r['scenario']==scene and r['profile']==profile and r['mode']==m and r['seed']==str(seed)) for m in MODES}
    data={m:trace(out,r) for m,r in selected.items()}
    refs=np.array([(float(r['x']),float(r['y'])) for r in read(out/'references.csv') if r['scenario']==scene])
    obstacles={}
    for r in read(out/'obstacles.csv'):
        if r['scenario']==scene:
            obstacles.setdefault(int(round(float(r['t'])*20)),[]).append(tuple(float(r[k]) for k in ('x','y','r')))
    body=json.loads((out/'geometry.json').read_text())['body_disks_xy_r']
    xmin=min(-.7,min(float(d['x'].min()) for d in data.values())-.5)
    xmax=max(3.5,max(float(d['x'].max()) for d in data.values())+.5)
    fig,axes=plt.subplots(1,3,figsize=(12,5),sharex=True,sharey=True)
    artists={}
    for ax,(mode,(label,color)) in zip(axes,MODES.items()):
        ax.plot(refs[:,0],refs[:,1],'k--',alpha=.3)
        ax.scatter(refs[-1,0],refs[-1,1],marker='*',color='green',s=60)
        ax.set(xlim=(xmin,xmax),ylim=(-1.85,1.85),xlabel='X / m',title=label)
        ax.set_aspect('equal');ax.grid(alpha=.2)
        trail,=ax.plot([],[],color=color,lw=2)
        discs=[]
        for _,_,r in body:
            circle=Circle((0,0),r,fill=False,color=color,lw=1.2);ax.add_patch(circle);discs.append(circle)
        heading,=ax.plot([],[],color=color,lw=2)
        obs=[]
        for _ in range(7):
            circle=Circle((0,0),.1,color='#c34840',alpha=.6,visible=False);ax.add_patch(circle);obs.append(circle)
        text=ax.text(.02,.98,'',transform=ax.transAxes,va='top',fontsize=8)
        artists[mode]=(trail,discs,heading,obs,text)
    stamp_text=fig.suptitle('',fontsize=12)
    fig.tight_layout(rect=[0,0,1,.81])
    def update(stamp):
        stamp_text.set_text(f'{NAMES[scene]}｜{PROFILES[profile]}｜种子 {seed}｜t={stamp:.1f}s\n局部控制层数值闭环，非完整 REMANI／Gazebo／实车；圆为固定整机覆盖')
        for mode,d in data.items():
            trail,discs,heading,obs,text=artists[mode]
            k=int(np.searchsorted(d['t'],stamp,side='right')-1)
            if k<0:
                x=y=yaw=v=w=0.;net='—';status='开始';trail.set_data([],[])
            else:
                p=d[k];x,y,yaw,v,w=(float(p[q]) for q in ('x','y','yaw','v','w'));net=f"{p['clearance']:.3f}m"
                status='运动／保持'
                if p['guard']:status='共同保护介入'
                if p['qp_failed']:status='QP／验证失败'
                if p['deadline_miss']:status='计算超时'
                if stamp>d['t'][-1]:status='已到达' if int(selected[mode]['reached']) else ('碰撞，运行结束' if int(selected[mode]['collision']) else '超时，运行结束')
                trail.set_data(d['x'][:k+1],d['y'][:k+1])
            for disk,(bx,by,r) in zip(discs,body):
                disk.center=(x+math.cos(yaw)*bx-math.sin(yaw)*by,y+math.sin(yaw)*bx+math.cos(yaw)*by)
            heading.set_data([x,x+.25*math.cos(yaw)],[y,y+.25*math.sin(yaw)])
            visible=obstacles.get(int(round(stamp*20)),[])
            for j,circle in enumerate(obs):
                circle.set_visible(j<len(visible))
                if j<len(visible):circle.center=visible[j][:2];circle.set_radius(visible[j][2])
            text.set_text(f'v={v:.2f} m/s，ω={w:.2f} rad/s\n真值净距：{net}\n{status}')
        return []
    end=max(float(d['t'][-1]) for d in data.values())
    animation=FuncAnimation(fig,update,frames=np.arange(0,end+.2,.2),interval=100,blit=False)
    path=out/f'animation_{scene}_{profile}.gif'
    animation.save(str(path),writer=PillowWriter(fps=10),dpi=85)
    plt.close(fig)
    print(path,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('out',type=Path)
    p.add_argument('--scenario',default='crossing');p.add_argument('--profile',default='workspace');p.add_argument('--seed',type=int,default=0)
    a=p.parse_args();animate(a.out.resolve(),a.scenario,a.profile,a.seed)

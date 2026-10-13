#!/usr/bin/env python3
"""Animate actual CSV states and actual obstacle positions (not an artist drawing)."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
import numpy as np
from run import COLORS, load_trace, read_obstacles


def animate(out,scenario,seed=0):
    traces={m:load_trace(out/f'{scenario}__{m}__{seed}.csv') for m in COLORS}
    body=json.loads((out/'geometry.json').read_text())['body_disks_xy_r']
    objects=read_obstacles(out)
    end=max(d['t'][-1] for d in traces.values())
    fig,axes=plt.subplots(1,2,figsize=(9,4));artists=[]
    max_objects=max([len(v) for (name,t),v in objects.items() if name==scenario] or [0])
    for ax,(mode,color) in zip(axes,COLORS.items()):
        ax.set(xlim=(-.6,3.5),ylim=(-1.6,1.6),xlabel='X [m]',ylabel='Y [m]',title=mode)
        ax.set_aspect('equal');ax.grid(alpha=.2);ax.axhline(1.5,c='gray');ax.axhline(-1.5,c='gray')
        ax.plot([0,3],[0,0],'k--',alpha=.2);ax.scatter([3],[0],c='green',marker='*',s=90)
        line,=ax.plot([],[],c=color,lw=1.5)
        robot=[Circle((0,0),r,fill=False,ec=color,lw=1.2) for _,_,r in body]
        for p in robot:ax.add_patch(p)
        obs=[Circle((0,0),.2,fc='red',alpha=.35) for _ in range(max_objects)]
        for p in obs:ax.add_patch(p)
        label=ax.text(.02,.02,'',transform=ax.transAxes,fontsize=8)
        artists.append((line,robot,obs,label))
    title=fig.suptitle(f'Synthetic closed loop - seed {seed} - not real-robot results')
    def update(t):
        for (mode,_),(line,robot,obstacles,label) in zip(COLORS.items(),artists):
            d=traces[mode];i=max(0,min(len(d)-1,int(np.searchsorted(d['t'],t,side='right')-1)))
            s=d[i];c,sn=np.cos(s['yaw']),np.sin(s['yaw'])
            line.set_data(d['x'][:i+1],d['y'][:i+1])
            for p,(x,y,r) in zip(robot,body):p.center=(s['x']+c*x-sn*y,s['y']+sn*x+c*y)
            positions=objects.get((scenario,round(round(t/.05)*.05,2)),[])
            for j,p in enumerate(obstacles):
                p.set_visible(j<len(positions))
                if j<len(positions):p.center=positions[j][:2];p.set_radius(positions[j][2])
            status=' | QP failed' if s['infeasible'] else ''
            if s['clearance']<0:status=' | COLLISION (run ended)'
            label.set_text(f"v={s['v']:.2f} m/s | clearance={s['clearance']:.2f} m{status}")
        title.set_text(f'{scenario} | t={t:.1f} s | seed {seed} | synthetic, 2x playback')
        return []
    fig.tight_layout(rect=[0,0,1,.92])
    animation=FuncAnimation(fig,update,frames=np.arange(0,end+.001,.2),interval=100,blit=False)
    suffix=f'_seed{seed}' if seed else ''
    path=out/f'animation_{scenario}{suffix}.gif';animation.save(path,writer=PillowWriter(fps=10),dpi=90);plt.close(fig)
    print(path)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('out',type=Path)
    parser.add_argument('--scenario',default='crossing');parser.add_argument('--seed',type=int,default=0)
    args=parser.parse_args();animate(args.out.resolve(),args.scenario,args.seed)

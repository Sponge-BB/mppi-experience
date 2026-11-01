#!/usr/bin/env python3
"""Select valid encounter trials without changing raw data; draw two simple figures."""
import argparse
import json
from pathlib import Path
import numpy as np
from matplotlib.patches import Circle
from matplotlib.lines import Line2D
from report import plt,metrics

LABELS={'remani':'当前 REMANI','mppi':'+ MPPI','mppi_cbf':'+ MPPI + CBF-QP'}
COLORS={'remani':'#66717D','mppi':'#207CA8','mppi_cbf':'#D88A26'}


def finalize(out,base,crossing):
    out.mkdir(parents=True,exist_ok=False);selection={}
    for scene in ('empty','static','crossing'):
        for mode in LABELS:
            name=scene+'_'+mode;source=(crossing if scene=='crossing' else base)/name
            if not (source/'result.json').exists():raise RuntimeError('missing completed trial '+str(source))
            (out/name).symlink_to(source.resolve(),target_is_directory=True);selection[name]=str(source.resolve())
    (out/'selection.json').write_text(json.dumps({'trials':selection,'reason':'Empty/static from main complete-online test; crossing replaced by motion-triggered encounter so all robots were already moving before obstacle arrival. Original startup-obstacle tests are retained as diagnostics.'},indent=2)+'\n')
    rows=[];data={}
    for name in selection:
        p=out/name;values=metrics(p);r,a,events,traj,d,obs=values;result=json.loads((p/'result.json').read_text())
        if len(obs):
            start=obs['t'][0];end=result['elapsed']-(1. if result['reason']=='goal' else 0.);dt=np.diff(a['t'],prepend=a['t'][0]);active=(a['t']>=start)&(a['t']<end)
            r['obstacle_low_speed_s']=float(dt[active&(abs(a['v'])<.02)].sum())
            r['obstacle_native_stop_s']=float(dt[active&(a['safety_limit']>=0)&(a['safety_limit']<=.001)].sum())
        else:r['obstacle_low_speed_s']=0.;r['obstacle_native_stop_s']=0.
        data[r['scene'],r['mode']]=values;rows.append(r)
    (out/'metrics.json').write_text(json.dumps(rows,indent=2,ensure_ascii=False)+'\n')
    handles=[Line2D([0],[0],color=COLORS[k],lw=2.8,label=LABELS[k]) for k in LABELS]
    fig,axes=plt.subplots(1,2,figsize=(11,4.8))
    for ax,scene,title in zip(axes,['static','crossing'],['静态障碍：绕行轨迹','动态障碍：运动后横穿']):
        for mode in LABELS:
            r,a,*_=data[scene,mode];ax.plot(a['x'],a['y'],color=COLORS[mode],lw=2.5)
            if r['reason']!='goal':
                ax.scatter(a['x'][-1],a['y'][-1],marker='s',s=60,color=COLORS[mode],zorder=6)
                ax.annotate('超时卡住',xy=(a['x'][-1],a['y'][-1]),xytext=(10,18),textcoords='offset points',color=COLORS[mode],fontsize=11)
            if r['collision_projection']:ax.scatter(a['x'],a['y'],c='red',s=2)
        if scene=='static':
            ax.add_patch(Circle((1.8,0),.23,facecolor='#E6B3B3',edgecolor='#BD6666'));ax.text(1.8,-.4,'新障碍',ha='center',fontsize=10,color='#904848')
        else:
            ax.annotate('',xy=(1.65,1.5),xytext=(1.65,-1.5),arrowprops=dict(arrowstyle='->',color='#BE7777',ls='--',lw=1.4))
            for y in [-1,0,1]:ax.add_patch(Circle((1.65,y),.23,facecolor='#E6B3B3',edgecolor='none',alpha=.45))
        ax.scatter([0],[0],color='black',s=24,zorder=6);ax.scatter([3],[0],color='#34824C',s=100,marker='*',zorder=6)
        ax.text(0,-.2,'起点',ha='center',fontsize=10);ax.text(3,-.2,'目标',ha='center',fontsize=10)
        ax.set_title(title,fontsize=13);ax.set_xlim(-.55,3.5);ax.set_ylim(-1.7,1.7);ax.set_aspect('equal');ax.set_xlabel('X / m');ax.set_ylabel('Y / m');ax.grid(alpha=.15)
        ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.99),ncol=3,frameon=False,fontsize=12)
    fig.text(.5,.015,'完整在线规划＋重规划 · 固定机械臂 · 数值仿真，非实车 · 曲线为实际运动记录',ha='center',fontsize=10,color='#555555')
    fig.tight_layout(rect=[0,.05,1,.89]);fig.savefig(out/'01_paths.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4.6));width=.23;x=np.arange(2)
    for j,mode in enumerate(LABELS):
        group=[data[s,mode][0] for s in ['static','crossing']]
        for ax,key in zip(axes,['duration_after_goal_s','obstacle_low_speed_s']):
            vals=[r[key] for r in group];bars=ax.bar(x+(j-1)*width,vals,width,color=COLORS[mode],label=LABELS[mode])
            for bar,r,v in zip(bars,group,vals):
                label=f'{v:.1f}'
                if r['reason']!='goal':bar.set_hatch('//');label+='\n未完成'
                ax.text(bar.get_x()+width/2,v+.7,label,ha='center',va='bottom',fontsize=10)
    for ax,title in zip(axes,['任务用时（秒，越少越好）','障碍出现后低速／停留（秒）']):
        ax.set_title(title,fontsize=13);ax.set_xticks(x);ax.set_xticklabels(['静态障碍','动态横穿'],fontsize=12);ax.grid(axis='y',alpha=.15);ax.set_axisbelow(True);ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
        ax.set_ylim(0,ax.get_ylim()[1]*1.22)
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.995),ncol=3,frameon=False,fontsize=12)
    fig.text(.5,.014,'单次探索测试；低速按 |v| < 0.02 m/s 统计，含原地转向，剔除终点稳定等待。斜线柱＝超时未到达。',ha='center',fontsize=9,color='#555555')
    fig.tight_layout(rect=[0,.06,1,.89]);fig.savefig(out/'02_times.png',dpi=160);plt.close(fig)
    print(out)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--base',type=Path,required=True);p.add_argument('--crossing',type=Path,required=True);a=p.parse_args();finalize(a.out,a.base,a.crossing)

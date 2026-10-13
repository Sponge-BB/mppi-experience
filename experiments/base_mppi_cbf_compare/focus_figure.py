#!/usr/bin/env python3
"""Compact two-scenario figure; the full unselected suite remains in RESULTS.md."""
import hashlib
import json
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from visualize import MODES, read, trace


def render(full,supplement):
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for ax,out,scene,title,ox in zip(axes,(full,supplement),('new_static','close_clearance'),
                                  ('新出现静态障碍','近距离出现，出现时留有净距'),(1.55,1.88)):
        rows=[r for r in read(out/'runs.csv') if r['scenario']==scene and r['profile']=='workspace']
        ax.plot([0,3],[0,0],'k--',alpha=.35,lw=1)
        ax.add_patch(Circle((ox,0),.25,color='#bd4540',alpha=.55))
        counts=[]
        for mode,(label,color) in MODES.items():
            group=[r for r in rows if r['mode']==mode]
            for row in group:
                d=trace(out,row)
                ax.plot(d['x'],d['y'],color=color,alpha=.95 if row['seed']=='0' else .35,lw=2 if row['seed']=='0' else 1,
                        label=label if row['seed']=='0' else None)
                ax.scatter(d['x'][-1],d['y'][-1],color=color,marker='o' if int(row['reached']) else 's',s=28)
            counts.append(f"{mode}: {sum(int(r['reached']) for r in group)}/{len(group)}")
        ax.scatter(0,0,c='black',s=35,zorder=5);ax.scatter(3,0,c='green',marker='*',s=80,zorder=5)
        ax.set(xlim=(-.5,3.5),ylim=(-1.4,1.4),xlabel='X / m',ylabel='Y / m',title=title+'\n32s 内到达：'+'，'.join(counts))
        ax.set_aspect('equal');ax.grid(alpha=.2)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='lower center',ncol=3,fontsize=9)
    fig.suptitle('固定机械臂：有／无 MPPI 与 CBF-QP 的底盘局部控制对照\n当前工程参数，全部种子；原 Tracker 组不含 REMANI 在线重规划',fontsize=13)
    fig.tight_layout(rect=[0,.16,1,.84]);target=full/'focused_comparison.png';fig.savefig(target,dpi=150);plt.close(fig)
    (full/'focused_comparison_manifest.json').write_text(json.dumps({
        'plotter_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'source_runs_sha256':{str(p/'runs.csv'):hashlib.sha256((p/'runs.csv').read_bytes()).hexdigest() for p in (full,supplement)},
        'scope':'two labeled static-appearance scenarios only; full suite including dynamic failures is in RESULTS.md'},indent=2)+'\n')
    print(target,flush=True)


if __name__=='__main__':
    render(Path(sys.argv[1]).resolve(),Path(sys.argv[2]).resolve())

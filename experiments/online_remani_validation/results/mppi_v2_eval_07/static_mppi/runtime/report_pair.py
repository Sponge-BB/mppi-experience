#!/usr/bin/env python3
"""Two-method report. No CBF panel or online robot access."""
import argparse
import json
import re
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.font_manager import FontProperties
from report import metrics

FONT=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
MODES=[('remani','REMANI + Tracker','#666666'),('mppi','REMANI + MPPI','#367594')]

def tracking(a,trajs):
    errors=[];curve_errors=[];bundle=[];cursor=0;count=0
    for s in a:
        while cursor<len(trajs) and trajs[cursor]['t']<=s['t']:
            t=trajs[cursor];cursor+=1
            if t['action']!=1:bundle=[]
            else:
                if t['id']==1:bundle=[];count+=1
                bundle.append(t)
        valid=[p for t in bundle if t['stamp']<=1000+s['t'] for p in t['points']]
        if len(valid)<2 or np.hypot(s['x']-3,s['y'])<=.35:continue
        p=np.array(valid)[:,1:3];v=np.diff(p,axis=0);d=np.array([s['x'],s['y']])-p[:-1]
        f=np.clip(np.sum(d*v,axis=1)/np.maximum(1e-15,np.sum(v*v,axis=1)),0,1)
        error=float(np.linalg.norm(d-f[:,None]*v,axis=1).min());errors.append(error)
        if count>=2:curve_errors.append(error)
    def stats(x):return {'mean_m':float(np.mean(x)),'p95_m':float(np.percentile(x,95)),'max_m':float(np.max(x)),'samples':len(x)} if x else None
    return {'all_active':stats(errors),'after_replan':stats(curve_errors),'definition':'nearest current activated polyline, excludes goal radius 0.35 m, includes stationary samples, each run has its own reference'}

def render(out):
    records=[];data={}
    for scene in ('empty','static','crossing'):
        for mode,_,_ in MODES:
            p=out/(scene+'_'+mode)
            if not (p/'result.json').exists():continue
            row,a,e,tr,d,obs=metrics(p);dt=np.diff(a['t']);v=a['v'][:-1]
            crash=re.findall(r'REQUIRED process \[([^\]]+)\] has died', (p/'launch_stdout.log').read_text(errors='replace'))
            if crash:row.update(reason='process_failure',failed_nodes=crash)
            row.update(reverse_distance_m=float(np.sum(np.maximum(-v,0)*dt)),reverse_time_s=float(np.sum(dt[v<-.02])),tracking=tracking(a,tr))
            records.append(row);data[scene,mode]=(a,tr,obs,row)
    (out/'pair_metrics.json').write_text(json.dumps(records,indent=2,ensure_ascii=False)+'\n')
    fig,axes=plt.subplots(2,2,figsize=(9,7.5))
    for j,scene in enumerate(('static','crossing')):
        for i,(mode,label,color) in enumerate(MODES):
            ax=axes[j,i];a,tr,obs,row=data[scene,mode]
            for t in tr:
                if t['action']==1 and t['points']:
                    p=np.array(t['points']);ax.plot(p[:,1],p[:,2],'--',c='#a6ada5',lw=1)
            ax.plot(a['x'],a['y'],c=color,lw=2)
            if len(obs):
                for k in ([0] if scene=='static' else np.linspace(0,len(obs)-1,4).astype(int)):
                    o=obs[k];ax.add_patch(Circle((o['x'],o['y']),o['r'],fc='#cccccc',alpha=.65))
            ax.scatter([3],[0],marker='x',c='black');ax.set(xlim=(-.6,3.6),ylim=(-2.1,2.1),xlabel='x / m',ylabel='y / m');ax.set_aspect('equal');ax.grid(alpha=.15)
            title=('静态' if scene=='static' else '动态')+' | '+label+(' | 到达' if row['reason']=='goal' else ' | 未完成')
            ax.set_title(title,fontproperties=FONT,fontsize=11)
    fig.suptitle('实线：实际运动   虚线：原生规划轨迹   灰色：障碍物\n单次在线数值仿真，非 Gazebo / 实车',fontproperties=FONT,fontsize=12)
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(out/'pair_paths.png',dpi=150);plt.close(fig)
    lines=['# MPPI v2 双组实验','', '对照：当前工作空间 REMANI + 原 Tracker，不是未经修改的论文 baseline。',
      '只运行 remani 与 mppi，没有 CBF。单次在线数值仿真，不构成统计性能结论。','',
      '|场景|方案|结果|用时/s|倒退/m|最小净距/m|内核P95/ms|超时周期|',
      '|---|---|---|---:|---:|---:|---:|---:|']
    for r in records:
        lines.append('|{}|{}|{}|{:.2f}|{:.3f}|{:.3f}|{}|{}|'.format(r['scene'],r['mode'],r['reason'],r['duration_after_goal_s'],r['reverse_distance_m'],r['min_clearance_m'],round(r['kernel_p95_ms'],2) if r['kernel_p95_ms'] is not None else '—',r['deadline_misses']))
    lines+=['','跟踪误差见 pair_metrics.json：每组自己的当前参考路径，包含停留样本，不是同轨迹性能排名。',
      '动态预测的远期不确定性在 v2 中为软风险项，现时不确定性和固定余量仍参与碰撞代价；这改变了保守性，不意味着实车安全已被证明。',
      '原生规划器/急停保持不变，因此上层零限速或轨迹交接等待仍可造成停车。']
    (out/'PAIR_RESULTS.md').write_text('\n'.join(lines)+'\n');print(json.dumps(records,indent=2,ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);render(p.parse_args().out)

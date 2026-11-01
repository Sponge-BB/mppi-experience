#!/usr/bin/env python3
"""Free-field counterfactual for a retained endpoint stall, no ROS/control I/O.

Not a replay of the sensor field: use a deliberately obstacle-free field to
test whether the frozen local kernel can stall without collision rejection.
Only initial lateral position changes; code, shape, limits, reference are fixed.
"""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from report_collab_factorial import read,lines,sha,csv_write
from test_mppi_v2 import Controller,DP,ptr


def diagnose(trial,build,out):
    out.mkdir(parents=True,exist_ok=False)
    ref=np.ascontiguousarray(lines(trial/'consumed_references.jsonl')[-1]['reference'])
    original=np.array(read(trial/'result.json')['final_state']);records=[];costs=[]
    far=np.array([[4.,3.5,.1,0.,.2,1.]])
    def controller(state):
        c=Controller(build,trial,seed=4)
        c.lib.online_decision_debug.argtypes=[ct.c_void_p,DP]
        c.lib.online_search_trace.argtypes=[ct.c_void_p,DP,ct.c_int];c.lib.online_search_trace.restype=ct.c_int
        # Prime the retained "has seen dynamic motion" branch; far actor poses
        # never obstruct any tested route. Later calls use no actors at all.
        c.step(state,ref,far);return c
    for y in sorted(set(np.arange(.20,.341,.01).tolist()+[original[1]])):
        state=original.copy();state[1]=y;c=controller(state)
        for _ in range(3):u=c.step(state,ref)
        debug=np.zeros(12);c.lib.online_decision_debug(c.handle,ptr(debug))
        records.append(dict(y=float(y),goal_distance_m=float(np.linalg.norm(state[:2]-ref[-1,:2])),
            v=float(u[0]),w=float(u[1]),selected=int(debug[5]),intervention=bool(debug[6])))
        n=c.lib.online_search_trace(c.handle,None,0);trace=np.zeros((n,8))
        c.lib.online_search_trace(c.handle,ptr(trace),n)
        for ident in np.unique(trace[:,0]):
            row=trace[trace[:,0]==ident][0]
            costs.append(dict(y=float(y),candidate=int(ident),cost=float(row[4]),gap=float(row[5]),
                feasible=bool(row[6]),selected=bool(row[7])))
        c.close()
    paths={};ends={}
    for label,y in [('recorded',original[1]),('inside_capture',.24)]:
        state=original.copy();state[1]=y;c=controller(state);rows=[]
        for k in range(400):
            u=c.step(state,ref);c.plant(state,u);rows.append([.05*(k+1),*state,*u])
        paths[label]=np.array(rows);ends[label]=dict(state=state.tolist(),goal_distance_m=float(np.linalg.norm(state[:2]-ref[-1,:2])))
        c.close()
    csv_write(out/'offset_sweep.csv',records);csv_write(out/'candidate_costs.csv',costs)
    np.savez_compressed(out/'free_field_plant.npz',**paths)
    result=dict(scope=__doc__,online_trial=str(trial),build=str(build),original_state=original.tolist(),
        original_goal_distance_m=float(np.linalg.norm(original[:2]-ref[-1,:2])),
        field='constant 10m; no obstacle after branch priming; not original sensor replay',
        limits=read(trial/'limits.json'),end_states=ends,
        source_sha256={str(p):sha(p) for p in [trial/'state.csv',trial/'limits.json',trial/'geometry.csv',
            trial/'consumed_references.jsonl',build/'build/libonline_mppi.so',Path(__file__)]})
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',size=10)
    fig,axes=plt.subplots(1,2,figsize=(10,3.8))
    axes[0].plot([r['goal_distance_m'] for r in records],[r['v'] for r in records],'o-',color='#467fa4')
    axes[0].axvline(.25,color='#999999',linestyle='--');axes[0].set_xlabel('距终点 / m',fontproperties=font)
    axes[0].set_ylabel('固定状态下输出线速度 / m/s',fontproperties=font);axes[0].grid(alpha=.2)
    for label,color,title in [('recorded','#b86b66','原记录状态'),('inside_capture','#548a76','仅初始y移到0.24m')]:
        a=paths[label];axes[1].plot(a[:,0],np.linalg.norm(a[:,1:3]-ref[-1,:2],axis=1),color=color,label=title)
    axes[1].set_xlabel('自由场数值推进 / s',fontproperties=font);axes[1].set_ylabel('距终点 / m',fontproperties=font)
    axes[1].legend(prop=font);axes[1].grid(alpha=.2)
    fig.suptitle('自由场补充诊断（冻结V12库，不是在线重放）',fontproperties=font)
    fig.tight_layout(rect=[0,0,1,.92]);fig.savefig(out/'endpoint_diagnostic.png',dpi=150);plt.close(fig)
    print('ENDPOINT_DIAGNOSIS',ends,flush=True);return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();diagnose(a.trial.resolve(),a.build.resolve(),a.out.resolve())

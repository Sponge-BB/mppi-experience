#!/usr/bin/env python3
"""Held original reference: no native replanning, ideal synthetic measurements.
This isolates local ability; it is NOT a full REMANI or safety-chain test.
"""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

def trial(build,fixture,scene,seed=1,crossing_speed=.26,mirror=False,crossing_offset=0.,separate_dynamic=False):
    ctrl=Controller(build,fixture,seed=seed)
    x=np.linspace(0,3,301);v=np.minimum(.18,.8*(3-x))
    reference=np.ascontiguousarray(np.column_stack((x,x*0,x*0,v)))
    state=np.zeros(5);trace=[];routes=[];diagnostics=[];anchor=None
    body=np.atleast_2d(np.loadtxt(fixture/'geometry.csv',delimiter=','))
    xx,yy=np.meshgrid(-7.5+.05*np.arange(300),-5.+.05*np.arange(200))
    if hasattr(ctrl.lib,'online_local_route'):
        ctrl.lib.online_local_route.argtypes=[ct.c_void_p,DP,ct.c_int]
        ctrl.lib.online_local_route.restype=ct.c_int
    if hasattr(ctrl.lib,'online_avoidance_debug'):
        ctrl.lib.online_avoidance_debug.argtypes=[ct.c_void_p,DP]
    if separate_dynamic:
        assert hasattr(ctrl.lib,'online_set_classified_static')
        ctrl.lib.online_set_classified_static.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int]
    reached=False
    for k in range(900):
        t=k*.05
        if anchor is None and state[0]>=.1:anchor=t
        obstacle=None
        if scene=='static' and t>=3.:obstacle=[1.8,0.,0.,0.,.23,0.]
        if scene=='crossing' and anchor is not None and 1.<=t-anchor<=14.:
            sign=-1 if mirror else 1
            obstacle=[1.65,sign*(-1.6+crossing_offset+crossing_speed*(t-anchor-1.)),0.,sign*crossing_speed,.23,1.]
        field=np.minimum(2.-yy,yy+2.)
        if obstacle is not None:field=np.minimum(field,np.hypot(xx-obstacle[0],yy-obstacle[1])-obstacle[4])
        ctrl.field=np.ascontiguousarray(np.maximum(0.,field-np.sqrt(2)*.05))
        if separate_dynamic:
            # Ideal synthetic sensor with known measurement membership. This
            # is NOT segmentation validated on real clouds. Static scenes keep
            # their obstacle; only the measured moving track uses the CV layer.
            static_field=field if scene!='crossing' else np.minimum(2.-yy,yy+2.)
            ctrl.classified_static=np.ascontiguousarray(np.maximum(0.,static_field-np.sqrt(2)*.05))
            ctrl.lib.online_set_classified_static(ctrl.handle,ptr(ctrl.classified_static),300,200)
        obs=np.array([obstacle]) if obstacle is not None and scene=='crossing' else np.empty((0,6))
        if len(obs):obs[:,:2]-=.04*obs[:,2:4] # Controller helper declares a 40 ms observation age.
        command=ctrl.step(state,reference,obs);ctrl.plant(state,command)
        if hasattr(ctrl.lib,'online_avoidance_debug'):
            info=np.zeros(8);ctrl.lib.online_avoidance_debug(ctrl.handle,ptr(info));diagnostics.append([t,*info.tolist()])
        clearance=1e3
        for bx,by,r in body:
            px=state[0]+np.cos(state[2])*bx-np.sin(state[2])*by
            py=state[1]+np.sin(state[2])*bx+np.cos(state[2])*by
            clearance=min(clearance,2.-abs(py)-r)
            if obstacle is not None:
                # Evaluate obstacle at the next plant time as well.
                ox=obstacle[0]+.05*obstacle[2];oy=obstacle[1]+.05*obstacle[3]
                clearance=min(clearance,np.hypot(px-ox,py-oy)-r-obstacle[4])
        trace.append([t,*state,*command,clearance,*(obstacle[:2] if obstacle else [None,None])])
        if k%10==0 and hasattr(ctrl.lib,'online_local_route'):
            points=np.zeros((1000,2));n=ctrl.lib.online_local_route(ctrl.handle,ptr(points),len(points))
            routes.append({'t':t,'points':points[:n].tolist()})
        if np.linalg.norm(state[:2]-reference[-1,:2])<.05 and abs(state[2])<.1 and abs(state[3])<.05:
            reached=True;break
    ctrl.close();a=np.array([r[:9] for r in trace],dtype=float)
    return {'scene':scene,'build':str(build),'seed':seed,'crossing_speed':crossing_speed,'mirror':mirror,'crossing_offset':crossing_offset,'separate_dynamic':separate_dynamic,'reached':reached,'duration_s':len(trace)*.05,
            'min_clearance_m':float(a[:,8].min()),'reverse_distance_m':float(np.maximum(0,-a[:,4]).sum()*.05),
            'low_speed_s':float(np.sum(np.abs(a[:,4])<.02)*.05),'kernel_p95_ms':float(np.percentile(ctrl.ms,95)),
            'reference_replacements':0,'trace':trace,'routes':routes,'diagnostics':diagnostics,
            'kernel_ms':ctrl.ms,'body':body.tolist()}

def render(out,results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for ax,scene in zip(axes,('static','crossing')):
        ax.plot([0,3],[0,0],'--',c='.7',label='Unchanged reference')
        for r in results:
            if r['scene']!=scene:continue
            a=np.array([z[:9] for z in r['trace']]);label=Path(r['build']).name
            ax.plot(a[:,1],a[:,2],label=label+(' (goal)' if r['reached'] else ' (timeout)'))
        ax.add_patch(Circle((1.8,0) if scene=='static' else (1.65,0),.23,color='.8',alpha=.6))
        if scene=='crossing':
            ax.annotate('',xy=(1.65,.8),xytext=(1.65,-.8),arrowprops={'arrowstyle':'->','color':'.6','lw':1})
            ax.text(1.72,-1.10,'Obstacle moves upward',fontsize=8,color='.4')
        ax.set(title=scene,xlim=(-.4,3.4),ylim=(-1.3,1.3),xlabel='x / m',ylabel='y / m',aspect='equal');ax.legend(fontsize=8)
    fig.suptitle('Local-only test: fixed reference, ideal measurements, no native replanner')
    fig.tight_layout();fig.savefig(out/'local_paths.png',dpi=140);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--builds',nargs='+',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False);results=[]
    for scene in ('static','crossing'):
        for build in a.builds:
            r=trial(build,a.fixture,scene);results.append(r)
            print({k:v for k,v in r.items() if k not in ('trace','routes')},flush=True)
            (a.out/(scene+'_'+build.name+'.json')).write_text(json.dumps(r,indent=2)+'\n')
    render(a.out,results)

#!/usr/bin/env python3
"""C API controller regressions, synthetic references, no ROS publishers."""
import argparse
import ctypes as ct
import json
import math
from pathlib import Path
import time
import numpy as np

DP=ct.POINTER(ct.c_double)
def ptr(a):return a.ctypes.data_as(DP)

class Controller:
    def __init__(self,build,fixture):
        self.lib=ct.CDLL(str(build.resolve()/'build/libonline_mppi.so'))
        self.lib.online_create.argtypes=[DP,DP,ct.c_int,ct.c_int];self.lib.online_create.restype=ct.c_void_p
        self.lib.online_step.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,ct.c_int,DP]
        self.lib.online_feedback.argtypes=[ct.c_void_p,ct.c_double,ct.c_double]
        self.lib.online_reset.argtypes=[ct.c_void_p];self.lib.online_destroy.argtypes=[ct.c_void_p]
        self.c=json.loads((fixture/'limits.json').read_text());p=np.array(list(self.c.values()))
        body=np.ascontiguousarray(np.loadtxt(str(fixture/'geometry.csv'),delimiter=','))
        self.handle=self.lib.online_create(ptr(p),ptr(body),len(body),1)
        self.field=np.full((200,300),10.);self.obs=np.empty((0,6));self.ms=[]
    def step(self,s,ref,obs=None):
        obs=self.obs if obs is None else obs;out=np.zeros(8)
        code=self.lib.online_step(self.handle,ptr(s),ptr(ref),len(ref),ptr(obs),len(obs),.04,ptr(self.field),300,200,-7.5,-5.,.05,0,ptr(out))
        assert code==0 and np.isfinite(out).all();self.ms.append(out[7]);self.lib.online_feedback(self.handle,*out[:2]);return out[:2]
    def plant(self,s,u):
        c=self.c;u=u.copy();h=c['track']/2
        u/=max(1,abs(u[0])/c['vmax'],abs(u[1])/c['wmax'],abs(u[0]-h*u[1])/c['wheel_v'],abs(u[0]+h*u[1])/c['wheel_v'])
        for _ in range(10):
            a=(u-s[3:])/.12;a/=max(1,abs(a[0])/c['accel'],abs(a[1])/c['angular_accel'],abs(a[0]-h*a[1])/c['wheel_a'],abs(a[0]+h*a[1])/c['wheel_a'])
            new=s[3:]+.005*a;v,w=(s[3:]+new)/2;theta=s[2]+.0025*w;s[:2]+=.005*v*np.array([np.cos(theta),np.sin(theta)]);s[2]=math.atan2(math.sin(s[2]+.005*w),math.cos(s[2]+.005*w));s[3:]=new
        return s
    def close(self):self.lib.online_destroy(self.handle)

def run(build,fixture,out):
    ctrl=Controller(build,fixture);rows=[]
    for name in ('straight','curve','reverse'):
        t=np.linspace(0,1,241);x=1.5*t;y=np.zeros_like(t);yaw=np.zeros_like(t);speed=np.full_like(t,.18)
        if name=='curve':y=.3*(1-np.cos(np.pi*t));yaw=np.arctan2(.3*np.pi*np.sin(np.pi*t),1.5)
        if name=='reverse':x=-x;speed=-speed
        speed*=np.minimum(1,(1-t)*8);ref=np.ascontiguousarray(np.column_stack((x,y,yaw,speed)));s=np.zeros(5);trace=[]
        ctrl.lib.online_reset(ctrl.handle)
        for i in range(380):
            u=ctrl.step(s,ref);ctrl.plant(s,u);trace.append(s.copy())
            if np.linalg.norm(s[:2]-ref[-1,:2])<.05 and abs(s[2]-ref[-1,2])<.1 and abs(s[3])<.05:break
        a=np.array(trace);err=np.sqrt(((a[:,:2,None]-ref[:,:2].T[None,:,:])**2).sum(axis=1)).min(axis=1)
        reached=np.linalg.norm(s[:2]-ref[-1,:2])<.05 and abs(s[2]-ref[-1,2])<.1
        row=dict(scene=name,reached=bool(reached),mean_error_m=float(err.mean()),max_error_m=float(err.max()),duration_s=len(a)*.05,final_state=s.tolist())
        rows.append(row);print(row,flush=True)
    # A remote future uncertainty tube must not cause an immediate large retreat.
    t=np.linspace(0,3,401);ref=np.ascontiguousarray(np.column_stack((t,t*0,t*0,t*0+.18)))
    ctrl.lib.online_reset(ctrl.handle);s=np.array([.43,0.,0.,.2,0.]);obs=np.array([[1.65,-1.47,.035,.24,.255,1.]])
    u=ctrl.step(s,ref,obs);rows.append(dict(scene='distant_crossing',command=u.tolist(),no_large_reverse=bool(u[0]>-.02)))
    # A legitimate reverse target must continue to admit negative commands.
    summary={'tests':rows,'kernel_p95_ms':float(np.percentile(ctrl.ms,95)),'scope':'controller unit simulations, no native replanner, no ROS master'}
    ctrl.close();out.write_text(json.dumps(summary,indent=2)+'\n')
    assert all(r.get('reached',True) for r in rows),summary
    assert all(r.get('max_error_m',0)<.10 for r in rows),summary
    assert rows[-1]['no_large_reverse'],summary
    print('PASS',out,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.build,a.fixture,a.out)

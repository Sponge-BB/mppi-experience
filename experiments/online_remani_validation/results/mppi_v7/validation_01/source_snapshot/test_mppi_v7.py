#!/usr/bin/env python3
"""V7 decision regressions. No ROS master or physical publishers."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

def run(build,fixture,out):
    c=Controller(build,fixture);lib=c.lib
    lib.online_decision_debug.argtypes=[ct.c_void_p,DP]
    lib.online_avoidance_debug.argtypes=[ct.c_void_p,DP]
    lib.online_set_classified_static.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int]
    x=np.linspace(0,3,301);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,x*0+.18)))
    s=np.array([.3,0.,0.,.18,0.]);obs=np.array([[1.65,-1.6,0.,.26,.23,1.]])
    u=c.step(s,ref,obs);d=np.zeros(12);lib.online_decision_debug(c.handle,ptr(d))
    assert d[0]==0 and d[6]==0 and abs(d[11])<.03 and u[0]>.1,(u,d)
    checks=['distant crossing: monitor without lateral action']
    # Rejoin while the moving target remains observed at every step.
    lib.online_reset(c.handle);s=np.array([.8,.25,0.,.15,0.]);initial=abs(s[1]);trace=[]
    for k in range(160):
        obs=np.array([[1.65,2.8+.05*k*.26,0.,.26,.23,1.]])
        u=c.step(s,ref,obs);c.plant(s,u);lib.online_decision_debug(c.handle,ptr(d));trace.append([k*.05,*s.tolist(),d[0]])
        if abs(s[1])<.04 and abs(s[2])<.15:break
    assert abs(s[1])<.04 and abs(s[2])<.15,(s,trace[-1])
    checks.append('rejoin with nonempty persistent obstacle list')
    lib.online_reset(c.handle);reverse=ref.copy();reverse[:,0]*=-1;reverse[:,3]*=-1;s=np.zeros(5)
    u=c.step(s,reverse,np.array([[3.,3.,0.,.26,.23,1.]]));assert u[0]<-.05,u
    checks.append('legitimate reverse reference remains supported')
    # A rejected call must consume queued classified input as well, so a later
    # accepted call cannot reuse old membership and erase newly occupied cells.
    c.field[:]=0.;free=np.full_like(c.field,10.);result=np.zeros(8);s=np.array([.3,0.,0.,0.,0.])
    obs=np.array([[1.65,-1.6,0.,.26,.23,1.]])
    lib.online_set_classified_static(c.handle,ptr(free),300,200)
    code=lib.online_step(c.handle,ptr(s),ptr(ref),len(ref),ptr(obs),len(obs),.36,ptr(c.field),300,200,-7.5,-5.,.05,0,ptr(result));assert code!=0
    u=c.step(s,ref,obs);a=np.zeros(8);lib.online_avoidance_debug(c.handle,ptr(a));assert a[7]==1 and np.max(abs(u))==0
    checks.append('rejected input invalidates queued classified static layer')
    lib.online_reset(c.handle);lib.online_decision_debug(c.handle,ptr(d));assert np.max(abs(d))==0
    checks.append('reset clears decision and side-commitment state')
    c.close();out.write_text(json.dumps({'passed':True,'checks':checks,'rejoin_trace':trace,'scope':'unit tests, not deployment/safety certification'},indent=2)+'\n');print('PASS',out)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.build,a.fixture,a.out)

#!/usr/bin/env python3
"""V6 safety/diagnostic contract checks. No ROS or vehicle commands."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

def run(build,fixture,out):
    c=Controller(build,fixture);lib=c.lib
    lib.online_avoidance_debug.argtypes=[ct.c_void_p,DP]
    lib.online_status.argtypes=[ct.c_void_p,DP]
    x=np.linspace(0,3,301);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,x*0+.18)))
    s=np.array([.6,0.,0.,.20,0.]);obs=np.array([[1.65,-1.13,0.,.26,.23,1.]])
    u=c.step(s,ref,obs);d=np.zeros(8);lib.online_avoidance_debug(c.handle,ptr(d))
    assert d[1]==1,'predicted crossing must activate before present reference blockage'
    assert d[2]>0 and d[4]>=0,'selected trajectory must pass sampled clearance test'
    # Same scenario without an obstacle: no unconditional positive speed gate.
    lib.online_reset(c.handle);c.field[:]=0.;u=c.step(s,ref,obs)
    lib.online_avoidance_debug(c.handle,ptr(d));assert d[7]==1 and np.max(abs(u))==0
    result=np.zeros(8)
    code=lib.online_step(c.handle,ptr(s),ptr(ref),len(ref),ptr(obs),len(obs),.04,ptr(c.field),300,200,-7.5,-5.,.05,0,ptr(result))
    assert code==0 and result[6]==1,'unverified brake must be visible over C API'
    # Reset must clear encounter state and diagnostics, including failure flag.
    lib.online_reset(c.handle);lib.online_avoidance_debug(c.handle,ptr(d));assert np.max(abs(d))==0
    # Observation ages up to the input bound must not exceed CV's 2 s validity.
    c.field[:]=10.
    for age in (.0,.04,.20,.35):
        code=lib.online_step(c.handle,ptr(s),ptr(ref),len(ref),ptr(obs),len(obs),age,ptr(c.field),300,200,-7.5,-5.,.05,0,ptr(result))
        assert code==0 and np.isfinite(result).all(),(age,code)
    # Classified static is one-step only, preserves fixed obstacles, and must
    # share the union field's dimensions. Missing data reverts to union field.
    lib.online_set_classified_static.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int]
    free=np.full_like(c.field,10.);blocked=np.zeros_like(c.field)
    c.field[:]=0.
    lib.online_reset(c.handle);lib.online_set_classified_static(c.handle,ptr(free),300,200)
    c.step(s,ref,obs);lib.online_avoidance_debug(c.handle,ptr(d));assert d[2]>0
    c.step(s,ref,obs);lib.online_avoidance_debug(c.handle,ptr(d));assert d[7]==1,'stale classified layer reused'
    c.field[:]=10.
    lib.online_set_classified_static(c.handle,ptr(blocked),300,200)
    c.step(s,ref,obs);lib.online_avoidance_debug(c.handle,ptr(d));assert d[7]==1,'static occupancy removed'
    c.field[:]=0.
    lib.online_set_classified_static(c.handle,ptr(free),299,200)
    c.step(s,ref,obs);lib.online_avoidance_debug(c.handle,ptr(d));assert d[7]==1,'grid mismatch accepted'
    c.close();out.write_text(json.dumps({'passed':True,'checks':['future conflict activation','sampled feasible selection','all-infeasible brake is explicitly unverified','reset','CV validity at observation-age boundary','classified static preserves obstacles','classified static one-step lifetime','mismatched grid falls back to measured union'],'scope':'unit, not safety certificate'},indent=2)+'\n')
    print('PASS',out)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.build,a.fixture,a.out)

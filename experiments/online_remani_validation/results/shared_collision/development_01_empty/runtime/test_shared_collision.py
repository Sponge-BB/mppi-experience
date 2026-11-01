#!/usr/bin/env python3
"""Independent math/gradient, C ABI and latest-frame rejection regressions."""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from shared_collision_node import pack_snapshot
from test_mppi_v2 import Controller,DP,ptr


def oracle(field,body,obs,pose,forecast,config):
    x,y,yaw=pose;co,si=np.cos(yaw),np.sin(yaw);physical=1e6;cost=0.
    for bx,by,r in body:
        px=x+co*bx-si*by;py=y+si*bx+co*by
        fx=(px+7.5)/.05;fy=(py+5.)/.05;ix=int(np.floor(fx));iy=int(np.floor(fy))
        if not 0<=ix<299 or not 0<=iy<199:d=0.
        else:
            a=fx-ix;b=fy-iy
            d=(1-b)*((1-a)*field[iy,ix]+a*field[iy,ix+1])+b*((1-a)*field[iy+1,ix]+a*field[iy+1,ix+1])
        for ox,oy,vx,vy,rad,moving in obs:
            d=min(d,np.hypot(px-ox-forecast*vx*moving,py-oy-forecast*vy*moving)-rad)
        gap=d-r;physical=min(physical,gap);cost+=config['weight']*max(0.,config['comfort']-gap)**3
    return np.array([physical,physical-config['margin'],cost])


def test(build,fixture,out):
    out.mkdir(parents=True,exist_ok=False)
    ctrl=Controller(build,fixture);lib=ctrl.lib
    lib.online_set_collision_snapshot.argtypes=[ct.c_void_p,DP,ct.c_int];lib.online_set_collision_snapshot.restype=ct.c_int
    lib.online_common_pose.argtypes=[ct.c_double,ct.c_double,ct.c_double,ct.c_double,DP];lib.online_common_pose.restype=ct.c_int
    lib.online_reference_changed.argtypes=[ct.c_void_p]
    lib.online_selected_controls.argtypes=[ct.c_void_p,DP,ct.c_int];lib.online_selected_controls.restype=ct.c_int
    body=np.atleast_2d(np.loadtxt(fixture/'geometry.csv',delimiter=','))
    config=dict(margin=.1,comfort=.12,weight=50000.,max_age=.35,max_prediction=3.)
    xx=-7.5+np.arange(300)*.05;yy=-5+np.arange(200)*.05
    field=np.ascontiguousarray(np.maximum(0.,2.-np.abs(yy[:,None]))+np.zeros((200,300)))
    obs=np.array([[1.5,.08,-.18,.02,.23,1.]])
    payload=pack_snapshot(field,obs,body,1000.,1,config,True)
    assert lib.online_set_collision_snapshot(ctrl.handle,ptr(payload),len(payload))==0
    # Independently compiled native-side common implementation, not a Python clone.
    probe_source=out/'native_probe.cpp'
    # This is derived test source, not an edit of any production file.
    source='''#include "shared_collision.hpp"
extern "C" int bind_native(const double*p,int n){return shared_collision::bind(p,n)?0:1;}
extern "C" int pose_native(double x,double y,double yaw,double t,double*out){
 auto m=shared_collision::snapshot();if(!m)return 1;
 auto v=shared_collision::evaluate(*m,x,y,yaw,t);
 double a[]={v.physical,v.gap,v.cost,v.gx,v.gy,v.gyaw,m->sequence};
 std::copy(a,a+7,out);return v.valid?0:2;}
'''
    probe_source.write_text(source)
    subprocess.run(['g++','-std=c++17','-O2','-fPIC','-shared','-I'+str(build/'source'),str(probe_source),'-o',str(out/'native_probe.so')],check=True)
    native=ct.CDLL(str(out/'native_probe.so'));native.bind_native.argtypes=[DP,ct.c_int]
    native.pose_native.argtypes=lib.online_common_pose.argtypes
    assert native.bind_native(ptr(payload),len(payload))==0
    rng=np.random.default_rng(9);max_error=0.;max_gradient_error=0.
    for _ in range(120):
        pose=np.array([rng.uniform(.4,2.4),rng.uniform(-.9,.9),rng.uniform(-1.,1.)]);t=rng.uniform(0,3)
        a=np.zeros(7);b=np.zeros(7)
        assert lib.online_common_pose(*pose,t,ptr(a))==native.pose_native(*pose,t,ptr(b))==0
        np.testing.assert_array_equal(a,b)
        expected=oracle(field,body,obs,pose,t,config)
        np.testing.assert_allclose(a[:3],expected,rtol=1e-9,atol=1e-8)
        max_error=max(max_error,float(np.max(np.abs(a[:3]-expected))))
        gradient=[];epsilon=1e-6
        for k in range(3):
            plus=pose.copy();minus=pose.copy();plus[k]+=epsilon;minus[k]-=epsilon
            gradient.append((oracle(field,body,obs,plus,t,config)[2]-oracle(field,body,obs,minus,t,config)[2])/(2*epsilon))
        np.testing.assert_allclose(a[3:6],gradient,rtol=2e-5,atol=1e-3)
        max_gradient_error=max(max_gradient_error,float(np.max(np.abs(a[3:6]-gradient))))
    assert lib.online_common_pose(0.,0.,0.,3.01,ptr(np.zeros(7)))!=0
    for data in (payload[:-1],payload.copy()):
        if len(data)==len(payload):data[16+2]+=.01
        assert lib.online_set_collision_snapshot(ctrl.handle,ptr(data),len(data))!=0
    pbad=payload.copy();pbad[-1]=np.nan
    assert lib.online_set_collision_snapshot(ctrl.handle,ptr(pbad),len(pbad))!=0
    ctrl.field=np.full((200,300),10.);no=np.empty((0,6))
    empty=pack_snapshot(ctrl.field,no,body,1000.,2,config,True)
    assert lib.online_set_collision_snapshot(ctrl.handle,ptr(empty),len(empty))==0
    x=np.linspace(0.,3.,301);reference=np.ascontiguousarray(np.column_stack((x,x*0,x*0,np.minimum(.18,.8*(3-x)))))
    command=ctrl.step(np.array([.5,0.,0.,.18,0.]),reference,no)
    assert command[0]>.05 and abs(command[1])<.05,command
    count=lib.online_selected_controls(ctrl.handle,None,0);assert count>0
    controls=np.zeros((count,2));lib.online_selected_controls(ctrl.handle,ptr(controls),count)
    lib.online_reference_changed(ctrl.handle)
    assert lib.online_selected_controls(ctrl.handle,None,0)==0
    # A newly occupied authoritative snapshot vetoes the OLD chosen rollout.
    occupied=pack_snapshot(np.zeros_like(ctrl.field),no,body,1000.,3,config,True)
    assert lib.online_set_collision_snapshot(ctrl.handle,ptr(occupied),len(occupied))==0
    lib.online_validate_execution.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,ct.c_double,
        DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,DP,ct.c_int,DP]
    rows=np.zeros((650,8));approval=np.zeros(6);state=np.array([.5,0.,0.,.18,0.])
    assert lib.online_validate_execution(ctrl.handle,ptr(state),ptr(controls),len(controls),ptr(no),0,.04,0.,
        ptr(ctrl.field),300,200,-7.5,-5.,.05,ptr(rows),len(rows),ptr(approval))!=0
    ctrl.field.fill(0);command=ctrl.step(state,reference,no)
    assert np.max(np.abs(command))==0.,command
    ctrl.close()
    report=dict(passed=True,poses_checked=120,max_math_error=max_error,max_gradient_error=max_gradient_error,
        shared_header_sha256=hashlib.sha256((build/'source/shared_collision.hpp').read_bytes()).hexdigest(),
        checks=['identical native/MPPI shared kernel outputs','independent cubic-cost and analytic-gradient verification',
                'malformed/nonfinite/body-drift input rejected','CV horizon bound enforced','safe straight reference progresses',
                'new reference clears old control warm start','latest occupied frame vetoes historical chosen controls',
                'occupied full body rejects forced movement'],
        scope='math and C API contracts; native hook integration requires the separate online suite')
    (out/'tests.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();test(a.build.resolve(),a.fixture.resolve(),a.out.resolve())

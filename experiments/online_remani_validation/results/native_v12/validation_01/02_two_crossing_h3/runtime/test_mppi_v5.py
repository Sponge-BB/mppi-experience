#!/usr/bin/env python3
"""Geometry equivalence, local rejoin location and cleared-path exit; no ROS."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

def run(build,fixture,out):
    ctrl=Controller(build,fixture);lib=ctrl.lib
    lib.online_status.argtypes=[ct.c_void_p,DP]
    lib.online_pose_clearance.argtypes=[ct.c_void_p,ct.c_double,ct.c_double,ct.c_double];lib.online_pose_clearance.restype=ct.c_double
    x=np.linspace(0,5,501);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,x*0+.18)))
    s=np.array([.5,0.,0.,.15,0.]);xx,yy=np.meshgrid(-7.5+.05*np.arange(300),-5.+.05*np.arange(200))
    ctrl.field=np.ascontiguousarray(np.maximum(0.,.5-np.abs(yy)))
    ctrl.step(s,ref)
    forward=lib.online_pose_clearance(ctrl.handle,.5,0.,0.)
    sideways=lib.online_pose_clearance(ctrl.handle,.5,0.,np.pi/2)
    body=np.loadtxt(fixture/'geometry.csv',delimiter=',')
    expected=min(.5-abs(row[1])-row[2]-ctrl.c['margin'] for row in body)
    circle=.5-max(np.hypot(row[0],row[1])+row[2] for row in body)-ctrl.c['margin']
    assert abs(forward-expected)<1e-8 and forward>.015 and sideways<0 and circle<0
    lib.online_reset(ctrl.handle)
    ctrl.field=np.ascontiguousarray(np.maximum(0.,np.minimum(2.-np.abs(yy),np.hypot(xx-1.8,yy)-.23)-np.sqrt(2)*.05))
    ctrl.step(s,ref);status=np.zeros(8);lib.online_status(ctrl.handle,ptr(status))
    assert status[5]>0 and status[3]<3.4,status
    blocked=status.tolist()
    ctrl.field=np.ascontiguousarray(np.maximum(0.,2.-np.abs(yy)-np.sqrt(2)*.05))
    for _ in range(5):ctrl.step(s,ref)
    lib.online_status(ctrl.handle,ptr(status))
    assert status[0]==0 and status[5]==0,'cleared path retained unnecessary detour'
    result={'checks':'PASS','forward_clearance_m':forward,'sideways_clearance_m':sideways,
            'old_circle_clearance_m':circle,'margin_m':ctrl.c['margin'],'blocked_status':blocked,'clear_status':status.tolist(),
            'scope':'same distance field, disk-union geometry; no physical collision checks removed'}
    ctrl.close();out.write_text(json.dumps(result,indent=2)+'\n');print(result)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();run(a.build,a.fixture,a.out)

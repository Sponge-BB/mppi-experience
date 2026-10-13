#!/usr/bin/env python3
"""Independently reintegrate every seed-0 run in Python at 5 ms.

Reads CSV commands, not C++ collision flags or its step()/truth() routines.
Checks the actual state transitions and time-aligned minimum clearance.
This validates the numerical experiment, not the physical robot model.
"""
import csv
import json
import math
from pathlib import Path
import sys
import numpy as np


def obstacles(name,t):
    if name in ('empty','curve','reverse'):
        return []
    if name=='new_static':
        return [] if t<2 else [(1.55,0,.25)]
    if name=='late_intrusion':
        return [] if t<4.5 else [(1.48,0,.25)]
    if name=='close_clearance':
        return [] if t<4.5 else [(1.88,0,.25)]
    if name=='blocked':
        return [(1.65,.52*i,.29) for i in range(-3,4)]
    return [(1.45,-1.70+.28*t,.23)]


def advance(s,u,c,tau):
    x,y,yaw,v,w=s
    av=(u[0]-v)/tau;aw=(u[1]-w)/tau
    left=av-c['track']*.5*aw;right=av+c['track']*.5*aw
    scale=max(1,abs(av)/c['accel'],abs(aw)/c['angular_accel'],abs(left)/c['wheel_a'],abs(right)/c['wheel_a'])
    av/=scale;aw/=scale
    vn=v+.005*av;wn=w+.005*aw;vm=(v+vn)*.5;wm=(w+wn)*.5
    x+=.005*vm*math.cos(yaw+.0025*wm);y+=.005*vm*math.sin(yaw+.0025*wm)
    yaw=math.atan2(math.sin(yaw+.005*wm),math.cos(yaw+.005*wm))
    return (x,y,yaw,vn,wn)


def clearance(state,obs,body):
    x,y,yaw=state[:3];cy=math.cos(yaw);sy=math.sin(yaw)
    best=math.inf
    for bx,by,radius in body:
        px=x+cy*bx-sy*by;py=y+sy*bx+cy*by
        best=min(best,1.8-abs(py)-radius)
        for ox,oy,rad in obs:
            best=min(best,math.hypot(px-ox,py-oy)-radius-rad)
    return best


def audit(out):
    with (out/'runs.csv').open() as stream:
        rows=[r for r in csv.DictReader(stream) if r['seed']=='0']
    c=json.loads((out/'limits.json').read_text())
    body=json.loads((out/'geometry.json').read_text())['body_disks_xy_r']
    cycles=0;max_state_error=0;max_clear_error=0
    for row in rows:
        stem='__'.join(row[k] for k in ('scenario','profile','mode','seed'))
        data=np.atleast_1d(np.genfromtxt(out/(stem+'.csv'),delimiter=',',names=True))
        s=(0,0,0,0,0);tau=.28 if row['scenario']=='lag_mismatch' else .12
        for tick,d in enumerate(data):
            t=tick*.05;u=(d['command_v'],d['command_w'])
            minimum=clearance(s,obstacles(row['scenario'],t),body)
            for sub in range(1,11):
                s=advance(s,u,c,tau)
                minimum=min(minimum,clearance(s,obstacles(row['scenario'],t+sub*.05/10),body))
            actual=np.array([d[k] for k in ('x','y','yaw','v','w')])
            error=np.asarray(s)-actual
            error[2]=math.atan2(math.sin(error[2]),math.cos(error[2]))
            max_state_error=max(max_state_error,float(abs(error).max()))
            max_clear_error=max(max_clear_error,abs(minimum-d['clearance']))
            assert abs(error).max()<2e-7,(stem,t,'plant state',error)
            assert abs(minimum-d['clearance'])<2e-7,(stem,t,'truth clearance',minimum,d['clearance'])
            cycles+=1
    result={'runs':len(rows),'cycles':cycles,'physics_substeps':cycles*10,
            'max_state_error':max_state_error,'max_clearance_error':max_clear_error,
            'scope':'independent Python reintegration and truth clearance of all seed-0 cases; no robot model validation'}
    (out/'independent_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print('PASS independent 5 ms audit:',result,flush=True)


if __name__=='__main__':
    audit(Path(sys.argv[1]).resolve())

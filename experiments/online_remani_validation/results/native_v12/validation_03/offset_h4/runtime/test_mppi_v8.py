#!/usr/bin/env python3
"""Recovery regressions using recorded V7 failures and map constraints."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller, DP, ptr
from suite_multi_dynamic import clearance, obstacles, plant

HERE=Path(__file__).resolve().parent

def run(build,out):
    fixture=HERE/'results/final_report/empty_mppi';checks=[];examples=[]
    c=Controller(build,fixture);lib=c.lib
    lib.online_recovery_debug.argtypes=[ct.c_void_p,DP]
    lib.online_avoidance_debug.argtypes=[ct.c_void_p,DP]
    lib.online_set_classified_static.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int]
    x=np.linspace(0,3,301);ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,np.minimum(.18,.8*(3-x)))))
    xx,yy=np.meshgrid(-7.5+.05*np.arange(300),-5+.05*np.arange(200))
    free=np.ascontiguousarray(np.maximum(0.,2.-abs(yy)-np.sqrt(2)*.05))
    body=np.loadtxt(fixture/'geometry.csv',delimiter=',')
    for seed in (1,2,3):
        record=json.loads((HERE/f'results/mppi_v7/multi_dynamic_03/04_four_dense/data/seed{seed}.json').read_text())
        row=next(v for v in record['trace'] if v['unverified_brake']);s=np.array(row['state']);start=row['t']
        lib.online_reset(c.handle);min_gap=1e9;max_motion=0.;expanded=False;failed=0
        for k in range(38):
            t=start+k*.05;obs=obstacles('04_four_dense',t);measured=obs.copy();measured[:,:2]-=.04*measured[:,2:4]
            c.field=free;lib.online_set_classified_static(c.handle,ptr(free),300,200)
            u=c.step(s,ref,measured);d=np.zeros(10);lib.online_recovery_debug(c.handle,ptr(d));expanded=expanded or d[0]>0
            safe=np.zeros(8);lib.online_avoidance_debug(c.handle,ptr(safe));failed+=int(safe[7]>0)
            min_gap=min(min_gap,plant(c.c,s,u,'04_four_dense',t,body));max_motion=max(max_motion,float(np.max(abs(u))))
        examples.append({'seed':seed,'min_gap':min_gap,'max_command_magnitude':max_motion,'expanded':bool(expanded),'unverified_cycles':failed})
        assert expanded and max_motion>.05 and min_gap>=.1-1e-6 and failed==0,examples[-1]
    checks.append('recovery from all three saved V7 failure states: nonzero evasive action, 1.9s clearance >= margin (reverse is not mandatory)')
    lib.online_reset(c.handle);s=np.array([.5,0.,0.,0.,0.]);obs=np.array([[1.5,.2,-.18,0.,.2,1.]])
    c.field=np.zeros_like(free);u=c.step(s,ref,obs);d=np.zeros(10);lib.online_recovery_debug(c.handle,ptr(d))
    safe=np.zeros(8);lib.online_avoidance_debug(c.handle,ptr(safe))
    assert np.max(abs(u))==0 and safe[7]==1 and d[2]>=100 and d[3]==0,(u,safe,d)
    checks.append('fully occupied map: exploration still runs; no false feasible result or forced motion')
    lib.online_reset(c.handle);lib.online_recovery_debug(c.handle,ptr(d));assert np.max(abs(d))==0
    checks.append('reset clears recovery diagnostics and mode')
    c.close();out.write_text(json.dumps({'passed':True,'checks':checks,'examples':examples},indent=2)+'\n')
    print('PASS',out,examples)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.build,a.out)

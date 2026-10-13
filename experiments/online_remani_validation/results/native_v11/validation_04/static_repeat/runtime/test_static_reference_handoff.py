#!/usr/bin/env python3
"""Controlled held-reference test, NOT replay of historical sensing or ROS timing."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

HERE=Path(__file__).resolve().parent

def run(build,reference_case,out):
    fixture=HERE/'results/native_v10/static_runtime_06'
    c=Controller(build,fixture);lib=c.lib
    field=np.ascontiguousarray(np.load(fixture/'local_stall.npz')['field'])
    ref=np.ascontiguousarray(json.loads((reference_case/'consumed_references.jsonl').read_text().splitlines()[-1])['reference'])
    states=np.genfromtxt(reference_case/'state.csv',names=True,delimiter=',')
    initial=states[np.argmin(abs(states['t']-18.))]
    state=np.array([initial[k] for k in ('x','y','yaw','v','w')])
    lib.online_selected_controls.argtypes=[ct.c_void_p,DP,ct.c_int]
    lib.online_validate_execution.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,ct.c_double,
        DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,DP,ct.c_int,DP]
    trace=[];rejected=0;reached=False
    for i in range(700):
        result=np.zeros(8)
        code=lib.online_step(c.handle,ptr(state),ptr(ref),len(ref),ptr(c.obs),0,.08,
            ptr(field),300,200,-7.5,-5.,.05,0,ptr(result));assert code==0
        n=lib.online_selected_controls(c.handle,None,0);us=np.zeros((n,2))
        lib.online_selected_controls(c.handle,ptr(us),n)
        report=np.zeros(6);rows=np.zeros((650,8))
        code=lib.online_validate_execution(c.handle,ptr(state),ptr(us),n,ptr(c.obs),0,.08,0.,
            ptr(field),300,200,-7.5,-5.,.05,ptr(rows),len(rows),ptr(report))
        command=result[:2].copy()
        if code or result[6]:command[:]=0;rejected+=1
        lib.online_feedback(c.handle,*command);c.plant(state,command)
        trace.append([i*.05,*state,*command,code,result[7]])
        if np.linalg.norm(state[:2]-ref[-1,:2])<.06 and abs(state[3])<.05:
            reached=True;break
    c.close();out.mkdir(parents=True,exist_ok=False)
    np.savetxt(out/'state.csv',trace,delimiter=',',header='t,x,y,yaw,v,w,cmd_v,cmd_w,validation_code,kernel_ms',comments='')
    report={'build':str(build),'reached':reached,'elapsed_s':len(trace)*.05,'final_state':state.tolist(),
        'validation_rejections':rejected,'reference_source':str(reference_case),
        'scope':'same recorded native reference and measured initial state; held static field from V10 at same obstacle location; fresh controller seed 1; no online replanning/ROS/compute-delay model; NOT exact historical replay'}
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--reference-case',type=Path,default=HERE/'results/native_v11/validation_03/static')
    args=p.parse_args();run(args.build,args.reference_case,args.out)

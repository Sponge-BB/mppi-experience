#!/usr/bin/env python3
"""Recorded static-stall regression and execution candidate ABI contracts."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

HERE=Path(__file__).resolve().parent


def checks(build):
    fixture=HERE/'results/native_v10/static_runtime_06'
    snap=dict(np.load(fixture/'local_stall.npz'))
    c=Controller(build,fixture)
    lib=c.lib
    lib.online_selected_controls.argtypes=[ct.c_void_p,DP,ct.c_int];lib.online_selected_controls.restype=ct.c_int
    lib.online_reference_changed.argtypes=[ct.c_void_p]
    lib.online_execution_candidates.argtypes=[ct.c_void_p,DP,ct.c_int,ct.c_int,DP]
    lib.online_execution_candidates.restype=ct.c_int
    lib.online_accept_execution_controls.argtypes=[ct.c_void_p,DP,ct.c_int]
    lib.online_validate_execution.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP,ct.c_int,ct.c_double,ct.c_double,
        DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,DP,ct.c_int,DP]
    def step(field,age=.08,obs=None):
        targets=np.empty((0,6)) if obs is None else obs;out=np.zeros(8)
        code=lib.online_step(c.handle,ptr(snap['state']),ptr(snap['reference']),len(snap['reference']),
            ptr(targets),len(targets),age,ptr(field),300,200,-7.5,-5.,.05,0,ptr(out))
        assert code==0
        return out
    def selected():
        n=lib.online_selected_controls(c.handle,None,0);us=np.zeros((n,2))
        assert lib.online_selected_controls(c.handle,ptr(us),n)==n
        return us
    def validate(us,field):
        rows=np.zeros((650,8));report=np.zeros(6);obs=np.empty((0,6))
        code=lib.online_validate_execution(c.handle,ptr(snap['state']),ptr(us),len(us),ptr(obs),0,.08,0,
            ptr(field),300,200,-7.5,-5,.05,ptr(rows),len(rows),ptr(report))
        return code,report
    result=[]
    # Induce recovery through the real planner path; do not reach into private
    # memory or just test a freshly reset controller at the historical stall.
    occupied=np.zeros_like(snap['field']);failed=step(occupied)
    assert failed[6]==1 and np.max(np.abs(failed[:2]))==0
    out=step(snap['field']);us=selected()
    assert len(us)==30 and out[0]>.02,(out,us.shape)
    code,report=validate(us,snap['field']);assert code==0,(code,report)
    result.append({'test':'recorded stall escapes induced recovery with complete checked static detour',
        'command':out[:2].tolist(),'horizon_s':len(us)*.1,'validation':report.tolist()})
    # Reference changes invalidate policy state, not the actual control seed.
    lib.online_reference_changed(c.handle);np.testing.assert_array_equal(selected(),us)
    result.append({'test':'reference transition retains control warm start'})
    # Static horizon must not shrink with the age of a frame containing NO CV targets.
    for age in (.04,.15,.30):
        out=step(snap['field'],age);assert len(selected())==30
    result.append({'test':'static horizon independent of unused CV age'})
    lib.online_reset(c.handle)
    open_field=np.full_like(snap['field'],10.)
    crossing=np.ascontiguousarray([[1.45,-.7,0.,.26,.23,1.]])
    step(open_field,.15,crossing)
    pool=np.zeros((8,64,2));lengths=np.zeros(8)
    n=lib.online_execution_candidates(c.handle,ptr(pool),8,64,ptr(lengths));assert 2<=n<=8
    np.testing.assert_array_equal(pool[0,:int(lengths[0])],selected())
    assert np.all(np.isfinite(pool)) and all(1<=k<=64 for k in lengths[:n])
    alternative=np.ascontiguousarray(pool[1,:int(lengths[1])])
    assert lib.online_accept_execution_controls(c.handle,ptr(alternative),len(alternative))==0
    np.testing.assert_array_equal(selected(),alternative)
    bad=np.ascontiguousarray([[10.,0.]])
    assert lib.online_accept_execution_controls(c.handle,ptr(bad),1)!=0
    result.append({'test':'bounded candidate export and exact accepted-sequence warm start','candidates':n})
    for i in range(n):
        candidate=np.ascontiguousarray(pool[i,:int(lengths[i])])
        code,_=validate(candidate,occupied);assert code!=0
    result.append({'test':'new full occupancy vetoes EVERY exported alternative'})
    # Dynamic prediction is still bound by its original 2s validity.
    targets=np.ascontiguousarray([[4.,3.,0.,.1,.23,1.]])
    step(snap['field'],.15,targets);assert len(selected())*.1+.15<2.
    result.append({'test':'dynamic rollout remains within 2s CV validity'})
    c.close();return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();rows=checks(args.build)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    with args.out.open('x') as stream:json.dump({'passed':True,'checks':rows},stream,indent=2)
    print(json.dumps(rows,indent=2))

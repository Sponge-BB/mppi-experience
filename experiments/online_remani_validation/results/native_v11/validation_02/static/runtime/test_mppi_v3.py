#!/usr/bin/env python3
"""No ROS: rejoin from either side and explicit no-uncertainty contract."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from test_mppi_v2 import Controller, DP, ptr

def run(build,fixture,out):
    ctrl=Controller(build,fixture)
    ctrl.lib.online_status.argtypes=[ct.c_void_p,DP]
    x=np.linspace(0,3,401)
    ref=np.ascontiguousarray(np.column_stack((x,x*0,x*0,x*0+.18)))
    rows=[]
    status=np.zeros(8)
    blocked_state=np.array([0.,0.,0.,.18,0.])
    obstacle=np.array([[.9,0.,0.,0.,.2,1.]])
    for _ in range(5):
        ctrl.step(blocked_state,ref,obstacle)
        ctrl.lib.online_status(ctrl.handle,ptr(status))
        assert int(status[0])==1 and int(status[1])==1,'blocked path did not retain avoid side'
    clear_state=np.array([0.,.2,0.,.18,0.])
    for cycle in range(4):
        ctrl.step(clear_state,ref)
        ctrl.lib.online_status(ctrl.handle,ptr(status))
        assert int(status[0])==(1 if cycle<3 else 2),'clear hysteresis failed'
    for offset in (-.20,.20):
        ctrl.lib.online_reset(ctrl.handle)
        state=np.array([.2,offset,0.,.15,0.]);phases=[];trace=[]
        for _ in range(160):
            command=ctrl.step(state,ref);ctrl.plant(state,command)
            status=np.zeros(8);ctrl.lib.online_status(ctrl.handle,ptr(status))
            phases.append(int(status[0]));trace.append(state.copy().tolist())
        row={'initial_offset_m':offset,'final_offset_m':float(state[1]),
             'phases':sorted(set(phases)),'minimum_velocity':float(min(s[3] for s in trace)),
             'trace':trace}
        rows.append(row)
        assert 2 in phases and abs(state[1])<.03,row
        assert row['minimum_velocity']>-.02,row
    # Nonzero uncertainty must be rejected instead of silently mixing profiles.
    cfg=dict(ctrl.c);cfg['pos_unc']=.08
    p=np.array(list(cfg.values()));body=np.ascontiguousarray(np.loadtxt(fixture/'geometry.csv',delimiter=','))
    bad=ctrl.lib.online_create(ptr(p),ptr(body),len(body),1)
    if bad:ctrl.lib.online_destroy(bad)
    assert not bad,'v3 accepted nonzero uncertainty'
    ctrl.close()
    out.write_text(json.dumps({'checks':'PASS','rejoin':rows,'avoid_clear_hysteresis':True,'uncertainty_rejection':True},indent=2)+'\n')
    print('PASS',out)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--fixture',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.build,a.fixture,a.out)

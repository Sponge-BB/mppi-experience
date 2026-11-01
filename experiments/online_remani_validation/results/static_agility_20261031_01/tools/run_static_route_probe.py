#!/usr/bin/env python3
"""Bounded offline route-construction diagnosis, never publishes ROS commands."""
import ctypes as ct
import argparse
import json
import os
from pathlib import Path
import numpy as np

TOOLS=Path(__file__).resolve().parent
TRIAL=TOOLS.parent/'static_early_repeat01'
os.environ['NATIVE_COLLISION_3D']='1'
os.environ['MPPI_STATIC_EARLY_DETOUR']='1'
DP=ct.POINTER(ct.c_double)
def ptr(a):return a.ctypes.data_as(DP)
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--suffix',default='');parser.add_argument('--library',type=Path,default=TOOLS/'static_route_probe.so');parser.add_argument('--crop-x-max',type=float);args=parser.parse_args()
    if args.suffix and not args.suffix.replace('_','').isalnum():raise ValueError('Invalid suffix')
    data=np.load(TRIAL/'native3d_event_input.npz')
    meta=json.loads((TRIAL/'native3d_event_input.json').read_text())
    lib=ct.CDLL(str(args.library))
    lib.online_create.argtypes=[DP,DP,ct.c_int,ct.c_int];lib.online_create.restype=ct.c_void_p
    lib.online_destroy.argtypes=[ct.c_void_p]
    lib.online_set_native_collision_snapshot.argtypes=[ct.c_void_p,DP,ct.c_int]
    lib.online_set_native_collision_clock.argtypes=[ct.c_void_p,ct.c_double]
    lib.probe_route.argtypes=[ct.c_void_p,DP,DP,ct.c_int,ct.c_char_p]
    lib.online_native_collision_query.argtypes=[ct.c_void_p,ct.c_double,ct.c_double,ct.c_double,ct.c_double,ct.c_int,DP]
    settings=np.array(list(json.loads((TRIAL/'limits.json').read_text()).values()),dtype=float)
    body=np.ascontiguousarray(np.loadtxt(TRIAL/'geometry.csv',delimiter=','))
    handle=lib.online_create(ptr(settings),ptr(body),len(body),11)
    assert handle
    payload=np.ascontiguousarray(data['payload'])
    crop_note='original saved crop'
    if args.crop_x_max is not None:
        count_float=(args.crop_x_max-payload[8])/payload[11];count=int(round(count_float))
        if abs(count-count_float)>1e-8 or not 2<=count<=int(payload[5]):raise ValueError('Crop must retain a whole supported x slab')
        prefix=32+7*int(payload[12])+7*int(payload[13])
        count_values=count*int(payload[6])*int(payload[7])
        payload=np.ascontiguousarray(payload[:prefix+count_values].copy());payload[5]=count
        crop_note='Synthetic narrower x crop of the same saved raw ESDF; no distance samples or geometry changed'
    assert lib.online_set_native_collision_snapshot(handle,ptr(payload),len(payload))==0
    assert lib.online_set_native_collision_clock(handle,meta['solve_ros_time'])==0
    first=json.loads((TRIAL/'consumed_references.jsonl').open().readline())
    oldref=np.ascontiguousarray(first['reference'],dtype=float)
    states=np.genfromtxt(TRIAL/'state.csv',delimiter=',',names=True)
    rows=[]
    for t in [15.225848017,16.,17.050098532]:
        at=states[np.argmin(abs(states['t']-t))]
        state=np.array([at[k] for k in ['x','y','yaw','v','w']],dtype=float)
        output=TOOLS/('route_probe_%.3f%s.json'%(t,args.suffix))
        assert lib.probe_route(handle,ptr(state),ptr(oldref),len(oldref),str(output).encode())==0
        r=json.loads(output.read_text());r.update(requested_time=t,nearest_state_time=float(at['t']),
            scope='Earlier measured state and original reference with later frozen STATIC ESDF; geometry diagnostic, not exact earlier decision replay',
            snapshot_time=meta['solve_ros_time'],source=str(TRIAL/'native3d_event_input.npz'),probe_library=str(args.library),crop_scope=crop_note)
        r['crop_edge_min']=payload[8:11].tolist()
        r['crop_edge_max']=(payload[8:11]+payload[5:8]*payload[11]).tolist()
        if r['route']:
            route=np.asarray(r['route']);closest=int(np.argmin(np.linalg.norm(route[:,:2]-state[:2],axis=1)))
            pairs=[(-1,state[:2],route[closest,:2])]+[(i,route[i,:2],route[i+1,:2]) for i in range(closest,len(route)-1)]
            for i,a,b in pairs:
                distance=float(np.linalg.norm(b-a));yaw=float(state[2] if distance<1e-5 else np.arctan2(*(b-a)[::-1]))
                count=max(1,int(np.ceil(distance/.04)));failed=False
                for u in np.linspace(0.,1.,count+1):
                    point=a+u*(b-a);value=np.zeros(5)
                    code=lib.online_native_collision_query(handle,*point,yaw,meta['solve_ros_time'],0,ptr(value))
                    if value[0]<.015:
                        r['first_full_suffix_rejection']=dict(index=i,a=a.tolist(),b=b.tolist(),sample=point.tolist(),yaw=yaw,gap=float(value[0]),valid=bool(value[4]),code=code)
                        failed=True;break
                if failed:break
        output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
        row={k:v for k,v in r.items() if k not in ['route','make_segments']}
        row['segment_calls']=len(r['make_segments']);row['last_segments']=r['make_segments'][-4:]
        rows.append(row)
    lib.online_destroy(handle)
    (TOOLS/('route_probe_summary%s.json'%args.suffix)).write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(rows,ensure_ascii=False,indent=2))
if __name__=='__main__':main()

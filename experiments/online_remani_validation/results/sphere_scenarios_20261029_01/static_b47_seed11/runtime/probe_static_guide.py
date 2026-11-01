#!/usr/bin/env python3
"""Inspect real kernel guide construction on an exact saved stall snapshot."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import numpy as np
from test_mppi_v2 import Controller,DP,ptr

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--case',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args();build=args.build.resolve();out=args.out.resolve();(out/'build').mkdir(parents=True,exist_ok=False)
    manifest=json.loads((build/'build_manifest.json').read_text());parent=Path(manifest['parent_build'])
    cm=parent/'build/CMakeFiles/online_mppi.dir';flags=(cm/'flags.make').read_text()
    includes=next(s.split('=',1)[1].strip() for s in flags.splitlines() if s.startswith('CXX_INCLUDES ='))
    obj=out/'build/probe.o'
    subprocess.run(['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',*shlex.split(includes),
        '-DONLINE_API_SOURCE="'+str(build/'source/online_api.cpp')+'"','-c',str(Path(__file__).with_suffix('.cpp')),'-o',str(obj)],check=True)
    link=shlex.split((cm/'link.txt').read_text());old=next(s for s in link if s.endswith('online_api.cpp.o'))
    link=[str(obj) if s==old else s for s in link];link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=parent/'build',check=True);shutil.copy2(build/'build_manifest.json',out/'build_manifest.json')
    c=Controller(out,args.case);snap=dict(np.load(args.case/'local_stall.npz'));result=np.zeros(8)
    code=c.lib.online_step(c.handle,ptr(snap['state']),ptr(snap['reference']),len(snap['reference']),ptr(c.obs),0,
        float(snap['age']),ptr(snap['field']),300,200,-7.5,-5.,.05,0,ptr(result));assert code==0
    c.lib.online_probe_static_guide.argtypes=[ct.c_void_p,DP,DP,ct.c_int,DP]
    data=np.zeros(6+10*len(snap['reference']))
    n=c.lib.online_probe_static_guide(c.handle,ptr(snap['state']),ptr(snap['reference']),len(snap['reference']),ptr(data))
    report={'state':snap['state'].tolist(),'fresh_controller_result':result.tolist(),
        'current_guide_nodes_usable_join_arc_gear_failed':data[:6].tolist(),
        'join_columns':['index','arc','x','y','goal_gap','rounded_gap','end_connector_clear','route_built','route_usable','route_nodes'],
        'joins':data[6:6+n*10].reshape(-1,10).tolist(),
        'scope':'exact saved state/reference/field; fresh controller history; diagnostic only, no command publication'}
    (out/'probe.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));c.close()

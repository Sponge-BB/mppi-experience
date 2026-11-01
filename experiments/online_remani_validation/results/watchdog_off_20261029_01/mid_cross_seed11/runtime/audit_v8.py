#!/usr/bin/env python3
"""Independent integrity/constraints audit. Safety outcomes are NOT hidden."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
from suite_v8 import sha,HERE

def audit(out):
    m=json.loads((out/'manifest.json').read_text());limits=json.loads((out/'fixture/limits.json').read_text())
    for v,b in m['builds'].items():assert sha(Path(b['path'])/'build/libonline_mppi.so')==b['sha256']
    for name,h in m['source_snapshot_sha256'].items():assert sha(out/'source_snapshot'/name)==h
    for name,h in m['baseline_source_files'].items():assert sha(Path(name))==h
    native=m['builds']['v8']['manifest']['production_sources_sha256']
    for name,h in native.items():assert sha(HERE.parents[1]/name)==h
    count=0;failure=[];violations=[];v8_stats=[]
    for case,actors in m['cases'].items():
        for version in ('v7','v8'):
            for seed in m['seeds']:
                r=json.loads((out/case/'data'/f'{version}_seed{seed}.json').read_text());count+=1
                rows=r['trace'];s=np.array([v['state'] for v in rows]);ns=np.array([v['next_state'] for v in rows]);u=np.array([v['command'] for v in rows]);times=np.array([v['t'] for v in rows])
                assert np.isfinite(s).all() and np.isfinite(ns).all() and np.isfinite(u).all()
                assert np.allclose(s[1:],ns[:-1],atol=1e-12)
                assert np.allclose(times,np.arange(len(rows))*.05)
                h=limits['track']/2
                for x in (u,ns[:,3:]):
                    assert np.max(abs(x[:,0]))<=limits['vmax']+1e-9
                    assert np.max(abs(x[:,1]))<=limits['wmax']+1e-9
                    assert np.max(abs(x[:,0])+h*abs(x[:,1]))<=limits['wheel_v']+1e-9
                acc=(ns[:,3:]-s[:,3:])/.05
                assert np.max(abs(acc[:,0]))<=limits['accel']+1e-8
                assert np.max(abs(acc[:,1]))<=limits['angular_accel']+1e-8
                assert np.max(abs(acc[:,0])+h*abs(acc[:,1]))<=limits['wheel_a']+1e-8
                for row in rows:
                    expected=np.array(actors,dtype=float) if row['t']>=2 else np.empty((0,6))
                    expected[:,:2]+=(row['t']-2)*expected[:,2:4]
                    assert np.allclose(np.array(row['obstacles']).reshape(-1,6),expected)
                for snap in r['searches']:
                    selected=[v for v in snap['candidates'] if v['selected']]
                    assert len(selected)<=1 and all(v['feasible'] for v in selected)
                    for c in snap['candidates']:
                        a=np.array(c['points']);assert np.isfinite(a).all()
                        assert a[-1,0]+.04<=2+1e-8
                        assert c['feasible']==(c['gap']>=0)
                if not r['metrics']['reached']:failure.append({'case':case,'version':version,'seed':seed,'termination':r['metrics']['termination']})
                if r['metrics']['margin_violation_cycles']:violations.append({'case':case,'version':version,'seed':seed,'min_gap':r['metrics']['min_gap_m']})
                if version=='v8':v8_stats.append(r['metrics'])
    gifs=list(out.glob('*/gif/*.gif'))
    for path in gifs:
        with Image.open(path) as im:assert im.n_frames>1;im.seek(im.n_frames-1);im.load()
    result={'integrity_passed':True,'trials':count,'production_files_unchanged':len(native),'readable_gifs':len(gifs),
            'failures':failure,'margin_violations':violations,
            'v8_reached':sum(r['reached'] for r in v8_stats),'v8_trials':len(v8_stats),
            'v8_unverified_brake_cycles':sum(r['unverified_brake_cycles'] for r in v8_stats),
            'v8_kernel_over_50ms':sum(r['kernel_over_50ms'] for r in v8_stats),
            'scope':'sampled numerical audit, not continuous safety or real-time hardware certification'}
    (out/'independent_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();audit(a.out)

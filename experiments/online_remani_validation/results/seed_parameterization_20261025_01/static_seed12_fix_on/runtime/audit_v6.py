#!/usr/bin/env python3
"""Audit recorded local trials; passing this is not deployment certification."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from PIL import Image

def main():
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--versions',nargs=2,default=['v5','v6']);a=p.parse_args()
    manifest=json.loads((a.out/'manifest.json').read_text());workspace=Path(__file__).resolve().parents[2]
    for version in a.versions:
        build=manifest['builds'][version]
        digest=hashlib.sha256((Path(build['path'])/'build/libonline_mppi.so').read_bytes()).hexdigest()
        assert digest==build['sha256']==build['manifest']['mppi_library_sha256'],version
    native=manifest['builds'][a.versions[1]]['manifest']['production_sources_sha256']
    for name,digest in native.items():assert hashlib.sha256((workspace/name).read_bytes()).hexdigest()==digest,name
    for name,digest in manifest['source_snapshot_sha256'].items():assert hashlib.sha256((a.out/'source_snapshot'/name).read_bytes()).hexdigest()==digest,name
    c=json.loads((a.out/'fixture'/'limits.json').read_text());notes=[];gifs=0
    for case in sorted(a.out.iterdir()):
        if not (case/'data').is_dir():continue
        for f in sorted((case/'data').glob('*.json')):
            r=json.loads(f.read_text());x=np.array([row[:9] for row in r['trace']]);assert np.isfinite(x).all()
            if a.versions[1]=='v7':
                assert r['separate_dynamic'],'V6/V7 must receive the same classification assumption'
                if r['scene']=='crossing':
                    assert r['persistent_obstacle']
                    present=[row[9] is not None for row in r['trace']]
                    if any(present):assert all(present[present.index(True):]),('script removed obstacle',f)
            assert np.allclose(np.diff(x[:,0]),.05)
            wheels=np.column_stack((x[:,4]-.5*c['track']*x[:,5],x[:,4]+.5*c['track']*x[:,5]))
            accel=np.diff(wheels,axis=0)/.05
            assert abs(wheels).max()<=c['wheel_v']+1e-8
            assert abs(accel).max()<=c['wheel_a']+1e-8
            assert x[:,8].min()>0,'physical overlap: '+str(f)
            if f.name.startswith(a.versions[1]+'_'):
                assert x[:,8].min()>=c['margin']-1e-3,('fixed margin violation',f)
                for d in r['diagnostics']:
                    if d[2] and not d[8]:assert d[5]>=0,('selected infeasible rollout',f,d)
            notes.append({'trial':str(f.relative_to(a.out)),'reached':r['reached'],'min_clearance_m':float(x[:,8].min()),'max_wheel_speed':float(abs(wheels).max()),'max_wheel_accel':float(abs(accel).max())})
        for name in (a.versions[0]+'.gif',a.versions[1]+'.gif','comparison.gif'):
            with Image.open(case/'gif'/name) as im:
                assert im.n_frames>10;im.seek(im.n_frames-1);im.load();gifs+=1
    result={'integrity_checks':'PASS','production_files_unchanged':len(native),'trials':notes,'gif_files_checked':gifs,
            'not_proven':['real sensor membership','full REMANI execution permission','continuous-time safety','statistical generalization','wall-clock whole-node 20 Hz']}
    (a.out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print('PASS',len(notes),'trials,',gifs,'GIFs,',len(native),'production files unchanged')

if __name__=='__main__':main()

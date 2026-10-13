#!/usr/bin/env python3
"""Index a declared two-method comparison without modifying recorded runs."""
import argparse
import hashlib
import json
from pathlib import Path

def select(baseline,mppi,out):
    out=out.resolve();out.mkdir(parents=True,exist_ok=False);records=[];libraries=set()
    for scene in ('empty','static','crossing'):
        a=(baseline/(scene+'_remani')).resolve();b=(mppi/(scene+'_mppi')).resolve()
        ma=json.loads((a/'manifest.json').read_text());mb=json.loads((b/'manifest.json').read_text())
        assert ma['native_binaries_sha256']==mb['native_binaries_sha256'],'native planner/controller/detector mismatch'
        assert ma['scenario_start']==mb['scenario_start'],'scenario trigger differs'
        for name in ('parameters.yaml','limits.json','geometry.csv'):
            assert (a/name).read_bytes()==(b/name).read_bytes(),name+' differs'
        libraries.add(mb['mppi_library_sha256'])
        for mode,source in (('remani',a),('mppi',b)):
            assert (source/'result.json').exists()
            (out/(scene+'_'+mode)).symlink_to(source,target_is_directory=True)
            records.append({'scene':scene,'mode':mode,'source':str(source),'manifest_sha256':hashlib.sha256((source/'manifest.json').read_bytes()).hexdigest()})
    assert len(libraries)==1,'MPPI revisions mixed'
    (out/'selection.json').write_text(json.dumps({'trials':records,'baseline_reused':True,'rationale':'Reuse all three predeclared Tracker runs with identical native binary hashes, parameters, geometry and scenario triggers. Every MPPI run comes from one revision. Earlier failures remain in their original directories.','scope':'single trials, native ROS numerical SIL, not Gazebo/physical robot','mppi_library_sha256':list(libraries)},indent=2)+'\n')
    print(out)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True);p.add_argument('--mppi',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();select(a.baseline,a.mppi,a.out)

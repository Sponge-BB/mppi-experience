#!/usr/bin/env python3
"""Paired no-dynamic regression: unchanged empty/static behavior."""
import argparse
import json
from pathlib import Path
import numpy as np
from test_local_independence import trial

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--new',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    root=Path(__file__).resolve().parent;a.out.mkdir(parents=True,exist_ok=False);checks=[]
    for scene in ('empty','static'):
        results=[]
        for version,build in (('v7',root/'builds/mppi_v7_search_01'),('v8',a.new)):
            r=trial(build,root/'results/final_report/empty_mppi',scene,seed=1,separate_dynamic=True,persistent_obstacle=True)
            (a.out/f'{scene}_{version}.json').write_text(json.dumps(r,separators=(',',':'))+'\n');results.append(r)
        before=np.array([v[:9] for v in results[0]['trace']]);after=np.array([v[:9] for v in results[1]['trace']])
        same=before.shape==after.shape and np.array_equal(before,after)
        assert same and all(r['reached'] for r in results),(scene,same)
        checks.append({'scene':scene,'identical_states_commands_gaps':same,'reached':True,'duration_s':results[1]['duration_s']})
        print('PASS',checks[-1],flush=True)
    (a.out/'summary.json').write_text(json.dumps(checks,indent=2)+'\n')

#!/usr/bin/env python3
"""Development trials: immutable output, no ROS or real robot."""
import argparse
import json
from pathlib import Path
from suite_multi_dynamic import trial

if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--cases',nargs='+',default=['04_four_dense','01_single'])
    p.add_argument('--seeds',nargs='+',type=int,default=[1]);a=p.parse_args()
    fixture=Path(__file__).resolve().parent/'results/final_report/empty_mppi'
    a.out.mkdir(parents=True,exist_ok=False)
    for case in a.cases:
        for seed in a.seeds:
            r=trial(a.build,fixture,case,seed,capture=True)
            (a.out/f'{case}_seed{seed}.json').write_text(json.dumps(r,separators=(',',':'))+'\n')
            print(case,seed,r['metrics'],flush=True)

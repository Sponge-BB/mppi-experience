#!/usr/bin/env python3
"""Counterfactual: reset old MPPI ONCE after one second of active standstill.

No new phase/projection/merit, no new compiled algorithm, no ROS. This is a
diagnostic cache-reset intervention, not a proposed watchdog deployment.
"""
import argparse
from pathlib import Path
import shutil

import numpy as np

from run_mppi_cusp_adjustment import CuspKernel, SOURCES, BUILDS, SCOPE, verify_build
from run_mppi_rejoin_diagnosis import HERE, trial, write_json, sha


class ResetKernel(CuspKernel):
    def __init__(self,*args):
        super().__init__(*args);self.zero_cycles=0;self.reset_done=False

    def solve_validate(self,state,reference,*args):
        event=False
        if self.zero_cycles>=20 and not self.reset_done:
            self.lib.online_reset(self.handle)
            self.lib.online_reference_changed(self.handle)
            self.reset_done=True;event=True
        answer=super().solve_validate(state,reference,*args)
        unfinished=np.linalg.norm(state[:2]-reference[-1,:2])>.05
        self.zero_cycles=self.zero_cycles+1 if answer['active_zero'] and unfinished and abs(state[3])<.01 else 0
        answer['one_time_reset']=event
        return answer


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(__file__,a.out/'runner_snapshot.py')
    build=BUILDS['baseline'];verified=verify_build(build)
    write_json(a.out/'protocol.json',dict(scope=SCOPE,variant='reset_only',build=verified,
        change='One reset after 20 consecutive unfinished active-zero cycles and actual |v|<.01; no phase/merit/parameter change',
        library_unchanged=True,script_sha256=sha(__file__)))
    results=[]
    for source,path in SOURCES.items():
        result=trial(build,path,a.out/source/'seed_01/reset_only/dynamic_crossing',
            'dynamic_crossing',seed=1,kernel_type=ResetKernel,
            experiment_context=dict(scope=SCOPE,variant='reset_only',**{k:v for k,v in verified.items() if k!='variant'}))
        results.append(dict(result,source=source,variant='reset_only'))
        print('RESET_CONTROL',source,'passed',result['passed'],'duration',result['simulated_duration_s'],flush=True)
    write_json(a.out/'summary.json',dict(cases=results))


if __name__=='__main__':main()

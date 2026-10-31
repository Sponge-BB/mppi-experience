#!/usr/bin/env python3
"""Paired cusp ablation: same native reference/plant/observations, no ROS."""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil

import numpy as np

from run_mppi_rejoin_diagnosis import (
    HERE, DP, Kernel, CRITERIA, DEFAULT_CASES, trial, write_json, sha, ptr)

SOURCES={name:HERE/'results/resource_contention/validation_02'/('static_remani_'+number)
         for name,number in (('source_01','01'),('source_02','02'))}
BUILDS=dict(baseline=HERE/'builds/native_v12_07',
            phase=HERE/'builds/cusp_phase_01',
            phase_objective=HERE/'builds/cusp_objective_01',
            gated_phase=HERE/'builds/cusp_gated_01')
SCOPE=('Recomputed paired fixed-native-reference numerical control experiment. '
       'Ideal synchronous CV observations, exact same full body/plant/constraints/validator; '
       'no ROS, no online native replan, no Gazebo, no real-vehicle publication. '
       'Actual 50ms controller decision deadline enforced; rendering excluded. '
       'Phase association/objective are experimental changes, not a production fix.')


class CuspKernel(Kernel):
    def __init__(self,*args):
        super().__init__(*args)
        self.getter=getattr(self.lib,'online_cusp_debug',None)
        if self.getter is not None:self.getter.argtypes=[ct.c_void_p,DP]
        self.bound=getattr(self.lib,'online_cusp_bound',None)
        if self.bound is not None:
            self.bound.argtypes=[ct.c_void_p];self.bound.restype=ct.c_int
        self.gate_gap=getattr(self.lib,'online_cusp_gate_gap',None)
        if self.gate_gap is not None:
            self.gate_gap.argtypes=[ct.c_void_p];self.gate_gap.restype=ct.c_double

    def solve_validate(self,*args):
        answer=super().solve_validate(*args)
        if self.getter is not None:
            phase=np.zeros(14);self.getter(self.handle,ptr(phase))
            answer['cusp_debug']=phase.tolist()
        if self.bound is not None:answer['cusp_bound']=bool(self.bound(self.handle))
        if self.gate_gap is not None:answer['cusp_estimated_gap_m']=float(self.gate_gap(self.handle))
        return answer


def verify_build(build):
    manifest=json.loads((build/'build_manifest.json').read_text())
    library=sha(build/'build/libonline_mppi.so')
    if library!=manifest['mppi_library_sha256']:raise ValueError('MPPI manifest mismatch')
    for name,digest in manifest['isolated_source_sha256'].items():
        if sha(build/'source'/name)!=digest:raise ValueError('changed build input '+name)
    return dict(path=str(build),library_sha256=library,manifest_sha256=sha(build/'build_manifest.json'),
                variant=manifest.get('cusp_variant','baseline'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--variants',nargs='+',choices=BUILDS,default=['baseline','gated_phase'])
    p.add_argument('--sources',nargs='+',choices=SOURCES,default=list(SOURCES))
    p.add_argument('--cases',nargs='+',choices=DEFAULT_CASES,default=list(DEFAULT_CASES))
    p.add_argument('--seeds',nargs='+',type=int,default=[1])
    p.add_argument('--duration',type=float,default=55.)
    p.add_argument('--search-stride',type=int,default=5)
    a=p.parse_args()
    if not 0<a.duration<=120:p.error('duration must be (0,120]')
    builds={v:verify_build(BUILDS[v]) for v in a.variants}
    a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(__file__,a.out/'runner_snapshot.py')
    shutil.copy2(HERE/'run_mppi_rejoin_diagnosis.py',a.out/'fixture_runner_snapshot.py')
    protocol=dict(scope=SCOPE,criteria=CRITERIA,builds=builds,sources={k:str(SOURCES[k]) for k in a.sources},
        variants=a.variants,cases=a.cases,seeds=a.seeds,script_sha256=sha(__file__),
        runner_sha256=sha(HERE/'run_mppi_rejoin_diagnosis.py'),
        preserved='No changes to physical geometry, margin, speed/wheel/acceleration limits, CV model or selected/brake validator.',
        caveat='Few deterministic fixtures, not a statistical real-world navigation success rate.')
    write_json(a.out/'protocol.json',protocol)
    results=[]
    for source in a.sources:
        for seed in a.seeds:
            for case in a.cases:
                for variant in a.variants:
                    out=a.out/source/('seed_%02d'%seed)/variant/case
                    report=trial(BUILDS[variant],SOURCES[source],out,case,seed,a.duration,a.search_stride,
                        kernel_type=CuspKernel,experiment_context=dict(scope=SCOPE,**builds[variant]))
                    results.append(dict(report,source=source,variant=variant,seed=seed))
                    write_json(a.out/'summary.json',dict(scope=SCOPE,completed=len(results),cases=results))
                    print('CUSP_RESULT',source,seed,variant,case,'pass',report['passed'],
                          'duration',report['simulated_duration_s'],'gap',report['min_physical_gap_m'],
                          'deadline',report['deadline_misses'],flush=True)
    for variant in a.variants:
        if verify_build(BUILDS[variant])!=builds[variant]:raise ValueError('build changed during experiment')
    print('CUSP_SUITE_COMPLETE',len(results),flush=True)


if __name__=='__main__':main()

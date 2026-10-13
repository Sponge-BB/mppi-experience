#!/usr/bin/env python3
"""Immutable V12 experiment: early measured motion + sampled side-pass MPPI."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE,sha,replace
from build_native_v10 import compile_source
from perception_v12 import transform_core,transform_node,profile_overrides,CHANGES


def find_snapshot(base,name):
    pending=[base];seen=set()
    while pending:
        p=pending.pop(0).resolve()
        if p in seen:continue
        seen.add(p)
        if (p/'source'/name).exists():return p/'source'/name
        manifest=p/'build_manifest.json'
        if manifest.exists():
            m=json.loads(manifest.read_text())
            pending.extend(Path(m[k]) for k in ('execution_parent','ablation_parent','parent_build') if k in m)
    raise FileNotFoundError(name)


def compile_target(parent,out,target,sources):
    cm=parent/'build/CMakeFiles'/(target+'.dir');link=shlex.split((cm/'link.txt').read_text())
    for source in sources:
        matches=[s for s in link if s.endswith('/'+source.name+'.o')]
        if len(matches)!=1:raise RuntimeError('link object ambiguous: '+source.name)
        old,obj=compile_source(parent,out,target,source,matches[0],out/'source/include')
        link=[obj if s==old else s for s in link]
    binary='libonline_mppi.so' if target=='online_mppi' else target
    link[link.index('-o')+1]=str(out/'build'/binary)
    subprocess.run(link,cwd=str(parent/'build'),check=True)


def build(base,out,perception=True,controller=True,perception_build=None):
    base=base.resolve();out=out.resolve();m=json.loads((base/'build_manifest.json').read_text())
    assert m['local_execution_owner'] and sha(base/'build/libonline_mppi.so')==m['mppi_library_sha256']
    out.mkdir(parents=True,exist_ok=False);shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    parent=Path(m['parent_build'])
    if controller:
        kernel=(base/'source/online_kernel.hpp').read_text()
        kernel=replace(kernel,'struct MPPI : V7MPPI {','struct LegacyMPPI : V7MPPI {')
        kernel=replace(kernel,'  MPPI(const C&config,int seed):V7MPPI(config,seed){}',
                       '  LegacyMPPI(const C&config,int seed):V7MPPI(config,seed){}')
        # Keep every inherited proposal inside the requested CV validity.
        marker='std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt)))'
        kernel=replace(kernel,marker,marker.replace('std::min(19,','std::min(40,'),3)
        kernel=replace(kernel,'} // namespace experiment','#include "mppi_v12.hpp"\n\n} // namespace experiment')
        (out/'source/online_kernel.hpp').write_text(kernel)
        shutil.copy2(HERE/'mppi_v12.hpp',out/'source/mppi_v12.hpp')
        api=(base/'source/online_api.cpp').read_text()
        api=replace(api,'m.committed_side=0;m.clear_count=0;}',
                    'm.committed_side=0;m.clear_count=0;m.referenceChanged();}')
        api+='\nextern "C" void online_dynamic_strategy_debug(void*p,double*out){if(p){auto&m=static_cast<Online*>(p)->m;double values[]={double(m.d_using_new),double(m.d_active),double(m.d_lateral_needed),double(m.d_had_dynamic)};std::copy(values,values+4,out);}}\n'
        (out/'source/online_api.cpp').write_text(api)
        compile_target(parent,out,'online_mppi',[out/'source/online_api.cpp'])
    else:shutil.copy2(base/'build/libonline_mppi.so',out/'build/libonline_mppi.so')
    if perception:
        node=find_snapshot(base,'dynamic_obstacle_tracker_node.cpp')
        core=parent/'source/mm_remani/src/dynamic_obstacle_tracker_core.cpp'
        header=parent/'source/mm_remani/include/mm_remani/dynamic_obstacle_tracker_core.h'
        (out/'source/include/mm_remani').mkdir(parents=True)
        shutil.copy2(header,out/'source/include/mm_remani'/header.name)
        (out/'source/dynamic_obstacle_tracker_node.cpp').write_text(transform_node(node.read_text()))
        (out/'source/dynamic_obstacle_tracker_core.cpp').write_text(transform_core(core.read_text()))
        if perception_build is not None:
            perception_build=perception_build.resolve()
            for name in ('dynamic_obstacle_tracker_node.cpp','dynamic_obstacle_tracker_core.cpp',
                         'include/mm_remani/dynamic_obstacle_tracker_core.h'):
                assert sha(out/'source'/name)==sha(perception_build/'source'/name),'detector reuse requires identical sources'
            shutil.copy2(perception_build/'build/dynamic_obstacle_tracker_node',out/'build/dynamic_obstacle_tracker_node')
        else:
            compile_target(parent,out,'dynamic_obstacle_tracker_node',
                           [out/'source/dynamic_obstacle_tracker_node.cpp',out/'source/dynamic_obstacle_tracker_core.cpp'])
    m.update(controller_revision='native_mppi_v12_side_pass' if controller else 'native_mppi_v12_perception_only',
             v12_controller=controller,v12_perception=perception,execution_parent=str(base),
             experiment_parameter_overrides=profile_overrides() if perception else {},
             default_prediction_s=3. if controller else 2.,
             mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
             controller_changes=CHANGES if perception else [])
    if controller:m['controller_changes']+=['dynamic horizon caps support2/3/4s bounded by observation age',
        'once-cycle CV cache and bounded sampled obstacle-aware passing proposals',
        'feasible same-physical-side exponential MPPI update with full mean recheck',
        'sustained lateral escape objective during predicted conflicts; checked nominal rejoin afterward',
        'separate frontal closing side-pass from crossing time-yield; no reward for chasing transverse obstacle motion',
        'rejoin orientation scores the attainable native connection; local endpoint signed correction remains a checked proposal',
        'endpoint merit measures final-pose distance reduction instead of wrong-branch arc progress at forward/reverse cusps',
        'original static policy, wheel/acceleration/geometry/margin and publication/brake checks retained']
    m['isolated_source_sha256']={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()}
    m['builder_sha256']=sha(Path(__file__))
    m['native_binaries_sha256']={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')}
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/native_v11_08')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--perception-only',action='store_true')
    p.add_argument('--controller-only',action='store_true')
    p.add_argument('--perception-build',type=Path,help='reuse detector only after identical node/core/header hashes')
    a=p.parse_args()
    if a.perception_only and a.controller_only:p.error('choose at most one ablation')
    build(a.base,a.out,not a.controller_only,not a.perception_only,a.perception_build)

#!/usr/bin/env python3
"""Immutable SIL build: consistent static horizon and reviewed alternatives."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE,sha,replace
from build_native_v10 import compile_source


def build(base,out):
    base=base.resolve();out=out.resolve()
    manifest=json.loads((base/'build_manifest.json').read_text())
    assert manifest.get('local_execution_owner')
    assert sha(base/'build/libonline_mppi.so')==manifest['mppi_library_sha256']
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    kernel=(base/'source/online_kernel.hpp').read_text()
    old='int horizon=std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));'
    kernel=replace(kernel,old,'int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));',2)
    # A complete, progressing static detour may end recovery even while direct
    # reference following remains blocked. Preserve the existing dynamic policy.
    kernel=replace(kernel,'    if(!failed&&!recovering){',
        '    if(!failed&&(!recovering||(z.os.empty()&&avoidance_debug[5]>.02))){')
    kernel=replace(kernel,'if(base.merit.feasible&&base.continuations>0&&terminalOpen(base.merit))return original;',
        'if(base.merit.feasible&&base.continuations>0&&terminalOpen(base.merit)){if(z.os.empty()){recovering=false;recovery_clear=0;}return original;}')
    (out/'source/online_kernel.hpp').write_text(kernel)
    api=(base/'source/online_api.cpp').read_text()
    api=replace(api,'return h;}catch(...){return nullptr;}',
        'h->m.trace_enabled=true;return h;}catch(...){return nullptr;}')
    api=replace(api,'m.route.path.clear();m.published_route.clear();}',
        'm.route.path.clear();m.published_route.clear();m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;}')
    api+='\n'+(HERE/'execution_candidates.inc').read_text()
    (out/'source/online_api.cpp').write_text(api)
    parent=Path(manifest['parent_build']);cm=parent/'build/CMakeFiles/online_mppi.dir'
    link=shlex.split((cm/'link.txt').read_text())
    oldobj=next(s for s in link if s.endswith('online_api.cpp.o'))
    oldobj,obj=compile_source(parent,out,'online_mppi',out/'source/online_api.cpp',oldobj)
    link=[obj if s==oldobj else s for s in link]
    link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    manifest.update(controller_revision='native_mppi_v11_execution_candidates',execution_candidates=True,
        execution_parent=str(base),mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        controller_changes=['static proposals and recovery retain the same 3s horizon; CV horizon unchanged',
            'complete feasible static detour can leave recovery without waiting for blocked nominal guide',
            'reference change invalidates recovery/side state while retaining control warm start',
            'ranked feasible candidate export; accepted alternative becomes the next warm start'],
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(out,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=HERE/'builds/native_v10_06')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();build(args.base,args.out)

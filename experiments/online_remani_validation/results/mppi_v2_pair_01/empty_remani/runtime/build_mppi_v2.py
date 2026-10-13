#!/usr/bin/env python3
"""New controller library, immutable native snapshot; never overwrite v1."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

HERE=Path(__file__).resolve().parent

def build(base,out):
    base=base.resolve();out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'source').mkdir();(out/'build').mkdir()
    manifest=json.loads((base/'build_manifest.json').read_text())
    for name in ('remani_planner_node','remani_planner_node_no_wait','mm_controller_node','dynamic_obstacle_tracker_node'):
        if (base/'build'/name).exists():shutil.copy2(base/'build'/name,out/'build'/name)
    old=(base/'source/online_kernel.hpp').read_text();start=old.index('struct MPPI{');end=old.index('} // namespace experiment',start)
    variant=(HERE/'mppi_v2.hpp').read_text();(out/'source/online_kernel.hpp').write_text(old[:start]+variant+'\n'+old[end:])
    api=(HERE/'online_api.cpp').read_text().replace('if(!ptr||nr<2||age<-.01||age>.35)return 1;','if(!ptr||nr<2||age<-.01||age>.35||cbf)return 1;')
    api+='\nextern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}\n'
    api+='extern "C" void online_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.debug.begin(),static_cast<Online*>(p)->m.debug.end(),out);}\n'
    source=out/'source/online_api.cpp';source.write_text(api)
    flags=(base/'build/CMakeFiles/online_mppi.dir/flags.make').read_text()
    includes=next(line.split('=',1)[1].strip() for line in flags.splitlines() if line.startswith('CXX_INCLUDES ='))
    obj=out/'build/online_api.cpp.o'
    subprocess.run(['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',*shlex.split(includes),'-c',str(source),'-o',str(obj)],check=True)
    link=shlex.split((base/'build/CMakeFiles/online_mppi.dir/link.txt').read_text());link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    link=[str(obj) if v=='CMakeFiles/online_mppi.dir/online_api.cpp.o' else v for v in link]
    subprocess.run(link,cwd=str(base/'build'),check=True)
    manifest.update(controller_revision='path_relative_mppi_v2',parent_build=str(base),cbf_enabled=False,
      controller_changes=['polyline distance and path-relative progress','feedback rollout proposals','soft future uncertainty / unchanged present margin','reference-relative reverse penalty','all terminal proposals evaluated by same cost'],
      v2_source_sha256=hashlib.sha256(variant.encode()).hexdigest(),mppi_library_sha256=hashlib.sha256((out/'build/libonline_mppi.so').read_bytes()).hexdigest())
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(out,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)

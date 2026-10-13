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

def build(base,out,revision='v2'):
    base=base.resolve();out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'source').mkdir();(out/'build').mkdir()
    manifest=json.loads((base/'build_manifest.json').read_text())
    for name in ('remani_planner_node','remani_planner_node_no_wait','mm_controller_node','dynamic_obstacle_tracker_node'):
        if (base/'build'/name).exists():shutil.copy2(base/'build'/name,out/'build'/name)
    old=(base/'source/online_kernel.hpp').read_text();start=old.index('struct MPPI{');end=old.index('} // namespace experiment',start)
    variant=(HERE/'mppi_v2.hpp').read_text()
    if revision in ('v3','v4'):
        # With all uncertainty coefficients zero, the two predicted geometries
        # are identical. Reuse the exact clearance; do not change cost weights.
        marker='double risk=clearance(s,uncertain[k],c,sc.ymax,c.margin);'
        assert variant.count(marker)==1
        variant=variant.replace(marker,'double risk=d; // v3: identical nominal/uncertain geometry')
        variant=variant.replace('struct MPPI {','struct BaseMPPI {',1).replace('  MPPI(const C& cc,int seed)','  BaseMPPI(const C& cc,int seed)',1)
        variant=variant.replace('  Score cost(', '  virtual Score cost(',1)
        marker='  V plan(const S& s,const Z& z,const Scene& sc){'
        assert variant.count(marker)==1
        variant=variant.replace(marker,'  virtual void adjustTargets(const S&,const Z&,const Scene&,const std::vector<double>&,std::vector<V>&)const {}\n'+marker)
        marker='    std::vector<std::vector<O>> nominal,uncertain;double age=std::max(0.,z.t-z.stamp);'
        assert variant.count(marker)==1
        variant=variant.replace(marker,'    adjustTargets(s,z,sc,arc,targets);\n'+marker)
        if revision=='v4':
            marker='const int horizon=z.os.empty()?30:c.horizon;'
            assert variant.count(marker)==1
            variant=variant.replace(marker,'const int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));')
            marker='        else u=(j%2?g:.5*g+.5*warm[k])+noise;'
            assert variant.count(marker)==1
            variant=variant.replace(marker,'        else if(j==11||j==12)u=k<(j==11?4:8)?V{0,0}:g;\n'+marker)
        variant+='\n'+(HERE/('mppi_'+revision+'.hpp')).read_text()
    (out/'source/online_kernel.hpp').write_text(old[:start]+variant+'\n'+old[end:])
    api=(HERE/'online_api.cpp').read_text().replace('if(!ptr||nr<2||age<-.01||age>.35)return 1;','if(!ptr||nr<2||age<-.01||age>.35||cbf)return 1;')
    api+='\nextern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}\n'
    api+='extern "C" void online_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.debug.begin(),static_cast<Online*>(p)->m.debug.end(),out);}\n'
    if revision in ('v3','v4'):
        api+='extern "C" void online_status(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.status.begin(),static_cast<Online*>(p)->m.status.end(),out);}\n'
    if revision=='v4':
        api+='extern "C" int online_local_route(void*p,double*out,int capacity){if(!p)return 0;auto&r=static_cast<Online*>(p)->m.route.path;int n=std::min(capacity,int(r.size()));for(int i=0;i<n;++i){out[2*i]=r[i].p.x;out[2*i+1]=r[i].p.y;}return n;}\n'
    source=out/'source/online_api.cpp';source.write_text(api)
    flags=(base/'build/CMakeFiles/online_mppi.dir/flags.make').read_text()
    includes=next(line.split('=',1)[1].strip() for line in flags.splitlines() if line.startswith('CXX_INCLUDES ='))
    obj=out/'build/online_api.cpp.o'
    subprocess.run(['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',*shlex.split(includes),'-c',str(source),'-o',str(obj)],check=True)
    link=shlex.split((base/'build/CMakeFiles/online_mppi.dir/link.txt').read_text());link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    link=[str(obj) if v=='CMakeFiles/online_mppi.dir/online_api.cpp.o' else v for v in link]
    subprocess.run(link,cwd=str(base/'build'),check=True)
    manifest.update(controller_revision='path_relative_mppi_'+revision,parent_build=str(base),cbf_enabled=False,
      experiment_profile='no_uncertainty' if revision in ('v3','v4') else 'original',
      controller_changes=['polyline distance and path-relative progress','feedback rollout proposals',
        'zero uncertainty in both native planner and MPPI; retain fixed margin' if revision in ('v3','v4') else 'soft future uncertainty / unchanged present margin',
        'reference-relative reverse penalty','all terminal proposals evaluated by same cost']+
        (['track/avoid/rejoin states','CV-guided bounded lateral proposals with side hysteresis','feasibility-conditioned rejoin attraction'] if revision=='v3' else []),
      v2_source_sha256=hashlib.sha256(variant.encode()).hexdigest(),mppi_library_sha256=hashlib.sha256((out/'build/libonline_mppi.so').read_bytes()).hexdigest())
    if revision=='v4':manifest['controller_changes'] += ['cached local grid-search detour with original-path rejoin','same detour interface for static and dynamic measured occupancy','time-shifted stop-then-go proposals','dynamic rollout horizon bounded by remaining CV validity','native stop/abort unchanged; shadow computation only after abort']
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(out,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--revision',choices=['v2','v3','v4'],default='v2')
    a=p.parse_args();build(a.base,a.out,a.revision)

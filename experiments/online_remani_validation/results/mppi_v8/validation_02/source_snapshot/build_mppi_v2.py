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
    if revision in ('v3','v4','v5','v6','v7','v8'):
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
        if revision in ('v4','v5','v6','v7','v8'):
            marker='const int horizon=z.os.empty()?30:c.horizon;'
            assert variant.count(marker)==1
            variant=variant.replace(marker,'const int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));')
            marker='        else u=(j%2?g:.5*g+.5*warm[k])+noise;'
            assert variant.count(marker)==1
            variant=variant.replace(marker,'        else if(j==11||j==12)u=k<(j==11?4:8)?V{0,0}:g;\n'+marker)
        if revision in ('v5','v6','v7','v8'):
            variant=variant.replace('  const C& c;', '  int proposal_divisor=1;\n  const C& c;',1)
            variant=variant.replace('c.samples*c.horizon/horizon','c.samples*c.horizon/horizon/proposal_divisor',1)
            guide=(HERE/'mppi_v4.hpp').read_text().replace('struct MPPI : BaseMPPI','struct LocalGuideMPPI : BaseMPPI',1).replace('  MPPI(const C& config,int seed)','  LocalGuideMPPI(const C& config,int seed)',1)
            guide=guide.replace('  double freeDistance(', '  virtual double freeDistance(',1).replace('  bool segment(', '  virtual bool segment(',1)
            guide_begin=guide.index('    for(size_t k=0;k<sc.path.size();++k)')
            guide_end=guide.index('    V goal=sc.path[join].p;',guide_begin)
            guide=guide[:guide_begin]+'    join=chooseJoin(s,sc);\n'+guide[guide_end:]
            guide=guide.replace('  bool makeRoute(', '  virtual size_t chooseJoin(const S&,const Scene& sc)const{return sc.path.size()-1;}\n  bool makeRoute(',1)
            guide=guide.replace('.08/std::max(.04,clearance_grid[v])','.02/std::max(.04,clearance_grid[v])',1)
            v5=(HERE/'mppi_v5.hpp').read_text()
            if revision in ('v6','v7','v8'):
                v5=v5.replace('struct MPPI : LocalGuideMPPI','struct V5MPPI : LocalGuideMPPI',1).replace('  MPPI(const C& config,int seed)','  V5MPPI(const C& config,int seed)',1)
            variant+='\n'+guide+'\n'+v5
            if revision in ('v6','v7'):variant+='\n'+(HERE/('mppi_'+revision+'.hpp')).read_text()
            if revision=='v8':
                v7=(HERE/'mppi_v7.hpp').read_text().replace('struct MPPI : V5MPPI','struct V7MPPI : V5MPPI',1).replace('  MPPI(const C& config,int seed)','  V7MPPI(const C& config,int seed)',1)
                variant+='\n'+v7+'\n'+(HERE/'mppi_v8.hpp').read_text()
        else:variant+='\n'+(HERE/('mppi_'+revision+'.hpp')).read_text()
    (out/'source/online_kernel.hpp').write_text(old[:start]+variant+'\n'+old[end:])
    api=(HERE/'online_api.cpp').read_text().replace('if(!ptr||nr<2||age<-.01||age>.35)return 1;','if(!ptr||nr<2||age<-.01||age>.35||cbf)return 1;')
    api+='\nextern "C" void online_reset(void*p){if(p)static_cast<Online*>(p)->m.reset();}\n'
    api+='extern "C" void online_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.debug.begin(),static_cast<Online*>(p)->m.debug.end(),out);}\n'
    if revision in ('v3','v4','v5','v6','v7','v8'):
        api+='extern "C" void online_status(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.status.begin(),static_cast<Online*>(p)->m.status.end(),out);}\n'
    if revision in ('v4','v5','v6','v7','v8'):
        api+='extern "C" int online_local_route(void*p,double*out,int capacity){if(!p)return 0;auto&r=static_cast<Online*>(p)->m.route.path;int n=std::min(capacity,int(r.size()));for(int i=0;i<n;++i){out[2*i]=r[i].p.x;out[2*i+1]=r[i].p.y;}return n;}\n'
    if revision in ('v5','v6','v7','v8'):
        api=api.replace('m.route.path;int n=', 'm.published_route;int n=')
        api+='extern "C" int online_revision(){return '+revision[1:]+';}\n'
        api+='extern "C" double online_pose_clearance(void*p,double x,double y,double yaw){return static_cast<Online*>(p)->m.poseDistance({x,y},yaw);}\n'
    if revision in ('v6','v7','v8'):
        api=api.replace('out[6]=f.unverified;', 'out[6]=h.m.avoidance_debug[7];')
        api=api.replace('struct Online {C c;', 'struct Online {const double* classified_static=nullptr;int static_width=0,static_height=0;C c;')
        api=api.replace('grid=field;nx=width;', 'grid=(h.classified_static&&h.static_width==width&&h.static_height==height&&no>0)?h.classified_static:field;nx=width;',1)
        api=api.replace('map_distance=distance;', 'map_distance=distance;h.classified_static=nullptr; // Classified layer is one-step only; never silently reuse stale membership.',1)
        api+='extern "C" void online_set_classified_static(void*p,const double*field,int width,int height){if(p){auto&h=*static_cast<Online*>(p);h.classified_static=field;h.static_width=width;h.static_height=height;}}\n'
        api+='extern "C" void online_avoidance_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.avoidance_debug.begin(),static_cast<Online*>(p)->m.avoidance_debug.end(),out);}\n'
    if revision in ('v7','v8'):
        api+='extern "C" void online_decision_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.decision_debug.begin(),static_cast<Online*>(p)->m.decision_debug.end(),out);}\n'
        api+='extern "C" void online_search_enable(void*p,int enabled){if(p){auto&m=static_cast<Online*>(p)->m;m.trace_enabled=enabled!=0;m.search_trace.clear();}}\n'
        api+='''extern "C" int online_search_trace(void*p,double*out,int capacity){
          if(!p)return 0;auto&h=*static_cast<Online*>(p);auto&m=h.m;int n=0;
          if(!out){for(const auto&r:m.search_trace)n+=int(r.controls.size())+1;return n;}
          for(size_t id=0;id<m.search_trace.size();++id){const auto&r=m.search_trace[id];auto s=m.trace_start;
            for(size_t k=0;k<=r.controls.size();++k){
              if(out&&n<capacity){double row[]={double(id),k*h.c.rdt,s.x,s.y,r.cost,r.gap,double(r.feasible),double(r.selected)};std::copy(row,row+8,out+8*n);}++n;
              if(k<r.controls.size()){s=experiment::step(s,r.controls[k],h.c.rdt/2,h.c);s=experiment::step(s,r.controls[k],h.c.rdt/2,h.c);}
            }
          }return n;}
'''
        api=api.replace('if(!ptr||nr<2||age<-.01||age>.35||cbf)return 1;', 'if(!ptr||nr<2||age<-.01||age>.35||cbf){if(ptr)static_cast<Online*>(ptr)->classified_static=nullptr;return 1;}')
        api=api.replace('static_cast<Online*>(ptr)->classified_static=nullptr;', '{auto&h=*static_cast<Online*>(ptr);h.classified_static=nullptr;h.m.search_trace.clear();}')
    if revision=='v8':
        api+='extern "C" void online_recovery_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.recovery_debug.begin(),static_cast<Online*>(p)->m.recovery_debug.end(),out);}\n'
        api=api.replace('h.m.search_trace.clear();}', 'h.m.search_trace.clear();h.m.recovery_debug={};}')
    source=out/'source/online_api.cpp';source.write_text(api)
    flags=(base/'build/CMakeFiles/online_mppi.dir/flags.make').read_text()
    includes=next(line.split('=',1)[1].strip() for line in flags.splitlines() if line.startswith('CXX_INCLUDES ='))
    obj=out/'build/online_api.cpp.o'
    subprocess.run(['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',*shlex.split(includes),'-c',str(source),'-o',str(obj)],check=True)
    link=shlex.split((base/'build/CMakeFiles/online_mppi.dir/link.txt').read_text());link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    link=[str(obj) if v=='CMakeFiles/online_mppi.dir/online_api.cpp.o' else v for v in link]
    subprocess.run(link,cwd=str(base/'build'),check=True)
    manifest.update(controller_revision='path_relative_mppi_'+revision,parent_build=str(base),cbf_enabled=False,
      experiment_profile='no_uncertainty' if revision in ('v3','v4','v5','v6','v7','v8') else 'original',
      controller_changes=['polyline distance and path-relative progress','feedback rollout proposals',
        'zero uncertainty in both native planner and MPPI; retain fixed margin' if revision in ('v3','v4','v5','v6','v7','v8') else 'soft future uncertainty / unchanged present margin',
        'reference-relative reverse penalty','all terminal proposals evaluated by same cost']+
        (['track/avoid/rejoin states','CV-guided bounded lateral proposals with side hysteresis','feasibility-conditioned rejoin attraction'] if revision=='v3' else []),
      v2_source_sha256=hashlib.sha256(variant.encode()).hexdigest(),mppi_library_sha256=hashlib.sha256((out/'build/libonline_mppi.so').read_bytes()).hexdigest())
    if revision=='v4':manifest['controller_changes'] += ['cached local grid-search detour with original-path rejoin','same detour interface for static and dynamic measured occupancy','time-shifted stop-then-go proposals','dynamic rollout horizon bounded by remaining CV validity','native stop/abort unchanged; shadow computation only after abort']
    if revision=='v5':manifest['controller_changes'] += ['orientation-dependent original body disks; removed circumscribed guide circle','earliest clear rejoin after blockage','clear-path shortcut/exit with hysteresis','common rollout score for original/wait and detour proposals; shared sample budget','native stop and abort unchanged']
    if revision=='v6':manifest['controller_changes'] += ['inherits v5 static avoidance','future nominal-rollout conflict activates continuous forward arc proposals','common progress/stall objective; feasibility-first candidate selection and rechecked MPPI mean','unchanged body/margin/CV validity; no forced minimum speed; zero fallback is not a safety certificate']
    if revision=='v7':manifest['controller_changes'] += ['separate monitoring from intervention using rollout conflict time and turn/brake budget','path-progress deficit replaces body-speed stall objective','compare tracking/speed scaling/delayed detour; soft side commitment; feedback rejoin','no unconditional obstacle-trailing preference; all candidates use unchanged collision model','classified occupancy remains an ideal-sensor experimental option, not deployment-ready']
    if revision=='v8':manifest['controller_changes'] += ['preserve V7 safe tracking and no-dynamic policy','bounded bidirectional turning and two-stage recovery primitives','exploration continues when V7 candidates are all infeasible','in-horizon alternative-continuation score; not recursive feasibility','unchanged geometry, margin, prediction validity and native authority']
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(out,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--base',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--revision',choices=['v2','v3','v4','v5','v6','v7','v8'],default='v2')
    a=p.parse_args();build(a.base,a.out,a.revision)

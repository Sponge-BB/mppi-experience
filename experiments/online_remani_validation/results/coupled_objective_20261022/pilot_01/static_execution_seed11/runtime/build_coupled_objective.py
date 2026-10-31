#!/usr/bin/env python3
"""Build additive slow/fast optimization experiments, never production code."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source


def transform_optimizer(text):
    text='#include "coupling_bridge.hpp"\n#include "coupling_native_integration.hpp"\n'+text
    marker='    probe.add("pieces",requested_pieces);probe.add("requested_duration_s",requested_duration);'
    text=replace(text,marker,marker+'''
    coupling_geometry::Scope coupled_geometry;
    if(coupling_geometry::enabled()&&!coupled_geometry.valid())return false;
    coupling_bridge::Optimizer coupled_optimizer(input_hash,initT_container.size(),requested_pieces,singul_container);
''')
    marker='''    for(int trajid = 0; trajid < opt->traj_num_; trajid++){
      double time_cost = 0.0;'''
    text=replace(text,marker,'''    double extra_geometry=0,extra_prediction=0;
    try{
      extra_geometry=coupling_native::addGeometry(opt->SnapOpt_container_,T_container,
        opt->singul_container_,gradT_container,opt->traj_dim_);
      extra_prediction=coupling_bridge::addPrediction(opt->SnapOpt_container_,T_container,
        opt->singul_container_,gradT_container);
    }catch(const std::exception& error){
      dual_probe::emit("coupling_objective_error",dual_probe::Fields().str("reason",error.what()));
      opt->force_stop_type_=STOP_FOR_ERROR;g.setZero();return 1.0e100;
    }

'''+marker)
    text=replace(text,'double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost;',
                 'double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost + extra_geometry + extra_prediction;')
    return text


def transform_fsm(text):
    text='#include "coupling_bridge.hpp"\n'+text
    text=replace(text,'    collab::configure(nh);','    collab::configure(nh);\n    coupling_geometry::initialize(nh);')
    text=text.replace('execution_reference_pub_.publish(transaction);',
        'execution_reference_pub_.publish(transaction);++coupling_bridge::referenceRevision();')
    text=replace(text,'execution_goal_key_=executionHex(*msg);publishExecutionState();',
        'execution_goal_key_=executionHex(*msg);++coupling_bridge::goalGeneration();publishExecutionState();')
    marker='    collab::goalToken()=dual_probe::hash(execution_goal_key_)&0xffffffffULL;'
    text=replace(text,marker,marker+'''
    coupling_bridge::Request coupled_request(execution_goal_key_,start_pos_,start_yaw_,start_singul_);
    coupling_geometry::Scope coupled_geometry;
''')
    marker='      dual_validation.success=true;dual_validation.finish();'
    text=replace(text,marker,'''      if(!coupling_bridge::projectedCandidate(planner_manager_->traj_container_.singul_traj_data)){
        dual_probe::emit("acceptance_reject",dual_probe::Fields().str("reason","additional_projected_qualification"));
        return finish_failed_planning_attempt();
      }
'''+marker)
    return text


def build(base,out):
    base=base.resolve();out=out.resolve();m=json.loads((base/'build_manifest.json').read_text())
    if not m.get('collab_factorial'):raise ValueError('requires audited fixed-seed/timeline frozen base')
    if shutil.disk_usage(out.parent).free<500*1024**2:raise RuntimeError('reserve at least 500MiB before build')
    parent=Path(m['parent_build']);out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for n in ('mm_controller_node','dynamic_obstacle_tracker_node','libexp_opt.a'):
        shutil.copy2(base/'build'/n,out/'build'/n)
    headers=('coupling_bridge.hpp','coupling_objective.hpp','coupling_collision.hpp',
             'coupling_geometry_ros.hpp','coupling_native_integration.hpp','shared_collision.hpp')
    for n in headers:shutil.copy2(HERE/n,out/'source'/n)
    (out/'source/poly_traj_optimizer.cpp').write_text(transform_optimizer((base/'source/poly_traj_optimizer.cpp').read_text()))
    (out/'source/remani_replan_fsm.cpp').write_text(transform_fsm((base/'source/remani_replan_fsm.cpp').read_text()))
    _,optobj=compile_source(parent,out,'exp_opt',out/'source/poly_traj_optimizer.cpp','unused',out/'source')
    subprocess.run(['ar','r',str(out/'build/libexp_opt.a'),optobj],check=True)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for n in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(x for x in link if x.endswith('/'+n+'.o'))
        if n=='remani_replan_fsm.cpp':_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/n,old,out/'source')
        else:
            obj=base/'build'/(n+'.o')
            if not obj.exists():_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/n,old,out/'source')
        link=[str(obj) if x==old else x for x in link]
    search=base/'build/libexp_search.a'
    link=[str(out/'build/libexp_opt.a') if x=='libexp_opt.a' else str(search) if x=='libexp_search.a' else x for x in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    # The MPPI policy/score is IDENTICAL to frozen V12. This small ABI binds
    # the common detector measurement, without enabling legacy soft-cost,
    # rejoin, warm-reset, cusp, extra-radius or execution-policy patches.
    api='#include "shared_collision.hpp"\n'+(base/'source/online_api.cpp').read_text()+'''
extern "C" int online_set_collision_snapshot(void*p,const double*data,int count){
  if(!p||!data||count<16)return 1;
  auto&c=static_cast<Online*>(p)->c;
  if(std::abs(data[8]-c.margin)>1e-9||data[11]!=c.body.size()||count<16+3*int(c.body.size()))return 3;
  for(size_t i=0;i<c.body.size();++i)if(std::abs(data[16+3*i]-c.body[i].p.x)>1e-9||
    std::abs(data[17+3*i]-c.body[i].p.y)>1e-9||std::abs(data[18+3*i]-c.body[i].r)>1e-9)return 3;
  return shared_collision::bind(data,count)?0:2;
}
'''
    (out/'source/online_api.cpp').write_text(api)
    mlink=shlex.split((parent/'build/CMakeFiles/online_mppi.dir/link.txt').read_text())
    old=next(x for x in mlink if x.endswith('/online_api.cpp.o'))
    _,obj=compile_source(parent,out,'online_mppi',out/'source/online_api.cpp',old,out/'source')
    mlink=[obj if x==old else x for x in mlink];mlink[mlink.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(mlink,cwd=str(parent/'build'),check=True)
    m.update(coupling_objective=True,coupling_parent=str(base),shared_collision=True,
        controller_revision='v12_additive_optimization_coupling',production_modified=False,
        shared_collision_source_sha256=sha(out/'source/shared_collision.hpp'),
        coupling_scope='soft same-gear geometric prediction consistency and estimated-delivery compatibility; no committed prefix',
        engineering_scope='add projected static and bounded forecast objective/qualification; retain all native 3D and long-horizon guards; NOT full dynamic feasibility equivalence',
        mppi_policy_unchanged=True,initialization_unchanged=True,rolling_enabled=False,
        dynamic_long_horizon_certified=False,
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
        coupling_builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/collab_factorial_20261022_03')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)

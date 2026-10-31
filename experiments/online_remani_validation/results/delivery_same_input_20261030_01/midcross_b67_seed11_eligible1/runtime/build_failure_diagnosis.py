#!/usr/bin/env python3
"""Generate an isolated V12 solver diagnosis; no production edits/install."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source


HOOK = r'''
    static bool diagnosis_done=false;
    auto diagnose=[&](const char* outcome,const SingulTrajData* candidate)->bool {
      const char* requested=std::getenv("FAILURE_DIAG_KIND");
      if(!requested||diagnosis_done||resource_probe::label()!="normal"||max_wall_time_>.61||
         (std::string(requested)!=outcome&&std::string(requested)!="first_refresh"))return false;
      diagnosis_done=true;diagnosis.diagnostic_only=true;probe.add("diagnostic_only",1);
      const auto saved_ini=iniState_container,saved_fin=finState_container,saved_pts=initInnerPts_container;
      const auto saved_times=initT_container;const auto saved_signs=singul_container;
      const double saved_budget=max_wall_time_,saved_wheel=max_wheel_omega_;
      const std::string saved_reason=diagnosis.reason;
      const double saved_value=diagnosis.rejection_value,saved_time=diagnosis.rejection_time;
      failure_probe::PauseOwned paused;
      const double frozen_sim=ros::Time::now().toSec();
      const auto frozen_raw=grid_map_->getDynamicObservationSeq(),frozen_esdf=grid_map_->getESDFObservationSeq();
      std::ostringstream request;request<<std::setprecision(17)<<"{\"kind\":\"snapshot\",\"outcome\":\""<<outcome
        <<"\",\"original_reason\":\""<<saved_reason<<"\",\"input_hash\":\""<<input_hash<<"\",\"sim\":"<<frozen_sim<<",\"raw_seq\":"<<frozen_raw
        <<",\"esdf_seq\":"<<frozen_esdf<<",\"initial_states\":";
      failure_probe::matrices(request,saved_ini);request<<",\"final_states\":";failure_probe::matrices(request,saved_fin);
      request<<",\"inner_points\":";failure_probe::matrices(request,saved_pts);
      request<<",\"initial_times\":";failure_probe::matrices(request,saved_times);request<<",\"directions\":[";
      for(size_t i=0;i<saved_signs.size();++i){if(i)request<<',';request<<saved_signs[i];}request<<"]}";
      failure_probe::emit(request.str());
      if(candidate){
        for(double limit:{3.28,3.50}){
          max_wheel_omega_=limit;diagnosis.reason="ok";
          const bool ok=IsTrajSafe(*candidate);
          std::ostringstream s;s<<std::setprecision(17)<<"{\"kind\":\"recheck\",\"wheel_limit\":"<<limit
            <<",\"input_hash\":\""<<input_hash<<"\",\"success\":"<<(ok?"true":"false")
            <<",\"reason\":\""<<(ok?"ok":diagnosis.reason)<<"\",\"value\":"<<diagnosis.rejection_value
            <<",\"t\":"<<diagnosis.rejection_time<<'}';failure_probe::emit(s.str());
        }
      }
      // Counterbalance short/long runs; rebuild from copied INPUT arrays,
      // never from the optimized output or previously mutated member state.
      unsetenv("FAILURE_PROFILE_DISABLED");
      const std::vector<double> budgets={.60,3.0,1.50,1.50,3.0,.60};
      for(size_t i=0;i<budgets.size();++i){
        max_wall_time_=budgets[i];max_wheel_omega_=saved_wheel;
        resource_probe::label()="budget_"+std::to_string(i);
        std::vector<Eigen::MatrixXd> cp,wp;std::vector<Eigen::VectorXd> times,ee;
        OptimizeTrajectory_lbfgs(saved_ini,saved_fin,saved_pts,saved_times,saved_signs,cp,wp,times,ee);
      }
      // Same request without leaf timers: quantify observer perturbation,
      // not just an empty clock-call microbenchmark.
      setenv("FAILURE_PROFILE_DISABLED","1",1);
      int unprofiled=0;for(double budget:{.60,3.0,1.50,1.50,3.0,.60}){
        max_wall_time_=budget;max_wheel_omega_=saved_wheel;
        resource_probe::label()="unprofiled_"+std::to_string(unprofiled++);
        std::vector<Eigen::MatrixXd> cp,wp;std::vector<Eigen::VectorXd> times,ee;
        OptimizeTrajectory_lbfgs(saved_ini,saved_fin,saved_pts,saved_times,saved_signs,cp,wp,times,ee);
      }
      int wheel_pass=0;for(double budget:{.60,3.0,1.50,1.50,3.0,.60}){
        max_wall_time_=budget;max_wheel_omega_=3.50;resource_probe::label()="wheel_350_"+std::to_string(wheel_pass++);
        std::vector<Eigen::MatrixXd> cp,wp;std::vector<Eigen::VectorXd> times,ee;
        OptimizeTrajectory_lbfgs(saved_ini,saved_fin,saved_pts,saved_times,saved_signs,cp,wp,times,ee);
      }
      // Natural online calls run WITHOUT expensive leaf timers.
      setenv("FAILURE_PROFILE_DISABLED","1",1);
      resource_probe::label()="normal";max_wall_time_=saved_budget;max_wheel_omega_=saved_wheel;
      diagnosis.reason=saved_reason;diagnosis.rejection_value=saved_value;diagnosis.rejection_time=saved_time;
      const bool stable=std::abs(ros::Time::now().toSec()-frozen_sim)<1e-9&&
        frozen_raw==grid_map_->getDynamicObservationSeq()&&frozen_esdf==grid_map_->getESDFObservationSeq();
      failure_probe::emit(std::string("{\"kind\":\"snapshot_end\",\"stable\":")+(stable?"true":"false")+"}");
      const char* done=std::getenv("FAILURE_DONE_PATH");
      if(done){std::ofstream f(done);f<<"{\"complete\":true,\"snapshot_stable\":"<<(stable?"true":"false")<<"}\n";}
      return true;
    };
'''


def build(base, out, reuse=None):
    base=base.resolve();out=out.resolve()
    m=json.loads((base/'build_manifest.json').read_text());parent=Path(m['parent_build'])
    out.mkdir(parents=True,exist_ok=False);shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    shutil.copy2(HERE/'failure_probe.hpp',out/'source/failure_probe.hpp')
    text=(base/'source/poly_traj_optimizer.cpp').read_text()
    text=replace(text,'#include "resource_probe.hpp"','#include "resource_probe.hpp"\n#include "failure_probe.hpp"')
    text=replace(text,'    probe.add("input_hash",input_hash);probe.add("budget_s",max_wall_time_);',
        '''    failure_probe::Frame diagnosis(max_wall_time_,max_wheel_omega_,input_hash,
        grid_map_->getDynamicObservationSeq(),grid_map_->getESDFObservationSeq());
    probe.add("input_hash",input_hash);probe.add("budget_s",max_wall_time_);'''+HOOK)
    # Remove the old static CPU-load experiment entirely; no nested diagnostics.
    first=text.index('    // Paired diagnostic only:')
    last=text.index('    if((result != lbfgs::LBFGS_CONVERGENCE)',first)
    text=text[:first]+'''    diagnosis.add("iterations",iter_num_);diagnosis.add("lbfgs_result",result);
    diagnosis.add("force_stop",int(force_stop_type_));diagnosis.add("final_cost",final_cost);
    diagnosis.add("lbfgs_wall_ms",(ros::WallTime::now()-optimization_start_wall_time_).toSec()*1000.);
'''+text[last:]
    text=replace(text,'    if(force_stop_type_ == STOP_FOR_TIMEOUT){',
        '    if(force_stop_type_ == STOP_FOR_TIMEOUT){\n      diagnosis.reason="budget_timeout";diagnose("timeout",nullptr);')
    text=replace(text,'    if(force_stop_type_ == STOP_FOR_ERROR){',
        '    if(force_stop_type_ == STOP_FOR_ERROR){\n      diagnosis.reason="nonfinite";')
    text=replace(text,'    good_traj = IsTrajSafe(singul_traj_data);', '''    diagnosis.reason="ok";
    good_traj = IsTrajSafe(singul_traj_data);
    failure_probe::trajectoryStats(singul_traj_data,mobile_base_wheel_base_,mobile_base_wheel_radius_);
    if(resource_probe::label().find("budget_")==0&&max_wall_time_==3.0){
      const double saved_limit=max_wheel_omega_,saved_value=diagnosis.rejection_value,saved_time=diagnosis.rejection_time;
      const std::string saved_reason=diagnosis.reason;
      for(double limit:{3.28,3.50}){
        max_wheel_omega_=limit;diagnosis.reason="ok";diagnosis.rejection_value=0;diagnosis.rejection_time=0;
        const bool ok=IsTrajSafe(singul_traj_data);
        std::ostringstream s;s<<std::setprecision(17)<<"{\\"kind\\":\\"recheck\\",\\"wheel_limit\\":"<<limit
          <<",\\"source_label\\":\\""<<resource_probe::label()<<"\\",\\"input_hash\\":\\""<<input_hash
          <<"\\",\\"success\\":"<<(ok?"true":"false")<<",\\"reason\\":\\""<<(ok?"ok":diagnosis.reason)
          <<"\\",\\"value\\":"<<diagnosis.rejection_value<<",\\"t\\":"<<diagnosis.rejection_time<<'}';
        failure_probe::emit(s.str());
      }
      max_wheel_omega_=saved_limit;diagnosis.reason=saved_reason;diagnosis.rejection_value=saved_value;diagnosis.rejection_time=saved_time;
    }
    if(!good_traj&&(diagnosis.reason=="left_wheel_speed"||diagnosis.reason=="right_wheel_speed")){
      if(diagnose("wheel_rejection",&singul_traj_data))return false;
    }
    if(diagnose("completed",&singul_traj_data))return false;''')
    text=replace(text,'    probe.success=good_traj;','    diagnosis.complete=good_traj;\n    probe.success=good_traj;')
    reject_names={'wheel_omega_left':'left_wheel_speed','wheel_omega_right':'right_wheel_speed',
                  'wheel_alpha_left':'left_wheel_acceleration','wheel_alpha_right':'right_wheel_acceleration'}
    for variable,reason in reject_names.items():
        anchor='      ROS_WARN("%f max '+('left' if 'left' in variable else 'right')+' wheel '+('omega' if 'omega' in variable else 'alpha')+' is not feasible at relative time %f! opt failed", '+variable+', t );'
        text=replace(text,anchor,'      failure_probe::reject("'+reason+'",'+variable+',t);\n'+anchor)
    text=replace(text,'      if(checkPlanningCollision(traj_data, t, coll_type)){',
                 '      if(checkPlanningCollision(traj_data, t, coll_type)){\n        failure_probe::reject("planning_collision",coll_type,t);')
    # Profile nested stages and the disjoint leaf functions inside integration.
    text=replace(text,'    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);',
        '    failure_probe::Timer callback_timer(failure_probe::COST);\n    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);')
    text=replace(text,'    opt->iter_num_ += 1;','    failure_probe::firstCost(costall,g);\n    opt->iter_num_ += 1;')
    text=replace(text,'    costs.setZero();\n    int N = gdT.size();',
                 '    failure_probe::Timer timer(failure_probe::INTEGRAL);\n    costs.setZero();\n    int N = gdT.size();')
    text=replace(text,'    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)',
                 '    failure_probe::Timer timer(failure_probe::OBSTACLE);\n    // if (i_dp == 0 || i_dp >= cps_.cp_size * 2 / 3)')
    text=replace(text,'    cost_mm_feasible = 0;','    failure_probe::Timer timer(failure_probe::BASE_FEAS);\n    cost_mm_feasible = 0;')
    joint=text.index('  bool PolyTrajOptimizer::feasibilityGradCostJoint(');brace=text.index('{',joint)
    text=text[:brace+1]+'\n    failure_probe::Timer timer(failure_probe::JOINT_FEAS);'+text[brace+1:]
    text=replace(text,'    SnapOpt_container_[trajid].initGradCost(gdT, cost);',
                 '    failure_probe::Timer timer(failure_probe::SMOOTH);\n    SnapOpt_container_[trajid].initGradCost(gdT, cost);')
    text=replace(text,'      opt->SnapOpt_container_[trajid].getGrad2TP(gradT_container[trajid], gradP_container[trajid], gradIni, gradFin);',
                 '      {failure_probe::Timer timer(failure_probe::PROPAGATE);\n      opt->SnapOpt_container_[trajid].getGrad2TP(gradT_container[trajid], gradP_container[trajid], gradIni, gradFin);}')
    # Measure each ESDF query without changing its result or evaluation order.
    for anchor in ('      grid_map_->evaluatePlanningEDTWithGrad(car_pts[i], dist, dist_grad);',
                   '        grid_map_->evaluatePlanningEDTWithGrad(\n            pt_on_link, dist, dist_grad);'):
        if anchor not in text:
            if 'car_pts' in anchor:
                anchor='      grid_map_->evaluatePlanningEDTWithGrad(\n          car_pts[i], dist, dist_grad);'
        text=replace(text,anchor,'{failure_probe::Timer timer(failure_probe::EDT);\n'+anchor+'}')
    (out/'source/poly_traj_optimizer.cpp').write_text(text)
    _,obj=compile_source(parent,out,'exp_opt',out/'source/poly_traj_optimizer.cpp','unused',out/'source')
    archive=out/'build/libexp_opt.a';shutil.copy2(base/'build/libexp_opt.a',archive)
    subprocess.run(['ar','r',str(archive),obj],check=True)
    finish_build(base,out,reuse)


def finish_build(base,out,reuse=None):
    base=base.resolve();out=out.resolve();reuse=reuse.resolve() if reuse else None
    m=json.loads((base/'build_manifest.json').read_text());parent=Path(m['parent_build'])
    archive=out/'build/libexp_opt.a'
    if not archive.exists():raise RuntimeError('optimizer archive not yet built')
    cm=parent/'build/CMakeFiles/remani_planner_node.dir';link=shlex.split((cm/'link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(s for s in link if s.endswith('/'+name+'.o'))
        if reuse:
            if sha(out/'source'/name)!=sha(reuse/'source'/name):raise RuntimeError('cannot reuse changed source '+name)
            obj=str(reuse/'build'/(name+'.o'))
        else:_,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if s==old else s for s in link]
    link=[str(archive) if s=='libexp_opt.a' else str(base/'build/libexp_search.a') if s=='libexp_search.a' else s for s in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node');subprocess.run(link,cwd=str(parent/'build'),check=True)
    # Test-only access to existing controller methods; no implementation change.
    inc=out/'source/mm_controller';inc.mkdir(exist_ok=True)
    header=(parent/'source/mm_controller/include/mm_controller/mm_controller_fsm.hpp').read_text()
    (inc/'mm_controller_fsm.hpp').write_text(header.replace('private:','public:'))
    shutil.copy2(HERE/'freeze_reference_probe.cpp',out/'source/freeze_reference_probe.cpp')
    if reuse:
        if sha(out/'source/freeze_reference_probe.cpp')!=sha(reuse/'source/freeze_reference_probe.cpp'):raise RuntimeError('changed freeze probe')
        shutil.copy2(reuse/'build/freeze_reference_probe',out/'build/freeze_reference_probe')
    else:
        _,obj=compile_source(parent,out,'mm_controller_node',out/'source/freeze_reference_probe.cpp','unused',out/'source')
        cm=parent/'build/CMakeFiles/mm_controller_node.dir';link=shlex.split((cm/'link.txt').read_text())
        old=next(s for s in link if s.endswith('/mm_controller_node.cpp.o'));link=[obj if s==old else s for s in link]
        link[link.index('-o')+1]=str(out/'build/freeze_reference_probe');subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(failure_diagnosis=True,diagnosis_parent=str(base),
             diagnosis_scope='frozen live request replay; instrumented objective; fixed-reference native clock probe',
             native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
             isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=HERE/'builds/resource_probe_v12_01')
    p.add_argument('--out',type=Path,required=True);p.add_argument('--reuse',type=Path);p.add_argument('--finish-only',action='store_true')
    a=p.parse_args();(finish_build if a.finish_only else build)(a.base,a.out,a.reuse)

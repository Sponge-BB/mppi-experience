#!/usr/bin/env python3
"""One static constraint-multiplier update/restart in the ORIGINAL SIL budget."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from audit_dual_timescale import build_integrity
from build_native_v9 import HERE,sha,replace
from build_native_v10 import compile_source

RESTORATION = r'''
    const auto nominal_clearance=clearance_control::scan(SnapOpt_container_,singul_container_,clearance_window);
    clearance_record.capture("nominal",SnapOpt_container_,singul_container_);
    clearance_control::emitScan("clearance_nominal",nominal_clearance,clearance_window);
    if(!nominal_clearance.valid){
      clearance_record.outcome="nominal_scan_"+nominal_clearance.reason;
      diagnosis.reason=clearance_record.outcome;return false;
    }
    if(clearance_control::enabled() && nominal_clearance.gap < -1e-8){
      if(!clearance_window.window.claim(ros::WallTime::now().toSec())){
        clearance_record.outcome="insufficient_remaining_budget";
        dual_probe::emit("clearance_skipped",dual_probe::Fields().str("reason",clearance_record.outcome)
          .add("remaining_s",clearance_window.window.remaining(ros::WallTime::now().toSec())));
        diagnosis.reason=clearance_record.outcome;return false;
      }
      const double restart_begin=ros::WallTime::now().toSec();
      std::vector<Eigen::VectorXd> recovery_times;
      for(const auto& opt:SnapOpt_container_)recovery_times.push_back(opt.get_T1());
      try {
        const auto multipliers=clearance_native::prepareMultipliers(SnapOpt_container_,recovery_times,
            singul_container_,traj_dim_,clearance_control::rho);
        if(clearance_window.window.recoveryExpired(ros::WallTime::now().toSec())){
          clearance_record.outcome="budget_exhausted_preparing_constraints";
          diagnosis.reason=clearance_record.outcome;return false;
        }
        // Fixed topology, immutable multipliers, fresh L-BFGS history, one restart.
        // DO NOT reset optimization_start_wall_time_ or max_wall_time_.
        clearance_native::Active restore(clearance_control::rho,multipliers);
        auto recovery_params=lbfgs_params;
        recovery_params.max_iterations=(lbfgs_params.max_iterations==0)?50:
            std::min(50,lbfgs_params.max_iterations);
        domain_step::reset();
        dual_probe::emit("clearance_restart_begin",dual_probe::Fields()
          .add("rho",clearance_control::rho).add("constraints",multipliers.size())
          .add("remaining_s",clearance_window.window.remaining(restart_begin))
          .add("validation_reserve_s",clearance_control::validation_reserve));
        result=domain_lbfgs::lbfgs_optimize(x,final_cost,PolyTrajOptimizer::costFunctionCallback,
            NULL,PolyTrajOptimizer::earlyExitCallback,this,recovery_params);
        dual_probe::emit("clearance_restart_end",dual_probe::Fields().add("lbfgs_result",result)
          .add("force_stop",int(force_stop_type_))
          .add("wall_ms",(ros::WallTime::now().toSec()-restart_begin)*1000.)
          .add("rejected_trials",domain_step::rejectedTrials())
          .add("accepted_steps_after_rejection",domain_step::acceptedStepsAfterRejection()));
        if(result<0 || force_stop_type_!=DONT_STOP){
          clearance_record.outcome="recovery_solver_failure";
          clearance_record.capture("failed_last_evaluated_trial_NOT_canonical_x",SnapOpt_container_,singul_container_);
          diagnosis.reason=clearance_record.outcome;return false;
        }
        // Rebuild EVERY callback-owned polynomial/control-point cache from x.
        Eigen::VectorXd coherent_gradient=Eigen::VectorXd::Zero(x.size());
        final_cost=PolyTrajOptimizer::costFunctionCallback(this,x,coherent_gradient);
        if(force_stop_type_!=DONT_STOP || !std::isfinite(final_cost)||!coherent_gradient.allFinite()){
          clearance_record.outcome="cache_reconstruction_failed";
          diagnosis.reason=clearance_record.outcome;return false;
        }
        clearance_record.capture("restored_canonical_x",SnapOpt_container_,singul_container_);
      }catch(const std::exception& error){
        clearance_record.outcome="recovery_error";
        dual_probe::emit("clearance_error",dual_probe::Fields().str("reason",error.what()));
        diagnosis.reason=clearance_record.outcome;return false;
      }
      const auto restored_clearance=clearance_control::scan(SnapOpt_container_,singul_container_,clearance_window);
      clearance_control::emitScan("clearance_restored",restored_clearance,clearance_window);
      if(!restored_clearance.valid || restored_clearance.gap < -1e-8){
        clearance_record.outcome="restored_static_margin_not_feasible";
        diagnosis.reason=clearance_record.outcome;return false;
      }
    }
    if(clearance_window.expired()){
      clearance_record.outcome="budget_exhausted_before_native_gate";
      diagnosis.reason=clearance_record.outcome;return false;
    }
'''


def transform_optimizer(text):
    text='#include "clearance_control.hpp"\n'+text
    marker='    optimization_start_wall_time_ = ros::WallTime::now();'
    text=replace(text,marker,marker+'''
    clearance_control::Frame clearance_window(optimization_start_wall_time_.toSec(),max_wall_time_);
    clearance_control::Record clearance_record;
''')
    text=replace(text,'    // test collision','    if(coupling_geometry::enabled()){\n'+RESTORATION+'    }\n\n    // test collision')
    text=replace(text,'    double extra_geometry=0,extra_prediction=0;',
        '    double extra_geometry=0,extra_prediction=0,extra_clearance=0;')
    marker='''      extra_prediction=coupling_bridge::addPrediction(opt->SnapOpt_container_,T_container,
        opt->singul_container_,gradT_container);'''
    text=replace(text,marker,marker+'''
      extra_clearance=clearance_native::addPenalty(opt->SnapOpt_container_,T_container,
        opt->singul_container_,gradT_container,opt->traj_dim_);''')
    text=replace(text,' + extra_geometry + extra_prediction;',
        ' + extra_geometry + extra_prediction + extra_clearance;')
    marker='    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(instance);'
    text=replace(text,marker,marker+'''
    if(clearance_control::stopRecovery()){
      opt->force_stop_type_=STOP_FOR_TIMEOUT;g.setZero();return 1.0e100;
    }''')
    marker='''    if((ros::WallTime::now() - opt->optimization_start_wall_time_).toSec() >=
       opt->max_wall_time_)
      opt->force_stop_type_ = STOP_FOR_TIMEOUT;'''
    text=replace(text,marker,'''    if(clearance_control::stopRecovery() ||
       (ros::WallTime::now() - opt->optimization_start_wall_time_).toSec() >= opt->max_wall_time_)
      opt->force_stop_type_ = STOP_FOR_TIMEOUT;''')
    marker='    diagnosis.complete=good_traj;'
    text=replace(text,marker,'''    if(clearance_window.expired()){
      clearance_record.outcome="original_budget_exhausted_in_native_validation";
      diagnosis.reason=clearance_record.outcome;
      dual_probe::emit("clearance_deadline_reject",dual_probe::Fields()
        .add("elapsed_s",clearance_window.window.elapsed(ros::WallTime::now().toSec()))
        .add("budget_s",max_wall_time_));
      return false;
    }
    clearance_record.outcome=good_traj?"native_qualified_pending_latest_gate":"native_reject_"+diagnosis.reason;
'''+marker)
    return text


def build(base,out):
    base,out=base.resolve(),out.resolve()
    if not build_integrity(base,{})['pass_integrity']:raise ValueError('Base integrity failed')
    m=json.loads((base/'build_manifest.json').read_text())
    if not m.get('domain_recovery_experiment') or m.get('clearance_recovery_experiment'):
        raise ValueError('Requires frozen domain-recovery build')
    if shutil.disk_usage(out.parent).free<2*1024**3:raise RuntimeError('Less than 2GiB free')
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');shutil.copytree(base/'build',out/'build')
    for name in ('clearance_penalty.hpp','clearance_native.hpp','clearance_control.hpp','recovery_budget.hpp'):
        shutil.copy2(HERE/name,out/'source'/name)
    source=out/'source/poly_traj_optimizer.cpp';source.write_text(transform_optimizer(source.read_text()))
    parent=Path(m['parent_build'])
    _,obj=compile_source(parent,out,'exp_opt',source,'',out/'source')
    subprocess.run(['ar','r',str(out/'build/libexp_opt.a'),obj],check=True)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(x for x in link if x.endswith('/'+name+'.o'))
        link=[str(out/'build'/(name+'.o')) if x==old else x for x in link]
    search=Path(m['coupling_parent'])/'build/libexp_search.a'
    link=[str(out/'build/libexp_opt.a') if x=='libexp_opt.a' else str(search) if x=='libexp_search.a' else x for x in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(clearance_recovery_experiment=True,clearance_parent=str(base),
      clearance_parent_manifest_sha256=sha(base/'build_manifest.json'),
      clearance_scope='one static collocation AL multiplier update and one fresh-history restart within original deadline',
      clearance_rho=1e5,clearance_validation_reserve_s=.04,clearance_minimum_restart_s=.02,
      clearance_restart_limit=1,clearance_max_restart_iterations=50,
      common_pair_budget_change='both OFF/ON include solve-snapshot scan and reject native validation over original optimization deadline',
      mppi_policy_unchanged=True,hard_margin_unchanged=True,production_modified=False,
      isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
      native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
      clearance_builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    if not build_integrity(out,{})['pass_integrity']:raise RuntimeError('Derived integrity failed')
    print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/domain_recovery_20261023_01')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)

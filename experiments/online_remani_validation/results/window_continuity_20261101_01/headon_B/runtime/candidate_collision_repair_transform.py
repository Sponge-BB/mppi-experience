"""Patch only the experimental optimizer: one bounded collision repair."""
from native_collision_runtime import replace_once


def transform(text):
    text = replace_once(text, '#include "temporal_collision_capture.hpp"',
                        '#include "temporal_collision_capture.hpp"\n#include "candidate_collision_repair.hpp"')
    text = replace_once(text, '    delivery_candidate::Scope accepted_candidate;',
                        '    candidate_collision_repair::Scope collision_repair;\n    delivery_candidate::Scope accepted_candidate;')
    text = replace_once(text, '    int result;\n    try {',
                        '    int result;\n    Eigen::VectorXd collision_repair_scale;\n    try {')
    text = replace_once(text, '      const auto prepared=proxy.estimate(x);',
                        '      const auto prepared=proxy.estimate(x);\n      collision_repair_scale=prepared.scale;')
    text = replace_once(text, '    delivery_candidate::beginEvaluation();',
                        '    delivery_candidate::beginEvaluation();\n    candidate_collision_repair::beginEvaluation();')
    text = replace_once(text, '    delivery_candidate::completed(x,g,costall);',
        '    delivery_candidate::completed(x,g,costall);\n'
        '    candidate_collision_repair::completed(x,g,costall,total_smoo_cost,total_obs_feas_costs,total_time_cost,extra_geometry,extra_prediction);')
    text = replace_once(text, '    if(opt->force_stop_type_==DONT_STOP)delivery_candidate::accepted(x,fx);',
        '    if(opt->force_stop_type_==DONT_STOP){delivery_candidate::accepted(x,fx);candidate_collision_repair::accepted(x);}')
    text = replace_once(text, '        // collision\n        gradp.setZero();',
        '        // A constant per-piece weight has zero timing derivative; all original\n'
        '        // spatial and explicit CV-time derivatives use the same multiplier.\n'
        '        const double repair_obstacle_weight=candidate_collision_repair::weight(trajid,i);\n'
        '        // collision\n        gradp.setZero();')
    text = replace_once(text, '        if(obstacleGradCostforMM(i_dp, pos, vel, trajid, gradp, gradv, cost_car, cost_mani, cost_self))',
        '        candidate_collision_repair::sample_weight()=repair_obstacle_weight;\n'
        '        if(obstacleGradCostforMM(i_dp, pos, vel, trajid, gradp, gradv, cost_car, cost_mani, cost_self))')
    text = replace_once(text, '        time_indexed_collision::accumulateTimeGradient(trajid,i,alpha,omg*step);',
        '        candidate_collision_repair::sample(trajid,i,omg*step,cost_car,cost_mani,cost_self,gradp,gradv,\n'
        '          time_indexed_collision::sample()?time_indexed_collision::sample()->temporal_cost_gradient:0.);\n'
        '        candidate_collision_repair::sample_weight()=1.;\n'
        '        time_indexed_collision::accumulateTimeGradient(trajid,i,alpha,omg*step);')
    # Scale environmental penalties only, including their explicit CV-time derivatives.
    start=text.index('  bool PolyTrajOptimizer::obstacleGradCostforMM(')
    end=text.index('    // check mani link with robot',start)
    body=text[start:end]
    body=replace_once(body,'    bool ret = false;',
        '    bool ret = false;\n'
        '    const double repair_weight=candidate_collision_repair::sample_weight();\n'
        '    const double environment_base_weight=wei_obs_*repair_weight;\n'
        '    const double environment_arm_weight=wei_mani_obs_*repair_weight;')
    # Ground terms retain their original weighting.
    env_end=body.index('        Eigen::VectorXd dzdP')
    body=body[:env_end].replace('wei_obs_ *','environment_base_weight *').replace('wei_obs_*3.','environment_base_weight*3.').replace('wei_mani_obs_ *','environment_arm_weight *').replace('wei_mani_obs_*factor','environment_arm_weight*factor')+body[env_end:]
    text=text[:start]+body+text[end:]
    anchor='    wheel_cause::nativeGate(good_traj,diagnosis.reason,diagnosis.rejection_value,diagnosis.rejection_time);'
    text=replace_once(text,anchor,REPAIR+'\n'+anchor)
    text=replace_once(text,'    probe.add("native_validation_ms",(ros::WallTime::now()-native_validation_start).toSec()*1000.);',
        '    probe.add("native_validation_ms",collision_repair_native_before_ms+collision_repair_native_after_ms);')
    return text


REPAIR=r'''    const double collision_repair_native_before_ms=(ros::WallTime::now()-native_validation_start).toSec()*1000.;
    double collision_repair_native_after_ms=0.;
    // The only treatment: reuse this final polynomial and the unspent original
    // wall budget for one environmental-collision repair, without another search.
    if(!good_traj && diagnosis.reason=="planning_collision"){
      const std::string original_reason=diagnosis.reason;
      const double original_rejection_time=diagnosis.rejection_time;
      const int original_collision_type=int(diagnosis.rejection_value);
      const int original_result=result;
      const double before_elapsed=(ros::WallTime::now()-optimization_start_wall_time_).toSec();
      std::vector<Eigen::VectorXd> actual_times;
      for(const auto& snap:SnapOpt_container_)actual_times.push_back(snap.get_T1());
      collision_repair.target(actual_times,original_rejection_time);
      candidate_collision_repair::candidate("before_repair",singul_traj_data,false,original_reason);
      auto original_stats=candidate_collision_repair::diagnostics(collision_repair,x);
      original_stats.add("enabled",collision_repair.enabled?"true":"false")
        .add("original_lbfgs_result",original_result).add("original_force_stop",int(force_stop_type_))
        .str("original_reason",original_reason).add("original_collision_type",original_collision_type)
        .add("original_rejection_time",original_rejection_time)
        .add("target_trajectory",collision_repair.target_trajectory).add("target_piece",collision_repair.target_piece)
        .add("factor",candidate_collision_repair::factor).add("shared_budget_s",max_wall_time_)
        .add("elapsed_s",before_elapsed).add("remaining_budget_s",std::max(0.,max_wall_time_-before_elapsed))
        .add("budget_remaining_ms",1000.*std::max(0.,max_wall_time_-before_elapsed))
        .add("original_solver_wall_ms",candidate_solver_wall*1000.)
        .add("native_check_before_ms",collision_repair_native_before_ms)
        .str("candidate_x_hash",std::to_string(delivery_candidate::hash(x)))
        .str("gradient_stat_semantics","integrated per-sample norm sums; not norm of propagated decision gradient");
      dual_probe::emit("candidate_collision_repair_diagnosis",original_stats);
      if(collision_repair.enabled && result>=0 && force_stop_type_==DONT_STOP &&
         original_collision_type>=0 && original_collision_type<=1 &&
         collision_repair.target_piece>=0 &&
         (ros::WallTime::now()-optimization_start_wall_time_).toSec()<max_wall_time_){
        const double repair_started=ros::WallTime::now().toSec();
        const double original_budget_start=optimization_start_wall_time_.toSec();
        dual_probe::emit("candidate_collision_repair_begin",original_stats);
        auto reconstruct=[&](const Eigen::VectorXd& point){
          std::vector<delivery_candidate::Part> parts;
          if(!delivery_candidate::decode(point,piece_num_container_,traj_dim_,iniState_container_,
               finState_container_,non_singul_v_,parts))return false;
          singul_traj_data.clearSingulTraj();double prefix=0.;
          for(int i=0;i<traj_num_;++i){
            SnapOpt_container_[i].generate(parts[i].points,parts[i].times,parts[i].initial,parts[i].final);
            cps_container_[i].points=SnapOpt_container_[i].getInitConstrainPoints(cps_num_prePiece_);
            if(!SnapOpt_container_[i].get_b().allFinite()||!cps_container_[i].points.allFinite())return false;
            singul_traj_data.addSingulTraj(SnapOpt_container_[i].getTraj(singul_container_[i]),prefix);
            prefix=singul_traj_data.duration;
          }
          return true;
        };
        const bool warm_start_rebuilt=reconstruct(x);
        bool repair_selected=false,repair_timed_out=false,repair_candidate_usable=false;
        double repair_solver_wall_ms=0.,repair_solve_start_elapsed_s=0.;
        int repair_accepted_steps=0;std::string repair_status="warm_start_decode_failed";
        if(warm_start_rebuilt){
          collision_repair.repairing=true;
          delivery_candidate::Scope repair_candidate;
          early_delivery::Session repair_early(early_delivery_enabled,early_delivery_checker.get(),
            [this](){return (ros::WallTime::now()-optimization_start_wall_time_).toSec();},max_wall_time_,
            [this](const SingulTrajData& candidate){
              delivery_replay::RestoreFailure restore;early_delivery::NativeVerdict verdict;
              verdict.pass=IsTrajSafe(candidate);const auto* frame=failure_probe::active();
              verdict.reason=verdict.pass?"ok":(frame?frame->reason:"native_reject");return verdict;
            });
          // Original accepted/selected vectors belong to the previous objective.
          // Fresh scopes prevent them from being reused as repair certificates.
          domain_step::reset();diagnosis.reason="repair_solving";
          try{
            const auto repair_solve_started=ros::WallTime::now();
            repair_solve_start_elapsed_s=(repair_solve_started-optimization_start_wall_time_).toSec();
            if(collision_repair_scale.size()==x.size()){
              cheap_online_callbacks::Context callback{this,PolyTrajOptimizer::costFunctionCallback,
                PolyTrajOptimizer::earlyExitCallback,x,collision_repair_scale,{}};
              Eigen::VectorXd z=Eigen::VectorXd::Zero(x.size());
              result=domain_lbfgs::lbfgs_optimize(z,final_cost,cheap_online_callbacks::Context::evaluate,NULL,
                cheap_online_callbacks::Context::progress,&callback,lbfgs_params);
              x=callback.physical(z);
            }else result=domain_lbfgs::lbfgs_optimize(x,final_cost,PolyTrajOptimizer::costFunctionCallback,NULL,
                PolyTrajOptimizer::earlyExitCallback,this,lbfgs_params);
            repair_solver_wall_ms=1000.*(ros::WallTime::now()-repair_solve_started).toSec();
            repair_selected=repair_early.selected;
            repair_timed_out=force_stop_type_==STOP_FOR_TIMEOUT;
            repair_candidate_usable=repair_candidate.usable();repair_accepted_steps=repair_candidate.accepted_steps;
            if(repair_selected)x=repair_early.selected_x;
            else if(repair_timed_out&&repair_candidate_usable){x=repair_candidate.x;salvage_attempted=true;}
            const bool terminal_ok=(result>=0 || (repair_timed_out&&repair_candidate_usable)) && force_stop_type_!=STOP_FOR_ERROR;
            const bool rebuilt=reconstruct(x);
            if(!terminal_ok){good_traj=false;diagnosis.reason=repair_timed_out?"budget_timeout":"terminal_solver_failure";}
            else if(!rebuilt){good_traj=false;diagnosis.reason="repair_candidate_rebuild_failed";}
            else{
              diagnosis.reason="ok";diagnosis.rejection_time=0.;diagnosis.rejection_value=0.;
              const auto repair_native_started=ros::WallTime::now();
              {handoff_alignment::FinalGate final_gate;good_traj=IsTrajSafe(singul_traj_data);}
              collision_repair_native_after_ms=1000.*(ros::WallTime::now()-repair_native_started).toSec();
            }
            repair_status=diagnosis.reason;
          }catch(const domain_step::RejectedTrial& error){
            good_traj=false;diagnosis.reason="repair_initial_static_map_domain";repair_status=diagnosis.reason;
          }
          collision_repair.repairing=false;
        }else{good_traj=false;diagnosis.reason=repair_status;}
        // A failed trial cache never becomes a published polynomial.
        candidate_collision_repair::candidate("after_repair",singul_traj_data,good_traj,diagnosis.reason);
        auto repaired_stats=candidate_collision_repair::diagnostics(collision_repair,x);
        repaired_stats.add("enabled","true").add("original_lbfgs_result",original_result)
          .str("original_reason",original_reason).add("original_rejection_time",original_rejection_time)
          .add("target_trajectory",collision_repair.target_trajectory).add("target_piece",collision_repair.target_piece)
          .add("factor",candidate_collision_repair::factor).add("repair_lbfgs_result",result)
          .add("repair_force_stop",int(force_stop_type_)).str("repair_reason",repair_status)
          .add("final_rejection_time",diagnosis.rejection_time).add("final_rejection_value",diagnosis.rejection_value)
          .add("native_pass",good_traj?"true":"false").add("warm_start_rebuilt",warm_start_rebuilt?"true":"false")
          .add("early_selected",repair_selected?"true":"false").add("timed_out",repair_timed_out?"true":"false")
          .add("candidate_usable",repair_candidate_usable?"true":"false").add("accepted_steps",repair_accepted_steps)
          .add("original_budget_start_s",original_budget_start).add("final_budget_start_s",optimization_start_wall_time_.toSec())
          .add("shared_budget_s",max_wall_time_).add("elapsed_s",(ros::WallTime::now()-optimization_start_wall_time_).toSec())
          .add("remaining_budget_s",std::max(0.,max_wall_time_-(ros::WallTime::now()-optimization_start_wall_time_).toSec()))
          .add("budget_remaining_ms",1000.*std::max(0.,max_wall_time_-(ros::WallTime::now()-optimization_start_wall_time_).toSec()))
          .add("original_solver_wall_ms",candidate_solver_wall*1000.)
          .add("repair_solver_wall_ms",repair_solver_wall_ms)
          .add("repair_solve_start_elapsed_s",repair_solve_start_elapsed_s)
          .add("native_check_before_ms",collision_repair_native_before_ms)
          .add("native_check_after_ms",collision_repair_native_after_ms)
          .add("repair_wall_ms",1000.*(ros::WallTime::now().toSec()-repair_started))
          .str("candidate_x_hash",std::to_string(delivery_candidate::hash(x)));
        dual_probe::emit("candidate_collision_repair_end",repaired_stats);
        probe.add("collision_repair_attempt",1);probe.add("collision_repair_native_pass",good_traj?1:0);
        diagnosis.add("collision_repair_attempt",1);diagnosis.add("collision_repair_native_pass",good_traj?1:0);
        diagnosis.add("collision_repair_lbfgs_result",result);
      }
    }'''

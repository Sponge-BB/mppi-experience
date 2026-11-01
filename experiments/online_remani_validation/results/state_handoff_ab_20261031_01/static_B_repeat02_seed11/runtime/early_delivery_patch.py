#!/usr/bin/env python3
"""Default-off accepted-iterate native gate for the isolated cheap child."""
from build_state_contract import Transformer


SESSION = r'''
    const bool early_delivery_enabled=early_delivery::requested();
    std::unique_ptr<early_delivery::Checker> early_delivery_checker;
    if(early_delivery_enabled)early_delivery_checker.reset(new early_delivery::Checker(
      piece_num_container_,singul_container_,iniState_container_,finState_container_,
      non_singul_v_,mobile_base_wheel_base_,mobile_base_wheel_radius_,max_wheel_omega_,max_wheel_alpha_));
    early_delivery::Session early_delivery_session(early_delivery_enabled,early_delivery_checker.get(),
      [this](){return (ros::WallTime::now()-optimization_start_wall_time_).toSec();},max_wall_time_,
      [this](const SingulTrajData& candidate){
        delivery_replay::RestoreFailure restore;
        early_delivery::NativeVerdict verdict;verdict.pass=IsTrajSafe(candidate);
        const auto* frame=failure_probe::active();verdict.reason=verdict.pass?"ok":(frame?frame->reason:"native_reject");
        return verdict;
      });
'''


REPORT = r'''
    // Emit buffered observations after solving; recorded monotonic timestamps
    // refer to the actual checks, not to these later output writes.
    double early_wheel_ms=0.,early_native_ms=0.;int early_native_count=0;
    for(const auto& check:early_delivery_session.checks){
      early_wheel_ms+=1000.*check.wheel_s;early_native_ms+=1000.*check.native_s;
      early_native_count+=check.native_checked?1:0;
      dual_probe::emit("early_delivery_check",dual_probe::Fields()
        .str("candidate_hash",std::to_string(check.candidate_hash)).add("accepted_iteration",check.accepted_iteration)
        .add("check_begin_mono_s",check.begin_mono_s).add("check_end_mono_s",check.end_mono_s)
        .add("elapsed_begin_ms",1000.*check.elapsed_begin_s).add("elapsed_end_ms",1000.*check.elapsed_end_s)
        .add("reconstruction_ms",1000.*check.reconstruction_s).add("wheel_check_ms",1000.*check.wheel_s)
        .add("native_gate_ms",1000.*check.native_s).add("wheel_pass",check.wheel_pass?1:0)
        .add("native_checked",check.native_checked?1:0).add("native_pass",check.native_pass?1:0)
        .add("selected_for_exit",check.selected?1:0).add("budget_expired",check.budget_expired?1:0)
        .str("reason",check.reason).str("native_reason",check.native_reason));
    }
    dual_probe::emit("early_delivery_summary",dual_probe::Fields()
      .add("enabled",early_delivery_enabled?1:0).add("selected",early_delivery_session.selected?1:0)
      .str("selected_hash",early_delivery_session.selected?std::to_string(delivery_candidate::hash(early_delivery_session.selected_x)):"")
      .add("check_count",early_delivery_session.checks.size()).add("native_check_count",early_native_count)
      .add("wheel_wall_ms",early_wheel_ms).add("native_wall_ms",early_native_ms)
      .add("elapsed_ms",1000.*(ros::WallTime::now()-optimization_start_wall_time_).toSec())
      .add("budget_s",max_wall_time_));
    if(early_delivery_session.selected){
      // Independent checking never touched the optimizer caches. Regenerate
      // from the certified accepted x so output samples cannot be stale trials.
      x=early_delivery_session.selected_x;
      std::vector<delivery_candidate::Part> early_parts;
      if(!delivery_candidate::decode(x,piece_num_container_,traj_dim_,iniState_container_,
           finState_container_,non_singul_v_,early_parts))throw std::runtime_error("early delivery selected decode");
      for(int i=0;i<traj_num_;++i){
        SnapOpt_container_[i].generate(early_parts[i].points,early_parts[i].times,early_parts[i].initial,early_parts[i].final);
        cps_container_[i].points=SnapOpt_container_[i].getInitConstrainPoints(cps_num_prePiece_);
      }
    }
'''


def transform_header(text):
    # The checker receives copied state plus a native predicate from a member
    # lambda; unlike cheap Context it needs no optimizer-private access.
    return text


def transform_optimizer(text):
    t=Transformer(text)
    t.after('#include "cheap_variable_scaling.hpp"','\n#include "early_delivery_gate.hpp"')
    t.before('    /* ---------- optimize ---------- */',SESSION)
    t.before('    probe.add("iterations",iter_num_);probe.add("lbfgs_result",result);',REPORT)
    t.after('    delivery_replay::progress(x,g,fx,step,k,ls);', '\n    const bool early_native_done=opt->force_stop_type_==DONT_STOP && early_delivery::accepted(x,k);')
    t.change('    return opt->force_stop_type_ != DONT_STOP;','    return early_native_done || opt->force_stop_type_ != DONT_STOP;')
    return t.finish()

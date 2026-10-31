#!/usr/bin/env python3
"""Add bounded, paused wheel-cause replay to a NEW candidate-audit child only."""
from build_state_contract import Transformer


def transform_optimizer(text):
    t = Transformer(text)
    t.after('#include "candidate_audit.hpp"', '\n#include "wheel_cause_replay.hpp"')
    t.change('      std::abs(max_wall_time_-.6)<1e-6 && native67_context &&',
             '      (std::abs(max_wall_time_-.6)<1e-6 || wheel_cause::running()) && native67_context &&')
    t.after('    const auto candidate_entry_esdf=grid_map_->getESDFObservationSeq();', r'''
    wheel_cause::Trial wheel_cause_trial(wheel_cause::running());
    if(wheel_cause_trial.enabled)wheel_cause_trial.capture=[&](){
      candidate_audit::Params p;
      p.wheel_base=mobile_base_wheel_base_;p.wheel_radius=mobile_base_wheel_radius_;
      p.wheel_limit=max_wheel_omega_;p.alpha_limit=max_wheel_alpha_;
      p.arm_radius=manipulator_thickness_;p.ground=ground_safe_dis_;
      p.base_safe_margin=safe_margin_;p.arm_safe_margin=safe_margin_mani_;
      p.joint_vel_limit=max_joint_vel_;p.joint_acc_limit=max_joint_acc_;
      return wheel_cause::capture(wheel_cause_trial,candidate_record,accepted_candidate,
        SnapOpt_container_,piece_num_container_,traj_dim_,iniState_container_,finState_container_,
        singul_container_,non_singul_v_,*mm_config_,*grid_map_,p,cps_num_prePiece_,dense_sample_resolution_,
        input_hash,candidate_entry_raw,candidate_entry_esdf,
        [&](const SingulTrajData& trajectory){return IsTrajSafe(trajectory);},
        [&](double squared_speed){double value=0.,gradient=0.;return smoothedLog(.1*.1-squared_speed,.001,value,gradient);});
    };
''')
    t.after('    const double candidate_solver_wall=(ros::WallTime::now()-optimization_start_wall_time_).toSec();',
            '\n    wheel_cause::solverFinished(result,int(force_stop_type_),candidate_solver_wall);')
    t.after('    good_traj = IsTrajSafe(singul_traj_data);', r'''
    wheel_cause::nativeGate(good_traj,diagnosis.reason,diagnosis.rejection_value,diagnosis.rejection_time);''')
    return t.finish()


def transform_finish(text):
    t = Transformer(text)
    t.change('    const bool audit_selected=candidate_record.enabled && !candidate_audit_used && !good_traj &&',
             '    const bool audit_selected=candidate_record.enabled && !wheel_cause::running() && !candidate_audit_used && !good_traj &&')
    t.before('      std::ofstream done(done_env);', '#include "wheel_cause_replay.inc"\n')
    return t.finish()


transform = transform_optimizer

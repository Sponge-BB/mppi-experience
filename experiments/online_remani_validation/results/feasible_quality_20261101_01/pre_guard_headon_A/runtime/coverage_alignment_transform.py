"""Minimal measured-epoch / new-candidate phase patch for the isolated A/B."""
from native_collision_runtime import replace_once


def transform_manager(text):
    text = replace_once(text, '#include "state_contract_trace.hpp"',
        '#include "state_contract_trace.hpp"\n#include "coverage_alignment.hpp"')
    return replace_once(text, '    if(have_local_traj){\n      double delta_replan_time',
        '    if(have_local_traj || coverage_alignment::active()){\n      double delta_replan_time')


def transform_fsm(text):
    text = replace_once(text, '#include "time_indexed_collision.hpp"',
        '#include "time_indexed_collision.hpp"\n#include "coverage_alignment.hpp"')
    anchor = '    state_contract_trace::Request state_contract_request(&dual_request.success);'
    text = replace_once(text, anchor, anchor + '\n'
        '    coverage_alignment::Request alignment_request(local_execution_owner_);')
    anchor = '    state_contract_trace::pvaj("state_contract_desired_boundary",desired_start_pt,desired_start_vel,'
    text = replace_once(text, anchor,
        '    if(coverage_alignment::active() && applied_raw_odom_)\n'
        '      desired_start_time=applied_raw_odom_->header.stamp.toSec();\n'
        '    dual_probe::emit("coverage_alignment_boundary",dual_probe::Fields()\n'
        '      .add("enabled",coverage_alignment::active()?"true":"false")\n'
        '      .add("measurement_epoch",applied_raw_odom_?applied_raw_odom_->header.stamp.toSec():0.)\n'
        '      .add("requested_epoch",desired_start_time).add("capture_now",ros::Time::now().toSec())\n'
        '      .add("boundary_x",desired_start_pt(0)).add("boundary_y",desired_start_pt(1))\n'
        '      .add("boundary_yaw",desired_start_yaw));\n' + anchor)
    text = replace_once(text,
        '      if (guarded_dynamic_stop_handoff && !candidate.singul_traj.empty())',
        '      if (!coverage_alignment::active() && guarded_dynamic_stop_handoff && !candidate.singul_traj.empty())')
    text = replace_once(text,
        '      else if (dynamic_repair && !candidate.singul_traj.empty())',
        '      else if (!coverage_alignment::active() && dynamic_repair && !candidate.singul_traj.empty())')
    text = replace_once(text,
        '      if (dynamic_point_flow_active_mode_)\n      {\n        const ros::Time validation_now = ros::Time::now();',
        '      if (dynamic_point_flow_active_mode_)\n      {\n'
        '        if(coverage_alignment::active())applyLatestOdom();\n'
        '        const ros::Time validation_now = ros::Time::now();')

    # Limit validator transformations to its body; other live TTC callers keep
    # their existing progress/time mapping and local execution checks.
    begin = text.index('  bool REMANIReplanFSM::validatePredictedDynamicTrajectory(')
    end = text.index('  void REMANIReplanFSM::publishDynamicPointFlowStatus(', begin)
    body = text[begin:end]
    anchor = '    auto reject = [&](const char* branch, double local_t, double wall_ahead,'
    body = replace_once(body, anchor,
        '    const bool phase_aligned=coverage_alignment::active();\n'
        '    const auto projected_phase=coverage_alignment::project(\n'
        '      traj,mm_state_pos_,mm_car_yaw_,std::max(0.05,global_plan_dynamic_check_time_step_));\n'
        '    const double trajectory_begin=phase_aligned?projected_phase.local_t:0.;\n'
        '    coverage_alignment::recordPhase(traj,mm_state_pos_,mm_car_yaw_,\n'
        '      applied_raw_odom_?applied_raw_odom_->header.stamp.toSec():0.,now.toSec(),projected_phase);\n' + anchor)
    anchor = '    const double observation_age =\n'
    body = replace_once(body, anchor,
        '    if(phase_aligned){\n'
        '      bool measured_query_valid=true;\n'
        '      const bool measured_collision=planner_manager_->mm_config_->checkPredictedDynamicObstacleCollision(\n'
        '        Eigen::Vector3d(mm_state_pos_(0),mm_state_pos_(1),mm_car_yaw_),\n'
        '        mm_state_pos_.tail(planner_manager_->mm_config_->getManiDof()),\n'
        '        now,snapshot,false,coll_type,measured_query_valid);\n'
        '      dual_probe::emit("candidate_measured_pose_check",dual_probe::Fields()\n'
        '        .add("query_epoch",now.toSec()).add("collision",measured_collision?"true":"false")\n'
        '        .add("query_valid",measured_query_valid?"true":"false").add("collision_type",coll_type));\n'
        '      if(measured_collision)return reject("measured_pose_collision",trajectory_begin,0.,0.,measured_query_valid,coll_type);\n'
        '      if(!measured_query_valid)return reject("measured_pose_query_invalid",trajectory_begin,0.,0.,false,coll_type);\n'
        '    }\n' + anchor)
    body = replace_once(body,
        '    const double start_delay = std::max(\n        0.0, traj.start_time - now.toSec());',
        '    const double start_delay = phase_aligned?0.:std::max(\n        0.0, traj.start_time - now.toSec());')
    body = replace_once(body,
        '      return reject("insufficient_horizon",0.,start_delay,0.,false,-1);',
        '      return reject("insufficient_horizon",trajectory_begin,start_delay,0.,false,-1);')
    body = replace_once(body,
        '    while (hold_wall <= hold_horizon + 1e-9)',
        '    while (!phase_aligned && hold_wall <= hold_horizon + 1e-9)')
    body = replace_once(body,
        '          (valid_wall_horizon - start_delay) * execution_scale);\n      double t = 0.0;',
        '          trajectory_begin+(valid_wall_horizon - start_delay) * execution_scale);\n      double t = trajectory_begin;')
    body = replace_once(body,
        '            start_delay + t / execution_scale;',
        '            start_delay + (t-trajectory_begin) / execution_scale;')
    return text[:begin] + body + text[end:]

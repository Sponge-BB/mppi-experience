#include <plan_manage/remani_replan_fsm.h>

#include <cmath>
#include <std_msgs/String.h>
#include <ros/serialization.h>
#include <limits>

namespace remani_planner
{
// Cooperative base-only SIL mode: execution safety is owned by the fast
// validated local rollout. The native polynomial remains a replanning guide.
template<class Message> std::string executionHex(const Message&message){
  uint32_t length=ros::serialization::serializationLength(message);
  std::vector<uint8_t> bytes(length);ros::serialization::OStream stream(bytes.data(),length);
  ros::serialization::serialize(stream,message);
  static const char digits[]="0123456789abcdef";std::string text; text.reserve(length*2);
  for(auto b:bytes){text.push_back(digits[b>>4]);text.push_back(digits[b&15]);}return text;
}

void REMANIReplanFSM::publishExecutionState(){
  if(!local_execution_owner_)return;
  std_msgs::String msg;msg.data=execution_goal_key_+"\n"+(have_target_?"1":"0")+"\n"+
    (exec_state_==EMERGENCY_STOP?"1":"0");
  if(msg.data!=last_execution_state_){execution_state_pub_.publish(msg);last_execution_state_=msg.data;}
}

void REMANIReplanFSM::localExecutionCallback(const nav_msgs::Path::ConstPtr&msg){
  local_execution_path_=*msg;
}

void REMANIReplanFSM::checkCooperativeExecution(){
  const ros::Time now=ros::Time::now();publishExecutionState();
  // Missing/expired local leases never authorize motion; the fast executor
  // already brakes without waiting for this (potentially busy) ROS thread.
  if(!have_target_||exec_state_!=EXEC_TRAJ||local_execution_path_.poses.size()<2||
     local_execution_path_.header.frame_id!="world"||
     (now-local_execution_path_.header.stamp).toSec()<-.05||
     (now-local_execution_path_.header.stamp).toSec()>.20)return;
  if(!last_cooperative_check_.isZero()&&(now-last_cooperative_check_).toSec()<.25)return;
  last_cooperative_check_=now;
  auto&info=planner_manager_->traj_container_.singul_traj_data;
  if(info.traj_id<=0)return;
  // A blocked GUIDE requests a slow refresh. It cannot cancel an accepted
  // local execution rollout or impose a speed cap on that different motion.
  double current=getCurrentTrajectoryTime(info,now);bool blocked=false;
  for(double t=current;t<std::min(info.duration,current+2.);t+=.10){
    int kind;if(planner_manager_->ploy_traj_opt_->checkCollision(info,t,kind)) {blocked=true;break;}
    if(planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(info,t,kind)){blocked=true;break;}
  }
  if(blocked&&(last_cooperative_replan_.isZero()||
      (now-last_cooperative_replan_).toSec()>.75)){
    last_cooperative_replan_=now;flag_relan_astar_=true;
    ROS_INFO("[LocalExecution] Native guide blocked; request refresh while the validated local rollout executes.");
    changeFSMExecState(REPLAN_TRAJ,"LOCAL_EXECUTION_GUIDE_REFRESH");
  }
}

  REMANIReplanFSM::~REMANIReplanFSM(){}
  void REMANIReplanFSM::init(ros::NodeHandle &nh)
  {
    nh.param("fsm/local_execution_owner",local_execution_owner_,false);
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;
    new_traj_fail_time_ = 0;

    /*  fsm param  */
    nh.param("fsm/target_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/replan_trajectory_time", replan_trajectory_time_, 0.0);
    nh.param("fsm/replan_feedback_position_threshold", replan_feedback_position_threshold_, 0.08);
    nh.param("fsm/replan_feedback_yaw_threshold", replan_feedback_yaw_threshold_, 0.15);
    nh.param("fsm/replan_feedback_arm_threshold", replan_feedback_arm_threshold_, 0.10);
    nh.param("fsm/goal_position_tolerance", goal_position_tolerance_, 0.10);
    nh.param("fsm/goal_yaw_tolerance", goal_yaw_tolerance_, 0.15);
    nh.param("fsm/goal_arm_tolerance", goal_arm_tolerance_, 0.12);
    nh.param("fsm/goal_yaw_alignment_timeout", goal_yaw_alignment_timeout_, 8.0);
    nh.param("fsm/actual_state_collision_check_interval",
             actual_state_collision_check_interval_, 0.05);
    nh.param("fsm/actual_base_emergency_margin",
             actual_base_emergency_margin_, 0.01);
    nh.param("fsm/esdf_freshness_wait_timeout",
             esdf_freshness_wait_timeout_, 0.75);
    nh.param("fsm/dynamic_local_replan_enable",
             dynamic_local_replan_enable_, true);
    nh.param("fsm/dynamic_replan_hold_arm",
             dynamic_replan_hold_arm_, true);
    nh.param("fsm/dynamic_replan_horizon",
             dynamic_replan_horizon_, 2.0);
    nh.param("fsm/dynamic_replan_handoff_time",
             dynamic_replan_handoff_time_, 0.30);
    nh.param("fsm/dynamic_astar_search_time",
             dynamic_astar_search_time_, 0.45);
    nh.param("fsm/dynamic_astar_replan_wall_time",
             dynamic_astar_replan_wall_time_, 0.60);
    nh.param("dynamic_point_flow/shadow_mode",
             dynamic_point_flow_shadow_mode_, true);
    nh.param("dynamic_point_flow/shadow_check_interval",
             dynamic_point_flow_shadow_check_interval_, 0.1);
    nh.param("dynamic_point_flow/active_mode",
             dynamic_point_flow_active_mode_, false);
    nh.param("dynamic_point_flow/active_replan",
             dynamic_point_flow_active_replan_, true);
    nh.param("dynamic_point_flow/fail_safe_on_unavailable",
             dynamic_point_flow_fail_safe_on_unavailable_, true);
    nh.param("dynamic_point_flow/active_confirm_frames",
             dynamic_point_flow_active_confirm_frames_, 2);
    nh.param("dynamic_point_flow/active_immediate_ttc",
             dynamic_point_flow_active_immediate_ttc_, 0.65);
    nh.param("dynamic_point_flow/active_emergency_ttc",
             dynamic_point_flow_active_emergency_ttc_, 0.45);
    nh.param("dynamic_point_flow/active_slowdown_ttc",
             dynamic_point_flow_active_slowdown_ttc_, 1.50);
    nh.param("dynamic_point_flow/active_clear_hold_time",
             dynamic_point_flow_active_clear_hold_time_, 0.30);
    nh.param("dynamic_point_flow/active_resume_scale",
             dynamic_point_flow_active_resume_scale_, 1.0);
    nh.param("fsm/global_plan_dynamic_lookahead_time", global_plan_dynamic_lookahead_time_, 2.0);
    nh.param("fsm/global_plan_dynamic_check_interval", global_plan_dynamic_check_interval_, 0.1);
    nh.param("fsm/global_plan_dynamic_check_time_step", global_plan_dynamic_check_time_step_, 0.05);
    nh.param("fsm/global_plan_static_check_time_step", global_plan_static_check_time_step_, 0.1);
    nh.param("fsm/global_plan_dynamic_lookahead_distance", global_plan_dynamic_lookahead_distance_, 2.0);
    nh.param("fsm/global_plan_dynamic_far_sample_distance", global_plan_dynamic_far_sample_distance_, 0.1);
    nh.param("fsm/global_plan_dynamic_immediate_confirm_distance", global_plan_dynamic_immediate_confirm_distance_, 0.8);
    nh.param("fsm/global_plan_dynamic_confirm_frames", global_plan_dynamic_confirm_frames_, 2);
    nh.param("fsm/global_plan_dynamic_confirm_timeout", global_plan_dynamic_confirm_timeout_, 0.6);
    nh.param("fsm/global_plan_dynamic_clear_hold_time", global_plan_dynamic_clear_hold_time_, 0.4);
    nh.param("fsm/global_plan_dynamic_replan_hold_time", global_plan_dynamic_replan_hold_time_, 0.6);
    nh.param("fsm/global_plan_dynamic_same_obstacle_radius", global_plan_dynamic_same_obstacle_radius_, 0.45);
    nh.param("fsm/global_plan_dynamic_slowdown_distance", global_plan_dynamic_slowdown_distance_, 2.0);
    nh.param("fsm/global_plan_dynamic_slowdown_min_speed", global_plan_dynamic_slowdown_min_speed_, 0.08);
    nh.param("fsm/global_plan_dynamic_slowdown_max_speed", global_plan_dynamic_slowdown_max_speed_, 0.20);
    nh.param("fsm/global_plan_dynamic_emergency_deceleration", global_plan_dynamic_emergency_deceleration_, 0.35);
    nh.param("fsm/global_plan_dynamic_emergency_reaction_time", global_plan_dynamic_emergency_reaction_time_, 0.15);
    nh.param("fsm/global_plan_dynamic_emergency_margin", global_plan_dynamic_emergency_margin_, 0.05);
    nh.param("fsm/global_plan_dynamic_emergency_min_distance", global_plan_dynamic_emergency_min_distance_, 0.15);
    nh.param("fsm/global_plan_dynamic_emergency_max_distance", global_plan_dynamic_emergency_max_distance_, 0.22);
    nh.param("fsm/controller_progress_timeout", controller_progress_timeout_, 0.5);
    nh.param("fsm/global_plan_max_failures", global_plan_max_failures_, 3);
    nh.param("fsm/global_plan_trial_times", global_plan_trial_times_, 3);
    nh.param("fsm/global_plan_max_wall_time", global_plan_max_wall_time_, 3.0);
    nh.param("fsm/rear_goal_plan_max_failures", rear_goal_plan_max_failures_, 3);
    nh.param("fsm/rear_goal_plan_trial_times", rear_goal_plan_trial_times_, 3);
    nh.param("fsm/rear_goal_plan_distance", rear_goal_plan_distance_, 0.5);
    nh.param("fsm/rear_goal_plan_cos", rear_goal_plan_cos_, -0.2);
    nh.param("fsm/time_for_gripper", time_for_gripper_, -1.0);
    nh.param("fsm/global_plan", global_plan_, false);
    if(global_plan_) planning_horizen_ = 1.0e3;

    global_plan_dynamic_lookahead_time_ = std::max(0.0, global_plan_dynamic_lookahead_time_);
    global_plan_dynamic_check_interval_ = std::max(0.01, global_plan_dynamic_check_interval_);
    global_plan_dynamic_check_time_step_ = std::max(0.01, global_plan_dynamic_check_time_step_);
    global_plan_static_check_time_step_ = std::max(0.01, global_plan_static_check_time_step_);
    global_plan_dynamic_lookahead_distance_ = std::max(0.0, global_plan_dynamic_lookahead_distance_);
    global_plan_dynamic_far_sample_distance_ = std::max(0.02, global_plan_dynamic_far_sample_distance_);
    global_plan_dynamic_immediate_confirm_distance_ = std::max(0.0, global_plan_dynamic_immediate_confirm_distance_);
    global_plan_dynamic_confirm_frames_ = std::max(1, global_plan_dynamic_confirm_frames_);
    global_plan_dynamic_confirm_timeout_ = std::max(
        global_plan_dynamic_check_interval_, global_plan_dynamic_confirm_timeout_);
    global_plan_dynamic_clear_hold_time_ = std::max(0.0, global_plan_dynamic_clear_hold_time_);
    global_plan_dynamic_replan_hold_time_ = std::max(0.0, global_plan_dynamic_replan_hold_time_);
    global_plan_dynamic_same_obstacle_radius_ = std::max(0.0, global_plan_dynamic_same_obstacle_radius_);
    global_plan_dynamic_slowdown_distance_ = std::max(0.0, global_plan_dynamic_slowdown_distance_);
    global_plan_dynamic_slowdown_min_speed_ = std::max(0.0, global_plan_dynamic_slowdown_min_speed_);
    global_plan_dynamic_slowdown_max_speed_ = std::max(global_plan_dynamic_slowdown_min_speed_,
                                                       global_plan_dynamic_slowdown_max_speed_);
    global_plan_dynamic_emergency_deceleration_ = std::max(0.05, global_plan_dynamic_emergency_deceleration_);
    global_plan_dynamic_emergency_reaction_time_ = std::max(0.0, global_plan_dynamic_emergency_reaction_time_);
    global_plan_dynamic_emergency_margin_ = std::max(0.0, global_plan_dynamic_emergency_margin_);
    global_plan_dynamic_emergency_min_distance_ = std::max(0.0, global_plan_dynamic_emergency_min_distance_);
    global_plan_dynamic_emergency_max_distance_ = std::max(global_plan_dynamic_emergency_min_distance_,
                                                           global_plan_dynamic_emergency_max_distance_);
    controller_progress_timeout_ = std::max(0.1, controller_progress_timeout_);
    rear_goal_plan_max_failures_ = std::max(1, rear_goal_plan_max_failures_);
    rear_goal_plan_trial_times_ = std::max(1, rear_goal_plan_trial_times_);
    rear_goal_plan_distance_ = std::max(0.0, rear_goal_plan_distance_);
    rear_goal_plan_cos_ = std::max(-1.0, std::min(1.0, rear_goal_plan_cos_));
    replan_feedback_position_threshold_ = std::max(0.0, replan_feedback_position_threshold_);
    replan_feedback_yaw_threshold_ = std::max(0.0, replan_feedback_yaw_threshold_);
    replan_feedback_arm_threshold_ = std::max(0.0, replan_feedback_arm_threshold_);
    goal_position_tolerance_ = std::max(0.01, goal_position_tolerance_);
    goal_yaw_tolerance_ = std::max(0.01, goal_yaw_tolerance_);
    goal_arm_tolerance_ = std::max(0.01, goal_arm_tolerance_);
    goal_yaw_alignment_timeout_ = std::max(0.5, goal_yaw_alignment_timeout_);
    actual_state_collision_check_interval_ =
        std::max(0.01, actual_state_collision_check_interval_);
    actual_base_emergency_margin_ = std::max(0.0, actual_base_emergency_margin_);
    esdf_freshness_wait_timeout_ =
        std::max(0.10, esdf_freshness_wait_timeout_);
    dynamic_replan_horizon_ = std::max(0.5, dynamic_replan_horizon_);
    dynamic_replan_handoff_time_ =
        std::max(0.05, dynamic_replan_handoff_time_);
    dynamic_astar_search_time_ =
        std::max(0.05, dynamic_astar_search_time_);
    dynamic_astar_replan_wall_time_ =
        std::max(0.20, dynamic_astar_replan_wall_time_);
    dynamic_point_flow_shadow_check_interval_ =
        std::max(0.02, dynamic_point_flow_shadow_check_interval_);
    dynamic_point_flow_active_confirm_frames_ =
        std::max(1, dynamic_point_flow_active_confirm_frames_);
    dynamic_point_flow_active_emergency_ttc_ =
        std::max(0.0, dynamic_point_flow_active_emergency_ttc_);
    dynamic_point_flow_active_immediate_ttc_ = std::max(
        dynamic_point_flow_active_emergency_ttc_,
        dynamic_point_flow_active_immediate_ttc_);
    dynamic_point_flow_active_slowdown_ttc_ = std::max(
        dynamic_point_flow_active_immediate_ttc_,
        dynamic_point_flow_active_slowdown_ttc_);
    dynamic_point_flow_active_clear_hold_time_ =
        std::max(0.0, dynamic_point_flow_active_clear_hold_time_);
    dynamic_point_flow_active_resume_scale_ = std::max(
        0.1, std::min(1.0, dynamic_point_flow_active_resume_scale_));
    global_plan_max_wall_time_ = std::max(0.5, global_plan_max_wall_time_);
    last_global_plan_dynamic_check_time_ = ros::Time(0);
    last_dynamic_point_flow_shadow_check_time_ = ros::Time(0);
    last_actual_state_collision_check_time_ = ros::Time(0);
    last_dynamic_collision_time_ = ros::Time(0);
    last_dynamic_seen_time_ = ros::Time(0);
    dynamic_replan_hold_until_ = ros::Time(0);
    next_dynamic_replan_retry_time_ = ros::Time(0);
    last_dynamic_observation_seq_ = 0;
    last_published_dynamic_validation_seq_ = 0;
    last_published_point_flow_validation_seq_ = 0;
    dynamic_event_required_esdf_seq_ = 0;
    planning_boundary_required_esdf_seq_ = 0;
    dynamic_esdf_wait_start_ = ros::SteadyTime();
    dynamic_esdf_sync_fallback_requested_ = false;
    dynamic_collision_confirm_count_ = 0;
    current_safety_speed_limit_ = std::numeric_limits<double>::quiet_NaN();
    last_dynamic_confirmation_pos_.setZero();
    last_dynamic_replan_collision_pos_.setZero();
    last_dynamic_replan_collision_type_ = -1;
    last_dynamic_replan_collision_from_point_flow_ = false;
    has_last_dynamic_confirmation_pos_ = false;
    has_last_dynamic_replan_collision_ = false;
    dynamic_safety_speed_limit_active_ = false;
    dynamic_replan_retry_pending_ = false;
    dynamic_event_astar_attempted_ = false;
    dynamic_event_from_point_flow_ = false;
    dynamic_point_flow_collision_present_ = false;
    dynamic_point_flow_hazard_active_ = false;
    dynamic_point_flow_collision_confirmed_ = false;
    dynamic_point_flow_input_unavailable_ = false;
    have_published_trajectory_ = false;
    dynamic_event_replan_attempts_ = 0;
    last_dynamic_replan_attempt_generation_ = 0;
    dynamic_point_flow_confirm_count_ = 0;
    last_dynamic_point_flow_sequence_ = 0;
    last_dynamic_point_flow_collision_time_ = ros::Time(0);
    dynamic_point_flow_clear_since_ = ros::Time(0);
    dynamic_event_first_detected_steady_ = ros::SteadyTime();
    last_dynamic_point_flow_collision_position_.setZero();
    last_dynamic_point_flow_evaluation_ = DynamicPointFlowEvaluation();
    have_controller_progress_ = false;
    tracking_fault_active_ = false;
    tracking_safety_stop_active_ = false;
    tracking_recovery_pending_ = false;
    tracking_recovery_candidate_published_ = false;
    controller_trajectory_start_time_ = 0.0;
    controller_trajectory_time_ = 0.0;
    controller_trajectory_scale_ = 1.0;
    controller_progress_stamp_ = ros::Time(0);
    goal_yaw_alignment_start_time_ = ros::Time(0);

    nh.param("mm/mobile_base_dof", mobile_base_dim_, -1);
    nh.param("mm/manipulator_dof", manipulator_dim_, -1);
    nh.param("mm/mobile_base_non_singul_vel", mobile_base_non_singul_vel_, -1.0);

    // Default arm stand-by comes from fsm/init_state. If that parameter is
    // missing, fall back to the first joint_state message.
    default_arm_from_config_ = false;
    has_default_arm_ = false;
    default_arm_config_ = Eigen::VectorXd::Zero(manipulator_dim_);
    std::vector<double> init_state_cfg;
    if (nh.getParam("fsm/init_state", init_state_cfg) &&
        init_state_cfg.size() >= static_cast<size_t>(mobile_base_dim_ + manipulator_dim_)) {
      for (int i = 0; i < manipulator_dim_; ++i) {
        default_arm_config_(i) = init_state_cfg[mobile_base_dim_ + i] * M_PI / 180.0;
      }
      has_default_arm_ = true;
      default_arm_from_config_ = true;
      ROS_INFO_STREAM("[FSM] Arm stand-by pose from fsm/init_state: "
                      << default_arm_config_.transpose());
    } else {
      ROS_WARN("[FSM] fsm/init_state is missing or too short; arm stand-by will use first joint_state.");
    }
    nh.param("fsm/manual_restore_standby_after_goal", manual_restore_standby_after_goal_, true);
    nh.param("fsm/manual_restore_tolerance", manual_restore_tolerance_, 0.05);
    nh.param("fsm/manual_use_work_arm_waypoint", manual_use_work_arm_waypoint_, false);
    nh.param("fsm/manual_target_end_arm_standby", manual_target_end_arm_standby_, true);
    nh.param("fsm/manual_work_arm_waypoint_id", manual_work_arm_waypoint_id_, 0);
    manual_restore_pending_ = false;
    manual_restore_running_ = false;
    manual_restore_tolerance_ = std::max(0.0, manual_restore_tolerance_);
    

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;

    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);

    gripper_flag_ = true;

    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    nh.param("fsm/waypoint_num", waypoint_num_, -1);

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      nh.param("fsm/waypoint" + to_string(i) + "_yaw", yaw_temp, -1.0);
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      nh.param("fsm/waypoint" + to_string(i) + "_gripper_close", gripper_close, true);
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp;
      nh.getParam("fsm/waypoint" + to_string(i), waypoints_temp);
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;
      }
      waypoints_.push_back(wp);
    }
    
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;
    map_state_ = 0;

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    if (dynamic_point_flow_active_mode_ &&
        !planner_manager_->grid_map_->dynamicPointFlowEnabled())
    {
      ROS_ERROR("[DynamicPointFlow][active] active_mode requested but the point-flow input is disabled; active mode will fail safe.");
    }
    else if (dynamic_point_flow_active_mode_ &&
             dynamic_point_flow_active_replan_ &&
             !planner_manager_->grid_map_->dynamicPointPlanningOverlayEnabled())
    {
      ROS_ERROR("[DynamicPointFlow][active] active_replan requires the swept planning overlay; disabling point-flow replanning and keeping the fail-safe supervisor active.");
      dynamic_point_flow_active_replan_ = false;
    }
    else if (dynamic_point_flow_active_mode_)
    {
      ROS_WARN("[DynamicPointFlow][active] Experimental predictive avoidance is ACTIVE. Existing occupancy/ESDF safety remains enabled.");
    }
    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.01), &REMANIReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &REMANIReplanFSM::mmCarOdomCallback, this);
    joint_state_sub_ = nh.subscribe("joint_state", 1, &REMANIReplanFSM::mmManiOdomCallback, this);
    gripper_state_sub_ = nh.subscribe("gripper_state", 1, &REMANIReplanFSM::gripperCallback, this);
    trajectory_progress_sub_ = nh.subscribe(
        "/mm_controller_node/trajectory_progress", 1,
        &REMANIReplanFSM::trajectoryProgressCallback, this);
    tracking_fault_sub_ = nh.subscribe(
        "/tracking_fault", 1, &REMANIReplanFSM::trackingFaultCallback, this);

    poly_traj_pub_ = nh.advertise<quadrotor_msgs::PolynomialTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    gripper_cmd_pub_ = nh.advertise<std_msgs::Bool>("gripper_cmd", 100);
    map_state_pub_ = nh.advertise<std_msgs::Int32>("/map_generator/map_state", 100);

    start_pub_ = nh.advertise<std_msgs::Bool>("planning/start", 1);
    reached_pub_ = nh.advertise<std_msgs::Bool>("planning/finish", 1);
    safety_speed_limit_pub_ = nh.advertise<std_msgs::Float64>("/planning/safety_speed_limit", 1, true);
    if(local_execution_owner_){
      execution_state_pub_=nh.advertise<std_msgs::String>("/planning/execution_state",1,true);
      execution_reference_pub_=nh.advertise<std_msgs::String>("/planning/execution_reference",10);
      local_execution_sub_=nh.subscribe("/planning/local_execution_rollout",1,&REMANIReplanFSM::localExecutionCallback,this);
      publishExecutionState();
    }
    dynamic_point_flow_state_pub_ = nh.advertise<std_msgs::Int32>(
        "/planning/dynamic_point_flow_state", 1, true);
    dynamic_point_flow_ttc_pub_ = nh.advertise<std_msgs::Float64>(
        "/planning/dynamic_point_flow_ttc", 1, true);
    publishSafetySpeedLimit(-1.0);
    publishDynamicPointFlowStatus(
        dynamic_point_flow_active_mode_ ? -1 : 0, -2.0);
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &REMANIReplanFSM::waypointCallback, this);
    
  }

  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();
    }

    switch (exec_state_){
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // return;
      }
      if (!planner_manager_->grid_map_->mapReadyForPlanning()){
        ROS_INFO_THROTTLE(1.0, "[FSM] Waiting for the static global-map ESDF before accepting goals.");
        goto force_return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // return;
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      // Global/recovery planning freezes one raw-map freshness boundary when
      // the attempt starts. Waiting for it blocks only candidate generation;
      // it does not create a zero-speed owner.
      if (!waitForRequiredESDF(
              "GEN_NEW_TRAJ", /*capture_planning_boundary=*/true))
        goto force_return;

      if(try_plan_after_emergency_){
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      // std::cout << "gen new traj 1\n";
      have_local_traj_ = false;
      const bool rear_distant_goal = isRearDistantGoal();
      const int trial_times = rear_distant_goal
          ? rear_goal_plan_trial_times_
          : global_plan_trial_times_;
      const int max_failures = rear_distant_goal
          ? rear_goal_plan_max_failures_
          : global_plan_max_failures_;
      bool success = planFromGlobalTraj(trial_times);
      // std::cout << "gen new traj 2\n";
      if (success){
        new_traj_fail_time_ = 0;
        // planFromGlobalTraj() returns true only after callReboundReplan()
        // validates and publishes a candidate.  A tracker-owned stop may use
        // that candidate for alignment, but it may resume only after the
        // tracker has also reported recovery.  This prevents a false message
        // alone from restarting the rejected old trajectory.
        if (tracking_safety_stop_active_)
        {
          tracking_recovery_candidate_published_ = true;
          if (!tracking_fault_active_)
          {
            tracking_safety_stop_active_ = false;
            tracking_recovery_candidate_published_ = false;
            ROS_INFO("[FSM] Validated a recovery trajectory after tracker alignment; release the tracking-owned stop.");
          }
          else
          {
            ROS_WARN_THROTTLE(
                1.0,
                "[FSM] Recovery trajectory is ready, but the tracker is not aligned yet; keep zero speed.");
          }
        }
        tracking_recovery_pending_ = false;
        if (dynamic_safety_speed_limit_active_ &&
            ((dynamic_point_flow_active_mode_ &&
              dynamic_point_flow_hazard_active_) ||
             dynamic_event_required_esdf_seq_ > 0))
        {
          ROS_WARN_THROTTLE(
              1.0,
              "[FSM] Keep the obstacle speed guard until the new trajectory "
              "is independently cleared by live safety checks.");
        }
        else if (dynamic_safety_speed_limit_active_)
        {
          clearDynamicSafetySpeedLimit(ros::Time::now(), true);
        }
        else
          publishSafetySpeedLimit(
              tracking_safety_stop_active_ ? 0.0 : -1.0);
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        try_plan_after_emergency_ = false;
      }
      else
      {
        ++new_traj_fail_time_;
        ROS_WARN("[FSM] Failed to generate new trajectory (%d/%d).",
                 new_traj_fail_time_, max_failures);
        if (new_traj_fail_time_ >= max_failures)
        {
          new_traj_fail_time_ = 0;
          have_target_ = false;
          have_new_target_ = false;
          have_local_traj_ = false;
          manual_restore_pending_ = false;
          manual_restore_running_ = false;
          tracking_recovery_pending_ = false;
          ROS_ERROR("[FSM] Give up current target and return to WAIT_TARGET. Please send a new goal.");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // Do not allow retry timing, point-flow ownership, or A* selection to
      // bypass a raw occupancy generation that has not reached the ESDF yet.
      if (!waitForRequiredESDF("REPLAN_TRAJ"))
        goto force_return;

      if (dynamic_replan_retry_pending_) {
        const ros::Time retry_now = ros::Time::now();
        if (retry_now < next_dynamic_replan_retry_time_)
          goto force_return;

        const std::uint64_t available_generation =
            dynamic_event_from_point_flow_ ?
                planner_manager_->grid_map_->getDynamicPointSnapshot(
                    retry_now).sequence :
                planner_manager_->grid_map_->getDynamicObservationSeq();
        if (available_generation <=
            last_dynamic_replan_attempt_generation_)
        {
          ROS_DEBUG_THROTTLE(
              0.5,
              "[FSM] Dynamic repair waits for a newer immutable observation (last=%lu, available=%lu).",
              static_cast<unsigned long>(
                  last_dynamic_replan_attempt_generation_),
              static_cast<unsigned long>(available_generation));
          goto force_return;
        }

        // A raw event waits only for its immutable collision generation. A
        // point-flow event plans from its frozen swept overlay and must not
        // chase the newest raw LiDAR generation at every retry.
        if (!waitForRequiredESDF("DYNAMIC_ASTAR_REPLAN"))
          goto force_return;

        dynamic_event_astar_attempted_ = true;
        last_dynamic_replan_attempt_generation_ = available_generation;
        ++dynamic_event_replan_attempts_;
        ROS_INFO("[FSM] Run short-horizon A* dynamic repair from observation generation %lu (attempt %d).",
                 static_cast<unsigned long>(available_generation),
                 dynamic_event_replan_attempts_);
        if (planFromLocalTraj(
                true, false, dynamic_astar_replan_wall_time_,
                /*dynamic_repair=*/true)) {
          replan_fail_time_ = 0;
          dynamic_replan_retry_pending_ = false;
          dynamic_replan_hold_until_ =
              ros::Time::now() + ros::Duration(global_plan_dynamic_replan_hold_time_);
          next_dynamic_replan_retry_time_ = dynamic_replan_hold_until_;
          flag_relan_astar_ = false;
          changeFSMExecState(EXEC_TRAJ, "DYNAMIC_ASTAR");
        } else {
          ++replan_fail_time_;
          // Keep only one pending request. Retry after a newer observation and
          // a bounded interval; the independent TTC/braking safety owner will
          // stop the robot if the situation becomes genuinely urgent.
          dynamic_replan_retry_pending_ = true;
          next_dynamic_replan_retry_time_ =
              ros::Time::now() +
              ros::Duration(global_plan_dynamic_replan_hold_time_);
          dynamic_replan_hold_until_ = next_dynamic_replan_retry_time_;
          ROS_WARN("[FSM] Dynamic A* repair failed; retain TTC-derived speed and retry from a newer observation after %.2f s.",
                   global_plan_dynamic_replan_hold_time_);
          changeFSMExecState(REPLAN_TRAJ, "DYNAMIC_ASTAR_RETRY");
        }
        break;
      }

      if (!waitForRequiredESDF(
              "ORDINARY_LOCAL_REPLAN",
              /*capture_planning_boundary=*/true))
        goto force_return;

      if(planFromLocalTraj(flag_relan_astar_)){
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        if((ros::Time::now() - t_last_Astar_ ).toSec() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;
          t_last_Astar_ = ros::Time::now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;
        flag_relan_astar_ = true;
        t_last_Astar_ = ros::Time::now();
        if(replan_fail_time_ >= 20){
          replan_fail_time_ = 0;
          ROS_ERROR("[FSM]:REPLAN fail over 20 times!!!");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      break;
    }
    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      const ros::Time now = ros::Time::now();
      double t_cur = getCurrentTrajectoryTime(*info, now);
      const bool trajectory_finished = t_cur >= info->duration - 1e-2;
      bool need_to_plan_next = trajectory_finished;
      bool need_to_gripper = trajectory_finished;
      t_cur = min(info->duration, t_cur);

      Eigen::VectorXd pos = info->getPos(t_cur);
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_);
      const double actual_goal_position_error =
          (mm_state_pos_.head(2) - end_pt_.head(2)).norm();
      const double actual_goal_yaw_error =
          fabs(atan2(sin(mm_car_yaw_ - end_yaw_), cos(mm_car_yaw_ - end_yaw_)));
      const double actual_goal_arm_error =
          (mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_) -
           end_pt_.segment(mobile_base_dim_, manipulator_dim_)).cwiseAbs().maxCoeff();
      const bool actual_position_at_goal =
          actual_goal_position_error <= goal_position_tolerance_;
      const bool actual_yaw_at_goal =
          actual_goal_yaw_error <= goal_yaw_tolerance_;
      const bool actual_arm_at_goal =
          actual_goal_arm_error <= goal_arm_tolerance_;
      const bool actual_position_and_arm_at_goal =
          actual_position_at_goal && actual_arm_at_goal;
      bool accept_manual_yaw_timeout = false;

      if(trajectory_finished && touch_the_goal &&
         actual_position_and_arm_at_goal && !actual_yaw_at_goal){
        if(goal_yaw_alignment_start_time_.isZero())
          goal_yaw_alignment_start_time_ = now;
        const double alignment_elapsed =
            (now - goal_yaw_alignment_start_time_).toSec();
        accept_manual_yaw_timeout =
            target_type_ == TARGET_TYPE::MANUAL_TARGET &&
            alignment_elapsed >= goal_yaw_alignment_timeout_;
        if(!accept_manual_yaw_timeout){
          ROS_INFO_THROTTLE(
              1.0,
              "[FSM] Position and arm reached; wait for bounded final yaw alignment "
              "(error=%.3f rad, %.1f/%.1f s).",
              actual_goal_yaw_error, alignment_elapsed,
              goal_yaw_alignment_timeout_);
          break;
        }
        ROS_WARN_THROTTLE(
            1.0,
            "[FSM] Manual goal yaw alignment timed out at %.3f rad; accept the "
            "position goal without another whole-body replan.",
            actual_goal_yaw_error);
      }else{
        goal_yaw_alignment_start_time_ = ros::Time(0);
      }
      const bool actual_at_goal =
          actual_position_and_arm_at_goal &&
          (actual_yaw_at_goal || accept_manual_yaw_timeout);

      if((target_type_ == TARGET_TYPE::PRESET_TARGET) && close_to_no_replan_thresh){
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
          gripper_flag_ = true;
        }else if(need_to_gripper && gripper_flag_){
          ++map_state_;
          std_msgs::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_];
          gripper_cmd_pub_.publish(gripper_cmd);
          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          ROS_INFO(gripper_cmd_str.c_str());
          std_msgs::Int32 map_state;
          map_state.data = map_state_;
          gripper_flag_ = false;
        }
      }else if(target_type_ == TARGET_TYPE::MANUAL_TARGET && trajectory_finished &&
               touch_the_goal && actual_at_goal){
        const double arm_standby_error = has_default_arm_ ?
            (mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_) - default_arm_config_).norm() : 0.0;
        const bool need_manual_restore =
            manual_restore_standby_after_goal_ && has_default_arm_ && !manual_restore_running_ &&
            arm_standby_error > manual_restore_tolerance_;

        if(need_manual_restore){
          manual_restore_pending_ = false;
          manual_restore_running_ = true;

          Eigen::VectorXd restore_pt = end_pt_;
          restore_pt.segment(mobile_base_dim_, manipulator_dim_) = default_arm_config_;
          const double restore_yaw = end_yaw_;

          ROS_INFO("[FSM] Manual target reached. Planning arm restore to stand-by pose. arm_error=%.4f",
                   arm_standby_error);
          if(planNextWaypoint(restore_pt, restore_yaw)){
            goto force_return;
          }

          manual_restore_running_ = false;
          ROS_WARN("[FSM] Failed to plan arm restore trajectory. Mark manual target as finished.");
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }else{
          manual_restore_running_ = false;
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
      }else if(trajectory_finished && touch_the_goal && actual_at_goal){
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");
          std_msgs::Bool msg;
          msg.data = true;
          reached_pub_.publish(msg);
          goto force_return;
        }
      }else if(trajectory_finished && !touch_the_goal){
        // A bounded dynamic A* repair intentionally ends at a point on the
        // global reference rather than at the final goal.  Global mode used to
        // assume every local trajectory reached end_pt_, so without this
        // explicit continuation it would remain forever in EXEC_TRAJ with a
        // finished polynomial. Re-enter through the measured-state path; any
        // live hazard/freshness guard remains authoritative in GEN_NEW_TRAJ.
        ROS_INFO("[FSM] Dynamic local segment completed; continue toward the global goal from feedback.");
        have_local_traj_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "LOCAL_SEGMENT_DONE");
        goto force_return;
      }else if(trajectory_finished && touch_the_goal && !actual_at_goal){
        ROS_WARN_THROTTLE(
            1.0,
            "[FSM] Reference trajectory ended before the real robot reached the goal "
            "(base=%.3f m, yaw=%.3f rad, arm=%.3f rad); replan from feedback.",
            actual_goal_position_error, actual_goal_yaw_error, actual_goal_arm_error);
        have_local_traj_ = false;
        changeFSMExecState(GEN_NEW_TRAJ, "GOAL_FEEDBACK");
        goto force_return;
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }
    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){ // Avoiding repeated calls
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_);
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          // A safety-caused stop snapshots the newest raw observation at this
          // recovery boundary. A tracker-caused stop takes the independent
          // branch below; GEN_NEW_TRAJ still captures a normal planning ESDF
          // boundary, but no false dynamic-obstacle event is manufactured.
          if (tracking_recovery_pending_)
          {
            // Tracking error is not obstacle evidence.  Keep its independent
            // zero-speed owner, then replan from odometry without fabricating
            // a dynamic event whose last-seen time can never be cleared.
            publishSafetySpeedLimit(0.0);
          }
          else
          {
            applyDynamicSafetySpeedLimit(0.0);
            raiseDynamicEventRequiredESDFSeq(
                planner_manager_->grid_map_->getDynamicObservationSeq());
          }
          try_plan_after_emergency_ = true;
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
      }

      flag_escape_emergency_ = false;

      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    publishExecutionState();
    exec_timer_.start();
  }

  void REMANIReplanFSM::trajectoryProgressCallback(
      const geometry_msgs::Vector3Stamped::ConstPtr &msg)
  {
    controller_trajectory_start_time_ = msg->vector.x;
    controller_trajectory_time_ = std::max(0.0, msg->vector.y);
    controller_trajectory_scale_ =
        std::max(0.0, std::min(1.0, msg->vector.z));
    controller_progress_stamp_ = msg->header.stamp;
    have_controller_progress_ = controller_trajectory_start_time_ > 0.0;
  }

  void REMANIReplanFSM::trackingFaultCallback(const std_msgs::Bool::ConstPtr &msg)
  {
    if(local_execution_owner_){tracking_fault_active_=msg->data;return;}
    if (!msg->data) {
      tracking_fault_active_ = false;

      // Do not resume the rejected old trajectory.  The stop is released only
      // when both independent conditions hold: the planner has published a
      // validated recovery candidate and the tracker is aligned to it.
      if (tracking_safety_stop_active_ &&
          tracking_recovery_candidate_published_ &&
          exec_state_ == EXEC_TRAJ)
      {
        tracking_safety_stop_active_ = false;
        tracking_recovery_candidate_published_ = false;
        if (!dynamic_safety_speed_limit_active_ &&
            !hasUnmetRequiredESDF())
        {
          publishSafetySpeedLimit(-1.0);
          ROS_INFO("[FSM] Tracker aligned to the validated recovery trajectory; release the tracking-owned stop.");
        }
        else
        {
          ROS_INFO("[FSM] Tracker recovered, but another safety owner retains the speed guard.");
        }
      }
      return;
    }
    if (tracking_fault_active_)
      return;

    tracking_fault_active_ = true;
    tracking_safety_stop_active_ = true;
    tracking_recovery_candidate_published_ = false;
    publishSafetySpeedLimit(0.0);
    if (have_target_ && exec_state_ != EMERGENCY_STOP) {
      tracking_recovery_pending_ = true;
      ROS_ERROR("[FSM] Tracker reported excessive trajectory error; abort old trajectory and replan from odometry.");
      changeFSMExecState(EMERGENCY_STOP, "TRACKING");
    }
  }

  double REMANIReplanFSM::getCurrentTrajectoryTime(
      const SingulTrajData &traj, const ros::Time &now) const
  {
    const bool progress_matches =
        have_controller_progress_ &&
        std::fabs(controller_trajectory_start_time_ - traj.start_time) < 1e-3 &&
        !controller_progress_stamp_.isZero();
    if (progress_matches) {
      const double age = std::max(0.0, (now - controller_progress_stamp_).toSec());
      if (age <= controller_progress_timeout_) {
        return std::max(
            0.0, std::min(
                traj.duration,
                controller_trajectory_time_ + age * controller_trajectory_scale_));
      }
    }

    return std::max(
        0.0, std::min(traj.duration, now.toSec() - traj.start_time));
  }

  double REMANIReplanFSM::getCurrentTrajectoryScale(
      const SingulTrajData &traj, const ros::Time &now) const
  {
    const bool progress_matches =
        have_controller_progress_ &&
        std::fabs(controller_trajectory_start_time_ - traj.start_time) < 1e-3 &&
        !controller_progress_stamp_.isZero() &&
        (now - controller_progress_stamp_).toSec() <= controller_progress_timeout_;
    return progress_matches ? controller_trajectory_scale_ : 1.0;
  }

  void REMANIReplanFSM::publishSafetySpeedLimit(double speed_limit)
  {
    const double normalized_limit = speed_limit < 0.0 ? -1.0 : std::max(0.0, speed_limit);
    if (std::isfinite(current_safety_speed_limit_) &&
        std::fabs(current_safety_speed_limit_ - normalized_limit) < 1e-3)
      return;

    std_msgs::Float64 msg;
    msg.data = normalized_limit;
    safety_speed_limit_pub_.publish(msg);
    current_safety_speed_limit_ = normalized_limit;
  }

  void REMANIReplanFSM::applyDynamicSafetySpeedLimit(double speed_limit)
  {
    double limited_speed = std::max(0.0, speed_limit);
    if (dynamic_safety_speed_limit_active_ &&
        std::isfinite(current_safety_speed_limit_) &&
        current_safety_speed_limit_ >= 0.0) {
      limited_speed = std::min(limited_speed, current_safety_speed_limit_);
    }

    dynamic_safety_speed_limit_active_ = true;
    publishSafetySpeedLimit(limited_speed);
  }

  void REMANIReplanFSM::latchDynamicEventRequiredESDFSeq(
      std::uint64_t observation_seq)
  {
    // The requirement is immutable for one obstacle event. Updating it for
    // every incoming point-cloud frame would make an asynchronous ESDF chase a
    // moving 10 Hz target and could prevent replanning indefinitely.
    if (observation_seq == 0 || dynamic_event_required_esdf_seq_ != 0)
      return;

    dynamic_event_required_esdf_seq_ = observation_seq;
    dynamic_esdf_wait_start_ = ros::SteadyTime();
    dynamic_esdf_sync_fallback_requested_ = false;
    ROS_INFO("[FSM] Latched collision-event ESDF requirement at seq=%lu.",
             static_cast<unsigned long>(dynamic_event_required_esdf_seq_));
  }

  void REMANIReplanFSM::raiseDynamicEventRequiredESDFSeq(
      std::uint64_t observation_seq)
  {
    if (observation_seq <= dynamic_event_required_esdf_seq_)
      return;

    dynamic_event_required_esdf_seq_ = observation_seq;
    dynamic_esdf_wait_start_ = ros::SteadyTime();
    // Preserve a prior fallback request for this event. The GridMap API is
    // idempotent, but the FSM still promises to invoke it only once before an
    // explicit event reset.
    ROS_INFO("[FSM] Raised stopped-state ESDF requirement to seq=%lu.",
             static_cast<unsigned long>(dynamic_event_required_esdf_seq_));
  }

  bool REMANIReplanFSM::hasUnmetRequiredESDF() const
  {
    const std::uint64_t required_seq = std::max(
        dynamic_event_required_esdf_seq_,
        planning_boundary_required_esdf_seq_);
    return required_seq > 0 &&
        planner_manager_->grid_map_->getESDFObservationSeq() < required_seq;
  }

  bool REMANIReplanFSM::waitForRequiredESDF(
      const char *planning_context, bool capture_planning_boundary)
  {
    const auto map = planner_manager_->grid_map_;
    const std::uint64_t esdf_observation_seq =
        map->getESDFObservationSeq();

    // A raw collision event deliberately owns an immutable generation. Every
    // actual planning attempt independently freezes the newest generation at
    // its callback boundary as well. This prevents an older event generation
    // from authorizing optimization after newer occupancy has already arrived,
    // while still avoiding a moving target: the boundary is captured only once
    // per attempt and later LiDAR frames do not move it. The final live-map
    // validation covers frames which arrive after this frozen boundary.
    if (capture_planning_boundary)
    {
      const std::uint64_t current_observation_seq =
          map->getDynamicObservationSeq();
      const bool no_boundary =
          planning_boundary_required_esdf_seq_ == 0;
      if (no_boundary &&
          current_observation_seq > esdf_observation_seq)
      {
        planning_boundary_required_esdf_seq_ = current_observation_seq;
        dynamic_esdf_wait_start_ = ros::SteadyTime();
        dynamic_esdf_sync_fallback_requested_ = false;
        ROS_INFO("[FSM] Frozen planning-boundary ESDF requirement at seq=%lu in %s.",
                 static_cast<unsigned long>(
                     planning_boundary_required_esdf_seq_),
                 planning_context);
      }
    }

    const std::uint64_t required_esdf_seq = std::max(
        dynamic_event_required_esdf_seq_,
        planning_boundary_required_esdf_seq_);
    if (required_esdf_seq == 0)
    {
      dynamic_esdf_wait_start_ = ros::SteadyTime();
      dynamic_esdf_sync_fallback_requested_ = false;
      return true;
    }

    if (esdf_observation_seq >= required_esdf_seq)
    {
      planning_boundary_required_esdf_seq_ = 0;
      dynamic_esdf_wait_start_ = ros::SteadyTime();
      if (dynamic_event_required_esdf_seq_ == 0)
        dynamic_esdf_sync_fallback_requested_ = false;
      return true;
    }

    // A stale ESDF must never become a planning input merely because waiting
    // took too long. Waiting blocks candidate generation only; it is not a
    // safety hazard by itself and therefore cannot own a zero-speed command.
    // TTC, braking distance, measured collision and tracking faults remain the
    // only stop owners while the current validated trajectory keeps running.
    const ros::SteadyTime now = ros::SteadyTime::now();
    if (dynamic_esdf_wait_start_.isZero())
      dynamic_esdf_wait_start_ = now;
    const double wait_seconds =
        (now - dynamic_esdf_wait_start_).toSec();

    if (wait_seconds >= esdf_freshness_wait_timeout_ &&
        !dynamic_esdf_sync_fallback_requested_)
    {
      dynamic_esdf_sync_fallback_requested_ = true;
      map->requestSynchronousESDFFallback();
      ROS_ERROR("[FSM] ESDF freshness watchdog expired after %.2f s in %s "
                "(required_seq=%lu, esdf_seq=%lu). Keep the current safety "
                "limit and request one synchronous fallback; freshness is "
                "still mandatory for the new candidate.",
                wait_seconds, planning_context,
                static_cast<unsigned long>(required_esdf_seq),
                static_cast<unsigned long>(esdf_observation_seq));
    }
    else
    {
      ROS_INFO_THROTTLE(
          0.5,
          "[FSM] Wait for required ESDF in %s "
          "(required_seq=%lu, esdf_seq=%lu, wait=%.2f s).",
          planning_context,
          static_cast<unsigned long>(required_esdf_seq),
          static_cast<unsigned long>(esdf_observation_seq), wait_seconds);
    }
    return false;
  }

  void REMANIReplanFSM::resetDynamicReplanEvent(
      bool preserve_esdf_requirement)
  {
    const bool preserve_wait_state =
        preserve_esdf_requirement ||
        planning_boundary_required_esdf_seq_ > 0;
    const std::uint64_t preserved_requirement =
        preserve_esdf_requirement ? dynamic_event_required_esdf_seq_ : 0;
    const ros::SteadyTime preserved_wait_start =
        preserve_wait_state ? dynamic_esdf_wait_start_ : ros::SteadyTime();
    const bool preserved_fallback_request =
        preserve_wait_state && dynamic_esdf_sync_fallback_requested_;

    dynamic_replan_retry_pending_ = false;
    dynamic_event_astar_attempted_ = false;
    dynamic_event_from_point_flow_ = false;
    dynamic_event_required_esdf_seq_ = preserved_requirement;
    dynamic_esdf_wait_start_ = preserved_wait_start;
    dynamic_esdf_sync_fallback_requested_ = preserved_fallback_request;
    dynamic_event_replan_attempts_ = 0;
    last_dynamic_replan_attempt_generation_ = 0;
    dynamic_event_first_detected_steady_ = ros::SteadyTime();
    dynamic_replan_hold_until_ = ros::Time(0);
    next_dynamic_replan_retry_time_ = ros::Time(0);
    last_dynamic_replan_collision_pos_.setZero();
    last_dynamic_replan_collision_type_ = -1;
    last_dynamic_replan_collision_from_point_flow_ = false;
    has_last_dynamic_replan_collision_ = false;
  }

  void REMANIReplanFSM::resetDynamicPointFlowDecision()
  {
    dynamic_point_flow_collision_present_ = false;
    dynamic_point_flow_hazard_active_ = false;
    dynamic_point_flow_collision_confirmed_ = false;
    dynamic_point_flow_input_unavailable_ = false;
    dynamic_point_flow_confirm_count_ = 0;
    last_dynamic_point_flow_sequence_ = 0;
    last_dynamic_point_flow_collision_time_ = ros::Time(0);
    dynamic_point_flow_clear_since_ = ros::Time(0);
    last_dynamic_point_flow_collision_position_.setZero();
    last_dynamic_point_flow_evaluation_ = DynamicPointFlowEvaluation();
  }

  void REMANIReplanFSM::clearDynamicSafetySpeedLimit(const ros::Time &now, bool force)
  {
    if (hasUnmetRequiredESDF())
    {
      // Neither an ordinary clear nor a force-clear may release a raw-event or
      // planning-boundary generation before its ESDF commits. This check must
      // cover both requirements: a raw obstacle can disappear from the direct
      // occupancy scan before the asynchronous EDT for its latched frame is
      // ready.
      waitForRequiredESDF("SAFETY_GUARD_CLEAR");
      return;
    }

    if (!dynamic_safety_speed_limit_active_) {
      if (force)
      {
        resetDynamicReplanEvent();
      }
      if (tracking_safety_stop_active_)
        publishSafetySpeedLimit(0.0);
      return;
    }
    if (!force &&
        (exec_state_ == EMERGENCY_STOP ||
         (dynamic_point_flow_active_mode_ &&
          dynamic_point_flow_hazard_active_) ||
         last_dynamic_seen_time_.isZero() ||
         (now - last_dynamic_seen_time_).toSec() < global_plan_dynamic_clear_hold_time_)) {
      return;
    }

    dynamic_safety_speed_limit_active_ = false;
    dynamic_collision_confirm_count_ = 0;
    last_dynamic_collision_time_ = ros::Time(0);
    last_dynamic_seen_time_ = ros::Time(0);
    has_last_dynamic_confirmation_pos_ = false;
    resetDynamicReplanEvent();
    publishSafetySpeedLimit(
        tracking_safety_stop_active_ ? 0.0 : -1.0);
  }

  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    if(local_execution_owner_){checkCooperativeExecution();return;}
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;
    const ros::Time now = ros::Time::now();

    if (!have_target_){
      if (!waitForRequiredESDF("NO_ACTIVE_TARGET"))
        return;
      clearDynamicSafetySpeedLimit(now, true);
      if (dynamic_point_flow_collision_present_ ||
          dynamic_point_flow_hazard_active_ ||
          dynamic_point_flow_input_unavailable_)
      {
        resetDynamicPointFlowDecision();
        publishDynamicPointFlowStatus(0, -1.0);
      }
      publishSafetySpeedLimit(
          tracking_safety_stop_active_ ? 0.0 : -1.0);
      return;
    }
    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0){
      // WAIT_TARGET can be a one-cycle handoff for an already accepted goal,
      // and traj_id is empty before its first local trajectory. Neither is
      // evidence that an obstacle event has cleared.
      waitForRequiredESDF("NO_CHECKABLE_TRAJECTORY");
      return;
    }
    if (exec_state_ == EMERGENCY_STOP)
      return;
    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout()){
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      dynamic_safety_speed_limit_active_ = false;
      publishSafetySpeedLimit(0.0);
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return;
    }
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = getCurrentTrajectoryTime(*info, now);
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_temp = 0;
    bool occ = false;
    bool collision_is_dynamic = false;
    bool collision_from_point_flow = false;
    std::uint64_t collision_dynamic_observation_seq = 0;
    bool point_flow_emergency = false;
    bool point_flow_requires_stop_for_planning = false;
    bool live_dynamic_collision_present = false;
    bool trajectory_safety_scan_complete = false;
    double dynamic_collision_path_distance = std::numeric_limits<double>::infinity();
    Eigen::Vector2d dynamic_collision_position = Eigen::Vector2d::Zero();
    const double base_speed = mm_state_vel_.head(2).norm();
    const double calculated_stop_distance =
        base_speed * base_speed / (2.0 * global_plan_dynamic_emergency_deceleration_) +
        base_speed * global_plan_dynamic_emergency_reaction_time_ +
        global_plan_dynamic_emergency_margin_;
    const double dynamic_emergency_distance = std::min(
        global_plan_dynamic_emergency_max_distance_,
        std::max(global_plan_dynamic_emergency_min_distance_, calculated_stop_distance));
    int coll_type;

    const bool point_flow_check_enabled =
        dynamic_point_flow_active_mode_ ||
        dynamic_point_flow_shadow_mode_;
    const bool point_flow_check_due =
        last_dynamic_point_flow_shadow_check_time_.isZero() ||
        (now - last_dynamic_point_flow_shadow_check_time_).toSec() >=
            dynamic_point_flow_shadow_check_interval_;
    if (point_flow_check_enabled && point_flow_check_due)
    {
      last_dynamic_point_flow_shadow_check_time_ = now;
      DynamicPointFlowEvaluation evaluation;
      if (map->dynamicPointFlowEnabled())
      {
        evaluation = evaluateDynamicPointFlow(*info, t_cur, now);
      }
      else
      {
        evaluation.status =
            DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
        evaluation.reason = "dynamic_point_flow/enable is false";
      }

      if (dynamic_point_flow_active_mode_)
      {
        updateDynamicPointFlowDecision(evaluation, now);
      }
      else if (evaluation.status ==
               DYNAMIC_POINT_FLOW_EVAL_STATUS::COLLISION)
      {
        // Shadow status topics are diagnostics only. They expose the measured
        // TTC needed for calibration without calling the active decision path
        // that owns speed limits, confirmation state and replanning.
        publishDynamicPointFlowStatus(
            1, evaluation.wall_time_ahead);
        ROS_WARN_THROTTLE(
            1.0,
            "[DynamicPointFlow][shadow] Predicted collision type=%d in %.2f s wall time (trajectory t=%.2f, seq=%lu); no control action in shadow mode.",
            evaluation.collision_type,
            evaluation.wall_time_ahead,
            evaluation.trajectory_time,
            static_cast<unsigned long>(evaluation.snapshot.sequence));
      }
      else if (evaluation.status ==
               DYNAMIC_POINT_FLOW_EVAL_STATUS::CLEAR)
      {
        publishDynamicPointFlowStatus(0, -1.0);
        ROS_INFO_THROTTLE(
            2.0,
            "[DynamicPointFlow][shadow] Predicted horizon clear (%zu points, seq=%lu).",
            evaluation.snapshot.points.size(),
            static_cast<unsigned long>(evaluation.snapshot.sequence));
      }
      else if (evaluation.status ==
               DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE)
      {
        publishDynamicPointFlowStatus(-1, -2.0);
        ROS_WARN_THROTTLE(
            2.0,
            "[DynamicPointFlow][shadow] Prediction unavailable (%s); existing live-map safety remains authoritative.",
            evaluation.reason.c_str());
      }
    }

    if (global_plan_) {
      const bool time_for_actual_state_check =
          last_actual_state_collision_check_time_.isZero() ||
          (now - last_actual_state_collision_check_time_).toSec() >=
              actual_state_collision_check_interval_;
      if (has_default_arm_ && time_for_actual_state_check){
        last_actual_state_collision_check_time_ = now;
        const Eigen::Vector3d actual_base_state(
            mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
        const Eigen::VectorXd actual_arm_state =
            mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_);
        const double actual_base_min_distance =
            planner_manager_->mm_config_->getCarObstacleMinDistance(
                actual_base_state, true);
        const double actual_base_hard_stop_distance =
            planner_manager_->mm_config_->getBaseCheckRadius() +
            actual_base_emergency_margin_;
        const double actual_base_guard_distance =
            planner_manager_->mm_config_->getBaseCheckRadius() +
            planner_manager_->mm_config_->getMapResolution();

        int actual_coll_type = -1;
        double actual_collision_distance = actual_base_min_distance;
        bool actual_collision =
            actual_base_min_distance < actual_base_hard_stop_distance;
        if(actual_collision){
          actual_coll_type = 0;
        }else{
          if(actual_base_min_distance < actual_base_guard_distance){
            ROS_WARN_THROTTLE(
                1.0,
                "[FSM] Real base entered the voxel guard band "
                "(distance=%.3f m, hard_stop=%.3f m); keep monitoring without "
                "locking the A* start state.",
                actual_base_min_distance, actual_base_hard_stop_distance);
          }
          if(planner_manager_->mm_config_->checkManiObsCollision(
                 actual_base_state, actual_arm_state, false,
                 actual_collision_distance)){
            actual_collision = true;
            actual_coll_type = 1;
          }else if(planner_manager_->mm_config_->checkCarManiCollision(
                        actual_arm_state, false, actual_collision_distance)){
            actual_collision = true;
            actual_coll_type = 2;
          }else if(planner_manager_->mm_config_->checkManiManiCollision(
                        actual_arm_state, false, actual_collision_distance)){
            actual_collision = true;
            actual_coll_type = 3;
          }
        }
        if(actual_collision){
          ROS_ERROR_THROTTLE(
              1.0,
              "[FSM] Real whole-body state is in hard collision "
              "(type=%d, distance=%.3f m); stop before using nominal trajectory.",
              actual_coll_type, actual_collision_distance);
          dynamic_replan_retry_pending_ = false;
          publishSafetySpeedLimit(0.0);
          changeFSMExecState(EMERGENCY_STOP, "ACTUAL_STATE");
          return;
        }
      }

      // Keep current-pose checking at 100 Hz for emergency reaction.
      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t_cur, coll_type)){
        if(coll_type == 0 && has_default_arm_){
          // The measured-pose check above owns hard base stopping. The trajectory
          // checker includes one full map voxel for planning clearance, which is
          // intentionally too conservative for declaring the real base collided.
          ROS_WARN_THROTTLE(
              1.0,
              "[FSM] Current trajectory sample entered the base planning guard "
              "band; defer hard stop to the measured-pose distance check.");
        }else if(coll_type == 0){
          ROS_WARN_THROTTLE(2.0, "car collision at current time %f!", t_cur);
        }else if (coll_type == 1){
          ROS_WARN_THROTTLE(2.0, "mani collision at current time %f!", t_cur);
        }else if (coll_type == 2){
          ROS_WARN_THROTTLE(2.0, "car-mani collision at current time %f!", t_cur);
        }else if (coll_type == 3){
          ROS_WARN_THROTTLE(2.0, "mani-mani collision at current time %f!", t_cur);
        }
        if(coll_type != 0 || !has_default_arm_){
          occ = true;
          t_temp = t_cur;
        }
      }

      // Scan the future trajectory against both the persistent PCD ESDF and
      // the live LiDAR layer. The latter is kept separate because its direct
      // occupancy query reacts before the ESDF update completes.
      const bool time_for_future_check = last_global_plan_dynamic_check_time_.isZero() ||
                                         (now - last_global_plan_dynamic_check_time_).toSec() >=
                                             global_plan_dynamic_check_interval_;
      if (!occ && global_plan_dynamic_lookahead_time_ > 0.0 && time_for_future_check){
        last_global_plan_dynamic_check_time_ = now;
        const std::uint64_t dynamic_observation_seq = map->getDynamicObservationSeq();
        const bool has_new_dynamic_observation =
            dynamic_observation_seq != last_dynamic_observation_seq_;
        const double time_check_end = std::min(info->duration, t_cur + global_plan_dynamic_lookahead_time_);
        double static_collision_time = std::numeric_limits<double>::infinity();
        double dynamic_collision_time = std::numeric_limits<double>::infinity();
        int static_coll_type = -1;
        int dynamic_coll_type = -1;

        for (double t_check = t_cur + global_plan_static_check_time_step_; t_check <= time_check_end;
             t_check += global_plan_static_check_time_step_){
          if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t_check, static_coll_type)){
            static_collision_time = t_check;
            break;
          }
        }

        Eigen::Vector2d previous_dynamic_position = info->getPos(t_cur).head(2);
        double scanned_path_distance = 0.0;
        double next_far_sample_distance = 0.0;
        for (double t_check = t_cur + global_plan_dynamic_check_time_step_; t_check <= info->duration;
             t_check += global_plan_dynamic_check_time_step_){
          const Eigen::Vector2d dynamic_position = info->getPos(t_check).head(2);
          scanned_path_distance += (dynamic_position - previous_dynamic_position).norm();
          previous_dynamic_position = dynamic_position;

          const bool within_time_horizon = t_check <= time_check_end;
          const bool far_sample_due = scanned_path_distance >= next_far_sample_distance;
          if (within_time_horizon || far_sample_due){
            if (!within_time_horizon)
              next_far_sample_distance = scanned_path_distance + global_plan_dynamic_far_sample_distance_;
            if (planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(*info, t_check, dynamic_coll_type)){
              dynamic_collision_time = t_check;
              dynamic_collision_path_distance = scanned_path_distance;
              dynamic_collision_position = dynamic_position;
              break;
            }
          }

          if (!within_time_horizon && scanned_path_distance >= global_plan_dynamic_lookahead_distance_)
            break;
        }

        const bool has_static_collision = static_collision_time < std::numeric_limits<double>::infinity();
        const bool has_dynamic_collision = dynamic_collision_time < std::numeric_limits<double>::infinity();
        live_dynamic_collision_present = has_dynamic_collision;
        bool confirmed_dynamic_collision = false;
        if (has_dynamic_collision) {
          const int required_confirm_frames =
              dynamic_collision_path_distance <= global_plan_dynamic_immediate_confirm_distance_ ?
                  1 : global_plan_dynamic_confirm_frames_;

          if (has_new_dynamic_observation) {
            const bool confirmation_is_fresh =
                !last_dynamic_collision_time_.isZero() &&
                (now - last_dynamic_collision_time_).toSec() <= global_plan_dynamic_confirm_timeout_;
            const bool confirmation_is_same_obstacle =
                !has_last_dynamic_confirmation_pos_ ||
                (dynamic_collision_position - last_dynamic_confirmation_pos_).norm() <=
                    global_plan_dynamic_same_obstacle_radius_;

            if (confirmation_is_fresh && confirmation_is_same_obstacle)
              ++dynamic_collision_confirm_count_;
            else
              dynamic_collision_confirm_count_ = 1;

            last_dynamic_collision_time_ = now;
            last_dynamic_seen_time_ = now;
            last_dynamic_confirmation_pos_ = dynamic_collision_position;
            has_last_dynamic_confirmation_pos_ = true;
            last_dynamic_observation_seq_ = dynamic_observation_seq;
          }

          confirmed_dynamic_collision = dynamic_collision_path_distance <= dynamic_emergency_distance ||
                                        required_confirm_frames == 1 ||
                                        dynamic_collision_confirm_count_ >= required_confirm_frames;

          if (confirmed_dynamic_collision)
          {
            // Latch before point-flow merge/overlay checks: those may select
            // an earlier predicted collision or return unavailable, but must
            // never hide a confirmed raw occupancy generation.
            latchDynamicEventRequiredESDFSeq(dynamic_observation_seq);
          }

          if (dynamic_collision_path_distance <= global_plan_dynamic_slowdown_distance_){
            const double slowdown_range = std::max(
                1e-3, global_plan_dynamic_slowdown_distance_ - dynamic_emergency_distance);
            const double slowdown_ratio = std::max(
                0.0, std::min(1.0,
                              (dynamic_collision_path_distance - dynamic_emergency_distance) / slowdown_range));
            const double speed_limit = global_plan_dynamic_slowdown_min_speed_ +
                slowdown_ratio * (global_plan_dynamic_slowdown_max_speed_ -
                                  global_plan_dynamic_slowdown_min_speed_);
            applyDynamicSafetySpeedLimit(
                dynamic_collision_path_distance <= dynamic_emergency_distance ? 0.0 : speed_limit);
          }

          if (!confirmed_dynamic_collision) {
            ROS_INFO_THROTTLE(
                1.0,
                "[FSM] Dynamic collision awaiting distinct map frames (%d/%d, map_seq=%lu).",
                dynamic_collision_confirm_count_, required_confirm_frames,
                static_cast<unsigned long>(dynamic_observation_seq));
          }
        } else {
          if (has_new_dynamic_observation) {
            last_dynamic_observation_seq_ = dynamic_observation_seq;
            if (!last_dynamic_collision_time_.isZero() &&
                (now - last_dynamic_collision_time_).toSec() > global_plan_dynamic_confirm_timeout_) {
              dynamic_collision_confirm_count_ = 0;
              last_dynamic_collision_time_ = ros::Time(0);
              has_last_dynamic_confirmation_pos_ = false;
            }
          }
        }

        if (has_static_collision || confirmed_dynamic_collision){
          const bool use_static_collision = has_static_collision &&
                                            (!confirmed_dynamic_collision || static_collision_time <= dynamic_collision_time);
          t_temp = use_static_collision ? static_collision_time : dynamic_collision_time;
          coll_type = use_static_collision ? static_coll_type : dynamic_coll_type;
          collision_is_dynamic = !use_static_collision;
          if (collision_is_dynamic)
            collision_dynamic_observation_seq = dynamic_observation_seq;
          occ = true;
          if (use_static_collision){
            ROS_WARN_THROTTLE(1.0, "[FSM] Future map collision type=%d detected %.2f s ahead.",
                              coll_type, t_temp - t_cur);
          } else {
            ROS_WARN_THROTTLE(1.0,
                              "[FSM] Future dynamic collision type=%d detected %.2f m / %.2f s ahead.",
                              coll_type, dynamic_collision_path_distance, t_temp - t_cur);
          }
        }
        trajectory_safety_scan_complete = true;
      }
    } else {
      // Local reactive mode: scan the entire remaining trajectory for collision.
      for (double t = t_cur; t < info->duration; t += time_step){

        if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
          if(coll_type == 0){
            ROS_WARN_THROTTLE(2.0, "car collision at relative time %f!", t / info->duration);
          }else if (coll_type == 1){
            ROS_WARN_THROTTLE(2.0, "mani collision at relative time %f!", t / info->duration);
          }else if (coll_type == 2){
            ROS_WARN_THROTTLE(2.0, "car-mani collision at relative time %f!", t / info->duration);
          }else if (coll_type == 3){
            ROS_WARN_THROTTLE(2.0, "mani-mani collision at relative time %f!", t / info->duration);
          }

          t_temp = t;
          occ = true;
          break;
        }
      }
      trajectory_safety_scan_complete = true;
    }

    // Merge point-flow only after the static and live-occupancy checks have
    // completed. Each source keeps its own observation sequence and
    // confirmation state; only the earliest confirmed collision owns the
    // replanning decision.
    if (dynamic_point_flow_active_mode_ &&
        dynamic_point_flow_active_replan_ &&
        dynamic_point_flow_collision_present_ &&
        dynamic_point_flow_collision_confirmed_ &&
        last_dynamic_point_flow_evaluation_.status ==
            DYNAMIC_POINT_FLOW_EVAL_STATUS::COLLISION)
    {
      const DynamicPointFlowEvaluation &flow =
          last_dynamic_point_flow_evaluation_;
      if (!map->dynamicPointPlanningOverlayReadyFor(
              flow.snapshot.sequence) &&
          !map->refreshDynamicPointPlanningOverlay(
              flow.snapshot, now))
      {
        ROS_WARN_THROTTLE(
            1.0,
            "[DynamicPointFlow][active] Planning snapshot seq=%lu is not ready yet; retain the TTC-derived speed limit while the isolated overlay worker catches up.",
            static_cast<unsigned long>(flow.snapshot.sequence));
        return;
      }
      if (!occ || flow.trajectory_time < t_temp)
      {
        occ = true;
        collision_is_dynamic = true;
        collision_from_point_flow = true;
        coll_type = flow.collision_type;
        t_temp = flow.trajectory_time;
        dynamic_collision_path_distance = flow.path_distance;
        dynamic_collision_position = flow.trajectory_position;
      }
      point_flow_emergency =
          flow.wall_time_ahead <=
          dynamic_point_flow_active_emergency_ttc_;
      point_flow_requires_stop_for_planning =
          point_flow_emergency;
    }

    // Release an obstacle/planning stop only after all safety sources have
    // evaluated the controller's actual published trajectory. In particular,
    // the earlier raw-occupancy branch must not release the guard before a
    // later static or point-flow collision is merged into `occ`.
    const bool point_flow_blocks_release =
        dynamic_point_flow_active_mode_ &&
        (dynamic_point_flow_hazard_active_ ||
         (dynamic_point_flow_fail_safe_on_unavailable_ &&
          dynamic_point_flow_input_unavailable_));
    if (trajectory_safety_scan_complete &&
        !occ && !live_dynamic_collision_present &&
        !point_flow_blocks_release)
    {
      clearDynamicSafetySpeedLimit(now);
      if (dynamic_replan_retry_pending_ &&
          !dynamic_safety_speed_limit_active_ &&
          exec_state_ == REPLAN_TRAJ && have_controller_progress_)
      {
        resetDynamicReplanEvent();
        changeFSMExecState(EXEC_TRAJ, "SAFETY_CLEAR");
      }
    }

    if (occ){
      /* Handle the collided case immediately */
      // Resolve the event boundary before an emergency branch can return. A
      // point-flow-only event may move enough to start a new repair budget.
      // A raw event is identified by its immutable observation sequence, not
      // by the trajectory intersection point (which naturally moves as the
      // robot/trajectory changes), so it must never re-latch/chase frames here.
      if (dynamic_event_required_esdf_seq_ == 0 &&
          collision_is_dynamic && collision_from_point_flow &&
          has_last_dynamic_replan_collision_ &&
          last_dynamic_replan_collision_from_point_flow_ &&
          (dynamic_collision_position - last_dynamic_replan_collision_pos_).norm() >
              global_plan_dynamic_same_obstacle_radius_) {
        ROS_INFO("[FSM] Dynamic collision moved %.2f m; start a new local repair event.",
                 (dynamic_collision_position - last_dynamic_replan_collision_pos_).norm());
        resetDynamicReplanEvent();
      }

      if (collision_is_dynamic && dynamic_event_first_detected_steady_.isZero())
        dynamic_event_first_detected_steady_ = ros::SteadyTime::now();

      if (collision_is_dynamic &&
          !(dynamic_event_required_esdf_seq_ > 0 &&
            collision_from_point_flow))
        dynamic_event_from_point_flow_ = collision_from_point_flow;

      if (collision_from_point_flow &&
          point_flow_requires_stop_for_planning){
        applyDynamicSafetySpeedLimit(0.0);
        ROS_ERROR_THROTTLE(
            0.5,
            "[DynamicPointFlow][active] Predicted collision is inside the emergency TTC; stop while the local repair is computed.");
      }
      if (collision_is_dynamic && !collision_from_point_flow &&
          dynamic_collision_path_distance <= dynamic_emergency_distance){
        ROS_ERROR_THROTTLE(1.0,
                           "[FSM] Dynamic obstacle only %.2f m ahead (stop threshold %.2f m); stop before replanning.",
                           dynamic_collision_path_distance, dynamic_emergency_distance);
        applyDynamicSafetySpeedLimit(0.0);
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        return;
      }

      // A raw collision requirement remains authoritative even if a
      // point-flow prediction or a static obstacle becomes the earlier
      // collision on a later callback. Never compare positions across those
      // sources to redefine the raw event, and never plan until its ESDF
      // generation has committed.
      if (!waitForRequiredESDF("COLLISION_REPLAN")) {
        if (collision_is_dynamic) {
          last_dynamic_replan_collision_pos_ = dynamic_collision_position;
          last_dynamic_replan_collision_type_ = coll_type;
          last_dynamic_replan_collision_from_point_flow_ =
              collision_from_point_flow;
          has_last_dynamic_replan_collision_ = true;
        }
        dynamic_replan_retry_pending_ = true;
        next_dynamic_replan_retry_time_ = ros::Time::now();
        dynamic_replan_hold_until_ = next_dynamic_replan_retry_time_;
        changeFSMExecState(REPLAN_TRAJ, "WAIT_DYNAMIC_ESDF");
        return;
      }

      const bool newer_raw_observation_invalidates_published_trajectory =
          collision_is_dynamic && !collision_from_point_flow &&
          collision_dynamic_observation_seq >
              last_published_dynamic_validation_seq_;
      const bool newer_point_flow_invalidates_published_trajectory =
          collision_is_dynamic && collision_from_point_flow &&
          last_dynamic_point_flow_evaluation_.snapshot.sequence >
              last_published_point_flow_validation_seq_;
      if (collision_is_dynamic && has_last_dynamic_replan_collision_ &&
          last_dynamic_replan_collision_from_point_flow_ ==
              collision_from_point_flow &&
          last_dynamic_replan_collision_type_ == coll_type &&
          now < dynamic_replan_hold_until_ &&
          !newer_raw_observation_invalidates_published_trajectory &&
          !newer_point_flow_invalidates_published_trajectory) {
        ROS_INFO_THROTTLE(
            1.0,
            "[FSM] Keep executing the new trajectory while the continuous "
            "dynamic event settles (%.2f s).",
            (dynamic_replan_hold_until_ - now).toSec());
        return;
      }
      if (newer_raw_observation_invalidates_published_trajectory &&
          now < dynamic_replan_hold_until_)
      {
        ROS_WARN_THROTTLE(
            0.5,
            "[FSM] New live map seq=%lu invalidates a trajectory validated "
            "at seq=%lu; bypass the %.2f s same-event hold and continue "
            "the bounded fallback chain.",
            static_cast<unsigned long>(
                collision_dynamic_observation_seq),
            static_cast<unsigned long>(
                last_published_dynamic_validation_seq_),
                (dynamic_replan_hold_until_ - now).toSec());
      }
      if (newer_point_flow_invalidates_published_trajectory &&
          now < dynamic_replan_hold_until_)
      {
        ROS_WARN_THROTTLE(
            0.5,
            "[FSM] New point-flow seq=%lu still intersects a trajectory "
            "validated at seq=%lu; bypass the %.2f s same-event hold.",
            static_cast<unsigned long>(
                last_dynamic_point_flow_evaluation_.snapshot.sequence),
            static_cast<unsigned long>(
                last_published_point_flow_validation_seq_),
            (dynamic_replan_hold_until_ - now).toSec());
      }
      if (collision_is_dynamic && dynamic_event_astar_attempted_) {
        const std::uint64_t available_generation =
            collision_from_point_flow ?
                last_dynamic_point_flow_evaluation_.snapshot.sequence :
                collision_dynamic_observation_seq;
        if (available_generation <=
                last_dynamic_replan_attempt_generation_ ||
            now < next_dynamic_replan_retry_time_)
        {
          ROS_INFO_THROTTLE(
              0.5,
              "[FSM] Dynamic A* retry is coalesced (last generation=%lu, available=%lu); TTC safety remains authoritative.",
              static_cast<unsigned long>(
                  last_dynamic_replan_attempt_generation_),
              static_cast<unsigned long>(available_generation));
          return;
        }
        dynamic_replan_retry_pending_ = true;
        changeFSMExecState(REPLAN_TRAJ, "DYNAMIC_NEW_GENERATION");
        return;
      }

      // Capture the ordinary/new-target freshness generation only when this
      // callback will actually enter the synchronous optimizer. Holding or
      // waiting for a retry must not chase high-rate LiDAR frames.
      if (!waitForRequiredESDF(
              "COLLISION_PLAN_NOW",
              /*capture_planning_boundary=*/!collision_is_dynamic))
      {
        if (collision_is_dynamic)
        {
          last_dynamic_replan_collision_pos_ = dynamic_collision_position;
          last_dynamic_replan_collision_type_ = coll_type;
          last_dynamic_replan_collision_from_point_flow_ =
              collision_from_point_flow;
          has_last_dynamic_replan_collision_ = true;
        }
        dynamic_replan_retry_pending_ =
            collision_is_dynamic || dynamic_event_required_esdf_seq_ > 0;
        next_dynamic_replan_retry_time_ = ros::Time::now();
        dynamic_replan_hold_until_ = next_dynamic_replan_retry_time_;
        changeFSMExecState(REPLAN_TRAJ, "WAIT_PLANNING_ESDF");
        return;
      }

      ROS_INFO("Try to replan a safe trajectory");
      if (collision_from_point_flow)
      {
        if (point_flow_requires_stop_for_planning)
        {
          // Zero-speed ownership was established above by emergency TTC.
          // Keep the guard explicit so the candidate starts from feedback.
          applyDynamicSafetySpeedLimit(0.0);
        }
        else
        {
          // The controller runs in another process, so it may continue along
          // the last published trajectory at the TTC-derived speed limit while
          // this bounded repair is computed. Publication still requires swept
          // prediction and newest live-occupancy validation.
          ROS_INFO_THROTTLE(
              0.5,
              "[DynamicPointFlow][active] Early collision warning; keep the current trajectory speed-limited while computing a point-flow-aware repair.");
        }
      }
      if(collision_is_dynamic) {
        dynamic_event_astar_attempted_ = true;
        last_dynamic_replan_attempt_generation_ =
            collision_from_point_flow ?
                last_dynamic_point_flow_evaluation_.snapshot.sequence :
                collision_dynamic_observation_seq;
        ++dynamic_event_replan_attempts_;
      }
      if (planFromLocalTraj(
              collision_is_dynamic, !collision_is_dynamic,
              collision_is_dynamic ?
                  dynamic_astar_replan_wall_time_ :
                  -1.0,
              /*dynamic_repair=*/collision_is_dynamic)){ // Make a chance
        dynamic_collision_confirm_count_ = 0;
        last_dynamic_collision_time_ = ros::Time(0);
        if (collision_is_dynamic) {
          last_dynamic_replan_collision_pos_ = dynamic_collision_position;
          last_dynamic_replan_collision_type_ = coll_type;
          last_dynamic_replan_collision_from_point_flow_ =
              collision_from_point_flow;
          has_last_dynamic_replan_collision_ = true;
          dynamic_replan_hold_until_ =
              ros::Time::now() + ros::Duration(global_plan_dynamic_replan_hold_time_);
          next_dynamic_replan_retry_time_ = dynamic_replan_hold_until_;
          dynamic_replan_retry_pending_ = false;
        }
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        if (!collision_is_dynamic && t_temp - t_cur < emergency_time_){
          ROS_WARN_THROTTLE(2.0, "Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else if (collision_is_dynamic){
          last_dynamic_replan_collision_pos_ = dynamic_collision_position;
          last_dynamic_replan_collision_type_ = coll_type;
          last_dynamic_replan_collision_from_point_flow_ =
              collision_from_point_flow;
          has_last_dynamic_replan_collision_ = true;
          dynamic_replan_retry_pending_ = true;
          next_dynamic_replan_retry_time_ =
              ros::Time::now() +
              ros::Duration(global_plan_dynamic_replan_hold_time_);
          dynamic_replan_hold_until_ = next_dynamic_replan_retry_time_;
          ROS_WARN("[FSM] Short-horizon dynamic A* failed; keep the old "
                   "trajectory TTC-limited and retry from a newer observation "
                   "after %.2f s.",
                   global_plan_dynamic_replan_hold_time_);
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }else{
          ROS_WARN("current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            ROS_INFO("Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  bool REMANIReplanFSM::planNextWaypoint(
      const Eigen::VectorXd next_wp, const double next_yaw,
      bool preserve_active_target_on_failure)
  {
    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw, Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;
      end_yaw_ = next_yaw;
      have_local_traj_ = false;
      start_singul_ = 0;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;
      new_traj_fail_time_ = 0;

      if (exec_state_ != INIT)
      {
        // A new goal should interrupt an unreachable old goal immediately.
        // Waiting for EXEC_TRAJ here can deadlock while GEN_NEW_TRAJ is failing.
        if (exec_state_ == GEN_NEW_TRAJ)
          continously_called_times_ = 0;
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(final_goal_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
      if (!preserve_active_target_on_failure)
      {
        have_target_ = false;
        have_new_target_ = false;
        have_local_traj_ = false;
        new_traj_fail_time_ = 0;
        if (exec_state_ != INIT)
          changeFSMExecState(WAIT_TARGET, "TRIG");
      }
    }

    return success;
  }

  // manual waypoint
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    if (!planner_manager_->grid_map_->mapReadyForPlanning()){
      ROS_WARN_THROTTLE(1.0,
                        "[FSM] Goal ignored because the static global-map ESDF is not ready yet. "
                        "Please resend the goal after the readiness message.");
      return;
    }

    // Validate the complete request before changing any target, event, guard,
    // or start-state bookkeeping. In particular, a malformed RViz message
    // must not release a stop that belongs to an uncommitted raw-map frame.
    if (!have_odom_ || mm_state_pos_.size() != traj_dim_ ||
        !mm_state_pos_.allFinite())
    {
      ROS_ERROR_THROTTLE(
          1.0, "[FSM] Goal ignored because the measured robot state is invalid.");
      return;
    }

    Eigen::VectorXd requested_end;
    double requested_yaw = 0.0;
    int requested_wpt_id = 0;
    bool requested_manual_restore = false;
    bool uses_work_arm_waypoint = false;
    bool uses_standby_arm = false;

    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      if (waypoint_num_ <= 0 || waypoints_.empty() ||
          waypoints_yaw_.empty() ||
          waypoints_.front().size() != traj_dim_ ||
          !waypoints_.front().allFinite() ||
          !std::isfinite(waypoints_yaw_.front()))
      {
        ROS_ERROR("[FSM] Preset goal ignored because waypoint0 is invalid.");
        return;
      }
      requested_end = waypoints_.front();
      requested_yaw = waypoints_yaw_.front();
    }
    else if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      const auto &position = msg->pose.position;
      const auto &orientation = msg->pose.orientation;
      const double quaternion_norm = std::sqrt(
          orientation.x * orientation.x + orientation.y * orientation.y +
          orientation.z * orientation.z + orientation.w * orientation.w);
      if (position.z < -0.1)
        return;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
          !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
          !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
          !std::isfinite(orientation.w) || quaternion_norm < 1e-6)
      {
        ROS_ERROR("[FSM] Manual goal ignored because its pose is invalid.");
        return;
      }

      requested_end = mm_state_pos_;
      requested_end(0) = position.x;
      requested_end(1) = position.y;
      requested_yaw = tf::getYaw(orientation);
      if (!std::isfinite(requested_yaw))
      {
        ROS_ERROR("[FSM] Manual goal ignored because its yaw is invalid.");
        return;
      }

      const bool valid_work_wp =
          manual_work_arm_waypoint_id_ >= 0 &&
          manual_work_arm_waypoint_id_ < waypoint_num_ &&
          manual_work_arm_waypoint_id_ <
              static_cast<int>(waypoints_.size()) &&
          waypoints_[manual_work_arm_waypoint_id_].size() == traj_dim_ &&
          waypoints_[manual_work_arm_waypoint_id_].allFinite();
      if (manual_use_work_arm_waypoint_ && valid_work_wp)
      {
        requested_wpt_id = manual_work_arm_waypoint_id_;
        requested_end.segment(mobile_base_dim_, manipulator_dim_) =
            waypoints_[manual_work_arm_waypoint_id_].segment(
                mobile_base_dim_, manipulator_dim_);
        uses_work_arm_waypoint = true;
      }
      else
      {
        requested_wpt_id = std::max(0, waypoint_num_ - 1);
        if (manual_target_end_arm_standby_ && has_default_arm_)
        {
          requested_end.segment(mobile_base_dim_, manipulator_dim_) =
              default_arm_config_;
          uses_standby_arm = true;
        }
      }
      requested_manual_restore =
          manual_restore_standby_after_goal_ && has_default_arm_ &&
          !uses_standby_arm &&
          (requested_end.segment(mobile_base_dim_, manipulator_dim_) -
           default_arm_config_).norm() > 1e-3;
    }
    else
    {
      ROS_ERROR("wrong target type: %d", target_type_);
      return;
    }

    const bool preserve_predictive_guard =
        dynamic_point_flow_active_mode_ &&
        dynamic_point_flow_hazard_active_;
    const bool raw_esdf_unmet = hasUnmetRequiredESDF();
    const bool preserve_required_esdf =
        dynamic_event_required_esdf_seq_ > 0;
    const bool preserve_active_dynamic_guard =
        dynamic_safety_speed_limit_active_;
    const bool preserve_any_dynamic_guard =
        raw_esdf_unmet || preserve_required_esdf ||
        preserve_active_dynamic_guard ||
        preserve_predictive_guard;

    // Global waypoint generation is synchronous on the default callback
    // queue. Freeze the old trajectory before entering it whenever any live
    // hazard guard is active, including when the new request later fails.
    if (preserve_any_dynamic_guard)
      applyDynamicSafetySpeedLimit(0.0);

    if (!planNextWaypoint(
            requested_end, requested_yaw,
            /*preserve_active_target_on_failure=*/true))
      return;

    if(local_execution_owner_){execution_goal_key_=executionHex(*msg);publishExecutionState();}
    cout << "Triggered! traget type: " << target_type_ << endl;
    wpt_id_ = requested_wpt_id;
    goal_yaw_alignment_start_time_ = ros::Time(0);

    if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      have_trigger_ = true;
      std_msgs::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = ros::Time::now();
      planner_manager_->start_flag_ = true;
      start_pub_.publish(flag_msg);
    }
    else
    {
      init_state_ = mm_state_pos_;
      manual_restore_pending_ = requested_manual_restore;
      manual_restore_running_ = false;
      if (uses_work_arm_waypoint)
      {
        ROS_INFO("[FSM] Manual target uses arm pose from waypoint%d. "
                 "Restore stand-by after goal: %s",
                 requested_wpt_id,
                 manual_restore_pending_ ? "true" : "false");
      }
      else if (uses_standby_arm)
      {
        ROS_INFO("[FSM] Manual target ends with configured stand-by arm pose.");
      }
      else
      {
        ROS_INFO("[FSM] Manual target keeps current arm pose. "
                 "Restore stand-by after goal: %s",
                 manual_restore_pending_ ? "true" : "false");
      }
    }

    // Reset the accepted goal's retry budget only after validation/planning.
    // Preserve an unmet raw generation and the monotonic watchdog verbatim.
    resetDynamicReplanEvent(preserve_required_esdf);
    last_dynamic_observation_seq_ = 0;
    dynamic_collision_confirm_count_ = 0;
    last_dynamic_collision_time_ = ros::Time(0);
    has_last_dynamic_confirmation_pos_ = false;

    if (raw_esdf_unmet)
    {
      ROS_WARN("[FSM] Accepted a new goal while an ESDF generation is still "
               "pending; keep the controller stopped until the planning "
               "freshness gate is satisfied.");
    }
    else if (preserve_required_esdf)
    {
      ROS_WARN("[FSM] Accepted a new goal while a raw obstacle guard is active "
               "(required_seq=%lu, freshness=%s); keep the controller stopped "
               "until live safety checks explicitly clear the event.",
               static_cast<unsigned long>(dynamic_event_required_esdf_seq_),
               raw_esdf_unmet ? "pending" : "committed");
    }
    else if (preserve_predictive_guard)
    {
      ROS_WARN("[DynamicPointFlow][active] Accepted a new goal while a "
               "predictive hazard is active; preserve its speed guard and "
               "last-seen time until the new trajectory is checked.");
    }
    else if (preserve_active_dynamic_guard)
    {
      ROS_WARN("[FSM] Accepted a new goal while a live-obstacle speed guard "
               "is active; keep the controller stopped until safety checks "
               "explicitly clear it.");
    }
    else
    {
      clearDynamicSafetySpeedLimit(ros::Time::now(), true);
      resetDynamicPointFlowDecision();
      last_dynamic_seen_time_ = ros::Time(0);
    }
  }

  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // std::cout << "odom: " << mm_state_pos_.transpose() << "\n";
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf::getYaw(msg->pose.pose.orientation);

    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    const double body_vx = msg->twist.twist.linear.x;
    const double body_vy = msg->twist.twist.linear.y;
    mm_state_vel_(0) = cos(mm_car_yaw_) * body_vx - sin(mm_car_yaw_) * body_vy;
    mm_state_vel_(1) = sin(mm_car_yaw_) * body_vx + cos(mm_car_yaw_) * body_vy;
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = car_head.dot(mm_state_vel_.head(2)) >= 0.0 ? 1 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;

    have_odom_ = true;
  }

  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg){
    for(int i = 0; i < manipulator_dim_; ++i){
      if(i < msg->position.size()){
        mm_state_pos_(mobile_base_dim_ + i) = msg->position[i];
      }
      if(i < msg->velocity.size()){
        mm_state_vel_(mobile_base_dim_ + i) = msg->velocity[i];
      }else{
        mm_state_vel_(mobile_base_dim_ + i) = 0.0;
      }
      if(i < msg->effort.size()){
        mm_state_acc_(mobile_base_dim_ + i) = msg->effort[i];
      }else{
        mm_state_acc_(mobile_base_dim_ + i) = 0.0;
      }
    }
    // Fallback only: keep the configured stand-by pose if fsm/init_state exists.
    if (!has_default_arm_) {
      has_default_arm_ = true;
      default_arm_config_ = mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_);
      default_arm_from_config_ = false;
      ROS_INFO_STREAM("[FSM] Arm stand-by pose captured from first joint_state: "
                      << default_arm_config_.transpose());
    }
  }

  void REMANIReplanFSM::gripperCallback(const std_msgs::Bool::ConstPtr &msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_);
    }
  }

  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    publishExecutionState();
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << string(dot_nums, '.') << endl;

    fflush(stdout);
  }

  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;
    
    std::vector<std::string> execution_segments;
    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      quadrotor_msgs::PolynomialTraj msg;
      msg.trajectory_id = data->singul_traj[i].traj_id;
      msg.header.stamp = ros::Time(data->start_time);
      msg.action = msg.ACTION_ADD;
      msg.singul = data->singul_traj[i].singul;
      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree();
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration();
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();
        piece.data.assign(cMat.data(),cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
      poly_traj_pub_.publish(msg);
      if(local_execution_owner_)execution_segments.push_back(executionHex(msg));
    }

    if(local_execution_owner_){
      std_msgs::String transaction;transaction.data=execution_goal_key_+"\n"+std::to_string(execution_segments.size());
      for(const auto&s:execution_segments)transaction.data+="\n"+s;
      execution_reference_pub_.publish(transaction);
    }
  }

  bool REMANIReplanFSM::isRearDistantGoal() const
  {
    if (end_pt_.size() < 2 || mm_state_pos_.size() < 2)
      return false;
    const Eigen::Vector2d offset =
        end_pt_.head(2) - mm_state_pos_.head(2);
    const double distance = offset.norm();
    if (distance < rear_goal_plan_distance_ || distance < 1e-6)
      return false;
    const Eigen::Vector2d heading(cos(mm_car_yaw_), sin(mm_car_yaw_));
    return heading.dot(offset / distance) <= rear_goal_plan_cos_;
  }

  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;
    else flag_random_poly_init = true;
    const ros::WallTime planning_start = ros::WallTime::now();
    for(int i = 0; i < trial_times; i++){
      const double elapsed =
          (ros::WallTime::now() - planning_start).toSec();
      if(elapsed >= global_plan_max_wall_time_){
        ROS_WARN("[FSM] Stop global planning retries after %.2f s wall-time budget.",
                 elapsed);
        break;
      }
      if(callReboundReplan(true, flag_random_poly_init)){
        return true;
      }
    }
    return false;
  }

  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init,
                                          bool allow_random_retry,
                                          double optimization_wall_time,
                                          bool dynamic_repair){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    const bool guarded_dynamic_stop_handoff =
        dynamic_safety_speed_limit_active_ &&
        std::isfinite(current_safety_speed_limit_) &&
        current_safety_speed_limit_ <= 1e-3;
    if (guarded_dynamic_stop_handoff)
    {
      // A genuine dynamic-safety zero-speed guard (emergency TTC or raw
      // braking distance) freezes controller progress. Anchor the repair at
      // the latest measured whole-body state instead of extrapolating the old
      // trajectory through time which the robot did not execute.
      start_pos_ = mm_state_pos_;
      start_vel_.setZero();
      start_acc_.setZero();
      start_jer_.setZero();
      start_yaw_ = mm_car_yaw_;
      start_singul_ = mm_car_singul_;
      ROS_INFO("[FSM] Dynamic-safety replan handoff anchored at the measured stopped state.");
    }
    else
    {
    const ros::Time now = ros::Time::now();
    const double t_now = getCurrentTrajectoryTime(*info, now);
    const double current_time_scale =
        getCurrentTrajectoryScale(*info, now);
    const double handoff_time = dynamic_repair ?
        dynamic_replan_handoff_time_ : replan_trajectory_time_;
    const double expected_progress = handoff_time * current_time_scale;
    const double t_cur = std::min(info->duration, t_now + expected_progress);

    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur) * current_time_scale;
    start_acc_    = info->getAcc(t_cur) *
                    (current_time_scale * current_time_scale);
    start_jer_   = info->getJer(t_cur) *
                   (current_time_scale * current_time_scale * current_time_scale);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.head(2).norm() >= mobile_base_non_singul_vel_) start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else start_yaw_ = mm_car_yaw_;

    // The original handoff assumed that the physical base stayed exactly on the
    // old trajectory. Preserve the measured base residual at the switch time so
    // the next trajectory starts from the real robot's predicted location.
    const Eigen::VectorXd expected_now = info->getPos(t_now);
    const Eigen::Vector2d position_error = mm_state_pos_.head(2) - expected_now.head(2);
    const Eigen::VectorXd arm_position_error =
        mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_) -
        expected_now.segment(mobile_base_dim_, manipulator_dim_);
    const Eigen::VectorXd expected_now_vel = info->getVel(t_now);
    const int expected_now_singul = info->getSingul(t_now);
    const double expected_now_yaw = expected_now_vel.head(2).norm() >= mobile_base_non_singul_vel_
        ? atan2(expected_now_singul * expected_now_vel(1), expected_now_singul * expected_now_vel(0))
        : mm_car_yaw_;
    const double yaw_error = atan2(sin(mm_car_yaw_ - expected_now_yaw), cos(mm_car_yaw_ - expected_now_yaw));

    if (position_error.norm() > replan_feedback_position_threshold_ ||
        fabs(yaw_error) > replan_feedback_yaw_threshold_) {
      start_pos_.head(2) += position_error;
      start_yaw_ = atan2(sin(start_yaw_ + yaw_error), cos(start_yaw_ + yaw_error));
      ROS_WARN("[FSM] Replan handoff feedback: pos_error=%.3f m, yaw_error=%.3f rad, predicted_start=(%.3f, %.3f)",
               position_error.norm(), yaw_error, start_pos_(0), start_pos_(1));
    }
    if(arm_position_error.cwiseAbs().maxCoeff() > replan_feedback_arm_threshold_){
      start_pos_.segment(mobile_base_dim_, manipulator_dim_) += arm_position_error;
      start_vel_.segment(mobile_base_dim_, manipulator_dim_) =
          mm_state_vel_.segment(mobile_base_dim_, manipulator_dim_);
      start_acc_.segment(mobile_base_dim_, manipulator_dim_).setZero();
      start_jer_.segment(mobile_base_dim_, manipulator_dim_).setZero();
      ROS_WARN("[FSM] Replan handoff uses measured arm state (max error %.3f rad).",
               arm_position_error.cwiseAbs().maxCoeff());
    }
    }

    const double original_wall_time =
        planner_manager_->ploy_traj_opt_->getMaxWallTime();
    const double original_astar_search_time =
        planner_manager_->ploy_traj_opt_->getAstarMaxSearchTime();
    if (optimization_wall_time > 0.0)
      planner_manager_->ploy_traj_opt_->setMaxWallTime(
          optimization_wall_time);
    if (dynamic_repair && flag_use_poly_init)
      planner_manager_->ploy_traj_opt_->setAstarMaxSearchTime(
          dynamic_astar_search_time_);

    bool frozen_prediction_snapshot = false;
    if (dynamic_repair &&
        planner_manager_->grid_map_->dynamicPointPlanningOverlayEnabled())
    {
      const ros::Time snapshot_time = ros::Time::now();
      const plan_env::DynamicPointSnapshot planning_snapshot =
          planner_manager_->grid_map_->getDynamicPointSnapshot(snapshot_time);
      if (planning_snapshot.valid && !planning_snapshot.points.empty())
      {
        frozen_prediction_snapshot =
            planner_manager_->grid_map_->beginDynamicPointPlanningSnapshot(
                planning_snapshot, snapshot_time);
        if (!frozen_prediction_snapshot && dynamic_event_from_point_flow_)
        {
          planner_manager_->ploy_traj_opt_->setMaxWallTime(
              original_wall_time);
          planner_manager_->ploy_traj_opt_->setAstarMaxSearchTime(
              original_astar_search_time);
          ROS_ERROR("[FSM] Point-flow repair rejected because no immutable planning snapshot could be frozen.");
          return false;
        }
      }
      else if (dynamic_event_from_point_flow_)
      {
        planner_manager_->ploy_traj_opt_->setMaxWallTime(
            original_wall_time);
        planner_manager_->ploy_traj_opt_->setAstarMaxSearchTime(
            original_astar_search_time);
        ROS_ERROR("[FSM] Point-flow repair rejected because the prediction snapshot is unavailable (%s).",
                  planning_snapshot.invalid_reason.c_str());
        return false;
      }
    }

    const ros::WallTime repair_start = ros::WallTime::now();
    bool success = callReboundReplan(
        flag_use_poly_init, false, dynamic_repair);
    if (!success && allow_random_retry){
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true, dynamic_repair);
        if (success)
          break;
      }
    }
    if (frozen_prediction_snapshot)
      planner_manager_->grid_map_->endDynamicPointPlanningSnapshot();
    planner_manager_->ploy_traj_opt_->setMaxWallTime(original_wall_time);
    planner_manager_->ploy_traj_opt_->setAstarMaxSearchTime(
        original_astar_search_time);

    const double repair_wall_ms =
        (ros::WallTime::now() - repair_start).toSec() * 1000.0;
    if (dynamic_repair)
    {
      const double event_age_ms = dynamic_event_first_detected_steady_.isZero()
          ? -1.0
          : (ros::SteadyTime::now() -
             dynamic_event_first_detected_steady_).toSec() * 1000.0;
      ROS_INFO("[FSM][DynamicLatency] repair=%s seed=%s wall=%.1fms event_to_result=%.1fms horizon=%s.",
               success ? "success" : "failed",
               flag_use_poly_init ? "astar" : "warm",
               repair_wall_ms, event_age_ms,
               (dynamic_local_replan_enable_ && flag_use_poly_init) ?
                   "local" : "legacy");
    }

    if (!success)
      return false;

    return true;
  }

  bool REMANIReplanFSM::validateStaticTrajectory(const SingulTrajData &traj, int &coll_type) const
  {
    for (double t = 0.0; t <= traj.duration; t += global_plan_static_check_time_step_)
    {
      if (planner_manager_->ploy_traj_opt_->checkPlanningCollision(
              traj, t, coll_type))
        return false;
    }
    if (traj.duration > 0.0 &&
        planner_manager_->ploy_traj_opt_->checkPlanningCollision(
            traj, traj.duration, coll_type))
      return false;
    return true;
  }

  bool REMANIReplanFSM::validateDynamicTrajectory(
      const SingulTrajData &traj, int &coll_type) const
  {
    const double check_step = std::max(0.05, global_plan_static_check_time_step_);
    for (double t = 0.0; t <= traj.duration; t += check_step)
    {
      if (planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(
              traj, t, coll_type))
        return false;
    }
    if (traj.duration > 0.0 &&
        planner_manager_->ploy_traj_opt_->checkDynamicObstacleCollision(
            traj, traj.duration, coll_type))
      return false;
    return true;
  }

  bool REMANIReplanFSM::validatePredictedDynamicTrajectory(
      const SingulTrajData &traj,
    const plan_env::DynamicPointSnapshot &snapshot,
      int &coll_type) const
  {
    if (!snapshot.valid)
      return false;

    const ros::Time now = ros::Time::now();
    const double observation_age =
        std::max(0.0, (now - snapshot.stamp).toSec());
    const double valid_wall_horizon =
        planner_manager_->grid_map_->dynamicPointMaxPredictionTime() -
        observation_age;
    const double start_delay = std::max(
        0.0, traj.start_time - now.toSec());
    if (valid_wall_horizon <= start_delay + 1e-3)
      return false;

    const double check_step = std::max(
        0.05, global_plan_dynamic_check_time_step_);

    // During a future activation delay the controller holds position.  For a
    // guarded point-flow repair traj(0) is explicitly anchored at the measured
    // stopped state, so validate that hold interval against the same snapshot.
    double hold_wall = 0.0;
    const double hold_horizon = std::min(start_delay, valid_wall_horizon);
    while (hold_wall <= hold_horizon + 1e-9)
    {
      bool query_valid = true;
      if (planner_manager_->ploy_traj_opt_->
              checkPredictedDynamicObstacleCollision(
                  traj, 0.0, now + ros::Duration(hold_wall),
                  snapshot, coll_type, query_valid))
        return false;
      if (!query_valid)
        return false;
      if (hold_wall >= hold_horizon - 1e-9)
        break;
      hold_wall = std::min(hold_horizon, hold_wall + check_step);
    }

    std::vector<double> execution_scales{
        dynamic_point_flow_active_resume_scale_,
        std::max(0.1, 0.5 * dynamic_point_flow_active_resume_scale_)};
    std::sort(execution_scales.begin(), execution_scales.end());
    execution_scales.erase(
        std::unique(execution_scales.begin(), execution_scales.end(),
                    [](double lhs, double rhs)
                    { return std::fabs(lhs - rhs) < 1e-3; }),
        execution_scales.end());

    for (const double execution_scale : execution_scales)
    {
      const double trajectory_horizon = std::min(
          traj.duration,
          (valid_wall_horizon - start_delay) * execution_scale);
      double t = 0.0;
      while (t <= trajectory_horizon + 1e-9)
      {
        const double wall_ahead =
            start_delay + t / execution_scale;
        bool query_valid = true;
        if (planner_manager_->ploy_traj_opt_->
                checkPredictedDynamicObstacleCollision(
                    traj, t, now + ros::Duration(wall_ahead),
                    snapshot, coll_type, query_valid))
          return false;
        if (!query_valid)
          return false;
        if (t >= trajectory_horizon - 1e-9)
          break;
        t = std::min(trajectory_horizon, t + check_step);
      }
    }
    return true;
  }

  void REMANIReplanFSM::publishDynamicPointFlowStatus(
      int state, double ttc)
  {
    std_msgs::Int32 state_msg;
    state_msg.data = state;
    dynamic_point_flow_state_pub_.publish(state_msg);
    std_msgs::Float64 ttc_msg;
    ttc_msg.data = ttc;
    dynamic_point_flow_ttc_pub_.publish(ttc_msg);
  }

  REMANIReplanFSM::DynamicPointFlowEvaluation
  REMANIReplanFSM::evaluateDynamicPointFlow(
      const SingulTrajData &traj, double t_cur, const ros::Time &now)
  {
    DynamicPointFlowEvaluation evaluation;
    const auto map = planner_manager_->grid_map_;
    evaluation.snapshot = map->getDynamicPointSnapshot(now);
    if (!evaluation.snapshot.valid)
    {
      evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
      evaluation.reason = evaluation.snapshot.invalid_reason;
      return evaluation;
    }
    if (evaluation.snapshot.points.empty())
    {
      evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::CLEAR;
      return evaluation;
    }

    const double progress_age =
        (now - controller_progress_stamp_).toSec();
    const bool progress_time_basis_valid =
        have_controller_progress_ &&
        std::fabs(controller_trajectory_start_time_ - traj.start_time) < 1e-3 &&
        !controller_progress_stamp_.isZero() &&
        progress_age >= -0.05 &&
        progress_age <= controller_progress_timeout_;
    if (!progress_time_basis_valid)
    {
      evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
      evaluation.reason = "fresh matching controller progress is unavailable";
      return evaluation;
    }

    double execution_scale = controller_trajectory_scale_;
    if (dynamic_point_flow_active_mode_)
    {
      // Safety prediction must use the fastest execution that can reasonably
      // occur before the next check.  Treating the current (possibly low)
      // controller scale as constant makes TTC optimistic when the controller
      // ramps back toward full speed, and a zero scale can deadlock a stopped
      // robot.  The configured resume scale is therefore a conservative lower
      // bound on the execution scale used by the active supervisor.
      execution_scale = std::min(
          1.0, std::max(execution_scale,
                        dynamic_point_flow_active_resume_scale_));
    }
    else if (execution_scale <= 1e-3)
    {
      evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
      evaluation.reason = "controller progress is paused";
      return evaluation;
    }

    const double observation_age =
        std::max(0.0, (now - evaluation.snapshot.stamp).toSec());
    const double valid_wall_horizon =
        map->dynamicPointMaxPredictionTime() - observation_age;
    if (valid_wall_horizon <= 1e-3)
    {
      evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
      evaluation.reason = "no valid point-prediction horizon remains";
      return evaluation;
    }

    DynamicPointFlowEvaluation earliest_collision = evaluation;
    bool found_collision = false;
    auto record_collision = [&](int collision_type,
                                double trajectory_time,
                                double wall_time_ahead,
                                double path_distance,
                                const Eigen::Vector2d &trajectory_position)
    {
      if (found_collision &&
          wall_time_ahead >= earliest_collision.wall_time_ahead)
        return;
      found_collision = true;
      earliest_collision.status =
          DYNAMIC_POINT_FLOW_EVAL_STATUS::COLLISION;
      earliest_collision.collision_type = collision_type;
      earliest_collision.trajectory_time = trajectory_time;
      earliest_collision.wall_time_ahead = wall_time_ahead;
      earliest_collision.path_distance = path_distance;
      earliest_collision.trajectory_position = trajectory_position;
    };

    const Eigen::Vector2d held_position = traj.getPos(t_cur).head(2);
    const bool controller_is_held =
        controller_trajectory_scale_ <= 0.05 ||
        (dynamic_safety_speed_limit_active_ &&
         std::isfinite(current_safety_speed_limit_) &&
         current_safety_speed_limit_ <= 1e-3);
    if (controller_is_held)
    {
      // The hold envelope is meaningful only while the controller is actually
      // paused. Applying it during normal motion can report a collision at a
      // position the robot will already have left and cause an unnecessary
      // stop/replan event.
      const double wall_check_horizon = std::min(
          valid_wall_horizon, global_plan_dynamic_lookahead_time_);
      const double wall_check_step = std::max(
          0.02, global_plan_dynamic_check_time_step_);
      for (double wall_ahead = 0.0;
           wall_ahead <= wall_check_horizon + 1e-9;
           wall_ahead += wall_check_step)
      {
        bool query_valid = true;
        int predicted_coll_type = -1;
        if (planner_manager_->ploy_traj_opt_->
                checkPredictedDynamicObstacleCollision(
                    traj, t_cur, now + ros::Duration(wall_ahead),
                    evaluation.snapshot, predicted_coll_type, query_valid))
        {
          record_collision(predicted_coll_type, t_cur, wall_ahead,
                           0.0, held_position);
          break;
        }
        if (!query_valid)
        {
          evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
          evaluation.reason =
              "hold-envelope query exceeded its valid time basis";
          return evaluation;
        }
      }
    }

    // Check a small execution-scale envelope rather than assuming one frozen
    // controller scale.  Full/nominal resume is the earliest-arrival bound;
    // half speed and the measured scale cover common ramp/slowdown timing.
    std::vector<double> execution_scales{execution_scale};
    if (dynamic_point_flow_active_mode_)
    {
      execution_scales.push_back(std::max(0.1, 0.5 * execution_scale));
      if (controller_trajectory_scale_ > 0.05)
        execution_scales.push_back(std::min(1.0,
                                            controller_trajectory_scale_));
    }
    std::sort(execution_scales.begin(), execution_scales.end());
    execution_scales.erase(
        std::unique(execution_scales.begin(), execution_scales.end(),
                    [](double lhs, double rhs)
                    { return std::fabs(lhs - rhs) < 1e-3; }),
        execution_scales.end());

    for (const double scale : execution_scales)
    {
      const double check_end = std::min(
          traj.duration,
          std::min(t_cur + global_plan_dynamic_lookahead_time_,
                   t_cur + valid_wall_horizon * scale));
      if (check_end <= t_cur + 1e-9)
        continue;

      Eigen::Vector2d previous_position = held_position;
      double path_distance = 0.0;
      double t_check = std::min(
          check_end, t_cur + global_plan_dynamic_check_time_step_);
      while (t_check <= check_end + 1e-9)
      {
        const Eigen::Vector2d trajectory_position =
            traj.getPos(t_check).head(2);
        path_distance +=
            (trajectory_position - previous_position).norm();
        previous_position = trajectory_position;

        const double wall_ahead = (t_check - t_cur) / scale;
        if (found_collision &&
            wall_ahead >= earliest_collision.wall_time_ahead)
          break;

        bool query_valid = true;
        int predicted_coll_type = -1;
        if (planner_manager_->ploy_traj_opt_->
                checkPredictedDynamicObstacleCollision(
                    traj, t_check, now + ros::Duration(wall_ahead),
                    evaluation.snapshot, predicted_coll_type, query_valid))
        {
          record_collision(predicted_coll_type, t_check, wall_ahead,
                           path_distance, trajectory_position);
          break;
        }
        if (!query_valid)
        {
          evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
          evaluation.reason =
              "resume-envelope query exceeded its valid time basis";
          return evaluation;
        }
        if (t_check >= check_end - 1e-9)
          break;
        t_check = std::min(
            check_end,
            t_check + global_plan_dynamic_check_time_step_);
      }
    }

    if (found_collision)
      return earliest_collision;
    evaluation.status = DYNAMIC_POINT_FLOW_EVAL_STATUS::CLEAR;
    return evaluation;
  }

  void REMANIReplanFSM::updateDynamicPointFlowDecision(
      const DynamicPointFlowEvaluation &evaluation,
      const ros::Time &now)
  {
    last_dynamic_point_flow_evaluation_ = evaluation;

    if (evaluation.status ==
        DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE)
    {
      const bool release_point_flow_only_guard =
          !dynamic_point_flow_fail_safe_on_unavailable_ &&
          dynamic_point_flow_hazard_active_ &&
          dynamic_event_required_esdf_seq_ == 0;
      dynamic_point_flow_input_unavailable_ = true;
      dynamic_point_flow_collision_present_ = false;
      dynamic_point_flow_collision_confirmed_ = false;
      dynamic_point_flow_confirm_count_ = 0;
      dynamic_point_flow_clear_since_ = ros::Time(0);
      if (dynamic_point_flow_fail_safe_on_unavailable_)
      {
        dynamic_point_flow_hazard_active_ = true;
        last_dynamic_seen_time_ = now;
        applyDynamicSafetySpeedLimit(0.0);
      }
      else
      {
        // Fail-open applies only to this auxiliary prediction owner. The
        // complete live-map scan later in the same callback remains
        // authoritative. Backdate a prediction-only guard so that scan can
        // release it if and only if no raw occupancy/ESDF owner is present.
        dynamic_point_flow_hazard_active_ = false;
        if (release_point_flow_only_guard)
        {
          last_dynamic_seen_time_ =
              now - ros::Duration(global_plan_dynamic_clear_hold_time_ +
                                  1e-3);
        }
      }
      publishDynamicPointFlowStatus(-1, -2.0);
      ROS_ERROR_THROTTLE(
          1.0,
          "[DynamicPointFlow][active] Input/time basis unavailable (%s); %s.",
          evaluation.reason.c_str(),
          dynamic_point_flow_fail_safe_on_unavailable_ ?
              "holding zero speed" : "falling back to occupancy safety");
      return;
    }

    dynamic_point_flow_input_unavailable_ = false;
    if (evaluation.status == DYNAMIC_POINT_FLOW_EVAL_STATUS::CLEAR)
    {
      dynamic_point_flow_collision_present_ = false;
      dynamic_point_flow_collision_confirmed_ = false;
      dynamic_point_flow_confirm_count_ = 0;
      if (dynamic_point_flow_clear_since_.isZero())
        dynamic_point_flow_clear_since_ = now;
      if ((now - dynamic_point_flow_clear_since_).toSec() >=
          dynamic_point_flow_active_clear_hold_time_)
      {
        dynamic_point_flow_hazard_active_ = false;
      }
      publishDynamicPointFlowStatus(0, -1.0);
      ROS_INFO_THROTTLE(
          1.0,
          "[DynamicPointFlow][active] Predicted horizon clear (seq=%lu).",
          static_cast<unsigned long>(evaluation.snapshot.sequence));
      return;
    }

    if (evaluation.status != DYNAMIC_POINT_FLOW_EVAL_STATUS::COLLISION)
      return;

    dynamic_point_flow_clear_since_ = ros::Time(0);
    dynamic_point_flow_collision_present_ = true;
    dynamic_point_flow_hazard_active_ = true;
    last_dynamic_seen_time_ = now;

    if (evaluation.snapshot.sequence != last_dynamic_point_flow_sequence_)
    {
      const bool confirmation_fresh =
          !last_dynamic_point_flow_collision_time_.isZero() &&
          (now - last_dynamic_point_flow_collision_time_).toSec() <=
              global_plan_dynamic_confirm_timeout_;
      const bool same_collision =
          last_dynamic_point_flow_sequence_ == 0 ||
          (evaluation.trajectory_position -
           last_dynamic_point_flow_collision_position_).norm() <=
              global_plan_dynamic_same_obstacle_radius_;
      if (confirmation_fresh && same_collision)
        ++dynamic_point_flow_confirm_count_;
      else
        dynamic_point_flow_confirm_count_ = 1;
      last_dynamic_point_flow_sequence_ = evaluation.snapshot.sequence;
      last_dynamic_point_flow_collision_time_ = now;
      last_dynamic_point_flow_collision_position_ =
          evaluation.trajectory_position;
    }

    const int required_frames =
        evaluation.wall_time_ahead <=
                dynamic_point_flow_active_immediate_ttc_ ?
            1 : dynamic_point_flow_active_confirm_frames_;
    dynamic_point_flow_collision_confirmed_ =
        dynamic_point_flow_confirm_count_ >= required_frames;

    double speed_limit = global_plan_dynamic_slowdown_max_speed_;
    if (evaluation.wall_time_ahead <=
        dynamic_point_flow_active_emergency_ttc_)
    {
      speed_limit = 0.0;
    }
    else if (evaluation.wall_time_ahead <=
             dynamic_point_flow_active_slowdown_ttc_)
    {
      const double slowdown_span = std::max(
          1e-3,
          dynamic_point_flow_active_slowdown_ttc_ -
              dynamic_point_flow_active_emergency_ttc_);
      const double ratio = std::max(
          0.0, std::min(
                   1.0,
                   (evaluation.wall_time_ahead -
                    dynamic_point_flow_active_emergency_ttc_) /
                       slowdown_span));
      speed_limit = global_plan_dynamic_slowdown_min_speed_ +
          ratio * (global_plan_dynamic_slowdown_max_speed_ -
                   global_plan_dynamic_slowdown_min_speed_);
    }
    applyDynamicSafetySpeedLimit(speed_limit);

    const bool emergency =
        evaluation.wall_time_ahead <=
        dynamic_point_flow_active_emergency_ttc_;
    publishDynamicPointFlowStatus(
        emergency ? 3 :
        (dynamic_point_flow_collision_confirmed_ ? 2 : 1),
        evaluation.wall_time_ahead);
    ROS_WARN_THROTTLE(
        0.5,
        "[DynamicPointFlow][active] collision type=%d TTC=%.2f s, path=%.2f m, confirm=%d/%d, speed<=%.2f m/s, seq=%lu.",
        evaluation.collision_type,
        evaluation.wall_time_ahead,
        evaluation.path_distance,
        dynamic_point_flow_confirm_count_, required_frames,
        speed_limit,
        static_cast<unsigned long>(evaluation.snapshot.sequence));
  }

  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init,
                                           bool flag_randomPolyTraj,
                                           bool dynamic_repair){
    // Current callers freeze a planning generation only when an optimization
    // is actually due. Do not capture a second, newer generation here: doing
    // so after the outer frozen generation becomes ready would turn normal
    // LiDAR progress into a false optimization failure. Existing raw/event
    // requirements remain fail-closed, and the final live-occupancy check
    // below covers frames newer than the frozen planning generation.
    if (!waitForRequiredESDF("REBOUND_REPLAN_GUARD"))
      return false;

    const bool preserve_executing_trajectory =
        have_published_trajectory_ &&
        !last_published_trajectory_.singul_traj.empty();
    const SingulTrajData executing_trajectory =
        last_published_trajectory_;
    auto finish_failed_planning_attempt = [&]()
    {
      if (preserve_executing_trajectory)
        planner_manager_->traj_container_.singul_traj_data =
            executing_trajectory;
      else
        planner_manager_->traj_container_.singul_traj_data.clearSingulTraj();

      return false;
    };

    bool reach_horizon;
    // Warm deformation keeps the historical target metadata because the
    // previous-trajectory initializer depends on it.  Once that bounded repair
    // fails, the A* topology fallback gets a genuinely local target projected
    // from the robot's current progress instead of optimizing the remaining
    // global route (global_plan historically changes its horizon to 1000 m).
    const bool use_dynamic_local_target =
        dynamic_repair && dynamic_local_replan_enable_ && flag_use_poly_init;
    const double target_horizon = use_dynamic_local_target ?
        dynamic_replan_horizon_ : planning_horizen_;
    if (!planner_manager_->getLocalTarget(
            target_horizon, start_pos_, start_yaw_, end_pt_, end_yaw_,
            local_target_pt_, local_target_vel_, local_target_acc_,
            reach_horizon,
            /*project_from_current=*/use_dynamic_local_target))
      return finish_failed_planning_attempt();
    if (use_dynamic_local_target && dynamic_replan_hold_arm_)
    {
      local_target_pt_.segment(mobile_base_dim_, manipulator_dim_) =
          start_pos_.segment(mobile_base_dim_, manipulator_dim_);
      local_target_vel_.segment(mobile_base_dim_, manipulator_dim_).setZero();
      local_target_acc_.segment(mobile_base_dim_, manipulator_dim_).setZero();
    }
    bool local_target_gripper;
    if(reach_horizon){
      local_target_gripper = gripper_state_;
    }else{
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }
    local_target_acc_.setZero();
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0)); // global traj is foreward, no need to take singul into account
    local_target_vel_.setZero();
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time;
    const bool guarded_dynamic_stop_handoff =
        dynamic_safety_speed_limit_active_ &&
        std::isfinite(current_safety_speed_limit_) &&
        current_safety_speed_limit_ <= 1e-3;
    
    if(have_local_traj_)
    {
      desired_start_time = ros::Time::now().toSec() +
          (guarded_dynamic_stop_handoff ? 0.0 :
           (dynamic_repair ? dynamic_replan_handoff_time_ :
                             replan_trajectory_time_));
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = start_singul_;
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
    }else{
      desired_start_time = ros::Time::now().toSec();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }
    // std::cout << "desired_start_singul: " << desired_start_singul << std::endl;
    double init_time, opt_time;
    
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc,desired_start_jerk, desired_start_yaw, desired_start_singul, gripper_state_,
        desired_start_time, local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, have_local_traj_, init_time, opt_time);
    have_new_target_ = false;

    if (plan_success){
      int coll_type = -1;
      std::uint64_t prediction_validation_seq = 0;
      SingulTrajData &candidate =
          planner_manager_->traj_container_.singul_traj_data;
      if (guarded_dynamic_stop_handoff && !candidate.singul_traj.empty())
      {
        // Optimization is synchronous. A start stamp captured before it ran
        // may already be hundreds of milliseconds old at publication, while
        // the guarded robot has stayed at trajectory t=0. Rebase only the
        // wall-clock metadata; the measured-state polynomial is unchanged.
        const ros::Time activation_time = ros::Time::now();
        double segment_start = activation_time.toSec();
        candidate.start_time = segment_start;
        candidate.start_time_ros = activation_time;
        for (auto &segment : candidate.singul_traj)
        {
          segment.start_time = segment_start;
          segment.end_time = segment_start + segment.duration;
          segment_start = segment.end_time;
        }
      }
      else if (dynamic_repair && !candidate.singul_traj.empty())
      {
        // A moving repair is constructed from a predicted future state. Never
        // publish it after that splice time has already passed; the topology
        // fallback will stop, re-anchor at feedback and rebase safely instead.
        const double activation_lead =
            candidate.start_time - ros::Time::now().toSec();
        constexpr double minimum_activation_lead = 0.02;
        if (activation_lead < minimum_activation_lead)
        {
          ROS_WARN("[FSM] Rejecting late moving repair: only %.3f s remains "
                   "before activation (minimum %.3f s).",
                   activation_lead, minimum_activation_lead);
          return finish_failed_planning_attempt();
        }
      }
      if (global_plan_ && !validateStaticTrajectory(candidate, coll_type)){
        ROS_WARN("[FSM] Rejecting replan candidate: static or swept point-flow planning collision type=%d remains after optimization.", coll_type);
        return finish_failed_planning_attempt();
      }
      if (dynamic_point_flow_active_mode_)
      {
        const ros::Time validation_now = ros::Time::now();
        const plan_env::DynamicPointSnapshot latest_snapshot =
            planner_manager_->grid_map_->getDynamicPointSnapshot(
                validation_now);
        if (!latest_snapshot.valid)
        {
          if (dynamic_point_flow_fail_safe_on_unavailable_)
          {
            DynamicPointFlowEvaluation unavailable;
            unavailable.status =
                DYNAMIC_POINT_FLOW_EVAL_STATUS::UNAVAILABLE;
            unavailable.snapshot = latest_snapshot;
            unavailable.reason = latest_snapshot.invalid_reason.empty() ?
                "latest point-flow snapshot is invalid" :
                latest_snapshot.invalid_reason;
            updateDynamicPointFlowDecision(unavailable, validation_now);
            ROS_ERROR("[FSM] Rejecting replan candidate: latest point-flow "
                      "snapshot is unavailable (%s) and fail-safe mode is enabled.",
                      unavailable.reason.c_str());
            return finish_failed_planning_attempt();
          }
          ROS_WARN_THROTTLE(
              1.0,
              "[FSM] Latest point-flow snapshot unavailable (%s); explicit "
              "fail-open configuration leaves occupancy safety authoritative.",
              latest_snapshot.invalid_reason.c_str());
        }
        else if (!validatePredictedDynamicTrajectory(
                     candidate, latest_snapshot, coll_type))
        {
          ROS_WARN("[FSM] Rejecting replan candidate: latest time-aligned "
                   "point-flow validation failed (collision type=%d or "
                   "prediction horizon unavailable, seq=%lu).",
                   coll_type,
                   static_cast<unsigned long>(latest_snapshot.sequence));
          return finish_failed_planning_attempt();
        }
        else
        {
          prediction_validation_seq = latest_snapshot.sequence;
        }
      }
      const std::uint64_t validation_seq = global_plan_ ?
          planner_manager_->grid_map_->getDynamicObservationSeq() : 0;
      if (global_plan_ &&
          !validateDynamicTrajectory(candidate, coll_type)){
        const auto map = planner_manager_->grid_map_;
        ROS_WARN("[FSM] Rejecting replan candidate: live obstacle collision "
                 "type=%d remains (map_seq=%lu, esdf_seq=%lu).",
                 coll_type,
                 static_cast<unsigned long>(
                     map->getDynamicObservationSeq()),
                 static_cast<unsigned long>(
                     map->getESDFObservationSeq()));
        return finish_failed_planning_attempt();
      }
      init_time_list_.push_back(init_time);
      opt_time_list_.push_back(opt_time);
      total_time_list_.push_back(init_time + opt_time);
      sendPolyTrajROSMsg();
      have_local_traj_ = true;
      last_published_trajectory_ =
          planner_manager_->traj_container_.singul_traj_data;
      have_published_trajectory_ = true;
      if (global_plan_)
        last_published_dynamic_validation_seq_ = validation_seq;
      last_published_point_flow_validation_seq_ =
          prediction_validation_seq;

      // vis local traj
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    } else {
      return finish_failed_planning_attempt();
    }

    return plan_success;
  }

  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    dynamic_replan_retry_pending_ = false;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::PolynomialTraj msg;
    msg.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    poly_traj_pub_.publish(msg);
    if(local_execution_owner_){std_msgs::String transaction;transaction.data=execution_goal_key_+"\n1\n"+executionHex(msg);execution_reference_pub_.publish(transaction);}
    have_published_trajectory_ = false;

    return true;
  }

} // namespace remani_planner

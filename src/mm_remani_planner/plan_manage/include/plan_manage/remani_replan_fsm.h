#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <fstream>
#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.hpp>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <plan_manage/planning_visualization.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <traj_utils/Assignment.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <sstream>
#include <fstream>
#include <iostream>
using std::vector;

namespace remani_planner
{

  class REMANIReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      // WAIT_GRIPPER,
      EMERGENCY_STOP,
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2
    };

    enum class DYNAMIC_POINT_FLOW_EVAL_STATUS
    {
      NOT_DUE,
      UNAVAILABLE,
      CLEAR,
      COLLISION
    };

    struct DynamicPointFlowEvaluation
    {
      DYNAMIC_POINT_FLOW_EVAL_STATUS status =
          DYNAMIC_POINT_FLOW_EVAL_STATUS::NOT_DUE;
      plan_env::DynamicPointSnapshot snapshot;
      int collision_type = -1;
      double trajectory_time = std::numeric_limits<double>::infinity();
      double wall_time_ahead = std::numeric_limits<double>::infinity();
      double path_distance = std::numeric_limits<double>::infinity();
      Eigen::Vector2d trajectory_position = Eigen::Vector2d::Zero();
      std::string reason;
    };
    
    /* planning utils */
    MMPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    int wpt_id_;
    double no_replan_thresh_, replan_thresh_;
    // double waypoints_[50][4];
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> waypoints_;
    std::vector<double> waypoints_yaw_;
    std::vector<bool> waypoint_gripper_close_;
    bool gripper_flag_; // 是否需要夹爪操作
    int waypoint_num_;
    double planning_horizen_;
    double emergency_time_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;
    double replan_feedback_position_threshold_;
    double replan_feedback_yaw_threshold_;
    double replan_feedback_arm_threshold_;
    double goal_position_tolerance_;
    double goal_yaw_tolerance_;
    double goal_arm_tolerance_;
    double goal_yaw_alignment_timeout_;
    double actual_state_collision_check_interval_;
    double actual_base_emergency_margin_;
    double esdf_freshness_wait_timeout_;
    bool dynamic_local_replan_enable_;
    bool dynamic_replan_hold_arm_;
    double dynamic_replan_horizon_;
    double dynamic_replan_handoff_time_;
    double dynamic_astar_search_time_;
    double dynamic_astar_replan_wall_time_;
    bool dynamic_point_flow_shadow_mode_;
    double dynamic_point_flow_shadow_check_interval_;
    bool dynamic_point_flow_active_mode_;
    bool dynamic_point_flow_active_replan_;
    bool dynamic_point_flow_fail_safe_on_unavailable_;
    int dynamic_point_flow_active_confirm_frames_;
    double dynamic_point_flow_active_immediate_ttc_;
    double dynamic_point_flow_active_emergency_ttc_;
    double dynamic_point_flow_active_slowdown_ttc_;
    double dynamic_point_flow_active_clear_hold_time_;
    double dynamic_point_flow_active_resume_scale_;
    double global_plan_dynamic_lookahead_time_;
    double global_plan_dynamic_check_interval_;
    double global_plan_dynamic_check_time_step_;
    double global_plan_static_check_time_step_;
    double global_plan_dynamic_lookahead_distance_;
    double global_plan_dynamic_far_sample_distance_;
    double global_plan_dynamic_immediate_confirm_distance_;
    int global_plan_dynamic_confirm_frames_;
    double global_plan_dynamic_confirm_timeout_;
    double global_plan_dynamic_clear_hold_time_;
    double global_plan_dynamic_replan_hold_time_;
    double global_plan_dynamic_same_obstacle_radius_;
    double global_plan_dynamic_slowdown_distance_;
    double global_plan_dynamic_slowdown_min_speed_;
    double global_plan_dynamic_slowdown_max_speed_;
    double global_plan_dynamic_emergency_deceleration_;
    double global_plan_dynamic_emergency_reaction_time_;
    double global_plan_dynamic_emergency_margin_;
    double global_plan_dynamic_emergency_min_distance_;
    double global_plan_dynamic_emergency_max_distance_;
    double controller_progress_timeout_;
    int replan_fail_time_;
    int new_traj_fail_time_;
    int global_plan_max_failures_;
    int global_plan_trial_times_;
    double global_plan_max_wall_time_;
    int rear_goal_plan_max_failures_;
    int rear_goal_plan_trial_times_;
    double rear_goal_plan_distance_;
    double rear_goal_plan_cos_;
    double time_for_gripper_;
    bool global_plan_;

    int mobile_base_dim_, manipulator_dim_, traj_dim_;
    double mobile_base_non_singul_vel_;
    Eigen::VectorXd default_arm_config_;                          // standby joint angles
    bool has_default_arm_;
    bool default_arm_from_config_;
    bool manual_restore_standby_after_goal_;
    double manual_restore_tolerance_;
    bool manual_use_work_arm_waypoint_;
    bool manual_target_end_arm_standby_;
    bool manual_restore_pending_;
    bool manual_restore_running_;
    int manual_work_arm_waypoint_id_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::VectorXd mm_state_pos_, mm_state_vel_, mm_state_acc_, init_state_; // odometry state
    bool gripper_state_, rcv_gripper_state_;
    int mm_car_singul_;
    Eigen::Quaterniond mm_car_orient_;
    double mm_car_yaw_, mm_car_yaw_rate_;

    Eigen::VectorXd start_pos_, start_vel_, start_acc_, start_jer_; // start state
    int start_singul_;
    double start_yaw_, end_yaw_;
    Eigen::VectorXd end_pt_;                                       // goal state
    Eigen::VectorXd local_target_pt_, local_target_vel_, local_target_acc_;                     // local target state
    int local_target_singul_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;
    bool try_plan_after_emergency_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, joint_state_sub_, gripper_state_sub_, trigger_sub_, assignment_sub_;
    ros::Subscriber trajectory_progress_sub_, tracking_fault_sub_;
    ros::Publisher replan_pub_, new_pub_, poly_traj_pub_, data_disp_pub_, gripper_cmd_pub_, map_state_pub_;
    ros::Publisher safety_speed_limit_pub_;
    ros::Publisher dynamic_point_flow_state_pub_;
    ros::Publisher dynamic_point_flow_ttc_pub_;

    ros::Publisher reached_pub_, start_pub_;

    ros::Time t_last_Astar_;
    ros::Time last_global_plan_dynamic_check_time_;
    ros::Time last_dynamic_point_flow_shadow_check_time_;
    ros::Time last_actual_state_collision_check_time_;
    ros::Time last_dynamic_collision_time_;
    ros::Time last_dynamic_seen_time_;
    ros::Time dynamic_replan_hold_until_;
    ros::Time next_dynamic_replan_retry_time_;
    std::uint64_t last_dynamic_observation_seq_;
    // Raw live-map generation against which the currently published
    // trajectory was last validated. A collision from a newer generation is
    // new evidence and must not be hidden by the old event's debounce hold.
    std::uint64_t last_published_dynamic_validation_seq_;
    // Prediction generation used to validate the trajectory most recently
    // sent to the controller. A newer point-flow generation which still
    // intersects that trajectory is new evidence and bypasses time debounce.
    std::uint64_t last_published_point_flow_validation_seq_;
    std::uint64_t dynamic_event_required_esdf_seq_;
    // Frozen only while a non-collision planning entry waits for ESDF. Keep
    // this separate from the immutable raw-obstacle event generation.
    std::uint64_t planning_boundary_required_esdf_seq_;
    ros::SteadyTime dynamic_esdf_wait_start_;
    bool dynamic_esdf_sync_fallback_requested_;
    int dynamic_collision_confirm_count_;
    double current_safety_speed_limit_;
    Eigen::Vector2d last_dynamic_confirmation_pos_;
    Eigen::Vector2d last_dynamic_replan_collision_pos_;
    int last_dynamic_replan_collision_type_;
    bool last_dynamic_replan_collision_from_point_flow_;
    bool has_last_dynamic_confirmation_pos_;
    bool has_last_dynamic_replan_collision_;
    bool dynamic_safety_speed_limit_active_;
    bool dynamic_replan_retry_pending_;
    bool dynamic_event_astar_attempted_;
    bool dynamic_event_from_point_flow_;
    bool dynamic_point_flow_collision_present_;
    bool dynamic_point_flow_hazard_active_;
    bool dynamic_point_flow_collision_confirmed_;
    bool dynamic_point_flow_input_unavailable_;
    bool have_controller_progress_;
    bool tracking_fault_active_;
    // A tracker stop is an independent safety owner.  It must not be folded
    // into the dynamic-obstacle guard: doing so can leave the latched speed
    // limit at zero when no dynamic observation ever existed.
    bool tracking_safety_stop_active_;
    bool tracking_recovery_pending_;
    bool tracking_recovery_candidate_published_;
    // The planner mutates traj_container_ while constructing a candidate.
    // Keep an independent copy of what was actually sent to the controller so
    // a rejected/failed candidate can never replace the safety-check basis.
    bool have_published_trajectory_;
    SingulTrajData last_published_trajectory_;
    double controller_trajectory_start_time_;
    double controller_trajectory_time_;
    double controller_trajectory_scale_;
    ros::Time controller_progress_stamp_;
    ros::Time goal_yaw_alignment_start_time_;
    int dynamic_event_replan_attempts_;
    std::uint64_t last_dynamic_replan_attempt_generation_;
    int dynamic_point_flow_confirm_count_;
    std::uint64_t last_dynamic_point_flow_sequence_;
    ros::Time last_dynamic_point_flow_collision_time_;
    ros::Time dynamic_point_flow_clear_since_;
    ros::SteadyTime dynamic_event_first_detected_steady_;
    Eigen::Vector2d last_dynamic_point_flow_collision_position_;
    DynamicPointFlowEvaluation last_dynamic_point_flow_evaluation_;

    int map_state_;

    std::vector<double> init_time_list_;
    std::vector<double> opt_time_list_;
    std::vector<double> total_time_list_;
    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj,
                           bool dynamic_repair = false);           // front-end and back-end method
    bool callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul); // front-end and back-end method
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init, bool allow_random_retry = true,
                           double optimization_wall_time = -1.0,
                           bool dynamic_repair = false);
    
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    /* ROS functions */
    void execFSMCallback(const ros::TimerEvent &e);
    void checkCollisionCallback(const ros::TimerEvent &e);
    void publishSafetySpeedLimit(double speed_limit);
    void applyDynamicSafetySpeedLimit(double speed_limit);
    void clearDynamicSafetySpeedLimit(const ros::Time &now, bool force = false);
    void latchDynamicEventRequiredESDFSeq(std::uint64_t observation_seq);
    void raiseDynamicEventRequiredESDFSeq(std::uint64_t observation_seq);
    bool waitForRequiredESDF(const char *planning_context,
                             bool capture_planning_boundary = false);
    bool hasUnmetRequiredESDF() const;
    void resetDynamicReplanEvent(bool preserve_esdf_requirement = false);
    double getCurrentTrajectoryTime(const SingulTrajData &traj, const ros::Time &now) const;
    double getCurrentTrajectoryScale(const SingulTrajData &traj, const ros::Time &now) const;
    bool validateStaticTrajectory(const SingulTrajData &traj, int &coll_type) const;
    bool validateDynamicTrajectory(const SingulTrajData &traj, int &coll_type) const;
    bool validatePredictedDynamicTrajectory(
        const SingulTrajData &traj,
        const plan_env::DynamicPointSnapshot &snapshot,
        int &coll_type) const;
    DynamicPointFlowEvaluation evaluateDynamicPointFlow(
        const SingulTrajData &traj, double t_cur, const ros::Time &now);
    void updateDynamicPointFlowDecision(
        const DynamicPointFlowEvaluation &evaluation,
        const ros::Time &now);
    void publishDynamicPointFlowStatus(int state, double ttc);
    void resetDynamicPointFlowDecision();
    bool isRearDistantGoal() const;
    bool planNextWaypoint(const Eigen::VectorXd next_wp, const double nect_yaw,
                          bool preserve_active_target_on_failure = false);
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg);
    void mmManiOdomCallback(const sensor_msgs::JointStateConstPtr &msg);
    void gripperCallback(const std_msgs::Bool::ConstPtr &msg);
    void trajectoryProgressCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
    void trackingFaultCallback(const std_msgs::Bool::ConstPtr &msg);
    void sendPolyTrajROSMsg();
    bool frontEndPathSearching();
    bool checkCollision();

  public:
    REMANIReplanFSM(/* args */)
    {
    }
    ~REMANIReplanFSM();


    void init(ros::NodeHandle &nh);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace remani_planner

#endif

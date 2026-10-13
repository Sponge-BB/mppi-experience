#ifndef __CONTROL_FSM_H
#define __CONTROL_FSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include "mm_controller/controller_utils.hpp"
#include "mm_controller/polynomial_trajectory.h"

namespace MMController{


class MMControllerFSM
{
public:

	State_Data_t state_data;
	Command_Data_t cmd_data;
	Trajectory_Data_t trajectory_data;

	Eigen::VectorXd stay_joint_;
	Eigen::Vector2d stay_pos_;
	double stay_yaw_;

	enum State_t{
		AUTO_STAY,
		CMD_CTRL,
	};

    enum Exec_Traj_State_t{
		STAY = 10, // execute the hover trajectory 
		POLY_TRAJ = 11, // execute the polynomial trajectory
	};

	State_t last_state_;

	MMControllerFSM(const ros::NodeHandle& nh);

	void process();
	void CMD_CTRL_process();
	bool state_is_received(const ros::Time &now_time);
	void safetySpeedLimitCallback(const std_msgs::Float64::ConstPtr &msg);

private:
	int mobile_base_dof_, manipulator_dof_;
	ros::Publisher traj_start_trigger_pub_, trajectory_progress_pub_;
	ros::Publisher car_cmd_pub_, mani_cmd_pub_, gripper_cmd_pub_;
	ros::Publisher arm_tracking_error_pub_;
	ros::Publisher base_tracking_error_pub_;
	State_t fsm_state_; // Should only be changed in PX4CtrlFSM::process() function!
	Exec_Traj_State_t exec_traj_state_;
	Eigen::VectorXd init_theta_;
	bool capture_startup_standby_;
	bool startup_standby_captured_;
	bool init_gripper_close_;
	int last_singul_;
	// Low-speed path tangents are noisy during trajectory handoff. Keep yaw and
	// curvature feedforward bounded until the base has meaningful forward speed.
	double yaw_hold_speed_;
	double curvature_ff_start_speed_;
	double curvature_ff_full_speed_;
	double max_feedforward_omega_;
	double odom_timeout_;
	double virtual_time_min_reference_speed_;
	double virtual_time_scale_rise_rate_;
	double virtual_time_scale_fall_rate_;
	double arm_tracking_slow_error_;
	double arm_tracking_stop_error_;
	double arm_tracking_resume_error_;
	double arm_tracking_min_time_scale_;
	double arm_tracking_joint_timeout_;
	double base_tracking_slow_distance_;
	double base_tracking_stop_distance_;
	double base_tracking_resume_distance_;
	double base_tracking_slow_yaw_;
	double base_tracking_stop_yaw_;
	double base_tracking_resume_yaw_;
	double base_tracking_min_time_scale_;
	double terminal_yaw_sample_window_;
	double terminal_yaw_min_speed_;
	double safety_speed_limit_;
	double virtual_traj_time_;
	double virtual_time_scale_;
	double max_arm_tracking_error_;
	double base_tracking_position_error_;
	double base_tracking_yaw_error_;
	double terminal_yaw_;
	bool arm_tracking_hold_active_;
	bool base_tracking_hold_active_;
	bool have_terminal_yaw_;
	std::uint64_t active_trajectory_seq_;
	ros::Time last_virtual_clock_stamp_;
	ros::Time trajectory_activation_stamp_;

	// Handles
	ros::NodeHandle nh_;

	void publish_ctrl_cmd(const ros::Time &stamp);
	void update_stay_state(bool mani_goto_init);
	void publish_trigger(const nav_msgs::Odometry &odom_msg);
	void syncTrajectoryClock(const ros::Time &now);
	void advanceTrajectoryClock(const ros::Time &now);
	void updateTerminalYaw();
	void publishTrajectoryProgress(const ros::Time &stamp);
	void cal_traj_ctrl_input(double duration);
	void cal_stay_ctrl_input();
	void limitErr(Eigen::VectorXd &x, const double x_low, const double x_upp);
};

}

#endif

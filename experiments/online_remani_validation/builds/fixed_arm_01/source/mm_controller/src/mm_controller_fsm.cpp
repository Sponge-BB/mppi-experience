#include "mm_controller/mm_controller_fsm.hpp"
using namespace std;

namespace MMController{

MMControllerFSM::MMControllerFSM(const ros::NodeHandle& nh) : nh_(nh){
	nh_.param("mm/mobile_base_dof", mobile_base_dof_, -1);
	nh_.param("mm/manipulator_dof", manipulator_dof_, -1);
	nh_.param("controller/yaw_hold_speed", yaw_hold_speed_, 0.04);
	nh_.param("controller/curvature_ff_start_speed", curvature_ff_start_speed_, 0.04);
	nh_.param("controller/curvature_ff_full_speed", curvature_ff_full_speed_, 0.10);
	nh_.param("controller/max_feedforward_omega", max_feedforward_omega_, 0.60);
	nh_.param("controller/odom_timeout", odom_timeout_, 0.25);
	nh_.param("controller/virtual_time_min_reference_speed", virtual_time_min_reference_speed_, 0.05);
	nh_.param("controller/virtual_time_scale_rise_rate", virtual_time_scale_rise_rate_, 0.8);
	nh_.param("controller/virtual_time_scale_fall_rate", virtual_time_scale_fall_rate_, 2.0);
	nh_.param("controller/arm_tracking_slow_error", arm_tracking_slow_error_, 0.10);
	nh_.param("controller/arm_tracking_stop_error", arm_tracking_stop_error_, 0.25);
	nh_.param("controller/arm_tracking_resume_error", arm_tracking_resume_error_, 0.06);
	nh_.param("controller/arm_tracking_min_time_scale", arm_tracking_min_time_scale_, 0.10);
	nh_.param("controller/arm_tracking_joint_timeout", arm_tracking_joint_timeout_, 0.25);
	nh_.param("controller/base_tracking_slow_distance", base_tracking_slow_distance_, 0.08);
	nh_.param("controller/base_tracking_stop_distance", base_tracking_stop_distance_, 0.16);
	nh_.param("controller/base_tracking_resume_distance", base_tracking_resume_distance_, 0.05);
	nh_.param("controller/base_tracking_slow_yaw", base_tracking_slow_yaw_, 0.25);
	nh_.param("controller/base_tracking_stop_yaw", base_tracking_stop_yaw_, 0.55);
	nh_.param("controller/base_tracking_resume_yaw", base_tracking_resume_yaw_, 0.15);
	nh_.param("controller/base_tracking_min_time_scale", base_tracking_min_time_scale_, 0.10);
	nh_.param("controller/terminal_yaw_sample_window", terminal_yaw_sample_window_, 1.0);
	nh_.param("controller/terminal_yaw_min_speed", terminal_yaw_min_speed_, 0.01);
	yaw_hold_speed_ = std::max(0.0, yaw_hold_speed_);
	curvature_ff_start_speed_ = std::max(0.0, curvature_ff_start_speed_);
	curvature_ff_full_speed_ = std::max(curvature_ff_start_speed_ + 1e-3,
	                                    curvature_ff_full_speed_);
	max_feedforward_omega_ = std::max(0.0, max_feedforward_omega_);
	odom_timeout_ = std::max(0.05, odom_timeout_);
	virtual_time_min_reference_speed_ = std::max(0.01, virtual_time_min_reference_speed_);
	virtual_time_scale_rise_rate_ = std::max(0.01, virtual_time_scale_rise_rate_);
	virtual_time_scale_fall_rate_ = std::max(0.01, virtual_time_scale_fall_rate_);
	arm_tracking_slow_error_ = std::max(0.01, arm_tracking_slow_error_);
	arm_tracking_stop_error_ =
		std::max(arm_tracking_slow_error_ + 0.01, arm_tracking_stop_error_);
	arm_tracking_resume_error_ =
		std::max(0.0, std::min(arm_tracking_slow_error_, arm_tracking_resume_error_));
	arm_tracking_min_time_scale_ =
		std::max(0.0, std::min(1.0, arm_tracking_min_time_scale_));
	arm_tracking_joint_timeout_ = std::max(0.05, arm_tracking_joint_timeout_);
	base_tracking_slow_distance_ = std::max(0.01, base_tracking_slow_distance_);
	base_tracking_stop_distance_ = std::max(
		base_tracking_slow_distance_ + 0.01, base_tracking_stop_distance_);
	base_tracking_resume_distance_ = std::max(
		0.0, std::min(base_tracking_slow_distance_, base_tracking_resume_distance_));
	base_tracking_slow_yaw_ = std::max(0.01, base_tracking_slow_yaw_);
	base_tracking_stop_yaw_ = std::max(
		base_tracking_slow_yaw_ + 0.01, base_tracking_stop_yaw_);
	base_tracking_resume_yaw_ = std::max(
		0.0, std::min(base_tracking_slow_yaw_, base_tracking_resume_yaw_));
	base_tracking_min_time_scale_ = std::max(
		0.0, std::min(1.0, base_tracking_min_time_scale_));
	terminal_yaw_sample_window_ = std::max(0.05, terminal_yaw_sample_window_);
	terminal_yaw_min_speed_ = std::max(1e-4, terminal_yaw_min_speed_);
	safety_speed_limit_ = -1.0;
	virtual_traj_time_ = 0.0;
	virtual_time_scale_ = 1.0;
	max_arm_tracking_error_ = 0.0;
	base_tracking_position_error_ = 0.0;
	base_tracking_yaw_error_ = 0.0;
	terminal_yaw_ = 0.0;
	arm_tracking_hold_active_ = false;
	base_tracking_hold_active_ = false;
	have_terminal_yaw_ = false;
	active_trajectory_seq_ = 0;
	last_virtual_clock_stamp_ = ros::Time(0);
	trajectory_activation_stamp_ = ros::Time(0);

	std::vector<double> init_state;
    nh.getParam("fsm/init_state", init_state);
	init_theta_.resize(manipulator_dof_);
    for(int i = 0; i < manipulator_dof_; i++){
        init_theta_(i) = init_state[i + mobile_base_dof_] / 180.0 * M_PI;
    }
	nh.getParam("fsm/init_gripper_close", init_gripper_close_);
	nh.param("fsm/capture_startup_standby", capture_startup_standby_, false);
	startup_standby_captured_ = false;

	last_singul_ = 0;

	car_cmd_pub_  = nh_.advertise<geometry_msgs::Twist>("car_cmd", 10);
	mani_cmd_pub_ = nh_.advertise<control_msgs::JointTrajectoryControllerState>("joint_cmd", 10);
	gripper_cmd_pub_ = nh_.advertise<std_msgs::Bool>("gripper_cmd", 10);
    traj_start_trigger_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
	trajectory_progress_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("trajectory_progress", 1);
	arm_tracking_error_pub_ = nh_.advertise<std_msgs::Float64>("arm_tracking_error", 1);
	base_tracking_error_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("base_tracking_error", 1);

	state_data.setParam(manipulator_dof_);
	fsm_state_ = AUTO_STAY;
	last_state_ = CMD_CTRL;
	exec_traj_state_ = STAY;
	stay_joint_ = Eigen::VectorXd::Zero(manipulator_dof_);
	stay_yaw_ = 0;
	
	cmd_data.joint_pos_cmd = Eigen::VectorXd::Zero(manipulator_dof_);
	cmd_data.joint_vel_cmd = Eigen::VectorXd::Zero(manipulator_dof_);
	cmd_data.joint_effort_cmd = Eigen::VectorXd::Zero(manipulator_dof_);
}

/* 
        Finite State Machine

           system start
               /
              /
             v           
------> AUTO_STAY          
|         ^   |             
|         |   |             
|         |	  |     
|         |   |
|         |   v
-------- CMD_CTRL

*/

void MMControllerFSM::process(){
	// return;
	ros::Time now_time = ros::Time::now();
	if(!state_data.have_car_ || !state_data.have_mani_) return;
	syncTrajectoryClock(now_time);

	static bool is_first_time = true;
	if(is_first_time){
		is_first_time = false;
		if(capture_startup_standby_ && state_data.have_mani_){
			init_theta_ = state_data.joint_p;
			startup_standby_captured_ = true;
			ROS_INFO_STREAM("[MMctrl] Capture startup arm stand-by pose: " << init_theta_.transpose());
		}
		update_stay_state(false);
		std_msgs::Bool gripper_cmd;
		gripper_cmd.data = init_gripper_close_;
		gripper_cmd_pub_.publish(gripper_cmd);
	}
	
	switch (fsm_state_){
		case AUTO_STAY:
		{
			if(last_state_ == CMD_CTRL){
				last_state_ = AUTO_STAY;
				update_stay_state(true);
			}
			cal_stay_ctrl_input();
			fsm_state_ = CMD_CTRL;
			exec_traj_state_ = STAY;
			ROS_INFO("\033[32m[MMctrl] AUTO_STAY(L1) --> CMD_CTRL(L2)\033[32m");
			publish_trigger(state_data.odom_msg);
			ROS_INFO("\033[32m[MMctrl] TRIGGER sent, allow user command.\033[32m");
			break;
		}

		case CMD_CTRL:
		{
			last_state_ = CMD_CTRL;
			CMD_CTRL_process();
			break;
		}
		default:
			break;
	}
	
	if (fsm_state_ == AUTO_STAY || fsm_state_ == CMD_CTRL){
		publish_ctrl_cmd(now_time);
		publishTrajectoryProgress(now_time);
	}
}
/* 
    Finite State Machine

       CMD_CTRL
           /
          /
         v
      STAY 
      ^   |             
      |   |             
      |   |             
      |   |             
      |   |             
      |   v             
    POLY_TRAJ          


*/
void MMControllerFSM::CMD_CTRL_process()
{
	ros::Time now_time = ros::Time::now();
	advanceTrajectoryClock(now_time);
	switch (exec_traj_state_){
		case STAY:
		{
			if(trajectory_data.exec_traj == 1 &&
			   !trajectory_data.singul_traj_data.singul_traj.empty() &&
			   now_time >= trajectory_activation_stamp_){
				update_stay_state(false);
				cal_traj_ctrl_input(virtual_traj_time_);

				exec_traj_state_ = POLY_TRAJ;
				ROS_INFO("[MMctrl] Receive the trajectory. STAY --> POLY_TRAJ");
			}else{
				if(now_time < trajectory_activation_stamp_)
					update_stay_state(false);
				cal_stay_ctrl_input();
			}
			break;
		}
		
		
		case POLY_TRAJ:
		{
			const bool trajectory_finished =
				!trajectory_data.singul_traj_data.singul_traj.empty() &&
				virtual_traj_time_ >= trajectory_data.singul_traj_data.duration - 1e-6;
			if(trajectory_data.exec_traj != 1 ||
			   trajectory_data.singul_traj_data.singul_traj.empty() ||
			   trajectory_finished){
				if(trajectory_data.exec_traj != -1){
					// tracking the end point of the trajectory
					// the hover pose is the end point of the trajectory
					auto & traj_info = trajectory_data.singul_traj_data.singul_traj.back().traj;
					stay_joint_ = traj_info.getJuncPos(traj_info.getPieceNum()).tail(manipulator_dof_);
					stay_pos_ = traj_info.getJuncPos(traj_info.getPieceNum()).head(mobile_base_dof_);
					stay_yaw_ = have_terminal_yaw_ ? terminal_yaw_ : state_data.car_yaw;
				}else{
					update_stay_state(false);
				}
				cal_stay_ctrl_input();
				exec_traj_state_ = STAY;
				ROS_INFO("[MMctrl] Stop execute the trajectory. POLY_TRAJ --> STAY");
				trajectory_data.exec_traj = 0;
			}else{
				if(now_time < trajectory_activation_stamp_){
					update_stay_state(false);
					cal_stay_ctrl_input();
					break;
				}
				update_stay_state(false);
				cal_traj_ctrl_input(virtual_traj_time_);
			}
			break;
		}
		default:
		{
			exec_traj_state_ = STAY;
			ROS_ERROR("[MMctrl] Unknown exec_traj_state_! Jump to hover");
			break;		
		}
	}
}

void MMControllerFSM::update_stay_state(bool mani_goto_init){
	if(mani_goto_init){
		stay_joint_ = init_theta_;
	}else if(state_data.have_mani_){
		stay_joint_ = state_data.joint_p;
	}
	if(state_data.have_car_){
		stay_pos_ = state_data.car_p.head(2);
		stay_yaw_ = mm_utils::get_yaw_from_quaternion(state_data.car_q);
	}
}

bool MMControllerFSM::state_is_received(const ros::Time &now_time){
	return (now_time - state_data.odom_rcv_stamp).toSec() < odom_timeout_ &&
	       (now_time - state_data.joint_rcv_stamp).toSec() < odom_timeout_;
}

void MMControllerFSM::safetySpeedLimitCallback(const std_msgs::Float64::ConstPtr &msg){
	if(!std::isfinite(msg->data) || msg->data < 0.0)
		safety_speed_limit_ = -1.0;
	else
		safety_speed_limit_ = std::max(0.0, msg->data);
}

void MMControllerFSM::syncTrajectoryClock(const ros::Time &now){
	if(active_trajectory_seq_ == trajectory_data.update_seq)
		return;

	active_trajectory_seq_ = trajectory_data.update_seq;
	virtual_traj_time_ = 0.0;
	virtual_time_scale_ = trajectory_data.exec_traj == 1 ? 1.0 : 0.0;
	max_arm_tracking_error_ = 0.0;
	base_tracking_position_error_ = 0.0;
	base_tracking_yaw_error_ = 0.0;
	arm_tracking_hold_active_ = false;
	base_tracking_hold_active_ = false;
	updateTerminalYaw();
	last_virtual_clock_stamp_ = now;
	trajectory_activation_stamp_ = now;
	if(trajectory_data.exec_traj == 1 &&
	   !trajectory_data.singul_traj_data.start_time_ros.isZero() &&
	   trajectory_data.singul_traj_data.start_time_ros > now)
		trajectory_activation_stamp_ =
			trajectory_data.singul_traj_data.start_time_ros;
	ROS_INFO("[MMctrl] Reset virtual trajectory clock for update %lu.",
	         static_cast<unsigned long>(active_trajectory_seq_));
}

void MMControllerFSM::updateTerminalYaw(){
	have_terminal_yaw_ = false;
	terminal_yaw_ = state_data.car_yaw;
	const auto &traj = trajectory_data.singul_traj_data;
	if(trajectory_data.exec_traj != 1 || traj.singul_traj.empty() || traj.duration <= 0.0)
		return;

	const double earliest_sample =
		std::max(0.0, traj.duration - terminal_yaw_sample_window_);
	for(double sample_time = traj.duration;
	    sample_time >= earliest_sample - 1e-6;
	    sample_time -= 0.05){
		const Eigen::VectorXd velocity = traj.getVel(std::max(0.0, sample_time));
		if(velocity.head(mobile_base_dof_).norm() < terminal_yaw_min_speed_)
			continue;
		const int singul = traj.getSingul(std::max(0.0, sample_time));
		terminal_yaw_ = atan2(
			singul * velocity(1), singul * velocity(0));
		have_terminal_yaw_ = true;
		ROS_INFO("[MMctrl] Terminal yaw captured from trajectory: %.3f rad.",
		         terminal_yaw_);
		return;
	}
	ROS_WARN("[MMctrl] Unable to recover terminal yaw from the final trajectory segment.");
}

void MMControllerFSM::advanceTrajectoryClock(const ros::Time &now){
	if(last_virtual_clock_stamp_.isZero()){
		last_virtual_clock_stamp_ = now;
		return;
	}

	const double dt = std::max(
		0.0, std::min(0.1, (now - last_virtual_clock_stamp_).toSec()));
	last_virtual_clock_stamp_ = now;
	if(trajectory_data.exec_traj != 1 ||
	   trajectory_data.singul_traj_data.singul_traj.empty())
		return;
	if(now < trajectory_activation_stamp_)
		return;

	double target_scale = 1.0;
	bool force_freeze = false;
	const bool odom_fresh =
		!state_data.odom_rcv_stamp.isZero() &&
		(now - state_data.odom_rcv_stamp).toSec() <= odom_timeout_;
	if(!odom_fresh){
		force_freeze = true;
		ROS_ERROR_THROTTLE(1.0, "[MMctrl] Odom stale; freezing trajectory clock.");
	}else{
		const double sample_time = std::min(
			virtual_traj_time_, trajectory_data.singul_traj_data.duration);
		const Eigen::VectorXd reference_position =
			trajectory_data.singul_traj_data.getPos(sample_time);
		const Eigen::VectorXd reference_velocity =
			trajectory_data.singul_traj_data.getVel(sample_time);
		base_tracking_position_error_ =
			(reference_position.head(2) - state_data.car_p.head(2)).norm();

		// The controller deliberately holds measured yaw around a stationary
		// trajectory sample, so such a sample must not create a false yaw error.
		base_tracking_yaw_error_ = 0.0;
		if(reference_velocity.head(mobile_base_dof_).norm() > yaw_hold_speed_){
			const int reference_singul =
				trajectory_data.singul_traj_data.getSingul(sample_time);
			const double reference_yaw = atan2(
				reference_singul * reference_velocity(1),
				reference_singul * reference_velocity(0));
			base_tracking_yaw_error_ = std::fabs(atan2(
				sin(reference_yaw - state_data.car_yaw),
				cos(reference_yaw - state_data.car_yaw)));
		}

		if(base_tracking_hold_active_ &&
		   base_tracking_position_error_ <= base_tracking_resume_distance_ &&
		   base_tracking_yaw_error_ <= base_tracking_resume_yaw_){
			base_tracking_hold_active_ = false;
			ROS_INFO("[MMctrl] Base tracking recovered (position %.3f m, yaw %.3f rad).",
			         base_tracking_position_error_, base_tracking_yaw_error_);
		}else if(!base_tracking_hold_active_ &&
		         (base_tracking_position_error_ >= base_tracking_stop_distance_ ||
		          base_tracking_yaw_error_ >= base_tracking_stop_yaw_)){
			base_tracking_hold_active_ = true;
			ROS_WARN("[MMctrl] Base tracking lag (position %.3f m, yaw %.3f rad); freeze reference progress until feedback catches up.",
			         base_tracking_position_error_, base_tracking_yaw_error_);
		}

		if(base_tracking_hold_active_){
			target_scale = 0.0;
			force_freeze = true;
		}else{
			double position_scale = 1.0;
			if(base_tracking_position_error_ > base_tracking_slow_distance_){
				const double ratio = std::max(0.0, std::min(
					1.0, (base_tracking_stop_distance_ - base_tracking_position_error_) /
					(base_tracking_stop_distance_ - base_tracking_slow_distance_)));
				position_scale = base_tracking_min_time_scale_ +
					ratio * (1.0 - base_tracking_min_time_scale_);
			}
			double yaw_scale = 1.0;
			if(base_tracking_yaw_error_ > base_tracking_slow_yaw_){
				const double ratio = std::max(0.0, std::min(
					1.0, (base_tracking_stop_yaw_ - base_tracking_yaw_error_) /
					(base_tracking_stop_yaw_ - base_tracking_slow_yaw_)));
				yaw_scale = base_tracking_min_time_scale_ +
					ratio * (1.0 - base_tracking_min_time_scale_);
			}
			target_scale = std::min(target_scale, std::min(position_scale, yaw_scale));
			if(target_scale < 0.999){
				ROS_WARN_THROTTLE(
					1.0,
					"[MMctrl] Base tracking lag position=%.3f m yaw=%.3f rad; scale whole-body progress to %.2f.",
					base_tracking_position_error_, base_tracking_yaw_error_, target_scale);
			}
		}
	}

	geometry_msgs::Vector3Stamped base_error_msg;
	base_error_msg.header.stamp = now;
	base_error_msg.header.frame_id = "base_tracking_error";
	base_error_msg.vector.x = base_tracking_position_error_;
	base_error_msg.vector.y = base_tracking_yaw_error_;
	base_error_msg.vector.z = base_tracking_hold_active_ ? 1.0 : 0.0;
	base_tracking_error_pub_.publish(base_error_msg);

	if(odom_fresh && safety_speed_limit_ >= 0.0){
		const double sample_time = std::min(
			virtual_traj_time_, trajectory_data.singul_traj_data.duration);
		const Eigen::VectorXd reference_velocity =
			trajectory_data.singul_traj_data.getVel(sample_time);
		const double reference_speed =
			reference_velocity.head(mobile_base_dof_).norm();
		if(safety_speed_limit_ <= 1e-3){
			force_freeze = true;
		}else{
			target_scale = std::min(
				target_scale, safety_speed_limit_ /
					std::max(reference_speed, virtual_time_min_reference_speed_));
		}
	}

	const bool joint_fresh =
		state_data.have_mani_ &&
		!state_data.joint_rcv_stamp.isZero() &&
		(now - state_data.joint_rcv_stamp).toSec() <= arm_tracking_joint_timeout_;
	if(!joint_fresh){
		force_freeze = true;
		ROS_ERROR_THROTTLE(1.0, "[MMctrl] Joint state stale; freezing trajectory clock.");
	}else{
		const double sample_time = std::min(
			virtual_traj_time_, trajectory_data.singul_traj_data.duration);
		const Eigen::VectorXd reference_position =
			trajectory_data.singul_traj_data.getPos(sample_time);
		if(reference_position.size() >= mobile_base_dof_ + manipulator_dof_ &&
		   state_data.joint_p.size() == manipulator_dof_){
			max_arm_tracking_error_ =
				(reference_position.tail(manipulator_dof_) -
				 state_data.joint_p).cwiseAbs().maxCoeff();

			if(arm_tracking_hold_active_ &&
			   max_arm_tracking_error_ <= arm_tracking_resume_error_){
				arm_tracking_hold_active_ = false;
				ROS_INFO("[MMctrl] Arm tracking recovered (max error %.3f rad).",
				         max_arm_tracking_error_);
			}else if(!arm_tracking_hold_active_ &&
			         max_arm_tracking_error_ >= arm_tracking_stop_error_){
				arm_tracking_hold_active_ = true;
				ROS_WARN("[MMctrl] Arm tracking error %.3f rad; pause base progress until the arm catches up.",
				         max_arm_tracking_error_);
			}

			if(arm_tracking_hold_active_){
				target_scale = 0.0;
			}else if(max_arm_tracking_error_ > arm_tracking_slow_error_){
				const double ratio = std::max(
					0.0, std::min(
						1.0,
						(arm_tracking_stop_error_ - max_arm_tracking_error_) /
						(arm_tracking_stop_error_ - arm_tracking_slow_error_)));
				const double arm_scale =
					arm_tracking_min_time_scale_ +
					ratio * (1.0 - arm_tracking_min_time_scale_);
				target_scale = std::min(target_scale, arm_scale);
				ROS_WARN_THROTTLE(
					1.0,
					"[MMctrl] Arm tracking error %.3f rad; scale whole-body progress to %.2f.",
					max_arm_tracking_error_, target_scale);
			}
		}
	}

	std_msgs::Float64 arm_error_msg;
	arm_error_msg.data = max_arm_tracking_error_;
	arm_tracking_error_pub_.publish(arm_error_msg);

	if(force_freeze){
		virtual_time_scale_ = 0.0;
	}else{
		const double rate = target_scale > virtual_time_scale_
			? virtual_time_scale_rise_rate_
			: virtual_time_scale_fall_rate_;
		const double max_change = rate * dt;
		virtual_time_scale_ = std::max(
			virtual_time_scale_ - max_change,
			std::min(virtual_time_scale_ + max_change, target_scale));
	}

	virtual_traj_time_ = std::min(
		trajectory_data.singul_traj_data.duration,
		virtual_traj_time_ + dt * virtual_time_scale_);
}

void MMControllerFSM::publishTrajectoryProgress(const ros::Time &stamp){
	geometry_msgs::Vector3Stamped msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "trajectory_progress";
	if(!trajectory_data.singul_traj_data.singul_traj.empty()){
		msg.vector.x = trajectory_data.singul_traj_data.start_time;
		msg.vector.y = virtual_traj_time_;
		msg.vector.z =
			trajectory_data.exec_traj == 1 && stamp >= trajectory_activation_stamp_
				? virtual_time_scale_ : 0.0;
	}else{
		msg.vector.x = 0.0;
		msg.vector.y = 0.0;
		msg.vector.z = 0.0;
	}
	trajectory_progress_pub_.publish(msg);
}

void MMControllerFSM::publish_ctrl_cmd(const ros::Time &stamp){
	geometry_msgs::Twist car_cmd;
	// true value
	car_cmd.linear.x = cmd_data.car_true_cmd(0); // x
	car_cmd.linear.y = cmd_data.car_true_cmd(1); // y
	car_cmd.linear.z = cmd_data.car_true_cmd(2); // yaw
	car_cmd.angular.x = cmd_data.car_true_cmd(3); //vx
	car_cmd.angular.y = cmd_data.car_true_cmd(4); //vy
	car_cmd.angular.z = cmd_data.car_omega_cmd;

	control_msgs::JointTrajectoryControllerState joint_cmd;
	joint_cmd.header.stamp = stamp;
	for(int i = 0; i < manipulator_dof_; ++i){
 		joint_cmd.joint_names.push_back("joint" + std::to_string(i + 1));
		joint_cmd.desired.positions.push_back(cmd_data.joint_pos_cmd(i));
		joint_cmd.desired.velocities.push_back(cmd_data.joint_vel_cmd(i));
		joint_cmd.desired.effort.push_back(cmd_data.joint_effort_cmd(i));
	}
	car_cmd_pub_.publish(car_cmd);
	mani_cmd_pub_.publish(joint_cmd);
}

void MMControllerFSM::publish_trigger(const nav_msgs::Odometry &odom_msg){
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	traj_start_trigger_pub_.publish(msg);
}

void MMControllerFSM::cal_traj_ctrl_input(double duration){
	cmd_data.cal_stamp = ros::Time::now();
	duration = std::min(duration, trajectory_data.singul_traj_data.duration);
	last_singul_ = trajectory_data.singul_traj_data.getSingul(duration);
	double temp_duration = duration;
	trajectory_data.singul_traj_data.locatePieceIdx(temp_duration);

	Eigen::VectorXd pos = trajectory_data.singul_traj_data.getPos(duration);
	Eigen::VectorXd vel = trajectory_data.singul_traj_data.getVel(duration);
	Eigen::VectorXd acc = trajectory_data.singul_traj_data.getAcc(duration);
	const Eigen::VectorXd reference_vel = vel;
	const double time_scale = std::max(0.0, std::min(1.0, virtual_time_scale_));
	vel *= time_scale;
	acc *= time_scale * time_scale;

	// true value
	cmd_data.car_true_cmd.resize(5);
	cmd_data.car_true_cmd(0) = pos(0);
	cmd_data.car_true_cmd(1) = pos(1);
	const double reference_base_speed =
		reference_vel.head(mobile_base_dof_).norm();
	if (reference_base_speed > yaw_hold_speed_)
			cmd_data.car_true_cmd(2) = atan2(
				last_singul_ * reference_vel(1),
				last_singul_ * reference_vel(0));
		else
			cmd_data.car_true_cmd(2) = state_data.car_yaw;
	cmd_data.car_true_cmd(3) = vel(0);
	cmd_data.car_true_cmd(4) = vel(1);

	// desired angular velocity: omega = (vx*ay - vy*ax) / (vx^2+vy^2)
	{
		const double base_speed = vel.head(mobile_base_dof_).norm();
		double v2 = vel(0) * vel(0) + vel(1) * vel(1);
		if (base_speed > curvature_ff_start_speed_ && v2 > 1e-6) {
			const double raw_omega = (vel(0) * acc(1) - vel(1) * acc(0)) / v2;
			const double speed_blend = std::min(
				1.0, (base_speed - curvature_ff_start_speed_) /
				(curvature_ff_full_speed_ - curvature_ff_start_speed_));
			const double bounded_omega = std::max(
				-max_feedforward_omega_, std::min(max_feedforward_omega_, raw_omega));
			cmd_data.car_omega_cmd = speed_blend * bounded_omega;
		} else {
			cmd_data.car_omega_cmd = 0.0;
		}
	}

	cmd_data.joint_pos_cmd = pos.tail(manipulator_dof_);
	cmd_data.joint_vel_cmd = vel.tail(manipulator_dof_);
	cmd_data.joint_effort_cmd = acc.tail(manipulator_dof_);
}

void MMControllerFSM::cal_stay_ctrl_input(){
	cmd_data.cal_stamp = ros::Time::now();
	cmd_data.car_true_cmd.resize(5);
	cmd_data.car_true_cmd.setZero();
	cmd_data.car_true_cmd(0) = stay_pos_(0);
	cmd_data.car_true_cmd(1) = stay_pos_(1);
	cmd_data.car_true_cmd(2) = stay_yaw_;
	cmd_data.car_omega_cmd = 0.0;

	cmd_data.joint_pos_cmd = stay_joint_;
	cmd_data.joint_vel_cmd.setZero();
	cmd_data.joint_effort_cmd.setZero();
}
}

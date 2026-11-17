#ifndef __CONTROL_UTILS_H
#define __CONTROL_UTILS_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>
#include <cstdint>
#include <deque>
#include <cmath>
#include <set>
#include <algorithm>

#include "traj_utils/poly_traj_utils.hpp"
#include "traj_utils/plan_container.hpp"
// #include "mm_controller/polynomial_trajectory.h"
#include "mm_utils/utils.h"

namespace MMController{

class State_Data_t
{
public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d car_p;
	Eigen::Quaterniond car_q;
	Eigen::Vector3d car_v;
	double car_v_norm;
	Eigen::Vector3d car_w;
	double car_yaw;
	Eigen::VectorXd joint_p;
	Eigen::VectorXd joint_v;

	nav_msgs::Odometry odom_msg;
	sensor_msgs::JointState joint_state_msg;
	int mani_dof_;
	ros::Time rcv_stamp;
	ros::Time odom_rcv_stamp;
	ros::Time joint_rcv_stamp;
	ros::Time odom_source_stamp, joint_source_stamp;
	bool have_car_;
	bool have_mani_;

	State_Data_t(){
		rcv_stamp = ros::Time(0);
		odom_rcv_stamp = ros::Time(0);
		joint_rcv_stamp = ros::Time(0);
		odom_source_stamp = joint_source_stamp = ros::Time(0);
		car_q.setIdentity();
		have_car_ = false;
		have_mani_ = false;
	}

	void setParam(int mani_dof){
		mani_dof_ = mani_dof;
		joint_p = Eigen::VectorXd::Zero(mani_dof_);
		joint_v = Eigen::VectorXd::Zero(mani_dof_);
	}

	static bool sourceFresh(const ros::Time &stamp, const ros::Time &now, double timeout){
		const double age = (now - stamp).toSec();
		return !stamp.isZero() && age >= -0.05 && age < timeout;
	}
	
	void feed_odom(const nav_msgs::Odometry::ConstPtr& pMsg){
		const ros::Time now = ros::Time::now();
		if(!odom_source_stamp.isZero() && pMsg->header.stamp <= odom_source_stamp) return;
		const auto &p = pMsg->pose.pose.position;
		const auto &q = pMsg->pose.pose.orientation;
		const auto &v = pMsg->twist.twist.linear;
		const auto &w = pMsg->twist.twist.angular;
		const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
		if(pMsg->header.stamp.isZero() || (pMsg->header.stamp-now).toSec() > 0.05){
			have_car_ = false;
			return;
		}
		odom_source_stamp = pMsg->header.stamp;
		if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
		   !std::isfinite(norm) || std::fabs(norm-1.0) > 0.02 ||
		   !std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
		   !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z)){
			have_car_ = false;
			return;
		}
		have_car_ = true;
		odom_msg = *pMsg;
		rcv_stamp = ros::Time::now();
		odom_rcv_stamp = rcv_stamp;

		car_p(0) = odom_msg.pose.pose.position.x;
		car_p(1) = odom_msg.pose.pose.position.y;
		car_p(2) = odom_msg.pose.pose.position.z;

		car_q.w() = odom_msg.pose.pose.orientation.w;
		car_q.x() = odom_msg.pose.pose.orientation.x;
		car_q.y() = odom_msg.pose.pose.orientation.y;
		car_q.z() = odom_msg.pose.pose.orientation.z;
		car_yaw = tf::getYaw(pMsg->pose.pose.orientation);

		car_v(0) = odom_msg.twist.twist.linear.x;
		car_v(1) = odom_msg.twist.twist.linear.y;
		car_v(2) = odom_msg.twist.twist.linear.z;
		car_v = car_q.toRotationMatrix() * car_v;
		car_v_norm = car_v.head(2).norm();

		car_w(0) = odom_msg.twist.twist.angular.x;
		car_w(1) = odom_msg.twist.twist.angular.y;
		car_w(2) = odom_msg.twist.twist.angular.z;
	}

	void feed_joint(const sensor_msgs::JointState::ConstPtr& pMsg){
		const ros::Time now = ros::Time::now();
		if(!joint_source_stamp.isZero() && pMsg->header.stamp <= joint_source_stamp) return;
		if(pMsg->header.stamp.isZero() || (pMsg->header.stamp-now).toSec() > 0.05){
			have_mani_ = false;
			return;
		}
		joint_source_stamp = pMsg->header.stamp;
		if(pMsg->name.size() != pMsg->position.size() ||
		   pMsg->velocity.size() != pMsg->name.size() ||
		   std::set<std::string>(pMsg->name.begin(), pMsg->name.end()).size() != pMsg->name.size()){
			have_mani_ = false;
			return;
		}
		Eigen::VectorXd positions(mani_dof_), velocities(mani_dof_);
		for(int i = 0; i < mani_dof_; ++i){
			const auto it = std::find(pMsg->name.begin(), pMsg->name.end(), "joint" + std::to_string(i+1));
			if(it == pMsg->name.end()) { have_mani_ = false; return; }
			const auto j = std::distance(pMsg->name.begin(), it);
			positions(i) = pMsg->position[j]; velocities(i) = pMsg->velocity[j];
		}
		if(!positions.allFinite() || !velocities.allFinite()) { have_mani_ = false; return; }
		have_mani_ = true;
		joint_state_msg = *pMsg;
		rcv_stamp = ros::Time::now();
		joint_rcv_stamp = rcv_stamp;

		joint_p = positions;
		joint_v = velocities;
	}
};

struct oneTraj_Data_t{
  public:
  ros::Time traj_start_time;
  ros::Time traj_end_time;
  poly_traj::Trajectory<7> traj;
};

class Trajectory_Data_t{
public:
	double total_traj_start_time;
	double total_traj_end_time;
	ros::Time total_traj_start_time_ros;
	ros::Time total_traj_end_time_ros;
	int exec_traj = 0; // 0: idle, -1: abort, 1: executing
	std::uint64_t update_seq = 0;
	ros::Time last_update_stamp;
	// std::deque<oneTraj_Data_t> traj_queue;
	remani_planner::SingulTrajData singul_traj_data;
	
	Trajectory_Data_t(){
		total_traj_start_time_ros = ros::Time(0);
		total_traj_end_time_ros = ros::Time(0);
		last_update_stamp = ros::Time(0);
		exec_traj = 0;
	}

	// void adjust_end_time(){
	// 	if (traj_queue.size() < 2){
	// 		return;
	// 	}
	// 	for (auto it = traj_queue.begin(); it != (traj_queue.end()-1); it++){
	// 		it->traj_end_time = (it+1)->traj_start_time;
	// 	}
	// }

	void feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg);
};

class Command_Data_t{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	double car_vel_cmd; // control
	Eigen::VectorXd car_true_cmd; // true value
	double car_omega_cmd;
	Eigen::VectorXd joint_pos_cmd;
	Eigen::VectorXd joint_vel_cmd;
	Eigen::VectorXd joint_effort_cmd;
	double yaw;

	ros::Time cal_stamp;

	Command_Data_t(){
		cal_stamp = ros::Time(0);
	}
};

}

#endif

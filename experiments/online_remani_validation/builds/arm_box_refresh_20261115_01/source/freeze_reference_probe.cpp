// Test harness links the unchanged native MMController object. No cmd publisher.
#include <mm_controller/mm_controller_fsm.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc,char**argv){
  ros::init(argc,argv,"freeze_reference_probe");ros::NodeHandle nh("~");
  if(argc<2)return 2;
  nh.setParam("mm/mobile_base_dof",2);nh.setParam("mm/manipulator_dof",6);
  nh.setParam("fsm/init_state",std::vector<double>(8,0));nh.setParam("fsm/init_gripper_close",false);
  nh.setParam("controller/max_feedforward_omega",.55);
  MMController::MMControllerFSM fsm(nh);
  std::ifstream in(argv[1]);int segments;in>>segments;
  fsm.trajectory_data.exec_traj=1;fsm.trajectory_data.singul_traj_data.clearSingulTraj();
  double start=1000;
  for(int i=0;i<segments;++i){
    int sign,pieces;in>>sign>>pieces;poly_traj::Trajectory<7> traj;
    for(int j=0;j<pieces;++j){double duration;in>>duration;Eigen::Matrix<double,8,8> c;
      for(int col=0;col<8;++col)for(int row=0;row<8;++row)in>>c(row,col);
      traj.emplace_back(duration,c,sign);}
    fsm.trajectory_data.singul_traj_data.addSingulTraj(traj,start);
    start=fsm.trajectory_data.singul_traj_data.singul_traj.back().end_time;
  }
  if(!in||segments<1)return 3;
  fsm.state_data.have_car_=true;fsm.state_data.have_mani_=true;
  fsm.state_data.joint_p=fsm.trajectory_data.singul_traj_data.getPos(0).tail(6);
  fsm.state_data.joint_v=Eigen::VectorXd::Zero(6);
  fsm.trajectory_activation_stamp_=ros::Time(1000);fsm.virtual_traj_time_=0;fsm.virtual_time_scale_=1;
  fsm.base_tracking_hold_active_=false;fsm.arm_tracking_hold_active_=false;
  std::cout<<"READY\n"<<std::flush;
  double t,x,y,yaw,reset_time;int reset;
  while(std::cin>>t>>x>>y>>yaw>>reset>>reset_time){
    ros::Time now;now.fromSec(1000+t);ros::Time::setNow(now);
    fsm.state_data.car_p<<x,y,0;fsm.state_data.car_yaw=yaw;
    fsm.state_data.odom_rcv_stamp=now;fsm.state_data.joint_rcv_stamp=now;
    if(reset){fsm.virtual_traj_time_=reset_time;fsm.virtual_time_scale_=1;
      fsm.base_tracking_hold_active_=false;fsm.arm_tracking_hold_active_=false;
      fsm.last_virtual_clock_stamp_=now;}
    fsm.advanceTrajectoryClock(now);fsm.cal_traj_ctrl_input(fsm.virtual_traj_time_);
    std::cout<<std::setprecision(17)<<"ROW "<<fsm.virtual_traj_time_<<' '<<fsm.virtual_time_scale_
      <<' '<<fsm.base_tracking_hold_active_<<' '<<fsm.base_tracking_position_error_<<' '<<fsm.base_tracking_yaw_error_;
    for(int i=0;i<5;++i)std::cout<<' '<<fsm.cmd_data.car_true_cmd(i);
    std::cout<<' '<<fsm.cmd_data.car_omega_cmd<<'\n'<<std::flush;
  }
  ros::shutdown();return 0;
}

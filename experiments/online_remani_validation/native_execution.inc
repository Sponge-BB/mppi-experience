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

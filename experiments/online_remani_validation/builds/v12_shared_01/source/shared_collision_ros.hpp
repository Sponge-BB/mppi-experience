#pragma once
#include "shared_collision.hpp"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
namespace shared_collision {
inline ros::Subscriber& subscriber(){static ros::Subscriber value;return value;}
inline Eigen::VectorXd& fixedArm(){static Eigen::VectorXd q;return q;}
inline std::shared_ptr<const Snapshot>& objectiveSnapshot(){static thread_local std::shared_ptr<const Snapshot> value;return value;}
inline double& objectiveTime(){static thread_local double value=0.;return value;}
struct ObjectiveFreeze {
  std::shared_ptr<const Snapshot>previous;double previous_time;
  ObjectiveFreeze():previous(objectiveSnapshot()),previous_time(objectiveTime()){
    if(!previous){objectiveSnapshot()=snapshot();objectiveTime()=ros::Time::now().toSec();}
  }
  bool valid()const{return objectiveSnapshot()&&fresh(*objectiveSnapshot(),objectiveTime());}
  ~ObjectiveFreeze(){objectiveSnapshot()=previous;objectiveTime()=previous_time;}
};
inline void initialize(ros::NodeHandle&nh){
  bool requested=false;nh.param("fsm/unified_projection_collision",requested,false);
  if(!requested)return;
  bool sil=false,sim=false;ros::param::param("/online_validation/enabled",sil,false);
  ros::param::param("/use_sim_time",sim,false);
  if(!sil||!sim)throw std::runtime_error("shared projection is restricted to isolated SIL");
  std::vector<double>lo,hi;nh.getParam("mm/manipulator_min_pos",lo);nh.getParam("mm/manipulator_max_pos",hi);
  if(lo.empty()||lo!=hi)throw std::runtime_error("shared projection requires locked arm joints");
  fixedArm()=Eigen::Map<Eigen::VectorXd>(lo.data(),lo.size());enabled()=true;
  subscriber()=nh.subscribe<std_msgs::Float64MultiArray>("/sim/unified_collision",1,
    [](const std_msgs::Float64MultiArray::ConstPtr&m){
      if(!bind(m->data.data(),m->data.size())){std::shared_ptr<const Snapshot>empty;std::atomic_store(&store(),empty);}
    });
}
inline bool armMatches(const Eigen::VectorXd&q){return q.size()==fixedArm().size()&&q.allFinite()&&
  (q-fixedArm()).cwiseAbs().maxCoeff()<=1e-5;}
inline Evaluation pose(const Eigen::Vector3d&q,double query_time=0,bool dynamic_only=false){
  auto m=objectiveSnapshot()?objectiveSnapshot():snapshot();
  double now=objectiveSnapshot()?objectiveTime():ros::Time::now().toSec();
  if(!m||!fresh(*m,now)){Evaluation invalid;invalid.valid=false;invalid.physical=invalid.gap=-1e6;invalid.cost=1e12;return invalid;}
  return evaluate(*m,q.x(),q.y(),q.z(),(query_time>0?query_time:now)-m->stamp,!dynamic_only,true);
}
}

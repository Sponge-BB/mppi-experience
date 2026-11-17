#pragma once
// Isolated SIL feedback: an already validated fast avoidance episode requests
// the existing slow refresh. It grants no command or collision permission.
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

namespace fast_refresh_feedback {
struct Packet {
  double stamp=0.,nominal_gap=0.;
  std::uint64_t generation=0,revision=0,goal_token=0,episode=0;
  bool valid=false,dynamic_avoid_active=false,actual_command_validated=false;
};
struct Identity {
  std::uint64_t generation=0,revision=0,goal_token=0;
};
struct State {
  Packet latest;
  // A continuing avoidance episode is consumed across guide replacements.
  std::uint64_t consumed_generation=0,consumed_episode=0;
};
inline bool integer(double value,double minimum,double maximum=9007199254740991.){
  return std::isfinite(value)&&value>=minimum&&value<=maximum&&value==std::floor(value);
}
inline bool parse(const std::vector<double>& values,Packet& out){
  out=Packet();
  if(values.size()!=9||values[0]!=1.)return false;
  for(double value:values)if(!std::isfinite(value))return false;
  if(values[1]<=0.||!integer(values[2],1.)||!integer(values[3],1.)||
     !integer(values[4],0.,4294967295.)||!integer(values[5],1.)||
     (values[6]!=0.&&values[6]!=1.)||(values[7]!=0.&&values[7]!=1.))return false;
  out.stamp=values[1];out.generation=std::uint64_t(values[2]);out.revision=std::uint64_t(values[3]);
  out.goal_token=std::uint64_t(values[4]);out.episode=std::uint64_t(values[5]);
  out.dynamic_avoid_active=values[6]==1.;out.actual_command_validated=values[7]==1.;
  out.nominal_gap=values[8];out.valid=true;return true;
}
inline bool eligible(const State& value,double now,const Identity& identity){
  const auto& p=value.latest;const double age=now-p.stamp;
  return p.valid&&std::isfinite(now)&&age>=-.05&&age<=.25&&
    p.generation==identity.generation&&p.revision==identity.revision&&p.goal_token==identity.goal_token&&
    p.dynamic_avoid_active&&p.actual_command_validated&&
    !(value.consumed_generation==p.generation&&p.episode<=value.consumed_episode);
}
inline void consume(State& value){
  value.consumed_generation=value.latest.generation;
  value.consumed_episode=value.latest.episode;
}
inline bool enabled(){
  const char* value=std::getenv("FAST_REFRESH_FEEDBACK");return value&&std::string(value)=="1";
}
inline State& state(){static State value;return value;}
inline ros::Subscriber& subscriber(){static ros::Subscriber value;return value;}
inline void receive(const std_msgs::Float64MultiArray::ConstPtr& message){
  parse(message->data,state().latest);
}
inline void configure(ros::NodeHandle& nh){
  subscriber()=nh.subscribe("/planning/local_refresh_request",1,receive);
}
} // namespace fast_refresh_feedback

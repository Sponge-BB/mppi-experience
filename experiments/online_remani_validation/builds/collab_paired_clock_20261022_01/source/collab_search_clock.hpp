#pragma once
#include <cstdlib>
#include <ros/ros.h>
namespace collab_budget {
inline ros::Time now(){
  const char* flag=std::getenv("COLLAB_PAIR");
  if(flag&&std::atoi(flag))return ros::Time(ros::WallTime::now().toSec());
  return ros::Time::now();
}
}

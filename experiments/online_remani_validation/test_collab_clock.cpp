#include "collab_search_clock.hpp"
#include <unistd.h>
#include <iostream>
int main(){
  ros::Time::init();ros::Time::setNow(ros::Time(1000.));
  setenv("COLLAB_PAIR","1",1);
  auto begin=collab_budget::now();usleep(25000);auto end=collab_budget::now();
  if((end-begin).toSec()<.015||ros::Time::now().toSec()!=1000.)return 1;
  setenv("COLLAB_PAIR","0",1);
  if(collab_budget::now().toSec()!=1000.)return 2;
  std::cout<<"PASS paused-world search deadline clock; online ROS clock unchanged\n";
}

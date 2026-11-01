#include "latest_odom.hpp"
#include <atomic>
#include <cassert>
#include <thread>

int main() {
  state_handoff::Inbox inbox;
  assert(!inbox.snapshot().message);
  std::atomic<bool> done{false};
  std::thread writer([&]{
    for(unsigned i=1;i<=100000;++i){
      nav_msgs::OdometryPtr msg(new nav_msgs::Odometry);
      msg->header.seq=i;msg->pose.pose.position.x=i;
      msg->twist.twist.linear.x=-double(i);msg->twist.twist.angular.z=.5;
      inbox.receive(msg,double(i),double(i)+.25);
    }
    done.store(true);
  });
  state_handoff::Sample retained;
  while(!done.load()){
    const auto s=inbox.snapshot();
    if(!s.message)continue;
    assert(s.sequence==s.message->header.seq);
    assert(s.message->pose.pose.position.x==s.receipt_ros);
    assert(s.message->twist.twist.linear.x==-s.receipt_ros);
    assert(s.receipt_mono==s.receipt_ros+.25);
    if(!retained.message)retained=s;
  }
  writer.join();
  const auto final=inbox.snapshot();
  assert(final.sequence==100000);
  if(retained.message)assert(retained.message->pose.pose.position.x==retained.receipt_ros);
  return 0;
}

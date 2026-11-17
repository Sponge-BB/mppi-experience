#pragma once
// The receive thread owns only this immutable-message mailbox, never the FSM.
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <mutex>
#include <memory>
#include <cstdint>

namespace state_handoff {
struct Sample {
  nav_msgs::OdometryConstPtr message;
  std::uint64_t sequence=0;
  double receipt_ros=0., receipt_mono=0.;
};
class Inbox {
  mutable std::mutex mutex_;
  Sample latest_;
public:
  void receive(const nav_msgs::OdometryConstPtr& message,double ros,double mono) {
    std::lock_guard<std::mutex> guard(mutex_);
    latest_.message=message;latest_.receipt_ros=ros;latest_.receipt_mono=mono;
    ++latest_.sequence;
  }
  Sample snapshot() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return latest_;
  }
};
}

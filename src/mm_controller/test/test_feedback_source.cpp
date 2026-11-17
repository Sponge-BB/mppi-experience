#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include <limits>
#include "mm_controller/controller_utils.hpp"

namespace {
using MMController::State_Data_t;

class FeedbackSource : public ::testing::Test {
protected:
  State_Data_t state;
  void SetUp() override {
    ros::Time::init();
    ros::Time::setNow(ros::Time(100.));
    state.setParam(6);
  }
  sensor_msgs::JointState::Ptr joints(double stamp=100.) {
    auto msg = boost::make_shared<sensor_msgs::JointState>();
    msg->header.stamp = ros::Time(stamp);
    for(int i=6; i>=1; --i) {
      msg->name.push_back("joint"+std::to_string(i));
      msg->position.push_back(i*.01);
      msg->velocity.push_back(i*.02);
    }
    return msg;
  }
};

TEST_F(FeedbackSource, SourceAgeCannotBeRenewedByQueuedDuplicate) {
  auto msg = joints();
  state.feed_joint(msg);
  ASSERT_TRUE(state.have_mani_);
  EXPECT_DOUBLE_EQ(state.joint_p(0), .01);  // Uses names, not incoming order.
  ros::Time::setNow(ros::Time(100.3));
  state.feed_joint(msg);
  EXPECT_FALSE(State_Data_t::sourceFresh(state.joint_source_stamp, ros::Time::now(), .25));
  EXPECT_DOUBLE_EQ(state.joint_source_stamp.toSec(), 100.);
}

TEST_F(FeedbackSource, MissingOrInvalidJointsCannotRefreshAndOlderPacketsCannotRestore) {
  state.feed_joint(joints());
  auto invalid = joints(100.01);
  invalid->velocity.clear();
  state.feed_joint(invalid);
  ASSERT_FALSE(state.have_mani_);
  state.feed_joint(joints());
  state.feed_joint(joints(100.01));
  EXPECT_FALSE(state.have_mani_);
  state.feed_joint(joints(100.02));
  EXPECT_TRUE(state.have_mani_);
  invalid = joints(100.03);
  invalid->position[2] = std::numeric_limits<double>::quiet_NaN();
  state.feed_joint(invalid);
  EXPECT_FALSE(state.have_mani_);
}

TEST_F(FeedbackSource, OdomRequiresValidSourceAndFiniteGeometry) {
  auto msg = boost::make_shared<nav_msgs::Odometry>();
  msg->pose.pose.orientation.w = 1.;
  state.feed_odom(msg);
  EXPECT_FALSE(state.have_car_);
  msg->header.stamp = ros::Time(100.);
  state.feed_odom(msg);
  EXPECT_TRUE(state.have_car_);
  msg->header.stamp = ros::Time(100.01);
  msg->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  state.feed_odom(msg);
  EXPECT_FALSE(state.have_car_);
}

TEST_F(FeedbackSource, FreshnessBoundaryAndFutureTolerance) {
  const ros::Time now(100.);
  EXPECT_FALSE(State_Data_t::sourceFresh(ros::Time(0), now, .25));
  EXPECT_FALSE(State_Data_t::sourceFresh(ros::Time(99.75), now, .25));
  EXPECT_TRUE(State_Data_t::sourceFresh(ros::Time(99.751), now, .25));
  EXPECT_TRUE(State_Data_t::sourceFresh(ros::Time(100.05), now, .25));
  EXPECT_FALSE(State_Data_t::sourceFresh(ros::Time(100.051), now, .25));
}
}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

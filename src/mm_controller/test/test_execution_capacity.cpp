#include <gtest/gtest.h>
#include <limits>
#include "mm_controller/mm_controller_fsm.hpp"

namespace {
using FSM = MMController::MMControllerFSM;
FSM::BaseExecutionLimits limits() {
  FSM::BaseExecutionLimits result;
  result.enabled = true;
  result.linear = .25;
  result.angular = .35;
  result.wheel = .25;
  result.track = .37;
  return result;
}
double cap(double vx, double vy, double ax, double ay,
           const FSM::BaseExecutionLimits &bound=limits(), double yaw_hold=.04) {
  return FSM::baseExecutionTimeScale(Eigen::Vector2d(vx, vy),
    Eigen::Vector2d(ax, ay), yaw_hold, bound);
}
TEST(ExecutionCapacity, StationaryDoesNotInventCurvature) {
  EXPECT_DOUBLE_EQ(cap(0., 0., 0., 0.), 1.);
  EXPECT_DOUBLE_EQ(cap(0., 0., .5, -.5), 1.);
  EXPECT_DOUBLE_EQ(cap(.04, 0., 0., .5), 1.);
  EXPECT_LT(cap(.040001, 0., 0., .5), 1.);
}
TEST(ExecutionCapacity, ForwardAndReverseLinearLimitsAgree) {
  EXPECT_DOUBLE_EQ(cap(.5, 0., 0., 0.), .5);
  EXPECT_DOUBLE_EQ(cap(-.5, 0., 0., 0.), .5);
  EXPECT_DOUBLE_EQ(cap(0., .5, 0., 0.), .5);
}
TEST(ExecutionCapacity, AngularDemandUsesUnscaledPolynomial) {
  // |omega|=.8 rad/s before the shared clock scale: .35/.8=.4375.
  EXPECT_NEAR(cap(.1, 0., 0., .08), .4375, 1e-12);
  EXPECT_NEAR(cap(-.1, 0., 0., -.08), .4375, 1e-12);
  EXPECT_NEAR(cap(.1, 0., 0., -.08), .4375, 1e-12);
}
TEST(ExecutionCapacity, SimultaneousLinearAndAngularDemandRespectsBothWheels) {
  // Both independent caps pass, but .24+.185*.3 > .25 m/s.
  const double actual = cap(.24, 0., 0., .072);
  EXPECT_NEAR(actual, .25/(.24+.185*.3), 1e-12);
  EXPECT_LE(actual*(.24+.185*.3), .25+1e-12);
  EXPECT_DOUBLE_EQ(cap(-.24, 0., 0., -.072), actual);
}
TEST(ExecutionCapacity, ExactAngularLimitDoesNotSlowReference) {
  EXPECT_NEAR(cap(.1, 0., 0., .035), 1., 1e-12);
}
TEST(ExecutionCapacity, IncreasingCurvatureRequiresNewPhaseLimit) {
  // Analytic path x=.1*t, y=t^3/6: omega rises across the integration tick.
  const auto at = [](double t) { return cap(.1, .5*t*t, 0., t); };
  const double old_phase=.04, dt=.01, old_limit=at(old_phase);
  const double new_phase=old_phase+dt*old_limit;
  const double new_limit=at(new_phase);
  ASSERT_LT(new_limit, old_limit);
  const double published=std::min(old_limit, new_limit);
  const double raw_omega=.1*new_phase/(.01+.25*std::pow(new_phase, 4));
  EXPECT_GT(old_limit*raw_omega, .35);
  EXPECT_LE(published*raw_omega, .35+1e-12);
}
TEST(ExecutionCapacity, RotationOfWorldFrameDoesNotChangeCapacity) {
  EXPECT_NEAR(cap(0., .1, -.08, 0.), cap(.1, 0., 0., .08), 1e-12);
}
TEST(ExecutionCapacity, ExplicitDisabledModePreservesNativeClock) {
  auto bound = limits(); bound.enabled = false;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_DOUBLE_EQ(cap(.5, 0., 0., 1., bound), 1.);
  EXPECT_DOUBLE_EQ(cap(nan, 0., nan, 0., bound), 1.);
}
TEST(ExecutionCapacity, NonfiniteReferenceOrInvalidLimitsCannotAdvanceClock) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_DOUBLE_EQ(cap(nan, 0., 0., 0.), 0.);
  EXPECT_DOUBLE_EQ(cap(.1, 0., 0., inf), 0.);
  EXPECT_DOUBLE_EQ(cap(.1, 0., 0., 0., limits(), nan), 0.);
  auto bound = limits(); bound.angular = 0.;
  EXPECT_DOUBLE_EQ(cap(.1, 0., 0., .05, bound), 0.);
  bound = limits(); bound.track = nan;
  EXPECT_DOUBLE_EQ(cap(.1, 0., 0., .05, bound), 0.);
  bound = limits(); bound.wheel = -1.;
  EXPECT_DOUBLE_EQ(cap(.1, 0., 0., .05, bound), 0.);
}
}  // namespace
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

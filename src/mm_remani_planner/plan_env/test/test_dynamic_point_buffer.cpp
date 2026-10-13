#include <gtest/gtest.h>

#include <plan_env/dynamic_point_buffer.h>

namespace
{

ros::Time timeFromSec(double seconds)
{
  ros::Time time;
  time.fromSec(seconds);
  return time;
}

TEST(DynamicPointBuffer, ConstantVelocityPredictionUsesObservationTime)
{
  plan_env::DynamicPoint point;
  point.position = Eigen::Vector3d(1.0, -2.0, 0.5);
  point.velocity = Eigen::Vector3d(0.5, 1.0, -0.25);

  Eigen::Vector3d predicted;
  double dt = -1.0;
  ASSERT_TRUE(plan_env::DynamicPointBuffer::predictPosition(
      point, timeFromSec(10.0), timeFromSec(11.5), 2.0, predicted, dt));
  EXPECT_NEAR(dt, 1.5, 1e-12);
  EXPECT_TRUE(predicted.isApprox(Eigen::Vector3d(1.75, -0.5, 0.125), 1e-12));
}

TEST(DynamicPointBuffer, PredictionRejectsInvalidTimeBasis)
{
  plan_env::DynamicPoint point;
  Eigen::Vector3d predicted;
  double dt = 0.0;

  EXPECT_FALSE(plan_env::DynamicPointBuffer::predictPosition(
      point, timeFromSec(10.0), timeFromSec(9.9), 2.0, predicted, dt));
  EXPECT_FALSE(plan_env::DynamicPointBuffer::predictPosition(
      point, timeFromSec(10.0), timeFromSec(12.1), 2.0, predicted, dt));
}

TEST(DynamicPointBuffer, UncertaintyGrowsMonotonically)
{
  const double uncertainty_0 =
      plan_env::DynamicPointBuffer::predictionUncertainty(0.0, 0.03, 0.15, 0.5);
  const double uncertainty_1 =
      plan_env::DynamicPointBuffer::predictionUncertainty(1.0, 0.03, 0.15, 0.5);
  const double uncertainty_2 =
      plan_env::DynamicPointBuffer::predictionUncertainty(2.0, 0.03, 0.15, 0.5);

  EXPECT_NEAR(uncertainty_0, 0.03, 1e-12);
  EXPECT_LT(uncertainty_0, uncertainty_1);
  EXPECT_LT(uncertainty_1, uncertainty_2);
}

TEST(DynamicPointBuffer, DistanceQueryIsConservative)
{
  plan_env::DynamicPointBuffer buffer;
  plan_env::DynamicPointSnapshot snapshot;
  snapshot.valid = true;
  snapshot.stamp = timeFromSec(10.0);
  plan_env::DynamicPoint point;
  point.position = Eigen::Vector3d(1.0, 0.0, 0.0);
  point.velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  snapshot.points.push_back(point);

  double distance = 0.0;
  ASSERT_TRUE(buffer.getPredictedDistance(
      snapshot, Eigen::Vector3d::Zero(), timeFromSec(11.0), 10.0, distance));
  // Raw distance is 2.0 m. Default uncertainty at 1 s is
  // 0.03 + 0.15 + 0.5 * 0.5 = 0.43 m.
  EXPECT_NEAR(distance, 1.57, 1e-12);
}

TEST(DynamicPointBuffer, EmptyAcceptedObservationMeansClear)
{
  plan_env::DynamicPointBuffer buffer;
  plan_env::DynamicPointSnapshot snapshot;
  snapshot.valid = true;
  snapshot.stamp = timeFromSec(10.0);

  double distance = 0.0;
  ASSERT_TRUE(buffer.getPredictedDistance(
      snapshot, Eigen::Vector3d::Zero(), timeFromSec(10.5), 3.0, distance));
  EXPECT_DOUBLE_EQ(distance, 3.0);
}

TEST(DynamicPointBuffer, PredictedBoundsIncludeMotionAndUncertainty)
{
  plan_env::DynamicPointBuffer buffer;
  plan_env::DynamicPointSnapshot snapshot;
  snapshot.valid = true;
  snapshot.stamp = timeFromSec(10.0);
  plan_env::DynamicPoint point;
  point.position = Eigen::Vector3d(1.0, 2.0, 0.0);
  point.velocity = Eigen::Vector3d(1.0, -1.0, 0.5);
  snapshot.points.push_back(point);

  Eigen::Vector3d minimum, maximum;
  bool has_points = false;
  ASSERT_TRUE(buffer.getPredictedBounds(
      snapshot, timeFromSec(11.0), minimum, maximum, has_points));
  ASSERT_TRUE(has_points);
  // Nominal position is [2, 1, 0.5], with the default 1 s uncertainty 0.43 m.
  EXPECT_TRUE(minimum.isApprox(Eigen::Vector3d(1.57, 0.57, 0.07), 1e-12));
  EXPECT_TRUE(maximum.isApprox(Eigen::Vector3d(2.43, 1.43, 0.93), 1e-12));
}

TEST(DynamicPointBuffer, EmptyBoundsStillEnforcePredictionHorizon)
{
  plan_env::DynamicPointBuffer buffer;
  plan_env::DynamicPointSnapshot snapshot;
  snapshot.valid = true;
  snapshot.stamp = timeFromSec(10.0);

  Eigen::Vector3d minimum, maximum;
  bool has_points = true;
  EXPECT_FALSE(buffer.getPredictedBounds(
      snapshot, timeFromSec(12.1), minimum, maximum, has_points));
  EXPECT_FALSE(has_points);
}

TEST(DynamicPointBuffer, PredictedPositionsShareOneUncertaintyEnvelope)
{
  plan_env::DynamicPointBuffer buffer;
  plan_env::DynamicPointSnapshot snapshot;
  snapshot.valid = true;
  snapshot.stamp = timeFromSec(10.0);
  plan_env::DynamicPoint first;
  first.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  first.velocity = Eigen::Vector3d(0.5, -1.0, 0.0);
  snapshot.points.push_back(first);
  plan_env::DynamicPoint second;
  second.position = Eigen::Vector3d(-1.0, 0.0, 0.5);
  second.velocity = Eigen::Vector3d(0.0, 2.0, -0.5);
  snapshot.points.push_back(second);

  plan_env::PredictedPositionVector positions;
  double uncertainty = -1.0;
  ASSERT_TRUE(buffer.getPredictedPositions(
      snapshot, timeFromSec(11.0), positions, uncertainty));
  ASSERT_EQ(positions.size(), 2u);
  EXPECT_TRUE(positions[0].isApprox(
      Eigen::Vector3d(1.5, 1.0, 3.0), 1e-12));
  EXPECT_TRUE(positions[1].isApprox(
      Eigen::Vector3d(-1.0, 2.0, 0.0), 1e-12));
  EXPECT_NEAR(uncertainty, 0.43, 1e-12);
}

TEST(DynamicPointBuffer, EmptyPredictedPositionsStillEnforcePredictionHorizon)
{
  plan_env::DynamicPointBuffer buffer;
  plan_env::DynamicPointSnapshot snapshot;
  snapshot.valid = true;
  snapshot.stamp = timeFromSec(10.0);

  plan_env::PredictedPositionVector positions;
  double uncertainty = -1.0;
  EXPECT_FALSE(buffer.getPredictedPositions(
      snapshot, timeFromSec(12.1), positions, uncertainty));
  EXPECT_TRUE(positions.empty());
}

TEST(DynamicPointBuffer, EmptyDetectorFramesRequireDebouncedClear)
{
  EXPECT_FALSE(
      plan_env::DynamicPointBuffer::emptyObservationConfirmed(1, 3));
  EXPECT_FALSE(
      plan_env::DynamicPointBuffer::emptyObservationConfirmed(2, 3));
  EXPECT_TRUE(
      plan_env::DynamicPointBuffer::emptyObservationConfirmed(3, 3));
}

TEST(DynamicPointBuffer, UnconfirmedEmptyFramesRetainOnlyValidHistory)
{
  EXPECT_TRUE(
      plan_env::DynamicPointBuffer::shouldRetainLastValidSnapshot(
          1, 3, true));
  EXPECT_TRUE(
      plan_env::DynamicPointBuffer::shouldRetainLastValidSnapshot(
          2, 3, true));
  EXPECT_FALSE(
      plan_env::DynamicPointBuffer::shouldRetainLastValidSnapshot(
          3, 3, true));
  EXPECT_FALSE(
      plan_env::DynamicPointBuffer::shouldRetainLastValidSnapshot(
          1, 3, false));
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

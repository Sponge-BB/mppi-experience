#include <gtest/gtest.h>
#include <initializer_list>

#include <plan_env/dynamic_point_buffer.h>

class DynamicPointBufferTestAccessor
{
public:
  static void feed(plan_env::DynamicPointBuffer &buffer,
                   const plan_env::DynamicPointSnapshot &incoming)
  {
    ros::Time::setNow(incoming.receive_time);
    buffer.enabled_ = true;
    buffer.recordObservation(incoming);
  }


};

namespace
{

ros::Time timeFromSec(double seconds)
{
  ros::Time time;
  time.fromSec(seconds);
  return time;
}

plan_env::DynamicPointSnapshot observation(
    double stamp, std::initializer_list<std::uint32_t> ids)
{
  plan_env::DynamicPointSnapshot value;
  value.stamp = value.receive_time = timeFromSec(stamp);
  value.frame_id = "world";
  value.valid = value.has_track_ids = true;
  for (const auto id : ids)
  {
    plan_env::DynamicPoint point;
    point.track_id = id;
    point.position = Eigen::Vector3d(id, 0.0, 1.0);
    point.velocity = Eigen::Vector3d(0.1, 0.0, 0.0);
    value.points.push_back(point);
  }
  return value;
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

TEST(DynamicPointBuffer, InitialEmptyMeansNoDynamicDetectionNotFreeSpace)
{
  plan_env::DynamicPointBuffer buffer;
  const auto snapshot = plan_env::DynamicPointBuffer::applyObservation(
      plan_env::DynamicPointSnapshot(), observation(10.0, {}));
  ASSERT_TRUE(snapshot.valid);
  EXPECT_EQ(snapshot.invalid_reason, "no_dynamic_detected");

  // A neutral contribution from the dynamic layer is not a free-space proof.
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

TEST(DynamicPointBuffer, FreshNoMotionFrameReplacesPredictionWithoutRestampingOldTargets)
{
  const auto old = observation(10., {1});
  const auto current = plan_env::DynamicPointBuffer::applyObservation(
      old, observation(10.1, {}));
  ASSERT_TRUE(current.valid);
  EXPECT_TRUE(current.points.empty());
  EXPECT_EQ("no_dynamic_detected", current.invalid_reason);
  EXPECT_EQ(timeFromSec(10.1), current.stamp);
  EXPECT_EQ(timeFromSec(10.), old.stamp);
  EXPECT_EQ(1u, old.points.size());
}

TEST(DynamicPointBuffer, PartialIdentityLossDoesNotInvalidateFreshRemainingPredictions)
{
  const auto old = observation(10., {1, 2});
  const auto current = plan_env::DynamicPointBuffer::applyObservation(
      old, observation(10.1, {2, 3}));
  ASSERT_TRUE(current.valid);
  ASSERT_EQ(2u, current.points.size());
  EXPECT_EQ(2u, current.points.front().track_id);
  EXPECT_EQ(3u, current.points.back().track_id);
  EXPECT_EQ(timeFromSec(10.1), current.stamp);
}

TEST(DynamicPointBuffer, PredictionDoesNotRequirePersistentTrackIdentity)
{
  auto current = observation(10.1, {0});
  current.has_track_ids = false;
  const auto accepted = plan_env::DynamicPointBuffer::applyObservation(
      observation(10., {1}), current);
  EXPECT_TRUE(accepted.valid);
  EXPECT_FALSE(accepted.has_track_ids);
  EXPECT_EQ(current.stamp, accepted.stamp);
}

TEST(DynamicPointBuffer, RepeatedIdChangesCannotExhaustHistoricalRecoveryState)
{
  plan_env::DynamicPointBuffer buffer;
  for (std::uint32_t i = 1; i <= 1000; ++i)
  {
    const double stamp = 10. + .1*i;
    DynamicPointBufferTestAccessor::feed(buffer, observation(stamp, {i}));
    const auto current = buffer.getSnapshot(timeFromSec(stamp));
    ASSERT_TRUE(current.valid);
    ASSERT_EQ(1u, current.points.size());
    EXPECT_EQ(i, current.points.front().track_id);
    EXPECT_EQ(i, current.sequence);
  }
}

TEST(DynamicPointBuffer, StaleAndFutureStreamsRemainUnavailableEvenWhenEmpty)
{
  for (const auto ids : {std::initializer_list<std::uint32_t>{},
                        std::initializer_list<std::uint32_t>{1u}})
  {
    plan_env::DynamicPointBuffer buffer;
    DynamicPointBufferTestAccessor::feed(buffer, observation(10., ids));
    const auto stale = buffer.getSnapshot(timeFromSec(10.4));
    EXPECT_FALSE(stale.valid);
    EXPECT_EQ("stale", stale.invalid_reason);
    const auto future = buffer.getSnapshot(timeFromSec(9.8));
    EXPECT_FALSE(future.valid);
    EXPECT_EQ("future_stamp", future.invalid_reason);
  }
}

TEST(DynamicPointBuffer, OlderObservationCannotReplaceNewerRealSourceEpoch)
{
  plan_env::DynamicPointBuffer buffer;
  DynamicPointBufferTestAccessor::feed(buffer, observation(10.2, {2}));
  DynamicPointBufferTestAccessor::feed(buffer, observation(10.1, {1}));
  const auto current = buffer.getSnapshot(timeFromSec(10.2));
  ASSERT_TRUE(current.valid);
  ASSERT_EQ(1u, current.points.size());
  EXPECT_EQ(2u, current.points.front().track_id);
  EXPECT_EQ(timeFromSec(10.2), current.stamp);
  EXPECT_EQ(1u, current.sequence);
}

TEST(DynamicPointBuffer, FutureOrDuplicatePacketCannotPoisonFollowingValidInput)
{
  plan_env::DynamicPointBuffer buffer;
  DynamicPointBufferTestAccessor::feed(buffer, observation(10., {1}));
  auto future = observation(90., {9});
  future.receive_time = timeFromSec(10.1);
  DynamicPointBufferTestAccessor::feed(buffer, future);
  EXPECT_EQ(timeFromSec(10.), buffer.getSnapshot(timeFromSec(10.1)).stamp);
  DynamicPointBufferTestAccessor::feed(buffer, observation(10.2, {2}));
  DynamicPointBufferTestAccessor::feed(buffer, observation(10.2, {9}));
  const auto current = buffer.getSnapshot(timeFromSec(10.2));
  ASSERT_TRUE(current.valid);
  ASSERT_EQ(1u, current.points.size());
  EXPECT_EQ(2u, current.points.front().track_id);
  EXPECT_EQ(2u, current.sequence);
}

}  // namespace

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}

#include <mm_remani/dynamic_obstacle_tracker_core.h>

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

namespace mm_remani
{
namespace
{

DynamicDetection detection(double x, double y = 0.0, double z = 0.5)
{
  DynamicDetection value;
  value.centroid = Eigen::Vector3d(x, y, z);
  return value;
}

DynamicDetection shapedDetection(
    double x, double extent_x, std::size_t point_count)
{
  DynamicDetection value = detection(x);
  value.extent = Eigen::Vector3d(extent_x, 0.30, 0.80);
  value.point_count = point_count;
  return value;
}

std::vector<Eigen::Vector3f> articulatedTargetPoints(
    double center_x, double leg_side)
{
  std::vector<Eigen::Vector3f> points;
  // A symmetric middle/upper body supplies a stable motion anchor.
  for (const double z : {0.70, 1.00})
  {
    for (const double x_offset : {-0.10, 0.10})
    {
      for (const double y_offset : {-0.10, 0.10})
      {
        points.emplace_back(
            static_cast<float>(center_x + x_offset),
            static_cast<float>(y_offset), static_cast<float>(z));
      }
    }
  }
  points.emplace_back(static_cast<float>(center_x), -0.08f, 1.40f);
  points.emplace_back(static_cast<float>(center_x), 0.08f, 1.40f);

  // Deliberately over-represent whichever leg is currently visible.  The
  // complete collision shape must retain these points, but the target motion
  // centre must not follow them sideways.
  for (int index = 0; index < 10; ++index)
  {
    const double x_offset = 0.01 * static_cast<double>(index % 3);
    points.emplace_back(
        static_cast<float>(center_x + x_offset),
        static_cast<float>(leg_side),
        static_cast<float>(0.02 * static_cast<double>(index % 2)));
  }
  return points;
}

TEST(StableMotionCenter, RejectsAlternatingLegSurfaceBias)
{
  StableMotionCenterParameters parameters;
  parameters.trim_fraction = 0.15;
  parameters.tall_target_min_height = 0.65;
  parameters.stable_core_lower_height_ratio = 0.35;
  parameters.stable_core_upper_height_ratio = 0.90;
  parameters.minimum_stable_core_points = 8;

  std::vector<Eigen::Vector3f> left_visible =
      articulatedTargetPoints(2.0, -0.32);
  std::vector<Eigen::Vector3f> right_visible =
      articulatedTargetPoints(2.1, 0.32);
  const std::size_t left_shape_size = left_visible.size();
  const std::size_t right_shape_size = right_visible.size();
  bool left_used_core = false;
  bool right_used_core = false;

  const Eigen::Vector3d left_center = estimateStableMotionCenter(
      left_visible, parameters, &left_used_core);
  const Eigen::Vector3d right_center = estimateStableMotionCenter(
      right_visible, parameters, &right_used_core);

  EXPECT_TRUE(left_used_core);
  EXPECT_TRUE(right_used_core);
  EXPECT_NEAR(left_center.x(), 2.0, 1e-6);
  EXPECT_NEAR(right_center.x(), 2.1, 1e-6);
  EXPECT_NEAR(left_center.y(), 0.0, 1e-6);
  EXPECT_NEAR(right_center.y(), 0.0, 1e-6);
  EXPECT_EQ(left_shape_size, left_visible.size());
  EXPECT_EQ(right_shape_size, right_visible.size());
}

TEST(StableMotionCenter, LowObstacleUsesCompleteRobustShape)
{
  StableMotionCenterParameters parameters;
  std::vector<Eigen::Vector3f> points{
      Eigen::Vector3f(0.9f, -0.2f, 0.2f),
      Eigen::Vector3f(1.0f, -0.1f, 0.3f),
      Eigen::Vector3f(1.0f, 0.1f, 0.3f),
      Eigen::Vector3f(1.1f, 0.2f, 0.4f)};
  bool used_core = true;
  const Eigen::Vector3d center =
      estimateStableMotionCenter(points, parameters, &used_core);
  EXPECT_FALSE(used_core);
  EXPECT_NEAR(center.x(), 1.0, 1e-6);
  EXPECT_NEAR(center.y(), 0.0, 1e-6);
  EXPECT_NEAR(center.z(), 0.3, 1e-6);
}

TEST(DynamicObstacleTrackerCore, ConfirmsAndEstimatesForwardVelocity)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  DynamicObstacleTrackerCore tracker(parameters);

  const auto first = tracker.update({detection(0.0)}, 10.0);
  ASSERT_EQ(first.size(), 1u);
  EXPECT_FALSE(first.front().confirmed);
  const std::uint32_t id = first.front().track_id;

  const auto second = tracker.update({detection(0.10)}, 10.10);
  ASSERT_EQ(second.size(), 1u);
  EXPECT_EQ(second.front().track_id, id);
  EXPECT_TRUE(second.front().confirmed);
  EXPECT_FALSE(second.front().motion_qualified);
  EXPECT_NEAR(second.front().velocity.x(), 1.0, 1e-9);
  EXPECT_NEAR(second.front().velocity.y(), 0.0, 1e-9);
}

TEST(DynamicObstacleTrackerCore, AssociatesUsingConstantVelocityPrediction)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.08;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  DynamicObstacleTrackerCore tracker(parameters);

  const auto first = tracker.update({detection(0.0)}, 1.0);
  const auto second = tracker.update({detection(0.05)}, 1.1);
  ASSERT_EQ(first.size(), 1u);
  ASSERT_EQ(second.size(), 1u);
  const std::uint32_t id = second.front().track_id;

  // This point is 0.1 m from the last measurement, outside the 0.08 m gate,
  // but exactly at the track's CV prediction and must keep the same identity.
  const auto third = tracker.update({detection(0.15)}, 1.3);
  ASSERT_EQ(third.size(), 1u);
  EXPECT_EQ(third.front().track_id, id);
  EXPECT_TRUE(third.front().confirmed);
}

TEST(DynamicObstacleTrackerCore, RejectsNonMonotonicAndInvalidTimestamps)
{
  DynamicObstacleTrackerCore tracker;
  tracker.update({detection(0.0)}, 5.0);
  ASSERT_EQ(tracker.trackCount(), 1u);

  EXPECT_TRUE(tracker.update({detection(0.1)}, 5.0).empty());
  EXPECT_TRUE(tracker.update(
      {detection(0.1)},
      std::numeric_limits<double>::quiet_NaN()).empty());
  EXPECT_DOUBLE_EQ(tracker.lastStamp(), 5.0);
  EXPECT_EQ(tracker.trackCount(), 1u);
}

TEST(DynamicObstacleTrackerCore, ExpiresOnlyAfterConfiguredMissBudget)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.max_missed_frames = 2;
  DynamicObstacleTrackerCore tracker(parameters);
  tracker.update({detection(0.0)}, 0.0);

  tracker.update({}, 0.1);
  EXPECT_EQ(tracker.trackCount(), 1u);
  tracker.update({}, 0.2);
  EXPECT_EQ(tracker.trackCount(), 1u);
  tracker.update({}, 0.3);
  EXPECT_EQ(tracker.trackCount(), 0u);
}

TEST(DynamicObstacleTrackerCore, AppliesVelocityDeadbandAndSpeedCap)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.2;
  parameters.max_speed = 1.5;
  parameters.association_gate = 2.0;
  parameters.association_position_allowance = 2.0;
  parameters.min_confirmed_hits = 1;
  DynamicObstacleTrackerCore tracker(parameters);

  tracker.update({detection(0.0)}, 0.0);
  auto tracked = tracker.update({detection(0.01)}, 0.1);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_DOUBLE_EQ(tracked.front().velocity.norm(), 0.0);

  tracked = tracker.update({detection(1.01)}, 0.2);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_NEAR(tracked.front().velocity.norm(), 1.5, 1e-9);
}

TEST(DynamicObstacleTrackerCore, MatureStationaryResidualIsNotDynamic)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.dynamic_enter_speed = 0.18;
  parameters.minimum_dynamic_hits = 3;
  parameters.minimum_dynamic_displacement = 0.06;
  DynamicObstacleTrackerCore tracker(parameters);

  const std::vector<double> jitter = {
      0.000, 0.005, -0.004, 0.006, -0.003, 0.004};
  for (std::size_t index = 0; index < jitter.size(); ++index)
  {
    const auto tracked = tracker.update(
        {detection(jitter[index])},
        1.0 + 0.1 * static_cast<double>(index));
    ASSERT_EQ(tracked.size(), 1u);
    if (index > 0)
      EXPECT_TRUE(tracked.front().confirmed);
    EXPECT_FALSE(tracked.front().motion_qualified);
  }
}

TEST(DynamicObstacleTrackerCore, QualifiesOnlyPersistentCoherentMotion)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.dynamic_enter_speed = 0.18;
  parameters.dynamic_exit_speed = 0.10;
  parameters.minimum_dynamic_hits = 3;
  parameters.minimum_dynamic_displacement = 0.06;
  parameters.direction_consistency_cosine = 0.30;
  DynamicObstacleTrackerCore tracker(parameters);

  tracker.update({detection(0.00)}, 2.0);
  auto tracked = tracker.update({detection(0.03)}, 2.1);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_FALSE(tracked.front().motion_qualified);
  tracked = tracker.update({detection(0.06)}, 2.2);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_FALSE(tracked.front().motion_qualified);
  tracked = tracker.update({detection(0.09)}, 2.3);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_TRUE(tracked.front().confirmed);
  EXPECT_TRUE(tracked.front().motion_qualified);
}

TEST(DynamicObstacleTrackerCore,
     HistoryRegressionSuppressesStraightWalkingLateralGait)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.regression_velocity_weight = 0.75;
  parameters.motion_direction_alpha = 0.30;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.dynamic_enter_speed = 0.18;
  parameters.dynamic_exit_speed = 0.08;
  parameters.minimum_dynamic_hits = 3;
  parameters.minimum_motion_duration = 0.25;
  parameters.motion_history_duration = 0.80;
  parameters.minimum_dynamic_displacement = 0.08;
  parameters.minimum_motion_efficiency = 0.60;
  parameters.minimum_direction_consistency_fraction = 0.60;
  parameters.minimum_motion_cluster_points = 15;
  DynamicObstacleTrackerCore tracker(parameters);

  std::vector<TrackedDetection> tracked;
  for (int index = 0; index < 6; ++index)
  {
    tracked = tracker.update(
        {shapedDetection(0.05 * index, 0.45, 40)},
        60.0 + 0.1 * index);
  }
  ASSERT_EQ(tracked.size(), 1u);
  ASSERT_TRUE(tracked.front().motion_qualified);

  for (int index = 6; index < 12; ++index)
  {
    DynamicDetection walking =
        shapedDetection(0.05 * index, 0.45, 40);
    walking.centroid.y() = index % 2 == 0 ? 0.03 : -0.03;
    tracked = tracker.update(
        {walking}, 60.0 + 0.1 * index);
    ASSERT_EQ(tracked.size(), 1u);
    ASSERT_TRUE(tracked.front().motion_qualified);
  }

  EXPECT_NEAR(tracked.front().velocity.x(), 0.50, 0.10);
  EXPECT_LT(std::fabs(tracked.front().velocity.y()), 0.15);
}

TEST(DynamicObstacleTrackerCore, RejectsAlternatingCentroidJitter)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.dynamic_enter_speed = 0.18;
  parameters.minimum_dynamic_hits = 2;
  parameters.minimum_dynamic_displacement = 0.05;
  parameters.direction_consistency_cosine = 0.30;
  DynamicObstacleTrackerCore tracker(parameters);

  const std::vector<double> alternating = {
      0.00, 0.04, 0.00, 0.04, 0.00, 0.04};
  for (std::size_t index = 0; index < alternating.size(); ++index)
  {
    const auto tracked = tracker.update(
        {detection(alternating[index])},
        4.0 + 0.1 * static_cast<double>(index));
    ASSERT_EQ(tracked.size(), 1u);
    EXPECT_FALSE(tracked.front().motion_qualified);
  }
}

TEST(DynamicObstacleTrackerCore, MotionQualificationUsesExitHysteresis)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.dynamic_enter_speed = 0.18;
  parameters.dynamic_exit_speed = 0.10;
  parameters.minimum_dynamic_hits = 2;
  parameters.minimum_stationary_hits = 3;
  parameters.minimum_dynamic_displacement = 0.05;
  parameters.minimum_motion_duration = 0.15;
  DynamicObstacleTrackerCore tracker(parameters);

  tracker.update({detection(0.00)}, 3.0);
  tracker.update({detection(0.03)}, 3.1);
  auto tracked = tracker.update({detection(0.06)}, 3.2);
  ASSERT_EQ(tracked.size(), 1u);
  ASSERT_TRUE(tracked.front().motion_qualified);

  // A velocity between the enter and exit thresholds must retain the dynamic
  // state instead of chattering on every scan.
  tracked = tracker.update({detection(0.072)}, 3.3);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_TRUE(tracked.front().motion_qualified);

  tracked = tracker.update({detection(0.077)}, 3.4);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_TRUE(tracked.front().motion_qualified);
  tracked = tracker.update({detection(0.082)}, 3.5);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_TRUE(tracked.front().motion_qualified);
  tracked = tracker.update({detection(0.087)}, 3.6);
  ASSERT_EQ(tracked.size(), 1u);
  EXPECT_FALSE(tracked.front().motion_qualified);
}

TEST(DynamicObstacleTrackerCore,
     StabilizedPredictionDoesNotDelayStoppedTargetExit)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.position_alpha = 0.65;
  parameters.velocity_alpha = 0.30;
  parameters.regression_velocity_weight = 0.75;
  parameters.motion_direction_alpha = 0.30;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.dynamic_enter_speed = 0.18;
  parameters.dynamic_exit_speed = 0.10;
  parameters.minimum_dynamic_hits = 3;
  parameters.minimum_stationary_hits = 3;
  parameters.minimum_dynamic_displacement = 0.08;
  parameters.minimum_motion_duration = 0.25;
  DynamicObstacleTrackerCore tracker(parameters);

  std::vector<TrackedDetection> tracked;
  for (int index = 0; index < 4; ++index)
  {
    tracked = tracker.update(
        {detection(0.04 * index)}, 8.0 + 0.1 * index);
  }
  ASSERT_EQ(tracked.size(), 1u);
  ASSERT_TRUE(tracked.front().motion_qualified);

  // The prediction output remains history-stabilized, but three genuinely
  // stationary target-level observations must still close the dynamic state.
  tracked = tracker.update({detection(0.12)}, 8.4);
  ASSERT_TRUE(tracked.front().motion_qualified);
  tracked = tracker.update({detection(0.12)}, 8.5);
  ASSERT_TRUE(tracked.front().motion_qualified);
  tracked = tracker.update({detection(0.12)}, 8.6);
  ASSERT_FALSE(tracked.front().motion_qualified);
}

TEST(DynamicObstacleTrackerCore, RejectsLowEfficiencyCentroidWander)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 3;
  parameters.dynamic_enter_speed = 0.18;
  parameters.minimum_dynamic_hits = 3;
  parameters.minimum_dynamic_displacement = 0.06;
  parameters.minimum_motion_duration = 0.25;
  parameters.minimum_motion_efficiency = 0.70;
  parameters.minimum_direction_consistency_fraction = 0.70;
  DynamicObstacleTrackerCore tracker(parameters);

  // The centroid makes locally forward-looking steps but repeatedly folds
  // back because a static cluster is splitting and merging.  Instantaneous
  // velocity can be high; net/path efficiency must keep it non-dynamic.
  const std::vector<double> wander = {
      0.00, 0.04, 0.08, 0.04, 0.08, 0.12, 0.08, 0.12};
  for (std::size_t index = 0; index < wander.size(); ++index)
  {
    const auto tracked = tracker.update(
        {detection(wander[index])},
        5.0 + 0.1 * static_cast<double>(index));
    ASSERT_EQ(tracked.size(), 1u);
    EXPECT_FALSE(tracked.front().motion_qualified);
  }
}

TEST(DynamicObstacleTrackerCore, ClusterShapeChangeStartsANewTrack)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.maximum_extent_change = 0.30;
  parameters.maximum_point_count_ratio = 2.5;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.minimum_dynamic_hits = 2;
  parameters.minimum_motion_duration = 0.10;
  parameters.minimum_dynamic_displacement = 0.05;
  DynamicObstacleTrackerCore tracker(parameters);

  tracker.update({shapedDetection(0.00, 0.40, 100)}, 6.0);
  const auto stable = tracker.update(
      {shapedDetection(0.03, 0.42, 105)}, 6.1);
  ASSERT_EQ(stable.size(), 1u);
  const std::uint32_t stable_id = stable.front().track_id;

  // A merge changes extent/count and shifts the centroid.  Associating it to
  // the old residual would synthesize a large velocity, so it must start a
  // fresh, unconfirmed identity.
  const auto merged = tracker.update(
      {shapedDetection(0.23, 1.10, 320)}, 6.2);
  ASSERT_EQ(merged.size(), 1u);
  EXPECT_NE(merged.front().track_id, stable_id);
  EXPECT_FALSE(merged.front().confirmed);
  EXPECT_FALSE(merged.front().motion_qualified);
}

TEST(DynamicObstacleTrackerCore, AdaptiveGateRejectsImplausibleFirstJump)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.80;
  parameters.association_position_allowance = 0.10;
  parameters.max_speed = 1.0;
  parameters.min_confirmed_hits = 2;
  DynamicObstacleTrackerCore tracker(parameters);

  const auto first = tracker.update({detection(0.0)}, 7.0);
  ASSERT_EQ(first.size(), 1u);
  const auto jumped = tracker.update({detection(0.40)}, 7.1);
  ASSERT_EQ(jumped.size(), 1u);
  EXPECT_NE(jumped.front().track_id, first.front().track_id);
  EXPECT_FALSE(jumped.front().motion_qualified);
}

TEST(DynamicObstacleTrackerCore,
     LearnsStationaryResidualAndSuppressesLaterCentroidDrift)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.minimum_dynamic_hits = 2;
  parameters.minimum_motion_duration = 0.20;
  parameters.minimum_dynamic_displacement = 0.06;
  parameters.dynamic_enter_speed = 0.18;
  parameters.max_missed_frames = 2;
  parameters.stationary_background_minimum_duration = 0.50;
  parameters.minimum_stationary_background_hits = 5;
  parameters.stationary_background_maximum_speed = 0.10;
  parameters.stationary_background_maximum_displacement = 0.06;
  parameters.stationary_background_merge_radius = 0.20;
  parameters.stationary_background_release_radius = 0.25;
  DynamicObstacleTrackerCore tracker(parameters);

  // This return is missing from the PCD but remains fixed in the world.  It
  // is learned only after a full second of stable evidence.
  for (int index = 0; index < 10; ++index)
  {
    const double jitter = index % 2 == 0 ? -0.005 : 0.005;
    const auto tracked = tracker.update(
        {detection(jitter)}, 10.0 + 0.1 * index);
    ASSERT_EQ(tracked.size(), 1u);
    EXPECT_FALSE(tracked.front().motion_qualified);
  }
  ASSERT_EQ(tracker.stationaryBackgroundAnchorCount(), 1u);

  // Expire the identity to reproduce cluster split/merge.  A new centroid
  // can then drift coherently, but motion inside the learned residual region
  // must not be exposed to predictive planning.
  tracker.update({}, 11.0);
  tracker.update({}, 11.1);
  tracker.update({}, 11.2);
  const std::vector<double> drift = {
      0.00, 0.04, 0.08, 0.12, 0.16, 0.20, 0.24};
  for (std::size_t index = 0; index < drift.size(); ++index)
  {
    const auto tracked = tracker.update(
        {detection(drift[index])}, 12.0 + 0.1 * index);
    ASSERT_EQ(tracked.size(), 1u);
    EXPECT_FALSE(tracked.front().motion_qualified);
  }
}

TEST(DynamicObstacleTrackerCore,
     MovingObjectCanLeaveStationaryResidualAnchor)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.minimum_dynamic_hits = 2;
  parameters.minimum_motion_duration = 0.20;
  parameters.minimum_dynamic_displacement = 0.06;
  parameters.dynamic_enter_speed = 0.18;
  parameters.stationary_background_minimum_duration = 0.30;
  parameters.minimum_stationary_background_hits = 2;
  parameters.stationary_background_maximum_speed = 0.10;
  parameters.stationary_background_maximum_displacement = 0.04;
  parameters.stationary_background_release_radius = 0.25;
  DynamicObstacleTrackerCore tracker(parameters);

  for (int index = 0; index < 6; ++index)
    tracker.update({detection(0.0)}, 20.0 + 0.1 * index);
  ASSERT_EQ(tracker.stationaryBackgroundAnchorCount(), 1u);

  bool qualified_after_release = false;
  const std::vector<double> motion = {
      0.04, 0.08, 0.12, 0.16, 0.20, 0.24, 0.28, 0.32};
  for (std::size_t index = 0; index < motion.size(); ++index)
  {
    const auto tracked = tracker.update(
        {detection(motion[index])}, 20.6 + 0.1 * index);
    ASSERT_EQ(tracked.size(), 1u);
    if (motion[index] <= 0.24)
      EXPECT_FALSE(tracked.front().motion_qualified);
    if (motion[index] >= 0.28 && tracked.front().motion_qualified)
      qualified_after_release = true;
  }
  EXPECT_TRUE(qualified_after_release);
}

TEST(DynamicObstacleTrackerCore,
     TrackResetMayPreserveOrClearStationaryResidualMemory)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.stationary_background_minimum_duration = 0.20;
  parameters.minimum_stationary_background_hits = 2;
  DynamicObstacleTrackerCore tracker(parameters);
  for (int index = 0; index < 5; ++index)
    tracker.update({detection(0.0)}, 30.0 + 0.1 * index);
  ASSERT_EQ(tracker.stationaryBackgroundAnchorCount(), 1u);

  tracker.reset(false);
  EXPECT_EQ(tracker.trackCount(), 0u);
  EXPECT_EQ(tracker.stationaryBackgroundAnchorCount(), 1u);
  tracker.reset();
  EXPECT_EQ(tracker.stationaryBackgroundAnchorCount(), 0u);
}

TEST(DynamicObstacleTrackerCore,
     BoundedLongTermCentroidFlickerBecomesBackground)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.60;
  parameters.association_position_allowance = 0.60;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.stationary_background_minimum_duration = 10.0;
  parameters.stationary_background_maximum_displacement = 0.001;
  parameters.stationary_background_maximum_speed = 0.001;
  parameters.stationary_background_dwell_duration = 2.0;
  parameters.stationary_background_dwell_radius = 0.30;
  DynamicObstacleTrackerCore tracker(parameters);

  // This deliberately defeats the low-speed learner: the centroid jumps by
  // decimetres as a static surface fragments, but never leaves one patch.
  const std::vector<double> flicker = {
      0.00, 0.15, -0.08, 0.18, -0.05, 0.16, -0.09, 0.12,
      -0.06, 0.17, -0.04, 0.13, -0.07, 0.16, -0.05, 0.14,
      -0.08, 0.17, -0.03, 0.15, -0.06, 0.16, -0.04};
  for (std::size_t index = 0; index < flicker.size(); ++index)
    tracker.update({detection(flicker[index])},
                   40.0 + 0.1 * static_cast<double>(index));
  EXPECT_EQ(tracker.stationaryBackgroundAnchorCount(), 1u);
}

TEST(DynamicObstacleTrackerCore,
     SparseClusterCannotEnterPredictiveMotionLayer)
{
  DynamicObstacleTrackerCore::Parameters parameters;
  parameters.association_gate = 0.50;
  parameters.association_position_allowance = 0.50;
  parameters.position_alpha = 1.0;
  parameters.velocity_alpha = 1.0;
  parameters.velocity_deadband = 0.0;
  parameters.min_confirmed_hits = 2;
  parameters.minimum_dynamic_hits = 2;
  parameters.minimum_motion_duration = 0.20;
  parameters.minimum_dynamic_displacement = 0.06;
  parameters.dynamic_enter_speed = 0.18;
  parameters.minimum_motion_cluster_points = 15;
  DynamicObstacleTrackerCore tracker(parameters);

  for (int index = 0; index < 6; ++index)
  {
    const auto tracked = tracker.update(
        {shapedDetection(0.04 * index, 0.20, 10)},
        50.0 + 0.1 * index);
    ASSERT_EQ(tracked.size(), 1u);
    EXPECT_FALSE(tracked.front().motion_qualified);
  }
}

TEST(DynamicPerceptionHealthGate, StartsClosedAndNeedsStableWarmup)
{
  DynamicPerceptionHealthGate::Parameters parameters;
  parameters.maximum_motion_tracks = 3;
  parameters.recovery_frames = 4;
  parameters.degradation_frames = 2;
  DynamicPerceptionHealthGate gate(parameters);

  EXPECT_FALSE(gate.healthy());
  EXPECT_FALSE(gate.update(0));
  EXPECT_FALSE(gate.update(1));
  EXPECT_FALSE(gate.update(2));
  EXPECT_TRUE(gate.update(1));
  EXPECT_EQ(gate.state(),
            DynamicPerceptionHealthGate::State::HEALTHY);
}

TEST(DynamicPerceptionHealthGate, DegradesAndRequiresFullRecovery)
{
  DynamicPerceptionHealthGate::Parameters parameters;
  parameters.maximum_motion_tracks = 2;
  parameters.recovery_frames = 3;
  parameters.degradation_frames = 2;
  DynamicPerceptionHealthGate gate(parameters);

  gate.update(0);
  gate.update(1);
  ASSERT_TRUE(gate.update(2));
  EXPECT_TRUE(gate.update(5));
  EXPECT_FALSE(gate.update(4));
  EXPECT_EQ(gate.state(),
            DynamicPerceptionHealthGate::State::DEGRADED);

  EXPECT_FALSE(gate.update(0));
  EXPECT_FALSE(gate.update(1));
  EXPECT_TRUE(gate.update(0));
}

TEST(DynamicPerceptionHealthGate, HardInvalidationClosesOutputImmediately)
{
  DynamicPerceptionHealthGate::Parameters parameters;
  parameters.maximum_motion_tracks = 2;
  parameters.recovery_frames = 3;
  parameters.degradation_frames = 5;
  DynamicPerceptionHealthGate gate(parameters);

  gate.update(0);
  gate.update(0);
  ASSERT_TRUE(gate.update(0));

  gate.invalidate();
  EXPECT_FALSE(gate.healthy());
  EXPECT_EQ(gate.state(),
            DynamicPerceptionHealthGate::State::DEGRADED);
  EXPECT_EQ(gate.consecutiveGoodFrames(), 0);
  EXPECT_EQ(gate.consecutiveBadFrames(), 0);

  EXPECT_FALSE(gate.update(0));
  EXPECT_FALSE(gate.update(0));
  EXPECT_TRUE(gate.update(0));
}

}  // namespace
}  // namespace mm_remani

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

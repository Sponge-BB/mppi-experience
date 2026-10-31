#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include <Eigen/Core>

namespace mm_remani
{

struct DynamicDetection
{
  // This is the target-level motion anchor used only for association and
  // velocity estimation.  The node keeps the original cluster surface points
  // separately and publishes those points for collision geometry.
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  // Cluster geometry is part of association.  A static residual that splits
  // or merges between scans can move its centroid by several decimetres even
  // though no physical object moved.  Keeping these lightweight descriptors
  // prevents that topology change from becoming a velocity measurement.
  Eigen::Vector3d extent = Eigen::Vector3d::Zero();
  std::size_t point_count = 0;
};

// Parameters for extracting a stable motion anchor from a complete obstacle
// cluster.  Tall articulated targets (notably people) expose alternating leg
// surfaces to the lidar; using the arithmetic mean of all visible points makes
// an otherwise straight trajectory oscillate sideways.  A robust upper-core
// centre is used for motion only.  Collision geometry must continue to use the
// complete cluster points.
struct StableMotionCenterParameters
{
  double trim_fraction = 0.15;
  double tall_target_min_height = 0.65;
  double stable_core_lower_height_ratio = 0.35;
  double stable_core_upper_height_ratio = 0.90;
  int minimum_stable_core_points = 8;
};

Eigen::Vector3d estimateStableMotionCenter(
    const std::vector<Eigen::Vector3f> &complete_cluster_points,
    const StableMotionCenterParameters &parameters,
    bool *used_stable_core = nullptr);

struct TrackedDetection
{
  std::size_t detection_index = 0;
  std::uint32_t track_id = 0;
  Eigen::Vector3d filtered_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double confidence = 0.0;
  bool confirmed = false;
  // `confirmed` only means that data association has kept this track alive.
  // Only motion-qualified tracks may enter the predictive planning layer.
  bool motion_qualified = false;
};

class DynamicObstacleTrackerCore
{
public:
  struct Parameters
  {
    double association_gate = 0.75;
    double association_position_allowance = 0.12;
    double association_shape_weight = 0.20;
    double maximum_extent_change = 0.45;
    double maximum_point_count_ratio = 4.0;
    double position_alpha = 0.75;
    double velocity_alpha = 0.45;
    // Once coherent motion exists, regress velocity over the bounded history
    // instead of allowing scan-to-scan centroid motion to dominate.  Direction
    // smoothing damps periodic gait motion while retaining bounded turn
    // response for genuinely curving targets.
    double regression_velocity_weight = 0.70;
    double motion_direction_alpha = 0.30;
    double velocity_deadband = 0.08;
    double max_speed = 2.0;
    double dynamic_enter_speed = 0.18;
    double dynamic_exit_speed = 0.10;
    double minimum_dynamic_displacement = 0.06;
    double direction_consistency_cosine = 0.30;
    double minimum_motion_duration = 0.25;
    double motion_history_duration = 0.60;
    double minimum_motion_efficiency = 0.70;
    double minimum_direction_consistency_fraction = 0.75;
    double maximum_motion_step_speed_ratio = 1.25;
    double min_dt = 0.03;
    double max_dt = 0.35;
    int min_confirmed_hits = 2;
    int minimum_dynamic_hits = 3;
    int minimum_motion_cluster_points = 0;
    int minimum_stationary_hits = 3;
    // Residual returns that repeatedly occupy the same world-space location
    // are omissions/misalignment in the PCD, not moving obstacles.  Learn a
    // small persistent background memory after the track has proved stable.
    // A real object can still become dynamic after it leaves the anchor's
    // release radius; live occupancy continues to represent it while still.
    int minimum_stationary_background_hits = 5;
    double stationary_background_minimum_duration = 0.50;
    double stationary_background_maximum_speed = 0.10;
    double stationary_background_maximum_displacement = 0.06;
    double stationary_background_merge_radius = 0.20;
    double stationary_background_release_radius = 0.25;
    double stationary_background_dwell_duration = 2.0;
    double stationary_background_dwell_radius = 0.30;
    int maximum_stationary_background_anchors = 128;
    int max_missed_frames = 3;
  };

  DynamicObstacleTrackerCore();
  explicit DynamicObstacleTrackerCore(const Parameters &parameters);

  std::vector<TrackedDetection> update(
      const std::vector<DynamicDetection> &detections,
      double stamp_seconds);

  // Static-map replacement invalidates the learned residual background.
  // Transient bad frames only need to forget track identities, so callers may
  // retain the persistent anchors by passing false.
  void reset(bool clear_stationary_background = true);
  std::size_t trackCount() const;
  std::size_t stationaryBackgroundAnchorCount() const;
  double lastStamp() const;

private:
  struct MotionSample
  {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double stamp_seconds = 0.0;
  };

  struct Track
  {
    std::uint32_t id = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d motion_reference_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d extent = Eigen::Vector3d::Zero();
    Eigen::Vector2d background_dwell_position = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_motion_direction = Eigen::Vector2d::Zero();
    std::deque<MotionSample> motion_history;
    double point_count = 0.0;
    double stamp_seconds = 0.0;
    double background_dwell_start_seconds = -1.0;
    int hit_count = 0;
    int motion_hit_count = 0;
    int stationary_hit_count = 0;
    int stationary_background_hit_count = 0;
    int missed_frames = 0;
    bool has_motion_direction = false;
    bool motion_qualified = false;
  };

  Parameters parameters_;
  std::vector<Track> tracks_;
  std::vector<Eigen::Vector2d> stationary_background_anchors_;
  std::uint32_t next_track_id_ = 1;
  double last_stamp_seconds_ = -1.0;
};

// A healthy dynamic scene for this robot contains a small number of coherent
// moving objects.  Many simultaneous "moving" clusters are instead the
// signature observed when PCD registration residuals or cluster topology
// changes contaminate the whole view.  This gate starts closed and therefore
// cannot feed unvalidated startup data to active planning.
class DynamicPerceptionHealthGate
{
public:
  enum class State
  {
    WARMING_UP = 0,
    HEALTHY = 1,
    DEGRADED = 2,
  };

  struct Parameters
  {
    int maximum_motion_tracks = 3;
    int recovery_frames = 15;
    int degradation_frames = 3;
  };

  DynamicPerceptionHealthGate();
  explicit DynamicPerceptionHealthGate(const Parameters &parameters);

  bool update(int motion_track_count);
  bool healthy() const;
  State state() const;
  int consecutiveGoodFrames() const;
  int consecutiveBadFrames() const;
  // Hard input failures (for example, a whole-scene PCD registration
  // residual) are not ordinary object-count fluctuations.  Close the output
  // immediately and require the normal stable recovery window again.
  void invalidate();
  void reset();

private:
  Parameters parameters_;
  State state_ = State::WARMING_UP;
  int consecutive_good_frames_ = 0;
  int consecutive_bad_frames_ = 0;
};

}  // namespace mm_remani

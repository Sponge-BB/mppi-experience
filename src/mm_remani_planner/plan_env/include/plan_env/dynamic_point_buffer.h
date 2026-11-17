#ifndef PLAN_ENV_DYNAMIC_POINT_BUFFER_H
#define PLAN_ENV_DYNAMIC_POINT_BUFFER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

class DynamicPointBufferTestAccessor;
class GridMapParallelTestAccessor;

namespace plan_env
{

struct DynamicPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  std::uint32_t track_id = 0;
  double confidence = 1.0;
};

using DynamicPointVector =
    std::vector<DynamicPoint, Eigen::aligned_allocator<DynamicPoint>>;
using PredictedPositionVector =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

struct DynamicPointSnapshot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ros::Time stamp;
  ros::Time receive_time;
  std::string frame_id;
  std::uint64_t sequence = 0;
  DynamicPointVector points;
  bool has_track_ids = false;
  bool valid = false;
  // An empty, fresh prediction means no qualified motion was detected.
  // It never clears the independent live occupancy map.
  std::string invalid_reason;
};

/**
 * Thread-safe holder for point-wise dynamic obstacle observations.
 *
 * The input PointCloud2 contract is deliberately strict: x/y/z and vx/vy/vz
 * must be FLOAT32 fields. track_id (UINT32 or non-negative INT32) and
 * confidence (FLOAT32) are optional. IDs support motion association only;
 * their disappearance must not invalidate unrelated current predictions.
 * Real obstacle occupancy is owned and cleared by the LiDAR raycaster.
 * Accepted observations are transformed to world_frame at the observation
 * timestamp before being published as an immutable snapshot.
 */
class DynamicPointBuffer
{
public:
  using Ptr = std::shared_ptr<DynamicPointBuffer>;

  DynamicPointBuffer() = default;

  void init(ros::NodeHandle &nh,
            const std::string &world_frame,
            const std::shared_ptr<tf2_ros::Buffer> &tf_buffer);

  bool enabled() const { return enabled_; }
  DynamicPointSnapshot getSnapshot(const ros::Time &now) const;

  /**
   * Query the minimum distance from query_position to all obstacle points
   * predicted at query_time. Prediction uncertainty is subtracted from the
   * geometric distance, so the returned value is conservative.
   *
   * Returns false when the snapshot/time basis is invalid. An accepted empty
   * snapshot returns true and max_search_distance for this dynamic layer only;
   * it does not certify that the queried space was observed free.
   */
  bool getPredictedDistance(const DynamicPointSnapshot &snapshot,
                            const Eigen::Vector3d &query_position,
                            const ros::Time &query_time,
                            double max_search_distance,
                            double &distance) const;

  /**
   * Compute an axis-aligned bound of all points at query_time, expanded by
   * the same prediction uncertainty used by getPredictedDistance().
   * False means the time basis/snapshot is invalid; an accepted empty
   * snapshot returns true with has_points=false.
   */
  bool getPredictedBounds(const DynamicPointSnapshot &snapshot,
                          const ros::Time &query_time,
                          Eigen::Vector3d &minimum,
                          Eigen::Vector3d &maximum,
                          bool &has_points) const;

  /**
   * Predict every nominal point once for an exact whole-body query. The
   * returned uncertainty is common to all points because a snapshot has one
   * observation timestamp. False means the snapshot/time basis is invalid;
   * an accepted empty snapshot returns true with an empty vector.
   */
  bool getPredictedPositions(const DynamicPointSnapshot &snapshot,
                             const ros::Time &query_time,
                             PredictedPositionVector &positions,
                             double &uncertainty) const;

  static bool predictPosition(const DynamicPoint &point,
                              const ros::Time &observation_stamp,
                              const ros::Time &query_time,
                              double max_prediction_time,
                              Eigen::Vector3d &predicted_position,
                              double &prediction_dt);

  static double predictionUncertainty(double prediction_dt,
                                      double position_uncertainty,
                                      double velocity_uncertainty,
                                      double acceleration_bound);

  // A snapshot contains one real source epoch; never merge or re-stamp old
  // targets when accepting a newer frame (including a no-motion frame).
  static DynamicPointSnapshot applyObservation(
      const DynamicPointSnapshot &previous,
      const DynamicPointSnapshot &incoming);
  static DynamicPointSnapshot checkSnapshotAge(
      DynamicPointSnapshot snapshot, const ros::Time &now,
      double max_age, double future_stamp_tolerance);

  /** Return the configured uncertainty envelope for a prediction offset. */
  double configuredPredictionUncertainty(double prediction_dt) const
  {
    return predictionUncertainty(prediction_dt, position_uncertainty_,
                                 velocity_uncertainty_, acceleration_bound_);
  }

  double maxPredictionTime() const { return max_prediction_time_; }

private:
  friend class ::DynamicPointBufferTestAccessor;
  friend class ::GridMapParallelTestAccessor;
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  // Caller holds mutex_. Only the newest actual observation is retained.
  void recordObservation(const DynamicPointSnapshot &candidate);

  bool enabled_ = false;
  std::string topic_ = "/dynamic_points";
  std::string world_frame_ = "world";
  double max_age_ = 0.35;
  double future_stamp_tolerance_ = 0.05;
  double max_prediction_time_ = 2.0;
  double max_point_speed_ = 3.0;
  double min_confidence_ = 0.1;
  double tf_timeout_ = 0.02;
  double position_uncertainty_ = 0.03;
  double velocity_uncertainty_ = 0.15;
  double acceleration_bound_ = 0.5;
  int max_points_ = 2000;
  bool require_world_frame_ = true;

  ros::Subscriber point_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  mutable std::mutex mutex_;
  DynamicPointSnapshot latest_snapshot_;
  ros::Time latest_input_stamp_;
  bool received_observation_ = false;
};

}  // namespace plan_env

#endif  // PLAN_ENV_DYNAMIC_POINT_BUFFER_H

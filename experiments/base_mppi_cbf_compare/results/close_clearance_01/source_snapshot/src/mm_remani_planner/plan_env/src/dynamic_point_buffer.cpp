#include <plan_env/dynamic_point_buffer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointField.h>
#include <tf2/exceptions.h>

namespace
{

const sensor_msgs::PointField *findField(const sensor_msgs::PointCloud2 &msg,
                                         const std::string &name)
{
  for (const auto &field : msg.fields)
  {
    if (field.name == name)
      return &field;
  }
  return nullptr;
}

bool fieldFitsPoint(const sensor_msgs::PointCloud2 &msg,
                    const sensor_msgs::PointField *field,
                    std::size_t value_size)
{
  return field != nullptr && field->count == 1 &&
         field->offset <= msg.point_step &&
         value_size <= static_cast<std::size_t>(msg.point_step - field->offset);
}

bool validFloatField(const sensor_msgs::PointCloud2 &msg,
                     const sensor_msgs::PointField *field)
{
  return fieldFitsPoint(msg, field, sizeof(float)) &&
         field->datatype == sensor_msgs::PointField::FLOAT32;
}

bool readFloat32(const sensor_msgs::PointCloud2 &msg,
                 std::size_t point_offset,
                 const sensor_msgs::PointField &field,
                 float &value)
{
  const std::size_t offset = point_offset + field.offset;
  if (offset + sizeof(float) > msg.data.size())
    return false;
  std::memcpy(&value, msg.data.data() + offset, sizeof(float));
  return true;
}

bool readTrackId(const sensor_msgs::PointCloud2 &msg,
                 std::size_t point_offset,
                 const sensor_msgs::PointField *field,
                 std::uint32_t fallback,
                 std::uint32_t &value)
{
  value = fallback;
  if (field == nullptr)
    return true;

  const std::size_t offset = point_offset + field->offset;
  if (offset + sizeof(std::uint32_t) > msg.data.size())
    return false;

  if (field->datatype == sensor_msgs::PointField::UINT32)
  {
    std::memcpy(&value, msg.data.data() + offset, sizeof(std::uint32_t));
    return true;
  }
  if (field->datatype == sensor_msgs::PointField::INT32)
  {
    std::int32_t signed_value = 0;
    std::memcpy(&signed_value, msg.data.data() + offset, sizeof(std::int32_t));
    if (signed_value < 0)
      return false;
    value = static_cast<std::uint32_t>(signed_value);
    return true;
  }
  return false;
}

}  // namespace

namespace plan_env
{

void DynamicPointBuffer::init(
    ros::NodeHandle &nh,
    const std::string &world_frame,
    const std::shared_ptr<tf2_ros::Buffer> &tf_buffer)
{
  world_frame_ = world_frame.empty() ? std::string("world") : world_frame;
  tf_buffer_ = tf_buffer;

  nh.param("dynamic_point_flow/enable", enabled_, false);
  nh.param("dynamic_point_flow/topic", topic_, std::string("/dynamic_points"));
  nh.param("dynamic_point_flow/max_age", max_age_, 0.35);
  nh.param("dynamic_point_flow/future_stamp_tolerance", future_stamp_tolerance_, 0.05);
  nh.param("dynamic_point_flow/max_prediction_time", max_prediction_time_, 2.0);
  nh.param("dynamic_point_flow/max_point_speed", max_point_speed_, 3.0);
  nh.param("dynamic_point_flow/min_confidence", min_confidence_, 0.1);
  nh.param("dynamic_point_flow/tf_timeout", tf_timeout_, 0.02);
  nh.param("dynamic_point_flow/position_uncertainty", position_uncertainty_, 0.03);
  nh.param("dynamic_point_flow/velocity_uncertainty", velocity_uncertainty_, 0.15);
  nh.param("dynamic_point_flow/acceleration_bound", acceleration_bound_, 0.5);
  nh.param("dynamic_point_flow/max_points", max_points_, 2000);
  nh.param("dynamic_point_flow/empty_clear_frames", empty_clear_frames_, 3);
  nh.param("dynamic_point_flow/require_world_frame", require_world_frame_, true);

  max_age_ = std::max(0.01, max_age_);
  future_stamp_tolerance_ = std::max(0.0, future_stamp_tolerance_);
  max_prediction_time_ = std::max(0.0, max_prediction_time_);
  max_point_speed_ = std::max(0.01, max_point_speed_);
  min_confidence_ = std::max(0.0, std::min(1.0, min_confidence_));
  tf_timeout_ = std::max(0.0, tf_timeout_);
  position_uncertainty_ = std::max(0.0, position_uncertainty_);
  velocity_uncertainty_ = std::max(0.0, velocity_uncertainty_);
  acceleration_bound_ = std::max(0.0, acceleration_bound_);
  max_points_ = std::max(1, max_points_);
  empty_clear_frames_ = std::max(1, empty_clear_frames_);

  if (!enabled_)
  {
    ROS_INFO("[DynamicPointFlow] Disabled. Existing occupancy/ESDF behavior is unchanged.");
    return;
  }

  point_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
      topic_, 1, &DynamicPointBuffer::pointCloudCallback, this,
      ros::TransportHints().tcpNoDelay());
  ROS_INFO("[DynamicPointFlow] Listening on %s in frame %s.",
           topic_.c_str(), world_frame_.c_str());
}

DynamicPointSnapshot DynamicPointBuffer::getSnapshot(const ros::Time &now) const
{
  DynamicPointSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = latest_snapshot_;
    if (!enabled_)
    {
      snapshot.valid = false;
      snapshot.invalid_reason = "disabled";
      return snapshot;
    }
    if (!received_observation_)
    {
      snapshot.valid = false;
      snapshot.invalid_reason = "no_observation";
      return snapshot;
    }
  }

  const double age = (now - snapshot.stamp).toSec();
  if (age > max_age_)
  {
    snapshot.valid = false;
    snapshot.invalid_reason = "stale";
  }
  else if (age < -future_stamp_tolerance_)
  {
    snapshot.valid = false;
    snapshot.invalid_reason = "future_stamp";
  }
  return snapshot;
}

bool DynamicPointBuffer::predictPosition(
    const DynamicPoint &point,
    const ros::Time &observation_stamp,
    const ros::Time &query_time,
    double max_prediction_time,
    Eigen::Vector3d &predicted_position,
    double &prediction_dt)
{
  prediction_dt = (query_time - observation_stamp).toSec();
  constexpr double kSmallNegativeTimeTolerance = 1e-6;
  if (!std::isfinite(prediction_dt) || prediction_dt < -kSmallNegativeTimeTolerance ||
      prediction_dt > max_prediction_time + kSmallNegativeTimeTolerance)
    return false;

  prediction_dt = std::max(0.0, std::min(max_prediction_time, prediction_dt));
  predicted_position = point.position + prediction_dt * point.velocity;
  return predicted_position.allFinite();
}

double DynamicPointBuffer::predictionUncertainty(
    double prediction_dt,
    double position_uncertainty,
    double velocity_uncertainty,
    double acceleration_bound)
{
  const double dt = std::max(0.0, prediction_dt);
  return std::max(0.0, position_uncertainty) +
         std::max(0.0, velocity_uncertainty) * dt +
         0.5 * std::max(0.0, acceleration_bound) * dt * dt;
}

bool DynamicPointBuffer::getPredictedDistance(
    const DynamicPointSnapshot &snapshot,
    const Eigen::Vector3d &query_position,
    const ros::Time &query_time,
    double max_search_distance,
    double &distance) const
{
  distance = std::max(0.0, max_search_distance);
  if (!snapshot.valid || !query_position.allFinite())
    return false;

  // Validate the requested horizon even for an accepted empty observation.
  // Otherwise an empty frame could incorrectly authorize an arbitrarily long
  // future trajectory beyond the configured prediction time basis.
  DynamicPoint reference_point;
  Eigen::Vector3d reference_prediction;
  double reference_dt = 0.0;
  if (!predictPosition(reference_point, snapshot.stamp, query_time,
                       max_prediction_time_, reference_prediction,
                       reference_dt))
    return false;

  for (const auto &point : snapshot.points)
  {
    Eigen::Vector3d predicted_position;
    double prediction_dt = 0.0;
    if (!predictPosition(point, snapshot.stamp, query_time,
                         max_prediction_time_, predicted_position, prediction_dt))
      return false;

    const double uncertainty = predictionUncertainty(
        prediction_dt, position_uncertainty_, velocity_uncertainty_,
        acceleration_bound_);
    const double conservative_distance =
        (predicted_position - query_position).norm() - uncertainty;
    distance = std::min(distance, conservative_distance);
  }
  return true;
}

bool DynamicPointBuffer::getPredictedBounds(
    const DynamicPointSnapshot &snapshot,
    const ros::Time &query_time,
    Eigen::Vector3d &minimum,
    Eigen::Vector3d &maximum,
    bool &has_points) const
{
  minimum = Eigen::Vector3d::Zero();
  maximum = Eigen::Vector3d::Zero();
  has_points = false;
  if (!snapshot.valid)
    return false;

  DynamicPoint reference_point;
  Eigen::Vector3d reference_prediction;
  double prediction_dt = 0.0;
  if (!predictPosition(reference_point, snapshot.stamp, query_time,
                       max_prediction_time_, reference_prediction,
                       prediction_dt))
    return false;
  if (snapshot.points.empty())
    return true;

  minimum = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  maximum = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  for (const auto &point : snapshot.points)
  {
    Eigen::Vector3d predicted_position;
    double point_prediction_dt = 0.0;
    if (!predictPosition(point, snapshot.stamp, query_time,
                         max_prediction_time_, predicted_position,
                         point_prediction_dt))
      return false;
    minimum = minimum.cwiseMin(predicted_position);
    maximum = maximum.cwiseMax(predicted_position);
  }

  const double uncertainty = predictionUncertainty(
      prediction_dt, position_uncertainty_, velocity_uncertainty_,
      acceleration_bound_);
  minimum.array() -= uncertainty;
  maximum.array() += uncertainty;
  has_points = true;
  return true;
}

bool DynamicPointBuffer::getPredictedPositions(
    const DynamicPointSnapshot &snapshot,
    const ros::Time &query_time,
    PredictedPositionVector &positions,
    double &uncertainty) const
{
  positions.clear();
  uncertainty = 0.0;
  if (!snapshot.valid)
    return false;

  DynamicPoint reference_point;
  Eigen::Vector3d reference_prediction;
  double prediction_dt = 0.0;
  if (!predictPosition(reference_point, snapshot.stamp, query_time,
                       max_prediction_time_, reference_prediction,
                       prediction_dt))
    return false;

  uncertainty = predictionUncertainty(
      prediction_dt, position_uncertainty_, velocity_uncertainty_,
      acceleration_bound_);
  if (!std::isfinite(uncertainty))
    return false;

  positions.reserve(snapshot.points.size());
  for (const auto &point : snapshot.points)
  {
    Eigen::Vector3d predicted_position;
    double point_prediction_dt = 0.0;
    if (!predictPosition(point, snapshot.stamp, query_time,
                         max_prediction_time_, predicted_position,
                         point_prediction_dt))
    {
      positions.clear();
      return false;
    }
    positions.push_back(predicted_position);
  }
  return true;
}

void DynamicPointBuffer::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (!msg || msg->header.stamp.isZero())
  {
    ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject point cloud without a valid timestamp.");
    return;
  }
  if (msg->header.frame_id.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject point cloud without frame_id.");
    return;
  }
  if (msg->is_bigendian)
  {
    ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Big-endian PointCloud2 is not supported.");
    return;
  }
  const std::size_t minimum_row_step =
      static_cast<std::size_t>(msg->width) * msg->point_step;
  const std::size_t minimum_data_size =
      static_cast<std::size_t>(msg->height) * msg->row_step;
  if (msg->point_step == 0 || msg->row_step < minimum_row_step ||
      msg->data.size() < minimum_data_size)
  {
    ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject cloud: invalid PointCloud2 layout.");
    return;
  }

  const auto *field_x = findField(*msg, "x");
  const auto *field_y = findField(*msg, "y");
  const auto *field_z = findField(*msg, "z");
  const auto *field_vx = findField(*msg, "vx");
  const auto *field_vy = findField(*msg, "vy");
  const auto *field_vz = findField(*msg, "vz");
  const auto *field_track = findField(*msg, "track_id");
  const auto *field_confidence = findField(*msg, "confidence");

  if (!validFloatField(*msg, field_x) || !validFloatField(*msg, field_y) ||
      !validFloatField(*msg, field_z) || !validFloatField(*msg, field_vx) ||
      !validFloatField(*msg, field_vy) || !validFloatField(*msg, field_vz))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[DynamicPointFlow] Reject cloud: x/y/z/vx/vy/vz FLOAT32 fields are required.");
    return;
  }
  if (field_confidence != nullptr && !validFloatField(*msg, field_confidence))
  {
    ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject cloud: confidence must be FLOAT32.");
    return;
  }
  if (field_track != nullptr &&
      (!fieldFitsPoint(*msg, field_track, sizeof(std::uint32_t)) ||
       (field_track->datatype != sensor_msgs::PointField::UINT32 &&
        field_track->datatype != sensor_msgs::PointField::INT32)))
  {
    ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject cloud: track_id must be UINT32 or INT32.");
    return;
  }

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  if (msg->header.frame_id != world_frame_)
  {
    if (require_world_frame_)
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[DynamicPointFlow] Reject frame '%s': active prediction requires positions and absolute velocities in world frame '%s'. Transform tracker output upstream, including ego velocity.",
          msg->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }
    if (!tf_buffer_)
    {
      ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject cloud: TF buffer is unavailable.");
      return;
    }
    try
    {
      const geometry_msgs::TransformStamped transform = tf_buffer_->lookupTransform(
          world_frame_, msg->header.frame_id, msg->header.stamp,
          ros::Duration(tf_timeout_));
      const auto &q = transform.transform.rotation;
      Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
      if (quaternion.norm() < 1e-9)
      {
        ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject cloud: invalid TF quaternion.");
        return;
      }
      rotation = quaternion.normalized().toRotationMatrix();
      translation << transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z;
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject cloud: TF %s -> %s failed: %s",
                        msg->header.frame_id.c_str(), world_frame_.c_str(), ex.what());
      return;
    }
  }

  const std::size_t total_points =
      static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
  const std::size_t stride = total_points > static_cast<std::size_t>(max_points_)
                                 ? (total_points + static_cast<std::size_t>(max_points_) - 1) /
                                       static_cast<std::size_t>(max_points_)
                                 : 1;

  DynamicPointSnapshot candidate;
  candidate.stamp = msg->header.stamp;
  candidate.receive_time = ros::Time::now();
  candidate.frame_id = world_frame_;
  candidate.valid = true;
  candidate.points.reserve(std::min(total_points, static_cast<std::size_t>(max_points_)));

  std::size_t malformed_points = 0;
  std::size_t sampled_index = 0;
  for (std::size_t row = 0; row < msg->height; ++row)
  {
    for (std::size_t column = 0; column < msg->width; ++column, ++sampled_index)
    {
      if (sampled_index % stride != 0)
        continue;

      const std::size_t point_offset = row * msg->row_step + column * msg->point_step;
      float x = 0.0f, y = 0.0f, z = 0.0f;
      float vx = 0.0f, vy = 0.0f, vz = 0.0f;
      float confidence = 1.0f;
      std::uint32_t track_id = static_cast<std::uint32_t>(sampled_index);
      const bool read_ok =
          readFloat32(*msg, point_offset, *field_x, x) &&
          readFloat32(*msg, point_offset, *field_y, y) &&
          readFloat32(*msg, point_offset, *field_z, z) &&
          readFloat32(*msg, point_offset, *field_vx, vx) &&
          readFloat32(*msg, point_offset, *field_vy, vy) &&
          readFloat32(*msg, point_offset, *field_vz, vz) &&
          (field_confidence == nullptr ||
           readFloat32(*msg, point_offset, *field_confidence, confidence)) &&
          readTrackId(*msg, point_offset, field_track,
                      static_cast<std::uint32_t>(sampled_index), track_id);
      if (!read_ok)
      {
        ++malformed_points;
        continue;
      }

      Eigen::Vector3d position(x, y, z);
      Eigen::Vector3d velocity(vx, vy, vz);
      if (!position.allFinite() || !velocity.allFinite() ||
          !std::isfinite(static_cast<double>(confidence)) || confidence < min_confidence_ ||
          confidence > 1.0f ||
          velocity.norm() > max_point_speed_)
      {
        ++malformed_points;
        continue;
      }

      DynamicPoint point;
      point.position = rotation * position + translation;
      point.velocity = rotation * velocity;
      point.track_id = track_id;
      point.confidence = confidence;
      candidate.points.push_back(point);
    }
  }

  if (total_points > 0 && candidate.points.empty() && malformed_points > 0)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DynamicPointFlow] Reject cloud: all sampled points were malformed or unsafe.");
    return;
  }

  int accepted_empty_count = 0;
  bool held_last_valid_snapshot = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (received_observation_ && candidate.stamp < latest_snapshot_.stamp)
    {
      ROS_WARN_THROTTLE(1.0, "[DynamicPointFlow] Reject out-of-order observation.");
      return;
    }
    if (candidate.points.empty())
    {
      ++consecutive_empty_observations_;
      accepted_empty_count = consecutive_empty_observations_;
      if (!emptyObservationConfirmed(consecutive_empty_observations_,
                                     empty_clear_frames_))
      {
        // A single detector miss is not evidence that a moving obstacle has
        // vanished. Keep the last immutable prediction (and its original
        // timestamp) until either the empty-frame hysteresis confirms CLEAR
        // or getSnapshot() invalidates it by max_age. Preserving the original
        // stamp is important: holding a dropout must never make stale data
        // look fresh.
        held_last_valid_snapshot = shouldRetainLastValidSnapshot(
            consecutive_empty_observations_, empty_clear_frames_,
            received_observation_ && latest_snapshot_.valid &&
                !latest_snapshot_.points.empty());
        if (!held_last_valid_snapshot)
        {
          candidate.valid = false;
          candidate.invalid_reason = "empty_dropout_unconfirmed_without_history";
        }
      }
    }
    else
    {
      consecutive_empty_observations_ = 0;
    }
    if (!held_last_valid_snapshot)
    {
      candidate.sequence = latest_snapshot_.sequence + 1;
      latest_snapshot_ = candidate;
    }
    received_observation_ = true;
  }

  if (held_last_valid_snapshot)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[DynamicPointFlow] Empty observation is an unconfirmed detector dropout (%d/%d); retain the last valid prediction with its original timestamp.",
        accepted_empty_count, empty_clear_frames_);
  }
  else if (candidate.points.empty() && !candidate.valid)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[DynamicPointFlow] Empty observation is treated as an unconfirmed detector dropout (%d/%d), not as immediate CLEAR.",
        accepted_empty_count, empty_clear_frames_);
  }

  if (malformed_points > 0)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DynamicPointFlow] Accepted %zu points and rejected %zu malformed points.",
                      candidate.points.size(), malformed_points);
  }
}

}  // namespace plan_env

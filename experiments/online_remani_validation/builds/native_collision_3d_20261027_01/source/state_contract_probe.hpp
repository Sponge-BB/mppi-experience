#pragma once
// Experiment-only measured-state contract. No model calls, spinner, per-odom
// logging, or controller behavior. Call receive() at callback entry, before any
// velocity floor; call processed() with its returned token after regularization.
#include "dual_timescale_probe.hpp"
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <ros/serialization.h>
#include <cstdint>
#include <limits>
#include <vector>

namespace state_contract {

inline double unavailable() { return std::numeric_limits<double>::quiet_NaN(); }

inline std::uint64_t fnv1a64(const std::uint8_t* bytes, std::size_t size) {
  std::uint64_t digest = UINT64_C(14695981039346656037);
  for (std::size_t index = 0; index < size; ++index) {
    digest ^= bytes[index];
    digest *= UINT64_C(1099511628211); // Unsigned arithmetic is modulo 2^64.
  }
  return digest;
}

inline std::string messageFnv64(const nav_msgs::Odometry& msg) {
  const std::uint32_t size = ros::serialization::serializationLength(msg);
  std::vector<std::uint8_t> bytes(size);
  ros::serialization::OStream stream(bytes.data(), size);
  ros::serialization::serialize(stream, msg);
  // A decimal JSON string avoids lossy uint64 parsing by JavaScript consumers.
  return std::to_string(fnv1a64(bytes.data(), bytes.size()));
}

struct Regularization {
  double vx = unavailable(), vy = unavailable();
  double floor_bias = unavailable(); // Signed-only upward speed adjustment.
  int gear = 0;
  bool valid = false;
  bool regularized = false;
};

// vx/vy are WORLD velocities, not body velocities. For finite inputs, legacy
// mode exactly follows the existing Eigen norm/dot branch and forward floor.
// A regularized velocity is an optimizer input, NOT measured motion, a
// prediction, or a certificate that a stopped robot is moving safely.
inline Regularization signedRegularization(double vx, double vy, double yaw,
                                           double min_speed, bool signed_mode,
                                           int previous_direction = 0) {
  Regularization out;
  // Do not turn invalid yaw/threshold/velocity into finite, safe-looking data.
  // This explicit invalid result is the sole exception to legacy arithmetic.
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(yaw) ||
      !std::isfinite(min_speed)) return out;
  const Eigen::Vector2d world(vx, vy);
  const Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
  out.valid = true;
  out.floor_bias = 0.0;
  out.vx = vx;
  out.vy = vy;
  if (world.norm() < min_speed) {
    int direction = 0;
    if (signed_mode) {
      const double longitudinal = heading.dot(world);
      const double epsilon = 1e-4;
      if (std::abs(longitudinal) > epsilon)
        direction = longitudinal > 0.0 ? 1 : -1;
      else if (previous_direction == 1 || previous_direction == -1)
        direction = previous_direction;
    }
    // Explicit unknown-direction policy: preserve legacy gear0/forward floor.
    // Callers can pass 0 to disable all history/hysteresis (the one-factor trial).
    double speed = min_speed;
    out.vx = (direction == 0 ? 1 : direction) * speed * std::cos(yaw);
    out.vy = (direction == 0 ? 1 : direction) * speed * std::sin(yaw);
    if (signed_mode && direction != 0) {
      // The native downstream singularity test is STRICTLY > min_speed. An
      // exact nominal floor could therefore erase the direction we retained.
      // Increase only this signed, low-speed regularizer by the least verified
      // representable speed increment (at most 16 ULP steps). This is numerical
      // optimizer conditioning, never a change to the raw measured velocity.
      double norm = Eigen::Vector2d(out.vx, out.vy).norm();
      for (unsigned step = 0; step < 16 && std::isfinite(norm) &&
           !(norm > min_speed); ++step) {
        speed = std::nextafter(speed, std::numeric_limits<double>::infinity());
        if (!std::isfinite(speed)) return Regularization();
        out.vx = direction * speed * std::cos(yaw);
        out.vy = direction * speed * std::sin(yaw);
        norm = Eigen::Vector2d(out.vx, out.vy).norm();
      }
      if (!std::isfinite(norm) || !(norm > min_speed)) return Regularization();
      out.floor_bias = speed - min_speed;
    }
    out.gear = direction;
    out.regularized = true;
  } else {
    // Never change speeds at or above the threshold, including pure lateral
    // motion. A zero longitudinal dot has the original legacy forward gear.
    out.gear = heading.dot(world) >= 0.0 ? 1 : -1;
  }
  return out;
}

struct Snapshot {
  bool received = false;
  bool raw_finite = false;
  bool processed_ready = false;
  bool processed_finite = false;
  bool sequence_exhausted = false;
  std::uint64_t seq = 0;                  // Internal callback-association token.
  std::uint32_t header_seq = 0;           // Source header.seq for external joins.
  ros::Time header_stamp;
  std::string frame_id, child_frame_id;
  std::string message_fnv64;             // Exact original ROS1 serialized bytes.
  ros::Time receipt_ros;
  double receipt_mono = unavailable();
  double x = unavailable(), y = unavailable(), yaw = unavailable();
  double body_vx = unavailable(), body_vy = unavailable();
  double world_vx = unavailable(), world_vy = unavailable();
  std::uint64_t processed_seq = 0;
  double processed_vx = unavailable(), processed_vy = unavailable();
  double processed_floor_bias = unavailable();
  int processed_gear = 0;
  // Fixed-width saturating counters; no per-frame collection is retained.
  std::uint64_t received_count = 0, processed_count = 0;
  std::uint64_t rejected_processed_count = 0, boundary_count = 0;
};

inline void incrementBounded(std::uint64_t& value) {
  if (value != std::numeric_limits<std::uint64_t>::max()) ++value;
}

// Independently instantiable for graph-free tests. Only the newest raw frame is
// retained; delayed processing of an older frame is rejected, never mislabeled.
class StateRecorder {
 public:
  std::uint64_t receive(const nav_msgs::Odometry& msg, double yaw) {
    Snapshot incoming;
    // Capture clocks before field decoding and before waiting for the mutex.
    incoming.receipt_ros = ros::Time::now();
    incoming.receipt_mono = dual_probe::mono();
    incoming.received = true;
    incoming.header_seq = msg.header.seq;
    incoming.header_stamp = msg.header.stamp;
    incoming.frame_id = msg.header.frame_id;
    incoming.child_frame_id = msg.child_frame_id;
    incoming.message_fnv64 = messageFnv64(msg);
    incoming.x = msg.pose.pose.position.x;
    incoming.y = msg.pose.pose.position.y;
    incoming.yaw = yaw;
    incoming.body_vx = msg.twist.twist.linear.x;
    incoming.body_vy = msg.twist.twist.linear.y;
    incoming.world_vx = std::cos(yaw) * incoming.body_vx -
                        std::sin(yaw) * incoming.body_vy;
    incoming.world_vy = std::sin(yaw) * incoming.body_vx +
                        std::cos(yaw) * incoming.body_vy;
    incoming.raw_finite = std::isfinite(incoming.x) && std::isfinite(incoming.y) &&
      std::isfinite(yaw) && std::isfinite(incoming.body_vx) &&
      std::isfinite(incoming.body_vy) && std::isfinite(incoming.world_vx) &&
      std::isfinite(incoming.world_vy) && std::isfinite(incoming.receipt_mono);

    std::lock_guard<std::mutex> guard(mutex_);
    incrementBounded(latest_.received_count);
    // Never wrap tokens and accidentally associate an old callback with new
    // state. The 0 failure token is permanently rejected by processed().
    if (latest_.seq == std::numeric_limits<std::uint64_t>::max()) {
      latest_.sequence_exhausted = true;
      return 0;
    }
    incoming.seq = latest_.seq + 1;
    incoming.received_count = latest_.received_count;
    incoming.processed_count = latest_.processed_count;
    incoming.rejected_processed_count = latest_.rejected_processed_count;
    incoming.boundary_count = latest_.boundary_count;
    latest_ = incoming;  // Clears all processed fields for this new raw frame.
    return latest_.seq;
  }

  bool processed(std::uint64_t seq, double vx, double vy, int gear,
                 double floor_bias = 0.0) {
    std::lock_guard<std::mutex> guard(mutex_);
    if (seq == 0 || !latest_.received || seq != latest_.seq ||
        latest_.processed_ready || latest_.sequence_exhausted) {
      incrementBounded(latest_.rejected_processed_count);
      return false;
    }
    latest_.processed_seq = seq;
    latest_.processed_vx = vx;
    latest_.processed_vy = vy;
    latest_.processed_gear = gear;
    latest_.processed_floor_bias = floor_bias;
    latest_.processed_finite = std::isfinite(vx) && std::isfinite(vy) &&
      std::isfinite(floor_bias) && floor_bias >= 0.0 &&
      (gear == -1 || gear == 0 || gear == 1);
    latest_.processed_ready = true;
    incrementBounded(latest_.processed_count);
    return true;
  }

  Snapshot snapshot() const {
    std::lock_guard<std::mutex> guard(mutex_);
    return latest_;  // Value copy: later callbacks cannot modify this snapshot.
  }

  Snapshot boundarySnapshot() {
    std::lock_guard<std::mutex> guard(mutex_);
    incrementBounded(latest_.boundary_count);
    return latest_;
  }

 private:
  mutable std::mutex mutex_;
  Snapshot latest_;
};

inline StateRecorder& recorder() { static StateRecorder value; return value; }
inline std::uint64_t receive(const nav_msgs::Odometry& msg, double yaw) {
  return recorder().receive(msg, yaw);
}
inline bool processed(std::uint64_t seq, double vx, double vy, int gear,
                      double floor_bias = 0.0) {
  return recorder().processed(seq, vx, vy, gear, floor_bias);
}
inline Snapshot snapshot() { return recorder().snapshot(); }

inline void number(dual_probe::Fields& fields, const std::string& name, double value) {
  if (std::isfinite(value)) fields.add(name, value);
  else fields.add(name, "null"); // Valid JSON, with validity separately recorded.
}

template<class Vector>
inline std::string vectorJson(const Vector& vector) {
  std::ostringstream out;
  out << std::setprecision(17) << '[';
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    if (i != 0) out << ',';
    if (std::isfinite(vector(i))) out << vector(i);
    else out << "null";
  }
  out << ']';
  return out.str();
}

// Also exposed without I/O so a unit test can inspect missing/invalid metadata.
template<class Position, class Velocity>
inline dual_probe::Fields boundaryFields(const Snapshot& raw,
    const Position& planner_p, const Velocity& planner_v, double planner_yaw,
    int planner_gear, double desired_start_time) {
  dual_probe::Fields fields;
  fields.add("state_contract_schema", 1)
    .str("state_contract_semantics", "measured_raw_and_optimizer_regularized_not_prediction")
    .add("raw_odom_missing", raw.received ? "false" : "true")
    .add("raw_odom_finite", raw.raw_finite ? "true" : "false")
    .add("raw_odom_seq", raw.seq)
    .add("raw_odom_header_seq", raw.header_seq)
    .add("raw_odom_stamp_sec", raw.header_stamp.sec)
    .add("raw_odom_stamp_nsec", raw.header_stamp.nsec)
    .str("raw_odom_frame_id", raw.frame_id)
    .str("raw_odom_child_frame_id", raw.child_frame_id)
    .str("raw_odom_message_fnv64", raw.message_fnv64)
    .str("raw_odom_message_hash_algorithm", "fnv1a64_ros1_serialized_standard")
    .add("raw_odom_receipt_ros_sec", raw.receipt_ros.sec)
    .add("raw_odom_receipt_ros_nsec", raw.receipt_ros.nsec)
    .add("raw_odom_processed_missing", raw.processed_ready ? "false" : "true")
    .add("raw_odom_processed_finite", raw.processed_finite ? "true" : "false")
    .add("raw_odom_processed_seq", raw.processed_seq)
    .add("raw_odom_processed_gear", raw.processed_gear)
    .add("raw_odom_received_count", raw.received_count)
    .add("raw_odom_processed_count", raw.processed_count)
    .add("raw_odom_rejected_processed_count", raw.rejected_processed_count)
    .add("raw_odom_sequence_exhausted", raw.sequence_exhausted ? "true" : "false")
    .add("state_contract_boundary_count", raw.boundary_count)
    .add("planner_p", vectorJson(planner_p))
    .add("planner_v", vectorJson(planner_v))
    .add("planner_gear", planner_gear);
  number(fields, "raw_odom_stamp", raw.received ? raw.header_stamp.toSec() : unavailable());
  number(fields, "raw_odom_receipt_ros", raw.received ? raw.receipt_ros.toSec() : unavailable());
  number(fields, "raw_odom_receipt_mono", raw.receipt_mono);
  number(fields, "raw_odom_x", raw.x);
  number(fields, "raw_odom_y", raw.y);
  number(fields, "raw_odom_yaw", raw.yaw);
  number(fields, "raw_odom_body_vx", raw.body_vx);
  number(fields, "raw_odom_body_vy", raw.body_vy);
  number(fields, "raw_odom_world_vx", raw.world_vx);
  number(fields, "raw_odom_world_vy", raw.world_vy);
  number(fields, "raw_odom_processed_vx", raw.processed_vx);
  number(fields, "raw_odom_processed_vy", raw.processed_vy);
  number(fields, "raw_odom_processed_floor_bias", raw.processed_floor_bias);
  number(fields, "planner_yaw", planner_yaw);
  number(fields, "planner_desired_start_time", desired_start_time);
  return fields;
}

// Only this explicit per-plan boundary call emits. The existing dual writer
// supplies mono/sim/pid/request_id, and remains opt-in via DUAL_TIMELINE_PATH.
template<class Position, class Velocity>
inline Snapshot emitBoundary(const std::string& event, const Position& planner_p,
    const Velocity& planner_v, double planner_yaw, int planner_gear,
    double desired_start_time) {
  const Snapshot raw = recorder().boundarySnapshot();
  dual_probe::emit(event, boundaryFields(raw, planner_p, planner_v, planner_yaw,
                                      planner_gear, desired_start_time));
  return raw; // received/processed_ready explicitly report any missing state.
}

} // namespace state_contract

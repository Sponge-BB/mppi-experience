#include <mm_remani/dynamic_obstacle_tracker_core.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace mm_remani
{

namespace
{

double trimmedCoordinateMean(
    const std::vector<Eigen::Vector3f> &points,
    int axis, double trim_fraction)
{
  std::vector<double> coordinates;
  coordinates.reserve(points.size());
  for (const Eigen::Vector3f &point : points)
  {
    if (point.allFinite())
      coordinates.push_back(static_cast<double>(point(axis)));
  }
  if (coordinates.empty())
    return 0.0;

  std::sort(coordinates.begin(), coordinates.end());
  const double bounded_trim = std::max(0.0, std::min(0.45, trim_fraction));
  std::size_t trim_count = static_cast<std::size_t>(
      std::floor(bounded_trim * static_cast<double>(coordinates.size())));
  if (2 * trim_count >= coordinates.size())
    trim_count = (coordinates.size() - 1) / 2;

  double sum = 0.0;
  for (std::size_t index = trim_count;
       index < coordinates.size() - trim_count; ++index)
    sum += coordinates[index];
  return sum / static_cast<double>(coordinates.size() - 2 * trim_count);
}

}  // namespace

Eigen::Vector3d estimateStableMotionCenter(
    const std::vector<Eigen::Vector3f> &complete_cluster_points,
    const StableMotionCenterParameters &parameters,
    bool *used_stable_core)
{
  if (used_stable_core != nullptr)
    *used_stable_core = false;

  std::vector<Eigen::Vector3f> finite_points;
  finite_points.reserve(complete_cluster_points.size());
  double minimum_z = std::numeric_limits<double>::infinity();
  double maximum_z = -std::numeric_limits<double>::infinity();
  for (const Eigen::Vector3f &point : complete_cluster_points)
  {
    if (!point.allFinite())
      continue;
    finite_points.push_back(point);
    minimum_z = std::min(minimum_z, static_cast<double>(point.z()));
    maximum_z = std::max(maximum_z, static_cast<double>(point.z()));
  }
  if (finite_points.empty())
    return Eigen::Vector3d::Zero();

  const double target_height = maximum_z - minimum_z;
  const double minimum_tall_height =
      std::max(0.0, parameters.tall_target_min_height);
  const double lower_ratio = std::max(
      0.0, std::min(1.0, parameters.stable_core_lower_height_ratio));
  const double upper_ratio = std::max(
      lower_ratio, std::min(1.0,
                            parameters.stable_core_upper_height_ratio));
  const std::size_t minimum_core_points = static_cast<std::size_t>(
      std::max(1, parameters.minimum_stable_core_points));

  std::vector<Eigen::Vector3f> stable_core;
  if (target_height >= minimum_tall_height && upper_ratio > lower_ratio)
  {
    const double lower_z = minimum_z + lower_ratio * target_height;
    const double upper_z = minimum_z + upper_ratio * target_height;
    stable_core.reserve(finite_points.size());
    for (const Eigen::Vector3f &point : finite_points)
    {
      if (point.z() >= lower_z && point.z() <= upper_z)
        stable_core.push_back(point);
    }
  }

  const bool use_stable_core = stable_core.size() >= minimum_core_points;
  const std::vector<Eigen::Vector3f> &motion_points =
      use_stable_core ? stable_core : finite_points;
  if (used_stable_core != nullptr)
    *used_stable_core = use_stable_core;

  Eigen::Vector3d center;
  for (int axis = 0; axis < 3; ++axis)
  {
    center(axis) = trimmedCoordinateMean(
        motion_points, axis, parameters.trim_fraction);
  }
  return center;
}

DynamicObstacleTrackerCore::DynamicObstacleTrackerCore()
  : DynamicObstacleTrackerCore(Parameters())
{
}

DynamicObstacleTrackerCore::DynamicObstacleTrackerCore(
    const Parameters &parameters)
  : parameters_(parameters)
{
  parameters_.association_gate =
      std::max(0.05, parameters_.association_gate);
  parameters_.association_position_allowance = std::max(
      0.01, std::min(parameters_.association_gate,
                     parameters_.association_position_allowance));
  parameters_.association_shape_weight =
      std::max(0.0, parameters_.association_shape_weight);
  parameters_.maximum_extent_change =
      std::max(0.0, parameters_.maximum_extent_change);
  parameters_.maximum_point_count_ratio =
      std::max(1.0, parameters_.maximum_point_count_ratio);
  parameters_.position_alpha =
      std::max(0.0, std::min(1.0, parameters_.position_alpha));
  parameters_.velocity_alpha =
      std::max(0.0, std::min(1.0, parameters_.velocity_alpha));
  parameters_.regression_velocity_weight = std::max(
      0.0, std::min(1.0, parameters_.regression_velocity_weight));
  parameters_.motion_direction_alpha = std::max(
      0.0, std::min(1.0, parameters_.motion_direction_alpha));
  parameters_.velocity_deadband =
      std::max(0.0, parameters_.velocity_deadband);
  parameters_.max_speed = std::max(0.05, parameters_.max_speed);
  parameters_.dynamic_enter_speed =
      std::max(0.0, parameters_.dynamic_enter_speed);
  parameters_.dynamic_exit_speed = std::max(
      0.0, std::min(parameters_.dynamic_enter_speed,
                    parameters_.dynamic_exit_speed));
  parameters_.minimum_dynamic_displacement =
      std::max(0.0, parameters_.minimum_dynamic_displacement);
  parameters_.direction_consistency_cosine = std::max(
      -1.0, std::min(1.0, parameters_.direction_consistency_cosine));
  parameters_.minimum_motion_duration =
      std::max(0.0, parameters_.minimum_motion_duration);
  parameters_.motion_history_duration = std::max(
      parameters_.minimum_motion_duration,
      parameters_.motion_history_duration);
  parameters_.minimum_motion_efficiency = std::max(
      0.0, std::min(1.0, parameters_.minimum_motion_efficiency));
  parameters_.minimum_direction_consistency_fraction = std::max(
      0.0, std::min(
               1.0,
               parameters_.minimum_direction_consistency_fraction));
  parameters_.maximum_motion_step_speed_ratio =
      std::max(1.0, parameters_.maximum_motion_step_speed_ratio);
  parameters_.min_dt = std::max(1e-3, parameters_.min_dt);
  parameters_.max_dt =
      std::max(parameters_.min_dt, parameters_.max_dt);
  parameters_.min_confirmed_hits =
      std::max(1, parameters_.min_confirmed_hits);
  parameters_.minimum_dynamic_hits =
      std::max(1, parameters_.minimum_dynamic_hits);
  parameters_.minimum_motion_cluster_points =
      std::max(0, parameters_.minimum_motion_cluster_points);
  parameters_.minimum_stationary_hits =
      std::max(1, parameters_.minimum_stationary_hits);
  parameters_.minimum_stationary_background_hits =
      std::max(1, parameters_.minimum_stationary_background_hits);
  parameters_.stationary_background_minimum_duration =
      std::max(0.0, parameters_.stationary_background_minimum_duration);
  parameters_.stationary_background_maximum_speed =
      std::max(0.0, parameters_.stationary_background_maximum_speed);
  parameters_.stationary_background_maximum_displacement =
      std::max(0.0,
               parameters_.stationary_background_maximum_displacement);
  parameters_.stationary_background_merge_radius =
      std::max(0.01, parameters_.stationary_background_merge_radius);
  parameters_.stationary_background_release_radius = std::max(
      parameters_.stationary_background_merge_radius,
      parameters_.stationary_background_release_radius);
  parameters_.stationary_background_dwell_duration =
      std::max(0.0, parameters_.stationary_background_dwell_duration);
  parameters_.stationary_background_dwell_radius =
      std::max(parameters_.stationary_background_merge_radius,
               parameters_.stationary_background_dwell_radius);
  parameters_.maximum_stationary_background_anchors =
      std::max(0, parameters_.maximum_stationary_background_anchors);
  parameters_.max_missed_frames =
      std::max(0, parameters_.max_missed_frames);
}

std::vector<TrackedDetection> DynamicObstacleTrackerCore::update(
    const std::vector<DynamicDetection> &detections,
    double stamp_seconds)
{
  std::vector<TrackedDetection> result;
  if (!std::isfinite(stamp_seconds) ||
      (last_stamp_seconds_ >= 0.0 &&
       stamp_seconds <= last_stamp_seconds_))
    return result;

  last_stamp_seconds_ = stamp_seconds;

  struct Candidate
  {
    double cost;
    std::size_t track_index;
    std::size_t detection_index;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(tracks_.size() * detections.size());

  for (std::size_t track_index = 0;
       track_index < tracks_.size(); ++track_index)
  {
    const Track &track = tracks_[track_index];
    const double raw_dt = stamp_seconds - track.stamp_seconds;
    const double prediction_dt = std::max(
        0.0, std::min(parameters_.max_dt, raw_dt));
    const Eigen::Vector3d predicted =
        track.position + prediction_dt * track.velocity;
    const double adaptive_gate = std::min(
        parameters_.association_gate,
        parameters_.association_position_allowance +
            parameters_.max_speed * prediction_dt);
    for (std::size_t detection_index = 0;
         detection_index < detections.size(); ++detection_index)
    {
      if (!detections[detection_index].centroid.allFinite())
        continue;
      const double distance =
          (detections[detection_index].centroid - predicted).norm();
      if (distance > adaptive_gate)
        continue;

      const DynamicDetection &detection = detections[detection_index];
      double shape_change = 0.0;
      if (track.extent.norm() > 1e-6 && detection.extent.norm() > 1e-6)
      {
        shape_change =
            (detection.extent - track.extent).head<2>().norm();
        if (parameters_.maximum_extent_change > 0.0 &&
            shape_change > parameters_.maximum_extent_change)
          continue;
      }
      if (track.point_count > 0.0 && detection.point_count > 0)
      {
        const double current_count =
            static_cast<double>(detection.point_count);
        const double count_ratio = std::max(
            current_count / track.point_count,
            track.point_count / current_count);
        if (count_ratio > parameters_.maximum_point_count_ratio)
          continue;
      }

      candidates.push_back(Candidate{
          distance + parameters_.association_shape_weight * shape_change,
          track_index, detection_index});
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate &lhs, const Candidate &rhs)
            {
              return lhs.cost < rhs.cost;
            });

  std::vector<int> detection_to_track(detections.size(), -1);
  std::vector<unsigned char> track_assigned(tracks_.size(), 0);
  for (const Candidate &candidate : candidates)
  {
    if (track_assigned[candidate.track_index] ||
        detection_to_track[candidate.detection_index] >= 0)
      continue;
    track_assigned[candidate.track_index] = 1;
    detection_to_track[candidate.detection_index] =
        static_cast<int>(candidate.track_index);
  }

  for (std::size_t detection_index = 0;
       detection_index < detections.size(); ++detection_index)
  {
    const int assigned_track = detection_to_track[detection_index];
    if (assigned_track < 0)
      continue;

    Track &track = tracks_[static_cast<std::size_t>(assigned_track)];
    const int previous_missed_frames = track.missed_frames;
    const Eigen::Vector3d previous_position = track.position;
    const double raw_dt = stamp_seconds - track.stamp_seconds;
    const double update_dt = std::max(
        parameters_.min_dt, std::min(parameters_.max_dt, raw_dt));
    const Eigen::Vector3d predicted =
        previous_position + update_dt * track.velocity;
    const Eigen::Vector3d residual =
        detections[detection_index].centroid - predicted;
    const Eigen::Vector3d measured_velocity =
        (detections[detection_index].centroid - previous_position) /
        update_dt;
    // Keep the newest target-level measurement separate from the stabilized
    // prediction velocity below.  History regression is desirable for a
    // smooth future path, but it must not make a stopped target look as if it
    // is still moving for an entire history window.
    const double measured_planar_speed =
        measured_velocity.head<2>().norm();

    track.position =
        predicted + parameters_.position_alpha * residual;
    track.velocity =
        (1.0 - parameters_.velocity_alpha) * track.velocity +
        parameters_.velocity_alpha * measured_velocity;
    const double speed = track.velocity.norm();
    if (speed > parameters_.max_speed)
      track.velocity *= parameters_.max_speed / speed;
    else if (speed < parameters_.velocity_deadband)
      track.velocity.setZero();
    track.stamp_seconds = stamp_seconds;
    ++track.hit_count;
    track.missed_frames = 0;
    const DynamicDetection &detection = detections[detection_index];
    if (detection.extent.allFinite() && detection.extent.norm() > 1e-6)
    {
      if (track.extent.norm() <= 1e-6)
        track.extent = detection.extent;
      else
        track.extent = 0.7 * track.extent + 0.3 * detection.extent;
    }
    if (detection.point_count > 0)
    {
      if (track.point_count <= 0.0)
        track.point_count = static_cast<double>(detection.point_count);
      else
        track.point_count =
            0.7 * track.point_count +
            0.3 * static_cast<double>(detection.point_count);
    }

    // Use a bounded history of raw observations for motion evidence.  A
    // cluster centroid may jitter quickly, so instantaneous filtered speed is
    // insufficient: real motion must make efficient net progress, keep a
    // consistent direction, and persist for a minimum wall-clock duration.
    if (previous_missed_frames > 0)
      track.motion_history.clear();
    track.motion_history.push_back(
        MotionSample{detection.centroid, stamp_seconds});
    const double history_cutoff =
        stamp_seconds - parameters_.motion_history_duration;
    while (track.motion_history.size() > 2 &&
           track.motion_history[1].stamp_seconds <= history_cutoff)
      track.motion_history.pop_front();

    double history_duration = 0.0;
    double history_path_length = 0.0;
    double maximum_step_speed = 0.0;
    double latest_observed_planar_speed = measured_planar_speed;
    int valid_step_count = 0;
    int consistent_step_count = 0;
    Eigen::Vector2d history_net = Eigen::Vector2d::Zero();
    Eigen::Vector2d history_direction = Eigen::Vector2d::Zero();
    if (track.motion_history.size() >= 2)
    {
      const MotionSample &first = track.motion_history.front();
      const MotionSample &last = track.motion_history.back();
      history_duration = last.stamp_seconds - first.stamp_seconds;
      history_net =
          last.position.head<2>() - first.position.head<2>();
      if (history_net.norm() > 1e-6)
        history_direction = history_net.normalized();
      for (std::size_t index = 1;
           index < track.motion_history.size(); ++index)
      {
        const MotionSample &prior = track.motion_history[index - 1];
        const MotionSample &current = track.motion_history[index];
        const double dt = current.stamp_seconds - prior.stamp_seconds;
        if (dt <= 1e-6)
          continue;
        const Eigen::Vector2d step =
            current.position.head<2>() - prior.position.head<2>();
        const double step_length = step.norm();
        if (index + 1 == track.motion_history.size())
          latest_observed_planar_speed = step_length / dt;
        history_path_length += step_length;
        maximum_step_speed = std::max(
            maximum_step_speed, step_length / dt);
        if (step_length <= 1e-4 || history_direction.norm() <= 1e-6)
          continue;
        ++valid_step_count;
        if ((step / step_length).dot(history_direction) >=
            parameters_.direction_consistency_cosine)
          ++consistent_step_count;
      }
    }

    // Estimate the target velocity from the complete bounded history instead
    // of only the newest centroid displacement.  Least-squares regression
    // averages periodic visible-surface motion (for example alternating human
    // legs) without erasing the target's net translation.
    Eigen::Vector2d regression_velocity = Eigen::Vector2d::Zero();
    bool regression_velocity_valid = false;
    if (track.motion_history.size() >= 3 && history_duration > 1e-6)
    {
      const double time_origin =
          track.motion_history.front().stamp_seconds;
      double mean_time = 0.0;
      Eigen::Vector2d mean_position = Eigen::Vector2d::Zero();
      for (const MotionSample &sample : track.motion_history)
      {
        mean_time += sample.stamp_seconds - time_origin;
        mean_position += sample.position.head<2>();
      }
      const double sample_count =
          static_cast<double>(track.motion_history.size());
      mean_time /= sample_count;
      mean_position /= sample_count;

      double time_variance = 0.0;
      Eigen::Vector2d time_position_covariance =
          Eigen::Vector2d::Zero();
      for (const MotionSample &sample : track.motion_history)
      {
        const double centered_time =
            sample.stamp_seconds - time_origin - mean_time;
        time_variance += centered_time * centered_time;
        time_position_covariance +=
            centered_time *
            (sample.position.head<2>() - mean_position);
      }
      if (time_variance > 1e-9)
      {
        regression_velocity = time_position_covariance / time_variance;
        regression_velocity_valid = regression_velocity.allFinite();
      }
    }

    const double net_displacement = history_net.norm();
    const double robust_speed = history_duration > 1e-6 ?
        net_displacement / history_duration : 0.0;
    const double motion_efficiency = history_path_length > 1e-6 ?
        net_displacement / history_path_length : 0.0;
    const double direction_consistency = valid_step_count > 0 ?
        static_cast<double>(consistent_step_count) /
            static_cast<double>(valid_step_count) : 0.0;
    const bool step_speed_plausible =
        maximum_step_speed <=
            parameters_.maximum_motion_step_speed_ratio *
                parameters_.max_speed;
    const bool stable_stationary_background =
        history_duration >=
            parameters_.stationary_background_minimum_duration &&
        net_displacement <=
            parameters_.stationary_background_maximum_displacement &&
        robust_speed <=
            parameters_.stationary_background_maximum_speed;
    const Eigen::Vector2d current_planar_position =
        detection.centroid.head<2>();

    if (!track.motion_qualified && stable_stationary_background)
      ++track.stationary_background_hit_count;
    else if (!stable_stationary_background)
      track.stationary_background_hit_count = 0;

    const auto remember_stationary_background =
        [this](const Eigen::Vector2d &candidate)
        {
          if (parameters_.maximum_stationary_background_anchors <= 0)
            return;
          for (const Eigen::Vector2d &anchor :
               stationary_background_anchors_)
          {
            if ((candidate - anchor).norm() <=
                parameters_.stationary_background_merge_radius)
              return;
          }
          if (stationary_background_anchors_.size() <
              static_cast<std::size_t>(
                  parameters_.maximum_stationary_background_anchors))
            stationary_background_anchors_.push_back(candidate);
        };

    if (track.stationary_background_hit_count ==
        parameters_.minimum_stationary_background_hits)
    {
      remember_stationary_background(
          track.motion_history.back().position.head<2>());
    }

    // Some omitted PCD surfaces do not look low-speed: scan occlusion makes
    // the centroid jump coherently inside one bounded patch.  If the same
    // identity remains trapped in that patch for seconds, learn the patch as
    // background.  A translating object repeatedly exits the dwell radius
    // and therefore never satisfies this test.
    if (track.background_dwell_start_seconds < 0.0)
    {
      track.background_dwell_position = current_planar_position;
      track.background_dwell_start_seconds = stamp_seconds;
    }
    else if ((current_planar_position -
              track.background_dwell_position).norm() >
             parameters_.stationary_background_dwell_radius)
    {
      track.background_dwell_position = current_planar_position;
      track.background_dwell_start_seconds = stamp_seconds;
    }
    else if (stamp_seconds - track.background_dwell_start_seconds >=
             parameters_.stationary_background_dwell_duration)
    {
      remember_stationary_background(
          0.5 * (track.background_dwell_position +
                 current_planar_position));
    }

    bool inside_stationary_background = false;
    for (const Eigen::Vector2d &anchor :
         stationary_background_anchors_)
    {
      if ((current_planar_position - anchor).norm() <=
          parameters_.stationary_background_release_radius)
      {
        inside_stationary_background = true;
        break;
      }
    }
    const bool coherent_motion =
        track.hit_count >= parameters_.min_confirmed_hits &&
        valid_step_count >= parameters_.minimum_dynamic_hits &&
        (parameters_.minimum_motion_cluster_points <= 0 ||
         detection.point_count >= static_cast<std::size_t>(
             parameters_.minimum_motion_cluster_points)) &&
        history_duration >= parameters_.minimum_motion_duration &&
        net_displacement >= parameters_.minimum_dynamic_displacement &&
        robust_speed >= parameters_.dynamic_enter_speed &&
        motion_efficiency >= parameters_.minimum_motion_efficiency &&
        direction_consistency >=
            parameters_.minimum_direction_consistency_fraction &&
        step_speed_plausible &&
        !inside_stationary_background;

    const auto stabilizeVelocityFromHistory = [&]()
    {
      Eigen::Vector2d history_velocity = Eigen::Vector2d::Zero();
      if (regression_velocity_valid)
        history_velocity = regression_velocity;
      else if (history_duration > 1e-6)
        history_velocity = history_net / history_duration;

      if (!history_velocity.allFinite() ||
          history_velocity.norm() <= 1e-6)
      {
        // Immediately after a short occlusion there may not yet be enough
        // samples to refit the motion.  Preserve the last mature direction for
        // one bounded history rebuild rather than interpreting the returning
        // visible surface as a lateral manoeuvre.
        if (track.motion_qualified && track.has_motion_direction)
        {
          const double retained_speed = track.velocity.head<2>().norm();
          track.velocity.head<2>() =
              retained_speed * track.last_motion_direction;
        }
        return;
      }

      const double history_speed = std::min(
          parameters_.max_speed, history_velocity.norm());
      const Eigen::Vector2d target_direction =
          history_velocity / history_velocity.norm();
      Eigen::Vector2d stabilized_direction = target_direction;
      if (track.has_motion_direction)
      {
        const Eigen::Vector2d blended_direction =
            (1.0 - parameters_.motion_direction_alpha) *
                track.last_motion_direction +
            parameters_.motion_direction_alpha * target_direction;
        if (blended_direction.norm() > 1e-6)
          stabilized_direction = blended_direction.normalized();
      }

      const double instantaneous_speed =
          track.velocity.head<2>().norm();
      const double stabilized_speed =
          (1.0 - parameters_.regression_velocity_weight) *
              instantaneous_speed +
          parameters_.regression_velocity_weight * history_speed;
      if (stabilized_speed < parameters_.velocity_deadband)
        track.velocity.head<2>().setZero();
      else
        track.velocity.head<2>() =
            std::min(parameters_.max_speed, stabilized_speed) *
            stabilized_direction;
      track.last_motion_direction = stabilized_direction;
      track.has_motion_direction = true;

      const double three_dimensional_speed = track.velocity.norm();
      if (three_dimensional_speed > parameters_.max_speed)
        track.velocity *= parameters_.max_speed /
                          three_dimensional_speed;
    };

    if (!track.motion_qualified && coherent_motion)
    {
      track.motion_qualified = true;
      track.motion_hit_count = valid_step_count;
      track.stationary_hit_count = 0;
      track.motion_reference_position =
          track.motion_history.front().position;
      stabilizeVelocityFromHistory();
    }
    else if (!track.motion_qualified)
    {
      track.motion_hit_count = coherent_motion ? valid_step_count : 0;
      track.stationary_hit_count = 0;
    }
    else
    {
      stabilizeVelocityFromHistory();
      const bool lost_motion_evidence =
          inside_stationary_background ||
          latest_observed_planar_speed <=
              parameters_.dynamic_exit_speed ||
          (history_duration >= parameters_.minimum_motion_duration &&
           (motion_efficiency <
                0.5 * parameters_.minimum_motion_efficiency ||
            direction_consistency <
                0.5 *
                    parameters_.minimum_direction_consistency_fraction));
      if (lost_motion_evidence)
        ++track.stationary_hit_count;
      else
        track.stationary_hit_count = 0;
      if (track.stationary_hit_count >=
          parameters_.minimum_stationary_hits)
      {
        track.motion_qualified = false;
        track.motion_hit_count = 0;
        track.stationary_hit_count = 0;
        track.motion_reference_position = track.position;
        track.last_motion_direction.setZero();
        track.has_motion_direction = false;
      }
      else if (!track.has_motion_direction &&
               history_direction.norm() > 1e-6)
      {
        track.last_motion_direction = history_direction;
        track.has_motion_direction = true;
      }
    }

    const double association_quality = std::max(
        0.0, 1.0 - residual.norm() / parameters_.association_gate);
    const double maturity = std::min(
        1.0,
        static_cast<double>(track.hit_count) /
            static_cast<double>(parameters_.min_confirmed_hits));
    const double confidence =
        std::max(0.0, std::min(1.0,
                              maturity * (0.70 +
                                          0.30 * association_quality)));

    TrackedDetection tracked;
    tracked.detection_index = detection_index;
    tracked.track_id = track.id;
    tracked.filtered_position = track.position;
    tracked.velocity = track.velocity;
    tracked.confidence = confidence;
    tracked.confirmed =
        track.hit_count >= parameters_.min_confirmed_hits;
    tracked.motion_qualified =
        tracked.confirmed && track.motion_qualified;
    result.push_back(tracked);
  }

  for (std::size_t track_index = 0;
       track_index < tracks_.size(); ++track_index)
  {
    if (track_assigned[track_index])
      continue;
    Track &track = tracks_[track_index];
    const double dt = std::max(
        0.0, std::min(parameters_.max_dt,
                      stamp_seconds - track.stamp_seconds));
    track.position += dt * track.velocity;
    track.stamp_seconds = stamp_seconds;
    ++track.missed_frames;
  }

  tracks_.erase(
      std::remove_if(
          tracks_.begin(), tracks_.end(),
          [this](const Track &track)
          {
            return track.missed_frames >
                   parameters_.max_missed_frames;
          }),
      tracks_.end());

  for (std::size_t detection_index = 0;
       detection_index < detections.size(); ++detection_index)
  {
    if (detection_to_track[detection_index] >= 0 ||
        !detections[detection_index].centroid.allFinite())
      continue;

    Track track;
    track.id = next_track_id_++;
    if (next_track_id_ == 0)
      next_track_id_ = 1;
    track.position = detections[detection_index].centroid;
    track.motion_reference_position = track.position;
    track.extent = detections[detection_index].extent;
    track.background_dwell_position = track.position.head<2>();
    track.background_dwell_start_seconds = stamp_seconds;
    track.point_count =
        static_cast<double>(detections[detection_index].point_count);
    track.stamp_seconds = stamp_seconds;
    track.hit_count = 1;
    track.motion_history.push_back(
        MotionSample{detections[detection_index].centroid,
                     stamp_seconds});
    tracks_.push_back(track);

    TrackedDetection tracked;
    tracked.detection_index = detection_index;
    tracked.track_id = track.id;
    tracked.filtered_position = track.position;
    tracked.velocity.setZero();
    tracked.confidence = std::min(
        1.0,
        1.0 / static_cast<double>(parameters_.min_confirmed_hits));
    tracked.confirmed = parameters_.min_confirmed_hits <= 1;
    tracked.motion_qualified = false;
    result.push_back(tracked);
  }

  std::sort(result.begin(), result.end(),
            [](const TrackedDetection &lhs,
               const TrackedDetection &rhs)
            {
              return lhs.detection_index < rhs.detection_index;
            });
  return result;
}

void DynamicObstacleTrackerCore::reset(bool clear_stationary_background)
{
  tracks_.clear();
  if (clear_stationary_background)
    stationary_background_anchors_.clear();
  next_track_id_ = 1;
  last_stamp_seconds_ = -1.0;
}

std::size_t DynamicObstacleTrackerCore::trackCount() const
{
  return tracks_.size();
}

std::size_t
DynamicObstacleTrackerCore::stationaryBackgroundAnchorCount() const
{
  return stationary_background_anchors_.size();
}

double DynamicObstacleTrackerCore::lastStamp() const
{
  return last_stamp_seconds_;
}

DynamicPerceptionHealthGate::DynamicPerceptionHealthGate()
  : DynamicPerceptionHealthGate(Parameters())
{
}

DynamicPerceptionHealthGate::DynamicPerceptionHealthGate(
    const Parameters &parameters)
  : parameters_(parameters)
{
  parameters_.maximum_motion_tracks =
      std::max(0, parameters_.maximum_motion_tracks);
  parameters_.recovery_frames =
      std::max(1, parameters_.recovery_frames);
  parameters_.degradation_frames =
      std::max(1, parameters_.degradation_frames);
}

bool DynamicPerceptionHealthGate::update(int motion_track_count)
{
  const bool plausible =
      motion_track_count >= 0 &&
      motion_track_count <= parameters_.maximum_motion_tracks;
  if (plausible)
  {
    consecutive_bad_frames_ = 0;
    ++consecutive_good_frames_;
    if (consecutive_good_frames_ >= parameters_.recovery_frames)
      state_ = State::HEALTHY;
  }
  else
  {
    consecutive_good_frames_ = 0;
    ++consecutive_bad_frames_;
    if (consecutive_bad_frames_ >= parameters_.degradation_frames)
      state_ = State::DEGRADED;
  }
  return healthy();
}

bool DynamicPerceptionHealthGate::healthy() const
{
  return state_ == State::HEALTHY;
}

DynamicPerceptionHealthGate::State
DynamicPerceptionHealthGate::state() const
{
  return state_;
}

int DynamicPerceptionHealthGate::consecutiveGoodFrames() const
{
  return consecutive_good_frames_;
}

int DynamicPerceptionHealthGate::consecutiveBadFrames() const
{
  return consecutive_bad_frames_;
}

void DynamicPerceptionHealthGate::invalidate()
{
  state_ = State::DEGRADED;
  consecutive_good_frames_ = 0;
  consecutive_bad_frames_ = 0;
}

void DynamicPerceptionHealthGate::reset()
{
  state_ = State::WARMING_UP;
  consecutive_good_frames_ = 0;
  consecutive_bad_frames_ = 0;
}

}  // namespace mm_remani

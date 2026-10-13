#include <mm_remani/dynamic_obstacle_tracker_core.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mm_remani
{
namespace
{

struct ClusterDetection
{
  DynamicDetection tracking_detection;
  std::vector<Eigen::Vector3f> points;
};

sensor_msgs::PointField makeField(const std::string &name,
                                  std::uint32_t offset,
                                  std::uint8_t datatype)
{
  sensor_msgs::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1;
  return field;
}

template <typename T>
void writeValue(std::vector<std::uint8_t> &data,
                std::size_t offset,
                const T &value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

}  // namespace

class DynamicObstacleTrackerNode
{
public:
  DynamicObstacleTrackerNode()
    : private_nh_("~"),
      tf_buffer_(ros::Duration(15.0)),
      tf_listener_(tf_buffer_)
  {
    private_nh_.param("input_topic", input_topic_,
                      std::string("/velodyne_points"));
    private_nh_.param("static_map_topic", static_map_topic_,
                      std::string("/map_generator/global_cloud"));
    private_nh_.param("output_topic", output_topic_,
                      std::string("/dynamic_points"));
    private_nh_.param("world_frame", world_frame_, std::string("world"));
    private_nh_.param("require_static_map", require_static_map_, true);
    private_nh_.param("tf_timeout", tf_timeout_, 0.03);
    private_nh_.param("max_cloud_age", max_cloud_age_, 0.30);
    private_nh_.param("future_stamp_tolerance",
                      future_stamp_tolerance_, 0.05);
    private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.10);
    private_nh_.param("static_map_leaf_size",
                      static_map_leaf_size_, 0.08);
    private_nh_.param("static_match_radius", static_match_radius_, 0.12);
    private_nh_.param("minimum_range", minimum_range_, 0.30);
    private_nh_.param("maximum_range", maximum_range_, 6.0);
    private_nh_.param("minimum_world_z", minimum_world_z_, 0.15);
    private_nh_.param("maximum_world_z", maximum_world_z_, 1.70);
    private_nh_.param("cluster_tolerance", cluster_tolerance_, 0.32);
    private_nh_.param("minimum_cluster_points",
                      minimum_cluster_points_, 3);
    private_nh_.param("maximum_cluster_points",
                      maximum_cluster_points_, 1200);
    private_nh_.param("maximum_cluster_extent",
                      maximum_cluster_extent_, 2.20);
    private_nh_.param("maximum_residual_points",
                      maximum_residual_points_, 4000);
    private_nh_.param("maximum_output_points",
                      maximum_output_points_, 240);
    private_nh_.param("motion_center_trim_fraction",
                      stable_motion_center_parameters_.trim_fraction, 0.15);
    private_nh_.param(
        "motion_center_tall_target_min_height",
        stable_motion_center_parameters_.tall_target_min_height, 0.65);
    private_nh_.param(
        "motion_center_stable_core_lower_height_ratio",
        stable_motion_center_parameters_.stable_core_lower_height_ratio,
        0.35);
    private_nh_.param(
        "motion_center_stable_core_upper_height_ratio",
        stable_motion_center_parameters_.stable_core_upper_height_ratio,
        0.90);
    private_nh_.param(
        "motion_center_minimum_stable_core_points",
        stable_motion_center_parameters_.minimum_stable_core_points, 8);

    DynamicObstacleTrackerCore::Parameters tracker_parameters;
    private_nh_.param("association_gate",
                      tracker_parameters.association_gate, 0.75);
    private_nh_.param("association_position_allowance",
                      tracker_parameters.association_position_allowance,
                      0.12);
    private_nh_.param("association_shape_weight",
                      tracker_parameters.association_shape_weight, 0.20);
    private_nh_.param("maximum_extent_change",
                      tracker_parameters.maximum_extent_change, 0.45);
    private_nh_.param("maximum_point_count_ratio",
                      tracker_parameters.maximum_point_count_ratio, 4.0);
    private_nh_.param("position_alpha",
                      tracker_parameters.position_alpha, 0.75);
    private_nh_.param("velocity_alpha",
                      tracker_parameters.velocity_alpha, 0.45);
    private_nh_.param("regression_velocity_weight",
                      tracker_parameters.regression_velocity_weight, 0.70);
    private_nh_.param("motion_direction_alpha",
                      tracker_parameters.motion_direction_alpha, 0.30);
    private_nh_.param("velocity_deadband",
                      tracker_parameters.velocity_deadband, 0.08);
    private_nh_.param("maximum_track_speed",
                      tracker_parameters.max_speed, 2.0);
    private_nh_.param("dynamic_enter_speed",
                      tracker_parameters.dynamic_enter_speed, 0.18);
    private_nh_.param("dynamic_exit_speed",
                      tracker_parameters.dynamic_exit_speed, 0.10);
    private_nh_.param("minimum_dynamic_displacement",
                      tracker_parameters.minimum_dynamic_displacement, 0.06);
    private_nh_.param("direction_consistency_cosine",
                      tracker_parameters.direction_consistency_cosine, 0.30);
    private_nh_.param("minimum_motion_duration",
                      tracker_parameters.minimum_motion_duration, 0.25);
    private_nh_.param("motion_history_duration",
                      tracker_parameters.motion_history_duration, 0.60);
    private_nh_.param("minimum_motion_efficiency",
                      tracker_parameters.minimum_motion_efficiency, 0.70);
    private_nh_.param("minimum_direction_consistency_fraction",
                      tracker_parameters.minimum_direction_consistency_fraction,
                      0.75);
    private_nh_.param("maximum_motion_step_speed_ratio",
                      tracker_parameters.maximum_motion_step_speed_ratio,
                      1.25);
    private_nh_.param("minimum_track_dt",
                      tracker_parameters.min_dt, 0.03);
    private_nh_.param("maximum_track_dt",
                      tracker_parameters.max_dt, 0.35);
    private_nh_.param("minimum_confirmed_hits",
                      tracker_parameters.min_confirmed_hits, 2);
    private_nh_.param("minimum_dynamic_hits",
                      tracker_parameters.minimum_dynamic_hits, 3);
    private_nh_.param("minimum_motion_cluster_points",
                      tracker_parameters.minimum_motion_cluster_points, 0);
    private_nh_.param("minimum_stationary_hits",
                      tracker_parameters.minimum_stationary_hits, 3);
    private_nh_.param("minimum_stationary_background_hits",
                      tracker_parameters.minimum_stationary_background_hits,
                      5);
    private_nh_.param("stationary_background_minimum_duration",
                      tracker_parameters.stationary_background_minimum_duration,
                      0.50);
    private_nh_.param("stationary_background_maximum_speed",
                      tracker_parameters.stationary_background_maximum_speed,
                      0.10);
    private_nh_.param("stationary_background_maximum_displacement",
                      tracker_parameters.stationary_background_maximum_displacement,
                      0.06);
    private_nh_.param("stationary_background_merge_radius",
                      tracker_parameters.stationary_background_merge_radius,
                      0.20);
    private_nh_.param("stationary_background_release_radius",
                      tracker_parameters.stationary_background_release_radius,
                      0.25);
    private_nh_.param("stationary_background_dwell_duration",
                      tracker_parameters.stationary_background_dwell_duration,
                      2.0);
    private_nh_.param("stationary_background_dwell_radius",
                      tracker_parameters.stationary_background_dwell_radius,
                      0.30);
    private_nh_.param("maximum_stationary_background_anchors",
                      tracker_parameters.maximum_stationary_background_anchors,
                      128);
    private_nh_.param("maximum_missed_frames",
                      tracker_parameters.max_missed_frames, 3);
    tracker_.reset(new DynamicObstacleTrackerCore(tracker_parameters));

    DynamicPerceptionHealthGate::Parameters health_parameters;
    private_nh_.param("health_maximum_motion_tracks",
                      health_parameters.maximum_motion_tracks, 3);
    private_nh_.param("health_recovery_frames",
                      health_parameters.recovery_frames, 15);
    private_nh_.param("health_degradation_frames",
                      health_parameters.degradation_frames, 3);
    maximum_healthy_motion_tracks_ = std::max(
        0, health_parameters.maximum_motion_tracks);
    health_gate_.reset(
        new DynamicPerceptionHealthGate(health_parameters));

    tf_timeout_ = std::max(0.0, tf_timeout_);
    max_cloud_age_ = std::max(0.05, max_cloud_age_);
    future_stamp_tolerance_ = std::max(0.0, future_stamp_tolerance_);
    voxel_leaf_size_ = std::max(0.03, voxel_leaf_size_);
    static_map_leaf_size_ = std::max(0.03, static_map_leaf_size_);
    static_match_radius_ = std::max(0.0, static_match_radius_);
    minimum_range_ = std::max(0.0, minimum_range_);
    maximum_range_ = std::max(minimum_range_ + 0.1, maximum_range_);
    maximum_world_z_ = std::max(minimum_world_z_ + 0.05,
                                maximum_world_z_);
    cluster_tolerance_ = std::max(voxel_leaf_size_, cluster_tolerance_);
    minimum_cluster_points_ = std::max(1, minimum_cluster_points_);
    maximum_cluster_points_ = std::max(minimum_cluster_points_,
                                       maximum_cluster_points_);
    maximum_cluster_extent_ = std::max(0.1, maximum_cluster_extent_);
    maximum_residual_points_ = std::max(100, maximum_residual_points_);
    maximum_output_points_ = std::max(1, maximum_output_points_);

    static_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    static_tree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    dynamic_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        output_topic_, 1);
    track_count_pub_ = private_nh_.advertise<std_msgs::Int32>(
        "confirmed_track_count", 1);
    motion_qualified_track_count_pub_ =
        private_nh_.advertise<std_msgs::Int32>(
            "motion_qualified_track_count", 1);
    stationary_confirmed_track_count_pub_ =
        private_nh_.advertise<std_msgs::Int32>(
            "stationary_confirmed_track_count", 1);
    stationary_background_anchor_count_pub_ =
        private_nh_.advertise<std_msgs::Int32>(
            "stationary_background_anchor_count", 1, true);
    cluster_count_pub_ = private_nh_.advertise<std_msgs::Int32>(
        "cluster_count", 1);
    processing_time_pub_ = private_nh_.advertise<std_msgs::Float64>(
        "processing_ms", 1);
    input_age_pub_ = private_nh_.advertise<std_msgs::Float64>(
        "input_age", 1);
    residual_count_pub_ = private_nh_.advertise<std_msgs::Int32>(
        "residual_point_count", 1);
    maximum_confirmed_speed_pub_ =
        private_nh_.advertise<std_msgs::Float64>(
            "maximum_confirmed_speed", 1);
    maximum_motion_qualified_speed_pub_ =
        private_nh_.advertise<std_msgs::Float64>(
            "maximum_motion_qualified_speed", 1);
    perception_healthy_pub_ = private_nh_.advertise<std_msgs::Bool>(
        "perception_healthy", 1, true);
    perception_health_state_pub_ =
        private_nh_.advertise<std_msgs::Int32>(
            "perception_health_state", 1, true);
    suppressed_motion_track_count_pub_ =
        private_nh_.advertise<std_msgs::Int32>(
            "suppressed_motion_track_count", 1);

    std_msgs::Bool initial_health;
    initial_health.data = false;
    perception_healthy_pub_.publish(initial_health);
    std_msgs::Int32 initial_health_state;
    initial_health_state.data = static_cast<int>(
        DynamicPerceptionHealthGate::State::WARMING_UP);
    perception_health_state_pub_.publish(initial_health_state);

    static_map_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        static_map_topic_, 1,
        &DynamicObstacleTrackerNode::staticMapCallback, this,
        ros::TransportHints().tcpNoDelay());
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        input_topic_, 1,
        &DynamicObstacleTrackerNode::cloudCallback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO("[DynamicTracker] Shadow producer ready: input=%s, static_map=%s, output=%s, frame=%s.",
             input_topic_.c_str(), static_map_topic_.c_str(),
             output_topic_.c_str(), world_frame_.c_str());
  }

private:
  bool lookupWorldTransform(const std::string &source_frame,
                            const ros::Time &stamp,
                            Eigen::Matrix3d &rotation,
                            Eigen::Vector3d &translation)
  {
    rotation.setIdentity();
    translation.setZero();
    if (source_frame == world_frame_)
      return true;
    try
    {
      const geometry_msgs::TransformStamped transform =
          tf_buffer_.lookupTransform(
              world_frame_, source_frame, stamp,
              ros::Duration(tf_timeout_));
      const auto &q = transform.transform.rotation;
      Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
      if (!std::isfinite(quaternion.norm()) ||
          quaternion.norm() < 1e-9)
        return false;
      rotation = quaternion.normalized().toRotationMatrix();
      translation << transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z;
      return rotation.allFinite() && translation.allFinite();
    }
    catch (const tf2::TransformException &exception)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[DynamicTracker] TF %s -> %s unavailable at cloud time: %s",
          source_frame.c_str(), world_frame_.c_str(), exception.what());
      return false;
    }
  }

  void staticMapCallback(
      const sensor_msgs::PointCloud2ConstPtr &message)
  {
    if (!message || message->header.frame_id.empty())
      return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*message, *input);
    if (input->empty())
      return;

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    const ros::Time transform_stamp =
        message->header.stamp.isZero() ? ros::Time(0) :
                                         message->header.stamp;
    if (!lookupWorldTransform(message->header.frame_id, transform_stamp,
                              rotation, translation))
      return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr world(
        new pcl::PointCloud<pcl::PointXYZ>());
    world->reserve(input->size());
    for (const pcl::PointXYZ &point : input->points)
    {
      if (!pcl::isFinite(point))
        continue;
      const Eigen::Vector3d transformed =
          rotation * Eigen::Vector3d(point.x, point.y, point.z) +
          translation;
      if (!transformed.allFinite())
        continue;
      world->push_back(pcl::PointXYZ(
          static_cast<float>(transformed.x()),
          static_cast<float>(transformed.y()),
          static_cast<float>(transformed.z())));
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(static_map_leaf_size_),
                      static_cast<float>(static_map_leaf_size_),
                      static_cast<float>(static_map_leaf_size_));
    voxel.setInputCloud(world);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
        new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*filtered);
    if (filtered->empty())
      return;

    static_cloud_ = filtered;
    static_tree_->setInputCloud(static_cloud_);
    static_map_ready_ = true;
    tracker_->reset();
    health_gate_->reset();
    std_msgs::Bool health_message;
    health_message.data = false;
    perception_healthy_pub_.publish(health_message);
    std_msgs::Int32 health_state_message;
    health_state_message.data = static_cast<int>(
        DynamicPerceptionHealthGate::State::WARMING_UP);
    perception_health_state_pub_.publish(health_state_message);
    ROS_INFO("[DynamicTracker] Static reference ready: %zu points in %s; tracks reset for the new map frame.",
             static_cloud_->size(), world_frame_.c_str());
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &message)
  {
    const ros::WallTime wall_start = ros::WallTime::now();
    if (!message || message->header.stamp.isZero() ||
        message->header.frame_id.empty())
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[DynamicTracker] Reject cloud without timestamp/frame_id.");
      return;
    }

    const double input_age =
        (ros::Time::now() - message->header.stamp).toSec();
    if (!std::isfinite(input_age) || input_age > max_cloud_age_ ||
        input_age < -future_stamp_tolerance_)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[DynamicTracker] Reject cloud age %.3f s outside [-%.3f, %.3f] s.",
          input_age, future_stamp_tolerance_, max_cloud_age_);
      return;
    }
    if (require_static_map_ && !static_map_ready_)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[DynamicTracker] Waiting for static PCD reference; do not publish a false clear frame.");
      return;
    }

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    if (!lookupWorldTransform(message->header.frame_id,
                              message->header.stamp,
                              rotation, translation))
      return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input(
        new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*message, *input);
    if (input->empty())
      return;

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setLeafSize(static_cast<float>(voxel_leaf_size_),
                      static_cast<float>(voxel_leaf_size_),
                      static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(
        new pcl::PointCloud<pcl::PointXYZ>());
    voxel.filter(*downsampled);

    pcl::PointCloud<pcl::PointXYZ>::Ptr residual(
        new pcl::PointCloud<pcl::PointXYZ>());
    residual->reserve(downsampled->size());
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    const double minimum_range_squared = minimum_range_ * minimum_range_;
    const double maximum_range_squared = maximum_range_ * maximum_range_;
    const float static_radius_squared = static_cast<float>(
        static_match_radius_ * static_match_radius_);

    for (const pcl::PointXYZ &point : downsampled->points)
    {
      if (!pcl::isFinite(point))
        continue;
      const Eigen::Vector3d sensor_point(point.x, point.y, point.z);
      const double range_squared = sensor_point.squaredNorm();
      if (range_squared < minimum_range_squared ||
          range_squared > maximum_range_squared)
        continue;

      const Eigen::Vector3d world_point =
          rotation * sensor_point + translation;
      if (!world_point.allFinite() ||
          world_point.z() < minimum_world_z_ ||
          world_point.z() > maximum_world_z_)
        continue;

      pcl::PointXYZ candidate(
          static_cast<float>(world_point.x()),
          static_cast<float>(world_point.y()),
          static_cast<float>(world_point.z()));
      bool matches_static = false;
      if (static_map_ready_ && static_match_radius_ > 0.0 &&
          static_tree_->nearestKSearch(
              candidate, 1, nearest_index,
              nearest_squared_distance) > 0)
      {
        matches_static =
            nearest_squared_distance.front() <= static_radius_squared;
      }
      if (!matches_static)
        residual->push_back(candidate);
    }

    if (residual->size() >
        static_cast<std::size_t>(maximum_residual_points_))
    {
      // This is a hard registration/filtering failure, not evidence for many
      // independent moving objects.  Forget identities learned in the bad
      // frame and close the output immediately.  A subsequent recovery must
      // pass the full stable-frame warm-up again.
      tracker_->reset(false);
      health_gate_->invalidate();
      std_msgs::Bool health_message;
      health_message.data = false;
      perception_healthy_pub_.publish(health_message);
      std_msgs::Int32 health_state_message;
      health_state_message.data = static_cast<int>(
          DynamicPerceptionHealthGate::State::DEGRADED);
      perception_health_state_pub_.publish(health_state_message);
      std_msgs::Int32 residual_count_message;
      residual_count_message.data = static_cast<int>(residual->size());
      residual_count_pub_.publish(residual_count_message);
      ROS_ERROR_THROTTLE(
          1.0,
          "[DynamicTracker] Reject residual cloud with %zu points (limit=%d). Static-map alignment/filtering is unsafe; do not label the scene dynamic.",
          residual->size(), maximum_residual_points_);
      return;
    }

    std::vector<ClusterDetection> clusters;
    if (!residual->empty())
    {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree(
          new pcl::search::KdTree<pcl::PointXYZ>());
      cluster_tree->setInputCloud(residual);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
      extraction.setClusterTolerance(cluster_tolerance_);
      extraction.setMinClusterSize(minimum_cluster_points_);
      extraction.setMaxClusterSize(maximum_cluster_points_);
      extraction.setSearchMethod(cluster_tree);
      extraction.setInputCloud(residual);
      extraction.extract(cluster_indices);

      clusters.reserve(cluster_indices.size());
      for (const pcl::PointIndices &indices : cluster_indices)
      {
        if (indices.indices.empty())
          continue;
        Eigen::Vector3d minimum =
            Eigen::Vector3d::Constant(
                std::numeric_limits<double>::infinity());
        Eigen::Vector3d maximum =
            Eigen::Vector3d::Constant(
                -std::numeric_limits<double>::infinity());
        ClusterDetection cluster;
        cluster.points.reserve(indices.indices.size());
        for (const int index : indices.indices)
        {
          const pcl::PointXYZ &point =
              residual->points[static_cast<std::size_t>(index)];
          const Eigen::Vector3d value(point.x, point.y, point.z);
          minimum = minimum.cwiseMin(value);
          maximum = maximum.cwiseMax(value);
          cluster.points.push_back(value.cast<float>());
        }
        if ((maximum - minimum).maxCoeff() > maximum_cluster_extent_)
          continue;
        // Estimate target motion from a robust target-level anchor.  Keep the
        // complete point list above untouched: it remains the collision shape
        // published to the planner, while only association/velocity use this
        // stable centre.
        cluster.tracking_detection.centroid =
            estimateStableMotionCenter(
                cluster.points, stable_motion_center_parameters_);
        cluster.tracking_detection.extent = maximum - minimum;
        cluster.tracking_detection.point_count = indices.indices.size();
        clusters.push_back(std::move(cluster));
      }
    }

    std::vector<DynamicDetection> detections;
    detections.reserve(clusters.size());
    for (const ClusterDetection &cluster : clusters)
      detections.push_back(cluster.tracking_detection);

    const std::vector<TrackedDetection> tracked =
        tracker_->update(detections, message->header.stamp.toSec());

    int confirmed_count = 0;
    int motion_qualified_count = 0;
    int stationary_confirmed_count = 0;
    double maximum_confirmed_speed = 0.0;
    double maximum_motion_qualified_speed = 0.0;
    for (const TrackedDetection &track : tracked)
    {
      if (track.confirmed)
      {
        ++confirmed_count;
        maximum_confirmed_speed = std::max(
            maximum_confirmed_speed, track.velocity.norm());
        if (track.motion_qualified)
        {
          ++motion_qualified_count;
          maximum_motion_qualified_speed = std::max(
              maximum_motion_qualified_speed,
              track.velocity.head<2>().norm());
        }
        else
        {
          ++stationary_confirmed_count;
        }
      }
    }

    const DynamicPerceptionHealthGate::State previous_health_state =
        health_gate_->state();
    const bool perception_healthy =
        health_gate_->update(motion_qualified_count);
    const DynamicPerceptionHealthGate::State current_health_state =
        health_gate_->state();
    const bool frame_motion_count_plausible =
        motion_qualified_count <= maximum_healthy_motion_tracks_;
    const bool publish_allowed =
        perception_healthy && frame_motion_count_plausible;

    if (current_health_state != previous_health_state)
    {
      if (current_health_state ==
          DynamicPerceptionHealthGate::State::HEALTHY)
      {
        ROS_INFO("[DynamicTracker] Perception health gate opened after %d stable frames.",
                 health_gate_->consecutiveGoodFrames());
      }
      else if (current_health_state ==
               DynamicPerceptionHealthGate::State::DEGRADED)
      {
        // Drop identities learned from an incoherent whole-scene motion
        // episode.  Recovery still requires a complete stable warm-up, so a
        // reset cannot immediately reopen the active output.
        tracker_->reset(false);
        ROS_ERROR("[DynamicTracker] Perception degraded: %d simultaneous motion-qualified tracks exceed the scene limit %d. Suppress point-flow output and fall back to live occupancy/ESDF.",
                  motion_qualified_count,
                  maximum_healthy_motion_tracks_);
      }
    }

    if (publish_allowed)
      publishDynamicPoints(message->header.stamp, clusters, tracked);

    std_msgs::Int32 track_count_message;
    track_count_message.data = confirmed_count;
    track_count_pub_.publish(track_count_message);
    std_msgs::Int32 motion_qualified_count_message;
    motion_qualified_count_message.data = motion_qualified_count;
    motion_qualified_track_count_pub_.publish(
        motion_qualified_count_message);
    std_msgs::Int32 stationary_confirmed_count_message;
    stationary_confirmed_count_message.data = stationary_confirmed_count;
    stationary_confirmed_track_count_pub_.publish(
        stationary_confirmed_count_message);
    std_msgs::Int32 stationary_background_anchor_count_message;
    stationary_background_anchor_count_message.data = static_cast<int>(
        tracker_->stationaryBackgroundAnchorCount());
    stationary_background_anchor_count_pub_.publish(
        stationary_background_anchor_count_message);
    std_msgs::Int32 cluster_count_message;
    cluster_count_message.data = static_cast<int>(clusters.size());
    cluster_count_pub_.publish(cluster_count_message);
    std_msgs::Float64 processing_message;
    processing_message.data =
        (ros::WallTime::now() - wall_start).toSec() * 1000.0;
    processing_time_pub_.publish(processing_message);
    std_msgs::Float64 age_message;
    age_message.data = input_age;
    input_age_pub_.publish(age_message);
    std_msgs::Int32 residual_count_message;
    residual_count_message.data = static_cast<int>(residual->size());
    residual_count_pub_.publish(residual_count_message);
    std_msgs::Float64 maximum_speed_message;
    maximum_speed_message.data = maximum_confirmed_speed;
    maximum_confirmed_speed_pub_.publish(maximum_speed_message);
    std_msgs::Float64 maximum_motion_speed_message;
    maximum_motion_speed_message.data = maximum_motion_qualified_speed;
    maximum_motion_qualified_speed_pub_.publish(
        maximum_motion_speed_message);
    std_msgs::Bool health_message;
    // One implausible frame is suppressed immediately even though the state
    // machine deliberately waits for consecutive failures before declaring a
    // persistent DEGRADED state.  This boolean describes whether this exact
    // frame was safe to expose to the planner.
    health_message.data = publish_allowed;
    perception_healthy_pub_.publish(health_message);
    std_msgs::Int32 health_state_message;
    health_state_message.data = static_cast<int>(current_health_state);
    perception_health_state_pub_.publish(health_state_message);
    std_msgs::Int32 suppressed_motion_message;
    suppressed_motion_message.data =
        publish_allowed ? 0 : motion_qualified_count;
    suppressed_motion_track_count_pub_.publish(
        suppressed_motion_message);

    ROS_INFO_THROTTLE(
        1.0,
        "[DynamicTracker] downsampled=%zu residual=%zu clusters=%zu confirmed=%d motion_qualified=%d rejected_stationary=%d background_anchors=%zu tracks=%zu health=%d publish=%s age=%.3fs processing=%.2fms.",
        downsampled->size(), residual->size(), clusters.size(),
        confirmed_count, motion_qualified_count,
        stationary_confirmed_count,
        tracker_->stationaryBackgroundAnchorCount(),
        tracker_->trackCount(),
        static_cast<int>(current_health_state),
        publish_allowed ? "yes" : "no", input_age,
        processing_message.data);
  }

  void publishDynamicPoints(
      const ros::Time &stamp,
      const std::vector<ClusterDetection> &clusters,
      const std::vector<TrackedDetection> &tracked)
  {
    struct OutputPoint
    {
      Eigen::Vector3f position;
      Eigen::Vector3f velocity;
      float confidence;
      std::uint32_t track_id;
    };

    std::size_t total_dynamic_points = 0;
    for (const TrackedDetection &track : tracked)
    {
      if (track.motion_qualified &&
          track.detection_index < clusters.size())
        total_dynamic_points +=
            clusters[track.detection_index].points.size();
    }
    const std::size_t stride = std::max<std::size_t>(
        1, (total_dynamic_points +
            static_cast<std::size_t>(maximum_output_points_) - 1) /
               static_cast<std::size_t>(maximum_output_points_));

    std::vector<OutputPoint> output;
    output.reserve(std::min(
        total_dynamic_points,
        static_cast<std::size_t>(maximum_output_points_)));
    std::size_t flattened_index = 0;
    for (const TrackedDetection &track : tracked)
    {
      if (!track.motion_qualified ||
          track.detection_index >= clusters.size())
        continue;
      const Eigen::Vector3f velocity = track.velocity.cast<float>();
      for (const Eigen::Vector3f &point :
           clusters[track.detection_index].points)
      {
        if (flattened_index++ % stride != 0)
          continue;
        if (output.size() >=
            static_cast<std::size_t>(maximum_output_points_))
          break;
        output.push_back(OutputPoint{
            point, velocity, static_cast<float>(track.confidence),
            track.track_id});
      }
    }

    sensor_msgs::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = world_frame_;
    message.height = 1;
    message.width = static_cast<std::uint32_t>(output.size());
    message.is_bigendian = false;
    message.is_dense = false;
    message.fields = {
        makeField("x", 0, sensor_msgs::PointField::FLOAT32),
        makeField("y", 4, sensor_msgs::PointField::FLOAT32),
        makeField("z", 8, sensor_msgs::PointField::FLOAT32),
        makeField("vx", 12, sensor_msgs::PointField::FLOAT32),
        makeField("vy", 16, sensor_msgs::PointField::FLOAT32),
        makeField("vz", 20, sensor_msgs::PointField::FLOAT32),
        makeField("confidence", 24,
                  sensor_msgs::PointField::FLOAT32),
        makeField("track_id", 28,
                  sensor_msgs::PointField::UINT32)};
    message.point_step = 32;
    message.row_step = message.point_step * message.width;
    message.data.resize(message.row_step);

    for (std::size_t index = 0; index < output.size(); ++index)
    {
      const std::size_t offset = index * message.point_step;
      writeValue(message.data, offset + 0, output[index].position.x());
      writeValue(message.data, offset + 4, output[index].position.y());
      writeValue(message.data, offset + 8, output[index].position.z());
      writeValue(message.data, offset + 12, output[index].velocity.x());
      writeValue(message.data, offset + 16, output[index].velocity.y());
      writeValue(message.data, offset + 20, output[index].velocity.z());
      writeValue(message.data, offset + 24, output[index].confidence);
      writeValue(message.data, offset + 28, output[index].track_id);
    }
    dynamic_points_pub_.publish(message);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber static_map_sub_;
  ros::Subscriber cloud_sub_;
  ros::Publisher dynamic_points_pub_;
  ros::Publisher track_count_pub_;
  ros::Publisher motion_qualified_track_count_pub_;
  ros::Publisher stationary_confirmed_track_count_pub_;
  ros::Publisher stationary_background_anchor_count_pub_;
  ros::Publisher cluster_count_pub_;
  ros::Publisher processing_time_pub_;
  ros::Publisher input_age_pub_;
  ros::Publisher residual_count_pub_;
  ros::Publisher maximum_confirmed_speed_pub_;
  ros::Publisher maximum_motion_qualified_speed_pub_;
  ros::Publisher perception_healthy_pub_;
  ros::Publisher perception_health_state_pub_;
  ros::Publisher suppressed_motion_track_count_pub_;

  std::string input_topic_;
  std::string static_map_topic_;
  std::string output_topic_;
  std::string world_frame_;
  bool require_static_map_ = true;
  bool static_map_ready_ = false;
  double tf_timeout_ = 0.03;
  double max_cloud_age_ = 0.30;
  double future_stamp_tolerance_ = 0.05;
  double voxel_leaf_size_ = 0.10;
  double static_map_leaf_size_ = 0.08;
  double static_match_radius_ = 0.12;
  double minimum_range_ = 0.30;
  double maximum_range_ = 6.0;
  double minimum_world_z_ = 0.15;
  double maximum_world_z_ = 1.70;
  double cluster_tolerance_ = 0.32;
  int minimum_cluster_points_ = 3;
  int maximum_cluster_points_ = 1200;
  double maximum_cluster_extent_ = 2.20;
  int maximum_residual_points_ = 4000;
  int maximum_output_points_ = 240;
  int maximum_healthy_motion_tracks_ = 3;
  StableMotionCenterParameters stable_motion_center_parameters_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr static_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr static_tree_;
  std::unique_ptr<DynamicObstacleTrackerCore> tracker_;
  std::unique_ptr<DynamicPerceptionHealthGate> health_gate_;
};

}  // namespace mm_remani

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_obstacle_tracker");
  mm_remani::DynamicObstacleTrackerNode node;
  ros::spin();
  return 0;
}

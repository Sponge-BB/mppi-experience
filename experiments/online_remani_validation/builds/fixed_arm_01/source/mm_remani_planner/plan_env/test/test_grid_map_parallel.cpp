#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <plan_env/grid_map.h>

class GridMapParallelTestAccessor
{
public:
  static void configureDynamicPlanningOverlay(
      GridMap &map,
      const std::vector<Eigen::Vector3d> &centers,
      const std::vector<float> &radii)
  {
    ASSERT_EQ(centers.size(), radii.size());
    map.mp_.dynamic_point_overlay_enable_ = true;
    map.mp_.dynamic_point_overlay_influence_distance_ = 2.0;
    map.dynamic_point_overlay_cloud_.reset(
        new pcl::PointCloud<pcl::PointXYZ>());
    map.dynamic_point_overlay_min_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::infinity());
    map.dynamic_point_overlay_max_ = Eigen::Vector3d::Constant(
        -std::numeric_limits<double>::infinity());
    map.dynamic_point_overlay_inflation_ = 0.0;
    for (std::size_t i = 0; i < centers.size(); ++i)
    {
      map.dynamic_point_overlay_cloud_->push_back(pcl::PointXYZ(
          static_cast<float>(centers[i].x()),
          static_cast<float>(centers[i].y()),
          static_cast<float>(centers[i].z())));
      map.dynamic_point_overlay_min_ =
          map.dynamic_point_overlay_min_.cwiseMin(centers[i]);
      map.dynamic_point_overlay_max_ =
          map.dynamic_point_overlay_max_.cwiseMax(centers[i]);
      map.dynamic_point_overlay_inflation_ = std::max(
          map.dynamic_point_overlay_inflation_,
          static_cast<double>(radii[i]));
    }
    map.dynamic_point_overlay_radii_ = radii;
    map.dynamic_point_overlay_cloud_->width =
        map.dynamic_point_overlay_cloud_->size();
    map.dynamic_point_overlay_cloud_->height = 1;
    map.dynamic_point_overlay_kdtree_.setInputCloud(
        map.dynamic_point_overlay_cloud_);
    map.dynamic_point_overlay_ready_ = !centers.empty();
  }

  static bool queryDynamicPlanningDistance(
      const GridMap &map, const Eigen::Vector3d &query,
      double max_search_distance, double &distance)
  {
    return map.getDynamicPointPlanningDistance(
        query, max_search_distance, distance, nullptr);
  }

  static void configure(GridMap &map, int parallel_threads)
  {
    map.mp_.resolution_ = 0.1;
    map.mp_.map_voxel_num_ = Eigen::Vector3i(17, 13, 9);
    map.mp_.parallel_threads_ = parallel_threads;
    map.mp_.parallel_voxel_threshold_ = 1;

    map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
    map.md_.local_bound_max_ =
        map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    map.md_.occupancy_bound_min_ = map.md_.local_bound_min_;
    map.md_.occupancy_bound_max_ = map.md_.local_bound_max_;

    const std::size_t voxel_count =
        static_cast<std::size_t>(map.mp_.map_voxel_num_.x()) *
        static_cast<std::size_t>(map.mp_.map_voxel_num_.y()) *
        static_cast<std::size_t>(map.mp_.map_voxel_num_.z());
    map.md_.occupancy_buffer_inflate_.assign(voxel_count, 0);
    map.md_.occupancy_buffer_neg_.assign(voxel_count, 0);
    map.md_.tmp_buffer1_.assign(voxel_count, 0.0);
    map.md_.tmp_buffer2_.assign(voxel_count, 0.0);
    map.md_.distance_buffer_.assign(voxel_count, 10000.0);
    map.md_.distance_buffer_neg_.assign(voxel_count, 10000.0);
    map.md_.distance_buffer_all_.assign(voxel_count, 10000.0);

    setOccupied(map, 2, 3, 1);
    setOccupied(map, 8, 7, 4);
    setOccupied(map, 15, 11, 7);
    setOccupied(map, 5, 10, 3);
  }

  static void compute(GridMap &map)
  {
    map.updateESDF3d();
  }

  static const std::vector<double> &distances(const GridMap &map)
  {
    return map.md_.distance_buffer_all_;
  }

  static void enableAsync(GridMap &map, std::uint64_t observation_seq)
  {
    map.mp_.async_esdf_enable_ = true;
    map.mp_.async_esdf_max_result_age_ = 5.0;
    map.mp_.enable_performance_diagnostics_ = false;
    map.mp_.show_esdf_time_ = false;
    map.md_.lidar_accepted_seq_ = observation_seq;
    map.md_.lidar_pending_seq_ = 0;
    map.md_.lidar_fused_seq_ = observation_seq;
    map.md_.dynamic_observation_seq_ = observation_seq;
    map.md_.esdf_observation_seq_ = 0;
    map.md_.esdf_need_update_ = true;
    map.md_.esdf_time_ = 0.0;
    map.md_.max_esdf_time_ = 0.0;
    map.md_.esdf_update_count_ = 0;
    map.md_.async_esdf_submitted_count_ = 0;
    map.md_.async_esdf_committed_count_ = 0;
    map.md_.async_esdf_rejected_count_ = 0;
    map.md_.async_esdf_occupancy_deferred_count_ = 0;
    map.md_.esdf_wall_sum_ms_ = 0.0;
    map.md_.esdf_wall_max_ms_ = 0.0;
    map.md_.esdf_commit_wall_sum_ms_ = 0.0;
    map.md_.esdf_commit_wall_max_ms_ = 0.0;
    map.md_.async_esdf_snapshot_wall_sum_ms_ = 0.0;
    map.md_.async_esdf_snapshot_wall_max_ms_ = 0.0;
    map.esdf_map_epoch_ = 1;
    map.esdf_next_request_id_ = 0;
    map.esdf_latest_submitted_request_id_ = 0;
    map.esdf_last_resolved_request_id_ = 0;
    map.esdf_committed_request_id_ = 0;
    map.async_esdf_fallback_to_sync_ = false;
    map.esdf_worker_.reset(new plan_env::ESDFWorker(
        map.mp_.parallel_threads_, map.mp_.parallel_voxel_threshold_));
  }

  static bool submitAsync(GridMap &map)
  {
    return map.submitAsyncESDFUpdate();
  }

  static bool waitForAsyncCompletionAndCommit(
      GridMap &map, const std::chrono::milliseconds &timeout)
  {
    const std::uint64_t target_request = map.esdf_next_request_id_;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      map.commitAsyncESDFResult();
      if (map.esdf_committed_request_id_ >= target_request)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  static bool waitForAsyncDecision(
      GridMap &map, const std::chrono::milliseconds &timeout)
  {
    const std::uint64_t initial_committed =
        map.md_.async_esdf_committed_count_;
    const std::uint64_t initial_rejected =
        map.md_.async_esdf_rejected_count_;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      map.commitAsyncESDFResult();
      if (map.md_.async_esdf_committed_count_ > initial_committed ||
          map.md_.async_esdf_rejected_count_ > initial_rejected)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  static bool waitForAsyncRejection(
      GridMap &map, std::uint64_t expected_rejected_count,
      const std::chrono::milliseconds &timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      map.commitAsyncESDFResult();
      if (map.md_.async_esdf_rejected_count_ >= expected_rejected_count)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  static std::uint64_t esdfObservationSeq(const GridMap &map)
  {
    return map.md_.esdf_observation_seq_;
  }

  static bool esdfNeedsUpdate(const GridMap &map)
  {
    return map.md_.esdf_need_update_;
  }

  static void setLocalBounds(GridMap &map,
                             const Eigen::Vector3i &minimum,
                             const Eigen::Vector3i &maximum)
  {
    map.md_.local_bound_min_ = minimum;
    map.md_.local_bound_max_ = maximum;
  }

  static void fillDistances(GridMap &map, double value)
  {
    std::fill(map.md_.distance_buffer_all_.begin(),
              map.md_.distance_buffer_all_.end(), value);
  }

  static std::uint64_t asyncRejectedCount(const GridMap &map)
  {
    return map.md_.async_esdf_rejected_count_;
  }

  static std::uint64_t asyncCommittedCount(const GridMap &map)
  {
    return map.md_.async_esdf_committed_count_;
  }

  static std::uint64_t deferredOccupancyCount(const GridMap &map)
  {
    return map.md_.async_esdf_occupancy_deferred_count_;
  }

  static bool asyncFallbackSelected(const GridMap &map)
  {
    return map.async_esdf_fallback_to_sync_;
  }

  static bool asyncWorkerStopped(const GridMap &map)
  {
    return map.esdf_worker_ && map.esdf_worker_->stats().stopped;
  }

  static std::uint64_t latestSubmittedRequestId(const GridMap &map)
  {
    return map.esdf_latest_submitted_request_id_;
  }

  static std::uint64_t committedRequestId(const GridMap &map)
  {
    return map.esdf_committed_request_id_;
  }

  static void injectWorkerFault(GridMap &map)
  {
    plan_env::ESDFJob invalid;
    invalid.min_index = Eigen::Vector3i::Zero();
    invalid.max_index = Eigen::Vector3i::Zero();
    invalid.resolution = 0.0;
    invalid.occupancy.assign(1, 0);
    EXPECT_FALSE(map.esdf_worker_->submit(std::move(invalid)));
  }

  static void processESDF(GridMap &map)
  {
    map.updateESDFCallback(ros::TimerEvent());
  }

  static void configureOccupancyUpdate(GridMap &map)
  {
    map.mp_.resolution_ = 0.5;
    map.mp_.resolution_inv_ = 2.0;
    map.mp_.map_origin_ = Eigen::Vector3d::Zero();
    map.mp_.map_size_ = Eigen::Vector3d(2.5, 2.5, 2.5);
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    map.mp_.map_voxel_num_ = Eigen::Vector3i(5, 5, 5);
    map.mp_.local_update_range_ = Eigen::Vector3d(1.0, 1.0, 1.0);
    map.mp_.local_bound_inflate_ = 0.0;
    map.mp_.local_map_margin_ = 0;
    map.mp_.max_ray_length_ = 3.0;
    map.mp_.obstacles_inflation_ = 0.0;
    map.mp_.virtual_ceil_height_ = -1.0;
    map.mp_.ground_height_ = 0.0;
    map.mp_.clamp_min_log_ = -2.0;
    map.mp_.clamp_max_log_ = 2.0;
    map.mp_.min_occupancy_log_ = 0.0;
    map.mp_.unknown_flag_ = 0.01;
    map.mp_.prob_hit_log_ = 2.0;
    map.mp_.prob_miss_log_ = -2.0;
    map.mp_.global_map_lidar_clear_ = false;
    map.mp_.global_map_lidar_clear_radius_ = 0.0;
    map.mp_.global_map_lidar_clear_min_misses_ = 1;
    map.mp_.live_obstacle_expiry_enable_ = true;
    map.mp_.live_obstacle_max_missed_fusions_ = 2;
    map.mp_.parallel_threads_ = 2;
    map.mp_.parallel_voxel_threshold_ = 1;
    map.mp_.use_global_map_ = false;
    map.mp_.live_lidar_health_monitor_enable_ = false;
    map.mp_.live_lidar_health_isolated_worker_ = false;
    map.mp_.live_lidar_pending_max_age_ = 0.35;
    map.mp_.live_lidar_message_max_age_ = 0.50;
    map.mp_.live_lidar_future_tolerance_ = 0.05;
    map.mp_.live_lidar_timeout_ = 0.75;
    map.mp_.live_lidar_startup_grace_ = 5.0;
    map.mp_.show_occ_time_ = false;
    map.mp_.enable_performance_diagnostics_ = false;

    const std::size_t voxel_count = 125;
    map.md_.occupancy_buffer_.assign(voxel_count,
                                     map.mp_.clamp_min_log_);
    map.md_.occupancy_buffer_inflate_.assign(voxel_count, 0);
    map.md_.static_occupancy_.assign(voxel_count, 0);
    map.md_.static_miss_count_.assign(voxel_count, 0);
    map.md_.live_occupancy_last_hit_cycle_.assign(voxel_count, 0);
    map.md_.live_occupancy_active_.assign(voxel_count, 0);
    map.md_.active_live_occupancy_indices_.clear();
    map.md_.live_fusion_cycle_ = 0;
    map.md_.expired_live_occupancy_count_ = 0;
    map.md_.count_hit_and_miss_.assign(voxel_count, 0);
    map.md_.count_hit_.assign(voxel_count, 0);
    map.md_.flag_rayend_.assign(voxel_count, -1);
    map.md_.flag_traverse_.assign(voxel_count, -1);
    map.md_.distance_buffer_all_.assign(voxel_count, 10000.0);
    map.md_.proj_points_.resize(16);
    map.md_.proj_points_cnt = 0;
    map.md_.raycast_num_ = 0;
    map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
    map.md_.local_bound_max_ =
        map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    map.md_.occupancy_bound_min_ = map.md_.local_bound_min_;
    map.md_.occupancy_bound_max_ = map.md_.local_bound_max_;
    map.md_.camera_pos_ = Eigen::Vector3d(0.75, 0.75, 0.5);
    map.md_.base_r_m_.setIdentity();
    map.md_.has_odom_ = true;
    map.md_.has_cloud_ = false;
    map.md_.occ_need_update_ = false;
    map.md_.local_updated_ = false;
    map.md_.esdf_need_update_ = false;
    map.md_.lidar_origin_valid_ = false;
    map.md_.lidar_accepted_seq_ = 0;
    map.md_.lidar_pending_seq_ = 0;
    map.md_.lidar_fused_seq_ = 0;
    map.md_.dynamic_observation_seq_ = 0;
    map.md_.last_occ_update_time_ = ros::Time(0);
    map.md_.flag_depth_odom_timeout_ = false;
    map.md_.flag_use_depth_fusion = false;
    map.md_.lidar_health_monitor_start_steady_ = ros::SteadyTime();
    map.md_.last_lidar_input_steady_ = ros::SteadyTime();
    map.lidar_health_steady_ns_.store(0, std::memory_order_relaxed);
    map.lidar_health_odom_steady_ns_.store(0, std::memory_order_relaxed);
    map.md_.last_lidar_fused_steady_ = ros::SteadyTime();
    map.md_.pending_lidar_received_steady_ = ros::SteadyTime();
    map.md_.pending_lidar_stamp_ = ros::Time(0);
    map.md_.stale_lidar_discard_count_ = 0;
    map.md_.live_lidar_timeout_active_ = false;
    map.md_.fuse_time_ = 0.0;
    map.md_.max_fuse_time_ = 0.0;
    map.md_.update_num_ = 0;
    map.md_.occupancy_update_count_ = 0;
    map.md_.async_esdf_occupancy_deferred_count_ = 0;
    map.md_.occupancy_wall_sum_ms_ = 0.0;
    map.md_.occupancy_wall_max_ms_ = 0.0;
  }

  static void configureFusedFullGlobalESDFWindow(GridMap &map)
  {
    map.mp_.use_global_map_ = true;
    map.mp_.fuse_global_map_ = true;
    map.mp_.local_esdf_follow_odom_ = true;
    map.mp_.global_map_full_esdf_ = true;
    map.mp_.live_lidar_health_monitor_enable_ = false;
    map.md_.has_first_depth_ = false;
    map.md_.has_odom_ = true;
    map.md_.has_cloud_ = true;
    map.md_.global_esdf_pending_ = false;
    map.md_.static_map_esdf_pending_ = false;
    map.md_.camera_pos_ = Eigen::Vector3d(0.75, 0.75, 0.50);
    map.updateLocalBoundsAroundRobot(false);
  }

  static void receiveOdometry(GridMap &map,
                              const Eigen::Vector3d &position)
  {
    nav_msgs::OdometryPtr odometry(new nav_msgs::Odometry());
    odometry->pose.pose.position.x = position.x();
    odometry->pose.pose.position.y = position.y();
    odometry->pose.pose.position.z = position.z();
    odometry->pose.pose.orientation.w = 1.0;
    map.odomCallback(odometry);
  }

  static Eigen::Vector3i esdfMinimum(const GridMap &map)
  {
    return map.md_.local_bound_min_;
  }

  static Eigen::Vector3i esdfMaximum(const GridMap &map)
  {
    return map.md_.local_bound_max_;
  }

  static Eigen::Vector3i occupancyMinimum(const GridMap &map)
  {
    return map.md_.occupancy_bound_min_;
  }

  static Eigen::Vector3i occupancyMaximum(const GridMap &map)
  {
    return map.md_.occupancy_bound_max_;
  }

  static void seedDynamicAndStaticOccupancy(GridMap &map)
  {
    const int dynamic = address(map, Eigen::Vector3i(1, 1, 1));
    const int inflated_only = address(map, Eigen::Vector3i(1, 2, 1));
    const int static_voxel = address(map, Eigen::Vector3i(3, 3, 3));

    map.md_.occupancy_buffer_[dynamic] = map.mp_.clamp_max_log_;
    map.md_.occupancy_buffer_inflate_[dynamic] = 1;
    map.md_.occupancy_buffer_inflate_[inflated_only] = 1;

    map.md_.occupancy_buffer_[static_voxel] = map.mp_.clamp_max_log_;
    map.md_.occupancy_buffer_inflate_[static_voxel] = 1;
    map.md_.static_occupancy_[static_voxel] = 1;
  }

  static void receiveIndependentCloud(
      GridMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud, *message);
    message->header.frame_id = "world";
    message->header.stamp = ros::Time::now();
    map.cloudCallback(message);
  }

  static void receiveSynchronizedCloud(
      GridMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud, *message);
    message->header.frame_id = "test_lidar";
    message->header.stamp = ros::Time::now();

    map.tf_buffer_.reset(new tf2_ros::Buffer());
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = message->header.frame_id;
    transform.transform.translation.x = 0.75;
    transform.transform.translation.y = 0.75;
    transform.transform.translation.z = 0.50;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(map.tf_buffer_->setTransform(transform, "grid_map_test", true));

    nav_msgs::OdometryPtr odometry(new nav_msgs::Odometry());
    odometry->header.stamp = message->header.stamp;
    odometry->pose.pose.position.x = 0.75;
    odometry->pose.pose.position.y = 0.75;
    odometry->pose.pose.position.z = 0.50;
    odometry->pose.pose.orientation.w = 1.0;
    map.cloudOdomCallback(message, odometry);
  }

  static void processOccupancy(GridMap &map)
  {
    map.updateOccupancyCallback(ros::TimerEvent());
  }

  static void invalidateLocalBounds(GridMap &map)
  {
    map.md_.local_bound_min_ = Eigen::Vector3i(4, 0, 0);
    map.md_.local_bound_max_ = Eigen::Vector3i(2, 4, 4);
  }

  static bool occupancyPending(const GridMap &map)
  {
    return map.md_.occ_need_update_;
  }

  static bool lidarObservationPending(const GridMap &map)
  {
    return map.md_.lidar_origin_valid_;
  }

  static int projectedPointCount(const GridMap &map)
  {
    return map.md_.proj_points_cnt;
  }

  static std::uint64_t dynamicObservationSeq(const GridMap &map)
  {
    return map.md_.dynamic_observation_seq_;
  }

  static std::uint64_t lidarAcceptedSeq(const GridMap &map)
  {
    return map.md_.lidar_accepted_seq_;
  }

  static std::uint64_t lidarPendingSeq(const GridMap &map)
  {
    return map.md_.lidar_pending_seq_;
  }

  static std::uint64_t lidarFusedSeq(const GridMap &map)
  {
    return map.md_.lidar_fused_seq_;
  }

  static std::uint64_t staleLidarDiscardCount(const GridMap &map)
  {
    return map.md_.stale_lidar_discard_count_;
  }

  static void agePendingLidar(GridMap &map, double seconds)
  {
    map.md_.pending_lidar_received_steady_ =
        ros::SteadyTime::now() - ros::WallDuration(seconds);
  }

  static void agePendingLidarMessageStamp(GridMap &map, double seconds)
  {
    map.md_.pending_lidar_stamp_ =
        ros::Time::now() - ros::Duration(seconds);
  }

  static void seedNonLidarObservation(GridMap &map,
                                      const Eigen::Vector3d &point)
  {
    map.md_.proj_points_[0] = point;
    map.md_.proj_points_cnt = 1;
    map.md_.lidar_origin_valid_ = false;
    map.md_.lidar_pending_seq_ = 0;
    map.md_.occ_need_update_ = true;
    map.md_.flag_use_depth_fusion = true;
  }

  static void setPendingLidarGeneration(GridMap &map,
                                        std::uint64_t generation)
  {
    map.md_.lidar_accepted_seq_ = generation;
    map.md_.lidar_pending_seq_ = generation;
    map.md_.lidar_origin_valid_ = true;
    map.md_.occ_need_update_ = true;
    map.md_.pending_lidar_received_steady_ = ros::SteadyTime::now();
  }

  static void setFusedLidarGeneration(GridMap &map,
                                      std::uint64_t generation)
  {
    map.md_.lidar_accepted_seq_ = generation;
    map.md_.lidar_pending_seq_ = 0;
    map.md_.lidar_fused_seq_ = generation;
    map.md_.dynamic_observation_seq_ = generation;
    map.md_.lidar_origin_valid_ = false;
    map.md_.occ_need_update_ = false;
    map.md_.last_lidar_fused_steady_ = ros::SteadyTime::now();
  }

  static void configureLiveLidarHealth(GridMap &map,
                                       double startup_age_seconds,
                                       bool use_global_map)
  {
    map.mp_.live_lidar_health_monitor_enable_ = true;
    map.mp_.live_lidar_pending_max_age_ = 0.35;
    map.mp_.live_lidar_timeout_ = 0.75;
    map.mp_.live_lidar_startup_grace_ = 1.0;
    map.mp_.use_global_map_ = use_global_map;
    map.mp_.fuse_global_map_ = true;
    // Health tests isolate the live-input watchdog after static-map startup;
    // never let an uninitialized static readiness flag decide their result.
    map.md_.static_map_esdf_ready_ = use_global_map;
    map.md_.has_odom_ = true;
    map.md_.lidar_health_monitor_start_steady_ =
        ros::SteadyTime::now() -
        ros::WallDuration(startup_age_seconds);
  }

  static void ageHealthyLidarInput(GridMap &map, double seconds)
  {
    map.md_.last_lidar_input_steady_ =
        ros::SteadyTime::now() - ros::WallDuration(seconds);
  }

  static void ageIsolatedLidarHealth(GridMap &map, double seconds)
  {
    const ros::SteadyTime sample_time =
        ros::SteadyTime::now() - ros::WallDuration(seconds);
    map.lidar_health_steady_ns_.store(
        sample_time.toNSec(), std::memory_order_release);
  }

  static bool hasIsolatedLidarHealth(const GridMap &map)
  {
    return map.lidar_health_steady_ns_.load(std::memory_order_acquire) != 0;
  }

  static void receiveIsolatedHealthOdometry(GridMap &map)
  {
    nav_msgs::OdometryPtr odometry(new nav_msgs::Odometry());
    odometry->header.stamp = ros::Time::now();
    odometry->pose.pose.orientation.w = 1.0;
    map.liveLidarHealthOdomCallback(odometry);
  }

  static void receiveIsolatedHealthCloud(
      GridMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud)
  {
    sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud, *message);
    message->header.frame_id = "unavailable_test_lidar_frame";
    message->header.stamp = ros::Time::now();
    map.liveLidarHealthCallback(message);
  }

  static bool hasHealthyLidarInput(const GridMap &map)
  {
    return !map.md_.last_lidar_input_steady_.isZero();
  }

  static bool mapReadyForPlanning(const GridMap &map)
  {
    return map.mapReadyForPlanning();
  }

  static bool sensorTimeoutNow(const GridMap &map)
  {
    return map.getOdomDepthTimeout();
  }

  static bool sensorTimeout(const GridMap &map)
  {
    return map.md_.flag_depth_odom_timeout_;
  }

  static double rawOccupancy(const GridMap &map,
                             const Eigen::Vector3i &index)
  {
    return map.md_.occupancy_buffer_[address(map, index)];
  }

  static int inflatedOccupancy(const GridMap &map,
                               const Eigen::Vector3i &index)
  {
    return map.md_.occupancy_buffer_inflate_[address(map, index)];
  }

  static std::size_t advanceLiveExpiryCycle(GridMap &map)
  {
    ++map.md_.live_fusion_cycle_;
    return map.expireStaleLiveOccupancy(map.md_.live_fusion_cycle_);
  }

  static void raycastProjectedLivePoint(GridMap &map,
                                        const Eigen::Vector3d &point)
  {
    map.md_.proj_points_[0] = point;
    map.md_.proj_points_cnt = 1;
    map.md_.lidar_origin_valid_ = false;
    map.raycastProcess(true);
  }

  static std::uint32_t liveFusionCycle(const GridMap &map)
  {
    return map.md_.live_fusion_cycle_;
  }

private:
  static int address(const GridMap &map, const Eigen::Vector3i &index)
  {
    return index.x() * map.mp_.map_voxel_num_.y() *
                           map.mp_.map_voxel_num_.z() +
           index.y() * map.mp_.map_voxel_num_.z() + index.z();
  }

  static void setOccupied(GridMap &map, int x, int y, int z)
  {
    const int address = x * map.mp_.map_voxel_num_.y() *
                            map.mp_.map_voxel_num_.z() +
                        y * map.mp_.map_voxel_num_.z() + z;
    map.md_.occupancy_buffer_inflate_[address] = 1;
  }
};

TEST(GridMapParallel, TimeVaryingRadiusSelectsMinimumClearanceNotNearestCenter)
{
  GridMap map;
  GridMapParallelTestAccessor::configureDynamicPlanningOverlay(
      map,
      {Eigen::Vector3d(0.20, 0.0, 0.0),
       Eigen::Vector3d(0.30, 0.0, 0.0)},
      {0.05f, 0.25f});

  double distance = 0.0;
  ASSERT_TRUE(GridMapParallelTestAccessor::queryDynamicPlanningDistance(
      map, Eigen::Vector3d::Zero(), 1.0, distance));
  // The nearest centre has 0.15 m clearance. The later, larger prediction
  // sphere has only 0.05 m and must therefore own the collision distance.
  EXPECT_NEAR(0.05, distance, 1e-6);
}

namespace
{

void ensureRosInitialized()
{
  if (ros::isInitialized())
    return;
  int argc = 0;
  char **argv = nullptr;
  ros::init(argc, argv, "test_grid_map_parallel",
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler |
                ros::init_options::NoRosout);
}

}  // namespace

TEST(GridMapParallel, MatchesSingleThreadDistanceField)
{
  ensureRosInitialized();
  GridMap serial_map;
  GridMap parallel_map;
  GridMapParallelTestAccessor::configure(serial_map, 1);
  GridMapParallelTestAccessor::configure(parallel_map, 3);

  GridMapParallelTestAccessor::compute(serial_map);
  GridMapParallelTestAccessor::compute(parallel_map);

  const auto &serial = GridMapParallelTestAccessor::distances(serial_map);
  const auto &parallel = GridMapParallelTestAccessor::distances(parallel_map);
  ASSERT_EQ(serial.size(), parallel.size());
  for (std::size_t index = 0; index < serial.size(); ++index)
  {
    ASSERT_TRUE(std::isfinite(serial[index])) << "voxel " << index;
    ASSERT_DOUBLE_EQ(serial[index], parallel[index]) << "voxel " << index;
  }
}

TEST(GridMapParallel, AsyncSnapshotCommitMatchesSynchronousGeneration)
{
  ensureRosInitialized();
  GridMap reference_map;
  GridMap async_map;
  GridMapParallelTestAccessor::configure(reference_map, 1);
  GridMapParallelTestAccessor::configure(async_map, 3);
  GridMapParallelTestAccessor::compute(reference_map);
  const std::vector<double> expected =
      GridMapParallelTestAccessor::distances(reference_map);

  const std::uint64_t captured_sequence = 42;
  GridMapParallelTestAccessor::enableAsync(async_map, captured_sequence);
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(async_map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::latestSubmittedRequestId(async_map));
  EXPECT_FALSE(GridMapParallelTestAccessor::esdfNeedsUpdate(async_map));
  EXPECT_EQ(0U,
            GridMapParallelTestAccessor::esdfObservationSeq(async_map));
  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncCompletionAndCommit(
      async_map, std::chrono::milliseconds(5000)));

  EXPECT_EQ(captured_sequence,
            GridMapParallelTestAccessor::esdfObservationSeq(async_map));
  const auto &actual = GridMapParallelTestAccessor::distances(async_map);
  ASSERT_EQ(expected.size(), actual.size());
  for (std::size_t index = 0; index < expected.size(); ++index)
    ASSERT_DOUBLE_EQ(expected[index], actual[index]) << "voxel " << index;
}

TEST(GridMapParallel, SameGenerationSecondRequestSupersedesFirstRequest)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configure(map, 3);
  GridMapParallelTestAccessor::enableAsync(map, 73);

  // Both snapshots intentionally have the same occupancy revision and bounds.
  // Request identity must still prevent the first calculation/result from
  // satisfying the second request's commit boundary.
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(map));
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(map));
  EXPECT_EQ(2U, GridMapParallelTestAccessor::latestSubmittedRequestId(map));

  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncCompletionAndCommit(
      map, std::chrono::milliseconds(5000)));
  EXPECT_EQ(2U, GridMapParallelTestAccessor::committedRequestId(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::asyncCommittedCount(map));
  EXPECT_EQ(73U, GridMapParallelTestAccessor::esdfObservationSeq(map));
}

TEST(GridMapParallel, PendingLidarDoesNotInvalidateCurrentOccupancyResult)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configure(map, 3);
  GridMapParallelTestAccessor::fillDistances(map, 3141.0);
  const std::vector<double> original =
      GridMapParallelTestAccessor::distances(map);
  GridMapParallelTestAccessor::enableAsync(map, 42);
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(map));

  GridMapParallelTestAccessor::setPendingLidarGeneration(map, 43);
  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncCompletionAndCommit(
      map, std::chrono::milliseconds(5000)));

  // The pending scan has not mutated occupancy yet. The result therefore
  // exactly represents the current fused map and may commit; the occupancy
  // callback will consume the queued scan only after this request resolves.
  EXPECT_EQ(1U, GridMapParallelTestAccessor::asyncCommittedCount(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::asyncRejectedCount(map));
  EXPECT_EQ(42U, GridMapParallelTestAccessor::esdfObservationSeq(map));
  EXPECT_NE(original, GridMapParallelTestAccessor::distances(map));
}

TEST(GridMapParallel, NewerFusedGenerationRejectsOlderAsyncResult)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configure(map, 3);
  GridMapParallelTestAccessor::fillDistances(map, 2718.0);
  const std::vector<double> original =
      GridMapParallelTestAccessor::distances(map);
  GridMapParallelTestAccessor::enableAsync(map, 42);
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(map));

  GridMapParallelTestAccessor::setFusedLidarGeneration(map, 43);
  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncDecision(
      map, std::chrono::milliseconds(5000)));

  EXPECT_EQ(0U, GridMapParallelTestAccessor::asyncCommittedCount(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::asyncRejectedCount(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::esdfObservationSeq(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(original, GridMapParallelTestAccessor::distances(map));
}

TEST(GridMapParallel,
     MovedBoundsRejectHistoricalResultBeforeCopy)
{
  ensureRosInitialized();
  GridMap async_map;
  GridMapParallelTestAccessor::configure(async_map, 3);

  const Eigen::Vector3i historical_min(1, 2, 1);
  const Eigen::Vector3i historical_max(9, 8, 6);
  const Eigen::Vector3i current_min(7, 4, 2);
  const Eigen::Vector3i current_max(15, 11, 7);
  const double untouched_sentinel = 4321.0;
  GridMapParallelTestAccessor::setLocalBounds(
      async_map, historical_min, historical_max);
  GridMapParallelTestAccessor::fillDistances(
      async_map, untouched_sentinel);
  const std::vector<double> untouched_distances =
      GridMapParallelTestAccessor::distances(async_map);

  const std::uint64_t captured_sequence = 42;
  GridMapParallelTestAccessor::enableAsync(async_map, captured_sequence);
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(async_map));

  // Move the planning window after capture but before main-queue commit. The
  // worker may already have completed; comparison happens only at commit.
  GridMapParallelTestAccessor::setLocalBounds(
      async_map, current_min, current_max);
  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncRejection(
      async_map, 1U, std::chrono::milliseconds(5000)));

  EXPECT_EQ(0U,
            GridMapParallelTestAccessor::esdfObservationSeq(async_map));
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(async_map));
  EXPECT_EQ(0U,
            GridMapParallelTestAccessor::asyncCommittedCount(async_map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::asyncRejectedCount(async_map));
  EXPECT_EQ(untouched_distances,
            GridMapParallelTestAccessor::distances(async_map));

  // A subsequent snapshot over the current bounds is allowed to satisfy the
  // same observation generation and clear the freshness request.
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(async_map));
  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncCompletionAndCommit(
      async_map, std::chrono::milliseconds(5000)));
  EXPECT_EQ(captured_sequence,
            GridMapParallelTestAccessor::esdfObservationSeq(async_map));
  EXPECT_FALSE(GridMapParallelTestAccessor::esdfNeedsUpdate(async_map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::asyncCommittedCount(async_map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::asyncRejectedCount(async_map));
}

TEST(GridMapParallel,
     FusedFullMapOdometryCannotInvalidateAsyncESDFWindow)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configure(map, 3);
  GridMapParallelTestAccessor::enableAsync(map, 84);
  GridMapParallelTestAccessor::configureFusedFullGlobalESDFWindow(map);

  const Eigen::Vector3i expected_minimum = Eigen::Vector3i::Zero();
  const Eigen::Vector3i expected_maximum(16, 12, 8);
  ASSERT_EQ(expected_minimum,
            GridMapParallelTestAccessor::esdfMinimum(map));
  ASSERT_EQ(expected_maximum,
            GridMapParallelTestAccessor::esdfMaximum(map));
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(map));

  // This used to replace a ray-tight async snapshot with an odometry-centred
  // (effectively full-map) window before commit.  In full-global mode the ESDF
  // window is now invariant, so the captured generation remains committable.
  GridMapParallelTestAccessor::receiveOdometry(
      map, Eigen::Vector3d(7.5, 5.0, 1.0));
  EXPECT_EQ(expected_minimum,
            GridMapParallelTestAccessor::esdfMinimum(map));
  EXPECT_EQ(expected_maximum,
            GridMapParallelTestAccessor::esdfMaximum(map));

  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncCompletionAndCommit(
      map, std::chrono::milliseconds(5000)));
  EXPECT_EQ(84U,
            GridMapParallelTestAccessor::esdfObservationSeq(map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::asyncCommittedCount(map));
  EXPECT_EQ(0U,
            GridMapParallelTestAccessor::asyncRejectedCount(map));
}

TEST(GridMapParallel, InvalidESDFBoundsFailClosedWithoutWorkerJoin)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configure(map, 3);
  GridMapParallelTestAccessor::fillDistances(map, 1234.0);
  const std::vector<double> original =
      GridMapParallelTestAccessor::distances(map);
  GridMapParallelTestAccessor::enableAsync(map, 51);
  GridMapParallelTestAccessor::setLocalBounds(
      map, Eigen::Vector3i(8, 2, 1), Eigen::Vector3i(7, 8, 6));

  GridMapParallelTestAccessor::processESDF(map);

  EXPECT_TRUE(GridMapParallelTestAccessor::asyncFallbackSelected(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::asyncWorkerStopped(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::esdfObservationSeq(map));
  EXPECT_EQ(original, GridMapParallelTestAccessor::distances(map));
}

TEST(GridMapParallel, WorkerFaultRequestsDeferredFallbackWithoutWorkerJoin)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configure(map, 3);
  GridMapParallelTestAccessor::fillDistances(map, 2468.0);
  const std::vector<double> original =
      GridMapParallelTestAccessor::distances(map);
  GridMapParallelTestAccessor::enableAsync(map, 52);
  GridMapParallelTestAccessor::injectWorkerFault(map);

  GridMapParallelTestAccessor::processESDF(map);

  EXPECT_TRUE(GridMapParallelTestAccessor::asyncFallbackSelected(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::asyncWorkerStopped(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::esdfObservationSeq(map));
  EXPECT_EQ(original, GridMapParallelTestAccessor::distances(map));
}

TEST(GridMapOccupancy, FilteredEmptyLidarPreservesPreviousGeneration)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);

  pcl::PointCloud<pcl::PointXYZ> ground_only_cloud;
  ground_only_cloud.push_back(pcl::PointXYZ(1.25f, 0.75f, 0.10f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map,
                                                       ground_only_cloud);

  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::lidarObservationPending(map));
  EXPECT_EQ(0, GridMapParallelTestAccessor::projectedPointCount(map));

  GridMapParallelTestAccessor::processOccupancy(map);

  const Eigen::Vector3i dynamic(1, 1, 1);
  const Eigen::Vector3i inflated_only(1, 2, 1);
  const Eigen::Vector3i static_voxel(3, 3, 3);
  EXPECT_DOUBLE_EQ(2.0,
                   GridMapParallelTestAccessor::rawOccupancy(map, dynamic));
  EXPECT_EQ(1, GridMapParallelTestAccessor::inflatedOccupancy(map, dynamic));
  EXPECT_EQ(1,
            GridMapParallelTestAccessor::inflatedOccupancy(map,
                                                           inflated_only));
  EXPECT_DOUBLE_EQ(
      2.0, GridMapParallelTestAccessor::rawOccupancy(map, static_voxel));
  EXPECT_EQ(1,
            GridMapParallelTestAccessor::inflatedOccupancy(map,
                                                           static_voxel));
  EXPECT_FALSE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::lidarObservationPending(map));
}

TEST(GridMapOccupancy, FilteredEmptyLidarDoesNotReplacePendingValidScan)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  ASSERT_TRUE(GridMapParallelTestAccessor::occupancyPending(map));
  ASSERT_TRUE(GridMapParallelTestAccessor::lidarObservationPending(map));
  ASSERT_EQ(1, GridMapParallelTestAccessor::projectedPointCount(map));

  pcl::PointCloud<pcl::PointXYZ> ground_only_cloud;
  ground_only_cloud.push_back(pcl::PointXYZ(1.25f, 0.75f, 0.10f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map,
                                                       ground_only_cloud);

  EXPECT_TRUE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::lidarObservationPending(map));
  EXPECT_EQ(1, GridMapParallelTestAccessor::projectedPointCount(map));

  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
}

TEST(GridMapOccupancy, CoalescedScanPublishesNewestAcceptedGeneration)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(1U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  ASSERT_EQ(1U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  ASSERT_EQ(0U, GridMapParallelTestAccessor::lidarFusedSeq(map));

  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(2U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  ASSERT_EQ(2U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  ASSERT_EQ(0U, GridMapParallelTestAccessor::lidarFusedSeq(map));

  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_EQ(2U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  EXPECT_EQ(2U, GridMapParallelTestAccessor::lidarFusedSeq(map));
  // Two accepted scans were coalesced into one actual occupancy mutation.
  EXPECT_EQ(1U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
}

TEST(GridMapOccupancy, StaleLiveEndpointExpiresAfterConfiguredMisses)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);

  pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
  obstacle_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, obstacle_cloud);
  GridMapParallelTestAccessor::processOccupancy(map);

  const Eigen::Vector3i obstacle_voxel(3, 3, 1);
  ASSERT_DOUBLE_EQ(
      2.0,
      GridMapParallelTestAccessor::rawOccupancy(map, obstacle_voxel));

  // Emulate successive valid fusion boundaries whose endpoints do not touch
  // the old voxel. One miss remains inside the configured two-fusion hold.
  EXPECT_EQ(0U,
            GridMapParallelTestAccessor::advanceLiveExpiryCycle(map));
  EXPECT_DOUBLE_EQ(
      2.0,
      GridMapParallelTestAccessor::rawOccupancy(map, obstacle_voxel));

  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::advanceLiveExpiryCycle(map));
  EXPECT_LT(GridMapParallelTestAccessor::rawOccupancy(map, obstacle_voxel),
            0.0);
}

TEST(GridMapOccupancy, ReobservedLiveEndpointDoesNotExpire)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);

  pcl::PointCloud<pcl::PointXYZ> obstacle_cloud;
  obstacle_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  for (int scan = 0; scan < 4; ++scan)
  {
    GridMapParallelTestAccessor::receiveIndependentCloud(map,
                                                         obstacle_cloud);
    GridMapParallelTestAccessor::processOccupancy(map);
  }

  EXPECT_DOUBLE_EQ(
      2.0,
      GridMapParallelTestAccessor::rawOccupancy(
          map, Eigen::Vector3i(3, 3, 1)));
}

TEST(GridMapOccupancy, RaycastFilteredScanDoesNotAgeLiveOccupancy)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);

  // This bypasses the callback-level filter to exercise raycastProcess's
  // fail-closed second line of defense. A frame with no point above the
  // raycast ground threshold is not a successfully fused observation.
  GridMapParallelTestAccessor::raycastProjectedLivePoint(
      map, Eigen::Vector3d(1.25, 0.75, 0.10));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::liveFusionCycle(map));

  GridMapParallelTestAccessor::raycastProjectedLivePoint(
      map, Eigen::Vector3d(1.50, 1.50, 0.50));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::liveFusionCycle(map));

  GridMapParallelTestAccessor::raycastProjectedLivePoint(
      map, Eigen::Vector3d(1.25, 0.75, 0.10));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::liveFusionCycle(map));
}

TEST(GridMapOccupancy, InFlightESDFDefersOccupancyUntilResultCommits)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::enableAsync(map, 7);
  ASSERT_TRUE(GridMapParallelTestAccessor::submitAsync(map));

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(8U, GridMapParallelTestAccessor::lidarPendingSeq(map));

  GridMapParallelTestAccessor::processOccupancy(map);

  // No fused-map mutation may invalidate the immutable request. The newest
  // sensor frame remains pending and replaces older frames in the same slot.
  EXPECT_EQ(7U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_EQ(7U, GridMapParallelTestAccessor::lidarFusedSeq(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::lidarObservationPending(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::deferredOccupancyCount(map));

  ASSERT_TRUE(GridMapParallelTestAccessor::waitForAsyncCompletionAndCommit(
      map, std::chrono::milliseconds(5000)));
  EXPECT_EQ(7U, GridMapParallelTestAccessor::esdfObservationSeq(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::asyncCommittedCount(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::asyncRejectedCount(map));

  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_EQ(8U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_EQ(8U, GridMapParallelTestAccessor::lidarFusedSeq(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::lidarObservationPending(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
}

TEST(GridMapOccupancy, StalePendingLidarIsDiscardedBeforeFusion)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(1U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  GridMapParallelTestAccessor::agePendingLidar(map, 1.0);

  GridMapParallelTestAccessor::processOccupancy(map);

  EXPECT_EQ(1U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarFusedSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::staleLidarDiscardCount(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::lidarObservationPending(map));
  EXPECT_DOUBLE_EQ(
      2.0, GridMapParallelTestAccessor::rawOccupancy(
               map, Eigen::Vector3i(1, 1, 1)));
}

TEST(GridMapOccupancy, StalePendingMessageStampIsRecheckedBeforeFusion)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(1U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  // Keep the steady queue age fresh. Only the original transport timestamp is
  // made stale, proving that callback-age and queue-age limits are both
  // enforced at the fusion boundary.
  GridMapParallelTestAccessor::agePendingLidarMessageStamp(map, 1.0);

  GridMapParallelTestAccessor::processOccupancy(map);

  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarFusedSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_EQ(1U, GridMapParallelTestAccessor::staleLidarDiscardCount(map));
  EXPECT_DOUBLE_EQ(
      2.0, GridMapParallelTestAccessor::rawOccupancy(
               map, Eigen::Vector3i(1, 1, 1)));
}

TEST(GridMapOccupancy, NonLidarMutationAdvancesCoherentRevision)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedNonLidarObservation(
      map, Eigen::Vector3d(1.50, 1.50, 0.50));

  GridMapParallelTestAccessor::processOccupancy(map);

  EXPECT_EQ(1U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarFusedSeq(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
}

TEST(GridMapOccupancy, LiveLidarStartupGraceAvoidsImmediateTimeout)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 0.0, true);

  GridMapParallelTestAccessor::processOccupancy(map);

  EXPECT_FALSE(GridMapParallelTestAccessor::sensorTimeout(map));
}

TEST(GridMapOccupancy, StaticGlobalMapDoesNotMaskLiveLidarTimeout)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 2.0, true);

  GridMapParallelTestAccessor::processOccupancy(map);

  EXPECT_TRUE(GridMapParallelTestAccessor::sensorTimeout(map));
}

TEST(GridMapOccupancy, FilteredFiniteLidarRefreshesHealthWithoutMutation)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 2.0, false);

  GridMapParallelTestAccessor::processOccupancy(map);
  ASSERT_TRUE(GridMapParallelTestAccessor::sensorTimeout(map));

  pcl::PointCloud<pcl::PointXYZ> ground_only_cloud;
  ground_only_cloud.push_back(pcl::PointXYZ(1.25f, 0.75f, 0.10f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map,
                                                       ground_only_cloud);
  ASSERT_TRUE(GridMapParallelTestAccessor::hasHealthyLidarInput(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));

  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_FALSE(GridMapParallelTestAccessor::sensorTimeout(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
}

TEST(GridMapOccupancy,
     SynchronizedFilteredLidarRefreshesHealthWithoutMutation)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  // exp0 uses a static global PCD plus the synchronized cloud/odometry live
  // path, so exercise that callback rather than only the simulation helper.
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 2.0, true);
  GridMapParallelTestAccessor::processOccupancy(map);
  ASSERT_TRUE(GridMapParallelTestAccessor::sensorTimeout(map));

  pcl::PointCloud<pcl::PointXYZ> ground_only_cloud;
  // The test transform places this finite return at world z=0.10, below the
  // obstacle filter. It is a health heartbeat, not an occupancy observation.
  ground_only_cloud.push_back(pcl::PointXYZ(0.50f, 0.0f, -0.40f));
  GridMapParallelTestAccessor::receiveSynchronizedCloud(
      map, ground_only_cloud);

  EXPECT_TRUE(GridMapParallelTestAccessor::hasHealthyLidarInput(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarPendingSeq(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));

  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_FALSE(GridMapParallelTestAccessor::sensorTimeout(map));
}

TEST(GridMapOccupancy, PlanningReadinessChecksLiveAgeWithoutTimerOrdering)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 0.0, false);

  // Startup grace permits the watchdog state, but planning remains closed
  // until at least one valid finite LiDAR input has actually arrived.
  EXPECT_FALSE(GridMapParallelTestAccessor::sensorTimeoutNow(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::mapReadyForPlanning(map));

  GridMapParallelTestAccessor::ageHealthyLidarInput(map, 0.0);
  EXPECT_TRUE(GridMapParallelTestAccessor::mapReadyForPlanning(map));

  // Do not run updateOccupancyCallback here: the accessor must detect stale
  // input directly, independent of the 20 Hz cached watchdog timer ordering.
  GridMapParallelTestAccessor::ageHealthyLidarInput(map, 1.0);
  EXPECT_TRUE(GridMapParallelTestAccessor::sensorTimeoutNow(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::mapReadyForPlanning(map));
}

TEST(GridMapOccupancy,
     FreshIsolatedHealthPreventsFalseTimeoutDuringPlannerQueueStall)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 2.0, true);

  // The map callback is older than the short watchdog deadline because the
  // serialized planner queue is busy, while the independent transport queue
  // continues to observe current valid LiDAR packets.
  GridMapParallelTestAccessor::ageHealthyLidarInput(map, 1.0);
  GridMapParallelTestAccessor::ageIsolatedLidarHealth(map, 0.0);

  EXPECT_FALSE(GridMapParallelTestAccessor::sensorTimeoutNow(map));
  EXPECT_TRUE(GridMapParallelTestAccessor::mapReadyForPlanning(map));
}

TEST(GridMapOccupancy,
     StaleIsolatedHealthCannotMaskRealLidarTimeout)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 2.0, true);

  GridMapParallelTestAccessor::ageHealthyLidarInput(map, 1.0);
  GridMapParallelTestAccessor::ageIsolatedLidarHealth(map, 1.0);

  EXPECT_TRUE(GridMapParallelTestAccessor::sensorTimeoutNow(map));
  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_TRUE(GridMapParallelTestAccessor::sensorTimeout(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::mapReadyForPlanning(map));
}

TEST(GridMapOccupancy,
     IsolatedHealthRequiresTransformOrValidOdometryFallback)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureLiveLidarHealth(map, 2.0, true);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.0f, 0.0f, 0.5f));

  // There is no TF buffer/transform in this test and no isolated odometry
  // heartbeat yet, so raw packets alone must not satisfy the watchdog.
  GridMapParallelTestAccessor::receiveIsolatedHealthCloud(map, valid_cloud);
  EXPECT_FALSE(
      GridMapParallelTestAccessor::hasIsolatedLidarHealth(map));

  GridMapParallelTestAccessor::receiveIsolatedHealthOdometry(map);
  GridMapParallelTestAccessor::receiveIsolatedHealthCloud(map, valid_cloud);
  EXPECT_TRUE(
      GridMapParallelTestAccessor::hasIsolatedLidarHealth(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::sensorTimeoutNow(map));
}

TEST(GridMapOccupancy, RawEmptyLidarIsNotAccepted)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);

  const pcl::PointCloud<pcl::PointXYZ> empty_cloud;
  GridMapParallelTestAccessor::receiveIndependentCloud(map, empty_cloud);
  GridMapParallelTestAccessor::processOccupancy(map);

  const Eigen::Vector3i dynamic(1, 1, 1);
  EXPECT_DOUBLE_EQ(2.0,
                   GridMapParallelTestAccessor::rawOccupancy(map, dynamic));
  EXPECT_EQ(1, GridMapParallelTestAccessor::inflatedOccupancy(map, dynamic));
  EXPECT_FALSE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
}

TEST(GridMapOccupancy,
     FusedFullMapRaycastKeepsIndependentESDFWindow)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::configureFusedFullGlobalESDFWindow(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(0.75f, 0.0f, 0.0f));
  GridMapParallelTestAccessor::receiveSynchronizedCloud(map, valid_cloud);
  GridMapParallelTestAccessor::processOccupancy(map);

  const Eigen::Vector3i full_minimum = Eigen::Vector3i::Zero();
  const Eigen::Vector3i full_maximum(4, 4, 4);
  EXPECT_EQ(full_minimum,
            GridMapParallelTestAccessor::esdfMinimum(map));
  EXPECT_EQ(full_maximum,
            GridMapParallelTestAccessor::esdfMaximum(map));

  // The ray only dirtied the sensor-to-return corridor.  Keeping this compact
  // region separate preserves occupancy performance without changing the
  // full ESDF generation which an async worker captured.
  EXPECT_NE(full_minimum,
            GridMapParallelTestAccessor::occupancyMinimum(map));
  EXPECT_NE(full_maximum,
            GridMapParallelTestAccessor::occupancyMaximum(map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::dynamicObservationSeq(map));
}

TEST(GridMapOccupancy,
     LiveOnlyRaycastStillOwnsESDFWindow)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map, valid_cloud);
  GridMapParallelTestAccessor::processOccupancy(map);

  // Preserve the native/baseline local-map policy: when no static global map
  // is fused, the current sensor corridor is both the occupancy dirty region
  // and the ESDF compute window.
  EXPECT_EQ(GridMapParallelTestAccessor::occupancyMinimum(map),
            GridMapParallelTestAccessor::esdfMinimum(map));
  EXPECT_EQ(GridMapParallelTestAccessor::occupancyMaximum(map),
            GridMapParallelTestAccessor::esdfMaximum(map));
  EXPECT_EQ(1U,
            GridMapParallelTestAccessor::dynamicObservationSeq(map));
}

TEST(GridMapOccupancy, InvalidBoundsFailClosedWithoutAdvancingGeneration)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  GridMapParallelTestAccessor::receiveIndependentCloud(map,
                                                       valid_cloud);
  ASSERT_TRUE(GridMapParallelTestAccessor::lidarObservationPending(map));
  GridMapParallelTestAccessor::invalidateLocalBounds(map);

  GridMapParallelTestAccessor::processOccupancy(map);

  const Eigen::Vector3i dynamic(1, 1, 1);
  EXPECT_DOUBLE_EQ(2.0,
                   GridMapParallelTestAccessor::rawOccupancy(map, dynamic));
  EXPECT_EQ(1, GridMapParallelTestAccessor::inflatedOccupancy(map, dynamic));
  EXPECT_FALSE(GridMapParallelTestAccessor::esdfNeedsUpdate(map));
  EXPECT_EQ(0U, GridMapParallelTestAccessor::dynamicObservationSeq(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::lidarObservationPending(map));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_grid_map_parallel",
            ros::init_options::AnonymousName |
                ros::init_options::NoSigintHandler |
                ros::init_options::NoRosout);
  return RUN_ALL_TESTS();
}

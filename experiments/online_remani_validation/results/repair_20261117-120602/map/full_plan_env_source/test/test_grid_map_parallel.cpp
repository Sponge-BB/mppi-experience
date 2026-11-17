#include <gtest/gtest.h>

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include <plan_env/grid_map.h>

#ifndef MAP_BENCHMARK_OLD_ABI
namespace plan_env
{
// Alter only a completed private result, while holding the worker's own lock.
// This makes whole-batch rejection deterministic without a product test hook.
class ESDFWorkerTestAccessor
{
public:
  static std::pair<std::size_t, std::size_t> readyPatchStats(ESDFWorker &worker)
  {
    std::lock_guard<std::mutex> lock(worker.queue_mutex_);
    if (!worker.ready_result_) return {0, 0};
    return {worker.ready_result_->patches.size(),
            worker.ready_result_->computed_voxels};
  }

  static bool damageLastReadyPatch(ESDFWorker &worker, bool remove_patch)
  {
    std::lock_guard<std::mutex> lock(worker.queue_mutex_);
    if (!worker.ready_result_ || worker.ready_result_->patches.size() < 2)
      return false;
    auto &patches = worker.ready_result_->patches;
    if (remove_patch)
      patches.pop_back();
    else
    {
      if (patches.back().distance_all.empty()) return false;
      patches.back().distance_all.back() = std::numeric_limits<double>::quiet_NaN();
    }
    return true;
  }
};
}  // namespace plan_env
#endif

class GridMapParallelTestAccessor
{
public:
  static void configureImmutableReads(GridMap &map)
  {
    configureOccupancyUpdate(map);
    map.isolated_mapping_ = true;
    map.md_.esdf_observation_seq_ = 0;
    map.md_.static_map_esdf_ready_ = true;
    map.md_.has_first_depth_ = false;
    map.esdf_map_epoch_ = 1;
    publishGeneration(map, 1, 1, -2.0, 0.75);
  }

  // The test owns one writer, exactly as the private mapping queue does.
  static void publishGeneration(GridMap &map, std::uint64_t occupancy_seq,
                                std::uint64_t esdf_seq, double occupancy,
                                double distance)
  {
    std::fill(map.md_.occupancy_buffer_.begin(),
              map.md_.occupancy_buffer_.end(), occupancy);
    std::fill(map.md_.occupancy_buffer_inflate_.begin(),
              map.md_.occupancy_buffer_inflate_.end(), occupancy > 0.0);
    std::fill(map.md_.distance_buffer_all_.begin(),
              map.md_.distance_buffer_all_.end(), distance);
    map.md_.dynamic_observation_seq_ = occupancy_seq;
    map.md_.esdf_observation_seq_ = esdf_seq;
    map.md_.lidar_accepted_seq_ = occupancy_seq;
    map.md_.lidar_fused_seq_ = occupancy_seq;
    map.md_.last_occ_update_time_ = ros::Time(100 + occupancy_seq, 0);
    ++map.read_distance_revision_;
    map.publishReadSnapshot();
  }

  static ros::Time pinnedObservationStamp(const GridMap &map)
  {
    return map.readSnapshot()->last_occupancy;
  }

  static bool republishReusesLayers(GridMap &map)
  {
    const auto before = std::atomic_load(&map.latest_read_snapshot_);
    map.publishReadSnapshot();
    const auto after = std::atomic_load(&map.latest_read_snapshot_);
    return before->occupancy == after->occupancy &&
           before->distance == after->distance &&
           before->last_occupancy == after->last_occupancy;
  }

  static void publishNewEpoch(GridMap &map)
  {
    ++map.esdf_map_epoch_;
    publishGeneration(map, 3, 2, 2.0, 0.25);
  }

  static bool fallbackOnlyRequested(const GridMap &map)
  {
    return map.synchronous_fallback_requested_.load() &&
           !map.async_esdf_fallback_to_sync_;
  }

  template <typename Function>
  static void asMappingWriter(GridMap &map, Function function)
  {
    const auto previous = GridMap::writer_context_;
    GridMap::writer_context_ = &map;
    function();
    GridMap::writer_context_ = previous;
  }

  static void useLegacyQueue(GridMap &map) { map.isolated_mapping_ = false; }

  static void configureDistanceRegion(GridMap &map,
                                      const Eigen::Vector3i &dimensions)
  {
    configureImmutableReads(map);
    map.mp_.map_voxel_num_ = dimensions;
    map.mp_.map_size_ = dimensions.cast<double>() * map.mp_.resolution_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    const std::size_t count = static_cast<std::size_t>(dimensions.x()) *
        static_cast<std::size_t>(dimensions.y()) * dimensions.z();
    map.md_.occupancy_buffer_.assign(count, -2.0);
    map.md_.occupancy_buffer_inflate_.assign(count, 0);
    map.md_.static_occupancy_.assign(count, 0);
    map.md_.distance_buffer_all_.resize(count);
    publishDistancePattern(map, 2, 0.0);
  }

  static void publishDistancePattern(GridMap &map, std::uint64_t sequence,
                                     double offset)
  {
    for (std::size_t i = 0; i < map.md_.distance_buffer_all_.size(); ++i)
      map.md_.distance_buffer_all_[i] = offset + double(i) * 0.125;
    map.md_.dynamic_observation_seq_ = sequence;
    map.md_.esdf_observation_seq_ = sequence;
    map.md_.lidar_accepted_seq_ = sequence;
    map.md_.lidar_fused_seq_ = sequence;
    map.md_.last_occ_update_time_ = ros::Time(100 + sequence, 0);
    ++map.read_distance_revision_;
    map.publishReadSnapshot();
  }

  static void setDistanceDimensions(GridMap &map,
                                     const Eigen::Vector3i &dimensions)
  {
    map.mp_.map_voxel_num_ = dimensions;
  }

  static void truncateDistanceStorage(GridMap &map)
  {
    map.md_.distance_buffer_all_.pop_back();
    ++map.read_distance_revision_;
    map.publishReadSnapshot();
  }

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

  static bool waitForAsyncReady(GridMap &map, std::uint64_t completed,
                               const std::chrono::milliseconds &timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      // Observe completion without consuming or committing the ready result.
      if (map.esdf_worker_->stats().completed >= completed) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

#ifndef MAP_BENCHMARK_OLD_ABI
  static void startMappingLoop(GridMap &map)
  {
    ASSERT_FALSE(map.mapping_thread_.joinable());
    map.isolated_mapping_ = true;
    map.mapping_stop_.store(false);
    map.mapping_queue_.enable();
    map.mapping_thread_ = std::thread(&GridMap::mappingLoop, &map);
  }

  static bool waitForLoopComplete(GridMap &map, std::uint64_t sequence,
                                 const std::chrono::milliseconds &timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      const auto snapshot = std::atomic_load(&map.latest_complete_snapshot_);
      if (snapshot && snapshot->observation == sequence && snapshot->esdf == sequence)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }

  static void advanceEpochWithReadyResult(GridMap &map)
  {
    // Keep the ready old-epoch result to exercise the commit-side guard.
    // Normal invalidation also drops that private result in the worker.
    ++map.esdf_map_epoch_;
  }

  static bool waitForLoopEpoch(GridMap &map, std::uint64_t epoch,
                              const std::chrono::milliseconds &timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      const auto snapshot = std::atomic_load(&map.latest_read_snapshot_);
      if (snapshot && snapshot->epoch == epoch)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
  }
#endif

  static void markAsyncOutstandingWithoutResult(GridMap &map)
  {
    // Deterministically exercise the coordinator's busy fence. A real 125-cell
    // job can finish before the test calls occupancy, so it is not a barrier.
    map.esdf_latest_submitted_request_id_ = ++map.esdf_next_request_id_;
    map.md_.esdf_need_update_ = false;
  }

  static void configureAsyncOccupancySnapshots(GridMap &map)
  {
    configureImmutableReads(map);  // Complete generation 1 remains available.
    enableAsync(map, 7);
    map.md_.esdf_observation_seq_ = 1;
    map.md_.last_occ_update_time_ = ros::Time(107, 0);
    seedDynamicAndStaticOccupancy(map);
    map.publishReadSnapshot();  // Live occupancy 7, old distance generation 1.
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
    // Real occupancy now invokes the ESDF callback in async mode. Initialize
    // its mode/static flags as well, instead of depending on stack contents.
    map.mp_ = MappingParameters{};
    map.md_ = MappingData{};
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
      GridMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud,
      bool exact_tf = true)
  {
    sensor_msgs::PointCloud2Ptr message(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(cloud, *message);
    message->header.frame_id = "test_lidar";
    message->header.stamp = ros::Time::now();

    map.tf_buffer_.reset(new tf2_ros::Buffer());
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = message->header.stamp - ros::Duration(exact_tf ? 0. : .5);
    transform.header.frame_id = "world";
    transform.child_frame_id = message->header.frame_id;
    transform.transform.translation.x = 0.75;
    transform.transform.translation.y = 0.75;
    transform.transform.translation.z = 0.50;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(map.tf_buffer_->setTransform(transform, "grid_map_test", exact_tf));

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

  static void processOccupancyAndPublish(GridMap &map)
  {
    asMappingWriter(map, [&]() {
      map.updateOccupancyCallback(ros::TimerEvent());
      map.publishReadSnapshot();  // The private mapping queue's callback epilogue.
    });
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

  static void raycastProjectedLivePoint(GridMap &map,
                                        const Eigen::Vector3d &point)
  {
    map.md_.proj_points_[0] = point;
    map.md_.proj_points_cnt = 1;
    map.md_.lidar_origin_valid_ = false;
    map.raycastProcess(true);
  }

  static void observeMotionThenNoMotion(GridMap &map,
                                   const Eigen::Vector3d &position,
                                   const ros::Time &now)
  {
    auto &buffer = map.dynamic_point_buffer_;
    buffer.enabled_ = true;
    buffer.position_uncertainty_ = 0.03;
    buffer.max_age_ = 0.35;
    plan_env::DynamicPointSnapshot previous;
    previous.stamp = now - ros::Duration(0.20);
    previous.receive_time = previous.stamp;
    previous.frame_id = "world";
    previous.has_track_ids = previous.valid = true;
    plan_env::DynamicPoint point;
    point.track_id = 7;
    point.position = position;
    previous.points.push_back(point);
    buffer.recordObservation(previous);
    plan_env::DynamicPointSnapshot current;
    current.stamp = now - ros::Duration(0.01);
    current.receive_time = current.stamp;
    current.frame_id = "world";
    current.valid = current.has_track_ids = true;
    buffer.recordObservation(current);
  }

  static void raycastLiveReturns(
      GridMap &map, const std::vector<Eigen::Vector3d> &endpoints,
      const ros::Time &stamp, bool live_lidar = true)
  {
    map.md_.lidar_origin_ = Eigen::Vector3d(0.25, 0.75, 0.75);
    map.md_.lidar_origin_valid_ = true;
    map.md_.pending_lidar_stamp_ = stamp;
    std::copy(endpoints.begin(), endpoints.end(), map.md_.proj_points_.begin());
    map.md_.proj_points_cnt = static_cast<int>(endpoints.size());
    map.raycastProcess(live_lidar);
  }

  static void fuseProjectedFrame(GridMap &map, const Eigen::Vector3d &endpoint,
                                 bool live_lidar)
  {
    // Exercise the real occupancy callback, including raycasting, perimeter
    // handling, inflation and consumption of the pending LiDAR generation.
    map.md_.camera_pos_ = map.md_.lidar_origin_ = Eigen::Vector3d(.25, .75, .75);
    map.md_.lidar_origin_valid_ = live_lidar;
    map.md_.proj_points_[0] = endpoint;
    map.md_.proj_points_cnt = 1;
    map.md_.occ_need_update_ = true;
    map.md_.flag_use_depth_fusion = true;
    if (live_lidar)
    {
      map.md_.lidar_pending_seq_ = ++map.md_.lidar_accepted_seq_;
      map.md_.pending_lidar_stamp_ = ros::Time::now();
      map.md_.pending_lidar_received_steady_ = ros::SteadyTime::now();
    }
    map.updateOccupancyCallback(ros::TimerEvent());
  }

  static void configureIncrementalTest(
      GridMap &map, bool incremental,
      const Eigen::Vector3i &dimensions = Eigen::Vector3i(61, 51, 31),
      double resolution = 0.1, int threads = 1)
  {
    // These tests invoke the real timer callback without initMap(). Explicit
    // value initialization prevents unrelated uninitialized runtime flags.
    map.mp_ = MappingParameters{};
    map.md_ = MappingData{};
    map.mp_.resolution_ = resolution;
    map.mp_.resolution_inv_ = 1.0 / resolution;
    map.mp_.map_voxel_num_ = dimensions;
    map.mp_.map_origin_ = Eigen::Vector3d::Zero();
    map.mp_.map_size_ = dimensions.cast<double>() * resolution;
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_size_;
    map.mp_.parallel_threads_ = threads;
    map.mp_.parallel_voxel_threshold_ = 1;
    map.mp_.incremental_esdf_ = incremental;
    map.mp_.esdf_distance_cap_ = 0.6;
    map.mp_.async_esdf_enable_ = false;
    map.mp_.async_esdf_max_result_age_ = 5.0;
    map.mp_.show_esdf_time_ = false;
    map.mp_.enable_performance_diagnostics_ = false;
    map.mp_.use_global_map_ = true;
    map.mp_.fuse_global_map_ = true;
    map.mp_.global_map_full_esdf_ = true;
    map.mp_.local_esdf_follow_odom_ = false;
    map.mp_.live_lidar_health_monitor_enable_ = false;
    map.mp_.clamp_min_log_ = -2.0;
    map.mp_.min_occupancy_log_ = 0.0;
    map.mp_.unknown_flag_ = 0.01;
    map.mp_.local_map_margin_ = 0;
    map.mp_.obstacles_inflation_ = 0.0;
    map.mp_.virtual_ceil_height_ = -1.0;
    map.mp_.global_map_lidar_clear_min_misses_ = 1;
    map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
    map.md_.local_bound_max_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    map.md_.occupancy_bound_min_ = map.md_.local_bound_min_;
    map.md_.occupancy_bound_max_ = map.md_.local_bound_max_;
    map.md_.has_odom_ = true;
    map.md_.has_cloud_ = true;
    map.md_.static_map_esdf_ready_ = true;
    map.md_.static_map_esdf_pending_ = false;
    map.md_.global_esdf_pending_ = false;
    const std::size_t count = static_cast<std::size_t>(dimensions.x()) *
        dimensions.y() * dimensions.z();
    map.md_.occupancy_buffer_.assign(count, -2.0);
    map.md_.occupancy_buffer_inflate_.assign(count, 0);
    map.md_.occupancy_buffer_neg_.assign(count, 0);
    map.md_.static_occupancy_.assign(count, 0);
    map.md_.static_miss_count_.assign(count, 0);
    map.md_.tmp_buffer1_.assign(count, 0.0);
    map.md_.tmp_buffer2_.assign(count, 0.0);
    map.md_.distance_buffer_.assign(count, 10000.0);
    map.md_.distance_buffer_neg_.assign(count, 10000.0);
    map.md_.distance_buffer_all_.assign(count, 10000.0);
    map.isolated_mapping_ = false;
    map.esdf_map_epoch_ = 1;
    map.esdf_occupancy_epoch_ = 0;
    map.esdf_committed_occupancy_.clear();
    map.incremental_patch_ready_ = false;
  }

  static void stageIncrementalOccupancy(GridMap &map,
                                        const std::vector<char> &occupancy,
                                        std::uint64_t sequence)
  {
    map.md_.occupancy_buffer_inflate_ = occupancy;
    for (std::size_t i = 0; i < occupancy.size(); ++i)
      map.md_.occupancy_buffer_[i] = occupancy[i] ? 2.0 : -2.0;
    map.md_.dynamic_observation_seq_ = sequence;
    map.md_.lidar_accepted_seq_ = sequence;
    map.md_.lidar_fused_seq_ = sequence;
    map.md_.last_occ_update_time_ = ros::Time(100 + sequence, 0);
    map.md_.esdf_need_update_ = true;
  }

  static Eigen::Vector3i writeMinimum(const GridMap &map) { return map.esdf_write_min_; }
  static Eigen::Vector3i writeMaximum(const GridMap &map) { return map.esdf_write_max_; }
  static std::uint64_t distanceRevision(const GridMap &map) { return map.read_distance_revision_; }
  static std::uint64_t computedCount(const GridMap &map) { return map.md_.esdf_update_count_; }
  static void invalidateIncrementalEpoch(GridMap &map) { map.invalidateAsyncESDFJobs(); }
  static const std::vector<char> &committedOccupancy(const GridMap &map)
  {
    return map.esdf_committed_occupancy_;
  }
  static std::uint64_t occupancyEpoch(const GridMap &map)
  {
    return map.esdf_occupancy_epoch_;
  }
  static void changeObservationOnly(GridMap &map, std::uint64_t sequence)
  {
    map.md_.dynamic_observation_seq_ = sequence;
  }
  static void commitReadyAndPublish(GridMap &map)
  {
    asMappingWriter(map, [&]() {
      map.commitAsyncESDFResult();
      map.publishReadSnapshot();
    });
  }
  static std::vector<double> timingTotals(const GridMap &map)
  {
    return {map.md_.esdf_wall_sum_ms_, map.md_.esdf_commit_wall_sum_ms_,
            map.md_.async_esdf_snapshot_wall_sum_ms_};
  }
#ifndef MAP_BENCHMARK_OLD_ABI
  static std::pair<std::size_t, std::size_t> readyPatchStats(GridMap &map)
  {
    return plan_env::ESDFWorkerTestAccessor::readyPatchStats(*map.esdf_worker_);
  }
  static bool damageLastReadyPatch(GridMap &map, bool remove_patch)
  {
    return plan_env::ESDFWorkerTestAccessor::damageLastReadyPatch(
        *map.esdf_worker_, remove_patch);
  }
#endif

  static int clearStaticCellForIncrementalTest(GridMap &map, const Eigen::Vector3i &cell)
  {
    const auto index = address(map, cell);
    map.md_.static_occupancy_[index] = 1;
    map.md_.static_miss_count_[index] = 1;
    return map.clearStaticOccupancyAround(cell, 0);
  }

  static void clearLocalPerimeterForIncrementalTest(GridMap &map)
  {
    map.md_.occupancy_bound_min_ = Eigen::Vector3i(28, 23, 13);
    map.md_.occupancy_bound_max_ = Eigen::Vector3i(32, 27, 17);
    map.clearAndInflateLocalMap();
  }

  static const std::vector<char> &inflatedLayer(const GridMap &map)
  {
    return map.md_.occupancy_buffer_inflate_;
  }

  static void requestESDFAfterLocalClearing(GridMap &map, std::uint64_t sequence)
  {
    // The occupancy callback advances this revision after both real clearing
    // operations. Do not replace their actual raw or inflated output here.
    map.md_.dynamic_observation_seq_ = sequence;
    map.md_.esdf_need_update_ = true;
  }

  static void publishIncrementalSnapshot(GridMap &map)
  {
    map.isolated_mapping_ = true;
    map.publishReadSnapshot();
  }

  static std::shared_ptr<const std::vector<double>> latestDistanceLayer(const GridMap &map)
  {
    return std::atomic_load(&map.latest_read_snapshot_)->distance;
  }

  static void enableIncrementalAsync(GridMap &map)
  {
    const auto sequence = map.md_.esdf_observation_seq_;
    enableAsync(map, sequence);
    map.md_.esdf_observation_seq_ = sequence;
  }

  static void incrementalCallbackAndPublish(GridMap &map)
  {
    asMappingWriter(map, [&]() {
      map.updateESDFCallback(ros::TimerEvent());
      map.publishReadSnapshot();
    });
  }

  static bool waitForIncrementalCallbackCommit(GridMap &map, std::uint64_t sequence)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
      incrementalCallbackAndPublish(map);
      if (map.md_.esdf_observation_seq_ == sequence && !map.md_.esdf_need_update_)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
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

std::vector<double> scalarDistanceRegion(GridMap &map,
                                         const Eigen::Vector3i &lo,
                                         const Eigen::Vector3i &hi)
{
  std::vector<double> result;
  for (int x = lo.x(); x <= hi.x(); ++x)
    for (int y = lo.y(); y <= hi.y(); ++y)
      for (int z = lo.z(); z <= hi.z(); ++z)
        result.push_back(map.getDistance(Eigen::Vector3i(x, y, z)));
  return result;
}

}  // namespace

#ifndef MAP_BENCHMARK_OLD_ABI
TEST(GridMapOccupancy, LiveHitEvidenceMatchesDistanceAndResetsWhenNoHit)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMap::DynamicObstacleHit hit;
  const Eigen::Vector3d query(.875, .75, .75);
  const double radius = .4;
  EXPECT_DOUBLE_EQ(radius, map.getDynamicObstacleDistance(query, radius, &hit));
  EXPECT_FALSE(hit.valid);
  EXPECT_DOUBLE_EQ(map.getDynamicObstacleDistance(query, radius),
                   map.getDynamicObstacleDistance(query, radius, &hit));

  GridMapParallelTestAccessor::seedDynamicAndStaticOccupancy(map);
  const double distance = map.getDynamicObstacleDistance(query, radius, &hit);
  ASSERT_TRUE(hit.valid);
  EXPECT_DOUBLE_EQ(.125, distance);
  EXPECT_DOUBLE_EQ(map.getDynamicObstacleDistance(query, radius), distance);
  EXPECT_TRUE(hit.voxel == Eigen::Vector3i(1, 1, 1));
  EXPECT_TRUE(hit.center == Eigen::Vector3d(.75, .75, .75));
  EXPECT_DOUBLE_EQ(distance, hit.distance);
  EXPECT_DOUBLE_EQ(distance, (query - hit.center).norm());
  EXPECT_DOUBLE_EQ(2., hit.occupancy);
  EXPECT_DOUBLE_EQ(0., hit.occupancy_threshold);

  // A static-map voxel is excluded, and must not retain the previous live hit.
  const Eigen::Vector3d static_query(1.75, 1.75, 1.75);
  EXPECT_DOUBLE_EQ(radius, map.getDynamicObstacleDistance(static_query, radius, &hit));
  EXPECT_FALSE(hit.valid);
  EXPECT_DOUBLE_EQ(map.getDynamicObstacleDistance(static_query, radius),
                   map.getDynamicObstacleDistance(static_query, radius, &hit));

  // Equality to the search limit remains a non-hit, just like the old API.
  const Eigen::Vector3d boundary_query(1., .75, .75);
  EXPECT_DOUBLE_EQ(.25, map.getDynamicObstacleDistance(boundary_query, .25, &hit));
  EXPECT_FALSE(hit.valid);
  EXPECT_DOUBLE_EQ(map.getDynamicObstacleDistance(boundary_query, .25),
                   map.getDynamicObstacleDistance(boundary_query, .25, &hit));
}

#endif

TEST(GridMapDistanceCopy, InclusiveBoundariesAndRandomCropsMatchScalarQueries)
{
  ensureRosInitialized();
  const Eigen::Vector3i dimensions(17, 13, 9);
  for (bool legacy : {false, true})
  {
    GridMap map;
    GridMapParallelTestAccessor::configureDistanceRegion(map, dimensions);
    if (legacy) GridMapParallelTestAccessor::useLegacyQueue(map);
    GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
    ASSERT_TRUE(complete.valid());
    std::vector<double> output;
    for (const Eigen::Vector3i &cell :
         {Eigen::Vector3i::Zero().eval(),
          (dimensions - Eigen::Vector3i::Ones()).eval()})
    {
      ASSERT_TRUE(map.copyDistanceRegion(cell, cell, output));
      ASSERT_EQ(1U, output.size());
      EXPECT_DOUBLE_EQ(map.getDistance(cell), output.front());
    }
    ASSERT_TRUE(map.copyDistanceRegion(Eigen::Vector3i::Zero(),
        dimensions - Eigen::Vector3i::Ones(), output));
    EXPECT_EQ(scalarDistanceRegion(map, Eigen::Vector3i::Zero(),
        dimensions - Eigen::Vector3i::Ones()), output);

    std::mt19937 random(913);
    for (int sample = 0; sample < 100; ++sample)
    {
      Eigen::Vector3i lo, hi;
      for (int axis = 0; axis < 3; ++axis)
      {
        std::uniform_int_distribution<int> index(0, dimensions[axis] - 1);
        const int first = index(random), second = index(random);
        lo[axis] = std::min(first, second);
        hi[axis] = std::max(first, second);
      }
      ASSERT_TRUE(map.copyDistanceRegion(lo, hi, output));
      const auto scalar = scalarDistanceRegion(map, lo, hi);
      ASSERT_EQ(scalar.size(), output.size());
      EXPECT_EQ(0, std::memcmp(scalar.data(), output.data(),
                              scalar.size() * sizeof(double)));
    }
  }
}

TEST(GridMapDistanceCopy, InvalidBoundsAndStorageDoNotModifyOutput)
{
  ensureRosInitialized();
  GridMap map;
  const Eigen::Vector3i dimensions(5, 4, 3);
  GridMapParallelTestAccessor::configureDistanceRegion(map, dimensions);
  const std::vector<double> original{91.0, 92.0};
  std::vector<double> output = original;
  EXPECT_FALSE(map.copyDistanceRegion(Eigen::Vector3i(-1, 0, 0),
                                      Eigen::Vector3i::Zero(), output));
  EXPECT_EQ(original, output);
  EXPECT_FALSE(map.copyDistanceRegion(Eigen::Vector3i::Zero(),
                                      dimensions, output));
  EXPECT_EQ(original, output);
  EXPECT_FALSE(map.copyDistanceRegion(Eigen::Vector3i(2, 0, 0),
                                      Eigen::Vector3i(1, 0, 0), output));
  EXPECT_EQ(original, output);
  GridMapParallelTestAccessor::setDistanceDimensions(map,
      Eigen::Vector3i(5, 0, 3));
  EXPECT_FALSE(map.copyDistanceRegion(Eigen::Vector3i::Zero(),
                                      Eigen::Vector3i::Zero(), output));
  EXPECT_EQ(original, output);
  GridMapParallelTestAccessor::setDistanceDimensions(map,
      Eigen::Vector3i::Constant(std::numeric_limits<int>::max()));
  EXPECT_FALSE(map.copyDistanceRegion(Eigen::Vector3i::Zero(),
                                      Eigen::Vector3i::Zero(), output));
  EXPECT_EQ(original, output);
  GridMapParallelTestAccessor::setDistanceDimensions(map, dimensions);
  GridMapParallelTestAccessor::truncateDistanceStorage(map);
  EXPECT_FALSE(map.copyDistanceRegion(Eigen::Vector3i::Zero(),
                                      dimensions - Eigen::Vector3i::Ones(),
                                      output));
  EXPECT_EQ(original, output);
}

TEST(GridMapDistanceCopy, CompleteScopeKeepsValuesVersionAndFusionStampPinned)
{
  ensureRosInitialized();
  GridMap map;
  const Eigen::Vector3i lo(1, 2, 0), hi(8, 5, 4);
  GridMapParallelTestAccessor::configureDistanceRegion(map,
      Eigen::Vector3i(11, 8, 6));
  GridMapParallelTestAccessor::asMappingWriter(map, [&]() {
    GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
    ASSERT_TRUE(complete.valid());
    const auto version = map.getReadESDFVersion();
    const auto fusion_stamp = map.getReadFusionTime();
    const auto expected = scalarDistanceRegion(map, lo, hi);
    std::thread writer([&]() {
      GridMapParallelTestAccessor::publishDistancePattern(map, 3, 1000000.0);
    });
    writer.join();
    std::vector<double> output;
    ASSERT_TRUE(map.copyDistanceRegion(lo, hi, output));
    EXPECT_EQ(expected, output);
    EXPECT_EQ(version, map.getReadESDFVersion());
    EXPECT_EQ(fusion_stamp, map.getReadFusionTime());
    EXPECT_EQ(2U, map.getDynamicObservationSeq());
    {
      GridMap::ReadScope next(map, GridMap::ReadScope::Complete);
      ASSERT_TRUE(next.valid());
      ASSERT_TRUE(map.copyDistanceRegion(lo, hi, output));
      EXPECT_EQ(scalarDistanceRegion(map, lo, hi), output);
      EXPECT_NE(expected, output);
      EXPECT_NE(version, map.getReadESDFVersion());
      EXPECT_EQ(ros::Time(103, 0), map.getReadFusionTime());
    }
    ASSERT_TRUE(map.copyDistanceRegion(lo, hi, output));
    EXPECT_EQ(expected, output);
    EXPECT_EQ(version, map.getReadESDFVersion());
    EXPECT_EQ(fusion_stamp, map.getReadFusionTime());
  });
}

TEST(GridMapDistanceCopy, ReportsScalarAndRowCopyTimesWithoutSpeedThreshold)
{
  ensureRosInitialized();
  GridMap map;
  const Eigen::Vector3i dimensions(82, 82, 60);
  const Eigen::Vector3i lo = Eigen::Vector3i::Zero();
  const Eigen::Vector3i hi = dimensions - Eigen::Vector3i::Ones();
  const std::size_t count = static_cast<std::size_t>(dimensions.x()) *
      dimensions.y() * dimensions.z();
  GridMapParallelTestAccessor::configureDistanceRegion(map, dimensions);
  GridMapParallelTestAccessor::asMappingWriter(map, [&]() {
    GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
    ASSERT_TRUE(complete.valid());
    std::vector<double> scalar, copied;
    scalar.reserve(count);
    copied.reserve(count);
    for (int round = 0; round < 3; ++round)
    {
      scalar.clear();
      const auto scalar_start = std::chrono::steady_clock::now();
      for (int x = lo.x(); x <= hi.x(); ++x)
        for (int y = lo.y(); y <= hi.y(); ++y)
          for (int z = lo.z(); z <= hi.z(); ++z)
            scalar.push_back(map.getDistance(Eigen::Vector3i(x, y, z)));
      const auto scalar_end = std::chrono::steady_clock::now();
      const auto copy_start = std::chrono::steady_clock::now();
      ASSERT_TRUE(map.copyDistanceRegion(lo, hi, copied));
      const auto copy_end = std::chrono::steady_clock::now();
      ASSERT_EQ(scalar.size(), copied.size());
      EXPECT_EQ(0, std::memcmp(scalar.data(), copied.data(),
                              scalar.size() * sizeof(double)));
      const double scalar_ms = std::chrono::duration<double, std::milli>(
          scalar_end - scalar_start).count();
      const double copy_ms = std::chrono::duration<double, std::milli>(
          copy_end - copy_start).count();
      std::cout << "[distance-region benchmark] shape=82x82x60 cells="
                << count << " round=" << round << " scalar_ms=" << scalar_ms
                << " row_copy_ms=" << copy_ms << std::endl;
    }
  });
}

TEST(GridMapImmutable, WriterContinuesWhileReaderPinsOneGeneration)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  const Eigen::Vector3i cell(1, 1, 1);
  {
    GridMap::ReadScope scope(map);
    std::thread writer([&]() {
      GridMapParallelTestAccessor::publishGeneration(map, 2, 2, 2.0, 0.25);
    });
    writer.join();
    EXPECT_EQ(1U, map.getDynamicObservationSeq());
    EXPECT_EQ(0, map.getOccupancy(cell));
    EXPECT_DOUBLE_EQ(0.75, map.getDistance(cell));
    EXPECT_EQ(ros::Time(101, 0),
              GridMapParallelTestAccessor::pinnedObservationStamp(map));
    map.refreshReadSnapshot();
    EXPECT_EQ(2U, map.getDynamicObservationSeq());
    EXPECT_EQ(1, map.getOccupancy(cell));
    EXPECT_DOUBLE_EQ(0.25, map.getDistance(cell));
    EXPECT_EQ(ros::Time(102, 0),
              GridMapParallelTestAccessor::pinnedObservationStamp(map));
  }
  EXPECT_EQ(2U, map.getDynamicObservationSeq());
  EXPECT_TRUE(GridMapParallelTestAccessor::republishReusesLayers(map));
}

TEST(GridMapImmutable, PlanningUsesCompletePairAndDeliverySeesLiveOccupancy)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  GridMapParallelTestAccessor::publishGeneration(map, 2, 1, 2.0, 0.75);
  GridMap::ReadScope scope(map);
  ASSERT_TRUE(map.beginPlanningReadSnapshot());
  EXPECT_EQ(1U, map.getDynamicObservationSeq());
  EXPECT_EQ(1U, map.getESDFObservationSeq());
  EXPECT_EQ(0, map.getOccupancy(Eigen::Vector3i(1, 1, 1)));
  map.refreshReadSnapshot();
  EXPECT_EQ(2U, map.getDynamicObservationSeq());
  EXPECT_EQ(1U, map.getESDFObservationSeq());
  EXPECT_EQ(1, map.getOccupancy(Eigen::Vector3i(1, 1, 1)));
  GridMapParallelTestAccessor::publishGeneration(map, 2, 2, 2.0, 0.25);
  ASSERT_TRUE(map.beginPlanningReadSnapshot());
  EXPECT_EQ(2U, map.getDynamicObservationSeq());
  EXPECT_DOUBLE_EQ(0.25, map.getDistance(Eigen::Vector3i(1, 1, 1)));
}

TEST(GridMapImmutable, DistanceRefreshDoesNotNeedNewLidarSequence)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  GridMap::ReadScope scope(map);
  GridMapParallelTestAccessor::publishGeneration(map, 1, 1, -2.0, 0.5);
  EXPECT_DOUBLE_EQ(0.75, map.getDistance(Eigen::Vector3i(1, 1, 1)));
  map.refreshReadSnapshot();
  EXPECT_EQ(1U, map.getESDFObservationSeq());
  EXPECT_DOUBLE_EQ(0.5, map.getDistance(Eigen::Vector3i(1, 1, 1)));
  EXPECT_TRUE(GridMapParallelTestAccessor::republishReusesLayers(map));
}

TEST(GridMapImmutable, OldEpochCannotAuthorizePlanAndFallbackIsOnlyRequested)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  GridMapParallelTestAccessor::publishNewEpoch(map);
  GridMap::ReadScope scope(map);
  EXPECT_FALSE(map.beginPlanningReadSnapshot());
  map.requestSynchronousESDFFallback();
  EXPECT_TRUE(GridMapParallelTestAccessor::fallbackOnlyRequested(map));
}

TEST(GridMapImmutable, ExplicitCompleteExportScopeOverridesOnlyItsWriterReads)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  GridMapParallelTestAccessor::publishGeneration(map, 2, 1, 2.0, 0.75);
  GridMapParallelTestAccessor::asMappingWriter(map, [&]() {
    EXPECT_EQ(2U, map.getDynamicObservationSeq());
    {
      GridMap::ReadScope ordinary(map);
      EXPECT_EQ(2U, map.getDynamicObservationSeq());
      GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
      EXPECT_TRUE(complete.valid());
      EXPECT_EQ(1U, map.getDynamicObservationSeq());
      EXPECT_EQ(1U, map.getESDFObservationSeq());
      EXPECT_EQ(0, map.getOccupancy(Eigen::Vector3i(1, 1, 1)));
      EXPECT_EQ(ros::Time(101, 0), map.getReadFusionTime());
    }
    EXPECT_EQ(2U, map.getDynamicObservationSeq());
    EXPECT_EQ(1, map.getOccupancy(Eigen::Vector3i(1, 1, 1)));
  });
}

TEST(GridMapImmutable, CompleteExportRetainsFusionStampUntilNewGeneration)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  GridMapParallelTestAccessor::publishGeneration(map, 2, 1, 2.0, 0.75);
  for (int sample = 0; sample < 3; ++sample)
  {
    EXPECT_TRUE(GridMapParallelTestAccessor::republishReusesLayers(map));
    GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
    EXPECT_TRUE(complete.valid());
    EXPECT_EQ(ros::Time(101, 0), map.getReadFusionTime());
    EXPECT_EQ(1U, map.getLidarAcceptedSeq());
    EXPECT_EQ(1U, map.getLidarFusedSeq());
  }
  GridMapParallelTestAccessor::publishGeneration(map, 2, 2, 2.0, 0.25);
  GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
  EXPECT_TRUE(complete.valid());
  EXPECT_EQ(ros::Time(102, 0), map.getReadFusionTime());
  EXPECT_EQ(2U, map.getLidarAcceptedSeq());
  EXPECT_EQ(2U, map.getLidarFusedSeq());
}

TEST(GridMapImmutable, CompleteExportRejectsResetEpochAndVersionsSameSeqRecompute)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  const auto original = map.getReadESDFVersion();
  GridMapParallelTestAccessor::publishGeneration(map, 1, 1, -2.0, 0.5);
  EXPECT_NE(original, map.getReadESDFVersion());
  EXPECT_EQ(original.first, map.getReadESDFVersion().first);
  EXPECT_EQ(1U, map.getESDFObservationSeq());
  GridMapParallelTestAccessor::publishNewEpoch(map);
  {
    GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
    EXPECT_FALSE(complete.valid());
  }
  GridMapParallelTestAccessor::publishGeneration(map, 3, 3, 2.0, 0.25);
  GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
  EXPECT_TRUE(complete.valid());
  EXPECT_NE(original.first, map.getReadESDFVersion().first);
  EXPECT_EQ(3U, map.getESDFObservationSeq());
}

TEST(GridMapImmutable, LegacyExportCannotUseIncompleteRawGeneration)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureImmutableReads(map);
  GridMapParallelTestAccessor::useLegacyQueue(map);
  GridMapParallelTestAccessor::publishGeneration(map, 2, 1, 2.0, 0.75);
  GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
  EXPECT_FALSE(complete.valid());
  EXPECT_EQ(2U, map.getDynamicObservationSeq());
}

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

TEST(GridMapOccupancy, SensorCloudRequiresExactEpochInsteadOfLatestTfFallback)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(.5f, .2f, .2f));
  GridMapParallelTestAccessor::receiveSynchronizedCloud(map, cloud, false);
  EXPECT_EQ(0U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  EXPECT_FALSE(GridMapParallelTestAccessor::occupancyPending(map));
  GridMapParallelTestAccessor::receiveSynchronizedCloud(map, cloud, true);
  EXPECT_EQ(1U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
}

TEST(GridMapOccupancy, GeometryFilteredRearReturnsRemainAvailableForMapping)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(-.5f, .2f, .2f));
  GridMapParallelTestAccessor::receiveSynchronizedCloud(map, cloud);
  EXPECT_EQ(1U, GridMapParallelTestAccessor::lidarAcceptedSeq(map));
  GridMapParallelTestAccessor::processOccupancy(map);
  EXPECT_EQ(1U, GridMapParallelTestAccessor::lidarFusedSeq(map));
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

TEST(GridMapOccupancy, OccludedLiveObstacleSurvivesUnrelatedFreshScans)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  const ros::Time now = ros::Time::now();
  const Eigen::Vector3d obstacle(1.25, .75, .75);
  GridMapParallelTestAccessor::raycastLiveReturns(map, {obstacle}, now);
  for (int frame = 0; frame < 20; ++frame)
    GridMapParallelTestAccessor::raycastLiveReturns(
        map, {Eigen::Vector3d(.75, .75, .75)}, now);
  EXPECT_DOUBLE_EQ(2., GridMapParallelTestAccessor::rawOccupancy(
      map, Eigen::Vector3i(2, 1, 1)));
}

TEST(GridMapOccupancy, ActualFreeRayClearsDepartedLiveObstacle)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  const ros::Time now = ros::Time::now();
  GridMapParallelTestAccessor::raycastLiveReturns(
      map, {Eigen::Vector3d(1.25, .75, .75)}, now);
  GridMapParallelTestAccessor::raycastLiveReturns(
      map, {Eigen::Vector3d(2.25, .75, .75)}, now);
  EXPECT_LT(GridMapParallelTestAccessor::rawOccupancy(
      map, Eigen::Vector3i(2, 1, 1)), 0.);
}

TEST(GridMapOccupancy, LiveBoundsShrinkAndExpandWithoutClearingUnobservedObstacle)
{
  ensureRosInitialized();
  using Access = GridMapParallelTestAccessor;
  const Eigen::Vector3i obstacle(2, 1, 1);
  for (const bool live_lidar : {true, false})
  {
    SCOPED_TRACE(live_lidar ? "live LiDAR" : "legacy depth");
    GridMap map;
    Access::configureOccupancyUpdate(map);
    Access::fuseProjectedFrame(map, Eigen::Vector3d(1.25, .75, .75), live_lidar);
    ASSERT_DOUBLE_EQ(2., Access::rawOccupancy(map, obstacle));
    ASSERT_EQ(1, Access::inflatedOccupancy(map, obstacle));

    // The nearer return shrinks x bounds; the old obstacle is now in the
    // five-cell perimeter. Neither this ray nor the next one crosses it.
    Access::fuseProjectedFrame(map, Eigen::Vector3d(.75, .75, .75), live_lidar);
    if (live_lidar)
      EXPECT_DOUBLE_EQ(2., Access::rawOccupancy(map, obstacle));
    else
      EXPECT_LT(Access::rawOccupancy(map, obstacle), -2.);  // Legacy reset retained.

    // A diagonal return expands bounds and rebuilds the old cell's inflation.
    // A test that checks only the shrink frame would miss delayed erasure.
    Access::fuseProjectedFrame(map, Eigen::Vector3d(2.25, 2.25, .75), live_lidar);
    EXPECT_EQ(live_lidar ? 1 : 0, Access::inflatedOccupancy(map, obstacle));
    if (live_lidar)
    {
      EXPECT_DOUBLE_EQ(2., Access::rawOccupancy(map, obstacle));
      // Real free-space evidence must still remove a departed live obstacle.
      Access::fuseProjectedFrame(map, Eigen::Vector3d(2.25, .75, .75), true);
      EXPECT_LT(Access::rawOccupancy(map, obstacle), 0.);
      EXPECT_EQ(0, Access::inflatedOccupancy(map, obstacle));
      EXPECT_EQ(4u, Access::lidarFusedSeq(map));
    }
  }
}

TEST(GridMapOccupancy, CurrentEndpointWinsOverOtherFreeRaysInSameVoxel)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  GridMapParallelTestAccessor::raycastLiveReturns(
      map, {Eigen::Vector3d(2.25, .55, .75),
            Eigen::Vector3d(2.25, .95, .75),
            Eigen::Vector3d(1.25, .75, .75)}, ros::Time::now());
  EXPECT_DOUBLE_EQ(2., GridMapParallelTestAccessor::rawOccupancy(
      map, Eigen::Vector3i(2, 1, 1)));
}

TEST(GridMapOccupancy, MissingPredictionIdDoesNotEraseOccupancyOrBlockOtherSpace)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  const ros::Time now = ros::Time::now();
  const Eigen::Vector3d obstacle(1.25, .75, .75);
  GridMapParallelTestAccessor::raycastLiveReturns(map, {obstacle}, now);
  GridMapParallelTestAccessor::observeMotionThenNoMotion(map, obstacle, now);
  const auto current = map.getDynamicPointSnapshot(now);
  EXPECT_TRUE(current.valid);
  EXPECT_TRUE(current.points.empty());
  EXPECT_EQ("no_dynamic_detected", current.invalid_reason);
  EXPECT_DOUBLE_EQ(2., GridMapParallelTestAccessor::rawOccupancy(
      map, Eigen::Vector3i(2, 1, 1)));
  GridMapParallelTestAccessor::raycastLiveReturns(
      map, {Eigen::Vector3d(2.25, .75, .75)}, now);
  EXPECT_LT(GridMapParallelTestAccessor::rawOccupancy(
      map, Eigen::Vector3i(2, 1, 1)), 0.);
}

TEST(GridMapOccupancy, FilteredScanCannotClearExistingObstacle)
{
  ensureRosInitialized();
  GridMap map;
  GridMapParallelTestAccessor::configureOccupancyUpdate(map);
  const ros::Time now = ros::Time::now();
  GridMapParallelTestAccessor::raycastLiveReturns(
      map, {Eigen::Vector3d(1.25, .75, .75)}, now);
  for (int frame = 0; frame < 20; ++frame)
    GridMapParallelTestAccessor::raycastProjectedLivePoint(
        map, Eigen::Vector3d(1.25, .75, .10));
  EXPECT_DOUBLE_EQ(2., GridMapParallelTestAccessor::rawOccupancy(
      map, Eigen::Vector3i(2, 1, 1)));
}

TEST(GridMapOccupancy, OutstandingESDFWithoutReadyResultDefersAllOccupancyMutation)
{
  ensureRosInitialized();
  using Access = GridMapParallelTestAccessor;
  GridMap map;
  Access::configureAsyncOccupancySnapshots(map);
  Access::markAsyncOutstandingWithoutResult(map);

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  Access::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(8U, Access::lidarPendingSeq(map));
  std::vector<double> raw;
  std::vector<int> inflated;
  for (int x = 0; x < 5; ++x)
    for (int y = 0; y < 5; ++y)
      for (int z = 0; z < 5; ++z)
      {
        raw.push_back(Access::rawOccupancy(map, Eigen::Vector3i(x, y, z)));
        inflated.push_back(Access::inflatedOccupancy(map, Eigen::Vector3i(x, y, z)));
      }
  const auto distance = Access::distances(map);
  Access::processOccupancyAndPublish(map);

  // No fused-map mutation may invalidate the immutable request. The newest
  // sensor frame remains pending and replaces older frames in the same slot.
  EXPECT_EQ(7U, Access::dynamicObservationSeq(map));
  EXPECT_EQ(7U, Access::lidarFusedSeq(map));
  EXPECT_EQ(8U, Access::lidarPendingSeq(map));
  EXPECT_EQ(1U, Access::esdfObservationSeq(map));
  EXPECT_TRUE(Access::occupancyPending(map));
  EXPECT_TRUE(Access::lidarObservationPending(map));
  EXPECT_EQ(1U, Access::deferredOccupancyCount(map));
  EXPECT_EQ(0U, Access::asyncCommittedCount(map));
  EXPECT_EQ(0U, Access::asyncRejectedCount(map));
  EXPECT_EQ(distance, Access::distances(map));
  std::size_t index = 0;
  for (int x = 0; x < 5; ++x)
    for (int y = 0; y < 5; ++y)
      for (int z = 0; z < 5; ++z, ++index)
      {
        EXPECT_DOUBLE_EQ(raw[index], Access::rawOccupancy(map, Eigen::Vector3i(x, y, z)));
        EXPECT_EQ(inflated[index], Access::inflatedOccupancy(map, Eigen::Vector3i(x, y, z)));
      }
  GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
  ASSERT_TRUE(complete.valid());
  EXPECT_EQ(1U, map.getDynamicObservationSeq());
  EXPECT_EQ(ros::Time(101, 0), map.getReadFusionTime());
}

TEST(GridMapOccupancy, ReadyESDFCommitsBeforeNextFusionAndSubmitsInSameCallback)
{
  ensureRosInitialized();
  using Access = GridMapParallelTestAccessor;
  GridMap map;
  Access::configureAsyncOccupancySnapshots(map);
  ASSERT_TRUE(Access::submitAsync(map));
  ASSERT_TRUE(Access::waitForAsyncReady(map, 1, std::chrono::milliseconds(5000)));
  ASSERT_EQ(0U, Access::asyncCommittedCount(map));

  pcl::PointCloud<pcl::PointXYZ> valid_cloud;
  valid_cloud.push_back(pcl::PointXYZ(1.50f, 1.50f, 0.50f));
  Access::receiveIndependentCloud(map, valid_cloud);
  ASSERT_EQ(8U, Access::lidarPendingSeq(map));
  Access::processOccupancyAndPublish(map);

  EXPECT_EQ(1U, Access::asyncCommittedCount(map));
  EXPECT_EQ(0U, Access::asyncRejectedCount(map));
  EXPECT_EQ(0U, Access::deferredOccupancyCount(map));
  EXPECT_EQ(7U, Access::esdfObservationSeq(map));
  EXPECT_EQ(8U, Access::dynamicObservationSeq(map));
  EXPECT_EQ(8U, Access::lidarFusedSeq(map));
  EXPECT_FALSE(Access::occupancyPending(map));
  EXPECT_FALSE(Access::lidarObservationPending(map));
  EXPECT_FALSE(Access::esdfNeedsUpdate(map));  // Already submitted, not waiting for a timer.
  EXPECT_EQ(2U, Access::latestSubmittedRequestId(map));
  EXPECT_FALSE(Access::asyncFallbackSelected(map));

  const Eigen::Vector3i new_obstacle(3, 3, 1);
  EXPECT_EQ(8U, map.getDynamicObservationSeq());
  EXPECT_EQ(7U, map.getESDFObservationSeq());
  EXPECT_EQ(1, map.getOccupancy(new_obstacle));
  const ros::Time next_fusion = map.getReadFusionTime();
  {
    GridMap::ReadScope complete(map, GridMap::ReadScope::Complete);
    ASSERT_TRUE(complete.valid());
    EXPECT_EQ(7U, map.getDynamicObservationSeq());
    EXPECT_EQ(7U, map.getESDFObservationSeq());
    EXPECT_EQ(7U, map.getLidarFusedSeq());
    EXPECT_EQ(ros::Time(107, 0), map.getReadFusionTime());
    EXPECT_EQ(0, map.getOccupancy(new_obstacle));
    EXPECT_DOUBLE_EQ(1.0, map.getDistance(new_obstacle));
    const auto version = map.getReadESDFVersion();

    // Complete generation 7 stays pinned while the sole writer commits 8.
    bool committed = false;
    std::thread writer([&]() {
      committed = Access::waitForAsyncCompletionAndCommit(map, std::chrono::milliseconds(5000));
      if (committed) Access::publishIncrementalSnapshot(map);
    });
    writer.join();
    ASSERT_TRUE(committed);
    EXPECT_EQ(7U, map.getDynamicObservationSeq());
    EXPECT_EQ(version, map.getReadESDFVersion());
    EXPECT_EQ(ros::Time(107, 0), map.getReadFusionTime());
    EXPECT_EQ(0, map.getOccupancy(new_obstacle));
    {
      GridMap::ReadScope latest_complete(map, GridMap::ReadScope::Complete);
      ASSERT_TRUE(latest_complete.valid());
      EXPECT_EQ(8U, map.getDynamicObservationSeq());
      EXPECT_EQ(8U, map.getESDFObservationSeq());
      EXPECT_EQ(next_fusion, map.getReadFusionTime());
      EXPECT_NE(version, map.getReadESDFVersion());
      EXPECT_EQ(1, map.getOccupancy(new_obstacle));
      EXPECT_DOUBLE_EQ(0.0, map.getDistance(new_obstacle));
    }
    EXPECT_EQ(version, map.getReadESDFVersion());
  }
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

class GridMapIncremental : public testing::Test
{
protected:
  using Access = GridMapParallelTestAccessor;
  GridMap incremental_, reference_;
  std::vector<char> occupancy_;
  std::uint64_t sequence_ = 0;

  void SetUp() override
  {
    ensureRosInitialized();
    Access::configureIncrementalTest(incremental_, true);
    Access::configureIncrementalTest(reference_, false);
    occupancy_.assign(61 * 51 * 31, 0);
  }

  static std::size_t cell(int x, int y, int z)
  {
    return (static_cast<std::size_t>(x) * 51 + y) * 31 + z;
  }

  void stage()
  {
    ++sequence_;
    Access::stageIncrementalOccupancy(incremental_, occupancy_, sequence_);
    Access::stageIncrementalOccupancy(reference_, occupancy_, sequence_);
  }

  void expectGlobalReference(const std::vector<double> *before = nullptr)
  {
    const auto &actual = Access::distances(incremental_);
    const auto &full = Access::distances(reference_);
    ASSERT_EQ(full.size(), actual.size());
    const auto lo = Access::writeMinimum(incremental_);
    const auto hi = Access::writeMaximum(incremental_);
    for (int x = 0; x < 61; ++x)
      for (int y = 0; y < 51; ++y)
        for (int z = 0; z < 31; ++z)
        {
          const auto i = cell(x, y, z);
          const double expected = std::max(-0.6, std::min(0.6, full[i]));
          ASSERT_TRUE(std::isfinite(actual[i])) << i;
          ASSERT_NEAR(expected, actual[i], 1e-10)
              << "frame " << sequence_ << " voxel " << x << ',' << y << ',' << z;
          if (before && (x < lo.x() || x > hi.x() || y < lo.y() ||
                         y > hi.y() || z < lo.z() || z > hi.z()))
            ASSERT_DOUBLE_EQ((*before)[i], actual[i])
                << "write outside committed interior at " << x << ',' << y << ',' << z;
        }
  }

  void syncFrame()
  {
    const auto before = Access::distances(incremental_);
    stage();
    Access::processESDF(reference_);
    Access::processESDF(incremental_);
    ASSERT_EQ(sequence_, Access::esdfObservationSeq(incremental_));
    ASSERT_FALSE(Access::esdfNeedsUpdate(incremental_));
    ASSERT_NO_FATAL_FAILURE(expectGlobalReference(&before));
  }
};

TEST_F(GridMapIncremental, EmptyAndFullyOccupiedInitializationMatchCappedGlobalEDT)
{
  syncFrame();
  EXPECT_DOUBLE_EQ(0.6, Access::distances(incremental_)[cell(30, 25, 15)]);
  std::fill(occupancy_.begin(), occupancy_.end(), 1);
  syncFrame();
  EXPECT_DOUBLE_EQ(-0.6, Access::distances(incremental_)[cell(30, 25, 15)]);
  std::fill(occupancy_.begin(), occupancy_.end(), 0);
  syncFrame();
}

TEST_F(GridMapIncremental, InsertionRemovalAndOutsideNeighborPreserveBoundaryDistances)
{
  // This unchanged neighbor is outside the next write interior, but is the
  // nearest source for its outermost voxel. An EDT without its halo is wrong.
  occupancy_[cell(38, 25, 15)] = 1;
  syncFrame();
  occupancy_[cell(30, 25, 15)] = 1;
  syncFrame();
  EXPECT_EQ(Eigen::Vector3i(23, 18, 8), Access::writeMinimum(incremental_));
  EXPECT_EQ(Eigen::Vector3i(37, 32, 22), Access::writeMaximum(incremental_));
  EXPECT_NEAR(0.1, Access::distances(incremental_)[cell(37, 25, 15)], 1e-10);
  occupancy_[cell(30, 25, 15)] = 0;
  syncFrame();
  EXPECT_NEAR(0.5, Access::distances(incremental_)[cell(33, 25, 15)], 1e-10);

  // Map-clipped faces, edges and corners must retain correct linear indexing.
  for (const Eigen::Vector3i &p : {Eigen::Vector3i(0, 0, 0),
                                 Eigen::Vector3i(60, 50, 30)})
  {
    occupancy_[cell(p.x(), p.y(), p.z())] = 1;
    syncFrame();
    occupancy_[cell(p.x(), p.y(), p.z())] = 0;
    syncFrame();
  }
}

TEST_F(GridMapIncremental, ExcavatingAndRefillingSolidObstacleMatchesSignedGlobalEDT)
{
  for (int x = 2; x <= 58; ++x)
    for (int y = 2; y <= 48; ++y)
      for (int z = 2; z <= 28; ++z)
        occupancy_[cell(x, y, z)] = 1;
  syncFrame();
  EXPECT_DOUBLE_EQ(-0.6, Access::distances(incremental_)[cell(30, 25, 15)]);
  occupancy_[cell(30, 25, 15)] = 0;
  syncFrame();
  EXPECT_NEAR(0.1, Access::distances(incremental_)[cell(30, 25, 15)], 1e-10);
  EXPECT_NEAR(-0.1, Access::distances(incremental_)[cell(32, 25, 15)], 1e-10);
  occupancy_[cell(30, 25, 15)] = 1;
  syncFrame();
  EXPECT_DOUBLE_EQ(-0.6, Access::distances(incremental_)[cell(30, 25, 15)]);
}

TEST_F(GridMapIncremental, ActualStaticAndPerimeterClearingPreserveDistancesUntilCommit)
{
  const Eigen::Vector3i center(30, 25, 15);
  const std::vector<Eigen::Vector3i> perimeter = {
      {26, 25, 15}, {34, 25, 15}, {30, 21, 15},
      {30, 29, 15}, {30, 25, 11}, {30, 25, 19}};
  occupancy_[cell(center.x(), center.y(), center.z())] = 1;
  for (const auto &p : perimeter)
    occupancy_[cell(p.x(), p.y(), p.z())] = 1;
  syncFrame();
  const auto before = Access::distances(incremental_);

  ASSERT_EQ(1, Access::clearStaticCellForIncrementalTest(incremental_, center));
  ASSERT_EQ(1, Access::clearStaticCellForIncrementalTest(reference_, center));
  EXPECT_EQ(0, Access::inflatedOccupancy(incremental_, center));
  EXPECT_EQ(before, Access::distances(incremental_));
  EXPECT_DOUBLE_EQ(10000.0, Access::distances(reference_)[cell(30, 25, 15)]);

  Access::clearLocalPerimeterForIncrementalTest(incremental_);
  Access::clearLocalPerimeterForIncrementalTest(reference_);
  for (const auto &p : perimeter)
  {
    // All six perimeter loops must execute. The legacy reference resetting
    // its distance is a positive control for the formerly corrupting write.
    EXPECT_LT(Access::rawOccupancy(incremental_, p), -2.0);
    EXPECT_DOUBLE_EQ(10000.0, Access::distances(reference_)[cell(p.x(), p.y(), p.z())]);
  }
  ASSERT_EQ(before, Access::distances(incremental_));
  ASSERT_EQ(Access::inflatedLayer(reference_), Access::inflatedLayer(incremental_));

  ++sequence_;
  Access::requestESDFAfterLocalClearing(reference_, sequence_);
  Access::requestESDFAfterLocalClearing(incremental_, sequence_);
  Access::processESDF(reference_);
  Access::processESDF(incremental_);
  EXPECT_EQ(sequence_, Access::esdfObservationSeq(incremental_));
  EXPECT_FALSE(Access::esdfNeedsUpdate(incremental_));
  ASSERT_NO_FATAL_FAILURE(expectGlobalReference(&before));
  EXPECT_NEAR(0.4, Access::distances(incremental_)[cell(30, 25, 15)], 1e-10);
}

TEST_F(GridMapIncremental, SeededMultiFrameChangesAcrossMapMatchEveryGlobalVoxel)
{
  occupancy_[cell(7, 8, 9)] = 1;
  occupancy_[cell(52, 41, 22)] = 1;
  syncFrame();
  std::mt19937 random(23);
  const std::vector<Eigen::Vector3i> centers = {
      {14, 13, 12}, {14, 13, 12}, {46, 38, 19}, {46, 38, 19},
      {4, 43, 4}, {4, 43, 4}, {55, 5, 26}, {55, 5, 26}};
  std::size_t local_updates = 0;
  for (const auto &center : centers)
  {
    for (int i = 0; i < 17; ++i)
    {
      const Eigen::Vector3i p = center + Eigen::Vector3i(
          int(random() % 5) - 2, int(random() % 5) - 2, int(random() % 5) - 2);
      auto &occupied = occupancy_[cell(p.x(), p.y(), p.z())];
      occupied = !occupied;
    }
    syncFrame();
    const Eigen::Vector3i size = Access::writeMaximum(incremental_) -
                                Access::writeMinimum(incremental_) + Eigen::Vector3i::Ones();
    if (size.prod() < 61 * 51 * 31) ++local_updates;
  }
  EXPECT_EQ(centers.size(), local_updates);
}

TEST_F(GridMapIncremental, UnchangedBinaryAdvancesSequenceButEpochInvalidationRecomputes)
{
  occupancy_[cell(30, 25, 15)] = 1;
  syncFrame();
  const auto previous = Access::distances(incremental_);
  const auto revision = Access::distanceRevision(incremental_);
  const auto computations = Access::computedCount(incremental_);
  syncFrame();
  EXPECT_EQ(previous, Access::distances(incremental_));
  EXPECT_EQ(revision, Access::distanceRevision(incremental_));
  EXPECT_EQ(computations, Access::computedCount(incremental_));

  // Equal bytes cannot authorize reuse across a reset/static-map epoch.
  Access::invalidateIncrementalEpoch(incremental_);
  syncFrame();
  EXPECT_EQ(Eigen::Vector3i::Zero(), Access::writeMinimum(incremental_));
  EXPECT_EQ(Eigen::Vector3i(60, 50, 30), Access::writeMaximum(incremental_));
  EXPECT_EQ(computations + 1, Access::computedCount(incremental_));
  EXPECT_GT(Access::distanceRevision(incremental_), revision);
}

TEST_F(GridMapIncremental, AsyncCallbackAndUnchangedReusePreservePinnedSnapshot)
{
  occupancy_[cell(38, 25, 15)] = 1;
  syncFrame();
  Access::publishIncrementalSnapshot(incremental_);
  Access::enableIncrementalAsync(incremental_);
  const Eigen::Vector3i query(30, 25, 15);
  GridMap::ReadScope pinned(incremental_);
  const auto old_sequence = incremental_.getESDFObservationSeq();
  const auto old_stamp = Access::pinnedObservationStamp(incremental_);
  const double old_distance = incremental_.getDistance(query);
  const auto before = Access::distances(incremental_);

  occupancy_[cell(30, 25, 15)] = 1;
  stage();
  Access::processESDF(reference_);
  ASSERT_TRUE(Access::waitForIncrementalCallbackCommit(incremental_, sequence_));
  ASSERT_EQ(1U, Access::asyncCommittedCount(incremental_));
  ASSERT_NO_FATAL_FAILURE(expectGlobalReference(&before));
  EXPECT_EQ(old_sequence, incremental_.getESDFObservationSeq());
  EXPECT_EQ(old_stamp, Access::pinnedObservationStamp(incremental_));
  EXPECT_DOUBLE_EQ(old_distance, incremental_.getDistance(query));

  const auto committed_layer = Access::latestDistanceLayer(incremental_);
  const auto submitted = Access::latestSubmittedRequestId(incremental_);
  const auto revision = Access::distanceRevision(incremental_);
  stage();  // Fresh occupancy observation, identical inflated binary geometry.
  Access::processESDF(reference_);
  Access::incrementalCallbackAndPublish(incremental_);
  ASSERT_EQ(sequence_, Access::esdfObservationSeq(incremental_));
  EXPECT_FALSE(Access::esdfNeedsUpdate(incremental_));
  EXPECT_EQ(submitted, Access::latestSubmittedRequestId(incremental_));
  EXPECT_EQ(1U, Access::asyncCommittedCount(incremental_));
  EXPECT_EQ(revision, Access::distanceRevision(incremental_));
  EXPECT_EQ(committed_layer, Access::latestDistanceLayer(incremental_));
  EXPECT_EQ(old_sequence, incremental_.getESDFObservationSeq());
  EXPECT_EQ(old_stamp, Access::pinnedObservationStamp(incremental_));
  EXPECT_DOUBLE_EQ(old_distance, incremental_.getDistance(query));
  ASSERT_NO_FATAL_FAILURE(expectGlobalReference());
  incremental_.refreshReadSnapshot();
  EXPECT_EQ(sequence_, incremental_.getESDFObservationSeq());
  EXPECT_EQ(ros::Time(100 + sequence_, 0), Access::pinnedObservationStamp(incremental_));
  EXPECT_NEAR(0.0, incremental_.getDistance(query), 1e-10);
}

#ifndef MAP_BENCHMARK_OLD_ABI
TEST_F(GridMapIncremental, MappingLoopCommitsWorkerWithoutTimersOrNewCloud)
{
  occupancy_[cell(38, 25, 15)] = 1;
  syncFrame();
  Access::publishIncrementalSnapshot(incremental_);
  Access::enableIncrementalAsync(incremental_);
  occupancy_[cell(30, 25, 15)] = 1;
  stage();
  Access::processESDF(reference_);
  // Submit one ordinary callback before starting the real writer. No ROS
  // timers or subsequent clouds can consume the result in this test.
  Access::incrementalCallbackAndPublish(incremental_);
  const auto fusion = ros::Time(100 + sequence_, 0);
  Access::startMappingLoop(incremental_);
  const bool completed = Access::waitForLoopComplete(
      incremental_, sequence_, std::chrono::milliseconds(1000));
  incremental_.stopMapping();

  ASSERT_TRUE(completed);
  EXPECT_EQ(1U, Access::asyncCommittedCount(incremental_));
  EXPECT_EQ(0U, Access::asyncRejectedCount(incremental_));
  EXPECT_EQ(sequence_, Access::esdfObservationSeq(incremental_));
  ASSERT_NO_FATAL_FAILURE(expectGlobalReference());
  GridMap::ReadScope complete(incremental_, GridMap::ReadScope::Complete);
  ASSERT_TRUE(complete.valid());
  EXPECT_EQ(sequence_, incremental_.getDynamicObservationSeq());
  EXPECT_EQ(sequence_, incremental_.getESDFObservationSeq());
  EXPECT_EQ(fusion, incremental_.getReadFusionTime());
  EXPECT_NEAR(0.0, incremental_.getDistance(Eigen::Vector3i(30, 25, 15)), 1e-10);
}

TEST_F(GridMapIncremental, MappingLoopRejectsReadyOldEpochBeforePublishingComplete)
{
  occupancy_[cell(38, 25, 15)] = 1;
  syncFrame();
  Access::publishIncrementalSnapshot(incremental_);
  Access::enableIncrementalAsync(incremental_);
  const auto previous = Access::distances(incremental_);
  const auto revision = Access::distanceRevision(incremental_);
  occupancy_[cell(30, 25, 15)] = 1;
  stage();
  Access::incrementalCallbackAndPublish(incremental_);
  ASSERT_TRUE(Access::waitForAsyncReady(
      incremental_, 1, std::chrono::milliseconds(5000)));
  Access::advanceEpochWithReadyResult(incremental_);
  Access::startMappingLoop(incremental_);
  const bool published = Access::waitForLoopEpoch(
      incremental_, 2, std::chrono::milliseconds(1000));
  incremental_.stopMapping();

  ASSERT_TRUE(published);
  EXPECT_EQ(0U, Access::asyncCommittedCount(incremental_));
  EXPECT_EQ(1U, Access::asyncRejectedCount(incremental_));
  EXPECT_EQ(previous, Access::distances(incremental_));
  EXPECT_EQ(revision, Access::distanceRevision(incremental_));
  EXPECT_EQ(ros::Time(100 + sequence_, 0), incremental_.getReadFusionTime());
  GridMap::ReadScope complete(incremental_, GridMap::ReadScope::Complete);
  EXPECT_FALSE(complete.valid());
}
#endif

class GridMapSparseIncremental : public testing::Test
{
protected:
  using Access = GridMapParallelTestAccessor;
  const Eigen::Vector3i dimensions_{161, 121, 41};
  const Eigen::Vector3i left_{20, 20, 20}, right_{140, 100, 20};
  GridMap map_, reference_;
  std::vector<char> occupancy_;
  std::uint64_t sequence_ = 1;

  std::size_t cell(const Eigen::Vector3i &p) const
  {
    return (static_cast<std::size_t>(p.x()) * dimensions_.y() + p.y()) *
        dimensions_.z() + p.z();
  }

  void SetUp() override
  {
    ensureRosInitialized();
    Access::configureIncrementalTest(map_, true, dimensions_, 0.1, 3);
    Access::configureIncrementalTest(reference_, false, dimensions_);
    occupancy_.assign(static_cast<std::size_t>(dimensions_.prod()), 0);
    // Unchanged neighbours must remain in each patch's EDT input halo.
    occupancy_[cell(left_ + Eigen::Vector3i(8, 0, 0))] = 1;
    occupancy_[cell(right_ - Eigen::Vector3i(8, 0, 0))] = 1;
    Access::stageIncrementalOccupancy(map_, occupancy_, sequence_);
    Access::stageIncrementalOccupancy(reference_, occupancy_, sequence_);
    Access::processESDF(map_);
    Access::processESDF(reference_);
    Access::publishIncrementalSnapshot(map_);
    Access::enableIncrementalAsync(map_);
  }

  void stage()
  {
    ++sequence_;
    Access::stageIncrementalOccupancy(map_, occupancy_, sequence_);
    Access::stageIncrementalOccupancy(reference_, occupancy_, sequence_);
  }

  void expectReference()
  {
    const auto &actual = Access::distances(map_);
    const auto &full = Access::distances(reference_);
    ASSERT_EQ(actual.size(), full.size());
    for (std::size_t i = 0; i < actual.size(); ++i)
      ASSERT_NEAR(std::max(-0.6, std::min(0.6, full[i])), actual[i], 1e-10)
          << "sequence " << sequence_ << " address " << i;
    EXPECT_EQ(occupancy_, Access::committedOccupancy(map_));
  }

  void completeFrame()
  {
    stage();
    Access::processESDF(reference_);
    ASSERT_TRUE(Access::waitForIncrementalCallbackCommit(map_, sequence_));
    ASSERT_NO_FATAL_FAILURE(expectReference());
  }

  void submitAndWait(std::uint64_t completed)
  {
    stage();
    Access::incrementalCallbackAndPublish(map_);
    ASSERT_TRUE(Access::waitForAsyncReady(map_, completed,
                                         std::chrono::seconds(5)));
  }
};

TEST_F(GridMapSparseIncremental, FarInsertionsRemovalsAndReinsertionsMatchEveryVoxel)
{
  GridMap::ReadScope old(map_, GridMap::ReadScope::Complete);
  ASSERT_TRUE(old.valid());
  const auto old_version = map_.getReadESDFVersion();
  const auto old_stamp = map_.getReadFusionTime();
  const double old_distance = map_.getDistance(left_);
  for (int frame = 0; frame < 3; ++frame)
  {
    occupancy_[cell(left_)] = occupancy_[cell(right_)] = frame != 1;
    ASSERT_NO_FATAL_FAILURE(completeFrame());
    EXPECT_EQ(old_version, map_.getReadESDFVersion());
    EXPECT_EQ(old_stamp, map_.getReadFusionTime());
    EXPECT_DOUBLE_EQ(old_distance, map_.getDistance(left_));
    GridMap::ReadScope current(map_, GridMap::ReadScope::Complete);
    ASSERT_TRUE(current.valid());
    EXPECT_EQ(sequence_, map_.getESDFObservationSeq());
    EXPECT_EQ(ros::Time(100 + sequence_, 0), map_.getReadFusionTime());
    EXPECT_DOUBLE_EQ(Access::distances(map_)[cell(left_)], map_.getDistance(left_));
    EXPECT_DOUBLE_EQ(Access::distances(map_)[cell(right_)], map_.getDistance(right_));
  }
  const auto layer = Access::latestDistanceLayer(map_);
  const auto submitted = Access::latestSubmittedRequestId(map_);
  ASSERT_NO_FATAL_FAILURE(completeFrame());  // Identical geometry, new source.
  EXPECT_EQ(submitted, Access::latestSubmittedRequestId(map_));
  EXPECT_EQ(layer, Access::latestDistanceLayer(map_));
  EXPECT_EQ(3U, Access::asyncCommittedCount(map_));
}

TEST_F(GridMapSparseIncremental, RejectedGenerationAndEpochNeverAdvanceBaselineOrComplete)
{
  const auto initial_layer = Access::latestDistanceLayer(map_);
  const auto initial_baseline = Access::committedOccupancy(map_);
  const auto initial_revision = Access::distanceRevision(map_);
  GridMap::ReadScope old(map_, GridMap::ReadScope::Complete);
  const auto old_version = map_.getReadESDFVersion();
  occupancy_[cell(left_)] = occupancy_[cell(right_)] = 1;
  ASSERT_NO_FATAL_FAILURE(submitAndWait(1));
  Access::changeObservationOnly(map_, ++sequence_);
  Access::commitReadyAndPublish(map_);
  EXPECT_EQ(1U, Access::asyncRejectedCount(map_));
  EXPECT_EQ(0U, Access::asyncCommittedCount(map_));
  EXPECT_EQ(*initial_layer, Access::distances(map_));
  EXPECT_EQ(initial_baseline, Access::committedOccupancy(map_));
  EXPECT_EQ(initial_revision, Access::distanceRevision(map_));
  {
    GridMap::ReadScope complete(map_, GridMap::ReadScope::Complete);
    ASSERT_TRUE(complete.valid());
    EXPECT_EQ(old_version, map_.getReadESDFVersion());
    EXPECT_EQ(1U, map_.getESDFObservationSeq());
  }
  ASSERT_NO_FATAL_FAILURE(completeFrame());  // Rejected changes remain dirty.
  const auto committed_layer = Access::latestDistanceLayer(map_);
  const auto committed_baseline = Access::committedOccupancy(map_);
  const auto committed_epoch = Access::occupancyEpoch(map_);
  const auto committed_revision = Access::distanceRevision(map_);
  occupancy_[cell(left_)] = occupancy_[cell(right_)] = 0;
  ASSERT_NO_FATAL_FAILURE(submitAndWait(3));
  Access::invalidateIncrementalEpoch(map_);
  Access::commitReadyAndPublish(map_);
  EXPECT_EQ(1U, Access::asyncCommittedCount(map_));
  EXPECT_EQ(*committed_layer, Access::distances(map_));
  EXPECT_EQ(committed_baseline, Access::committedOccupancy(map_));
  EXPECT_EQ(committed_epoch, Access::occupancyEpoch(map_));
  EXPECT_EQ(committed_revision, Access::distanceRevision(map_));
  {
    GridMap::ReadScope complete(map_, GridMap::ReadScope::Complete);
    EXPECT_FALSE(complete.valid());  // Old epoch cannot authorize a new reader.
  }
  EXPECT_EQ(old_version, map_.getReadESDFVersion());
  ASSERT_NO_FATAL_FAILURE(completeFrame());
  EXPECT_GT(Access::occupancyEpoch(map_), committed_epoch);
}

#ifndef MAP_BENCHMARK_OLD_ABI
TEST_F(GridMapSparseIncremental, MissingOrNonfiniteLastPatchRejectsWholeBatch)
{
  const auto initial_layer = Access::latestDistanceLayer(map_);
  const auto baseline = Access::committedOccupancy(map_);
  const auto revision = Access::distanceRevision(map_);
  GridMap::ReadScope old(map_, GridMap::ReadScope::Complete);
  const auto version = map_.getReadESDFVersion();
  const auto stamp = map_.getReadFusionTime();
  occupancy_[cell(left_)] = occupancy_[cell(right_)] = 1;
  for (int attempt = 0; attempt < 2; ++attempt)
  {
    ASSERT_NO_FATAL_FAILURE(submitAndWait(attempt + 1));
    const auto stats = Access::readyPatchStats(map_);
    ASSERT_GE(stats.first, 2U);  // Exercise actual bounded sparse worker output.
    ASSERT_LE(stats.first, 8U);
    EXPECT_LT(stats.second, static_cast<std::size_t>(dimensions_.prod()));
    ASSERT_TRUE(Access::damageLastReadyPatch(map_, attempt == 0));
    Access::commitReadyAndPublish(map_);
    EXPECT_EQ(std::uint64_t(attempt + 1), Access::asyncRejectedCount(map_));
    EXPECT_EQ(0U, Access::asyncCommittedCount(map_));
    EXPECT_TRUE(Access::esdfNeedsUpdate(map_));
    // The first patch was valid. It must not have been written before the
    // missing/NaN last patch was detected, nor may the dirty baseline advance.
    EXPECT_EQ(*initial_layer, Access::distances(map_));
    EXPECT_EQ(baseline, Access::committedOccupancy(map_));
    EXPECT_EQ(revision, Access::distanceRevision(map_));
    GridMap::ReadScope complete(map_, GridMap::ReadScope::Complete);
    ASSERT_TRUE(complete.valid());
    EXPECT_EQ(version, map_.getReadESDFVersion());
    EXPECT_EQ(stamp, map_.getReadFusionTime());
    EXPECT_EQ(1U, map_.getESDFObservationSeq());
  }
  ASSERT_NO_FATAL_FAILURE(completeFrame());
  EXPECT_EQ(1U, Access::asyncCommittedCount(map_));
  EXPECT_EQ(2U, Access::asyncRejectedCount(map_));
}
#endif

TEST_F(GridMapIncremental, ByteBlockBoundariesAndFinalPartialBlockRemainObservable)
{
  syncFrame();
  const std::vector<std::size_t> addresses = {
      0, 4095, 4096, 8191, 8192, occupancy_.size() - 1};
  for (const auto i : addresses)
  {
    occupancy_[i] = 1;
    syncFrame();
    occupancy_[i] = 0;
    syncFrame();
    EXPECT_EQ(occupancy_, Access::committedOccupancy(incremental_));
  }
}

// Explicit opt-in synthetic benchmark. Run this single test in separate
// processes against the preserved old library and the new library. It never
// constructs ESDFJob/ESDFResult, whose ABI changes in the sparse worker.
TEST(GridMapBenchmark, Synthetic300x200x60AsyncThreeThreads)
{
  const char *enabled = std::getenv("REMANI_MAP_BENCHMARK");
  if (!enabled || std::strcmp(enabled, "1") != 0)
  {
    std::cout << "Synthetic map benchmark disabled; set REMANI_MAP_BENCHMARK=1."
              << std::endl;
    return;
  }
  ensureRosInitialized();
  using Access = GridMapParallelTestAccessor;
  using Clock = std::chrono::steady_clock;
  const auto milliseconds = [](Clock::time_point from, Clock::time_point to) {
    return std::chrono::duration<double, std::milli>(to - from).count();
  };
  GridMap map;
  const Eigen::Vector3i dimensions(300, 200, 60);
  Access::configureIncrementalTest(map, true, dimensions, 0.05, 3);
  std::vector<char> occupancy(static_cast<std::size_t>(dimensions.prod()), 0);
  std::uint64_t sequence = 0;
  const auto fill = [&](const Eigen::Vector3i &center, char value) {
    for (int x = center.x() - 2; x <= center.x() + 2; ++x)
      for (int y = center.y() - 2; y <= center.y() + 2; ++y)
        for (int z = center.z() - 2; z <= center.z() + 2; ++z)
          occupancy[(static_cast<std::size_t>(x) * dimensions.y() + y) *
                    dimensions.z() + z] = value;
  };
  std::cout << "[map-benchmark] synthetic=1 dimensions=300x200x60 resolution=.05"
               " cap=.6 threads=3; stage_ms is fixture setup, not LiDAR fusion;"
               " callback_ms includes prepare/submit/commit; total includes polling"
            << std::endl;
  std::cout << "[map-benchmark-csv] scenario,round,stage_ms,first_callback_ms,"
               "callback_ms,total_ms,worker_or_sync_ms,commit_ms,input_copy_ms,"
               "publish_ms,distance_sum,submitted,committed" << std::endl;
  const auto run = [&](const char *scenario, int round) {
    const auto stage_start = Clock::now();
    Access::stageIncrementalOccupancy(map, occupancy, ++sequence);
    const auto start = Clock::now();
    const auto timing_before = Access::timingTotals(map);
    double callbacks_ms = 0.0, first_callback_ms = 0.0;
    bool first = true;
    do
    {
      const auto callback_start = Clock::now();
      Access::processESDF(map);
      const double callback_ms = milliseconds(callback_start, Clock::now());
      callbacks_ms += callback_ms;
      if (first) first_callback_ms = callback_ms;
      first = false;
      if (Access::esdfObservationSeq(map) == sequence && !Access::esdfNeedsUpdate(map))
        break;
      ASSERT_LT(milliseconds(start, Clock::now()), 30000.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (true);
    const auto finished = Clock::now();
    const auto timing_after = Access::timingTotals(map);
    Access::publishIncrementalSnapshot(map);
    const auto published = Clock::now();
    long double sum = 0.0;
    for (const double value : Access::distances(map))
    {
      ASSERT_TRUE(std::isfinite(value));
      ASSERT_GE(value, -0.6);
      ASSERT_LE(value, 0.6);
      sum += value;
    }
    EXPECT_EQ(occupancy, Access::committedOccupancy(map));
    std::cout << std::fixed << std::setprecision(6) << "[map-benchmark-csv] "
              << scenario << ',' << round << ',' << milliseconds(stage_start, start)
              << ',' << first_callback_ms << ',' << callbacks_ms << ','
              << milliseconds(start, published) << ','
              << timing_after[0] - timing_before[0] << ','
              << timing_after[1] - timing_before[1] << ','
              << timing_after[2] - timing_before[2] << ','
              << milliseconds(finished, published) << ',' << double(sum) << ','
              << Access::latestSubmittedRequestId(map) << ','
              << Access::asyncCommittedCount(map) << std::endl;
  };
  ASSERT_NO_FATAL_FAILURE(run("initial_sync", 0));
  Access::enableIncrementalAsync(map);
  for (int round = 0; round < 3; ++round)
  {
    fill(Eigen::Vector3i(150, 100, 30), 1);
    ASSERT_NO_FATAL_FAILURE(run("concentrated_insert", round));
    fill(Eigen::Vector3i(50, 40, 30), 1);
    fill(Eigen::Vector3i(250, 160, 30), 1);
    ASSERT_NO_FATAL_FAILURE(run("dispersed_insert", round));
    std::fill(occupancy.begin(), occupancy.end(), 0);
    ASSERT_NO_FATAL_FAILURE(run("dispersed_remove", round));
    ASSERT_NO_FATAL_FAILURE(run("unchanged", round));
    // 37 * 25 * 15 = 13,875 separate changed cells exceed the 8,192-cell
    // sparse bookkeeping limit. Both libraries receive exactly this pattern.
    for (int x = 4; x < dimensions.x(); x += 8)
      for (int y = 4; y < dimensions.y(); y += 8)
        for (int z = 2; z < dimensions.z(); z += 4)
          occupancy[(static_cast<std::size_t>(x) * dimensions.y() + y) *
                    dimensions.z() + z] = 1;
    ASSERT_NO_FATAL_FAILURE(run("widespread_insert", round));
    ASSERT_NO_FATAL_FAILURE(run("widespread_unchanged", round));
    std::fill(occupancy.begin(), occupancy.end(), 0);
    ASSERT_NO_FATAL_FAILURE(run("widespread_remove", round));
  }
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

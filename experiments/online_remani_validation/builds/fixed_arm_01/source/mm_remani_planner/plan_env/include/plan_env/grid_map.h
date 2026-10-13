#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <stdexcept>
#include <tuple>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <plan_env/raycast.h>
#include <plan_env/dynamic_point_buffer.h>
#include <plan_env/esdf_worker.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t>
{
  std::size_t operator()(T const &matrix) const
  {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i)
    {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters
{

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_; // map range in pos
  Eigen::Vector3i map_voxel_num_;                       // map range in index
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;
  int pose_type_;

  /* camera parameters */
  double cx_, cy_, fx_, fy_;

  /* time out */
  double odom_depth_timeout_;
  bool live_lidar_health_monitor_enable_;
  bool live_lidar_health_isolated_worker_;
  double live_lidar_pending_max_age_;
  double live_lidar_message_max_age_;
  double live_lidar_future_tolerance_;
  double live_lidar_timeout_;
  double live_lidar_startup_grace_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_, depth_filter_tolerance_;
  int depth_filter_margin_;
  bool use_depth_filter_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                  // logit of occupancy probability
  double min_ray_length_, max_ray_length_; // range of doing raycasting

  /* local map update and clear */
  int local_map_margin_;

  /* visualization and computation time display */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_, virtual_ceil_yp_, virtual_ceil_yn_;
  double visualization_interval_;
  int esdf_visualization_stride_;
  bool enable_visualization_;
  bool show_occ_time_;

  /* active mapping */
  double unknown_flag_;

  /* esdf */
  double esdf_slice_height_;
  bool show_esdf_time_;
  double local_bound_inflate_;

  /* bounded data parallelism and runtime diagnostics */
  int parallel_threads_;
  int parallel_voxel_threshold_;
  bool enable_performance_diagnostics_;
  double occupancy_warn_ms_;
  double esdf_warn_ms_;
  double performance_log_interval_;

  /* optional asynchronous live ESDF pipeline */
  bool async_esdf_enable_;
  double async_esdf_max_result_age_;

  bool use_global_map_;
  bool fuse_global_map_;
  bool local_esdf_follow_odom_;
  bool global_map_full_esdf_;
  double global_map_obstacles_inflation_;
  bool global_map_lidar_clear_;
  double global_map_lidar_clear_radius_;
  int global_map_lidar_clear_min_misses_;
  double dynamic_static_match_radius_;
  bool live_obstacle_expiry_enable_;
  int live_obstacle_max_missed_fusions_;
  bool self_filter_enable_;
  Eigen::Vector3d self_filter_min_, self_filter_max_;

  /* velocity-aware planning overlay (disabled unless explicitly enabled) */
  bool dynamic_point_overlay_enable_;
  double dynamic_point_overlay_update_interval_;
  double dynamic_point_overlay_horizon_;
  double dynamic_point_overlay_time_step_;
  double dynamic_point_overlay_extra_inflation_;
  double dynamic_point_overlay_max_inflation_;
  double dynamic_point_overlay_influence_distance_;
  bool dynamic_point_isolated_worker_;
};

// intermediate mapping data for fusion

struct MappingData
{
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_neg_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<char> static_occupancy_;
  std::vector<unsigned char> static_miss_count_;
  // Livox sampling does not guarantee that a later free ray traverses the
  // exact 5 cm voxel of an earlier endpoint. Track only non-static live hits
  // across successfully fused scans so departed obstacles cannot leave
  // permanent ghosts and a sensor outage can never erase an obstacle.
  std::vector<std::uint32_t> live_occupancy_last_hit_cycle_;
  std::vector<unsigned char> live_occupancy_active_;
  std::vector<int> active_live_occupancy_indices_;
  std::uint32_t live_fusion_cycle_;
  std::uint64_t expired_live_occupancy_count_;
  ros::Time global_map_stamp_;
  bool global_esdf_pending_;
  bool static_map_esdf_pending_;
  bool static_map_esdf_ready_;
  // A live scan has three explicit generation boundaries: accepted by the
  // callback, fused into occupancy, and represented by the active ESDF.
  // dynamic_observation_seq_ is the planner-facing coherent occupancy
  // revision. It advances for every completed occupancy mutation, while the
  // LiDAR counters retain accepted/pending/fused sensor diagnostics.
  std::uint64_t lidar_accepted_seq_;
  std::uint64_t lidar_pending_seq_;
  std::uint64_t lidar_fused_seq_;
  std::uint64_t dynamic_observation_seq_;
  std::uint64_t esdf_observation_seq_;

  std::vector<double> tmp_buffer1_, tmp_buffer2_;
  std::vector<double> distance_buffer_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_all_;

  // Astar her data

  std::vector<bool> astar_visited_;
  std::vector<double> astar_free_length_;
  std::vector<Eigen::Vector2d> astar_next_state_list_;
  Eigen::Vector2i astar_min_idx_;
  Eigen::Vector2i astar_max_idx_;

  // camera position and pose data

  Eigen::Vector3d camera_pos_, last_camera_pos_;
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
  Eigen::Matrix4d cam2body_;

  // depth image data

  cv::Mat depth_image_, last_depth_image_;
  int image_cnt_;

  // flags of map state

  bool occ_need_update_, local_updated_;
  bool has_first_depth_;
  bool has_odom_, has_cloud_;

  // odom_depth_timeout_
  ros::Time last_occ_update_time_;
  bool flag_depth_odom_timeout_;
  bool flag_use_depth_fusion;
  ros::SteadyTime lidar_health_monitor_start_steady_;
  // Heartbeat of a fresh, transformable LiDAR message containing at least one
  // finite return. Ground/self-filtered frames prove sensor health without
  // falsely clearing occupancy or advancing the fused generation.
  ros::SteadyTime last_lidar_input_steady_;
  ros::SteadyTime last_lidar_fused_steady_;
  ros::SteadyTime pending_lidar_received_steady_;
  ros::Time pending_lidar_stamp_;
  std::uint64_t stale_lidar_discard_count_;
  bool live_lidar_timeout_active_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // Two spatial generations must not share one mutable boundary.  The
  // occupancy bounds describe only the voxels dirtied by the latest raycast;
  // the local bounds describe the ESDF compute/query window.  In the fused
  // global-map mode the latter is owned by the global/odometry policy and must
  // never be overwritten by a LiDAR callback while an async job is running.
  Eigen::Vector3i occupancy_bound_min_, occupancy_bound_max_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  // esdf map
  bool esdf_need_update_;
  double esdf_time_, max_esdf_time_;

  /* wall-clock pipeline diagnostics (one writer: map callback queue) */
  std::uint64_t occupancy_update_count_;
  std::uint64_t esdf_update_count_;
  std::uint64_t coalesced_cloud_count_;
  std::uint64_t async_esdf_submitted_count_;
  std::uint64_t async_esdf_committed_count_;
  std::uint64_t async_esdf_rejected_count_;
  std::uint64_t async_esdf_occupancy_deferred_count_;
  double occupancy_wall_sum_ms_, occupancy_wall_max_ms_;
  double esdf_wall_sum_ms_, esdf_wall_max_ms_;
  double esdf_commit_wall_sum_ms_, esdf_commit_wall_max_ms_;
  double async_esdf_snapshot_wall_sum_ms_;
  double async_esdf_snapshot_wall_max_ms_;
  double async_esdf_last_age_ms_;
  ros::WallTime performance_window_start_;

  // LiDAR origin in world frame (for correct raycasting origin)
  Eigen::Vector3d lidar_origin_;
  bool lidar_origin_valid_;

  // Base orientation (from odometry, used to rotate LiDAR offset to world frame)
  Eigen::Matrix3d base_r_m_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap
{
public:
  GridMap() {}
  ~GridMap();

  enum
  {
    POSE_STAMPED = 1,
    ODOMETRY = 2,
    INVALID_IDX = -10000
  };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  void initAstarBuffer();
  void resetAstarBuffer();

  inline void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  inline Eigen::Vector2i pos2dToIndex(const Eigen::Vector2d &pos);
  inline void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  inline Eigen::Vector2d index2dToPos(const Eigen::Vector2i &id);
  inline int toAddress(const Eigen::Vector3i &id);
  inline int toAddress(int &x, int &y, int &z);
  inline int toAddress2d(int &x, int &y);
  inline bool isInMap(const Eigen::Vector3d &pos);
  inline bool isInMap(const Eigen::Vector2d &pos);
  inline bool isInMap(const Eigen::Vector3i &idx);
  inline bool isInMap(const Eigen::Vector2i &idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos);

  inline void setAstarHeu(const bool visited, Eigen::Vector2i id, const Eigen::Vector2d &next_state, const double length);
  inline void setAstarHeu(const bool visited, Eigen::Vector2i id);
  inline bool getAstarData(const Eigen::Vector2d &pos, Eigen::Vector2d &next_state, double length);

  inline void boundIndex(Eigen::Vector3i &id);
  inline void boundIndex(Eigen::Vector2i &id);
  inline bool isUnknown(const Eigen::Vector3i &id);
  inline bool isUnknown(const Eigen::Vector3d &pos);
  inline bool isKnownFree(const Eigen::Vector3i &id);
  inline bool isKnownOccupied(const Eigen::Vector3i &id);
  inline bool isKnownOccupied(const Eigen::Vector3d &pt);

  void initMap(ros::NodeHandle &nh);

  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishDepth();

  bool hasDepthObservation();
  bool odomValid();
  void getVoxelNum(Eigen::Vector3i &voxel_num);
  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  // int getVoxelNum();
  bool getOdomDepthTimeout() const { return sensorTimeoutNow(); }
  bool mapReadyForPlanning() const
  {
    const bool static_map_ready =
        !mp_.use_global_map_ || !mp_.fuse_global_map_ ||
        md_.static_map_esdf_ready_;
    const bool live_lidar_seen =
        !mp_.live_lidar_health_monitor_enable_ ||
        !md_.last_lidar_input_steady_.isZero();
    return static_map_ready && live_lidar_seen && !sensorTimeoutNow();
  }
  std::uint64_t getDynamicObservationSeq() const { return md_.dynamic_observation_seq_; }
  std::uint64_t getESDFObservationSeq() const { return md_.esdf_observation_seq_; }
  std::uint64_t getLidarAcceptedSeq() const { return md_.lidar_accepted_seq_; }
  std::uint64_t getLidarFusedSeq() const { return md_.lidar_fused_seq_; }
  /**
   * Permanently select the serialized ESDF path for this process and request
   * an update from the latest occupancy generation. This operation is
   * idempotent and is intended for a safety watchdog, not normal scheduling.
   */
  void requestSynchronousESDFFallback();
  bool dynamicPointFlowEnabled() const { return dynamic_point_buffer_.enabled(); }
  double dynamicPointMaxPredictionTime() const
  {
    return dynamic_point_buffer_.maxPredictionTime();
  }
  bool dynamicPointPlanningOverlayEnabled() const
  {
    return mp_.dynamic_point_overlay_enable_;
  }
  bool refreshDynamicPointPlanningOverlay(
      const plan_env::DynamicPointSnapshot &snapshot,
      const ros::Time &now);
  // Freeze one immutable swept-overlay generation for a synchronous planning
  // attempt. The dynamic worker may keep receiving observations, but it cannot
  // replace the KD-tree until endDynamicPointPlanningSnapshot().
  bool beginDynamicPointPlanningSnapshot(
      const plan_env::DynamicPointSnapshot &snapshot,
      const ros::Time &now);
  void endDynamicPointPlanningSnapshot();
  bool dynamicPointPlanningOverlayReadyFor(
      std::uint64_t minimum_sequence) const;
  plan_env::DynamicPointSnapshot getDynamicPointSnapshot(const ros::Time &now) const
  {
    return dynamic_point_buffer_.getSnapshot(now);
  }
  bool getPredictedDynamicObstacleDistance(
      const plan_env::DynamicPointSnapshot &snapshot,
      const Eigen::Vector3d &pos,
      const ros::Time &query_time,
      double max_search_distance,
      double &distance) const
  {
    return dynamic_point_buffer_.getPredictedDistance(
        snapshot, pos, query_time, max_search_distance, distance);
  }
  bool getPredictedDynamicObstacleBounds(
      const plan_env::DynamicPointSnapshot &snapshot,
      const ros::Time &query_time,
      Eigen::Vector3d &minimum,
      Eigen::Vector3d &maximum,
      bool &has_points) const
  {
    return dynamic_point_buffer_.getPredictedBounds(
        snapshot, query_time, minimum, maximum, has_points);
  }
  bool getPredictedDynamicObstaclePositions(
      const plan_env::DynamicPointSnapshot &snapshot,
      const ros::Time &query_time,
      plan_env::PredictedPositionVector &positions,
      double &uncertainty) const
  {
    return dynamic_point_buffer_.getPredictedPositions(
        snapshot, query_time, positions, uncertainty);
  }

  /**
   * Query the short-horizon swept point-flow cloud used only by planning.
   * The returned distance already subtracts the configured prediction
   * uncertainty. False means that no fresh planning overlay is available.
   */
  bool getDynamicPointPlanningDistance(
      const Eigen::Vector3d &pos, double max_search_distance,
      double &distance, Eigen::Vector3d *gradient = nullptr) const;
  double getPlanningDistance(const Eigen::Vector3d &pos, bool precise = false);
  pair<double, Eigen::Vector3d> evaluatePlanningEDTWithGrad(
      const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad);

  // esdf
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  void updateESDF3d();
  void publishESDF();
  inline double getDistance(const Eigen::Vector3d& pos);
  inline double getDistance(const Eigen::Vector3i& id);
  double getPreciseDistance(const Eigen::Vector3d& pos);
  double getDynamicObstacleDistance(const Eigen::Vector3d& pos, double max_search_distance);
  void getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2], Eigen::Vector3d& diff);
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinearEDT(double values[2][2][2], const Eigen::Vector3d& diff, double& value);
  pair<double, Eigen::Vector3d> interpolateTrilinear(double values[2][2][2],const Eigen::Vector3d& diff,
                                                              double& value,Eigen::Vector3d& grad);
  void interpolateTrilinearFirstGrad(double values[2][2][2], const Eigen::Vector3d& diff, Eigen::Vector3d& grad);

  void evaluateEDT(const Eigen::Vector3d& pos, double& dist);
  void evaluateFirstGrad(const Eigen::Vector3d& pos, Eigen::Vector3d& grad);
  pair<double, Eigen::Vector3d> evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                                                    double& dist, Eigen::Vector3d& grad);

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend class GridMapParallelTestAccessor;

  MappingParameters mp_;
  MappingData md_;

  // get depth image and camera pose
  void depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                         const geometry_msgs::PoseStampedConstPtr &pose);
  void extrinsicCallback(const nav_msgs::OdometryConstPtr &odom);
  void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, const nav_msgs::OdometryConstPtr &odom);
  void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const nav_msgs::OdometryConstPtr &odom);
  void liveLidarHealthCallback(
      const sensor_msgs::PointCloud2ConstPtr &cloud);
  void liveLidarHealthOdomCallback(
      const nav_msgs::OdometryConstPtr &odom);
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom);

  // update occupancy by raycasting
  void updateOccupancyCallback(const ros::TimerEvent & /*event*/);
  void visCallback(const ros::TimerEvent & /*event*/);
  void updateESDFCallback(const ros::TimerEvent& /*event*/);
  void updateDynamicPointOverlayCallback(const ros::TimerEvent& /*event*/);

  // main update process
  void projectDepthImage();
  void raycastProcess(bool live_lidar_observation = false);
  std::size_t expireStaleLiveOccupancy(std::uint32_t current_cycle);
  void clearAndInflateLocalMap();
  void clearStaticGlobalMap();
  int clearStaticOccupancyAround(const Eigen::Vector3i &center, int clear_step);
  void updateLocalBoundsAroundRobot(bool force_esdf_update = false);
  void noteLidarHealthMonitorStart();
  void updateSensorTimeoutState();
  bool sensorTimeoutNow() const;
  bool liveLidarMessageStampFresh(const ros::Time &stamp,
                                  const char *source) const;
  ros::SteadyTime latestLiveLidarHealthTime() const;
  bool hasPendingLidarObservation() const;
  void clearPendingLidarObservation();
  void maybeLogPerformanceDiagnostics();
  bool fusedGlobalMapOwnsESDFBounds() const;
  bool validOccupancyBounds() const;
  bool validESDFBounds() const;
  bool asyncESDFUpdateOutstanding() const;
  bool submitAsyncESDFUpdate();
  void commitAsyncESDFResult();
  void invalidateAsyncESDFJobs();
  void recordESDFTiming(double compute_ms, double commit_ms,
                        double result_age_ms, bool asynchronous);

  inline void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

  // typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // nav_msgs::Odometry> SyncPolicyImageOdom; typedef
  // message_filters::sync_policies::ExactTime<sensor_msgs::Image,
  // geometry_msgs::PoseStamped> SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
      SyncPolicyImageOdom;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped>
      SyncPolicyImagePose;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
      SyncPolicyCloudOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImagePose>> SynchronizerImagePose;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> SynchronizerImageOdom;
  typedef shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;

  ros::NodeHandle node_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
  shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
  shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
  shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
  SynchronizerImagePose sync_image_pose_;
  SynchronizerImageOdom sync_image_odom_;
  SynchronizerCloudOdom sync_cloud_odom_;

  ros::Subscriber indep_cloud_sub_, indep_odom_sub_, extrinsic_sub_;
  ros::Publisher map_pub_, map_inf_pub_, esdf_pub_, dynamic_point_overlay_pub_;
  ros::Publisher dynamic_prediction_nominal_pub_;
  ros::Publisher dynamic_prediction_markers_pub_;
  ros::Timer occ_timer_, vis_timer_, esdf_timer_, dynamic_point_overlay_timer_;

  // TF2 — used by cloudOdomCallback to transform points at the exact scan timestamp
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // A planning/optimization callback can occupy the legacy ROS queue for
  // hundreds of milliseconds. This queue observes only LiDAR transport
  // health, and never mutates occupancy, ESDF, or planner-visible map state.
  // The map therefore keeps its single-writer invariant while the watchdog
  // remains responsive during synchronous planning.
  ros::CallbackQueue lidar_health_callback_queue_;
  std::unique_ptr<ros::NodeHandle> lidar_health_node_;
  std::unique_ptr<ros::AsyncSpinner> lidar_health_spinner_;
  ros::Subscriber lidar_health_sub_, lidar_health_odom_sub_;
  std::atomic<std::uint64_t> lidar_health_steady_ns_{0};
  std::atomic<std::uint64_t> lidar_health_odom_steady_ns_{0};

  // Dynamic prediction is isolated from the legacy map/FSM callback queue.
  // Its shared products are immutable snapshots or mutex-protected overlays.
  ros::CallbackQueue dynamic_callback_queue_;
  std::unique_ptr<ros::NodeHandle> dynamic_node_;
  std::unique_ptr<ros::AsyncSpinner> dynamic_spinner_;
  plan_env::DynamicPointBuffer dynamic_point_buffer_;
  mutable std::mutex dynamic_point_overlay_mutex_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_point_overlay_cloud_;
  mutable pcl::KdTreeFLANN<pcl::PointXYZ> dynamic_point_overlay_kdtree_;
  // Radius aligned one-to-one with dynamic_point_overlay_cloud_. Unlike the
  // old single worst-horizon radius, this preserves the uncertainty of each
  // prediction time slice.
  std::vector<float> dynamic_point_overlay_radii_;
  double dynamic_point_overlay_inflation_ = 0.0;
  Eigen::Vector3d dynamic_point_overlay_min_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d dynamic_point_overlay_max_ = Eigen::Vector3d::Zero();
  std::uint64_t dynamic_point_overlay_sequence_ = 0;
  ros::Time dynamic_point_overlay_refresh_time_;
  bool dynamic_point_overlay_ready_ = false;
  bool dynamic_point_overlay_frozen_ = false;
  std::uint64_t dynamic_point_overlay_frozen_sequence_ = 0;

  // Only live/local ESDF updates use this worker. Occupancy mutation, static
  // PCD loading, planning queries and result publication remain serialized on
  // the default ROS callback queue.
  std::unique_ptr<plan_env::ESDFWorker> esdf_worker_;
  std::uint64_t esdf_map_epoch_ = 1;
  std::uint64_t esdf_next_request_id_ = 0;
  std::uint64_t esdf_latest_submitted_request_id_ = 0;
  // Unlike committed_request_id_, this advances for both accepted and
  // rejected/cancelled results. It is the single-flight backpressure fence
  // which prevents live occupancy from invalidating an in-flight snapshot.
  std::uint64_t esdf_last_resolved_request_id_ = 0;
  std::uint64_t esdf_committed_request_id_ = 0;
  bool async_esdf_fallback_to_sync_ = false;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i &id)
{
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int &x, int &y, int &z)
{
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline int GridMap::toAddress2d(int &x, int &y)
{
  return x * mp_.map_voxel_num_(1) + y;
}

inline void GridMap::boundIndex(Eigen::Vector3i &id)
{
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline void GridMap::boundIndex(Eigen::Vector2i &id)
{
  Eigen::Vector2i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d &pos)
{
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i &id)
{
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3d &pt)
{
  if(!isInMap(pt)){
    return true;
  }
  Eigen::Vector3i id1;
  posToIndex(pt, id1);
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_inflate_[id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
                                id(1) * mp_.map_voxel_num_(2) + id(2)] = 1;
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ)
{
  if (occ != 1 && occ != 0)
  {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos))
    return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  md_.occupancy_buffer_[toAddress(id)] = occ;
}

inline void GridMap::setAstarHeu(const bool visited, Eigen::Vector2i id, const Eigen::Vector2d &next_state, const double length){
  md_.astar_visited_[toAddress2d(id(0), id(1))] = visited;
  if(visited){
    md_.astar_free_length_[toAddress2d(id(0), id(1))] = length;
    md_.astar_next_state_list_[toAddress2d(id(0), id(1))] = next_state;
    md_.astar_min_idx_(0) = std::min(id(0), md_.astar_min_idx_(0));
    md_.astar_min_idx_(1) = std::min(id(1), md_.astar_min_idx_(1));
    md_.astar_max_idx_(0) = std::max(id(0), md_.astar_max_idx_(0));
    md_.astar_max_idx_(1) = std::max(id(1), md_.astar_max_idx_(1));
  }
}

inline void GridMap::setAstarHeu(const bool visited, Eigen::Vector2i id){
  md_.astar_visited_[toAddress2d(id(0), id(1))] = visited;
}

inline bool GridMap::getAstarData(const Eigen::Vector2d &pos, Eigen::Vector2d &next_state, double length){
  Eigen::Vector2i id = pos2dToIndex(pos);
  bool visited = md_.astar_visited_[toAddress2d(id(0), id(1))];
  if(visited){
    next_state = md_.astar_next_state_list_[toAddress2d(id(0), id(1))];
    length = md_.astar_free_length_[toAddress2d(id(0), id(1))];
  }
  return visited;
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
  if (!isInMap(pos))
    return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline int GridMap::getOccupancy(Eigen::Vector3i id)
{
  if (id(0) < 0 || id(0) >= mp_.map_voxel_num_(0) || id(1) < 0 || id(1) >= mp_.map_voxel_num_(1) ||
      id(2) < 0 || id(2) >= mp_.map_voxel_num_(2))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector2d &pos)
{
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i &idx)
{
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1)
  {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector2i &idx)
{
  if (idx(0) < 0 || idx(1) < 0)
  {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1)
  {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline Eigen::Vector2i GridMap::pos2dToIndex(const Eigen::Vector2d &pos){
  Eigen::Vector2i id;
  for (int i = 0; i < 2; ++i)
    id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
  return id;
}

inline void GridMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline Eigen::Vector2d GridMap::index2dToPos(const Eigen::Vector2i &id)
{
  Eigen::Vector2d pos;
  for(int i = 0; i < 2; ++i)
    pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
  return pos;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts)
{
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  //   for (int y = -step; y <= step; ++y)
  //     for (int z = -step; z <= step; ++z)
  //     {
  //       pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
  //     }
  // do not infloat z
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
    {
      pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2));
    }

  // for (int y = -step; y <= step; ++y)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

inline double GridMap::getDistance(const Eigen::Vector3d& pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  boundIndex(id);

  return md_.distance_buffer_all_[toAddress(id)];
}

inline double GridMap::getDistance(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.distance_buffer_all_[toAddress(id1)];
}

#endif

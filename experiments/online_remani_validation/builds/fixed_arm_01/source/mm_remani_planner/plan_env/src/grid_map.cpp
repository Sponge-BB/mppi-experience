#include "plan_env/grid_map.h"

#include <algorithm>
#include <map>
#include <sstream>
#include <thread>

namespace
{

template <typename Function>
void boundedParallelFor(std::size_t item_count,
                        int requested_threads,
                        int parallel_threshold,
                        const Function &function)
{
  if (item_count == 0)
    return;

  int worker_count = std::max(1, requested_threads);
  worker_count = std::min(worker_count, static_cast<int>(item_count));
  if (worker_count == 1 ||
      item_count < static_cast<std::size_t>(std::max(1, parallel_threshold)))
  {
    for (std::size_t index = 0; index < item_count; ++index)
      function(index);
    return;
  }

  const std::size_t block_size =
      (item_count + static_cast<std::size_t>(worker_count) - 1) /
      static_cast<std::size_t>(worker_count);
  std::vector<std::thread> workers;
  workers.reserve(static_cast<std::size_t>(worker_count - 1));

  // The caller handles one block, so requested_threads is a hard upper bound
  // on CPU concurrency instead of the number of additional threads.
  for (int worker = 1; worker < worker_count; ++worker)
  {
    const std::size_t begin = static_cast<std::size_t>(worker) * block_size;
    const std::size_t end = std::min(item_count, begin + block_size);
    if (begin >= end)
      break;
    workers.emplace_back([begin, end, &function]() {
      for (std::size_t index = begin; index < end; ++index)
        function(index);
    });
  }

  const std::size_t caller_end = std::min(item_count, block_size);
  for (std::size_t index = 0; index < caller_end; ++index)
    function(index);

  for (auto &worker : workers)
    worker.join();
}

bool validIndexBounds(const Eigen::Vector3i &minimum,
                      const Eigen::Vector3i &maximum,
                      const Eigen::Vector3i &voxel_count)
{
  return (voxel_count.array() > 0).all() &&
         (minimum.array() >= 0).all() &&
         (minimum.array() <= maximum.array()).all() &&
         (maximum.array() < voxel_count.array()).all();
}

}  // namespace

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

GridMap::~GridMap()
{
  occ_timer_.stop();
  esdf_timer_.stop();
  vis_timer_.stop();
  dynamic_point_overlay_timer_.stop();

  if (lidar_health_spinner_)
    lidar_health_spinner_->stop();
  lidar_health_sub_.shutdown();
  lidar_health_odom_sub_.shutdown();
  lidar_health_callback_queue_.disable();
  lidar_health_callback_queue_.clear();

  // The ESDF worker owns no GridMap storage, but it must still be joined before
  // this object's buffers are destroyed. Never detach a map-computation thread.
  if (esdf_worker_)
  {
    esdf_worker_->stop();
    esdf_worker_.reset();
  }
  if (dynamic_spinner_)
    dynamic_spinner_->stop();
  dynamic_callback_queue_.disable();
  dynamic_callback_queue_.clear();
}

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  // TF2 — used by cloudOdomCallback to look up transforms at the exact scan timestamp
  tf_buffer_.reset(new tf2_ros::Buffer());
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node_.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node_.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);
  node_.param("grid_map/visualization_interval", mp_.visualization_interval_, 0.05);
  node_.param("grid_map/esdf_visualization_stride", mp_.esdf_visualization_stride_, 1);
  node_.param("grid_map/enable_visualization", mp_.enable_visualization_, true);
  mp_.visualization_interval_ = std::max(0.05, mp_.visualization_interval_);
  mp_.esdf_visualization_stride_ = std::max(1, mp_.esdf_visualization_stride_);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 2);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 0.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 14.0);
  node_.param("grid_map/live_lidar_health_monitor_enable",
              mp_.live_lidar_health_monitor_enable_, false);
  node_.param("grid_map/live_lidar_health_isolated_worker",
              mp_.live_lidar_health_isolated_worker_, true);
  node_.param("grid_map/live_lidar_pending_max_age",
              mp_.live_lidar_pending_max_age_, 0.35);
  node_.param("grid_map/live_lidar_message_max_age",
              mp_.live_lidar_message_max_age_, 0.50);
  node_.param("grid_map/live_lidar_future_tolerance",
              mp_.live_lidar_future_tolerance_, 0.05);
  node_.param("grid_map/live_lidar_timeout",
              mp_.live_lidar_timeout_, 0.75);
  node_.param("grid_map/live_lidar_startup_grace",
              mp_.live_lidar_startup_grace_, 5.0);
  mp_.live_lidar_pending_max_age_ =
      std::max(0.05, mp_.live_lidar_pending_max_age_);
  mp_.live_lidar_message_max_age_ =
      std::max(0.05, mp_.live_lidar_message_max_age_);
  mp_.live_lidar_future_tolerance_ =
      std::max(0.0, mp_.live_lidar_future_tolerance_);
  mp_.live_lidar_timeout_ = std::max(
      mp_.live_lidar_pending_max_age_, mp_.live_lidar_timeout_);
  mp_.live_lidar_startup_grace_ =
      std::max(mp_.live_lidar_timeout_, mp_.live_lidar_startup_grace_);
  // add esdf
  node_.param("grid_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
  node_.param("grid_map/show_esdf_time", mp_.show_esdf_time_, false);
  node_.param("grid_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
  node_.param("grid_map/parallel_threads", mp_.parallel_threads_, 2);
  node_.param("grid_map/parallel_voxel_threshold",
              mp_.parallel_voxel_threshold_, 50000);
  node_.param("grid_map/enable_performance_diagnostics",
              mp_.enable_performance_diagnostics_, true);
  node_.param("grid_map/occupancy_warn_ms", mp_.occupancy_warn_ms_, 50.0);
  node_.param("grid_map/esdf_warn_ms", mp_.esdf_warn_ms_, 50.0);
  node_.param("grid_map/performance_log_interval",
              mp_.performance_log_interval_, 5.0);
  node_.param("grid_map/async_esdf_enable",
              mp_.async_esdf_enable_, false);
  node_.param("grid_map/async_esdf_max_result_age",
              mp_.async_esdf_max_result_age_, 0.75);
  mp_.parallel_threads_ = std::max(1, std::min(4, mp_.parallel_threads_));
  mp_.parallel_voxel_threshold_ =
      std::max(1000, mp_.parallel_voxel_threshold_);
  mp_.occupancy_warn_ms_ = std::max(1.0, mp_.occupancy_warn_ms_);
  mp_.esdf_warn_ms_ = std::max(1.0, mp_.esdf_warn_ms_);
  mp_.performance_log_interval_ =
      std::max(1.0, mp_.performance_log_interval_);
  mp_.async_esdf_max_result_age_ =
      std::max(0.10, mp_.async_esdf_max_result_age_);

  node_.param("grid_map/use_global_map", mp_.use_global_map_, false);
  node_.param("grid_map/fuse_global_map", mp_.fuse_global_map_, true);
  node_.param("grid_map/local_esdf_follow_odom", mp_.local_esdf_follow_odom_, true);
  node_.param("grid_map/global_map_full_esdf", mp_.global_map_full_esdf_, true);
  node_.param("grid_map/global_map_obstacles_inflation", mp_.global_map_obstacles_inflation_, mp_.obstacles_inflation_);
  node_.param("grid_map/global_map_lidar_clear", mp_.global_map_lidar_clear_, true);
  node_.param("grid_map/global_map_lidar_clear_radius", mp_.global_map_lidar_clear_radius_, 0.0);
  node_.param("grid_map/global_map_lidar_clear_min_misses", mp_.global_map_lidar_clear_min_misses_, 3);
  node_.param("grid_map/dynamic_static_match_radius", mp_.dynamic_static_match_radius_, 0.075);
  node_.param("grid_map/live_obstacle_expiry_enable",
              mp_.live_obstacle_expiry_enable_, true);
  node_.param("grid_map/live_obstacle_max_missed_fusions",
              mp_.live_obstacle_max_missed_fusions_, 4);
  if (mp_.global_map_lidar_clear_min_misses_ < 1)
    mp_.global_map_lidar_clear_min_misses_ = 1;
  mp_.dynamic_static_match_radius_ = std::max(0.0, mp_.dynamic_static_match_radius_);
  mp_.live_obstacle_max_missed_fusions_ =
      std::max(1, mp_.live_obstacle_max_missed_fusions_);
  node_.param("grid_map/self_filter_enable", mp_.self_filter_enable_, false);
  node_.param("grid_map/self_filter_min_x", mp_.self_filter_min_(0), -0.8);
  node_.param("grid_map/self_filter_max_x", mp_.self_filter_max_(0), 0.45);
  node_.param("grid_map/self_filter_min_y", mp_.self_filter_min_(1), -0.85);
  node_.param("grid_map/self_filter_max_y", mp_.self_filter_max_(1), 0.55);
  node_.param("grid_map/self_filter_min_z", mp_.self_filter_min_(2), -0.20);
  node_.param("grid_map/self_filter_max_z", mp_.self_filter_max_(2), 0.90);
  node_.param("dynamic_point_flow/planning_overlay_enable",
              mp_.dynamic_point_overlay_enable_, false);
  node_.param("dynamic_point_flow/planning_overlay_update_interval",
              mp_.dynamic_point_overlay_update_interval_, 0.10);
  node_.param("dynamic_point_flow/planning_overlay_horizon",
              mp_.dynamic_point_overlay_horizon_, 1.0);
  node_.param("dynamic_point_flow/planning_overlay_time_step",
              mp_.dynamic_point_overlay_time_step_, 0.20);
  node_.param("dynamic_point_flow/planning_overlay_extra_inflation",
              mp_.dynamic_point_overlay_extra_inflation_, 0.03);
  node_.param("dynamic_point_flow/planning_overlay_max_inflation",
              mp_.dynamic_point_overlay_max_inflation_, 0.25);
  node_.param("dynamic_point_flow/planning_overlay_influence_distance",
              mp_.dynamic_point_overlay_influence_distance_, 0.75);
  node_.param("dynamic_point_flow/isolated_worker_enable",
              mp_.dynamic_point_isolated_worker_, true);
  bool dynamic_point_active_mode = false;
  node_.param("dynamic_point_flow/active_mode",
              dynamic_point_active_mode, false);
  mp_.dynamic_point_overlay_update_interval_ =
      std::max(0.05, mp_.dynamic_point_overlay_update_interval_);
  mp_.dynamic_point_overlay_horizon_ =
      std::max(0.0, mp_.dynamic_point_overlay_horizon_);
  mp_.dynamic_point_overlay_time_step_ =
      std::max(0.05, mp_.dynamic_point_overlay_time_step_);
  mp_.dynamic_point_overlay_extra_inflation_ =
      std::max(0.0, mp_.dynamic_point_overlay_extra_inflation_);
  mp_.dynamic_point_overlay_max_inflation_ = std::max(
      mp_.dynamic_point_overlay_extra_inflation_,
      mp_.dynamic_point_overlay_max_inflation_);
  mp_.dynamic_point_overlay_influence_distance_ = std::max(
      0.25, mp_.dynamic_point_overlay_influence_distance_);
  Eigen::Vector3d lidar_offset;
  node_.param("grid_map/lidar_offset_x", lidar_offset(0), 0.28);
  node_.param("grid_map/lidar_offset_y", lidar_offset(1), 0.0);
  node_.param("grid_map/lidar_offset_z", lidar_offset(2), 0.002);
  bool global_plan;
  node_.param("fsm/global_plan", global_plan, false);
  if(global_plan){
    mp_.use_global_map_ = true;
  }
  bool use_load_map_;
  nh.param("grid_map/use_load_map", use_load_map_, false);

  // Dynamic point flow is an independent, time-aware layer. It is disabled by
  // default and therefore cannot change the existing occupancy/ESDF behavior.
  dynamic_node_.reset(new ros::NodeHandle(node_));
  if (mp_.dynamic_point_isolated_worker_)
    dynamic_node_->setCallbackQueue(&dynamic_callback_queue_);
  dynamic_point_buffer_.init(*dynamic_node_, mp_.frame_id_, tf_buffer_);
  if (mp_.dynamic_point_overlay_enable_ && !dynamic_point_buffer_.enabled())
  {
    ROS_ERROR("[DynamicPointFlow][overlay] planning_overlay_enable requires dynamic_point_flow/enable; disabling overlay.");
    mp_.dynamic_point_overlay_enable_ = false;
  }
  if (mp_.dynamic_point_overlay_enable_ && !dynamic_point_active_mode)
  {
    ROS_ERROR("[DynamicPointFlow][overlay] planning_overlay_enable is only valid with active_mode=true; disabling overlay so shadow/A-B modes cannot change planning.");
    mp_.dynamic_point_overlay_enable_ = false;
  }
  dynamic_point_overlay_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

  if(mp_.local_bound_inflate_ > 1e-3) mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
  else mp_.local_bound_inflate_ = 0.0;

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  // cout << "hit: " << mp_.prob_hit_log_ << endl;
  // cout << "miss: " << mp_.prob_miss_log_ << endl;
  // cout << "min log: " << mp_.clamp_min_log_ << endl;
  // cout << "max: " << mp_.clamp_max_log_ << endl;
  // cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  if(mp_.virtual_ceil_height_ >= z_size + mp_.ground_height_ )
  {
    mp_.virtual_ceil_height_ = z_size + mp_.ground_height_ - mp_.resolution_;
  }

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.static_occupancy_ = vector<char>(buffer_size, 0);
  md_.static_miss_count_ = vector<unsigned char>(buffer_size, 0);
  if (mp_.live_obstacle_expiry_enable_)
  {
    md_.live_occupancy_last_hit_cycle_ =
        vector<std::uint32_t>(buffer_size, 0);
    md_.live_occupancy_active_ =
        vector<unsigned char>(buffer_size, 0);
  }
  md_.active_live_occupancy_indices_.clear();
  md_.live_fusion_cycle_ = 0;
  md_.expired_live_occupancy_count_ = 0;
  md_.global_map_stamp_ = ros::Time(0);
  md_.global_esdf_pending_ = false;
  md_.static_map_esdf_pending_ = false;
  md_.static_map_esdf_ready_ = false;
  md_.lidar_accepted_seq_ = 0;
  md_.lidar_pending_seq_ = 0;
  md_.lidar_fused_seq_ = 0;
  md_.dynamic_observation_seq_ = 0;
  md_.esdf_observation_seq_ = 0;

  md_.astar_visited_ = vector<bool>(mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1), false);
  md_.astar_free_length_ = vector<double>(mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1), 1e10);
  md_.astar_next_state_list_ = vector<Eigen::Vector2d>(mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1));
  md_.astar_min_idx_.setZero();
  md_.astar_max_idx_(0) = mp_.map_voxel_num_(0) - 1;
  md_.astar_max_idx_(1) = mp_.map_voxel_num_(1) - 1;

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.occupancy_buffer_neg_ = vector<char>(buffer_size, 0);
  md_.distance_buffer_ = vector<double>(buffer_size, 10000);//10000
  md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
  md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);
  md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
  md_.tmp_buffer2_ = vector<double>(buffer_size, 0);

  md_.raycast_num_ = 0;

  // md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_.resize(21000);
  md_.proj_points_cnt = 0;

  // md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
  //                 -1.0, 0.0, 0.0, 0.0,
  //                 0.0, -1.0, 0.0, 0.0,
  //                 0.0, 0.0, 0.0, 1.0;

  // LiDAR mount relative to base_link. This is only the odometry fallback in
  // cloudOdomCallback; the timestamped TF lookup remains the primary path.
  md_.cam2body_ << 1.0,  0.0, 0.0, lidar_offset(0),
                   0.0,  1.0, 0.0, lidar_offset(1),
                   0.0,  0.0, 1.0, lidar_offset(2),
                   0.0,  0.0, 0.0, 1.0;

  /* initialize local bounds to full map extent before any callbacks fire */
  resetBuffer();

  /* init callback */
  
  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "grid_map/depth", 50));
  // This is the live LiDAR path. A deep queue makes obstacle observations stale
  // under load, while the static global-map subscription below remains unchanged.
  cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "grid_map/cloud", 1));
  // extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
  //     "/vins_estimator/extrinsic", 10, &GridMap::extrinsicCallback, this); //sub
  
  if (mp_.pose_type_ == POSE_STAMPED)
  {
    pose_sub_.reset(
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "grid_map/pose", 25));

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
    sync_image_pose_->registerCallback(boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
  }
  else if (mp_.pose_type_ == ODOMETRY)
  {
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));

    // sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
    //     SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
    // sync_image_odom_->registerCallback(boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
        SyncPolicyCloudOdom(5), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&GridMap::cloudOdomCallback, this, _1, _2));
  }
  
  // use odometry and point cloud 
  // sim
  if(use_load_map_){
    // std::cout << "use load map or sim" << std::endl;
    if(mp_.use_global_map_){
      // std::cout << "use global map" << std::endl;
      indep_cloud_sub_ =
         node_.subscribe<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 10, &GridMap::cloudCallback, this);
    }else{
      // std::cout << "use local map" << std::endl;
      indep_cloud_sub_ =
        node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this);
    }
    
  }

  // Keep a lightweight odometry heartbeat independent of cloud/odom
  // synchronization. Otherwise a completely missing LiDAR prevents the
  // synchronizer from ever starting the live-sensor startup watchdog.
  indep_odom_sub_ = node_.subscribe<nav_msgs::Odometry>(
      "grid_map/odom", 1, &GridMap::odomCallback, this,
      ros::TransportHints().tcpNoDelay());

  if (mp_.live_lidar_health_monitor_enable_ &&
      mp_.live_lidar_health_isolated_worker_)
  {
    lidar_health_node_.reset(new ros::NodeHandle(node_));
    lidar_health_node_->setCallbackQueue(&lidar_health_callback_queue_);
    lidar_health_sub_ = lidar_health_node_->subscribe<sensor_msgs::PointCloud2>(
        "grid_map/cloud", 1, &GridMap::liveLidarHealthCallback, this,
        ros::TransportHints().tcpNoDelay());
    lidar_health_odom_sub_ = lidar_health_node_->subscribe<nav_msgs::Odometry>(
        "grid_map/odom", 1, &GridMap::liveLidarHealthOdomCallback, this,
        ros::TransportHints().tcpNoDelay());
  }
  
  
  occ_timer_  = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
  esdf_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateESDFCallback, this);
  if (mp_.dynamic_point_overlay_enable_)
  {
    dynamic_point_overlay_timer_ = dynamic_node_->createTimer(
        ros::Duration(mp_.dynamic_point_overlay_update_interval_),
        &GridMap::updateDynamicPointOverlayCallback, this);
  }
  vis_timer_  = node_.createTimer(
      ros::Duration(mp_.visualization_interval_), &GridMap::visCallback, this);
  
  map_pub_      = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_  = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);
  esdf_pub_     = node_.advertise<sensor_msgs::PointCloud2>("grid_map/esdf", 10);
  dynamic_point_overlay_pub_ = node_.advertise<sensor_msgs::PointCloud2>(
      "grid_map/dynamic_point_swept", 1, true);
  dynamic_prediction_nominal_pub_ = node_.advertise<sensor_msgs::PointCloud2>(
      "grid_map/dynamic_prediction_nominal", 1, true);
  dynamic_prediction_markers_pub_ =
      node_.advertise<visualization_msgs::MarkerArray>(
          "grid_map/dynamic_prediction_markers", 1, true);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);
  md_.lidar_health_monitor_start_steady_ = ros::SteadyTime();
  md_.last_lidar_input_steady_ = ros::SteadyTime();
  md_.last_lidar_fused_steady_ = ros::SteadyTime();
  md_.pending_lidar_received_steady_ = ros::SteadyTime();
  md_.pending_lidar_stamp_ = ros::Time(0);
  md_.stale_lidar_discard_count_ = 0;
  md_.live_lidar_timeout_active_ = false;
  lidar_health_steady_ns_.store(0, std::memory_order_relaxed);
  lidar_health_odom_steady_ns_.store(0, std::memory_order_relaxed);
  md_.esdf_need_update_ = false;
  md_.lidar_origin_valid_ = false;
  md_.camera_r_m_.setIdentity();
  md_.base_r_m_.setIdentity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.esdf_time_ = 0.0;
  md_.max_esdf_time_ = 0.0;
  md_.occupancy_update_count_ = 0;
  md_.esdf_update_count_ = 0;
  md_.coalesced_cloud_count_ = 0;
  md_.async_esdf_submitted_count_ = 0;
  md_.async_esdf_committed_count_ = 0;
  md_.async_esdf_rejected_count_ = 0;
  md_.async_esdf_occupancy_deferred_count_ = 0;
  md_.occupancy_wall_sum_ms_ = 0.0;
  md_.occupancy_wall_max_ms_ = 0.0;
  md_.esdf_wall_sum_ms_ = 0.0;
  md_.esdf_wall_max_ms_ = 0.0;
  md_.esdf_commit_wall_sum_ms_ = 0.0;
  md_.esdf_commit_wall_max_ms_ = 0.0;
  md_.async_esdf_snapshot_wall_sum_ms_ = 0.0;
  md_.async_esdf_snapshot_wall_max_ms_ = 0.0;
  md_.async_esdf_last_age_ms_ = 0.0;
  md_.performance_window_start_ = ros::WallTime::now();

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());

  if (dynamic_point_buffer_.enabled() &&
      mp_.dynamic_point_isolated_worker_)
  {
    dynamic_spinner_.reset(
        new ros::AsyncSpinner(1, &dynamic_callback_queue_));
    dynamic_spinner_->start();
    ROS_INFO("[GridMap] Dynamic prediction callbacks use an isolated worker queue.");
  }
  else if (dynamic_point_buffer_.enabled())
  {
    ROS_WARN("[GridMap] Dynamic prediction callbacks share the legacy callback queue.");
  }

  if (lidar_health_sub_)
  {
    lidar_health_spinner_.reset(
        new ros::AsyncSpinner(1, &lidar_health_callback_queue_));
    lidar_health_spinner_->start();
    ROS_INFO("[GridMap][LiDAR] Health heartbeat uses an isolated callback queue; occupancy and ESDF remain single-writer serialized.");
  }

  if (mp_.async_esdf_enable_)
  {
    esdf_worker_.reset(new plan_env::ESDFWorker(
        mp_.parallel_threads_, mp_.parallel_voxel_threshold_));
    ROS_INFO("[GridMap][AsyncESDF] Enabled for live/local updates: "
             "threads=%d, max_result_age=%.3fs. Initial/reloaded static-map "
             "ESDF remains synchronous.",
             mp_.parallel_threads_, mp_.async_esdf_max_result_age_);
  }
  else
  {
    ROS_INFO("[GridMap][AsyncESDF] Disabled; using the synchronous baseline path.");
  }

  ROS_INFO("[GridMap] Bounded map parallelism: threads=%d, voxel_threshold=%d.",
           mp_.parallel_threads_, mp_.parallel_voxel_threshold_);
  ROS_INFO("[GridMap][LiDAR] pending/message max age=%.3f/%.3fs, "
           "future_tolerance=%.3fs, health_monitor=%s "
           "isolated_heartbeat=%s timeout/startup_grace=%.3f/%.3fs.",
           mp_.live_lidar_pending_max_age_,
           mp_.live_lidar_message_max_age_,
           mp_.live_lidar_future_tolerance_,
           mp_.live_lidar_health_monitor_enable_ ? "on" : "off",
           (mp_.live_lidar_health_monitor_enable_ &&
            mp_.live_lidar_health_isolated_worker_) ? "on" : "off",
           mp_.live_lidar_timeout_, mp_.live_lidar_startup_grace_);
  ROS_INFO("[GridMap][LiDAR] live obstacle expiry=%s, "
           "max_missed_fusions=%d; static PCD occupancy is never expired.",
           mp_.live_obstacle_expiry_enable_ ? "on" : "off",
           mp_.live_obstacle_max_missed_fusions_);
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  md_.occupancy_bound_min_ = md_.local_bound_min_;
  md_.occupancy_bound_max_ = md_.local_bound_max_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        md_.distance_buffer_[toAddress(x, y, z)] = 10000;
      }
}

void GridMap::noteLidarHealthMonitorStart()
{
  if (!mp_.live_lidar_health_monitor_enable_ ||
      !md_.lidar_health_monitor_start_steady_.isZero())
    return;

  // The grace period begins only once odometry makes live-cloud fusion
  // possible. A latched global PCD arriving during startup must neither arm
  // nor satisfy this independent live-sensor watchdog.
  md_.lidar_health_monitor_start_steady_ = ros::SteadyTime::now();
}

bool GridMap::liveLidarMessageStampFresh(const ros::Time &stamp,
                                         const char *source) const
{
  if (stamp.isZero() || !ros::Time::isValid())
  {
    ROS_WARN_THROTTLE(
        5.0,
        "[GridMap][LiDAR] %s cloud has no usable ROS timestamp; transport-age validation is unavailable (pending-age validation remains active).",
        source);
    return true;
  }

  const double age = (ros::Time::now() - stamp).toSec();
  if (!std::isfinite(age))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][LiDAR] Rejected %s cloud with a non-finite timestamp age.",
        source);
    return false;
  }
  if (age < -mp_.live_lidar_future_tolerance_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][LiDAR] Rejected future-dated %s cloud: stamp is %.3fs "
        "ahead (tolerance %.3fs).",
        source, -age, mp_.live_lidar_future_tolerance_);
    return false;
  }
  if (age <= mp_.live_lidar_message_max_age_)
    return true;

  ROS_WARN_THROTTLE(
      1.0,
      "[GridMap][LiDAR] Rejected stale %s cloud before projection: stamp age %.3fs exceeds %.3fs.",
      source, age, mp_.live_lidar_message_max_age_);
  return false;
}

ros::SteadyTime GridMap::latestLiveLidarHealthTime() const
{
  ros::SteadyTime latest = md_.last_lidar_input_steady_;
  const std::uint64_t ingress_ns =
      lidar_health_steady_ns_.load(std::memory_order_acquire);
  if (ingress_ns == 0)
    return latest;

  ros::SteadyTime ingress;
  ingress.fromNSec(ingress_ns);
  if (latest.isZero() || ingress > latest)
    latest = ingress;
  return latest;
}

void GridMap::liveLidarHealthCallback(
    const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  if (!cloud || cloud->data.empty() || cloud->point_step == 0 ||
      cloud->width == 0 || cloud->height == 0 ||
      !liveLidarMessageStampFresh(cloud->header.stamp,
                                  "isolated-health"))
    return;

  // Do not let a syntactically non-empty packet mask a dead/corrupt sensor.
  // One finite XYZ return is sufficient for transport health; obstacle
  // filtering and map-frame transformation remain owned by the serialized
  // mapping callback.
  bool has_finite_xyz = false;
  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (std::isfinite(*iter_x) && std::isfinite(*iter_y) &&
          std::isfinite(*iter_z))
      {
        has_finite_xyz = true;
        break;
      }
    }
  }
  catch (const std::runtime_error &error)
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GridMap][LiDAR] Isolated health callback rejected malformed XYZ fields: %s",
        error.what());
    return;
  }

  if (!has_finite_xyz)
    return;

  // The serialized mapping path first tries timestamped/latest TF, then uses
  // synchronized odometry as a fallback. Mirror that availability rule here
  // without reading MappingData from this worker thread. This prevents raw
  // LiDAR traffic from masking a broken transform/odometry chain.
  bool transform_available = false;
  if (tf_buffer_ && !cloud->header.frame_id.empty())
  {
    transform_available = tf_buffer_->canTransform(
        "world", cloud->header.frame_id, cloud->header.stamp,
        ros::Duration(0.0));
    if (!transform_available)
      transform_available = tf_buffer_->canTransform(
          "world", cloud->header.frame_id, ros::Time(0),
          ros::Duration(0.0));
  }

  const std::uint64_t odom_ns =
      lidar_health_odom_steady_ns_.load(std::memory_order_acquire);
  const std::uint64_t now_ns = ros::SteadyTime::now().toNSec();
  const double odom_age = odom_ns == 0 || now_ns < odom_ns
      ? std::numeric_limits<double>::infinity()
      : static_cast<double>(now_ns - odom_ns) * 1e-9;
  const bool odom_fallback_available =
      odom_age <= mp_.live_lidar_timeout_;

  if (!transform_available && !odom_fallback_available)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][LiDAR] Isolated health saw fresh finite XYZ, but neither TF nor a fresh valid odometry fallback is available.");
    return;
  }

  lidar_health_steady_ns_.store(now_ns, std::memory_order_release);
}

void GridMap::liveLidarHealthOdomCallback(
    const nav_msgs::OdometryConstPtr &odom)
{
  if (!odom)
    return;

  const auto &position = odom->pose.pose.position;
  const auto &orientation = odom->pose.pose.orientation;
  const bool finite_pose =
      std::isfinite(position.x) && std::isfinite(position.y) &&
      std::isfinite(position.z) && std::isfinite(orientation.x) &&
      std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
      std::isfinite(orientation.w);
  const double quaternion_norm_squared =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
  if (!finite_pose || quaternion_norm_squared < 1e-8)
    return;

  if (!odom->header.stamp.isZero() && ros::Time::isValid())
  {
    const double stamp_age = (ros::Time::now() - odom->header.stamp).toSec();
    if (!std::isfinite(stamp_age) ||
        stamp_age < -mp_.live_lidar_future_tolerance_ ||
        stamp_age > mp_.live_lidar_message_max_age_)
      return;
  }

  lidar_health_odom_steady_ns_.store(
      ros::SteadyTime::now().toNSec(), std::memory_order_release);
}

bool GridMap::hasPendingLidarObservation() const
{
  // Sequence ownership is the primary invariant. Keeping this independent of
  // the origin-valid bit makes result commit fail closed if metadata ever
  // becomes inconsistent; the occupancy callback then rejects and clears it.
  return md_.lidar_pending_seq_ != 0 &&
         md_.lidar_pending_seq_ > md_.lidar_fused_seq_;
}

void GridMap::clearPendingLidarObservation()
{
  md_.lidar_origin_valid_ = false;
  md_.lidar_pending_seq_ = 0;
  md_.pending_lidar_received_steady_ = ros::SteadyTime();
  md_.pending_lidar_stamp_ = ros::Time(0);
  md_.proj_points_cnt = 0;
}

bool GridMap::sensorTimeoutNow() const
{
  const ros::Time now_ros = ros::Time::now();
  const bool legacy_depth_or_odom_timeout =
      !mp_.use_global_map_ && md_.flag_use_depth_fusion &&
      md_.last_occ_update_time_.toSec() >= 1.0 &&
      (now_ros - md_.last_occ_update_time_).toSec() >
          mp_.odom_depth_timeout_;

  bool live_lidar_timeout = false;
  if (mp_.live_lidar_health_monitor_enable_ && md_.has_odom_)
  {
    const ros::SteadyTime latest_live_input =
        latestLiveLidarHealthTime();
    const bool has_healthy_live_input = !latest_live_input.isZero();
    const ros::SteadyTime reference = has_healthy_live_input
        ? latest_live_input
        : md_.lidar_health_monitor_start_steady_;
    const double deadline = has_healthy_live_input
        ? mp_.live_lidar_timeout_
        : mp_.live_lidar_startup_grace_;

    // Once odometry says fusion is possible, a missing watchdog reference is
    // inconsistent state and must fail closed. In normal operation odometry
    // starts this clock in the same serialized callback.
    live_lidar_timeout = reference.isZero() ||
        std::max(0.0,
                 (ros::SteadyTime::now() - reference).toSec()) > deadline;
  }

  return legacy_depth_or_odom_timeout || live_lidar_timeout;
}

void GridMap::updateSensorTimeoutState()
{
  const ros::Time now_ros = ros::Time::now();
  bool legacy_depth_or_odom_timeout = false;
  if (!mp_.use_global_map_ && md_.flag_use_depth_fusion &&
      md_.last_occ_update_time_.toSec() >= 1.0)
  {
    legacy_depth_or_odom_timeout =
        (now_ros - md_.last_occ_update_time_).toSec() >
        mp_.odom_depth_timeout_;
  }

  bool live_lidar_timeout = false;
  double live_lidar_age = 0.0;
  double live_lidar_deadline = mp_.live_lidar_startup_grace_;
  if (mp_.live_lidar_health_monitor_enable_ && md_.has_odom_)
  {
    noteLidarHealthMonitorStart();
    const ros::SteadyTime now_steady = ros::SteadyTime::now();
    const ros::SteadyTime latest_live_input =
        latestLiveLidarHealthTime();
    const bool has_healthy_live_input = !latest_live_input.isZero();
    const ros::SteadyTime reference = has_healthy_live_input
        ? latest_live_input
        : md_.lidar_health_monitor_start_steady_;
    live_lidar_deadline = has_healthy_live_input
        ? mp_.live_lidar_timeout_
        : mp_.live_lidar_startup_grace_;
    if (!reference.isZero())
    {
      live_lidar_age = std::max(0.0, (now_steady - reference).toSec());
      live_lidar_timeout = live_lidar_age > live_lidar_deadline;
    }
    else
    {
      // This should be unreachable because every has_odom_ transition arms
      // the watchdog in the same callback. Keep cached and immediate timeout
      // decisions fail-closed and identical if state is ever inconsistent.
      live_lidar_timeout = true;
    }
  }

  if (live_lidar_timeout && !md_.live_lidar_timeout_active_)
  {
    ROS_ERROR(
        "[GridMap][LiDAR] Live input timed out: age %.3fs exceeds %.3fs "
        "(accepted/fused/pending=%lu/%lu/%lu). Static PCD availability does "
        "not mask this fault.",
        live_lidar_age, live_lidar_deadline,
        static_cast<unsigned long>(md_.lidar_accepted_seq_),
        static_cast<unsigned long>(md_.lidar_fused_seq_),
        static_cast<unsigned long>(md_.lidar_pending_seq_));
  }
  else if (!live_lidar_timeout && md_.live_lidar_timeout_active_)
  {
    ROS_INFO(
        "[GridMap][LiDAR] Live input recovered (accepted/fused=%lu/%lu).",
        static_cast<unsigned long>(md_.lidar_accepted_seq_),
        static_cast<unsigned long>(md_.lidar_fused_seq_));
  }
  md_.live_lidar_timeout_active_ = live_lidar_timeout;

  if (legacy_depth_or_odom_timeout)
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GridMap] odometry/depth occupancy update timed out: age %.3fs exceeds %.3fs.",
        (now_ros - md_.last_occ_update_time_).toSec(),
        mp_.odom_depth_timeout_);
  }
  md_.flag_depth_odom_timeout_ =
      legacy_depth_or_odom_timeout || live_lidar_timeout;
}

void GridMap::updateLocalBoundsAroundRobot(bool force_esdf_update)
{
  if (!md_.has_odom_)
    return;
  if (mp_.global_map_full_esdf_ && md_.global_esdf_pending_)
    return;

  Eigen::Vector3i min_id, max_id;
  if (mp_.global_map_full_esdf_ && fusedGlobalMapOwnsESDFBounds())
  {
    // A full static PCD initializes every ESDF cell.  Keep that compute window
    // spatially stable for later live updates; changing it to a ray-tight box
    // is what made otherwise valid async results historical at commit time.
    min_id = Eigen::Vector3i::Zero();
    max_id = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  }
  else
  {
    const Eigen::Vector3d local_range_min =
        md_.camera_pos_ - mp_.local_update_range_;
    const Eigen::Vector3d local_range_max =
        md_.camera_pos_ + mp_.local_update_range_;
    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);

    int esdf_inf = 0;
    if (mp_.local_bound_inflate_ > 1e-3)
      esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);

    min_id -= esdf_inf * Eigen::Vector3i::Ones();
    max_id += esdf_inf * Eigen::Vector3i::Ones();
    boundIndex(min_id);
    boundIndex(max_id);
  }

  const bool bounds_changed = (min_id != md_.local_bound_min_) || (max_id != md_.local_bound_max_);
  md_.local_bound_min_ = min_id;
  md_.local_bound_max_ = max_id;

  if (force_esdf_update || bounds_changed)
    md_.esdf_need_update_ = true;
}

void GridMap::clearStaticGlobalMap()
{
  for (size_t idx = 0; idx < md_.static_occupancy_.size(); ++idx)
  {
    md_.static_miss_count_[idx] = 0;

    if (!md_.static_occupancy_[idx])
      continue;

    md_.static_occupancy_[idx] = 0;
    md_.occupancy_buffer_inflate_[idx] = 0;
    md_.occupancy_buffer_[idx] = mp_.clamp_min_log_;
    md_.occupancy_buffer_neg_[idx] = 0;
    md_.distance_buffer_[idx] = 10000;
    md_.distance_buffer_neg_[idx] = 10000;
    md_.distance_buffer_all_[idx] = 10000;
  }
}

int GridMap::clearStaticOccupancyAround(const Eigen::Vector3i &center, int clear_step)
{
  int cleared = 0;

  for (int dx = -clear_step; dx <= clear_step; ++dx)
    for (int dy = -clear_step; dy <= clear_step; ++dy)
      for (int dz = -clear_step; dz <= clear_step; ++dz)
      {
        Eigen::Vector3i id(center(0) + dx, center(1) + dy, center(2) + dz);
        if (!isInMap(id))
          continue;

        int idx = toAddress(id);
        if (!md_.static_occupancy_[idx])
          continue;
        if (md_.static_miss_count_[idx] < mp_.global_map_lidar_clear_min_misses_)
          continue;

        md_.static_occupancy_[idx] = 0;
        md_.static_miss_count_[idx] = 0;
        md_.occupancy_buffer_inflate_[idx] = 0;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_;
        md_.occupancy_buffer_neg_[idx] = 1;
        md_.distance_buffer_[idx] = 10000;
        md_.distance_buffer_neg_[idx] = 10000;
        md_.distance_buffer_all_[idx] = 10000;
        ++cleared;
      }

  return cleared;
}

void GridMap::initAstarBuffer(){
  md_.astar_min_idx_.setZero();
  md_.astar_max_idx_(0) = mp_.map_voxel_num_(0) - 1;
  md_.astar_max_idx_(1) = mp_.map_voxel_num_(1) - 1;
}

void GridMap::resetAstarBuffer(){
  boundIndex(md_.astar_min_idx_);
  boundIndex(md_.astar_max_idx_);

  /* reset occ and dist buffer */
  for (int x = md_.astar_min_idx_(0); x <= md_.astar_max_idx_(0); ++x)
    for (int y = md_.astar_min_idx_(1); y <= md_.astar_max_idx_(1); ++y){
      md_.astar_visited_[toAddress2d(x, y)] = false;
      md_.astar_free_length_[toAddress2d(x, y)] = 1e10;
      // md_.astar_next_state_list_[toAddress2d(x, y)] = ;
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;
  if(!isInMap(pos)){
    return INVALID_IDX;
  }

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  if (idx_ctns < 0 || idx_ctns >= (int)md_.occupancy_buffer_.size()) {
    ROS_WARN_THROTTLE(2.0, "[GridMap] setCacheOccupancy idx=%d out of range [0,%zu), pos=(%.2f,%.2f,%.2f) id=(%d,%d,%d)",
                      idx_ctns, md_.occupancy_buffer_.size(),
                      pos(0), pos(1), pos(2), id(0), id(1), id(2));
    return INVALID_IDX;
  }

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess(bool live_lidar_observation)
{
  if (md_.proj_points_cnt == 0)
  {
    // No retained endpoint is not evidence that the whole local volume is
    // free. In particular, an all-NaN/ground/self-filtered scan must not erase
    // an obstacle generation. LiDAR callbacks reject this before enqueueing;
    // keep the raycaster fail-closed as a second line of defense.
    return;
  }

  // Use LiDAR origin for raycasting when available (correct beam path),
  // fall back to camera_pos_ for depth camera data.
  const Eigen::Vector3d& ray_origin = md_.lidar_origin_valid_ ? md_.lidar_origin_ : md_.camera_pos_;

  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;
  bool processed_projected_point = false;
  std::uint32_t current_live_fusion_cycle = 0;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // ====== 过滤世界坐标系中 z=0.05 以下的地面点云 ======
    if (!pt_w.allFinite() || pt_w(2) < mp_.ground_height_ + 0.15)
      continue;

    processed_projected_point = true;

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, ray_origin);

      length = (pt_w - ray_origin).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - ray_origin) / length * mp_.max_ray_length_ + ray_origin;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - ray_origin).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - ray_origin) / length * mp_.max_ray_length_ + ray_origin;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between LiDAR/camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, ray_origin / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - ray_origin).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  if (!processed_projected_point)
  {
    // A future raycast-level filter must retain the same fail-closed rule.
    return;
  }

  if (live_lidar_observation && mp_.live_obstacle_expiry_enable_)
  {
    // Count only scans which survive every raycast-level filter. Rejected
    // scans must not consume a missed-fusion allowance and thereby age a
    // valid obstacle while no replacement observation was fused.
    // Wrap is practically unreachable. Rebase tracked cells so unsigned age
    // arithmetic still fails safe in a very long-lived process.
    if (md_.live_fusion_cycle_ ==
        std::numeric_limits<std::uint32_t>::max())
    {
      for (const int idx : md_.active_live_occupancy_indices_)
      {
        if (idx >= 0 &&
            idx < static_cast<int>(
                      md_.live_occupancy_last_hit_cycle_.size()) &&
            md_.live_occupancy_active_[idx])
          md_.live_occupancy_last_hit_cycle_[idx] = 0;
      }
      md_.live_fusion_cycle_ = 1;
    }
    else
    {
      ++md_.live_fusion_cycle_;
    }
    current_live_fusion_cycle = md_.live_fusion_cycle_;
  }

  min_x = min(min_x, ray_origin(0));
  min_y = min(min_y, ray_origin(1));
  min_z = min(min_z, ray_origin(2));

  max_x = max(max_x, ray_origin(0));
  max_y = max(max_y, ray_origin(1));
  max_z = max(max_z, ray_origin(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z),
             md_.occupancy_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z),
             md_.occupancy_bound_min_);

  int esdf_inf;
  if(mp_.local_bound_inflate_ > 1e-3)
    esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
  else
    esdf_inf = 0;
  // std::cout << "esdf_inf: " << esdf_inf;

  md_.occupancy_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 1);
  md_.occupancy_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 1);
  boundIndex(md_.occupancy_bound_min_);
  boundIndex(md_.occupancy_bound_max_);

  // Without a fused global map, the live sensor remains the sole owner of the
  // ESDF window, matching the native local-map behavior.  With a fused global
  // map, only the static/odometry policy may move that window.
  if (!fusedGlobalMapOwnsESDFBounds())
  {
    md_.local_bound_min_ = md_.occupancy_bound_min_;
    md_.local_bound_max_ = md_.occupancy_bound_max_;
  }

  md_.local_updated_ = true;

  // update occupancy cached in queue
  // Keep all work for this queued observation tied to its captured sensor
  // origin. A newer rejected synchronized frame may already have refreshed
  // camera_pos_, but must not change the pending valid scan's fusion window.
  Eigen::Vector3d local_range_min = ray_origin - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = ray_origin + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  int static_clear_step = 0;
  if (mp_.global_map_lidar_clear_)
  {
    static_clear_step = ceil(max(mp_.global_map_lidar_clear_radius_, 0.0) / mp_.resolution_);
  }
  int cleared_static_voxels = 0;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (md_.static_occupancy_[idx_ctns])
    {
      if (log_odds_update < 0)
      {
        if (!mp_.global_map_lidar_clear_)
          continue;

        if (md_.static_miss_count_[idx_ctns] < 255)
          ++md_.static_miss_count_[idx_ctns];

        if (md_.static_miss_count_[idx_ctns] < mp_.global_map_lidar_clear_min_misses_)
          continue;

        cleared_static_voxels += clearStaticOccupancyAround(idx, static_clear_step);
        continue;
      }

      md_.static_miss_count_[idx_ctns] = 0;
    }

    if (log_odds_update > 0)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_max_log_;
      if (current_live_fusion_cycle > 0 &&
          !md_.static_occupancy_[idx_ctns])
      {
        if (!md_.live_occupancy_active_[idx_ctns])
        {
          md_.live_occupancy_active_[idx_ctns] = 1;
          md_.active_live_occupancy_indices_.push_back(idx_ctns);
        }
        md_.live_occupancy_last_hit_cycle_[idx_ctns] =
            current_live_fusion_cycle;
      }
    }
    else
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }
  }

  if (current_live_fusion_cycle > 0)
  {
    const std::size_t expired =
        expireStaleLiveOccupancy(current_live_fusion_cycle);
    if (expired > 0)
    {
      md_.expired_live_occupancy_count_ += expired;
      ROS_INFO_THROTTLE(
          1.0,
          "[GridMap][LiDAR] Expired %zu stale live obstacle voxels after "
          "%d missed fused scans (total=%lu); static PCD unchanged.",
          expired, mp_.live_obstacle_max_missed_fusions_,
          static_cast<unsigned long>(
              md_.expired_live_occupancy_count_));
    }
  }

  if (cleared_static_voxels > 0)
  {
    ROS_INFO_THROTTLE(2.0, "[GridMap] LiDAR cleared %d local static-map voxels from free rays.",
                      cleared_static_voxels);
  }
}

std::size_t GridMap::expireStaleLiveOccupancy(
    std::uint32_t current_cycle)
{
  if (!mp_.live_obstacle_expiry_enable_ || current_cycle == 0 ||
      md_.live_occupancy_active_.empty())
    return 0;

  std::size_t write_index = 0;
  std::size_t expired_count = 0;
  for (const int idx : md_.active_live_occupancy_indices_)
  {
    if (idx < 0 ||
        idx >= static_cast<int>(md_.occupancy_buffer_.size()) ||
        !md_.live_occupancy_active_[idx])
      continue;

    if (md_.static_occupancy_[idx] ||
        md_.occupancy_buffer_[idx] <= mp_.min_occupancy_log_)
    {
      md_.live_occupancy_active_[idx] = 0;
      md_.live_occupancy_last_hit_cycle_[idx] = 0;
      continue;
    }

    const std::uint32_t last_hit_cycle =
        md_.live_occupancy_last_hit_cycle_[idx];
    const std::uint32_t missed_fusions =
        current_cycle >= last_hit_cycle
            ? current_cycle - last_hit_cycle
            : std::numeric_limits<std::uint32_t>::max();
    if (missed_fusions >= static_cast<std::uint32_t>(
                              mp_.live_obstacle_max_missed_fusions_))
    {
      md_.occupancy_buffer_[idx] = mp_.clamp_min_log_;
      md_.occupancy_buffer_inflate_[idx] = 0;
      md_.live_occupancy_active_[idx] = 0;
      md_.live_occupancy_last_hit_cycle_[idx] = 0;

      // Expired voxels can lie outside the newest rays. Extend the dirty
      // bounds so inflation is rebuilt and the next ESDF generation contains
      // the removal as well as the newest hits.
      const int yz_size =
          mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z();
      const int x = idx / yz_size;
      const int yz = idx % yz_size;
      const int y = yz / mp_.map_voxel_num_.z();
      const int z = yz % mp_.map_voxel_num_.z();
      const Eigen::Vector3i expired_index(x, y, z);
      md_.occupancy_bound_min_ =
          md_.occupancy_bound_min_.cwiseMin(expired_index);
      md_.occupancy_bound_max_ =
          md_.occupancy_bound_max_.cwiseMax(expired_index);
      ++expired_count;
      continue;
    }

    md_.active_live_occupancy_indices_[write_index++] = idx;
  }
  md_.active_live_occupancy_indices_.resize(write_index);
  return expired_count;
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.occupancy_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.occupancy_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {
      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        if (md_.static_occupancy_[idx]) continue;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        if (md_.static_occupancy_[idx]) continue;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        if (md_.static_occupancy_[idx]) continue;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        if (md_.static_occupancy_[idx]) continue;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        if (md_.static_occupancy_[idx]) continue;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        if (md_.static_occupancy_[idx]) continue;
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        md_.distance_buffer_all_[idx] = 10000;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step;
  if(mp_.obstacles_inflation_ < 1e-3){
    inf_step = 0;
  }else{
    inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  }
  
  // int inf_step_z = 1;
  // inf_step = 0;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // std::cout << "inf_step: " << inf_step << "\n";
  // std::cout << "inf_pts size: " << inf_pts.size() << std::endl;
  // inf_pts.resize(4 * inf_step + 3);

  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.occupancy_bound_min_(0); x <= md_.occupancy_bound_max_(0); ++x)
    for (int y = md_.occupancy_bound_min_(1); y <= md_.occupancy_bound_max_(1); ++y)
      for (int z = md_.occupancy_bound_min_(2); z <= md_.occupancy_bound_max_(2); ++z)
      {
        if (md_.static_occupancy_[toAddress(x, y, z)]) continue;
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.occupancy_bound_min_(0); x <= md_.occupancy_bound_max_(0); ++x)
    for (int y = md_.occupancy_bound_min_(1); y <= md_.occupancy_bound_max_(1); ++y)
      for (int z = md_.occupancy_bound_min_(2); z <= md_.occupancy_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
          

          for (unsigned int k = 0; k < inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);

    for (int x = md_.occupancy_bound_min_(0); x <= md_.occupancy_bound_max_(0); ++x)
      for (int y = md_.occupancy_bound_min_(1); y <= md_.occupancy_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  if (!mp_.enable_visualization_)
  {
    // Production launches disable the broad/heavy visualization pass to keep
    // the planner callback queue responsive.  ESDF and inflated-occupancy
    // diagnostics are nevertheless useful during a targeted RViz audit.
    // Both publishers already return immediately without subscribers, so
    // make these two layers demand-driven instead of silently advertising
    // topics which never produce a message.
    publishMapInflate(true);
    publishESDF();
    return;
  }

  publishMapInflate(true);
  publishMap();

  publishESDF();
}

void GridMap::recordESDFTiming(double compute_ms, double commit_ms,
                               double result_age_ms, bool asynchronous)
{
  ++md_.esdf_update_count_;
  md_.esdf_time_ += compute_ms / 1000.0;
  md_.max_esdf_time_ = std::max(md_.max_esdf_time_, compute_ms / 1000.0);
  md_.esdf_wall_sum_ms_ += compute_ms;
  md_.esdf_wall_max_ms_ = std::max(md_.esdf_wall_max_ms_, compute_ms);
  md_.esdf_commit_wall_sum_ms_ += commit_ms;
  md_.esdf_commit_wall_max_ms_ =
      std::max(md_.esdf_commit_wall_max_ms_, commit_ms);
  if (asynchronous)
    md_.async_esdf_last_age_ms_ = result_age_ms;

  if (!mp_.enable_performance_diagnostics_)
    return;

  if (asynchronous && compute_ms > mp_.esdf_warn_ms_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][AsyncESDF][worker] Computation %.2fms exceeds %.2fms; "
        "the ROS callback queue remains available.",
        compute_ms, mp_.esdf_warn_ms_);
  }
  else if (!asynchronous && compute_ms > mp_.esdf_warn_ms_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][deadline] Synchronous ESDF update %.2fms exceeds %.2fms budget.",
        compute_ms, mp_.esdf_warn_ms_);
  }

  if (asynchronous && commit_ms > mp_.esdf_warn_ms_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][deadline] ESDF result commit %.2fms exceeds %.2fms budget.",
        commit_ms, mp_.esdf_warn_ms_);
  }
}

void GridMap::maybeLogPerformanceDiagnostics()
{
  if (!mp_.enable_performance_diagnostics_)
    return;

  const ros::WallTime now = ros::WallTime::now();
  const double window_seconds =
      (now - md_.performance_window_start_).toSec();
  if (window_seconds < mp_.performance_log_interval_)
    return;

  const double occupancy_average = md_.occupancy_update_count_ > 0
      ? md_.occupancy_wall_sum_ms_ /
            static_cast<double>(md_.occupancy_update_count_)
      : 0.0;
  const double esdf_average = md_.esdf_update_count_ > 0
      ? md_.esdf_wall_sum_ms_ / static_cast<double>(md_.esdf_update_count_)
      : 0.0;
  const double commit_average = md_.esdf_update_count_ > 0
      ? md_.esdf_commit_wall_sum_ms_ /
            static_cast<double>(md_.esdf_update_count_)
      : 0.0;
  const double snapshot_average = md_.async_esdf_submitted_count_ > 0
      ? md_.async_esdf_snapshot_wall_sum_ms_ /
            static_cast<double>(md_.async_esdf_submitted_count_)
      : 0.0;
  const std::uint64_t observation_lag =
      md_.dynamic_observation_seq_ > md_.esdf_observation_seq_
          ? md_.dynamic_observation_seq_ - md_.esdf_observation_seq_
          : 0;
  plan_env::ESDFWorkerStats worker_stats;
  if (esdf_worker_)
    worker_stats = esdf_worker_->stats();

  ROS_INFO("[GridMap][pipeline] window=%.2fs mode=%s "
           "occ=%lu avg/max=%.2f/%.2fms esdf=%lu compute_avg/max=%.2f/%.2fms "
           "main_commit_avg/max=%.2f/%.2fms snapshot_avg/max=%.2f/%.2fms "
           "age=%.2fms obs_lag=%lu lidar(accepted/fused/pending/stale_drop)="
           "%lu/%lu/%lu/%lu clouds_coalesced=%lu "
           "async(submit/commit/reject/occ_defer)=%lu/%lu/%lu/%lu "
           "worker(run/pending/coalesced/superseded/faults)="
           "%d/%d/%lu/%lu/%lu threads=%d",
           window_seconds,
           mp_.async_esdf_enable_ && !async_esdf_fallback_to_sync_
               ? "async" : "sync",
           static_cast<unsigned long>(md_.occupancy_update_count_),
           occupancy_average, md_.occupancy_wall_max_ms_,
           static_cast<unsigned long>(md_.esdf_update_count_),
           esdf_average, md_.esdf_wall_max_ms_,
           commit_average, md_.esdf_commit_wall_max_ms_,
           snapshot_average, md_.async_esdf_snapshot_wall_max_ms_,
           md_.async_esdf_last_age_ms_,
           static_cast<unsigned long>(observation_lag),
           static_cast<unsigned long>(md_.lidar_accepted_seq_),
           static_cast<unsigned long>(md_.lidar_fused_seq_),
           static_cast<unsigned long>(md_.lidar_pending_seq_),
           static_cast<unsigned long>(md_.stale_lidar_discard_count_),
           static_cast<unsigned long>(md_.coalesced_cloud_count_),
           static_cast<unsigned long>(md_.async_esdf_submitted_count_),
           static_cast<unsigned long>(md_.async_esdf_committed_count_),
           static_cast<unsigned long>(md_.async_esdf_rejected_count_),
           static_cast<unsigned long>(
               md_.async_esdf_occupancy_deferred_count_),
           worker_stats.running ? 1 : 0,
           worker_stats.pending ? 1 : 0,
           static_cast<unsigned long>(worker_stats.coalesced),
           static_cast<unsigned long>(worker_stats.superseded),
           static_cast<unsigned long>(worker_stats.faults),
           mp_.parallel_threads_);

  md_.occupancy_update_count_ = 0;
  md_.esdf_update_count_ = 0;
  md_.coalesced_cloud_count_ = 0;
  md_.async_esdf_submitted_count_ = 0;
  md_.async_esdf_committed_count_ = 0;
  md_.async_esdf_rejected_count_ = 0;
  md_.async_esdf_occupancy_deferred_count_ = 0;
  md_.occupancy_wall_sum_ms_ = 0.0;
  md_.occupancy_wall_max_ms_ = 0.0;
  md_.esdf_wall_sum_ms_ = 0.0;
  md_.esdf_wall_max_ms_ = 0.0;
  md_.esdf_commit_wall_sum_ms_ = 0.0;
  md_.esdf_commit_wall_max_ms_ = 0.0;
  md_.async_esdf_snapshot_wall_sum_ms_ = 0.0;
  md_.async_esdf_snapshot_wall_max_ms_ = 0.0;
  md_.performance_window_start_ = now;
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (md_.last_occ_update_time_.toSec() < 1.0)
    md_.last_occ_update_time_ = ros::Time::now();

  updateSensorTimeoutState();

  if (!md_.occ_need_update_)
    return;

  // Keep the occupancy revision immutable until the worker's snapshot has
  // reached a commit/reject decision. Incoming LiDAR callbacks continue to
  // coalesce into the one pending slot, so this applies backpressure without
  // blocking perception or processing an obsolete queue of scans.
  if (asyncESDFUpdateOutstanding())
  {
    ++md_.async_esdf_occupancy_deferred_count_;
    maybeLogPerformanceDiagnostics();
    return;
  }

  /* update occupancy */
  ros::Time t1, t2;
  t1 = ros::Time::now();
  const ros::WallTime wall_start = ros::WallTime::now();

  const bool accepted_lidar_observation = hasPendingLidarObservation();
  const std::uint64_t pending_lidar_generation =
      md_.lidar_pending_seq_;

  // A valid callback can still sit behind other work before this timer gets a
  // turn. Never fuse such an old robot-relative snapshot using a now-obsolete
  // planning instant.
  if (accepted_lidar_observation)
  {
    const double pending_age = md_.pending_lidar_received_steady_.isZero()
        ? std::numeric_limits<double>::infinity()
        : std::max(
              0.0,
              (ros::SteadyTime::now() -
               md_.pending_lidar_received_steady_).toSec());
    const bool message_stamp_fresh = liveLidarMessageStampFresh(
        md_.pending_lidar_stamp_, "pending");
    if (!message_stamp_fresh ||
        pending_age > mp_.live_lidar_pending_max_age_)
    {
      ++md_.stale_lidar_discard_count_;
      ROS_WARN_THROTTLE(
          1.0,
          "[GridMap][LiDAR] Discarded pending generation %lu before fusion: "
          "message_stamp=%s, queue age %.3f/%.3fs.",
          static_cast<unsigned long>(pending_lidar_generation),
          message_stamp_fresh ? "fresh" : "stale",
          pending_age,
          mp_.live_lidar_pending_max_age_);
      md_.occ_need_update_ = false;
      md_.local_updated_ = false;
      clearPendingLidarObservation();
      updateSensorTimeoutState();
      return;
    }
  }

  // local_bound_* is converted to unsigned work sizes below.  Reject corrupt
  // state before that conversion and before mutating occupancy; a rejected
  // observation must not become visible through the generation sequence.
  if ((md_.lidar_origin_valid_ && !accepted_lidar_observation) ||
      (accepted_lidar_observation &&
      (md_.proj_points_cnt <= 0 || !md_.lidar_origin_.allFinite() ||
       !isInMap(md_.lidar_origin_) ||
       !validIndexBounds(md_.local_bound_min_, md_.local_bound_max_,
                         mp_.map_voxel_num_))))
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GridMap] Rejected live LiDAR occupancy update: inconsistent generation, no usable returns, invalid origin, or invalid local bounds.");
    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
    clearPendingLidarObservation();
    updateSensorTimeoutState();
    return;
  }

  // Never interpret a sparse scan as evidence that the entire local volume is
  // free. raycastProcess() applies misses only along beams which were actually
  // observed; clearAndInflateLocalMap() then rebuilds inflation from that
  // conservative raw layer. This prevents one partial frame from erasing an
  // occluded dynamic obstacle.

  // projectDepthImage();
  raycastProcess(accepted_lidar_observation);

  if (md_.local_updated_ && !validOccupancyBounds())
  {
    ROS_FATAL_THROTTLE(
        1.0,
        "[GridMap] Invalid occupancy dirty bounds [%d %d %d]-[%d %d %d]; refusing to publish this map generation.",
        md_.occupancy_bound_min_.x(), md_.occupancy_bound_min_.y(),
        md_.occupancy_bound_min_.z(), md_.occupancy_bound_max_.x(),
        md_.occupancy_bound_max_.y(), md_.occupancy_bound_max_.z());
    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
    clearPendingLidarObservation();
    updateSensorTimeoutState();
    return;
  }

  if (md_.local_updated_)
  {
    clearAndInflateLocalMap();
    md_.esdf_need_update_ = true;

    // This is the coherent occupancy revision consumed by ESDF and the FSM.
    // It must advance for every completed mutation, not only LiDAR, otherwise
    // an older async snapshot with the same LiDAR sequence could be committed
    // after depth/static-clear work changed the active occupancy buffers.
    ++md_.dynamic_observation_seq_;

    // Publish the independent LiDAR accepted->fused transition only after both
    // occupancy layers are coherent and their ESDF recomputation is requested.
    if (accepted_lidar_observation)
    {
      md_.lidar_fused_seq_ = pending_lidar_generation;
      md_.last_lidar_fused_steady_ = ros::SteadyTime::now();
    }
    md_.last_occ_update_time_ = ros::Time::now();
  }
  else if (accepted_lidar_observation)
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GridMap][LiDAR] Generation %lu produced no coherent occupancy update; preserving fused generation %lu.",
        static_cast<unsigned long>(pending_lidar_generation),
        static_cast<unsigned long>(md_.lidar_fused_seq_));
  }

  // Reset all pending metadata atomically; no later ESDF callback may confuse
  // this consumed scan with a still-unfused generation.
  if (md_.lidar_origin_valid_ || md_.lidar_pending_seq_ != 0)
    clearPendingLidarObservation();

  t2 = ros::Time::now();
  const double wall_ms =
      (ros::WallTime::now() - wall_start).toSec() * 1000.0;

  md_.fuse_time_ += (t2 - t1).toSec();
  md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());
  ++md_.occupancy_update_count_;
  md_.occupancy_wall_sum_ms_ += wall_ms;
  md_.occupancy_wall_max_ms_ =
      std::max(md_.occupancy_wall_max_ms_, wall_ms);

  if (mp_.enable_performance_diagnostics_ &&
      wall_ms > mp_.occupancy_warn_ms_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][deadline] Occupancy update %.2fms exceeds %.2fms budget.",
        wall_ms, mp_.occupancy_warn_ms_);
  }

  if (mp_.show_occ_time_)
    ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
             md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  updateSensorTimeoutState();
  maybeLogPerformanceDiagnostics();
}

void GridMap::invalidateAsyncESDFJobs()
{
  ++esdf_map_epoch_;
  if (esdf_map_epoch_ == 0)
    esdf_map_epoch_ = 1;
  if (esdf_worker_)
    esdf_worker_->invalidateBeforeEpoch(esdf_map_epoch_);
  // An epoch transition guarantees that older work can no longer publish a
  // result. Resolve its single-flight fence immediately instead of waiting for
  // a result which the worker intentionally discards.
  esdf_last_resolved_request_id_ = esdf_latest_submitted_request_id_;
}

bool GridMap::fusedGlobalMapOwnsESDFBounds() const
{
  return mp_.use_global_map_ && mp_.fuse_global_map_;
}

bool GridMap::validOccupancyBounds() const
{
  return validIndexBounds(md_.occupancy_bound_min_,
                          md_.occupancy_bound_max_,
                          mp_.map_voxel_num_);
}

bool GridMap::validESDFBounds() const
{
  return validIndexBounds(md_.local_bound_min_, md_.local_bound_max_,
                          mp_.map_voxel_num_);
}

bool GridMap::asyncESDFUpdateOutstanding() const
{
  return mp_.async_esdf_enable_ && !async_esdf_fallback_to_sync_ &&
      esdf_latest_submitted_request_id_ > esdf_last_resolved_request_id_;
}

void GridMap::requestSynchronousESDFFallback()
{
  // GridMap and the FSM are serviced by the same default ROS callback queue,
  // so this transition cannot race occupancy mutation or result commit. Keep
  // the worker alive until destruction: its private snapshot is harmless and
  // joining a long-running job here would reintroduce callback blockage.
  if (!async_esdf_fallback_to_sync_)
  {
    async_esdf_fallback_to_sync_ = true;
    invalidateAsyncESDFJobs();
    ROS_ERROR("[GridMap][AsyncESDF] Selected the synchronous fallback for "
              "the remainder of this process.");
  }
  md_.esdf_need_update_ = true;
}

bool GridMap::submitAsyncESDFUpdate()
{
  if (!esdf_worker_ || async_esdf_fallback_to_sync_)
    return false;

  const ros::SteadyTime snapshot_start = ros::SteadyTime::now();
  plan_env::ESDFJob job;
  job.request_id = ++esdf_next_request_id_;
  job.map_epoch = esdf_map_epoch_;
  job.observation_seq = md_.dynamic_observation_seq_;
  job.min_index = md_.local_bound_min_;
  job.max_index = md_.local_bound_max_;
  job.resolution = mp_.resolution_;
  job.captured_steady = snapshot_start;
  const std::uint64_t submitted_request_id = job.request_id;

  if (!validESDFBounds())
  {
    ROS_FATAL("[GridMap][AsyncESDF] Invalid snapshot bounds [%d %d %d]-[%d %d %d]; preserve the last valid ESDF and defer synchronous fallback until bounds recover.",
              job.min_index.x(), job.min_index.y(), job.min_index.z(),
              job.max_index.x(), job.max_index.y(), job.max_index.z());
    requestSynchronousESDFFallback();
    return false;
  }

  const Eigen::Vector3i size =
      job.max_index - job.min_index + Eigen::Vector3i::Ones();
  const std::size_t nz = static_cast<std::size_t>(size.z());
  const std::size_t compact_yz =
      static_cast<std::size_t>(size.y()) * nz;
  const std::size_t voxel_count =
      static_cast<std::size_t>(size.x()) * compact_yz;

  try
  {
    job.occupancy.resize(voxel_count);
    for (int x = job.min_index.x(); x <= job.max_index.x(); ++x)
      for (int y = job.min_index.y(); y <= job.max_index.y(); ++y)
      {
        const std::size_t compact_address =
            static_cast<std::size_t>(x - job.min_index.x()) * compact_yz +
            static_cast<std::size_t>(y - job.min_index.y()) * nz;
        const int map_address =
            x * mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z() +
            y * mp_.map_voxel_num_.z() + job.min_index.z();
        std::copy_n(md_.occupancy_buffer_inflate_.begin() + map_address,
                    nz, job.occupancy.begin() + compact_address);
      }
  }
  catch (const std::exception &error)
  {
    ROS_ERROR("[GridMap][AsyncESDF] Failed to capture occupancy snapshot: %s. Falling back to sync.",
              error.what());
    requestSynchronousESDFFallback();
    return false;
  }

  const double snapshot_ms =
      (ros::SteadyTime::now() - snapshot_start).toSec() * 1000.0;
  if (!esdf_worker_->submit(std::move(job)))
  {
    const plan_env::ESDFWorkerStats worker_stats = esdf_worker_->stats();
    ROS_ERROR("[GridMap][AsyncESDF] Worker rejected request: %s. Falling back to synchronous ESDF.",
              worker_stats.last_fault.empty() ? "worker stopped or unavailable" :
                                                worker_stats.last_fault.c_str());
    requestSynchronousESDFFallback();
    return false;
  }

  // This watermark is written only after the immutable snapshot has been
  // accepted by the worker.  commitAsyncESDFResult() therefore cannot mistake
  // an older, same-generation/same-bounds result for the request that is
  // currently expected by the map.
  esdf_latest_submitted_request_id_ = submitted_request_id;
  ++md_.async_esdf_submitted_count_;
  md_.async_esdf_snapshot_wall_sum_ms_ += snapshot_ms;
  md_.async_esdf_snapshot_wall_max_ms_ =
      std::max(md_.async_esdf_snapshot_wall_max_ms_, snapshot_ms);
  md_.esdf_need_update_ = false;

  if (mp_.enable_performance_diagnostics_ && snapshot_ms > mp_.esdf_warn_ms_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][deadline] ESDF occupancy snapshot %.2fms exceeds %.2fms budget.",
        snapshot_ms, mp_.esdf_warn_ms_);
  }
  return true;
}

void GridMap::commitAsyncESDFResult()
{
  if (!esdf_worker_)
    return;

  plan_env::ESDFResult result;
  if (!esdf_worker_->tryTakeResult(result))
    return;

  // From this point the request has a definitive main-queue decision. Advance
  // the fence before any rejection path so occupancy can recover and produce
  // a fresh generation instead of remaining permanently deferred.
  esdf_last_resolved_request_id_ =
      std::max(esdf_last_resolved_request_id_, result.request_id);

  const bool matches_current_fused_generation =
      result.observation_seq == md_.dynamic_observation_seq_;
  const bool matches_latest_request =
      result.request_id == esdf_latest_submitted_request_id_;
  if (!matches_latest_request)
  {
    ++md_.async_esdf_rejected_count_;
    // If this is an older result, a newer submitted request already owns its
    // immutable snapshot. Preserve esdf_need_update_ exactly: a mutation after
    // that submission has already set it, while setting it here would create
    // an unnecessary third request and cancel the correct second one.
    if (result.request_id > esdf_latest_submitted_request_id_)
      md_.esdf_need_update_ = true;
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][AsyncESDF] Rejected non-current result id=%lu; latest "
        "submitted request=%lu. Active ESDF was not modified.",
        static_cast<unsigned long>(result.request_id),
        static_cast<unsigned long>(esdf_latest_submitted_request_id_));
    return;
  }

  if (!matches_current_fused_generation)
  {
    // The active distance field is a single coherent generation. Even the
    // latest numerically valid patch must not overwrite it after occupancy has
    // advanced past its snapshot. A pending LiDAR frame is intentionally not
    // part of that revision yet: single-flight backpressure keeps it queued
    // until this result is resolved.
    ++md_.async_esdf_rejected_count_;
    md_.esdf_need_update_ = true;
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][AsyncESDF] Rejected latest result id=%lu generation=%lu; "
        "current occupancy=%lu, pending LiDAR=%lu. Active ESDF was not modified.",
        static_cast<unsigned long>(result.request_id),
        static_cast<unsigned long>(result.observation_seq),
        static_cast<unsigned long>(md_.dynamic_observation_seq_),
        static_cast<unsigned long>(md_.lidar_pending_seq_));
    return;
  }

  const bool obsolete = result.map_epoch != esdf_map_epoch_ ||
      result.request_id <= esdf_committed_request_id_ ||
      result.observation_seq < md_.esdf_observation_seq_;
  if (obsolete)
  {
    ++md_.async_esdf_rejected_count_;
    return;
  }

  bool valid_bounds = true;
  for (int axis = 0; axis < 3; ++axis)
  {
    valid_bounds = valid_bounds && result.min_index(axis) >= 0 &&
        result.max_index(axis) >= result.min_index(axis) &&
        result.max_index(axis) < mp_.map_voxel_num_(axis);
  }
  const Eigen::Vector3i size =
      result.max_index - result.min_index + Eigen::Vector3i::Ones();
  const std::size_t expected_size = valid_bounds
      ? static_cast<std::size_t>(size.x()) *
            static_cast<std::size_t>(size.y()) *
            static_cast<std::size_t>(size.z())
      : 0;
  const double result_age_ms = std::max(
      0.0, (ros::SteadyTime::now() - result.captured_steady).toSec() * 1000.0);
  const ros::SteadyTime commit_start = ros::SteadyTime::now();
  const bool finite_result = expected_size == result.distance_all.size() &&
      std::all_of(result.distance_all.begin(), result.distance_all.end(),
                  [](double distance) { return std::isfinite(distance); });

  if (!valid_bounds || !finite_result ||
      result_age_ms > mp_.async_esdf_max_result_age_ * 1000.0)
  {
    ++md_.async_esdf_rejected_count_;
    md_.esdf_need_update_ = true;
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][AsyncESDF] Rejected result id=%lu: bounds=%s size=%zu/%zu age=%.2f/%.2fms; requesting a fresh snapshot.",
        static_cast<unsigned long>(result.request_id),
        valid_bounds ? "ok" : "invalid", result.distance_all.size(),
        expected_size, result_age_ms,
        mp_.async_esdf_max_result_age_ * 1000.0);
    return;
  }

  const bool matches_current_bounds =
      result.min_index == md_.local_bound_min_ &&
      result.max_index == md_.local_bound_max_;
  if (!matches_current_bounds)
  {
    // A complete historical patch is still the wrong spatial generation for
    // the current planning window. Reject it before touching the active
    // distance buffer; otherwise its overlap could transiently overwrite a
    // newer current-window field even though freshness is not advanced.
    ++md_.async_esdf_rejected_count_;
    md_.esdf_need_update_ = true;
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap][AsyncESDF] Rejected historical result id=%lu bounds=[%d %d %d]-[%d %d %d]; current bounds=[%d %d %d]-[%d %d %d].",
        static_cast<unsigned long>(result.request_id),
        result.min_index.x(), result.min_index.y(), result.min_index.z(),
        result.max_index.x(), result.max_index.y(), result.max_index.z(),
        md_.local_bound_min_.x(), md_.local_bound_min_.y(),
        md_.local_bound_min_.z(), md_.local_bound_max_.x(),
        md_.local_bound_max_.y(), md_.local_bound_max_.z());
    return;
  }

  const std::size_t nz = static_cast<std::size_t>(size.z());
  const std::size_t compact_yz =
      static_cast<std::size_t>(size.y()) * nz;
  for (int x = result.min_index.x(); x <= result.max_index.x(); ++x)
    for (int y = result.min_index.y(); y <= result.max_index.y(); ++y)
    {
      const std::size_t compact_address =
          static_cast<std::size_t>(x - result.min_index.x()) * compact_yz +
          static_cast<std::size_t>(y - result.min_index.y()) * nz;
      const int map_address =
          x * mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z() +
          y * mp_.map_voxel_num_.z() + result.min_index.z();
      std::copy_n(result.distance_all.begin() + compact_address, nz,
                  md_.distance_buffer_all_.begin() + map_address);
    }

  const double commit_ms =
      (ros::SteadyTime::now() - commit_start).toSec() * 1000.0;
  esdf_committed_request_id_ = result.request_id;
  md_.esdf_observation_seq_ =
      std::max(md_.esdf_observation_seq_, result.observation_seq);
  ++md_.async_esdf_committed_count_;
  recordESDFTiming(result.compute_ms, commit_ms, result_age_ms, true);

  if (mp_.show_esdf_time_)
  {
    ROS_WARN("Async ESDF: compute=%.3fms queue=%.3fms commit=%.3fms age=%.3fms seq=%lu",
             result.compute_ms, result.queue_wait_ms, commit_ms, result_age_ms,
             static_cast<unsigned long>(result.observation_seq));
  }
}

void GridMap::updateESDFCallback(const ros::TimerEvent& /*event*/)
{
  // Never consume a ready patch or enter the synchronous EDT with corrupt
  // bounds. Preserve the last valid field, cancel the private generation and
  // retry only after a later odometry/map callback repairs the window.
  if (!validESDFBounds())
  {
    if (mp_.async_esdf_enable_ && !async_esdf_fallback_to_sync_)
      requestSynchronousESDFFallback();
    else
      md_.esdf_need_update_ = true;
    ROS_FATAL_THROTTLE(
        1.0,
        "[GridMap][ESDF] Invalid local bounds [%d %d %d]-[%d %d %d] for map [%d %d %d]; preserving the last valid ESDF and refusing computation.",
        md_.local_bound_min_.x(), md_.local_bound_min_.y(),
        md_.local_bound_min_.z(), md_.local_bound_max_.x(),
        md_.local_bound_max_.y(), md_.local_bound_max_.z(),
        mp_.map_voxel_num_.x(), mp_.map_voxel_num_.y(),
        mp_.map_voxel_num_.z());
    maybeLogPerformanceDiagnostics();
    return;
  }

  // Results must be drained even when no newer occupancy update is waiting.
  // This callback runs on the same default queue as all planning queries, so
  // the patch copy below is an atomic generation boundary to those readers.
  commitAsyncESDFResult();

  if (esdf_worker_ && !async_esdf_fallback_to_sync_)
  {
    const plan_env::ESDFWorkerStats worker_stats = esdf_worker_->stats();
    if (worker_stats.faults > 0)
    {
      ROS_ERROR("[GridMap][AsyncESDF] Worker fault: %s. Cancel its private generation and request the synchronous safety path.",
                worker_stats.last_fault.empty() ? "unknown error" :
                                                  worker_stats.last_fault.c_str());
      requestSynchronousESDFFallback();
      maybeLogPerformanceDiagnostics();
      return;
    }
  }

  // Do not submit a replacement while one immutable snapshot is running or
  // waiting for this main queue to consume it. Otherwise a sensor rate above
  // the full-map EDT rate can cancel every job and starve ESDF forever.
  if (asyncESDFUpdateOutstanding())
  {
    maybeLogPerformanceDiagnostics();
    return;
  }

  // Occupancy fusion owns the accepted->fused generation transition. Waiting
  // one timer cycle is cheap and prevents either the async snapshotter or the
  // synchronous fallback from computing over the previous occupancy while a
  // newer live scan is already queued.
  if (hasPendingLidarObservation())
  {
    maybeLogPerformanceDiagnostics();
    return;
  }

  if (!md_.esdf_need_update_)
  {
    maybeLogPerformanceDiagnostics();
    return;
  }

  const bool static_or_initial_update =
      md_.static_map_esdf_pending_ || md_.global_esdf_pending_;
  if (mp_.async_esdf_enable_ && !async_esdf_fallback_to_sync_ &&
      !static_or_initial_update)
  {
    if (submitAsyncESDFUpdate())
    {
      maybeLogPerformanceDiagnostics();
      return;
    }
    // A valid async request which failed switches mode non-blockingly. Give
    // other safety/perception callbacks a chance to run before the next timer
    // performs the necessarily serialized fallback EDT.
    if (async_esdf_fallback_to_sync_)
    {
      maybeLogPerformanceDiagnostics();
      return;
    }
  }

  // Defensive duplicate of the entry guard: submitAsyncESDFUpdate() also
  // validates its captured bounds, and no future change may turn that failure
  // into an unsafe synchronous call in this callback.
  if (!validESDFBounds())
  {
    ROS_FATAL_THROTTLE(
        1.0,
        "[GridMap][ESDF] Refusing synchronous EDT because local bounds became invalid.");
    md_.esdf_need_update_ = true;
    return;
  }

  const bool completes_static_map_esdf = md_.static_map_esdf_pending_;
  const std::uint64_t computed_observation_seq =
      md_.dynamic_observation_seq_;
  const ros::SteadyTime wall_start = ros::SteadyTime::now();
  updateESDF3d();
  const double wall_ms =
      (ros::SteadyTime::now() - wall_start).toSec() * 1000.0;

  md_.esdf_observation_seq_ =
      std::max(md_.esdf_observation_seq_, computed_observation_seq);
  recordESDFTiming(wall_ms, wall_ms, 0.0, false);

  if (mp_.show_esdf_time_)
  {
    const double update_count =
        std::max<std::uint64_t>(1, md_.esdf_update_count_);
    ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf",
             wall_ms / 1000.0, md_.esdf_time_ / update_count,
             md_.max_esdf_time_);
  }

  // Preserve this ordering: updateLocalBoundsAroundRobot() below may request
  // the first local update after a one-shot full static-map ESDF.
  md_.esdf_need_update_ = false;
  if (completes_static_map_esdf)
  {
    md_.static_map_esdf_pending_ = false;
    md_.static_map_esdf_ready_ = true;
    ROS_INFO("[GridMap] Static global map ESDF is ready for planning.");
  }
  if (md_.global_esdf_pending_)
  {
    md_.global_esdf_pending_ = false;
    ROS_INFO("[GridMap] Static global map full ESDF is ready.");
    if (mp_.local_esdf_follow_odom_)
      updateLocalBoundsAroundRobot(false);
  }
  maybeLogPerformanceDiagnostics();
}

void GridMap::updateDynamicPointOverlayCallback(
    const ros::TimerEvent& /*event*/)
{
  if (!mp_.dynamic_point_overlay_enable_)
    return;

  const ros::Time now = ros::Time::now();
  const plan_env::DynamicPointSnapshot snapshot =
      dynamic_point_buffer_.getSnapshot(now);
  refreshDynamicPointPlanningOverlay(snapshot, now);
}

bool GridMap::refreshDynamicPointPlanningOverlay(
    const plan_env::DynamicPointSnapshot &snapshot,
    const ros::Time &now)
{
  if (!mp_.dynamic_point_overlay_enable_)
    return false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr swept_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<float> swept_radii;
  pcl::PointCloud<pcl::PointXYZI>::Ptr nominal_prediction_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  struct PredictionSlice
  {
    double wall_time_ahead = 0.0;
    double prediction_dt = 0.0;
    double model_inflation = 0.0;
    double guide_inflation = 0.0;
    std::vector<geometry_msgs::Point> points;
  };
  std::vector<PredictionSlice> prediction_slices;
  const bool prediction_visualization_requested =
      dynamic_prediction_nominal_pub_.getNumSubscribers() > 0 ||
      dynamic_prediction_markers_pub_.getNumSubscribers() > 0;
  double conservative_inflation = 0.0;
  double max_nominal_speed = 0.0;
  double max_sample_interval = 0.0;
  double observation_age = 0.0;
  double prediction_horizon = 0.0;
  double final_prediction_dt = 0.0;
  Eigen::Vector3d overlay_min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d overlay_max = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());

  if (snapshot.valid && !snapshot.points.empty())
  {
    observation_age =
        std::max(0.0, (now - snapshot.stamp).toSec());
    const double available_horizon = std::max(
        0.0, dynamic_point_buffer_.maxPredictionTime() - observation_age);
    prediction_horizon = std::min(
        mp_.dynamic_point_overlay_horizon_, available_horizon);
    max_sample_interval = std::min(
        prediction_horizon, mp_.dynamic_point_overlay_time_step_);
    for (const auto &point : snapshot.points)
      max_nominal_speed = std::max(
          max_nominal_speed, point.velocity.norm());
    const double edge_allowance =
        dynamic_point_buffer_.configuredPredictionUncertainty(
            observation_age + prediction_horizon) +
        mp_.dynamic_point_overlay_extra_inflation_ +
        0.5 * max_nominal_speed * max_sample_interval;

    double ahead = 0.0;
    while (ahead <= prediction_horizon + 1e-9)
    {
      const ros::Time query_time = now + ros::Duration(ahead);
      double prediction_dt_for_slice = 0.0;
      bool slice_has_point = false;
      PredictionSlice visualization_slice;
      visualization_slice.wall_time_ahead = ahead;
      const double nominal_prediction_dt = std::min(
          dynamic_point_buffer_.maxPredictionTime(),
          std::max(0.0, observation_age + ahead));
      const double full_slice_inflation =
          dynamic_point_buffer_.configuredPredictionUncertainty(
              nominal_prediction_dt) +
          mp_.dynamic_point_overlay_extra_inflation_ +
          0.5 * max_nominal_speed * max_sample_interval;
      // The swept KD-tree is a fast search guide, not the final safety proof.
      // Bound its radius so a distant uncertain prediction cannot erase the
      // entire local search space. Candidate publication below is still
      // checked against the uncapped, time-aligned prediction model.
      const double guide_slice_inflation = std::min(
          full_slice_inflation,
          mp_.dynamic_point_overlay_max_inflation_);

      for (const auto &point : snapshot.points)
      {
        Eigen::Vector3d predicted_position;
        double prediction_dt = 0.0;
        if (!plan_env::DynamicPointBuffer::predictPosition(
                point, snapshot.stamp, query_time,
                dynamic_point_buffer_.maxPredictionTime(),
                predicted_position, prediction_dt))
          continue;
        // Keep a point just outside the map when its conservative envelope can
        // still intrude into the planning volume.
        if ((predicted_position.array() <
             (mp_.map_min_boundary_.array() - edge_allowance)).any() ||
            (predicted_position.array() >
             (mp_.map_max_boundary_.array() + edge_allowance)).any())
          continue;

        pcl::PointXYZ pcl_point;
        pcl_point.x = static_cast<float>(predicted_position.x());
        pcl_point.y = static_cast<float>(predicted_position.y());
        pcl_point.z = static_cast<float>(predicted_position.z());
        swept_cloud->push_back(pcl_point);
        swept_radii.push_back(
            static_cast<float>(guide_slice_inflation));
        if (prediction_visualization_requested)
        {
          pcl::PointXYZI time_point;
          time_point.x = pcl_point.x;
          time_point.y = pcl_point.y;
          time_point.z = pcl_point.z;
          // RViz's Intensity transformer now directly represents seconds
          // into the prediction horizon, rather than an arbitrary colour.
          time_point.intensity = static_cast<float>(ahead);
          nominal_prediction_cloud->push_back(time_point);

          geometry_msgs::Point marker_point;
          marker_point.x = predicted_position.x();
          marker_point.y = predicted_position.y();
          marker_point.z = predicted_position.z();
          visualization_slice.points.push_back(marker_point);
        }
        overlay_min = overlay_min.cwiseMin(predicted_position);
        overlay_max = overlay_max.cwiseMax(predicted_position);
        prediction_dt_for_slice = std::max(
            prediction_dt_for_slice, prediction_dt);
        slice_has_point = true;
      }

      if (slice_has_point)
      {
        final_prediction_dt = std::max(
            final_prediction_dt, prediction_dt_for_slice);
        conservative_inflation = std::max(
            conservative_inflation, guide_slice_inflation);
        if (prediction_visualization_requested)
        {
          visualization_slice.prediction_dt = prediction_dt_for_slice;
          visualization_slice.model_inflation = full_slice_inflation;
          visualization_slice.guide_inflation = guide_slice_inflation;
          prediction_slices.push_back(std::move(visualization_slice));
        }
      }

      if (ahead >= prediction_horizon - 1e-9)
        break;
      ahead = std::min(
          prediction_horizon,
          ahead + mp_.dynamic_point_overlay_time_step_);
    }
  }

  const double full_horizon_inflation =
      dynamic_point_buffer_.configuredPredictionUncertainty(
          final_prediction_dt) +
      mp_.dynamic_point_overlay_extra_inflation_ +
      0.5 * max_nominal_speed * max_sample_interval;
  if (full_horizon_inflation >
      mp_.dynamic_point_overlay_max_inflation_ + 1e-9)
  {
    ROS_INFO_THROTTLE(
        5.0,
        "[DynamicPointFlow][overlay] Full time-aware uncertainty reaches %.3f m; bound only the search guide at %.3f m and retain the full envelope for final validation.",
        full_horizon_inflation,
        mp_.dynamic_point_overlay_max_inflation_);
  }
  swept_cloud->width = swept_cloud->points.size();
  swept_cloud->height = 1;
  swept_cloud->is_dense = true;

  bool accepted_refresh = true;
  {
    std::lock_guard<std::mutex> lock(dynamic_point_overlay_mutex_);
    // A synchronous replan may request a refresh while the dedicated worker is
    // also building one. Never allow an older build to overwrite a newer
    // snapshot merely because it finished later.
    if (dynamic_point_overlay_frozen_ ||
        (!dynamic_point_overlay_refresh_time_.isZero() &&
         now < dynamic_point_overlay_refresh_time_) ||
        snapshot.sequence < dynamic_point_overlay_sequence_)
    {
      accepted_refresh = false;
    }
    else
    {
      dynamic_point_overlay_cloud_ = swept_cloud;
      dynamic_point_overlay_radii_ = swept_radii;
      dynamic_point_overlay_inflation_ = conservative_inflation;
      if (!swept_cloud->empty())
      {
        dynamic_point_overlay_min_ = overlay_min;
        dynamic_point_overlay_max_ = overlay_max;
      }
      else
      {
        dynamic_point_overlay_min_.setZero();
        dynamic_point_overlay_max_.setZero();
      }
      dynamic_point_overlay_sequence_ = snapshot.sequence;
      dynamic_point_overlay_refresh_time_ = now;
      dynamic_point_overlay_ready_ = snapshot.valid && !swept_cloud->empty();
      if (dynamic_point_overlay_ready_)
        dynamic_point_overlay_kdtree_.setInputCloud(
            dynamic_point_overlay_cloud_);
    }
  }

  if (!accepted_refresh)
  {
    ROS_DEBUG_THROTTLE(
        1.0,
        "[DynamicPointFlow][overlay] Discarded stale concurrent refresh seq=%lu.",
        static_cast<unsigned long>(snapshot.sequence));
    return dynamicPointPlanningOverlayReadyFor(snapshot.sequence);
  }

  sensor_msgs::PointCloud2 overlay_msg;
  pcl::toROSMsg(*swept_cloud, overlay_msg);
  overlay_msg.header.stamp = now;
  overlay_msg.header.frame_id = mp_.frame_id_;
  dynamic_point_overlay_pub_.publish(overlay_msg);

  if (prediction_visualization_requested)
  {
    nominal_prediction_cloud->width = nominal_prediction_cloud->points.size();
    nominal_prediction_cloud->height = 1;
    nominal_prediction_cloud->is_dense = true;
    sensor_msgs::PointCloud2 nominal_msg;
    pcl::toROSMsg(*nominal_prediction_cloud, nominal_msg);
    nominal_msg.header.stamp = now;
    nominal_msg.header.frame_id = mp_.frame_id_;
    dynamic_prediction_nominal_pub_.publish(nominal_msg);

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear_marker;
    clear_marker.header = nominal_msg.header;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    int marker_id = 0;
    const ros::Duration marker_lifetime(std::max(
        0.25, 3.0 * mp_.dynamic_point_overlay_update_interval_));
    // Visualize the exact time-varying radii used by the planning KD-tree.
    // A separate namespace below shows the uncapped final-validation model.
    for (const auto &slice : prediction_slices)
    {
      if (slice.points.empty() || slice.guide_inflation <= 1e-6)
        continue;
      const double ratio = prediction_horizon > 1e-6 ?
          std::max(0.0, std::min(
              1.0, slice.wall_time_ahead / prediction_horizon)) : 0.0;
      visualization_msgs::Marker planning_envelope;
      planning_envelope.header = nominal_msg.header;
      planning_envelope.ns = "time_varying_planning_guide";
      planning_envelope.id = marker_id++;
      planning_envelope.type = visualization_msgs::Marker::SPHERE_LIST;
      planning_envelope.action = visualization_msgs::Marker::ADD;
      planning_envelope.pose.orientation.w = 1.0;
      planning_envelope.scale.x = 2.0 * slice.guide_inflation;
      planning_envelope.scale.y = 2.0 * slice.guide_inflation;
      planning_envelope.scale.z = 2.0 * slice.guide_inflation;
      planning_envelope.color.r = 1.0f;
      planning_envelope.color.g = static_cast<float>(0.8 * (1.0 - ratio));
      planning_envelope.color.b = 0.02f;
      planning_envelope.color.a = 0.015f;
      planning_envelope.lifetime = marker_lifetime;
      planning_envelope.points = slice.points;
      marker_array.markers.push_back(std::move(planning_envelope));
    }

    // The model envelope is the uncapped safety model used by final
    // time-aligned candidate validation.
    for (const auto &slice : prediction_slices)
    {
      if (slice.points.empty() || slice.model_inflation <= 1e-6)
        continue;
      const double ratio = prediction_horizon > 1e-6 ?
          std::max(0.0, std::min(
              1.0, slice.wall_time_ahead / prediction_horizon)) : 0.0;
      visualization_msgs::Marker model_envelope;
      model_envelope.header = nominal_msg.header;
      model_envelope.ns = "time_varying_model_envelope";
      model_envelope.id = marker_id++;
      model_envelope.type = visualization_msgs::Marker::SPHERE_LIST;
      model_envelope.action = visualization_msgs::Marker::ADD;
      model_envelope.pose.orientation.w = 1.0;
      model_envelope.scale.x = 2.0 * slice.model_inflation;
      model_envelope.scale.y = 2.0 * slice.model_inflation;
      model_envelope.scale.z = 2.0 * slice.model_inflation;
      model_envelope.color.r = static_cast<float>(ratio);
      model_envelope.color.g = static_cast<float>(1.0 - ratio);
      model_envelope.color.b = 1.0f;
      model_envelope.color.a = 0.004f;
      model_envelope.lifetime = marker_lifetime;
      model_envelope.points = slice.points;
      marker_array.markers.push_back(std::move(model_envelope));
    }

    // One velocity arrow and label per tracker ID keeps the view readable
    // even though every obstacle surface point carries the same track motion.
    std::map<std::uint32_t,
             std::vector<const plan_env::DynamicPoint *> > track_points;
    if (snapshot.valid)
    {
      for (const auto &point : snapshot.points)
        track_points[point.track_id].push_back(&point);
    }
    for (const auto &track : track_points)
    {
      Eigen::Vector3d current_center = Eigen::Vector3d::Zero();
      Eigen::Vector3d mean_velocity = Eigen::Vector3d::Zero();
      std::size_t accepted_points = 0;
      for (const auto *point : track.second)
      {
        Eigen::Vector3d current_position;
        double prediction_dt = 0.0;
        if (!plan_env::DynamicPointBuffer::predictPosition(
                *point, snapshot.stamp, now,
                dynamic_point_buffer_.maxPredictionTime(),
                current_position, prediction_dt))
          continue;
        current_center += current_position;
        mean_velocity += point->velocity;
        ++accepted_points;
      }
      if (accepted_points == 0)
        continue;
      current_center /= static_cast<double>(accepted_points);
      mean_velocity /= static_cast<double>(accepted_points);

      const double arrow_seconds = std::min(1.0, prediction_horizon);
      visualization_msgs::Marker velocity_arrow;
      velocity_arrow.header = nominal_msg.header;
      velocity_arrow.ns = "track_velocity";
      velocity_arrow.id = marker_id++;
      velocity_arrow.type = visualization_msgs::Marker::ARROW;
      velocity_arrow.action = visualization_msgs::Marker::ADD;
      velocity_arrow.pose.orientation.w = 1.0;
      velocity_arrow.scale.x = 0.035;
      velocity_arrow.scale.y = 0.075;
      velocity_arrow.scale.z = 0.10;
      velocity_arrow.color.r = 0.05f;
      velocity_arrow.color.g = 0.95f;
      velocity_arrow.color.b = 0.95f;
      velocity_arrow.color.a = 1.0f;
      velocity_arrow.lifetime = marker_lifetime;
      geometry_msgs::Point arrow_start;
      arrow_start.x = current_center.x();
      arrow_start.y = current_center.y();
      arrow_start.z = current_center.z();
      geometry_msgs::Point arrow_end = arrow_start;
      arrow_end.x += mean_velocity.x() * arrow_seconds;
      arrow_end.y += mean_velocity.y() * arrow_seconds;
      arrow_end.z += mean_velocity.z() * arrow_seconds;
      velocity_arrow.points.push_back(arrow_start);
      velocity_arrow.points.push_back(arrow_end);
      marker_array.markers.push_back(std::move(velocity_arrow));

      visualization_msgs::Marker track_label;
      track_label.header = nominal_msg.header;
      track_label.ns = "track_label";
      track_label.id = marker_id++;
      track_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      track_label.action = visualization_msgs::Marker::ADD;
      track_label.pose.position = arrow_start;
      track_label.pose.position.z += 0.25;
      track_label.pose.orientation.w = 1.0;
      track_label.scale.z = 0.16;
      track_label.color.r = 0.0f;
      track_label.color.g = 0.1f;
      track_label.color.b = 0.1f;
      track_label.color.a = 1.0f;
      track_label.lifetime = marker_lifetime;
      std::ostringstream label;
      label.setf(std::ios::fixed);
      label.precision(2);
      label << "track " << track.first << "  v="
            << mean_velocity.norm() << " m/s";
      track_label.text = label.str();
      marker_array.markers.push_back(std::move(track_label));
    }

    if (!swept_cloud->empty())
    {
      const double uncertainty =
          dynamic_point_buffer_.configuredPredictionUncertainty(
              final_prediction_dt);
      const double sampling_padding =
          0.5 * max_nominal_speed * max_sample_interval;
      visualization_msgs::Marker audit_text;
      audit_text.header = nominal_msg.header;
      audit_text.ns = "prediction_audit";
      audit_text.id = marker_id++;
      audit_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      audit_text.action = visualization_msgs::Marker::ADD;
      audit_text.pose.position.x = overlay_min.x();
      audit_text.pose.position.y = overlay_min.y();
      audit_text.pose.position.z = overlay_max.z() + 0.55;
      audit_text.pose.orientation.w = 1.0;
      audit_text.scale.z = 0.14;
      audit_text.color.r = 0.05f;
      audit_text.color.g = 0.05f;
      audit_text.color.b = 0.05f;
      audit_text.color.a = 1.0f;
      audit_text.lifetime = marker_lifetime;
      std::ostringstream audit;
      audit.setf(std::ios::fixed);
      audit.precision(2);
      audit << "prediction age=" << observation_age
            << "s  horizon=" << prediction_horizon << "s\n"
            << "uncertainty=" << uncertainty
            << " + extra=" << mp_.dynamic_point_overlay_extra_inflation_
            << " + sampling=" << sampling_padding << " m\n"
            << "SEARCH GUIDE max=" << conservative_inflation
            << " m  cap=" << mp_.dynamic_point_overlay_max_inflation_
            << " m\nFINAL validation keeps uncapped r(t)";
      audit_text.text = audit.str();
      marker_array.markers.push_back(std::move(audit_text));
    }

    dynamic_prediction_markers_pub_.publish(marker_array);
  }

  if (!snapshot.valid)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[DynamicPointFlow][overlay] Planning overlay unavailable (%s); static/live-map planning remains active.",
        snapshot.invalid_reason.c_str());
    return false;
  }

  ROS_INFO_THROTTLE(
      1.0,
      "[DynamicPointFlow][overlay] Swept planning cloud: %zu points, seq=%lu, max time-varying guide radius=%.3f m.",
      swept_cloud->size(),
      static_cast<unsigned long>(snapshot.sequence),
      conservative_inflation);
  return dynamicPointPlanningOverlayReadyFor(snapshot.sequence);
}

bool GridMap::beginDynamicPointPlanningSnapshot(
    const plan_env::DynamicPointSnapshot &snapshot,
    const ros::Time &now)
{
  if (!mp_.dynamic_point_overlay_enable_ || !snapshot.valid ||
      snapshot.points.empty())
    return false;

  if (!dynamicPointPlanningOverlayReadyFor(snapshot.sequence) &&
      !refreshDynamicPointPlanningOverlay(snapshot, now))
    return false;

  std::lock_guard<std::mutex> lock(dynamic_point_overlay_mutex_);
  if (!dynamic_point_overlay_ready_ ||
      dynamic_point_overlay_sequence_ < snapshot.sequence)
    return false;
  dynamic_point_overlay_frozen_ = true;
  dynamic_point_overlay_frozen_sequence_ =
      dynamic_point_overlay_sequence_;
  ROS_DEBUG("[DynamicPointFlow][overlay] Freeze planning snapshot seq=%lu.",
            static_cast<unsigned long>(
                dynamic_point_overlay_frozen_sequence_));
  return true;
}

void GridMap::endDynamicPointPlanningSnapshot()
{
  std::lock_guard<std::mutex> lock(dynamic_point_overlay_mutex_);
  if (!dynamic_point_overlay_frozen_)
    return;
  ROS_DEBUG("[DynamicPointFlow][overlay] Release planning snapshot seq=%lu.",
            static_cast<unsigned long>(
                dynamic_point_overlay_frozen_sequence_));
  dynamic_point_overlay_frozen_ = false;
  dynamic_point_overlay_frozen_sequence_ = 0;
}

bool GridMap::dynamicPointPlanningOverlayReadyFor(
    std::uint64_t minimum_sequence) const
{
  std::lock_guard<std::mutex> lock(dynamic_point_overlay_mutex_);
  return dynamic_point_overlay_ready_ &&
         dynamic_point_overlay_sequence_ >= minimum_sequence;
}

void GridMap::depthPoseCallback(const sensor_msgs::ImageConstPtr &img,
                                const geometry_msgs::PoseStampedConstPtr &pose)
{
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    noteLidarHealthMonitorStart();
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if(md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;
  // Save base orientation (used to rotate LiDAR offset to world frame)
  md_.base_r_m_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                     odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y,
                                     odom->pose.pose.orientation.z)
                      .toRotationMatrix();

  md_.has_odom_ = true;
  noteLidarHealthMonitorStart();

  if (fusedGlobalMapOwnsESDFBounds() && md_.has_cloud_ &&
      mp_.local_esdf_follow_odom_)
  {
    const bool initial_static_esdf_waiting =
        md_.static_map_esdf_pending_ && !md_.global_esdf_pending_;
    updateLocalBoundsAroundRobot(initial_static_esdf_waiting);
  }
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  const ros::SteadyTime callback_received_steady = ros::SteadyTime::now();

  if (mp_.use_global_map_ && md_.has_cloud_ && img->header.stamp == md_.global_map_stamp_)
  {
    return;
  }

  if (mp_.use_global_map_ && !mp_.fuse_global_map_)
  {
    ROS_INFO("[GridMap] Static global map is not fused into local ESDF; live LiDAR is used for collision avoidance.");
    md_.has_cloud_ = true;
    md_.global_map_stamp_ = img->header.stamp;
    return;
  }

  if (!mp_.use_global_map_ &&
      !liveLidarMessageStampFresh(img->header.stamp, "independent"))
    return;

  if (mp_.use_global_map_)
  {
    // Static-map samples are consumed as world coordinates; silently accepting
    // another frame would turn a valid PCD into obstacles at the wrong place.
    auto normalize_frame = [](std::string frame) {
      while (!frame.empty() && frame.front() == '/')
        frame.erase(frame.begin());
      return frame;
    };
    const std::string cloud_frame = normalize_frame(img->header.frame_id);
    const std::string map_frame = normalize_frame(mp_.frame_id_);
    if (cloud_frame.empty() || cloud_frame != map_frame)
    {
      ROS_ERROR("[GridMap] Rejected static global map in frame '%s'; expected '%s'.",
                img->header.frame_id.c_str(), mp_.frame_id_.c_str());
      return;
    }
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  // A static global map is already expressed in world coordinates and does not
  // depend on the robot pose.  Requiring odometry here creates a startup race:
  // a latched map received before the first odometry sample is otherwise dropped
  // forever.  The local/live-cloud branch still requires odometry below.
  if (!mp_.use_global_map_ && !md_.has_odom_)
  {
    ROS_WARN_THROTTLE(2.0, "[GridMap] Waiting for odometry before loading local map...");
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (!mp_.use_global_map_ &&
      (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))))
    return;

  if (mp_.use_global_map_)
  {
    // Static-map load/reload changes the meaning of every ESDF cell. Results
    // captured before this boundary must never be committed afterwards.
    invalidateAsyncESDFJobs();

    if (md_.has_cloud_)
    {
      ROS_WARN("[GridMap] Reload static global map: stamp %.3f -> %.3f",
               md_.global_map_stamp_.toSec(), img->header.stamp.toSec());
      clearStaticGlobalMap();
      // Clearing the previous static layer is itself a coherent occupancy
      // mutation. Advance the revision even if the replacement PCD is later
      // rejected, so no consumer can identify the cleared map as the old
      // generation.
      ++md_.dynamic_observation_seq_;
      md_.has_cloud_ = false;
      md_.global_map_stamp_ = ros::Time(0);
      md_.static_map_esdf_pending_ = false;
      md_.static_map_esdf_ready_ = false;
      md_.global_esdf_pending_ = false;
    }

    // ===== Global/static map (simulation with map_generator) =====
    // Direct write to occupancy, single-shot accumulation, no clearing.
    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    int inf_step;
    if(mp_.global_map_obstacles_inflation_ > 1e-3)
      inf_step = ceil(mp_.global_map_obstacles_inflation_ / mp_.resolution_);
    else
      inf_step = 0;
    int inf_step_z = inf_step;
    size_t accepted_points = 0;
    size_t static_voxels_added = 0;
    size_t nonfinite_points = 0;

    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
      pt = latest_cloud.points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        ++nonfinite_points;
        continue;
      }
      if (pt.z < mp_.ground_height_ + 0.15) continue;

      p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
      if (!isInMap(p3d)) continue;
      ++accepted_points;

      Eigen::Vector3i inf_pt;
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {
            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            posToIndex(p3d_inf, inf_pt);
            if (!isInMap(inf_pt)) continue;

            int idx_inf = toAddress(inf_pt);
            if (!md_.static_occupancy_[idx_inf])
              ++static_voxels_added;
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
            md_.static_occupancy_[idx_inf] = 1;
            md_.static_miss_count_[idx_inf] = 0;
            md_.occupancy_buffer_[idx_inf] = mp_.clamp_max_log_;
          }
    }

    if (nonfinite_points > 0)
      ROS_WARN("[GridMap] Ignored %zu non-finite points in the static global map.",
               nonfinite_points);

    if (static_voxels_added == 0)
    {
      ROS_ERROR("[GridMap] Rejected static global map: input=%zu, accepted=%zu, "
                "occupied voxels=0. Check frame, height and configured map bounds.",
                latest_cloud.points.size(), accepted_points);
      return;
    }

    ROS_INFO("[GridMap] Static global map loaded: input=%zu, accepted=%zu, "
             "occupied voxels=%zu (inflation=%.3fm).",
             latest_cloud.points.size(), accepted_points, static_voxels_added,
             mp_.global_map_obstacles_inflation_);

    // Loading/reloading static occupancy is also a coherent map mutation.
    // Static ESDF remains synchronous, but the common revision prevents any
    // same-epoch live snapshot from being mistaken for this map state.
    ++md_.dynamic_observation_seq_;

    md_.has_cloud_ = true;
    md_.global_map_stamp_ = img->header.stamp;
    md_.static_map_esdf_pending_ = true;
    md_.static_map_esdf_ready_ = false;
    // The direct static-map mutation covers the whole map.  The next live
    // raycast will replace only this occupancy dirty-region metadata; it must
    // not replace the independent ESDF window below.
    md_.occupancy_bound_min_ = Eigen::Vector3i::Zero();
    md_.occupancy_bound_max_ =
        mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

    if (mp_.global_map_full_esdf_)
    {
      md_.local_bound_min_ << 0, 0, 0;
      md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
      md_.esdf_need_update_ = true;
      md_.global_esdf_pending_ = true;
      ROS_INFO("[GridMap] Static global map fused into full ESDF once.");
    }
    else if (mp_.local_esdf_follow_odom_)
    {
      updateLocalBoundsAroundRobot(true);
    }
    else
    {
      md_.local_bound_min_ << 0, 0, 0;
      md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
      md_.esdf_need_update_ = true;
    }
  }
  else
  {
    // ===== Local dynamic map (real robot with LiDAR) =====
    // Populate proj_points_ and trigger the raycasting pipeline,
    // using the LiDAR's actual position as the ray origin.
    // LiDAR offset relative to base_link (from URDF: mount 0.25 0 0.15 + scan 0 0 0.0377)
    static const Eigen::Vector3d LIDAR_OFFSET(0.25, 0.0, 0.1877);
    Eigen::Vector3d lidar_offset_world = md_.base_r_m_ * LIDAR_OFFSET;
    const Eigen::Vector3d lidar_origin = md_.camera_pos_ + lidar_offset_world;
    if (!lidar_origin.allFinite() || !isInMap(lidar_origin))
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GridMap] Rejected live LiDAR cloud: sensor origin is invalid or outside the map.");
      return;
    }

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d;
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> projected_points;
    projected_points.reserve(std::min(
        latest_cloud.points.size(), md_.proj_points_.size()));
    std::size_t truncated_points = 0;
    std::size_t finite_input_points = 0;

    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
      pt = latest_cloud.points[i];

      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
          !std::isfinite(pt.z))
        continue;
      ++finite_input_points;

      // Filter ground points
      if (pt.z < mp_.ground_height_ + 0.15) continue;

      p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

      // Filter robot body
      Eigen::Vector3d devi = p3d - md_.camera_pos_;
      if (devi.norm() < 1.0) continue;

      // Keep only points within the local update range
      if (fabs(devi(0)) < mp_.local_update_range_(0) &&
          fabs(devi(1)) < mp_.local_update_range_(1) &&
          fabs(devi(2)) < mp_.local_update_range_(2))
      {
        if (projected_points.size() < md_.proj_points_.size())
          projected_points.push_back(p3d);
        else
          ++truncated_points;
      }
    }

    // A fresh finite frame proves that the transport and transform chain is
    // alive even when every return is ground/body/range-filtered. It must not
    // clear occupancy or advance a LiDAR generation, but it does satisfy the
    // independent sensor-health heartbeat.
    if (finite_input_points > 0)
      md_.last_lidar_input_steady_ = callback_received_steady;

    if (projected_points.empty())
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GridMap] Rejected live LiDAR cloud with no usable obstacle returns; preserving the previous dynamic map generation.");
      return;
    }
    if (truncated_points > 0)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GridMap] Truncated %zu valid LiDAR points at the projection buffer capacity (%zu).",
          truncated_points, md_.proj_points_.size());
    }

    // Commit a fully validated observation atomically. A rejected frame must
    // not overwrite a valid observation that is still awaiting fusion.
    if (md_.occ_need_update_)
      ++md_.coalesced_cloud_count_;
    std::copy(projected_points.begin(), projected_points.end(),
              md_.proj_points_.begin());
    md_.proj_points_cnt = static_cast<int>(projected_points.size());
    md_.lidar_origin_ = lidar_origin;
    md_.lidar_origin_valid_ = true;
    ++md_.lidar_accepted_seq_;
    md_.lidar_pending_seq_ = md_.lidar_accepted_seq_;
    md_.pending_lidar_received_steady_ = callback_received_steady;
    md_.pending_lidar_stamp_ = img->header.stamp;
    md_.last_lidar_input_steady_ = callback_received_steady;

    // Trigger the occupancy update pipeline (raycastProcess + clearAndInflateLocalMap)
    md_.occ_need_update_ = true;
    md_.has_cloud_ = true;
  }
}

void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.occupancy_bound_min_;
  Eigen::Vector3i max_cut = md_.occupancy_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.occupancy_bound_min_;
  Eigen::Vector3i max_cut = md_.occupancy_bound_max_;
  // Eigen::Vector3i min_cut;
  // min_cut.setZero();
  // Eigen::Vector3i max_cut;
  // max_cut(0) = mp_.map_voxel_num_(0) - 1;
  // max_cut(1) = mp_.map_voxel_num_(1) - 1;
  // max_cut(2) = mp_.map_voxel_num_(2) - 1;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

void GridMap::getVoxelNum(Eigen::Vector3i &voxel_num){
  voxel_num = mp_.map_voxel_num_;
}

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::OdometryConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::ImageConstPtr &img,
                                const nav_msgs::OdometryConstPtr &odom)
{
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}

void GridMap::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                const nav_msgs::OdometryConstPtr &odom){
  const ros::SteadyTime callback_received_steady = ros::SteadyTime::now();

  // Save base orientation from odometry (used by global map cloudCallback)
  md_.base_r_m_ = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                     odom->pose.pose.orientation.x,
                                     odom->pose.pose.orientation.y,
                                     odom->pose.pose.orientation.z)
                      .toRotationMatrix();
  md_.has_odom_ = true;
  noteLidarHealthMonitorStart();

  if (!liveLidarMessageStampFresh(cloud->header.stamp, "synchronized"))
    return;

  // Look up cloud_frame → world transform at the exact scan timestamp.
  // This is critical during rotation: using a later timestamp would apply
  // extra rotation, making point clouds appear to rotate with the robot.
  // Two-step fallback: (1) cloud stamp → (2) latest TF → (3) odometry.
  try {
    geometry_msgs::TransformStamped tf_stamped;
    try {
      // Best: exact scan timestamp for accurate orientation
      tf_stamped = tf_buffer_->lookupTransform("world", cloud->header.frame_id,
                                                cloud->header.stamp, ros::Duration(0.02));
    } catch (tf2::ExtrapolationException &ex) {
      // Cloud stamp slightly ahead of TF data (<1ms) — use latest available
      tf_stamped = tf_buffer_->lookupTransform("world", cloud->header.frame_id,
                                                ros::Time(0), ros::Duration(0.05));
    }
    const auto &t = tf_stamped.transform.translation;
    const auto &q = tf_stamped.transform.rotation;
    md_.camera_pos_(0) = t.x;
    md_.camera_pos_(1) = t.y;
    md_.camera_pos_(2) = t.z;
    md_.camera_r_m_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
  } catch (tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(2.0, "[GridMap] TF lookup %s→world failed: %s. Falling back to odom.",
                      cloud->header.frame_id.c_str(), ex.what());
    // Last resort: use the synced odometry
    Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                   odom->pose.pose.orientation.x,
                                                   odom->pose.pose.orientation.y,
                                                   odom->pose.pose.orientation.z);
    Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
    Eigen::Matrix4d body2world;
    body2world.block<3, 3>(0, 0) = body_r_m;
    body2world(0, 3) = odom->pose.pose.position.x;
    body2world(1, 3) = odom->pose.pose.position.y;
    body2world(2, 3) = odom->pose.pose.position.z;
    body2world(3, 3) = 1.0;
    Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
    md_.camera_pos_(0) = cam_T(0, 3);
    md_.camera_pos_(1) = cam_T(1, 3);
    md_.camera_pos_(2) = cam_T(2, 3);
    md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);
  }
  Eigen::Vector3d base_pos(odom->pose.pose.position.x,
                           odom->pose.pose.position.y,
                           odom->pose.pose.position.z);

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*cloud, latest_cloud);

  if (latest_cloud.points.size() == 0)
    return;

  const Eigen::Vector3d lidar_origin = md_.camera_pos_;
  if (!lidar_origin.allFinite() || !md_.camera_r_m_.allFinite() ||
      !isInMap(lidar_origin))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap] Rejected synchronized LiDAR cloud: transform origin is invalid or outside the map.");
    return;
  }

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, pt_world;
  std::vector<Eigen::Vector3d,
              Eigen::aligned_allocator<Eigen::Vector3d>> projected_points;
  projected_points.reserve(std::min(
      latest_cloud.points.size(), md_.proj_points_.size()));
  std::size_t truncated_points = 0;
  std::size_t finite_input_points = 0;

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    // p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
        !std::isfinite(pt.z))
      continue;

    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    pt_world = md_.camera_r_m_ * p3d + md_.camera_pos_;
    if (!pt_world.allFinite())
      continue;
    ++finite_input_points;

    if (fabs(pt.x) < mp_.local_update_range_(0) && fabs(pt.y) < mp_.local_update_range_(1) &&
        fabs(pt.z) < mp_.local_update_range_(2)){
      if(pt.x > -0.3){  // keep forward points, discard only points behind LiDAR (robot body)
          // ====== 过滤世界坐标系中 z=0.05 以下的地面点云 ======
          if (pt_world(2) < mp_.ground_height_ + 0.15) continue;

        if (mp_.self_filter_enable_) {
          Eigen::Vector3d pt_base = md_.base_r_m_.transpose() * (pt_world - base_pos);
          if ((pt_base.array() >= mp_.self_filter_min_.array()).all() &&
              (pt_base.array() <= mp_.self_filter_max_.array()).all()) {
            continue;
          }
        }

        if (projected_points.size() < md_.proj_points_.size())
          projected_points.push_back(pt_world);
        else
          ++truncated_points;
      }
    }
  }

  // Health is based on fresh, finite, transformable input rather than on
  // whether this particular scan happens to contain an obstacle return after
  // filtering. The latter would falsely declare an open/ground-only scene a
  // sensor outage.
  if (finite_input_points > 0)
    md_.last_lidar_input_steady_ = callback_received_steady;

  if (truncated_points > 0)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap] Truncated %zu valid LiDAR points at the projection buffer capacity (%zu).",
        truncated_points, md_.proj_points_.size());
  }

  if (projected_points.empty())
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GridMap] Rejected synchronized LiDAR cloud with no usable obstacle returns; preserving the previous dynamic map generation.");
    return;
  }

  // The map pipeline deliberately keeps only the newest *valid* observation.
  // Counting replacements makes overload visible without allowing a rejected
  // frame to destroy an older scan which is still awaiting fusion.
  if (md_.occ_need_update_)
    ++md_.coalesced_cloud_count_;
  std::copy(projected_points.begin(), projected_points.end(),
            md_.proj_points_.begin());
  md_.proj_points_cnt = static_cast<int>(projected_points.size());
  md_.lidar_origin_ = lidar_origin;
  md_.lidar_origin_valid_ = true;
  ++md_.lidar_accepted_seq_;
  md_.lidar_pending_seq_ = md_.lidar_accepted_seq_;
  md_.pending_lidar_received_steady_ = callback_received_steady;
  md_.pending_lidar_stamp_ = cloud->header.stamp;
  md_.last_lidar_input_steady_ = callback_received_steady;
  
  md_.occ_need_update_ = true;
  md_.flag_use_depth_fusion = true;
}

template <typename F_get_val, typename F_set_val>
void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
  int v[mp_.map_voxel_num_(dim)];
  double z[mp_.map_voxel_num_(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q)
      k++;
    double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}

void GridMap::updateESDF3d (){
  const Eigen::Vector3i min_esdf = md_.local_bound_min_;
  const Eigen::Vector3i max_esdf = md_.local_bound_max_;
  const Eigen::Vector3i size =
      max_esdf - min_esdf + Eigen::Vector3i::Ones();
  const int nx = size.x();
  const int ny = size.y();
  const int nz = size.z();

  const auto line_threshold = [&](int line_length) {
    return std::max(1, mp_.parallel_voxel_threshold_ /
                           std::max(1, line_length));
  };
  const auto address = [&](int x, int y, int z) {
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
           y * mp_.map_voxel_num_(2) + z;
  };

  /* ========== compute positive DT ========== */
  boundedParallelFor(
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
      mp_.parallel_threads_, line_threshold(nz),
      [&](std::size_t line) {
        int x = min_esdf.x() + static_cast<int>(line / ny);
        int y = min_esdf.y() + static_cast<int>(line % ny);
        fillESDF(
            [&](int z) {
              return md_.occupancy_buffer_inflate_[address(x, y, z)] == 1
                         ? 0.0
                         : std::numeric_limits<double>::max();
            },
            [&](int z, double val) {
              md_.tmp_buffer1_[address(x, y, z)] = val;
            },
            min_esdf.z(), max_esdf.z(), 2);
      });

  boundedParallelFor(
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(nz),
      mp_.parallel_threads_, line_threshold(ny),
      [&](std::size_t line) {
        int x = min_esdf.x() + static_cast<int>(line / nz);
        int z = min_esdf.z() + static_cast<int>(line % nz);
        fillESDF(
            [&](int y) { return md_.tmp_buffer1_[address(x, y, z)]; },
            [&](int y, double val) {
              md_.tmp_buffer2_[address(x, y, z)] = val;
            },
            min_esdf.y(), max_esdf.y(), 1);
      });

  boundedParallelFor(
      static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz),
      mp_.parallel_threads_, line_threshold(nx),
      [&](std::size_t line) {
        int y = min_esdf.y() + static_cast<int>(line / nz);
        int z = min_esdf.z() + static_cast<int>(line % nz);
        fillESDF(
            [&](int x) { return md_.tmp_buffer2_[address(x, y, z)]; },
            [&](int x, double val) {
              md_.distance_buffer_[address(x, y, z)] =
                  mp_.resolution_ * std::sqrt(val);
            },
            min_esdf.x(), max_esdf.x(), 0);
      });

  /* ========== compute negative distance ========== */
  const std::size_t yz_size =
      static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
  const std::size_t voxel_count =
      static_cast<std::size_t>(nx) * yz_size;
  boundedParallelFor(
      voxel_count, mp_.parallel_threads_, mp_.parallel_voxel_threshold_,
      [&](std::size_t linear_index) {
        const int x = min_esdf.x() +
            static_cast<int>(linear_index / yz_size);
        const std::size_t yz_index = linear_index % yz_size;
        const int y = min_esdf.y() + static_cast<int>(yz_index / nz);
        const int z = min_esdf.z() + static_cast<int>(yz_index % nz);
        const int index = address(x, y, z);
        md_.occupancy_buffer_neg_[index] =
            md_.occupancy_buffer_inflate_[index] == 0 ? 1 : 0;
      });

  // tmp_buffer1_/tmp_buffer2_ are reused below. Each pass writes disjoint
  // complete grid lines, and boundedParallelFor joins before the next pass.
  boundedParallelFor(
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
      mp_.parallel_threads_, line_threshold(nz),
      [&](std::size_t line) {
        int x = min_esdf.x() + static_cast<int>(line / ny);
        int y = min_esdf.y() + static_cast<int>(line % ny);
        fillESDF(
            [&](int z) {
              return md_.occupancy_buffer_neg_[address(x, y, z)] == 1
                         ? 0.0
                         : std::numeric_limits<double>::max();
            },
            [&](int z, double val) {
              md_.tmp_buffer1_[address(x, y, z)] = val;
            },
            min_esdf.z(), max_esdf.z(), 2);
      });

  boundedParallelFor(
      static_cast<std::size_t>(nx) * static_cast<std::size_t>(nz),
      mp_.parallel_threads_, line_threshold(ny),
      [&](std::size_t line) {
        int x = min_esdf.x() + static_cast<int>(line / nz);
        int z = min_esdf.z() + static_cast<int>(line % nz);
        fillESDF(
            [&](int y) { return md_.tmp_buffer1_[address(x, y, z)]; },
            [&](int y, double val) {
              md_.tmp_buffer2_[address(x, y, z)] = val;
            },
            min_esdf.y(), max_esdf.y(), 1);
      });

  boundedParallelFor(
      static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz),
      mp_.parallel_threads_, line_threshold(nx),
      [&](std::size_t line) {
        int y = min_esdf.y() + static_cast<int>(line / nz);
        int z = min_esdf.z() + static_cast<int>(line % nz);
        fillESDF(
            [&](int x) { return md_.tmp_buffer2_[address(x, y, z)]; },
            [&](int x, double val) {
              md_.distance_buffer_neg_[address(x, y, z)] =
                  mp_.resolution_ * std::sqrt(val);
            },
            min_esdf.x(), max_esdf.x(), 0);
      });

  /* ========== combine positive and negative DT ========== */
  boundedParallelFor(
      voxel_count, mp_.parallel_threads_, mp_.parallel_voxel_threshold_,
      [&](std::size_t linear_index) {
        const int x = min_esdf.x() +
            static_cast<int>(linear_index / yz_size);
        const std::size_t yz_index = linear_index % yz_size;
        const int y = min_esdf.y() + static_cast<int>(yz_index / nz);
        const int z = min_esdf.z() + static_cast<int>(yz_index % nz);
        const int index = address(x, y, z);
        md_.distance_buffer_all_[index] = md_.distance_buffer_[index];
        if (md_.distance_buffer_neg_[index] > 0.0)
        {
          md_.distance_buffer_all_[index] +=
              -md_.distance_buffer_neg_[index] + mp_.resolution_;
        }
      });
}

void GridMap::publishESDF(){
  if (esdf_pub_.getNumSubscribers() <= 0)
    return;
  double dist;
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt;

  const double min_dist = -3.0;
  const double max_dist = 3.0;

  // Eigen::Vector3i min_cut = local_bound_min_ -
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  // Eigen::Vector3i max_cut = local_bound_max_ +
  //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;
  // Eigen::Vector3i min_cut;
  // min_cut.setZero();
  // Eigen::Vector3i max_cut;
  // max_cut(0) = mp_.map_voxel_num_(0) - 1;
  // max_cut(1) = mp_.map_voxel_num_(1) - 1;
  // max_cut(2) = mp_.map_voxel_num_(2) - 1;
  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); x += mp_.esdf_visualization_stride_)
    for (int y = min_cut(1); y <= max_cut(1); y += mp_.esdf_visualization_stride_) {

      Eigen::Vector3d pos;
      indexToPos(Eigen::Vector3i(x, y, -0.5), pos);
      pos(2) = mp_.esdf_slice_height_;

      dist = getDistance(pos);
      dist = min(dist, max_dist);
      dist = max(dist, min_dist);

      pt.x = pos(0);
      pt.y = pos(1);
      pt.z = pos(2);
      pt.intensity = (dist - min_dist) / (max_dist - min_dist);
      cloud.push_back(pt);
    }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);

  esdf_pub_.publish(cloud_msg);
}

/* use for ESDF API*/

void GridMap::getSurroundPts(const Eigen::Vector3d& pos, 
                             Eigen::Vector3d pts[2][2][2],
                             Eigen::Vector3d& diff){
  /* interpolation position */
  Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
  Eigen::Vector3i idx;
  Eigen::Vector3d idx_pos;

  posToIndex(pos_m, idx);
  indexToPos(idx, idx_pos);
  diff = (pos - idx_pos) * mp_.resolution_inv_;

  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        Eigen::Vector3d current_pos;
        indexToPos(current_idx, current_pos);
        pts[x][y][z] = current_pos;
      }
    }
  }
}

void GridMap::getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]){
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
          dists[x][y][z] = getDistance(pts[x][y][z]);
      }
    }
  }
}

pair<double, Eigen::Vector3d> GridMap::interpolateTrilinear(double values[2][2][2],
                                                                   const Eigen::Vector3d& diff,
                                                                   double& value,
                                                                   Eigen::Vector3d& grad) {
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

  value = (1 - diff(2)) * v0 + diff(2) * v1;

  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_.resolution_inv_;

  return std::make_pair(value,grad);
}


void GridMap::interpolateTrilinearEDT(double values[2][2][2],
                                      const Eigen::Vector3d& diff,
                                      double& value){
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // e
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // f
  
  value = (1 - diff(2)) * v0 + diff(2) * v1;
}

void GridMap::interpolateTrilinearFirstGrad(double values[2][2][2],
                                            const Eigen::Vector3d& diff,
                                            Eigen::Vector3d& grad){
  // trilinear interpolation
  double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0]; // b
  double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1]; // d
  double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0]; // a
  double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1]; // c
  double v0 = (1 - diff(1)) * v00 + diff(1) * v10;  // e
  double v1 = (1 - diff(1)) * v01 + diff(1) * v11;  // f
  
  grad[2] = (v1 - v0) * mp_.resolution_inv_;
  grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_.resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
  grad[0] *= mp_.resolution_inv_;
}

void GridMap::evaluateEDT(const Eigen::Vector3d& pos,
                          double& dist){
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearEDT(dists, diff, dist);
}

double GridMap::getPreciseDistance(const Eigen::Vector3d& pos){
  double dist;
  evaluateEDT(pos, dist);
  return dist;
}

bool GridMap::getDynamicPointPlanningDistance(
    const Eigen::Vector3d &pos, double max_search_distance,
    double &distance, Eigen::Vector3d *gradient) const
{
  distance = max_search_distance;
  if (gradient != nullptr)
    gradient->setZero();
  if (!mp_.dynamic_point_overlay_enable_ || !pos.allFinite() ||
      std::isnan(max_search_distance) || max_search_distance <= 0.0)
    return false;

  std::lock_guard<std::mutex> lock(dynamic_point_overlay_mutex_);
  if (!dynamic_point_overlay_ready_ ||
      !dynamic_point_overlay_cloud_ ||
      dynamic_point_overlay_cloud_->empty() ||
      dynamic_point_overlay_radii_.size() !=
          dynamic_point_overlay_cloud_->size())
    return false;

  // The dynamic layer can only affect collision/penalty decisions inside a
  // bounded near field.  Reject remote queries before allocating vectors or
  // traversing the KD-tree; otherwise every arm sample pays a global nearest-
  // neighbour lookup even when the moving obstacle is several metres away.
  const double influence_distance = std::min(
      max_search_distance, mp_.dynamic_point_overlay_influence_distance_);
  const double geometric_search_limit =
      influence_distance + dynamic_point_overlay_inflation_;
  for (int axis = 0; axis < 3; ++axis)
  {
    if (pos(axis) < dynamic_point_overlay_min_(axis) - geometric_search_limit ||
        pos(axis) > dynamic_point_overlay_max_(axis) + geometric_search_limit)
      return false;
  }

  pcl::PointXYZ query;
  query.x = static_cast<float>(pos.x());
  query.y = static_cast<float>(pos.y());
  query.z = static_cast<float>(pos.z());
  std::vector<int> candidate_indices;
  std::vector<float> candidate_squared_distances;
  if (dynamic_point_overlay_kdtree_.radiusSearch(
          query, geometric_search_limit, candidate_indices,
          candidate_squared_distances) <= 0)
    return false;

  double best_clearance = std::numeric_limits<double>::infinity();
  double best_geometric_distance = 0.0;
  int best_index = -1;
  for (std::size_t i = 0; i < candidate_indices.size(); ++i)
  {
    const int index = candidate_indices[i];
    if (index < 0 ||
        static_cast<std::size_t>(index) >= dynamic_point_overlay_radii_.size())
      continue;
    const double geometric_distance = std::sqrt(std::max(
        0.0, static_cast<double>(candidate_squared_distances[i])));
    const double clearance = geometric_distance -
        static_cast<double>(dynamic_point_overlay_radii_[index]);
    if (clearance < best_clearance)
    {
      best_clearance = clearance;
      best_geometric_distance = geometric_distance;
      best_index = index;
    }
  }
  if (best_index < 0 || best_clearance > influence_distance)
    return false;
  distance = best_clearance;
  if (gradient != nullptr && best_geometric_distance > 1e-6)
  {
    const pcl::PointXYZ &nearest =
        dynamic_point_overlay_cloud_->points[best_index];
    *gradient = (pos - Eigen::Vector3d(
        nearest.x, nearest.y, nearest.z)) / best_geometric_distance;
  }
  return true;
}

double GridMap::getPlanningDistance(
    const Eigen::Vector3d &pos, bool precise)
{
  double distance = precise ? getPreciseDistance(pos) : getDistance(pos);
  double dynamic_distance = distance;
  if (getDynamicPointPlanningDistance(
          pos, std::max(distance, 0.0), dynamic_distance, nullptr))
    distance = std::min(distance, dynamic_distance);
  return distance;
}

pair<double, Eigen::Vector3d> GridMap::evaluatePlanningEDTWithGrad(
    const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad)
{
  evaluateEDTWithGrad(pos, dist, grad);
  double dynamic_distance = dist;
  Eigen::Vector3d dynamic_gradient = Eigen::Vector3d::Zero();
  if (getDynamicPointPlanningDistance(
          pos, std::max(dist, 0.0), dynamic_distance,
          &dynamic_gradient) && dynamic_distance < dist)
  {
    dist = dynamic_distance;
    if (dynamic_gradient.squaredNorm() <= 1e-12)
    {
      // Exact overlap with a sampled swept point has no geometric direction.
      // Reuse a valid static-ESDF direction when available; otherwise choose a
      // deterministic finite direction so LBFGS can leave the singular point.
      dynamic_gradient =
          grad.squaredNorm() > 1e-12 ? grad.normalized() :
                                       Eigen::Vector3d::UnitX();
    }
    grad = dynamic_gradient;
  }
  return std::make_pair(dist, grad);
}

double GridMap::getDynamicObstacleDistance(const Eigen::Vector3d& pos, double max_search_distance){
  if (!isInMap(pos))
    return max_search_distance;

  Eigen::Vector3i center_id;
  posToIndex(pos, center_id);
  const int search_step = std::max(0, static_cast<int>(std::ceil(max_search_distance * mp_.resolution_inv_))) + 1;
  const int static_match_step = std::max(
      0, static_cast<int>(std::ceil(mp_.dynamic_static_match_radius_ * mp_.resolution_inv_)));
  const double static_match_radius_sq =
      mp_.dynamic_static_match_radius_ * mp_.dynamic_static_match_radius_;
  double min_dist = max_search_distance;

  const auto matches_static_map = [&](const Eigen::Vector3i& dynamic_id) {
    if (static_match_step == 0)
      return md_.static_occupancy_[toAddress(dynamic_id)] != 0;

    for (int sx = -static_match_step; sx <= static_match_step; ++sx)
      for (int sy = -static_match_step; sy <= static_match_step; ++sy)
        for (int sz = -static_match_step; sz <= static_match_step; ++sz)
        {
          const Eigen::Vector3i static_id = dynamic_id + Eigen::Vector3i(sx, sy, sz);
          if (!isInMap(static_id) || !md_.static_occupancy_[toAddress(static_id)])
            continue;
          if ((static_id - dynamic_id).cast<double>().squaredNorm() * mp_.resolution_ * mp_.resolution_ <=
              static_match_radius_sq)
            return true;
        }
    return false;
  };

  for (int dx = -search_step; dx <= search_step; ++dx)
    for (int dy = -search_step; dy <= search_step; ++dy)
      for (int dz = -search_step; dz <= search_step; ++dz)
      {
        const Eigen::Vector3i id = center_id + Eigen::Vector3i(dx, dy, dz);
        if (!isInMap(id))
          continue;

        const int idx = toAddress(id);
        if (md_.occupancy_buffer_[idx] <= mp_.min_occupancy_log_ || matches_static_map(id))
          continue;

        Eigen::Vector3d voxel_center;
        indexToPos(id, voxel_center);
        // ESDF collision checks measure distance to occupied voxel centers.
        // Keep the live layer on the same convention so a trajectory accepted
        // by fresh ESDF data is not rejected by a second, larger implicit margin.
        const double dist = (voxel_center - pos).norm();
        min_dist = std::min(min_dist, dist);
      }

  return min_dist;
}

void GridMap::evaluateFirstGrad(const Eigen::Vector3d& pos,
                                Eigen::Vector3d& grad){
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);

  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateTrilinearFirstGrad(dists, diff, grad);
}

pair<double, Eigen::Vector3d> GridMap::evaluateEDTWithGrad(const Eigen::Vector3d& pos, double& dist,
                                                                  Eigen::Vector3d& grad) {
  Eigen::Vector3d diff;
  Eigen::Vector3d sur_pts[2][2][2];
  getSurroundPts(pos, sur_pts, diff);
  double dists[2][2][2];
  getSurroundDistance(sur_pts, dists);

  return interpolateTrilinear(dists, diff, dist, grad);
}

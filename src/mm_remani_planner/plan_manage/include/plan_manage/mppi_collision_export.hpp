#pragma once
// Adapted from experiments/online_remani_validation/native_collision_export.hpp.
// Immutable Complete snapshots can be sampled and published by the existing
// single output worker. Legacy maps/models retain serialized-map sampling.
#include <mm_config/mm_config.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Empty.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace mppi_collision_export {

// Schema 3.1: header32, body7, dynamic7, optional ESDF doubles.
// Body = base-frame xyz, hard radius, soft radius, weight, base0/arm1.
// ESDF is x-major ((x*ny)+y)*nz+z; header origin is a voxel EDGE.
// h3 = original map fusion time; h14 = exported field cache version;
// h29 = selected complete native occupancy/ESDF seq; h30/31 = its LiDAR seqs.
struct Cache {
  std::array<double, 32> header{};
  std::vector<double> body;
  std::shared_ptr<const std::vector<double>> esdf;
  std::uint64_t generation = 0;
  double odom_stamp = 0.;
};

class Exporter {
  ros::NodeHandle nh_;
  std::shared_ptr<GridMap> map_;
  remani_planner::MMConfig::Ptr model_;
  Eigen::VectorXd q_;
  Eigen::Vector2d position_ = Eigen::Vector2d::Zero();
  double joint_stamp_ = 0., odom_stamp_ = 0.;
  bool have_joints_ = false, have_odom_ = false;
  double arm_radius_, car_margin_, arm_margin_, weight_, ground_;
  double max_age_, max_prediction_, pos_unc_, vel_unc_, acc_unc_;
  double state_max_age_, crop_half_extent_;
  int max_cells_;
  bool dynamic_enabled_;
  Eigen::Vector3i crop_lo_ = Eigen::Vector3i::Constant(-1), crop_hi_ = crop_lo_;
  std::pair<std::uint64_t, std::uint64_t> esdf_version_{};
  std::uint64_t generation_ = 0, sequence_ = 0, sent_generation_ = 0;
  int last_subscribers_ = -1;
  bool last_health_ = false;
  bool full_requested_ = false;
  double last_full_stamp_ = 0.;
  std::shared_ptr<const Cache> cache_;
  ros::CallbackQueue output_queue_;
  ros::Publisher publisher_, diagnostic_publisher_;
  int diagnostic_subscribers_ = 0, diagnostic_remaining_ = 0;
  ros::SteadyTime diagnostic_last_;
  ros::Subscriber odom_sub_, joint_sub_, resync_sub_;
  ros::Timer sample_timer_, output_timer_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;

  bool fresh(double stamp, double now) const {
    return std::isfinite(stamp) && stamp > 0. &&
           now - stamp >= -.05 && now - stamp <= state_max_age_;
  }
  void resync(const std_msgs::Empty::ConstPtr&) {
    // Runs on the output worker: coalesce requests without copying a field or
    // extending its original source time/validity.
    full_requested_ = true;
  }
  void odom(const nav_msgs::Odometry::ConstPtr& msg) {
    position_ = Eigen::Vector2d(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_stamp_ = msg->header.stamp.toSec();
    have_odom_ = position_.allFinite() && fresh(odom_stamp_, ros::Time::now().toSec());
  }
  void joints(const sensor_msgs::JointState::ConstPtr& msg) {
    have_joints_ = msg->position.size() == std::size_t(q_.size()) &&
                   msg->name.size() == std::size_t(q_.size());
    for (Eigen::Index i = 0; have_joints_ && i < q_.size(); ++i)
      have_joints_ = msg->name[i] == "joint" + std::to_string(i + 1) &&
                     std::isfinite(msg->position[i]);
    joint_stamp_ = msg->header.stamp.toSec();
    have_joints_ = have_joints_ && fresh(joint_stamp_, ros::Time::now().toSec());
    if (have_joints_)
      q_ = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), q_.size());
  }
  // Optional, bounded evidence only. A recorder subscribes on the user's goal;
  // no subscriber means no extra map copy. The existing control wire is unchanged.
  // Diagnostic schema: native header32, magic, schema, epoch, distance revision,
  // ESDF observation seq, acquisition ROS time, cell count, copy wall seconds;
  // then N exact ESDF doubles and N occupancy codes (bit0 raw, bit1 inflated).
  // Every value is sampled inside the caller's SAME immutable Complete scope.
  void diagnostic(const Cache& cache, const Eigen::Vector3i& lo,
                  const Eigen::Vector3i& hi,
                  const std::pair<std::uint64_t, std::uint64_t>& version,
                  std::uint64_t esdf_seq, double acquired) {
    const int subscribers = diagnostic_publisher_.getNumSubscribers();
    if (subscribers == 0) {
      diagnostic_subscribers_ = diagnostic_remaining_ = 0;
      return;
    }
    if (diagnostic_subscribers_ == 0) {
      diagnostic_remaining_ = 16;
      diagnostic_last_ = ros::SteadyTime();
    }
    diagnostic_subscribers_ = subscribers;
    const auto began = ros::SteadyTime::now();
    if (!map_->hasReadSnapshots() || diagnostic_remaining_ == 0 ||
        (!diagnostic_last_.isZero() && (began-diagnostic_last_).toSec() < 1.) ||
        double(esdf_seq) != cache.header[29]) return;
    // This method must never turn diagnostic failure into a control failure.
    try {
      const std::size_t cells = cache.esdf->size();
      std::vector<unsigned char> occupancy;
      if (!map_->copyOccupancyEvidence(lo,hi,occupancy) || occupancy.size()!=cells)
        return;
      std_msgs::Float64MultiArray message;
      auto& data = message.data;
      data.reserve(40 + 2*cells);
      data.assign(cache.header.begin(), cache.header.end());
      data.insert(data.end(), {20261112., 1., double(version.first),
        double(version.second), double(esdf_seq), acquired, double(cells), 0.});
      data.insert(data.end(), cache.esdf->begin(), cache.esdf->end());
      for (unsigned char code : occupancy) data.push_back(double(code));
      data[39] = (ros::SteadyTime::now()-began).toSec();
      diagnostic_publisher_.publish(message);
      diagnostic_last_ = began;
      --diagnostic_remaining_;
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(5., "[MPPI diagnostic] Evidence copy failed: %s", error.what());
    }
  }
  void sample(const ros::TimerEvent&) {
    const auto sample_began = ros::SteadyTime::now();
    if (!have_odom_ || !have_joints_ || publisher_.getNumSubscribers() == 0) return;
    GridMap::ReadScope map_read(*map_, GridMap::ReadScope::Complete);
    if (!map_read.valid()) {
      // At the next sample, an epoch reset revokes the old cache even if its
      // time budget remains. Output retains the original TTL in the interim;
      // a later complete generation can restore validity.
      const auto cached = std::atomic_load(&cache_);
      if (cached) {
        auto invalid = std::make_shared<Cache>(*cached);
        invalid->header[4] = 0.;
        std::shared_ptr<const Cache> immutable = invalid;
        std::atomic_store(&cache_, immutable);
      }
      return;
    }
    const double acquired = ros::Time::now().toSec();
    const double fusion = map_->getReadFusionTime().toSec();
    // Map age belongs to its original fusion, independently of current q/odom.
    // A fresh sample or repeated crop must never renew an old map's authority.
    if (!(fusion > 0.) || acquired - fusion < -.05 || acquired - fusion > max_age_) return;
    auto next = std::make_shared<Cache>();
    const auto previous = std::atomic_load(&cache_);
    auto& h = next->header;
    h[0] = 3.; h[1] = 1.; h[3] = fusion;
    h[11] = map_->getResolution(); h[15] = max_age_; h[16] = max_prediction_;
    h[21] = ground_; h[24] = joint_stamp_;
    h[25] = pos_unc_; h[26] = vel_unc_; h[27] = acc_unc_;
    h[28] = dynamic_enabled_ ? 1. : 0.;

    std::vector<Eigen::Vector3d> car;
    model_->getCarPts(Eigen::Vector3d::Zero(), car);
    for (std::size_t i = 0; i < car.size(); ++i) {
      const auto& p = car[i];
      const double radius = model_->getBaseSphereRadius(i);
      next->body.insert(next->body.end(), {p.x(), p.y(), p.z(),
        radius + model_->getMapResolution(), radius + car_margin_, weight_, 0.});
    }
    std::vector<Eigen::Matrix4d> transforms, unused;
    model_->getJointTrans(q_, transforms, unused);
    const auto points = model_->getLinkPoint();
    Eigen::Matrix4d transform = model_->getTq0();
    bool ground_ok = true;
    for (int i = 0; i < model_->getManiDof(); ++i) {
      transform *= transforms[i];
      for (Eigen::Index j = 0; j < points[i].cols(); ++j) {
        const Eigen::Vector3d p = (transform * points[i].col(j)).head<3>();
        ground_ok = ground_ok && p.allFinite() && p.z() >= ground_;
        next->body.insert(next->body.end(), {p.x(), p.y(), p.z(),
          arm_radius_, arm_radius_ + arm_margin_, weight_ / 5., 1.});
      }
    }
    double unused_distance = 0.;
    const bool self_ok = !model_->checkCarManiCollision(q_, false, unused_distance) &&
                        !model_->checkManiManiCollision(q_, false, unused_distance);
    h[12] = next->body.size() / 7; h[22] = ground_ok ? 1. : 0.; h[23] = self_ok ? 1. : 0.;

    Eigen::Vector3i dimensions, lo, hi;
    map_->getVoxelNum(dimensions);
    const auto origin = map_->getOrigin();
    const double res = h[11];
    if ((dimensions.array() < 2).any() || !origin.allFinite()) return;
    for (int a = 0; a < 2; ++a) {
      lo[a] = int(std::max(0., std::min(double(dimensions[a] - 1),
        std::floor((position_[a] - crop_half_extent_ - origin[a]) / res))));
      hi[a] = int(std::max(0., std::min(double(dimensions[a] - 1),
        std::ceil((position_[a] + crop_half_extent_ - origin[a]) / res))));
    }
    // Keep the full native vertical extent: a moving arm must not be cropped
    // to its currently folded height. Outside the crop the consumer rejects.
    lo.z() = 0; hi.z() = dimensions.z() - 1;
    const Eigen::Vector3i shape = hi - lo + Eigen::Vector3i::Ones();
    const std::uint64_t cells = std::uint64_t(shape.x()) * shape.y() * shape.z();
    if ((shape.array() < 2).any() || cells > std::uint64_t(max_cells_)) {
      ROS_WARN_THROTTLE(1., "[MPPI snapshot] Native-resolution crop exceeds bounds/cell budget; no export.");
      return;
    }
    const auto seq = map_->getESDFObservationSeq();
    const auto occupancy_seq = map_->getDynamicObservationSeq();
    const auto version = map_->getReadESDFVersion();
    // A newer fused observation with identical occupancy reuses the field;
    // its real fusion stamp/sequence still advance independently below.
    const bool refresh = !previous || version != esdf_version_ ||
                         lo != crop_lo_ || hi != crop_hi_;
    if (refresh) {
      auto field = std::make_shared<std::vector<double>>();
      if (!map_->copyDistanceRegion(lo, hi, *field)) return;
      h[17] = acquired;  // ESDF-copy acquisition time, never original sensor time.
      next->esdf = field;
      esdf_version_ = version; crop_lo_ = lo; crop_hi_ = hi; ++generation_;
    } else {
      next->esdf = previous->esdf;
      h[17] = previous->header[17];
    }
    for (int a = 0; a < 3; ++a) { h[5 + a] = shape[a]; h[8 + a] = origin[a] + lo[a] * res; }
    // h14 is the exported ESDF cache version, unique also across map epochs
    // and same-observation recomputations. h29 retains the native coherent seq.
    h[14] = double(generation_); h[29] = double(occupancy_seq);
    h[30] = double(map_->getLidarAcceptedSeq()); h[31] = double(map_->getLidarFusedSeq());
    next->generation = generation_;
    next->odom_stamp = odom_stamp_;
    const bool finite = std::all_of(next->body.begin(), next->body.end(), [](double v) { return std::isfinite(v); }) &&
      std::all_of(next->esdf->begin(), next->esdf->end(), [](double v) { return std::isfinite(v); });
    h[4] = fresh(odom_stamp_, acquired) && fresh(joint_stamp_, acquired) &&
      map_->mapReadyForPlanning() && seq >= occupancy_seq && finite && ground_ok && self_ok ? 1. : 0.;
    std::shared_ptr<const Cache> immutable = next;
    std::atomic_store(&cache_, immutable);
    ROS_DEBUG_NAMED("latency", "[LatencyTrace] stage=export_sample stamp=%.9f epoch=%lu rev=%lu obs=%lu field=%lu fusion=%.9f acquired=%.9f refresh=%d sample_ms=%.3f valid=%d odom_stamp=%.9f joint_stamp=%.9f",
      ros::Time::now().toSec(), static_cast<unsigned long>(version.first),
      static_cast<unsigned long>(version.second), static_cast<unsigned long>(occupancy_seq),
      static_cast<unsigned long>(generation_), fusion, acquired, int(refresh),
      (ros::SteadyTime::now()-sample_began).toSec()*1000., int(h[4]), odom_stamp_, joint_stamp_);
    diagnostic(*next, lo, hi, version, seq, acquired);
  }
  void output(const ros::TimerEvent&) {
    const auto cached = std::atomic_load(&cache_);
    if (!cached || publisher_.getNumSubscribers() == 0) return;
    const ros::Time now = ros::Time::now();
    const auto output_began = ros::SteadyTime::now();
    const auto dynamic = map_->getDynamicPointSnapshot(now);  // Thread-safe holder.
    auto h = cached->header;
    h[2] = double(++sequence_);
    h[18] = dynamic.stamp.toSec();
    // Dynamic observations are mandatory only when that layer is enabled.
    // Disabled is not the same as a stale enabled producer. Never invent a
    // sensor stamp or mark an enabled-but-unhealthy stream as disabled.
    h[19] = dynamic_enabled_ && dynamic.valid ? 1. : 0.;
    h[13] = h[19] == 1. ? dynamic.points.size() : 0.;
    const double cache_age = now.toSec() - h[3];
    h[4] = h[4] == 1. && (!dynamic_enabled_ || h[19] == 1.) && cache_age >= -.05 && cache_age <= max_age_ &&
      fresh(h[24], now.toSec()) && fresh(cached->odom_stamp, now.toSec()) ? 1. : 0.;
    const int subscribers = publisher_.getNumSubscribers();
    const bool health = h[4] == 1.;
    const bool full = full_requested_ || cached->generation != sent_generation_ || subscribers != last_subscribers_ ||
      (health && !last_health_) || now.toSec() - last_full_stamp_ >= 1.;
    h[20] = full ? 1. : 0.;
    std_msgs::Float64MultiArray message;
    auto& wire = message.data;
    wire.reserve(32 + cached->body.size() + 7 * std::size_t(h[13]) + (full ? cached->esdf->size() : 0));
    wire.assign(h.begin(), h.end());
    wire.insert(wire.end(), cached->body.begin(), cached->body.end());
    if (h[19] == 1.) for (const auto& p : dynamic.points)
      wire.insert(wire.end(), {p.position.x(), p.position.y(), p.position.z(),
        p.velocity.x(), p.velocity.y(), p.velocity.z(), double(p.track_id)});
    if (full) wire.insert(wire.end(), cached->esdf->begin(), cached->esdf->end());
    publisher_.publish(message);
    if (full) full_requested_ = false;
    ROS_DEBUG_NAMED("latency", "[LatencyTrace] stage=export_output stamp=%.9f wire=%lu field=%lu obs=%.0f fusion=%.9f full=%d valid=%d cached_valid=%d dynamic_valid=%d cached_joint_age_ms=%.3f cached_odom_age_ms=%.3f output_ms=%.3f bytes=%zu",
      ros::Time::now().toSec(), static_cast<unsigned long>(sequence_),
      static_cast<unsigned long>(cached->generation), h[29], h[3], int(full), int(h[4]),
      int(cached->header[4]), int(h[19]), (now.toSec()-h[24])*1000.,
      (now.toSec()-cached->odom_stamp)*1000., (ros::SteadyTime::now()-output_began).toSec()*1000.,
      wire.size()*sizeof(double));
    sent_generation_ = cached->generation; last_subscribers_ = subscribers; last_health_ = health;
    if (full) last_full_stamp_ = now.toSec();
  }
  void sampleAndOutput(const ros::TimerEvent &event) {
    sample(event);
    // Even a rejected sample must publish the cached packet's current health
    // and original source expiry; never turn an early return into silent loss
    // of the invalid/stale heartbeat.
    output(event);
  }
public:
  Exporter(ros::NodeHandle nh, const std::shared_ptr<GridMap>& map,
           const remani_planner::MMConfig::Ptr& model) : nh_(nh), map_(map), model_(model) {
    // Piper's model geometry is immutable after initialization. FastArmer's
    // gripper callback can change link points, so retain its existing path.
    const bool independent = map_->hasReadSnapshots() && !model_->getUseFastArmer();
    nh_.setCallbackQueue(independent ? &output_queue_ : map_->mappingCallbackQueue());
    q_.resize(model_->getManiDof());  // Never read until a complete real message arrives.
    nh_.param("mm/manipulator_thickness", arm_radius_, .034);
    nh_.param("optimization/safe_margin", car_margin_, .1);
    nh_.param("optimization/safe_margin_mani", arm_margin_, .1);
    nh_.param("optimization/weight_obstacle", weight_, -1.);
    nh_.param("optimization/ground_safe_dis", ground_, .03);
    nh_.param("dynamic_point_flow/enable", dynamic_enabled_, false);
    nh_.param("dynamic_point_flow/max_age", max_age_, .35);
    nh_.param("dynamic_point_flow/max_prediction_time", max_prediction_, 2.);
    nh_.param("dynamic_point_flow/position_uncertainty", pos_unc_, .03);
    nh_.param("dynamic_point_flow/velocity_uncertainty", vel_unc_, .15);
    nh_.param("dynamic_point_flow/acceleration_bound", acc_unc_, .5);
    nh_.param("mppi/state_max_age", state_max_age_, .25);
    nh_.param("mppi/crop_half_extent", crop_half_extent_, 2.);
    nh_.param("mppi/max_snapshot_cells", max_cells_, 500000);
    const std::array<double, 12> parameters{{arm_radius_, car_margin_, arm_margin_, weight_, ground_,
      max_age_, max_prediction_, pos_unc_, vel_unc_, acc_unc_, state_max_age_, crop_half_extent_}};
    if (!std::all_of(parameters.begin(), parameters.end(), [](double v) { return std::isfinite(v) && v >= 0.; }) ||
        arm_radius_ <= 0. || max_age_ <= 0. || max_prediction_ <= 0. || state_max_age_ <= 0. ||
        crop_half_extent_ <= 0. || max_cells_ < 8 || max_cells_ > 500000 || !(map_->getResolution() > 0.))
      throw std::runtime_error("invalid MPPI collision snapshot bounds/parameters");
    publisher_ = nh_.advertise<std_msgs::Float64MultiArray>("/mppi/collision_snapshot", 1, false);
    diagnostic_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(
        "/mppi/collision_diagnostic", 1, false);
    odom_sub_ = nh_.subscribe("odom_world", 1, &Exporter::odom, this);
    joint_sub_ = nh_.subscribe("joint_state", 1, &Exporter::joints, this);
    ros::NodeHandle resync_nh(nh_);
    resync_nh.setCallbackQueue(&output_queue_);
    resync_sub_ = resync_nh.subscribe("/mppi/collision_snapshot_resync", 1,
                                    &Exporter::resync, this);
    if (independent) {
      sample_timer_ = nh_.createTimer(ros::Duration(.05), &Exporter::sampleAndOutput, this);
      ROS_INFO("[MPPI snapshot] Independent Complete-snapshot sampling and output on one existing worker.");
    } else {
      sample_timer_ = nh_.createTimer(ros::Duration(.05), &Exporter::sample, this);
      ros::NodeHandle output_nh(nh_);
      output_nh.setCallbackQueue(&output_queue_);
      output_timer_ = output_nh.createTimer(ros::Duration(.05), &Exporter::output, this);
      ROS_INFO("[MPPI snapshot] Compatibility path: serialized-map sampling and independent output.");
    }
    spinner_.reset(new ros::AsyncSpinner(1, &output_queue_));
    spinner_->start();
    ROS_INFO("[MPPI snapshot] Existing native map, <=%d cells; native collision policy retained.", max_cells_);
  }
  ~Exporter() {
    output_timer_.stop(); sample_timer_.stop();
    odom_sub_.shutdown(); joint_sub_.shutdown(); resync_sub_.shutdown();
    if (spinner_) spinner_->stop();
    // The output worker has joined above. The compatibility sample may still
    // be in the map writer; join it before destroying shared callback fields.
    map_->stopMapping();
  }
};

inline void initialize(ros::NodeHandle& nh, const std::shared_ptr<GridMap>& map,
                       const remani_planner::MMConfig::Ptr& model) {
  bool enabled = false;
  nh.param("mppi/enable", enabled, false);
  ROS_INFO("[Execution mode] %s", enabled ?
      "REMANI + whole-body MPPI; MPPI owns base/arm command topics" :
      "Native REMANI bridges; MPPI is disabled");
  static std::shared_ptr<Exporter> exporter;
  if (enabled && !exporter) exporter = std::make_shared<Exporter>(nh, map, model);
}
}  // namespace mppi_collision_export

// Offline production-ESDF benchmark. No initMap(), subscriptions, timers,
// hardware drivers, SDK calls or motion publishers are created here.
// Geometry follows hardware_startup_20261108_183157/
// planner_parameters_at_attempt.json and GridMap::initMap's origin formula.
#include <plan_env/grid_map.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
constexpr int kNx = 300, kNy = 200, kNz = 60;
constexpr std::size_t kCount = std::size_t(kNx) * kNy * kNz;
constexpr double kResolution = 0.05, kCap = 0.6;
double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}
std::size_t address(int x, int y, int z) {
  return std::size_t(x) * kNy * kNz + std::size_t(y) * kNz + z;
}
struct Timing {
  double fixture_copy_ms = 0, prepare_ms = 0, submit_ms = 0;
  double snapshot_copy_ms = 0, compute_ms = 0, queue_wait_ms = 0;
  double commit_ms = 0, pipeline_wall_ms = 0;
  std::size_t compute_voxels = 0, write_voxels = 0;
  bool reused = false;
};
struct Row {
  std::string name;
  std::size_t changed = 0, occupied = 0, mismatches = 0;
  double max_error = 0;
  Timing full, local;
};
void writeTiming(std::ostream &out, const Timing &t) {
  out << "{\"fixture_copy_ms\":" << t.fixture_copy_ms
      << ",\"prepare_ms\":" << t.prepare_ms
      << ",\"submit_ms\":" << t.submit_ms
      << ",\"snapshot_copy_ms\":" << t.snapshot_copy_ms
      << ",\"compute_ms\":" << t.compute_ms
      << ",\"queue_wait_ms\":" << t.queue_wait_ms
      << ",\"commit_ms\":" << t.commit_ms
      << ",\"pipeline_wall_ms\":" << t.pipeline_wall_ms
      << ",\"compute_voxels\":" << t.compute_voxels
      << ",\"write_voxels\":" << t.write_voxels
      << ",\"reused\":" << (t.reused ? "true" : "false") << "}";
}
void addBox(std::vector<char> &occupancy, int x, int y, int z,
            int sx = 5, int sy = 5, int sz = 12) {
  for (int ix = x; ix < x + sx; ++ix)
    for (int iy = y; iy < y + sy; ++iy)
      for (int iz = z; iz < z + sz; ++iz)
        if (ix >= 0 && ix < kNx && iy >= 0 && iy < kNy && iz >= 0 && iz < kNz)
          occupancy[address(ix, iy, iz)] = 1;
}
}  // namespace

class GridMapParallelTestAccessor {
public:
  static double initialize(GridMap &map, const std::vector<char> &occupancy,
                           bool incremental) {
    // Value initialize scalar bookkeeping as initMap normally does, without
    // creating any of initMap's live I/O. Eigen fields actually used below
    // are assigned explicitly.
    map.mp_ = MappingParameters{};
    map.md_ = MappingData{};
    map.mp_.resolution_ = kResolution;
    map.mp_.resolution_inv_ = 1.0 / kResolution;
    map.mp_.map_voxel_num_ = Eigen::Vector3i(kNx, kNy, kNz);
    map.mp_.map_origin_ = Eigen::Vector3d(-7.5, -5.0, 0.0);
    map.mp_.map_size_ = Eigen::Vector3d(15.0, 10.0, 3.0);
    map.mp_.map_min_boundary_ = map.mp_.map_origin_;
    map.mp_.map_max_boundary_ = map.mp_.map_origin_ + map.mp_.map_size_;
    map.mp_.parallel_threads_ = 3;
    map.mp_.parallel_voxel_threshold_ = 50000;
    map.mp_.async_esdf_enable_ = true;
    map.mp_.async_esdf_max_result_age_ = 5.0;  // Offline timeout, not production TTL.
    map.mp_.incremental_esdf_ = incremental;
    map.mp_.esdf_distance_cap_ = kCap;
    map.mp_.enable_performance_diagnostics_ = false;
    map.mp_.show_esdf_time_ = false;
    map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
    map.md_.local_bound_max_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    map.md_.occupancy_bound_min_ = map.md_.local_bound_min_;
    map.md_.occupancy_bound_max_ = map.md_.local_bound_max_;
    map.md_.occupancy_buffer_inflate_ = occupancy;
    map.md_.occupancy_buffer_neg_.assign(kCount, 0);
    map.md_.tmp_buffer1_.assign(kCount, 0.0);
    map.md_.tmp_buffer2_.assign(kCount, 0.0);
    map.md_.distance_buffer_.assign(kCount, 10000.0);
    map.md_.distance_buffer_neg_.assign(kCount, 10000.0);
    map.md_.distance_buffer_all_.assign(kCount, 10000.0);
    map.md_.static_map_esdf_ready_ = true;
    map.esdf_map_epoch_ = 1;
    const auto start = Clock::now();
    map.prepareIncrementalESDF();
    map.updateESDF3d();
    map.finishIncrementalESDF();
    const double initialization_ms = milliseconds(start);

    // Future frames use ESDFWorker's own bounded scratch. Release only this
    // offline synchronous initialization scratch to keep peak RSS controlled.
    std::vector<double>().swap(map.md_.tmp_buffer1_);
    std::vector<double>().swap(map.md_.tmp_buffer2_);
    std::vector<double>().swap(map.md_.distance_buffer_);
    std::vector<double>().swap(map.md_.distance_buffer_neg_);
    std::vector<char>().swap(map.md_.occupancy_buffer_neg_);
    map.esdf_worker_.reset(new plan_env::ESDFWorker(3, 50000));
    return initialization_ms;
  }

  static Timing update(GridMap &map, const std::vector<char> &occupancy,
                       std::uint64_t sequence) {
    Timing timing;
    auto start = Clock::now();
    map.md_.occupancy_buffer_inflate_ = occupancy;
    timing.fixture_copy_ms = milliseconds(start);
    map.md_.dynamic_observation_seq_ = sequence;
    map.md_.lidar_accepted_seq_ = sequence;
    map.md_.lidar_fused_seq_ = sequence;
    map.md_.last_occ_update_time_ = ros::Time::now();
    map.md_.esdf_need_update_ = true;
    if (!map.mp_.incremental_esdf_) {
      map.md_.local_bound_min_ = Eigen::Vector3i::Zero();
      map.md_.local_bound_max_ = map.mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
    }
    const auto pipeline_start = Clock::now();
    start = Clock::now();
    const bool needs_computation = map.prepareIncrementalESDF();
    timing.prepare_ms = milliseconds(start);
    timing.reused = !needs_computation;
    if (needs_computation) {
      timing.compute_voxels = (map.md_.local_bound_max_ - map.md_.local_bound_min_ +
          Eigen::Vector3i::Ones()).cast<std::size_t>().prod();
      timing.write_voxels = map.mp_.incremental_esdf_
          ? (map.esdf_write_max_ - map.esdf_write_min_ + Eigen::Vector3i::Ones())
                .cast<std::size_t>().prod()
          : timing.compute_voxels;
      const double before_copy = map.md_.async_esdf_snapshot_wall_sum_ms_;
      const double before_compute = map.md_.esdf_wall_sum_ms_;
      const double before_commit = map.md_.esdf_commit_wall_sum_ms_;
      start = Clock::now();
      if (!map.submitAsyncESDFUpdate())
        throw std::runtime_error("Production async submission failed");
      timing.submit_ms = milliseconds(start);
      const auto request = map.esdf_latest_submitted_request_id_;
      const auto deadline = Clock::now() + std::chrono::seconds(10);
      while (map.esdf_committed_request_id_ != request) {
        map.commitAsyncESDFResult();
        if (map.md_.async_esdf_rejected_count_ != 0)
          throw std::runtime_error("Production async commit rejected a result");
        if (Clock::now() >= deadline)
          throw std::runtime_error("Production async commit timeout");
        if (map.esdf_committed_request_id_ != request)
          std::this_thread::sleep_for(std::chrono::microseconds(250));
      }
      timing.snapshot_copy_ms = map.md_.async_esdf_snapshot_wall_sum_ms_ - before_copy;
      timing.compute_ms = map.md_.esdf_wall_sum_ms_ - before_compute;
      timing.commit_ms = map.md_.esdf_commit_wall_sum_ms_ - before_commit;
      timing.queue_wait_ms = map.esdf_worker_->stats().last_queue_wait_ms;
    }
    timing.pipeline_wall_ms = milliseconds(pipeline_start);
    if (map.md_.esdf_observation_seq_ != sequence || map.md_.esdf_need_update_)
      throw std::runtime_error("Distance field did not acknowledge the fused occupancy generation");
    return timing;
  }

  static void compare(const GridMap &reference, const GridMap &local, Row &row) {
    const auto &a = reference.md_.distance_buffer_all_;
    const auto &b = local.md_.distance_buffer_all_;
    if (a.size() != kCount || b.size() != kCount)
      throw std::runtime_error("Unexpected field size");
    for (std::size_t i = 0; i < kCount; ++i) {
      const double expected = std::max(-kCap, std::min(kCap, a[i]));
      const double error = std::abs(b[i] - expected);
      if (!std::isfinite(a[i]) || !std::isfinite(b[i]) || error > 1e-10)
        ++row.mismatches;
      row.max_error = std::max(row.max_error, error);
    }
  }
};

int main(int argc, char **argv) {
  try {
    const char *master = std::getenv("ROS_MASTER_URI");
    if (!master || (std::string(master).find("http://127.0.0.1:") != 0 &&
                    std::string(master).find("http://localhost:") != 0))
      throw std::runtime_error("Use an isolated localhost ROS master for this offline benchmark");
    ros::init(argc, argv, "offline_local_esdf_benchmark", ros::init_options::NoRosout);
    const std::string pcd_path = argc > 1 ? argv[1] : "src/mm_remani/maps/exp0_planner.pcd";
    const std::string output = argc > 2 ? argv[2] :
        "experiments/online_remani_validation/results/local_esdf_20261108/benchmark";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile(pcd_path, cloud) < 0)
      throw std::runtime_error("Cannot load static PCD");
    std::vector<char> background(kCount, 0);
    std::size_t accepted = 0;
    for (const auto &p : cloud) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
      if (p.z < 0.15) continue;  // Native static-map ground filter.
      // Match native isInMap's small boundary tolerance before posToIndex.
      if (p.x < -7.5 + 1e-4 || p.y < -5.0 + 1e-4 || p.z < 1e-4 ||
          p.x > 7.5 - 1e-4 || p.y > 5.0 - 1e-4 || p.z > 3.0 - 1e-4) continue;
      const int x = std::floor((p.x + 7.5) * (1.0 / kResolution));
      const int y = std::floor((p.y + 5.0) * (1.0 / kResolution));
      const int z = std::floor(double(p.z) * (1.0 / kResolution));
      background[address(x, y, z)] = 1;
      ++accepted;
    }
    const auto static_voxels = std::count(background.begin(), background.end(), char(1));
    GridMap full, local;
    const double full_initial_ms = GridMapParallelTestAccessor::initialize(full, background, false);
    const double local_initial_ms = GridMapParallelTestAccessor::initialize(local, background, true);
    Row initial;
    GridMapParallelTestAccessor::compare(full, local, initial);
    std::vector<Row> rows;
    auto previous = background;
    bool all_equal = initial.mismatches == 0;
    for (int frame = 0; frame < 14; ++frame) {
      Row row;
      auto occupancy = background;
      if (frame < 9) {
        row.name = "nearby_motion_" + std::to_string(frame);
        addBox(occupancy, 160 + frame, 108, 12);  // z=0.60..1.20 m, arm-height box.
      } else if (frame == 9) {
        row.name = "nearby_removal";
      } else if (frame == 10) {
        row.name = "nearby_reappearance";
        addBox(occupancy, 165, 116, 16);
      } else if (frame == 11) {
        row.name = "nearby_second_removal";
      } else if (frame == 12) {
        row.name = "widely_separated_changes";
        addBox(occupancy, 2, 2, 2, 2, 2, 2);
        addBox(occupancy, 296, 196, 56, 2, 2, 2);
      } else {
        row.name = "unchanged_occupancy";
        occupancy = previous;
      }
      for (std::size_t i = 0; i < kCount; ++i) {
        row.changed += occupancy[i] != previous[i];
        row.occupied += occupancy[i] != 0;
      }
      // Alternate serial execution order; never run two 3-thread workers at once.
      if (frame % 2 == 0) {
        row.full = GridMapParallelTestAccessor::update(full, occupancy, frame + 1);
        row.local = GridMapParallelTestAccessor::update(local, occupancy, frame + 1);
      } else {
        row.local = GridMapParallelTestAccessor::update(local, occupancy, frame + 1);
        row.full = GridMapParallelTestAccessor::update(full, occupancy, frame + 1);
      }
      GridMapParallelTestAccessor::compare(full, local, row);
      all_equal = all_equal && row.mismatches == 0;
      std::cout << row.name << ": full=" << row.full.pipeline_wall_ms
                << "ms local=" << row.local.pipeline_wall_ms
                << "ms computed=" << row.local.compute_voxels
                << " mismatches=" << row.mismatches << std::endl;
      previous.swap(occupancy);
      rows.push_back(row);
    }
    std::ofstream json(output + ".json");
    if (!json) throw std::runtime_error("Cannot open JSON output");
    json << std::setprecision(12)
         << "{\n\"scope\":\"offline EDT pipeline, not live sensor-to-controller latency\",\n"
         << "\"timing_exclusions\":\"PCD load, occupancy fusion, fixture copy, field comparison, immutable publication, ROS transport, MPPI export\",\n"
         << "\"timing_notes\":\"pipeline includes prepare, async input snapshot, worker queue/compute, polling and production commit; initial synchronous work listed separately\",\n"
         << "\"map_origin\":[-7.5,-5,0],\"map_voxels\":[300,200,60],\"resolution_m\":0.05,"
         << "\"distance_cap_m\":0.6,\"worker_threads\":3,\"parallel_voxel_threshold\":50000,\n"
         << "\"pcd_points\":" << cloud.size() << ",\"accepted_pcd_points\":" << accepted
         << ",\"static_occupied_voxels\":" << static_voxels << ",\n"
         << "\"full_initial_ms\":" << full_initial_ms << ",\"local_initial_ms\":" << local_initial_ms
         << ",\"initial_mismatches\":" << initial.mismatches
         << ",\"initial_max_abs_error_m\":" << initial.max_error << ",\n"
         << "\"all_equal\":" << (all_equal ? "true" : "false") << ",\"frames\":[\n";
    std::ofstream csv(output + ".csv");
    if (!csv) throw std::runtime_error("Cannot open CSV output");
    csv << "frame,changed_voxels,occupied_voxels,full_wall_ms,local_wall_ms,local_prepare_ms,local_input_copy_ms,local_compute_ms,local_commit_ms,local_compute_voxels,local_write_voxels,max_abs_error_m,mismatches\n";
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const Row &r = rows[i];
      json << "{\"name\":\"" << r.name << "\",\"changed_voxels\":" << r.changed
           << ",\"occupied_voxels\":" << r.occupied << ",\"compared_voxels\":" << kCount
           << ",\"mismatches\":" << r.mismatches << ",\"max_abs_error_m\":" << r.max_error
           << ",\"full\":";
      writeTiming(json, r.full);
      json << ",\"local\":";
      writeTiming(json, r.local);
      json << "}" << (i + 1 == rows.size() ? "\n" : ",\n");
      csv << std::setprecision(12) << r.name << ',' << r.changed << ',' << r.occupied
          << ',' << r.full.pipeline_wall_ms << ',' << r.local.pipeline_wall_ms
          << ',' << r.local.prepare_ms << ',' << r.local.snapshot_copy_ms
          << ',' << r.local.compute_ms << ',' << r.local.commit_ms
          << ',' << r.local.compute_voxels << ',' << r.local.write_voxels
          << ',' << r.max_error << ',' << r.mismatches << '\n';
    }
    json << "]}\n";
    return all_equal ? 0 : 2;
  } catch (const std::exception &error) {
    std::cerr << "Offline benchmark failed: " << error.what() << '\n';
    return 1;
  }
}

#include <plan_env/esdf_worker.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <vector>

#ifndef BENCH_VARIANT
#define BENCH_VARIANT "unspecified"
#endif

struct Frame {
  std::string name, group, file;
  Eigen::Vector3i size, minimum;
  double resolution;
};

int main(int argc, char **argv) {
  try {
    if (argc != 5) throw std::runtime_error("Usage: benchmark FIXTURES REPEATS THREADS OUTPUT");
    const std::string fixtures(argv[1]), output(argv[4]);
    const int repeats = std::stoi(argv[2]), threads = std::stoi(argv[3]);
    if (repeats < 1 || repeats > 10 || threads < 1 || threads > 3)
      throw std::runtime_error("Require repeats 1..10 and threads 1..3");
    std::ifstream table(fixtures + "/frames.tsv");
    if (!table) throw std::runtime_error("Cannot open fixture manifest");
    std::vector<Frame> frames;
    Frame f;
    while (table >> f.name >> f.group >> f.file >> f.size.x() >> f.size.y() >> f.size.z()
                 >> f.minimum.x() >> f.minimum.y() >> f.minimum.z() >> f.resolution) {
      if ((f.size.array() <= 0).any() || f.resolution != .05)
        throw std::runtime_error("Invalid fixture geometry");
      frames.push_back(f);
    }
    if (!table.eof() || frames.empty()) throw std::runtime_error("Malformed/empty fixture manifest");
    plan_env::ESDFWorker worker(threads);
    std::uint64_t request = 0;
    std::cout << std::setprecision(17);
    for (int repeat = 0; repeat < repeats; ++repeat) {
      for (const Frame &frame : frames) {
        const std::size_t count = frame.size.cast<std::size_t>().prod();
        plan_env::ESDFJob job;
        job.request_id = ++request;
        job.map_epoch = request;  // Independent immutable input; never reuse occupancy.
        job.observation_seq = request;
        job.min_index = frame.minimum;
        job.max_index = frame.minimum + frame.size - Eigen::Vector3i::Ones();
        job.resolution = frame.resolution;
        job.distance_cap = 0.0;
        job.trace_timing = true;
        job.occupancy.resize(count);
        std::ifstream input(fixtures + "/" + frame.file, std::ios::binary | std::ios::ate);
        if (!input || input.tellg() != std::streampos(count))
          throw std::runtime_error("Occupancy input size mismatch: " + frame.name);
        input.seekg(0);
        input.read(job.occupancy.data(), static_cast<std::streamsize>(count));
        if (!input) throw std::runtime_error("Cannot read input: " + frame.name);
        for (const char value : job.occupancy)
          if (value != 0 && value != 1) throw std::runtime_error("Nonbinary occupancy");
        job.captured_steady = ros::SteadyTime::now();
        const auto start = std::chrono::steady_clock::now();
        if (!worker.submit(std::move(job))) throw std::runtime_error("Worker rejected input");
        plan_env::ESDFResult result;
        if (!worker.waitForResult(&result, std::chrono::milliseconds(10000)))
          throw std::runtime_error("Worker timeout/fault: " + worker.stats().last_fault);
        const double wall = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        if (result.request_id != request || result.map_epoch != request ||
            result.observation_seq != request || result.min_index != frame.minimum ||
            result.max_index != frame.minimum + frame.size - Eigen::Vector3i::Ones() ||
            !result.patches.empty() || result.distance_all.size() != count ||
            result.computed_voxels != count)
          throw std::runtime_error("Result identity/geometry/dense coverage mismatch");
        for (const double value : result.distance_all)
          if (!std::isfinite(value)) throw std::runtime_error("Nonfinite native signed distance");
        if (repeat == 0) {
          std::ofstream field(output + "/" + frame.name + ".distance.f64", std::ios::binary);
          if (!field) throw std::runtime_error("Cannot create raw output");
          field.write(reinterpret_cast<const char*>(result.distance_all.data()),
                      static_cast<std::streamsize>(count * sizeof(double)));
          if (!field) throw std::runtime_error("Raw output write failed");
        }
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        std::cout << "{\"variant\":\"" << BENCH_VARIANT << "\",\"frame\":\"" << frame.name
          << "\",\"group\":\"" << frame.group << "\",\"repeat\":" << repeat
          << ",\"threads\":" << threads << ",\"voxels\":" << count
          << ",\"compute_ms\":" << result.compute_ms << ",\"submit_wait_ms\":" << wall
          << ",\"queue_wait_ms\":" << result.queue_wait_ms
          << ",\"prepare_ms\":" << result.prepare_ms << ",\"partition_ms\":" << result.partition_ms
          << ",\"positive_zyx_ms\":[" << result.positive_ms[0] << ',' << result.positive_ms[1] << ',' << result.positive_ms[2]
          << "],\"near_free_ms\":" << result.near_free_ms
          << ",\"negative_zyx_ms\":[" << result.negative_ms[0] << ',' << result.negative_ms[1] << ',' << result.negative_ms[2]
          << "],\"negative_apply_ms\":" << result.negative_apply_ms
          << ",\"negative_fallback_count\":" << result.negative_fallback_count
#ifdef BENCH_NEGATIVE_SUBSET_METRICS
          << ",\"negative_subset_count\":" << result.negative_subset_count
          << ",\"negative_subset_cells\":" << result.negative_subset_cells
          << ",\"negative_subset_z_layers\":" << result.negative_subset_z_layers
          << ",\"negative_subset_yz_lines\":" << result.negative_subset_yz_lines
#endif
          << ",\"peak_rss_kib\":" << usage.ru_maxrss << "}" << std::endl;
      }
    }
    worker.stop();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "dense_worker_benchmark: " << error.what() << std::endl;
    return 1;
  }
}

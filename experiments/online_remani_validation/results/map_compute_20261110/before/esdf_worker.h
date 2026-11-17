#ifndef PLAN_ENV_ESDF_WORKER_H_
#define PLAN_ENV_ESDF_WORKER_H_

#include <Eigen/Core>

#include <ros/time.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace plan_env
{

/**
 * Immutable input for one local ESDF calculation.
 *
 * occupancy is a compact x-major array over [min_index, max_index], with z
 * as the contiguous dimension.  A value of one is occupied; zero is free.
 * The worker owns its queued copy, so it never reads GridMap storage.
 */
struct ESDFJob
{
  std::uint64_t request_id = 0;
  std::uint64_t map_epoch = 0;
  std::uint64_t observation_seq = 0;
  Eigen::Vector3i min_index = Eigen::Vector3i::Zero();
  Eigen::Vector3i max_index = Eigen::Vector3i::Constant(-1);
  double resolution = 0.0;
  ros::SteadyTime captured_steady;
  std::vector<char> occupancy;
};

/** Result corresponding to exactly one ESDFJob snapshot. */
struct ESDFResult
{
  std::uint64_t request_id = 0;
  std::uint64_t map_epoch = 0;
  std::uint64_t observation_seq = 0;
  Eigen::Vector3i min_index = Eigen::Vector3i::Zero();
  Eigen::Vector3i max_index = Eigen::Vector3i::Constant(-1);
  ros::SteadyTime captured_steady;
  std::vector<double> distance_all;
  double queue_wait_ms = 0.0;
  double compute_ms = 0.0;
};

struct ESDFWorkerStats
{
  std::uint64_t submitted = 0;
  std::uint64_t coalesced = 0;
  std::uint64_t superseded = 0;
  std::uint64_t completed = 0;
  std::uint64_t result_replaced = 0;
  std::uint64_t invalidated = 0;
  std::uint64_t faults = 0;
  bool running = false;
  bool pending = false;
  bool stopped = false;
  double last_queue_wait_ms = 0.0;
  double last_compute_ms = 0.0;
  std::string last_fault;
};

/**
 * Bounded asynchronous signed-ESDF executor.
 *
 * The coordinator thread is one of parallel_threads.  The remaining threads
 * form a persistent helper pool, so an ESDF update does not repeatedly create
 * and destroy threads.  At most one job is running and one job is pending;
 * submitting while a job is pending replaces that pending snapshot with the
 * newest one.  Request ids must be positive and strictly increasing: a higher
 * request id also cancels an older running calculation at bounded checkpoints
 * and drops an older result that has not yet been consumed.
 */
class ESDFWorker
{
public:
  explicit ESDFWorker(int parallel_threads,
                      int parallel_voxel_threshold = 1);
  ~ESDFWorker();

  ESDFWorker(const ESDFWorker &) = delete;
  ESDFWorker &operator=(const ESDFWorker &) = delete;

  // Passing an lvalue copies the full occupancy snapshot; passing an rvalue
  // transfers ownership.  In both cases the queued job is worker-owned.
  // Returns false for request_id zero, duplicate ids, and out-of-order ids.
  bool submit(ESDFJob job);

  // Moves the newest completed result to result without blocking.
  bool tryTakeResult(ESDFResult &result);

  // Test/diagnostic helper.  Runtime users should normally use tryTakeResult().
  bool waitForResult(ESDFResult *result,
                     const std::chrono::milliseconds &timeout);

  ESDFWorkerStats stats() const;

  // Invalidates queued/ready jobs from older map epochs.  A currently running
  // older calculation observes this generation change and cancels at a bounded
  // interval inside each distance-transform pass.
  void invalidateBeforeEpoch(std::uint64_t minimum_epoch);

  // Idempotent.  Requests cancellation, joins the coordinator, then joins all
  // helper threads.  No thread owned by this object is detached.
  void stop();

private:
  struct QueuedJob
  {
    ESDFJob job;
    ros::SteadyTime enqueued_steady;
  };

  typedef std::function<void(std::size_t, std::size_t)> RangeTask;

  static bool validateJob(const ESDFJob &job, std::string *reason);
  bool compute(const ESDFJob &job, ESDFResult *result);
  bool parallelFor(std::size_t item_count,
                   int parallel_threshold,
                   const std::function<void(std::size_t)> &function);
  void coordinatorLoop();
  void helperLoop(std::size_t helper_index);
  void recordFault(const std::string &message);
  bool computationCancelled() const;

  const int parallel_threads_;
  const int parallel_voxel_threshold_;

  std::atomic<bool> stop_requested_;
  std::atomic<std::uint64_t> minimum_epoch_;
  std::atomic<std::uint64_t> active_job_epoch_;
  std::atomic<std::uint64_t> newest_request_id_;
  std::atomic<std::uint64_t> active_job_request_id_;
  std::mutex stop_mutex_;
  std::thread coordinator_thread_;
  std::vector<std::thread> helper_threads_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable result_cv_;
  std::unique_ptr<QueuedJob> pending_job_;
  std::unique_ptr<ESDFResult> ready_result_;
  ESDFWorkerStats stats_;

  // The following state is used only to dispatch a single coordinator-owned
  // pass to the fixed helper pool.
  std::mutex parallel_mutex_;
  std::condition_variable parallel_cv_;
  std::condition_variable parallel_done_cv_;
  std::uint64_t parallel_generation_;
  std::size_t parallel_item_count_;
  std::size_t parallel_active_helpers_;
  std::size_t parallel_helpers_done_;
  bool helpers_stopping_;
  bool parallel_failed_;
  std::string parallel_error_;
  std::shared_ptr<const RangeTask> parallel_task_;

  // Coordinator-only scratch buffers, retained between jobs to avoid repeated
  // allocation of the two large EDT intermediate grids.
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
};

}  // namespace plan_env

#endif  // PLAN_ENV_ESDF_WORKER_H_

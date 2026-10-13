#include "plan_env/esdf_worker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{

std::string currentExceptionMessage()
{
  try
  {
    throw;
  }
  catch (const std::exception &exception)
  {
    return exception.what();
  }
  catch (...)
  {
    return "unknown exception";
  }
}

bool compactVoxelCount(const Eigen::Vector3i &min_index,
                       const Eigen::Vector3i &max_index,
                       Eigen::Vector3i *size,
                       std::size_t *voxel_count)
{
  Eigen::Vector3i checked_size;
  for (int dimension = 0; dimension < 3; ++dimension)
  {
    const std::int64_t length =
        static_cast<std::int64_t>(max_index(dimension)) -
        static_cast<std::int64_t>(min_index(dimension)) + 1;
    if (length <= 0 ||
        length > static_cast<std::int64_t>(std::numeric_limits<int>::max()))
      return false;
    checked_size(dimension) = static_cast<int>(length);
  }

  std::size_t count = 1;
  for (int dimension = 0; dimension < 3; ++dimension)
  {
    const std::size_t length =
        static_cast<std::size_t>(checked_size(dimension));
    if (count > std::numeric_limits<std::size_t>::max() / length)
      return false;
    count *= length;
  }

  *size = checked_size;
  *voxel_count = count;
  return true;
}

template <typename GetValue, typename SetValue>
void fillDistanceTransform(int length,
                           int coordinate_offset,
                           const GetValue &get_value,
                           const SetValue &set_value)
{
  // These buffers are private to each persistent worker thread and are reused
  // for every grid line handled by that thread.
  thread_local std::vector<int> sites;
  thread_local std::vector<double> boundaries;
  sites.resize(static_cast<std::size_t>(length));
  boundaries.resize(static_cast<std::size_t>(length) + 1U);

  int k = 0;
  sites[0] = 0;
  boundaries[0] = -std::numeric_limits<double>::max();
  boundaries[1] = std::numeric_limits<double>::max();

  for (int local_q = 1; local_q < length; ++local_q)
  {
    ++k;
    double intersection = 0.0;
    const int q = coordinate_offset + local_q;

    do
    {
      --k;
      const int site = coordinate_offset + sites[static_cast<std::size_t>(k)];
      intersection =
          ((get_value(local_q) + q * q) -
           (get_value(sites[static_cast<std::size_t>(k)]) + site * site)) /
          (2 * q - 2 * site);
    } while (intersection <= boundaries[static_cast<std::size_t>(k)]);

    ++k;
    sites[static_cast<std::size_t>(k)] = local_q;
    boundaries[static_cast<std::size_t>(k)] = intersection;
    boundaries[static_cast<std::size_t>(k + 1)] =
        std::numeric_limits<double>::max();
  }

  k = 0;
  for (int local_q = 0; local_q < length; ++local_q)
  {
    const int q = coordinate_offset + local_q;
    while (boundaries[static_cast<std::size_t>(k + 1)] < q)
      ++k;
    const int delta =
        local_q - sites[static_cast<std::size_t>(k)];
    const double value =
        delta * delta + get_value(sites[static_cast<std::size_t>(k)]);
    set_value(local_q, value);
  }
}

}  // namespace

namespace plan_env
{

ESDFWorker::ESDFWorker(int parallel_threads,
                       int parallel_voxel_threshold)
    : parallel_threads_(std::max(1, parallel_threads)),
      parallel_voxel_threshold_(std::max(1, parallel_voxel_threshold)),
      stop_requested_(false),
      minimum_epoch_(0),
      active_job_epoch_(std::numeric_limits<std::uint64_t>::max()),
      newest_request_id_(0),
      active_job_request_id_(std::numeric_limits<std::uint64_t>::max()),
      parallel_generation_(0),
      parallel_item_count_(0),
      parallel_active_helpers_(0),
      parallel_helpers_done_(0),
      helpers_stopping_(false),
      parallel_failed_(false)
{
  try
  {
    helper_threads_.reserve(static_cast<std::size_t>(parallel_threads_ - 1));
    for (int helper = 0; helper < parallel_threads_ - 1; ++helper)
    {
      helper_threads_.emplace_back(
          &ESDFWorker::helperLoop, this, static_cast<std::size_t>(helper));
    }
    coordinator_thread_ = std::thread(&ESDFWorker::coordinatorLoop, this);
  }
  catch (...)
  {
    stop_requested_.store(true, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(parallel_mutex_);
      helpers_stopping_ = true;
      ++parallel_generation_;
    }
    parallel_cv_.notify_all();
    for (std::thread &helper : helper_threads_)
    {
      if (helper.joinable())
        helper.join();
    }
    throw;
  }
}

ESDFWorker::~ESDFWorker()
{
  stop();
}

bool ESDFWorker::validateJob(const ESDFJob &job, std::string *reason)
{
  if (!std::isfinite(job.resolution) || job.resolution <= 0.0)
  {
    *reason = "ESDF resolution must be finite and positive";
    return false;
  }

  Eigen::Vector3i size;
  std::size_t voxel_count = 0;
  if (!compactVoxelCount(job.min_index, job.max_index, &size, &voxel_count))
  {
    *reason = "invalid ESDF bounds or voxel-count overflow";
    return false;
  }
  if (job.occupancy.size() != voxel_count)
  {
    std::ostringstream stream;
    stream << "occupancy snapshot has " << job.occupancy.size()
           << " voxels, expected " << voxel_count;
    *reason = stream.str();
    return false;
  }
  return true;
}

bool ESDFWorker::submit(ESDFJob job)
{
  std::string reason;
  if (!validateJob(job, &reason))
  {
    recordFault(reason);
    return false;
  }

  if (stop_requested_.load(std::memory_order_acquire))
    return false;

  const std::uint64_t minimum_epoch =
      minimum_epoch_.load(std::memory_order_acquire);
  if (job.map_epoch < minimum_epoch)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    ++stats_.invalidated;
    return false;
  }

  std::unique_ptr<QueuedJob> queued(new QueuedJob());
  queued->job = std::move(job);
  queued->enqueued_steady = ros::SteadyTime::now();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stop_requested_.load(std::memory_order_acquire))
      return false;
    if (queued->job.map_epoch <
        minimum_epoch_.load(std::memory_order_acquire))
    {
      ++stats_.invalidated;
      return false;
    }

    const std::uint64_t newest_request_id =
        newest_request_id_.load(std::memory_order_acquire);
    if (queued->job.request_id <= newest_request_id)
    {
      ++stats_.superseded;
      return false;
    }

    ++stats_.submitted;
    if (pending_job_)
    {
      ++stats_.coalesced;
      ++stats_.superseded;
    }
    pending_job_ = std::move(queued);
    stats_.pending = true;

    // Publish the new watermark only after its immutable snapshot is in the
    // pending slot.  The coordinator can then cancel its older active job
    // without ever observing a watermark for which no replacement is queued.
    if (pending_job_->job.request_id > newest_request_id)
    {
      newest_request_id_.store(pending_job_->job.request_id,
                               std::memory_order_release);
      if (ready_result_ &&
          ready_result_->request_id < pending_job_->job.request_id)
      {
        ready_result_.reset();
        ++stats_.superseded;
      }
    }
  }
  queue_cv_.notify_one();
  return true;
}

bool ESDFWorker::tryTakeResult(ESDFResult &result)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (!ready_result_)
    return false;
  result = std::move(*ready_result_);
  ready_result_.reset();
  return true;
}

bool ESDFWorker::waitForResult(
    ESDFResult *result, const std::chrono::milliseconds &timeout)
{
  if (result == nullptr)
    return false;

  std::unique_lock<std::mutex> lock(queue_mutex_);
  result_cv_.wait_for(lock, timeout, [this]() {
    return ready_result_ ||
           stop_requested_.load(std::memory_order_acquire);
  });
  if (!ready_result_)
    return false;
  *result = std::move(*ready_result_);
  ready_result_.reset();
  return true;
}

ESDFWorkerStats ESDFWorker::stats() const
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return stats_;
}

void ESDFWorker::invalidateBeforeEpoch(std::uint64_t minimum_epoch)
{
  std::uint64_t observed = minimum_epoch_.load(std::memory_order_acquire);
  while (observed < minimum_epoch &&
         !minimum_epoch_.compare_exchange_weak(
             observed, minimum_epoch,
             std::memory_order_acq_rel, std::memory_order_acquire))
  {
  }

  if (observed >= minimum_epoch)
    return;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (pending_job_ && pending_job_->job.map_epoch < minimum_epoch)
    {
      pending_job_.reset();
      stats_.pending = false;
      ++stats_.invalidated;
    }
    if (ready_result_ && ready_result_->map_epoch < minimum_epoch)
    {
      ready_result_.reset();
      ++stats_.invalidated;
    }
  }
  queue_cv_.notify_all();
  result_cv_.notify_all();
}

void ESDFWorker::stop()
{
  std::lock_guard<std::mutex> lifecycle_lock(stop_mutex_);
  stop_requested_.store(true, std::memory_order_release);
  queue_cv_.notify_all();
  result_cv_.notify_all();

  if (coordinator_thread_.joinable())
  {
    if (coordinator_thread_.get_id() == std::this_thread::get_id())
      throw std::logic_error("ESDFWorker::stop called by its coordinator");
    coordinator_thread_.join();
  }

  {
    std::lock_guard<std::mutex> lock(parallel_mutex_);
    helpers_stopping_ = true;
    ++parallel_generation_;
  }
  parallel_cv_.notify_all();
  for (std::thread &helper : helper_threads_)
  {
    if (helper.joinable())
      helper.join();
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_job_.reset();
    stats_.pending = false;
    stats_.running = false;
    stats_.stopped = true;
  }
}

void ESDFWorker::recordFault(const std::string &message)
{
  std::lock_guard<std::mutex> lock(queue_mutex_);
  ++stats_.faults;
  stats_.last_fault = message;
}

bool ESDFWorker::computationCancelled() const
{
  return stop_requested_.load(std::memory_order_acquire) ||
         active_job_epoch_.load(std::memory_order_acquire) <
             minimum_epoch_.load(std::memory_order_acquire) ||
         active_job_request_id_.load(std::memory_order_acquire) <
             newest_request_id_.load(std::memory_order_acquire);
}

void ESDFWorker::coordinatorLoop()
{
  while (true)
  {
    std::unique_ptr<QueuedJob> queued;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return pending_job_ ||
               stop_requested_.load(std::memory_order_acquire);
      });
      if (stop_requested_.load(std::memory_order_acquire))
        break;

      queued = std::move(pending_job_);
      stats_.pending = false;
      stats_.running = true;
    }

    active_job_epoch_.store(queued->job.map_epoch,
                            std::memory_order_release);
    active_job_request_id_.store(queued->job.request_id,
                                 std::memory_order_release);
    ESDFResult result;
    const ros::SteadyTime compute_start = ros::SteadyTime::now();
    result.queue_wait_ms =
        std::max(0.0, (compute_start - queued->enqueued_steady).toSec() * 1e3);

    bool completed = false;
    try
    {
      completed = compute(queued->job, &result);
    }
    catch (...)
    {
      recordFault(std::string("ESDF computation failed: ") +
                  currentExceptionMessage());
    }
    const ros::SteadyTime compute_end = ros::SteadyTime::now();
    result.compute_ms =
        std::max(0.0, (compute_end - compute_start).toSec() * 1e3);
    active_job_epoch_.store(std::numeric_limits<std::uint64_t>::max(),
                            std::memory_order_release);
    active_job_request_id_.store(
        std::numeric_limits<std::uint64_t>::max(),
        std::memory_order_release);

    const bool epoch_valid =
        queued->job.map_epoch >=
        minimum_epoch_.load(std::memory_order_acquire);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stats_.running = false;
      const bool request_current =
          queued->job.request_id ==
          newest_request_id_.load(std::memory_order_acquire);
      if (completed && epoch_valid && request_current &&
          !stop_requested_.load(std::memory_order_acquire))
      {
        if (ready_result_)
          ++stats_.result_replaced;
        ready_result_.reset(new ESDFResult(std::move(result)));
        ++stats_.completed;
        stats_.last_queue_wait_ms = ready_result_->queue_wait_ms;
        stats_.last_compute_ms = ready_result_->compute_ms;
      }
      else if (!epoch_valid)
      {
        ++stats_.invalidated;
      }
      else if (!request_current)
      {
        ++stats_.superseded;
      }
    }
    result_cv_.notify_all();
  }
}

void ESDFWorker::helperLoop(std::size_t helper_index)
{
  std::uint64_t observed_generation = 0;
  while (true)
  {
    std::shared_ptr<const RangeTask> task;
    std::size_t begin = 0;
    std::size_t end = 0;
    bool participates = false;
    {
      std::unique_lock<std::mutex> lock(parallel_mutex_);
      parallel_cv_.wait(lock, [this, observed_generation]() {
        return helpers_stopping_ ||
               parallel_generation_ != observed_generation;
      });
      if (helpers_stopping_)
        return;
      observed_generation = parallel_generation_;

      participates = helper_index < parallel_active_helpers_;
      if (participates)
      {
        const std::size_t worker_count = parallel_active_helpers_ + 1U;
        const std::size_t block_size =
            (parallel_item_count_ + worker_count - 1U) / worker_count;
        begin = (helper_index + 1U) * block_size;
        end = std::min(parallel_item_count_, begin + block_size);
        task = parallel_task_;
      }
    }

    std::string error;
    if (participates && begin < end)
    {
      try
      {
        (*task)(begin, end);
      }
      catch (...)
      {
        error = currentExceptionMessage();
      }
    }

    if (participates)
    {
      std::lock_guard<std::mutex> lock(parallel_mutex_);
      if (!error.empty() && !parallel_failed_)
      {
        parallel_failed_ = true;
        parallel_error_ = error;
      }
      ++parallel_helpers_done_;
      if (parallel_helpers_done_ >= parallel_active_helpers_)
        parallel_done_cv_.notify_one();
    }
  }
}

bool ESDFWorker::parallelFor(
    std::size_t item_count,
    int parallel_threshold,
    const std::function<void(std::size_t)> &function)
{
  if (item_count == 0)
    return !computationCancelled();

  int worker_count = std::min(
      parallel_threads_, static_cast<int>(item_count));
  if (worker_count <= 1 ||
      item_count < static_cast<std::size_t>(std::max(1, parallel_threshold)))
    worker_count = 1;

  const std::shared_ptr<const RangeTask> task(
      new RangeTask([this, &function](std::size_t begin, std::size_t end) {
        for (std::size_t index = begin; index < end; ++index)
        {
          if (((index - begin) & 63U) == 0U && computationCancelled())
            return;
          function(index);
        }
      }));

  if (worker_count == 1)
  {
    (*task)(0, item_count);
    return !computationCancelled();
  }

  const std::size_t active_helpers =
      static_cast<std::size_t>(worker_count - 1);
  const std::size_t block_size =
      (item_count + static_cast<std::size_t>(worker_count) - 1U) /
      static_cast<std::size_t>(worker_count);
  {
    std::lock_guard<std::mutex> lock(parallel_mutex_);
    parallel_item_count_ = item_count;
    parallel_active_helpers_ = active_helpers;
    parallel_helpers_done_ = 0;
    parallel_failed_ = false;
    parallel_error_.clear();
    parallel_task_ = task;
    ++parallel_generation_;
  }
  parallel_cv_.notify_all();

  std::string caller_error;
  try
  {
    (*task)(0, std::min(item_count, block_size));
  }
  catch (...)
  {
    caller_error = currentExceptionMessage();
  }

  std::string parallel_error;
  {
    std::unique_lock<std::mutex> lock(parallel_mutex_);
    if (!caller_error.empty() && !parallel_failed_)
    {
      parallel_failed_ = true;
      parallel_error_ = caller_error;
    }
    parallel_done_cv_.wait(lock, [this, active_helpers]() {
      return parallel_helpers_done_ >= active_helpers;
    });
    if (parallel_failed_)
      parallel_error = parallel_error_;
    parallel_task_.reset();
  }

  if (!parallel_error.empty())
    throw std::runtime_error(parallel_error);
  return !computationCancelled();
}

bool ESDFWorker::compute(const ESDFJob &job, ESDFResult *result)
{
  Eigen::Vector3i size;
  std::size_t voxel_count = 0;
  if (!compactVoxelCount(
          job.min_index, job.max_index, &size, &voxel_count))
    throw std::invalid_argument("invalid compact ESDF bounds");

  result->request_id = job.request_id;
  result->map_epoch = job.map_epoch;
  result->observation_seq = job.observation_seq;
  result->min_index = job.min_index;
  result->max_index = job.max_index;
  result->captured_steady = job.captured_steady;

  // A replacement can be accepted between dequeue and compute().  Check
  // before touching the large distance buffers so a superseded request does
  // not impose avoidable allocation/fill latency on the newest snapshot.
  if (computationCancelled())
    return false;

  result->distance_all.assign(voxel_count, 10000.0);
  tmp_buffer1_.resize(voxel_count);
  tmp_buffer2_.resize(voxel_count);

  if (computationCancelled())
    return false;

  const int nx = size.x();
  const int ny = size.y();
  const int nz = size.z();
  const auto address = [ny, nz](int x, int y, int z) {
    return (static_cast<std::size_t>(x) * static_cast<std::size_t>(ny) +
            static_cast<std::size_t>(y)) * static_cast<std::size_t>(nz) +
           static_cast<std::size_t>(z);
  };
  const auto lineThreshold = [this](int line_length) {
    return std::max(1, parallel_voxel_threshold_ /
                           std::max(1, line_length));
  };

  const double maximum = std::numeric_limits<double>::max();

  // Positive distance: z, y, then x transforms.
  if (!parallelFor(
          static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
          lineThreshold(nz),
          [&](std::size_t line) {
            const int x = static_cast<int>(line / static_cast<std::size_t>(ny));
            const int y = static_cast<int>(line % static_cast<std::size_t>(ny));
            fillDistanceTransform(
                nz, job.min_index.z(),
                [&](int z) {
                  return job.occupancy[address(x, y, z)] == 1 ? 0.0 : maximum;
                },
                [&](int z, double value) {
                  tmp_buffer1_[address(x, y, z)] = value;
                });
          }))
    return false;

  if (!parallelFor(
          static_cast<std::size_t>(nx) * static_cast<std::size_t>(nz),
          lineThreshold(ny),
          [&](std::size_t line) {
            const int x = static_cast<int>(line / static_cast<std::size_t>(nz));
            const int z = static_cast<int>(line % static_cast<std::size_t>(nz));
            fillDistanceTransform(
                ny, job.min_index.y(),
                [&](int y) { return tmp_buffer1_[address(x, y, z)]; },
                [&](int y, double value) {
                  tmp_buffer2_[address(x, y, z)] = value;
                });
          }))
    return false;

  if (!parallelFor(
          static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz),
          lineThreshold(nx),
          [&](std::size_t line) {
            const int y = static_cast<int>(line / static_cast<std::size_t>(nz));
            const int z = static_cast<int>(line % static_cast<std::size_t>(nz));
            fillDistanceTransform(
                nx, job.min_index.x(),
                [&](int x) { return tmp_buffer2_[address(x, y, z)]; },
                [&](int x, double value) {
                  result->distance_all[address(x, y, z)] =
                      job.resolution * std::sqrt(value);
                });
          }))
    return false;

  // Negative distance.  The original implementation materializes an inverse
  // occupancy byte grid; reading the inverse predicate directly is equivalent
  // and avoids another full-size allocation and pass.
  if (!parallelFor(
          static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
          lineThreshold(nz),
          [&](std::size_t line) {
            const int x = static_cast<int>(line / static_cast<std::size_t>(ny));
            const int y = static_cast<int>(line % static_cast<std::size_t>(ny));
            fillDistanceTransform(
                nz, job.min_index.z(),
                [&](int z) {
                  return job.occupancy[address(x, y, z)] == 0 ? 0.0 : maximum;
                },
                [&](int z, double value) {
                  tmp_buffer1_[address(x, y, z)] = value;
                });
          }))
    return false;

  if (!parallelFor(
          static_cast<std::size_t>(nx) * static_cast<std::size_t>(nz),
          lineThreshold(ny),
          [&](std::size_t line) {
            const int x = static_cast<int>(line / static_cast<std::size_t>(nz));
            const int z = static_cast<int>(line % static_cast<std::size_t>(nz));
            fillDistanceTransform(
                ny, job.min_index.y(),
                [&](int y) { return tmp_buffer1_[address(x, y, z)]; },
                [&](int y, double value) {
                  tmp_buffer2_[address(x, y, z)] = value;
                });
          }))
    return false;

  if (!parallelFor(
          static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz),
          lineThreshold(nx),
          [&](std::size_t line) {
            const int y = static_cast<int>(line / static_cast<std::size_t>(nz));
            const int z = static_cast<int>(line % static_cast<std::size_t>(nz));
            fillDistanceTransform(
                nx, job.min_index.x(),
                [&](int x) { return tmp_buffer2_[address(x, y, z)]; },
                [&](int x, double value) {
                  const double negative_distance =
                      job.resolution * std::sqrt(value);
                  const std::size_t index = address(x, y, z);
                  if (negative_distance > 0.0)
                  {
                    result->distance_all[index] +=
                        -negative_distance + job.resolution;
                  }
                });
          }))
    return false;

  return !computationCancelled();
}

}  // namespace plan_env

#include "plan_env/esdf_worker.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
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

struct PatchRegion
{
  Eigen::Vector3i changed_min, changed_max;
  Eigen::Vector3i write_min, write_max;
  Eigen::Vector3i input_min, input_max;
  std::size_t voxel_count = 0;
  std::vector<std::size_t> members;
};

struct ChangeBounds
{
  Eigen::Vector3i minimum = Eigen::Vector3i::Zero();
  Eigen::Vector3i maximum = Eigen::Vector3i::Zero();
  std::size_t count = 0;

  void include(const Eigen::Vector3i &cell)
  {
    if (count++ == 0)
      minimum = maximum = cell;
    else
    {
      minimum = minimum.cwiseMin(cell);
      maximum = maximum.cwiseMax(cell);
    }
  }

  void include(const ChangeBounds &other)
  {
    if (other.count == 0) return;
    if (count == 0)
      *this = other;
    else
    {
      minimum = minimum.cwiseMin(other.minimum);
      maximum = maximum.cwiseMax(other.maximum);
      count += other.count;
    }
  }
};

Eigen::Vector3i expandedBound(const Eigen::Vector3i &bound, int padding,
                              const Eigen::Vector3i &limit, bool lower)
{
  Eigen::Vector3i result;
  for (int axis = 0; axis < 3; ++axis)
  {
    const std::int64_t expanded = static_cast<std::int64_t>(bound(axis)) +
        (lower ? -static_cast<std::int64_t>(padding) : padding);
    result(axis) = static_cast<int>(lower
        ? std::max(expanded, static_cast<std::int64_t>(limit(axis)))
        : std::min(expanded, static_cast<std::int64_t>(limit(axis))));
  }
  return result;
}

void setRegionGeometry(PatchRegion *region, const plan_env::ESDFJob &job,
                       int radius)
{
  region->write_min = expandedBound(region->changed_min, radius, job.min_index, true);
  region->write_max = expandedBound(region->changed_max, radius, job.max_index, false);
  region->input_min = expandedBound(region->write_min, radius, job.min_index, true);
  region->input_max = expandedBound(region->write_max, radius, job.max_index, false);
  Eigen::Vector3i size;
  if (!compactVoxelCount(region->input_min, region->input_max,
                         &size, &region->voxel_count))
    throw std::invalid_argument("invalid ESDF patch bounds");
}

template <typename GetValue, typename SetValue>
void fillDistanceTransform(int length,
                           int coordinate_offset,
                           const GetValue &get_value,
                           const SetValue &set_value)
{
  // Gather each strided y/x line once. Envelope construction and evaluation
  // then read contiguous scratch instead of repeatedly revisiting the grid.
  // Scratch is axis-sized and private to each persistent worker thread.
  thread_local std::vector<double> values;
  thread_local std::vector<int> sites;
  thread_local std::vector<double> boundaries;
  values.resize(static_cast<std::size_t>(length));
  bool constant = true;
  for (int local_q = 0; local_q < length; ++local_q)
  {
    values[static_cast<std::size_t>(local_q)] = get_value(local_q);
    if (local_q > 0 && values[static_cast<std::size_t>(local_q)] != values[0])
      constant = false;
  }
  // min_i((q-i)^2 + c) == c, including the no-seed MAX sentinel. This is
  // common for planar/empty lines and needs no parabola envelope.
  if (constant)
  {
    for (int local_q = 0; local_q < length; ++local_q)
      set_value(local_q, values[0]);
    return;
  }
  sites.resize(static_cast<std::size_t>(length));
  boundaries.resize(static_cast<std::size_t>(length) + 1U);

  const double maximum = std::numeric_limits<double>::max();
  int k = -1;
  for (int local_q = 0; local_q < length; ++local_q)
  {
    // MAX marks a line without a seed after an earlier axis. Such a site
    // cannot minimize the result when any finite-distance site is present.
    if (values[static_cast<std::size_t>(local_q)] == maximum)
      continue;
    if (k < 0)
    {
      k = 0;
      sites[0] = local_q;
      boundaries[0] = -maximum;
      boundaries[1] = maximum;
      continue;
    }
    ++k;
    double intersection = 0.0;
    const int q = coordinate_offset + local_q;

    do
    {
      --k;
      const int site = coordinate_offset + sites[static_cast<std::size_t>(k)];
      intersection =
          ((values[static_cast<std::size_t>(local_q)] + q * q) -
           (values[static_cast<std::size_t>(sites[static_cast<std::size_t>(k)])] + site * site)) /
          (2 * q - 2 * site);
    } while (intersection <= boundaries[static_cast<std::size_t>(k)]);

    ++k;
    sites[static_cast<std::size_t>(k)] = local_q;
    boundaries[static_cast<std::size_t>(k)] = intersection;
    boundaries[static_cast<std::size_t>(k + 1)] = maximum;
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
        delta * delta + values[static_cast<std::size_t>(sites[static_cast<std::size_t>(k)])];
    set_value(local_q, value);
  }
}

template <typename IsSeed, typename GetValue, typename SetValue>
void fillBinaryDistanceTransform(int length, const IsSeed &is_seed,
                                 const GetValue &get_value,
                                 const SetValue &set_value)
{
  // The first axis has only zero-cost seeds and MAX elsewhere. Its nearest
  // seed is either the nearest one on the left or the nearest one on the
  // right; no parabola envelope is needed. Reuse the output as forward-pass
  // storage. Local coordinate differences are invariant to the global offset.
  const double maximum = std::numeric_limits<double>::max();
  int previous = -1;
  for (int coordinate = 0; coordinate < length; ++coordinate)
  {
    if (is_seed(coordinate)) previous = coordinate;
    const double delta = static_cast<double>(coordinate) - previous;
    set_value(coordinate, previous >= 0 ? delta * delta : maximum);
  }
  int next = -1;
  for (int coordinate = length - 1; coordinate >= 0; --coordinate)
  {
    if (is_seed(coordinate)) next = coordinate;
    if (next < 0) continue;
    const double delta = static_cast<double>(next) - coordinate;
    const double value = delta * delta;
    if (value < get_value(coordinate)) set_value(coordinate, value);
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
  if (!std::isfinite(job.distance_cap) || job.distance_cap < 0.0 ||
      (job.distance_cap > 0.0 &&
       std::ceil(job.distance_cap / job.resolution) + 1.0 >
           static_cast<double>(std::numeric_limits<int>::max())))
  {
    *reason = "ESDF distance cap must be finite, nonnegative and fit grid indices";
    return false;
  }
  for (const Eigen::Vector3i &cell : job.changed_cells)
    if ((cell.array() < job.min_index.array()).any() ||
        (cell.array() > job.max_index.array()).any())
    {
      *reason = "changed ESDF cell outside immutable occupancy snapshot";
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
    result.compute_started_steady = compute_start;
    result.compute_completed_steady = compute_end;
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
  result->patches.clear();
  result->distance_all.clear();
  result->computed_voxels = 0;
  result->prepare_ms = result->partition_ms = result->near_free_ms = 0.0;
  result->negative_apply_ms = 0.0;
  result->positive_ms.fill(0.0);
  result->negative_ms.fill(0.0);
  result->negative_fallback_count = 0;
  if (job.distance_cap <= 0.0 || job.changed_cells.empty())
    return computeDense(job, result);

  const ros::SteadyTime partition_start = job.trace_timing
      ? ros::SteadyTime::now() : ros::SteadyTime();
  const int radius = static_cast<int>(
      std::ceil(job.distance_cap / job.resolution)) + 1;
  PatchRegion root;
  root.changed_min = job.max_index;
  root.changed_max = job.min_index;
  root.members.resize(job.changed_cells.size());
  std::iota(root.members.begin(), root.members.end(), std::size_t(0));
  for (std::size_t index = 0; index < job.changed_cells.size(); ++index)
  {
    if ((index & 4095U) == 0U && computationCancelled()) return false;
    root.changed_min = root.changed_min.cwiseMin(job.changed_cells[index]);
    root.changed_max = root.changed_max.cwiseMax(job.changed_cells[index]);
  }
  setRegionGeometry(&root, job, radius);
  std::vector<PatchRegion> regions;
  regions.push_back(std::move(root));

  // Split only when the additional halos still save at least 15% of a
  // region's actual EDT work. This bounds both duplicate halo work and the
  // number of result patches without introducing a second scheduler.
  while (regions.size() < 8U)
  {
    if (computationCancelled()) return false;
    std::size_t best_region = regions.size(), best_gain = 0;
    int best_axis = 0, best_midpoint = 0;
    PatchRegion best_left, best_right;
    for (std::size_t index = 0; index < regions.size(); ++index)
    {
      const PatchRegion &region = regions[index];
      if (region.members.size() < 2U) continue;
      for (int axis = 0; axis < 3; ++axis)
      {
        const int span = region.changed_max(axis) - region.changed_min(axis);
        if (span <= 0) continue;
        // Coordinate bins let every occupied cut be considered without
        // rescanning all members per cut. Scratch storage is one axis long;
        // no occupancy/distance grid is copied while choosing a partition.
        std::vector<ChangeBounds> bins(static_cast<std::size_t>(span) + 1U);
        for (std::size_t member_index = 0; member_index < region.members.size(); ++member_index)
        {
          if ((member_index & 4095U) == 0U && computationCancelled()) return false;
          const Eigen::Vector3i &cell = job.changed_cells[region.members[member_index]];
          bins[static_cast<std::size_t>(cell(axis) - region.changed_min(axis))].include(cell);
        }
        std::vector<ChangeBounds> suffix(bins.size() + 1U);
        for (std::size_t coordinate = bins.size(); coordinate > 0; --coordinate)
        {
          if ((coordinate & 4095U) == 0U && computationCancelled()) return false;
          suffix[coordinate - 1U] = suffix[coordinate];
          suffix[coordinate - 1U].include(bins[coordinate - 1U]);
        }
        ChangeBounds prefix;
        for (std::size_t coordinate = 0; coordinate + 1U < bins.size(); ++coordinate)
        {
          if ((coordinate & 4095U) == 0U && computationCancelled()) return false;
          prefix.include(bins[coordinate]);
          if (bins[coordinate].count == 0 || suffix[coordinate + 1U].count == 0)
            continue;
          PatchRegion left, right;
          left.changed_min = prefix.minimum; left.changed_max = prefix.maximum;
          right.changed_min = suffix[coordinate + 1U].minimum;
          right.changed_max = suffix[coordinate + 1U].maximum;
          setRegionGeometry(&left, job, radius);
          setRegionGeometry(&right, job, radius);
          const double child_work = static_cast<double>(left.voxel_count) +
                                    static_cast<double>(right.voxel_count);
          if (child_work > .85 * static_cast<double>(region.voxel_count)) continue;
          const std::size_t gain = region.voxel_count - left.voxel_count - right.voxel_count;
          if (gain > best_gain)
          {
            best_region = index; best_gain = gain;
            best_axis = axis;
            best_midpoint = region.changed_min(axis) + static_cast<int>(coordinate);
            best_left = std::move(left); best_right = std::move(right);
          }
        }
      }
    }
    if (best_region == regions.size()) break;
    for (std::size_t member : regions[best_region].members)
      (job.changed_cells[member](best_axis) <= best_midpoint
          ? best_left.members : best_right.members).push_back(member);
    regions[best_region] = std::move(best_left);
    regions.push_back(std::move(best_right));
  }

  std::size_t computed = 0;
  for (const PatchRegion &region : regions)
  {
    if (region.voxel_count > std::numeric_limits<std::size_t>::max() - computed)
    {
      if (job.trace_timing)
        result->partition_ms = (ros::SteadyTime::now() - partition_start).toSec() * 1e3;
      return computeDense(job, result);
    }
    computed += region.voxel_count;
  }
  if (job.trace_timing)
    result->partition_ms = (ros::SteadyTime::now() - partition_start).toSec() * 1e3;
  if (regions.size() < 2U ||
      static_cast<double>(computed) > .85 * static_cast<double>(job.occupancy.size()))
    return computeDense(job, result);

  const Eigen::Vector3i parent_size = job.max_index - job.min_index + Eigen::Vector3i::Ones();
  std::vector<ESDFPatch> patches;
  patches.reserve(regions.size());
  for (const PatchRegion &region : regions)
  {
    if (computationCancelled()) return false;
    ESDFJob local;
    local.request_id = job.request_id; local.map_epoch = job.map_epoch;
    local.observation_seq = job.observation_seq;
    local.min_index = region.input_min; local.max_index = region.input_max;
    local.resolution = job.resolution; local.captured_steady = job.captured_steady;
    local.trace_timing = job.trace_timing;
    const Eigen::Vector3i input_size = region.input_max - region.input_min + Eigen::Vector3i::Ones();
    local.occupancy.resize(region.voxel_count);
    for (int x = 0; x < input_size.x(); ++x)
      for (int y = 0; y < input_size.y(); ++y)
      {
        if ((y & 63) == 0 && computationCancelled()) return false;
        const std::size_t source =
            (static_cast<std::size_t>(x + region.input_min.x() - job.min_index.x()) * parent_size.y() +
             y + region.input_min.y() - job.min_index.y()) * parent_size.z() +
            region.input_min.z() - job.min_index.z();
        const std::size_t destination = (static_cast<std::size_t>(x) * input_size.y() + y) * input_size.z();
        std::copy_n(job.occupancy.begin() + source, input_size.z(), local.occupancy.begin() + destination);
      }
    ESDFResult dense;
    if (!computeDense(local, &dense)) return false;
    result->prepare_ms += dense.prepare_ms;
    result->near_free_ms += dense.near_free_ms;
    result->negative_apply_ms += dense.negative_apply_ms;
    result->negative_fallback_count += dense.negative_fallback_count;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      result->positive_ms[axis] += dense.positive_ms[axis];
      result->negative_ms[axis] += dense.negative_ms[axis];
    }
    ESDFPatch patch;
    patch.min_index = region.write_min; patch.max_index = region.write_max;
    const Eigen::Vector3i output_size = region.write_max - region.write_min + Eigen::Vector3i::Ones();
    patch.distance_all.resize(static_cast<std::size_t>(output_size.x()) * output_size.y() * output_size.z());
    for (int x = 0; x < output_size.x(); ++x)
      for (int y = 0; y < output_size.y(); ++y)
      {
        if ((y & 63) == 0 && computationCancelled()) return false;
        const std::size_t source =
            (static_cast<std::size_t>(x + region.write_min.x() - region.input_min.x()) * input_size.y() +
             y + region.write_min.y() - region.input_min.y()) * input_size.z() +
            region.write_min.z() - region.input_min.z();
        const std::size_t destination = (static_cast<std::size_t>(x) * output_size.y() + y) * output_size.z();
        std::copy_n(dense.distance_all.begin() + source, output_size.z(), patch.distance_all.begin() + destination);
      }
    patches.push_back(std::move(patch));
  }
  if (computationCancelled()) return false;
  // Publish one complete batch only. No partially computed leaf is visible to
  // the coordinator, and the parent generation/source time remain unchanged.
  result->request_id = job.request_id; result->map_epoch = job.map_epoch;
  result->observation_seq = job.observation_seq;
  result->min_index = job.min_index; result->max_index = job.max_index;
  result->captured_steady = job.captured_steady;
  result->computed_voxels = computed;
  result->patches = std::move(patches);
  return true;
}

bool ESDFWorker::computeDense(const ESDFJob &job, ESDFResult *result)
{
  ros::SteadyTime phase_start = job.trace_timing
      ? ros::SteadyTime::now() : ros::SteadyTime();
  const auto finish_phase = [&](double &elapsed_ms) {
    if (!job.trace_timing) return;
    const ros::SteadyTime now = ros::SteadyTime::now();
    elapsed_ms += (now - phase_start).toSec() * 1e3;
    phase_start = now;
  };
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
  result->patches.clear();
  result->computed_voxels = voxel_count;

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
  finish_phase(result->prepare_ms);

  // Positive distance: z, y, then x transforms.
  if (!parallelFor(
          static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
          lineThreshold(nz),
          [&](std::size_t line) {
            const int x = static_cast<int>(line / static_cast<std::size_t>(ny));
            const int y = static_cast<int>(line % static_cast<std::size_t>(ny));
            fillBinaryDistanceTransform(
                nz,
                [&](int z) { return job.occupancy[address(x, y, z)] == 1; },
                [&](int z) { return tmp_buffer1_[address(x, y, z)]; },
                [&](int z, double value) {
                  tmp_buffer1_[address(x, y, z)] = value;
                });
          }))
    return false;
  finish_phase(result->positive_ms[0]);

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
  finish_phase(result->positive_ms[1]);

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
  finish_phase(result->positive_ms[2]);

  // All integer offsets within radius two, ordered by squared distance.
  // Scratch holds exact candidates; a single unresolved occupied cell sends
  // the whole job through the original negative EDT before results change.
  static const int near_free[][4] = {
      {-1,0,0,1}, {1,0,0,1}, {0,-1,0,1}, {0,1,0,1}, {0,0,-1,1}, {0,0,1,1},
      {-1,-1,0,2}, {-1,1,0,2}, {1,-1,0,2}, {1,1,0,2},
      {-1,0,-1,2}, {-1,0,1,2}, {1,0,-1,2}, {1,0,1,2},
      {0,-1,-1,2}, {0,-1,1,2}, {0,1,-1,2}, {0,1,1,2},
      {-1,-1,-1,3}, {-1,-1,1,3}, {-1,1,-1,3}, {-1,1,1,3},
      {1,-1,-1,3}, {1,-1,1,3}, {1,1,-1,3}, {1,1,1,3},
      {-2,0,0,4}, {2,0,0,4}, {0,-2,0,4}, {0,2,0,4}, {0,0,-2,4}, {0,0,2,4}};
  std::atomic<bool> unresolved(false);
  if (!parallelFor(static_cast<std::size_t>(nx) * ny, lineThreshold(nz),
          [&](std::size_t line) {
            if (unresolved.load(std::memory_order_relaxed)) return;
            const int x = static_cast<int>(line / ny), y = static_cast<int>(line % ny);
            for (int z = 0; z < nz; ++z)
            {
              const std::size_t index = address(x, y, z);
              if (job.occupancy[index] == 0) continue;
              bool found = false;
              if (job.occupancy[index] == 1)
                for (const auto &offset : near_free)
                {
                  const int sx = x + offset[0], sy = y + offset[1], sz = z + offset[2];
                  if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz)
                    continue;  // Outside this immutable ROI is not free space.
                  if (job.occupancy[address(sx, sy, sz)] == 0)
                  {
                    tmp_buffer1_[index] = offset[3];
                    found = true;
                    break;
                  }
                }
              if (!found)
              {
                unresolved.store(true, std::memory_order_relaxed);
                return;
              }
            }
          }))
    return false;
  finish_phase(result->near_free_ms);
  if (!unresolved.load(std::memory_order_relaxed))
  {
    if (!parallelFor(static_cast<std::size_t>(nx) * ny, lineThreshold(nz),
            [&](std::size_t line) {
              for (int z = 0; z < nz; ++z)
              {
                const std::size_t index = line * nz + z;
                if (job.occupancy[index] != 1) continue;
                const double negative_distance = job.resolution * std::sqrt(tmp_buffer1_[index]);
                if (negative_distance > 0.0)
                  result->distance_all[index] += -negative_distance + job.resolution;
              }
            }))
      return false;
    finish_phase(result->negative_apply_ms);
    return !computationCancelled();
  }

  ++result->negative_fallback_count;
  // Negative distance.  The original implementation materializes an inverse
  // occupancy byte grid; reading the inverse predicate directly is equivalent
  // and avoids another full-size allocation and pass.
  if (!parallelFor(
          static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny),
          lineThreshold(nz),
          [&](std::size_t line) {
            const int x = static_cast<int>(line / static_cast<std::size_t>(ny));
            const int y = static_cast<int>(line % static_cast<std::size_t>(ny));
            fillBinaryDistanceTransform(
                nz,
                [&](int z) { return job.occupancy[address(x, y, z)] == 0; },
                [&](int z) { return tmp_buffer1_[address(x, y, z)]; },
                [&](int z, double value) {
                  tmp_buffer1_[address(x, y, z)] = value;
                });
          }))
    return false;
  finish_phase(result->negative_ms[0]);

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
  finish_phase(result->negative_ms[1]);

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
  finish_phase(result->negative_ms[2]);

  return !computationCancelled();
}

}  // namespace plan_env

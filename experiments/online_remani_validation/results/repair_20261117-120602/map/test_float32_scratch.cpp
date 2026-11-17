#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <thread>
#include <vector>

#include <plan_env/esdf_worker.h>

namespace
{

std::size_t voxelCount(const plan_env::ESDFJob &job)
{
  const Eigen::Vector3i size =
      job.max_index - job.min_index + Eigen::Vector3i::Ones();
  return static_cast<std::size_t>(size.x()) * size.y() * size.z();
}

std::vector<double> bruteForceSignedDistance(
    const plan_env::ESDFJob &job)
{
  const Eigen::Vector3i size =
      job.max_index - job.min_index + Eigen::Vector3i::Ones();
  const auto address = [&size](int x, int y, int z) {
    return (static_cast<std::size_t>(x) * size.y() + y) * size.z() + z;
  };
  std::vector<double> result(voxelCount(job), 0.0);
  for (int x = 0; x < size.x(); ++x)
  {
    for (int y = 0; y < size.y(); ++y)
    {
      for (int z = 0; z < size.z(); ++z)
      {
        double positive_squared = std::numeric_limits<double>::max();
        double negative_squared = std::numeric_limits<double>::max();
        for (int source_x = 0; source_x < size.x(); ++source_x)
        {
          for (int source_y = 0; source_y < size.y(); ++source_y)
          {
            for (int source_z = 0; source_z < size.z(); ++source_z)
            {
              const int dx = x - source_x;
              const int dy = y - source_y;
              const int dz = z - source_z;
              const double squared = dx * dx + dy * dy + dz * dz;
              if (job.occupancy[address(source_x, source_y, source_z)] == 1)
                positive_squared = std::min(positive_squared, squared);
              else
                negative_squared = std::min(negative_squared, squared);
            }
          }
        }
        const std::size_t index = address(x, y, z);
        result[index] = job.resolution * std::sqrt(positive_squared);
        const double negative_distance =
            job.resolution * std::sqrt(negative_squared);
        if (negative_distance > 0.0)
          result[index] += -negative_distance + job.resolution;
      }
    }
  }
  return result;
}

plan_env::ESDFJob makeJob(std::uint64_t request_id,
                          std::uint64_t epoch,
                          const Eigen::Vector3i &min_index,
                          const Eigen::Vector3i &size,
                          int pattern)
{
  plan_env::ESDFJob job;
  job.request_id = request_id;
  job.map_epoch = epoch;
  job.observation_seq = request_id * 10;
  job.min_index = min_index;
  job.max_index = min_index + size - Eigen::Vector3i::Ones();
  job.resolution = 0.05;
  job.captured_steady = ros::SteadyTime::now();
  job.occupancy.assign(voxelCount(job), 0);

  if (pattern == 1)
  {
    std::fill(job.occupancy.begin(), job.occupancy.end(), 1);
  }
  else if (pattern == 2)
  {
    for (int x = 0; x < size.x(); ++x)
    {
      for (int y = 0; y < size.y(); ++y)
      {
        for (int z = 0; z < size.z(); ++z)
        {
          if (((x * 37 + y * 17 + z * 11) % 41) == 0 ||
              (x == 4 && y >= 2 && y <= 8 && z == 3))
          {
            const std::size_t address =
                (static_cast<std::size_t>(x) * size.y() + y) * size.z() + z;
            job.occupancy[address] = 1;
          }
        }
      }
    }
  }
  return job;
}

bool waitUntilRunning(plan_env::ESDFWorker *worker,
                      const std::chrono::milliseconds &timeout)
{
  const std::chrono::steady_clock::time_point deadline =
      std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (worker->stats().running)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

bool waitUntilCompleted(plan_env::ESDFWorker *worker,
                        std::uint64_t completed,
                        const std::chrono::milliseconds &timeout)
{
  const std::chrono::steady_clock::time_point deadline =
      std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline)
  {
    if (worker->stats().completed >= completed)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

std::size_t globalAddress(const plan_env::ESDFJob &job,
                          const Eigen::Vector3i &cell)
{
  const Eigen::Vector3i size = job.max_index - job.min_index + Eigen::Vector3i::Ones();
  const Eigen::Vector3i local = cell - job.min_index;
  return (static_cast<std::size_t>(local.x()) * size.y() + local.y()) * size.z() + local.z();
}

double capped(double value, double cap)
{
  return std::max(-cap, std::min(cap, value));
}

void expectCompletePatchUpdate(const plan_env::ESDFJob &job,
                              const std::vector<double> &previous,
                              const std::vector<double> &expected,
                              const plan_env::ESDFResult &result)
{
  ASSERT_TRUE(result.distance_all.empty());
  ASSERT_GE(result.patches.size(), 2U);
  ASSERT_LE(result.patches.size(), 8U);
  ASSERT_EQ(previous.size(), expected.size());
  EXPECT_EQ(job.request_id, result.request_id);
  EXPECT_EQ(job.map_epoch, result.map_epoch);
  EXPECT_EQ(job.observation_seq, result.observation_seq);
  EXPECT_EQ(job.min_index, result.min_index);
  EXPECT_EQ(job.max_index, result.max_index);
  EXPECT_EQ(job.captured_steady, result.captured_steady);
  EXPECT_LT(result.computed_voxels, voxelCount(job));
  std::vector<double> merged(previous.size());
  for (std::size_t index = 0; index < merged.size(); ++index)
    merged[index] = capped(previous[index], job.distance_cap);
  std::vector<char> written(previous.size(), 0);
  for (const plan_env::ESDFPatch &patch : result.patches)
  {
    ASSERT_TRUE((patch.min_index.array() >= job.min_index.array()).all());
    ASSERT_TRUE((patch.max_index.array() <= job.max_index.array()).all());
    const Eigen::Vector3i size = patch.max_index - patch.min_index + Eigen::Vector3i::Ones();
    ASSERT_TRUE((size.array() > 0).all());
    ASSERT_EQ(static_cast<std::size_t>(size.x()) * size.y() * size.z(), patch.distance_all.size());
    for (int x = 0; x < size.x(); ++x)
      for (int y = 0; y < size.y(); ++y)
        for (int z = 0; z < size.z(); ++z)
        {
          const std::size_t source = (static_cast<std::size_t>(x) * size.y() + y) * size.z() + z;
          const std::size_t destination = globalAddress(job, patch.min_index + Eigen::Vector3i(x, y, z));
          ASSERT_TRUE(std::isfinite(patch.distance_all[source]));
          const double value = capped(patch.distance_all[source], job.distance_cap);
          ASSERT_DOUBLE_EQ(capped(expected[destination], job.distance_cap), value)
              << "patch source=" << source << " map destination=" << destination;
          if (written[destination]) EXPECT_DOUBLE_EQ(merged[destination], value);
          merged[destination] = value;
          written[destination] = 1;
        }
  }
  // Check the whole map after one atomic commit, including cells outside all
  // output patches. Deletions must not leave an old nearest obstacle behind.
  for (std::size_t index = 0; index < merged.size(); ++index)
    ASSERT_DOUBLE_EQ(capped(expected[index], job.distance_cap), merged[index]) << index;
}

void expectShapeUsesSparsePatches(plan_env::ESDFJob previous,
                                 const std::vector<Eigen::Vector3i> &changes,
                                 bool require_overlap)
{
  plan_env::ESDFWorker worker(3, 1);
  ASSERT_TRUE(worker.submit(previous));
  plan_env::ESDFResult old_dense;
  ASSERT_TRUE(worker.waitForResult(&old_dense, std::chrono::milliseconds(10000)));
  std::uint64_t request_id = previous.request_id;
  for (int frame = 0; frame < 2; ++frame)
  {
    // Toggle twice: each shape must support both additions and deletions.
    plan_env::ESDFJob dense = previous;
    dense.request_id = ++request_id;
    ++dense.observation_seq;
    for (const Eigen::Vector3i &cell : changes)
      dense.occupancy[globalAddress(dense, cell)] ^= 1;
    ASSERT_TRUE(worker.submit(dense));
    plan_env::ESDFResult expected;
    ASSERT_TRUE(worker.waitForResult(&expected, std::chrono::milliseconds(10000)));
    plan_env::ESDFJob sparse = dense;
    sparse.request_id = ++request_id;
    sparse.distance_cap = .6;
    sparse.changed_cells = changes;
    ASSERT_TRUE(worker.submit(sparse));
    plan_env::ESDFResult actual;
    ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
    expectCompletePatchUpdate(sparse, old_dense.distance_all, expected.distance_all, actual);
    EXPECT_LT(actual.computed_voxels, .7 * static_cast<double>(voxelCount(sparse)));
    if (require_overlap)
    {
      bool overlaps = false;
      for (std::size_t first = 0; first < actual.patches.size(); ++first)
        for (std::size_t second = first + 1U; second < actual.patches.size(); ++second)
          overlaps |= (actual.patches[first].min_index.cwiseMax(actual.patches[second].min_index).array() <=
                       actual.patches[first].max_index.cwiseMin(actual.patches[second].max_index).array()).all();
      EXPECT_TRUE(overlaps);
    }
    previous = std::move(dense);
    old_dense = std::move(expected);
  }
}

}  // namespace

TEST(ESDFWorker, MatchesSignedDistanceReferenceForEmptyFullAndMixedSnapshots)
{
  plan_env::ESDFWorker worker(3, 1);

  for (int pattern = 0; pattern <= 2; ++pattern)
  {
    plan_env::ESDFJob job = makeJob(
        static_cast<std::uint64_t>(pattern + 1), 1,
        Eigen::Vector3i(2, 3, 1), Eigen::Vector3i(14, 11, 9), pattern);
    const std::vector<double> expected = bruteForceSignedDistance(job);

    // Submit as an lvalue, then destroy the caller's data.  This also checks
    // that the worker owns an immutable deep snapshot.
    ASSERT_TRUE(worker.submit(job));
    std::fill(job.occupancy.begin(), job.occupancy.end(),
              pattern == 1 ? 0 : 1);
    job.occupancy.clear();

    plan_env::ESDFResult result;
    ASSERT_TRUE(worker.waitForResult(
        &result, std::chrono::milliseconds(5000)));
    ASSERT_EQ(expected.size(), result.distance_all.size());
    EXPECT_TRUE(result.patches.empty());
    EXPECT_EQ(expected.size(), result.computed_voxels);
    EXPECT_EQ(static_cast<std::uint64_t>(pattern + 1), result.request_id);
    EXPECT_EQ(1U, result.map_epoch);
    EXPECT_EQ(static_cast<std::uint64_t>((pattern + 1) * 10),
              result.observation_seq);
    for (std::size_t index = 0; index < expected.size(); ++index)
    {
      ASSERT_TRUE(std::isfinite(result.distance_all[index])) << index;
      ASSERT_DOUBLE_EQ(expected[index], result.distance_all[index]) << index;
    }
  }

  const plan_env::ESDFWorkerStats stats = worker.stats();
  EXPECT_EQ(3U, stats.submitted);
  EXPECT_EQ(3U, stats.completed);
  EXPECT_EQ(0U, stats.faults) << stats.last_fault;
}

TEST(ESDFWorker, StridedEmptyPrefixesPlanesAndThickBlocksKeepExactSignedDistances)
{
  // Permute the long axis so seeds separated by empty lines exercise both
  // strided transforms. Nonzero/negative origins retain the native coordinate
  // convention; planar layers exercise nonzero constant intermediate lines.
  const std::vector<Eigen::Vector3i> sizes{
      Eigen::Vector3i(23, 7, 5), Eigen::Vector3i(5, 23, 7),
      Eigen::Vector3i(7, 5, 23), Eigen::Vector3i(33, 35, 3),
      Eigen::Vector3i(33, 3, 35), Eigen::Vector3i(3, 33, 35)};
  plan_env::ESDFWorker worker(3, 1);
  std::uint64_t request_id = 0;
  for (const auto &size : sizes)
    for (int pattern = 0; pattern < 6; ++pattern)
    {
      const Eigen::Vector3i origin = pattern % 2
          ? Eigen::Vector3i(-31, 19, -6) : Eigen::Vector3i(13, -17, 4);
      auto job = makeJob(++request_id, 1, origin, size, 0);
      for (int x = 0; x < size.x(); ++x)
        for (int y = 0; y < size.y(); ++y)
          for (int z = 0; z < size.z(); ++z)
          {
            const bool first = x == 0 && y == size.y()-1 && z == size.z()/2;
            const bool last = x == size.x()-1 && y == 0 && z == size.z()/2;
            const bool middle = x == size.x()/2 && y == size.y()/2 && z == 0;
            const bool occupied = pattern == 0 ? first :
                pattern == 1 ? last :
                pattern == 2 ? first || last || middle :
                pattern == 3 ? z == size.z()/2 :
                pattern == 4 ? x > 0 && y > 0 && z > 0 :
                ((x*13+y*7+z*3) % 37 == 0);
            job.occupancy[(static_cast<std::size_t>(x)*size.y()+y)*size.z()+z] = occupied;
          }
      const auto expected = bruteForceSignedDistance(job);
      ASSERT_TRUE(worker.submit(job));
      plan_env::ESDFResult actual;
      ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
      ASSERT_EQ(expected.size(), actual.distance_all.size());
      for (std::size_t index = 0; index < expected.size(); ++index)
        ASSERT_EQ(expected[index], actual.distance_all[index])
            << "size=" << size.transpose() << " pattern=" << pattern << " voxel=" << index;
    }
}

TEST(ESDFWorker, PhaseTimingPreservesDistancesAndCoversBothNegativePaths)
{
  plan_env::ESDFWorker worker(3, 1);
  std::uint64_t request = 0;
  for (int pattern = 0; pattern <= 2; ++pattern)
  {
    plan_env::ESDFJob job = makeJob(++request, 1, Eigen::Vector3i::Zero(),
                                  Eigen::Vector3i(14, 11, 9), pattern);
    ASSERT_TRUE(worker.submit(job));
    plan_env::ESDFResult baseline;
    ASSERT_TRUE(worker.waitForResult(&baseline, std::chrono::milliseconds(5000)));
    job.request_id = ++request;
    job.trace_timing = true;
    ASSERT_TRUE(worker.submit(job));
    plan_env::ESDFResult traced;
    ASSERT_TRUE(worker.waitForResult(&traced, std::chrono::milliseconds(5000)));
    EXPECT_EQ(baseline.distance_all, traced.distance_all);
    EXPECT_EQ(baseline.computed_voxels, traced.computed_voxels);
    EXPECT_EQ(baseline.negative_fallback_count, traced.negative_fallback_count);
    EXPECT_GE(traced.compute_started_steady, job.captured_steady);
    EXPECT_GE(traced.compute_completed_steady, traced.compute_started_steady);
    double measured = traced.prepare_ms + traced.partition_ms +
                      traced.near_free_ms + traced.negative_apply_ms;
    EXPECT_GE(measured, 0.0);
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      EXPECT_GE(traced.positive_ms[axis], 0.0);
      EXPECT_GE(traced.negative_ms[axis], 0.0);
      measured += traced.positive_ms[axis] + traced.negative_ms[axis];
      EXPECT_EQ(0.0, baseline.positive_ms[axis]);
      EXPECT_EQ(0.0, baseline.negative_ms[axis]);
    }
    EXPECT_GT(measured, 0.0);
    EXPECT_LE(measured, traced.compute_ms + 1e-6);
    if (pattern == 0) EXPECT_EQ(0U, traced.negative_fallback_count);
    if (pattern == 1) EXPECT_EQ(1U, traced.negative_fallback_count);
  }
}

TEST(ESDFWorker, ExactNearFreeAndFallbackDistancesMatchBruteForceAtROIBounds)
{
  plan_env::ESDFWorker worker(3, 1);
  std::uint64_t request_id = 800;
  const auto check = [&](int width, int touching_axes,
                         const Eigen::Vector3i &free_offset) {
    auto job = makeJob(++request_id, 14, Eigen::Vector3i(3, 4, 2),
                       Eigen::Vector3i(11, 11, 11), 0);
    Eigen::Vector3i lower = Eigen::Vector3i::Constant((11 - width) / 2);
    for (int axis = 0; axis < touching_axes; ++axis) lower(axis) = 0;
    for (int x = 0; x < width; ++x)
      for (int y = 0; y < width; ++y)
        for (int z = 0; z < width; ++z)
          job.occupancy[globalAddress(job, job.min_index + lower +
              Eigen::Vector3i(x, y, z))] = 1;
    if (free_offset.squaredNorm() > 0)
      job.occupancy[globalAddress(job, job.min_index +
          Eigen::Vector3i::Constant(5) + free_offset)] = 0;
    const auto expected = bruteForceSignedDistance(job);
    ASSERT_TRUE(worker.submit(job));
    plan_env::ESDFResult actual;
    ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
    ASSERT_EQ(expected.size(), actual.distance_all.size());
    EXPECT_TRUE(actual.patches.empty());
    for (std::size_t index = 0; index < expected.size(); ++index)
      ASSERT_EQ(expected[index], actual.distance_all[index])
          << "width=" << width << " touching_axes=" << touching_axes
          << " hole=" << free_offset.transpose() << " cell=" << index;
  };
  for (int width : {3, 5})
    for (int touching_axes : {0, 1, 3})
    {
      // A centered/face-touching 3^3 block has maximum free distance two.
      // A 5^3 block needs distance three: every negative value must come
      // from the original full EDT, including cells resolved by the probe.
      // At a corner even 3^3 needs distance three; outside-ROI is not free.
      check(width, touching_axes, Eigen::Vector3i::Zero());
    }
  // The first free cell may be diagonal, closer than all axis-two sources.
  check(3, 0, Eigen::Vector3i(1, 1, 0));
  check(3, 0, Eigen::Vector3i(1, 1, 1));
}

TEST(ESDFWorker, CoalescesPendingWorkAndKeepsNewestCompletedResult)
{
  plan_env::ESDFWorker worker(1, 1);
  plan_env::ESDFJob slow = makeJob(
      100, 7, Eigen::Vector3i::Zero(), Eigen::Vector3i(200, 160, 100), 2);
  ASSERT_TRUE(worker.submit(std::move(slow)));
  ASSERT_TRUE(waitUntilRunning(&worker, std::chrono::milliseconds(2000)));

  plan_env::ESDFJob superseded = makeJob(
      101, 7, Eigen::Vector3i::Zero(), Eigen::Vector3i(7, 6, 5), 2);
  plan_env::ESDFJob newest = makeJob(
      102, 7, Eigen::Vector3i::Zero(), Eigen::Vector3i(8, 7, 6), 2);
  ASSERT_TRUE(worker.submit(std::move(superseded)));
  ASSERT_TRUE(worker.submit(std::move(newest)));

  plan_env::ESDFResult first;
  ASSERT_TRUE(worker.waitForResult(
      &first, std::chrono::milliseconds(10000)));
  EXPECT_EQ(102U, first.request_id)
      << "neither the older running job nor superseded pending request may "
         "be published after request 102 is accepted";

  const plan_env::ESDFWorkerStats stats = worker.stats();
  EXPECT_EQ(3U, stats.submitted);
  EXPECT_GE(stats.superseded, 2U);
  EXPECT_GE(stats.completed, 1U);
  EXPECT_LE(stats.completed, 3U);
  EXPECT_EQ(0U, stats.faults) << stats.last_fault;
}

TEST(ESDFWorker, NewRequestDropsAnUnconsumedReadyResult)
{
  plan_env::ESDFWorker worker(1, 1);
  plan_env::ESDFJob old_job = makeJob(
      300, 9, Eigen::Vector3i::Zero(), Eigen::Vector3i(8, 7, 6), 2);
  ASSERT_TRUE(worker.submit(std::move(old_job)));
  ASSERT_TRUE(waitUntilCompleted(
      &worker, 1, std::chrono::milliseconds(5000)));

  // Do not consume request 300.  Accepting request 301 must remove that ready
  // result synchronously, otherwise waitForResult() can return stale data
  // before the replacement has even started.
  plan_env::ESDFJob current_job = makeJob(
      301, 9, Eigen::Vector3i::Zero(), Eigen::Vector3i(9, 8, 7), 2);
  ASSERT_TRUE(worker.submit(std::move(current_job)));

  plan_env::ESDFResult result;
  ASSERT_TRUE(worker.waitForResult(
      &result, std::chrono::milliseconds(5000)));
  EXPECT_EQ(301U, result.request_id);
  EXPECT_GE(worker.stats().superseded, 1U);
}

TEST(ESDFWorker, DuplicateRequestIdRejected)
{
  plan_env::ESDFWorker worker(1, 1);

  plan_env::ESDFJob zero = makeJob(
      0, 10, Eigen::Vector3i::Zero(), Eigen::Vector3i(5, 4, 3), 2);
  EXPECT_FALSE(worker.submit(std::move(zero)));

  plan_env::ESDFJob accepted = makeJob(
      400, 10, Eigen::Vector3i::Zero(), Eigen::Vector3i(9, 8, 7), 2);
  ASSERT_TRUE(worker.submit(std::move(accepted)));

  plan_env::ESDFJob duplicate = makeJob(
      400, 10, Eigen::Vector3i::Zero(), Eigen::Vector3i(6, 5, 4), 1);
  EXPECT_FALSE(worker.submit(std::move(duplicate)));
  plan_env::ESDFJob out_of_order = makeJob(
      399, 10, Eigen::Vector3i::Zero(), Eigen::Vector3i(6, 5, 4), 0);
  EXPECT_FALSE(worker.submit(std::move(out_of_order)));

  plan_env::ESDFResult result;
  ASSERT_TRUE(worker.waitForResult(
      &result, std::chrono::milliseconds(5000)));
  EXPECT_EQ(400U, result.request_id);

  const plan_env::ESDFWorkerStats stats = worker.stats();
  EXPECT_EQ(1U, stats.submitted);
  EXPECT_EQ(3U, stats.superseded);
  EXPECT_EQ(0U, stats.faults) << stats.last_fault;
}

TEST(ESDFWorker, EpochInvalidationCancelsOldWork)
{
  plan_env::ESDFWorker worker(2, 1);
  plan_env::ESDFJob old_job = makeJob(
      200, 3, Eigen::Vector3i::Zero(), Eigen::Vector3i(180, 140, 90), 2);
  ASSERT_TRUE(worker.submit(std::move(old_job)));
  ASSERT_TRUE(waitUntilRunning(&worker, std::chrono::milliseconds(2000)));

  worker.invalidateBeforeEpoch(4);
  plan_env::ESDFJob current_job = makeJob(
      201, 4, Eigen::Vector3i(1, 1, 1), Eigen::Vector3i(9, 8, 7), 2);
  ASSERT_TRUE(worker.submit(std::move(current_job)));

  plan_env::ESDFResult result;
  ASSERT_TRUE(worker.waitForResult(
      &result, std::chrono::milliseconds(5000)));
  EXPECT_EQ(201U, result.request_id);
  EXPECT_EQ(4U, result.map_epoch);
  EXPECT_GE(worker.stats().invalidated, 1U);
}

TEST(ESDFWorker, InvalidInputRecordsFaultAndStopIsIdempotent)
{
  plan_env::ESDFWorker worker(2, 1);
  plan_env::ESDFJob invalid;
  invalid.min_index = Eigen::Vector3i::Zero();
  invalid.max_index = Eigen::Vector3i::Ones();
  invalid.resolution = 0.0;
  invalid.occupancy.assign(8, 0);
  EXPECT_FALSE(worker.submit(std::move(invalid)));
  EXPECT_EQ(1U, worker.stats().faults);

  worker.stop();
  worker.stop();
  EXPECT_TRUE(worker.stats().stopped);
}

TEST(ESDFWorker, SparseInsertionsAndDeletionsMatchEveryCappedDenseVoxel)
{
  plan_env::ESDFWorker worker(3, 1);
  plan_env::ESDFJob previous = makeJob(1, 4, Eigen::Vector3i(7, 11, 3),
                                       Eigen::Vector3i(100, 78, 40), 0);
  const std::vector<Eigen::Vector3i> centers = {
      Eigen::Vector3i(12, 12, 12), Eigen::Vector3i(85, 65, 28)};
  for (const Eigen::Vector3i &center : centers)
    for (int x = -3; x <= 3; ++x)
      for (int y = -3; y <= 3; ++y)
        for (int z = -3; z <= 3; ++z)
          previous.occupancy[globalAddress(previous, previous.min_index + center + Eigen::Vector3i(x, y, z))] = 1;
  ASSERT_TRUE(worker.submit(previous));
  plan_env::ESDFResult old_dense;
  ASSERT_TRUE(worker.waitForResult(&old_dense, std::chrono::milliseconds(10000)));
  const std::vector<Eigen::Vector3i> changed_local = {
      centers[0], centers[1], Eigen::Vector3i(10, 63, 28), Eigen::Vector3i(88, 15, 12)};
  for (int frame = 0; frame < 2; ++frame)
  {
    plan_env::ESDFJob dense = previous;
    dense.request_id = 2 + frame * 2;
    dense.observation_seq += frame + 1;
    for (const Eigen::Vector3i &cell : changed_local)
    {
      const std::size_t address = globalAddress(dense, dense.min_index + cell);
      dense.occupancy[address] = 1 - dense.occupancy[address];
    }
    ASSERT_TRUE(worker.submit(dense));
    plan_env::ESDFResult expected;
    ASSERT_TRUE(worker.waitForResult(&expected, std::chrono::milliseconds(10000)));
    plan_env::ESDFJob sparse = dense;
    sparse.request_id += 1;
    sparse.distance_cap = .15;
    for (const Eigen::Vector3i &cell : changed_local)
      sparse.changed_cells.push_back(sparse.min_index + cell);
    ASSERT_TRUE(worker.submit(sparse));
    plan_env::ESDFResult actual;
    ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
    expectCompletePatchUpdate(sparse, old_dense.distance_all, expected.distance_all, actual);
    previous = std::move(dense);
    old_dense = std::move(expected);
  }
}

TEST(ESDFWorker, SparseSplitUsesShorterAxisWhenLongestAxisHalosCostMore)
{
  auto job = makeJob(600, 12, Eigen::Vector3i(7, 11, 3),
                    Eigen::Vector3i(180, 140, 60), 0);
  std::vector<Eigen::Vector3i> changes;
  for (int x = 30; x < 130; ++x)
    for (int y : {30, 110})
      for (int z = 20; z < 29; ++z)
        changes.push_back(job.min_index + Eigen::Vector3i(x, y, z));
  ASSERT_EQ(1800U, changes.size());
  // x is longest, but splitting it duplicates both y bands and their halos.
  // A y cut separates the bands, avoiding the old 1,512,000-voxel fallback.
  expectShapeUsesSparsePatches(std::move(job), changes, false);
}

TEST(ESDFWorker, SparseSplitFindsOffMidpointCornerAndPreservesOverlappingPatches)
{
  // Start occupied so the first toggle also tests negative signed distances
  // at the real .6m cap, then restore all deleted cells in the second frame.
  auto job = makeJob(700, 13, Eigen::Vector3i(9, 4, 2),
                    Eigen::Vector3i(130, 130, 50), 1);
  std::vector<Eigen::Vector3i> changes;
  for (int x = 0; x <= 100; ++x)
    changes.push_back(job.min_index + Eigen::Vector3i(x, 0, 20));
  for (int y = 1; y <= 100; ++y)
    changes.push_back(job.min_index + Eigen::Vector3i(100, y, 20));
  // Every axis midpoint fails the 15% work-saving test for this L shape;
  // cutting just before its corner works even though there is no empty bin.
  // The write halos overlap near that corner and must agree voxel for voxel.
  expectShapeUsesSparsePatches(std::move(job), changes, true);
}

TEST(ESDFWorker, PatchHalosPreserveOutsideNeighborsAndSignedDistancesAtMapEdges)
{
  plan_env::ESDFWorker worker(2, 1);
  for (int pattern = 0; pattern <= 1; ++pattern)
  {
    plan_env::ESDFJob old = makeJob(10 + pattern * 3, 7, Eigen::Vector3i(9, 4, 2),
                                    Eigen::Vector3i(72, 20, 16), pattern);
    // With cap=.1, radius=3. These opposite-class sources lie just outside
    // the corner output patches, but must be retained in their input halos.
    for (const Eigen::Vector3i &local : {Eigen::Vector3i(4, 0, 0), Eigen::Vector3i(67, 19, 15)})
      old.occupancy[globalAddress(old, old.min_index + local)] = 1 - pattern;
    ASSERT_TRUE(worker.submit(old));
    plan_env::ESDFResult previous;
    ASSERT_TRUE(worker.waitForResult(&previous, std::chrono::milliseconds(10000)));
    plan_env::ESDFJob dense = old;
    dense.request_id += 1;
    const std::vector<Eigen::Vector3i> corners = {dense.min_index, dense.max_index};
    for (const Eigen::Vector3i &cell : corners)
      dense.occupancy[globalAddress(dense, cell)] = 1 - pattern;
    ASSERT_TRUE(worker.submit(dense));
    plan_env::ESDFResult expected;
    ASSERT_TRUE(worker.waitForResult(&expected, std::chrono::milliseconds(10000)));
    plan_env::ESDFJob sparse = dense;
    sparse.request_id += 1;
    sparse.distance_cap = .1;
    sparse.changed_cells = corners;
    ASSERT_TRUE(worker.submit(sparse));
    plan_env::ESDFResult actual;
    ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
    expectCompletePatchUpdate(sparse, previous.distance_all, expected.distance_all, actual);
  }
}

TEST(ESDFWorker, SeededSparseFrameChangesMatchTheWholeCappedField)
{
  plan_env::ESDFWorker worker(3, 1);
  plan_env::ESDFJob previous = makeJob(20, 8, Eigen::Vector3i(5, 9, 2),
                                      Eigen::Vector3i(90, 70, 32), 0);
  std::mt19937 random(23);
  for (char &occupied : previous.occupancy) occupied = random() % 17 == 0 ? 1 : 0;
  ASSERT_TRUE(worker.submit(previous));
  plan_env::ESDFResult old_dense;
  ASSERT_TRUE(worker.waitForResult(&old_dense, std::chrono::milliseconds(10000)));
  const std::vector<Eigen::Vector3i> centers = {
      Eigen::Vector3i(7, 7, 7), Eigen::Vector3i(81, 7, 24),
      Eigen::Vector3i(7, 61, 24), Eigen::Vector3i(81, 61, 7)};
  for (int frame = 0; frame < 2; ++frame)
  {
    plan_env::ESDFJob dense = previous;
    dense.request_id = 21 + 2 * frame;
    dense.observation_seq += 1;
    std::vector<Eigen::Vector3i> changed;
    for (const Eigen::Vector3i &center : centers)
      for (int sample = 0; sample < 4; ++sample)
      {
        const Eigen::Vector3i cell = dense.min_index + center + Eigen::Vector3i(
            static_cast<int>(random() % 3) - 1,
            static_cast<int>(random() % 3) - 1,
            static_cast<int>(random() % 3) - 1);
        dense.occupancy[globalAddress(dense, cell)] ^= 1;
        changed.push_back(cell);
      }
    ASSERT_TRUE(worker.submit(dense));
    plan_env::ESDFResult expected;
    ASSERT_TRUE(worker.waitForResult(&expected, std::chrono::milliseconds(10000)));
    plan_env::ESDFJob sparse = dense;
    sparse.request_id += 1;
    sparse.distance_cap = .1;
    sparse.changed_cells = std::move(changed);
    ASSERT_TRUE(worker.submit(sparse));
    plan_env::ESDFResult actual;
    ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
    expectCompletePatchUpdate(sparse, old_dense.distance_all, expected.distance_all, actual);
    previous = std::move(dense);
    old_dense = std::move(expected);
  }
}

TEST(ESDFWorker, ConcentratedOrDenseChangesRetainOriginalDenseResult)
{
  plan_env::ESDFWorker worker(1, 1);
  plan_env::ESDFJob reference = makeJob(40, 3, Eigen::Vector3i(2, 3, 1),
                                      Eigen::Vector3i(24, 20, 16), 2);
  ASSERT_TRUE(worker.submit(reference));
  plan_env::ESDFResult expected;
  ASSERT_TRUE(worker.waitForResult(&expected, std::chrono::milliseconds(10000)));
  for (int pattern = 0; pattern < 4; ++pattern)
  {
    plan_env::ESDFJob job = reference;
    job.request_id = 41 + pattern;
    job.distance_cap = .15;
    if (pattern == 0)
      job.changed_cells = {job.min_index + Eigen::Vector3i(11, 9, 7),
                           job.min_index + Eigen::Vector3i(12, 10, 8)};
    else if (pattern == 1)
      for (int x = job.min_index.x(); x <= job.max_index.x(); ++x)
        for (int y = job.min_index.y(); y <= job.max_index.y(); ++y)
          for (int z = job.min_index.z(); z <= job.max_index.z(); ++z)
            job.changed_cells.emplace_back(x, y, z);
    else if (pattern == 3)
    {
      job.distance_cap = 0.;
      job.changed_cells = {job.min_index, job.max_index};
    }
    // pattern 2 verifies the empty changed-list compatibility path, and
    // pattern 3 verifies that a zero cap disables patching even with changes.
    ASSERT_TRUE(worker.submit(std::move(job)));
    plan_env::ESDFResult result;
    ASSERT_TRUE(worker.waitForResult(&result, std::chrono::milliseconds(10000)));
    EXPECT_TRUE(result.patches.empty());
    EXPECT_EQ(expected.distance_all, result.distance_all);
    EXPECT_EQ(voxelCount(reference), result.computed_voxels);
  }
}

TEST(ESDFWorker, InvalidPatchCapAndChangedIndicesAreRejected)
{
  plan_env::ESDFWorker worker(1, 1);
  const plan_env::ESDFJob valid = makeJob(1, 1, Eigen::Vector3i(3, 4, 5),
                                        Eigen::Vector3i(4, 3, 2), 0);
  for (double cap : {-1., std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN(), 1e300})
  {
    plan_env::ESDFJob invalid = valid;
    invalid.distance_cap = cap;
    EXPECT_FALSE(worker.submit(std::move(invalid)));
  }
  for (int axis = 0; axis < 3; ++axis)
    for (int side = 0; side < 2; ++side)
    {
      plan_env::ESDFJob invalid = valid;
      Eigen::Vector3i cell = side ? invalid.max_index : invalid.min_index;
      cell(axis) += side ? 1 : -1;
      invalid.distance_cap = .1;
      invalid.changed_cells.push_back(cell);
      EXPECT_FALSE(worker.submit(std::move(invalid)));
    }
  EXPECT_EQ(10U, worker.stats().faults);
  EXPECT_EQ(0U, worker.stats().submitted);
}

TEST(ESDFWorker, SupersedingAPatchBatchNeverPublishesPartialLeaves)
{
  plan_env::ESDFWorker worker(1, 1);
  plan_env::ESDFJob old = makeJob(100, 11, Eigen::Vector3i::Zero(),
                                Eigen::Vector3i(200, 160, 100), 2);
  old.distance_cap = .6;
  for (int x : {10, 25, 175, 190})
    for (int y : {0, 159})
      for (int z : {0, 99})
        old.changed_cells.emplace_back(x, y, z);
  ASSERT_TRUE(worker.submit(std::move(old)));
  ASSERT_TRUE(waitUntilRunning(&worker, std::chrono::milliseconds(2000)));
  plan_env::ESDFJob newest = makeJob(101, 11, Eigen::Vector3i(1, 2, 3),
                                   Eigen::Vector3i(9, 8, 7), 2);
  ASSERT_TRUE(worker.submit(newest));
  plan_env::ESDFResult result;
  ASSERT_TRUE(worker.waitForResult(&result, std::chrono::milliseconds(10000)));
  EXPECT_EQ(101U, result.request_id);
  EXPECT_TRUE(result.patches.empty());
  EXPECT_EQ(voxelCount(newest), result.distance_all.size());
  EXPECT_EQ(bruteForceSignedDistance(newest), result.distance_all);
  EXPECT_GE(worker.stats().superseded, 1U);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(ESDFWorker, ExactFloatScratchGuardIncludesLimitAndFallsBackAboveIt)
{
  const std::vector<Eigen::Vector3i> sizes{
      Eigen::Vector3i(1, 4097, 1), Eigen::Vector3i(1, 4098, 1),
      Eigen::Vector3i(1, 1, 4097), Eigen::Vector3i(1, 1, 4098),
      Eigen::Vector3i(2, 4097, 1), Eigen::Vector3i(7, 9, 11)};
  for (const int threads : {1, 3})
  {
    plan_env::ESDFWorker worker(threads, 1);
    std::uint64_t request = 0;
    for (const auto &size : sizes)
    {
      auto job = makeJob(++request, 1, Eigen::Vector3i(-3, 5, 2), size, 0);
      job.occupancy[0] = 1;
      std::uint64_t bound = 0;
      for (int axis = 0; axis < 3; ++axis)
        bound += static_cast<std::uint64_t>(size[axis]-1) * (size[axis]-1);
      ASSERT_TRUE(worker.submit(job));
      plan_env::ESDFResult actual;
      ASSERT_TRUE(worker.waitForResult(&actual, std::chrono::milliseconds(10000)));
      ASSERT_EQ(voxelCount(job), actual.distance_all.size());
      EXPECT_EQ(bound <= (std::uint64_t{1} << 24) ? 1U : 0U,
                actual.exact_float_scratch_count) << size.transpose();
      for (int x = 0; x < size.x(); ++x)
        for (int y = 0; y < size.y(); ++y)
          for (int z = 0; z < size.z(); ++z)
          {
            const std::size_t q = (static_cast<std::size_t>(x)*size.y()+y)*size.z()+z;
            const double expected = job.resolution * std::sqrt(static_cast<double>(x*x+y*y+z*z));
            ASSERT_EQ(expected, actual.distance_all[q]) << "threads=" << threads << " size=" << size.transpose() << " voxel=" << q;
          }
    }
  }
}

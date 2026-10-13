#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

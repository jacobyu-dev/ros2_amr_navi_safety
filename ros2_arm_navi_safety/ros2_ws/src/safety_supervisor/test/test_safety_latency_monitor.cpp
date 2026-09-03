#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "safety_supervisor/safety_latency_monitor.hpp"

namespace safety_supervisor
{
namespace
{

TEST(SafetyLatencySampleTest, CalculatesEachPipelineDuration)
{
  const SafetyLatencySample sample{7U, "lidar_timeout", 1000000000LL, 1100000000LL,
    1105000000LL, 1107000000LL, "STOP"};
  EXPECT_DOUBLE_EQ(sample.detection_latency_ms(), 100.0);
  EXPECT_DOUBLE_EQ(sample.dispatch_latency_ms(), 5.0);
  EXPECT_DOUBLE_EQ(sample.decision_latency_ms(), 2.0);
  EXPECT_DOUBLE_EQ(sample.total_latency_ms(), 107.0);
}

TEST(SafetyLatencyStatisticsTest, CalculatesSummaryAndNearestRankPercentiles)
{
  const auto statistics = calculate_latency_statistics({1.0, 2.0, 3.0, 4.0, 100.0});
  EXPECT_EQ(statistics.count, 5U);
  EXPECT_DOUBLE_EQ(statistics.min_ms, 1.0);
  EXPECT_DOUBLE_EQ(statistics.max_ms, 100.0);
  EXPECT_DOUBLE_EQ(statistics.mean_ms, 22.0);
  EXPECT_DOUBLE_EQ(statistics.median_ms, 3.0);
  EXPECT_DOUBLE_EQ(statistics.p95_ms, 100.0);
  EXPECT_DOUBLE_EQ(statistics.p99_ms, 100.0);
  EXPECT_GT(statistics.standard_deviation_ms, 0.0);
}

TEST(SafetyLatencyStatisticsTest, HandlesEmptyAndSingleSample)
{
  const auto empty = calculate_latency_statistics({});
  EXPECT_EQ(empty.count, 0U);
  EXPECT_DOUBLE_EQ(empty.mean_ms, 0.0);
  const auto one = calculate_latency_statistics({4.5});
  EXPECT_EQ(one.count, 1U);
  EXPECT_DOUBLE_EQ(one.min_ms, 4.5);
  EXPECT_DOUBLE_EQ(one.median_ms, 4.5);
  EXPECT_DOUBLE_EQ(one.p99_ms, 4.5);
  EXPECT_DOUBLE_EQ(one.standard_deviation_ms, 0.0);
}

TEST(SafetyLatencyMonitorTest, CorrelatesOnlyMatchingEventIds)
{
  SafetyLatencyMonitor monitor;
  monitor.record_detection(101U, "lidar_timeout", 1000LL, 2000LL);
  monitor.record_detection(102U, "tf_timeout", 3000LL, 5000LL);
  monitor.record_supervisor_start(102U, "tf_timeout", 3000LL, 5000LL, 5500LL);
  monitor.record_supervisor_start(101U, "lidar_timeout", 1000LL, 2000LL, 2600LL);
  ASSERT_TRUE(monitor.complete_decision(102U, 6000LL, "FAULT").has_value());
  ASSERT_TRUE(monitor.complete_decision(101U, 2800LL, "STOP").has_value());
  const auto samples = monitor.samples();
  ASSERT_EQ(samples.size(), 2U);
  EXPECT_EQ(samples[0].event_id, 102U);
  EXPECT_EQ(samples[0].fault_type, "tf_timeout");
  EXPECT_EQ(samples[1].event_id, 101U);
  EXPECT_EQ(samples[1].result_state, "STOP");
  const auto summary = monitor.summary("lidar_timeout");
  EXPECT_EQ(summary.total.count, 1U);
  EXPECT_DOUBLE_EQ(summary.detection.mean_ms, 0.001);
  EXPECT_DOUBLE_EQ(summary.dispatch.mean_ms, 0.0006);
  EXPECT_FALSE(monitor.complete_decision(101U, 3000LL, "FAULT").has_value());
}

TEST(SafetyLatencyMonitorTest, AcceptsConcurrentEventsSafely)
{
  SafetyLatencyMonitor monitor;
  constexpr std::uint64_t worker_count = 8U;
  constexpr std::uint64_t events_per_worker = 100U;
  std::vector<std::thread> workers;
  for (std::uint64_t worker = 0U; worker < worker_count; ++worker) {
    workers.emplace_back([&monitor, worker]() {
      for (std::uint64_t index = 0U; index < events_per_worker; ++index) {
        const std::uint64_t event_id = worker * events_per_worker + index + 1U;
        const auto t0 = static_cast<std::int64_t>(event_id * 1000U);
        monitor.record_supervisor_start(event_id, "lidar_timeout", t0, t0 + 100LL, t0 + 200LL);
        static_cast<void>(monitor.complete_decision(event_id, t0 + 300LL, "STOP"));
      }
    });
  }
  for (auto & worker : workers) {
    worker.join();
  }
  EXPECT_EQ(monitor.samples().size(), worker_count * events_per_worker);
}

}  // namespace
}  // namespace safety_supervisor

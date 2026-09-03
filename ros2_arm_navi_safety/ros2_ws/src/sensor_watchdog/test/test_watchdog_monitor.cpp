#include <chrono>

#include "gtest/gtest.h"
#include "sensor_watchdog/watchdog_monitor.hpp"

namespace sensor_watchdog
{
namespace
{

using namespace std::chrono_literals;
constexpr std::int64_t kNanosecondsPerMillisecond = 1000000LL;

std::int64_t ns(std::int64_t milliseconds)
{
  return milliseconds * kNanosecondsPerMillisecond;
}

WatchdogMonitor make_monitor(std::chrono::milliseconds grace = 200ms)
{
  return WatchdogMonitor({{"lidar", 500ms, true}, {"tf_monitor", 500ms, true}}, grace, 0);
}

const WatchdogEntry & source(const WatchdogEvaluation & evaluation, const std::string & source_name)
{
  for (const auto & entry : evaluation.sources) {
    if (entry.source_name == source_name) {
      return entry;
    }
  }
  throw std::out_of_range("test source missing");
}

TEST(WatchdogMonitorTest, FreshUpdatesAreHealthy)
{
  auto monitor = make_monitor();
  monitor.record_update("lidar", ns(100));
  monitor.record_update("tf_monitor", ns(100));

  const auto evaluation = monitor.evaluate(ns(400));
  EXPECT_EQ(evaluation.overall_state, WatchdogState::HEALTHY);
  EXPECT_TRUE(evaluation.all_critical_sources_healthy);
}

TEST(WatchdogMonitorTest, ElapsedAtTimeoutBoundaryIsTimeout)
{
  auto monitor = make_monitor();
  monitor.record_update("lidar", 0);
  monitor.record_update("tf_monitor", 0);

  const auto evaluation = monitor.evaluate(ns(500));
  EXPECT_EQ(source(evaluation, "lidar").state, WatchdogState::TIMEOUT);
  EXPECT_EQ(evaluation.overall_state, WatchdogState::TIMEOUT);
}

TEST(WatchdogMonitorTest, NeverReceivedWaitsThenTimesOutAfterGracePeriod)
{
  auto monitor = make_monitor();

  EXPECT_EQ(monitor.evaluate(ns(199)).overall_state, WatchdogState::WAITING);
  const auto timed_out = monitor.evaluate(ns(200));
  EXPECT_EQ(source(timed_out, "lidar").state, WatchdogState::TIMEOUT);
  EXPECT_EQ(timed_out.overall_state, WatchdogState::TIMEOUT);
}

TEST(WatchdogMonitorTest, FreshUpdateRecoversFromTimeout)
{
  auto monitor = make_monitor();
  monitor.record_update("lidar", 0);
  monitor.record_update("tf_monitor", 0);
  EXPECT_EQ(monitor.evaluate(ns(500)).overall_state, WatchdogState::TIMEOUT);

  monitor.record_update("lidar", ns(600));
  monitor.record_update("tf_monitor", ns(600));
  EXPECT_EQ(monitor.evaluate(ns(601)).overall_state, WatchdogState::HEALTHY);
}

TEST(WatchdogMonitorTest, OneTimedOutCriticalSensorMakesOverallUnhealthy)
{
  auto monitor = make_monitor();
  monitor.record_update("lidar", ns(400));
  monitor.record_update("tf_monitor", 0);

  const auto evaluation = monitor.evaluate(ns(500));
  EXPECT_EQ(source(evaluation, "lidar").state, WatchdogState::HEALTHY);
  EXPECT_EQ(source(evaluation, "tf_monitor").state, WatchdogState::TIMEOUT);
  EXPECT_FALSE(evaluation.all_critical_sources_healthy);
  EXPECT_EQ(evaluation.overall_state, WatchdogState::TIMEOUT);
}

}  // namespace
}  // namespace sensor_watchdog

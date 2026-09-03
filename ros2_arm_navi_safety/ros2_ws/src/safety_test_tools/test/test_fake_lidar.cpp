#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

#include "gtest/gtest.h"
#include "safety_test_tools/fake_lidar.hpp"

namespace safety_test_tools
{
namespace
{

builtin_interfaces::msg::Time time_from_ns(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return time;
}

TEST(FakeLidarMessageFactoryTest, SupportsEveryDefinedFaultMode)
{
  FaultMode mode{};
  EXPECT_TRUE(fault_mode_from_u8(0U, mode));
  EXPECT_EQ(mode, FaultMode::NORMAL);
  EXPECT_TRUE(fault_mode_from_u8(1U, mode));
  EXPECT_EQ(mode, FaultMode::STOP_PUBLISH);
  EXPECT_TRUE(fault_mode_from_u8(2U, mode));
  EXPECT_EQ(mode, FaultMode::SLOW_PUBLISH);
  EXPECT_TRUE(fault_mode_from_u8(3U, mode));
  EXPECT_EQ(mode, FaultMode::INVALID_DATA);
  EXPECT_TRUE(fault_mode_from_u8(4U, mode));
  EXPECT_EQ(mode, FaultMode::STALE_TIMESTAMP);
  EXPECT_FALSE(fault_mode_from_u8(255U, mode));
}

TEST(FakeLidarMessageFactoryTest, NormalScanContainsClearValidMeasurements)
{
  FakeLidarMessageFactory factory{FakeLidarConfig{}};
  const auto scan = factory.make_scan(FaultMode::NORMAL, time_from_ns(5000000000LL));

  ASSERT_EQ(scan.ranges.size(), 7U);
  EXPECT_EQ(scan.header.stamp.sec, 5);
  for (const auto range : scan.ranges) {
    EXPECT_TRUE(std::isfinite(range));
    EXPECT_GT(range, scan.range_min);
    EXPECT_LT(range, scan.range_max);
  }
}

TEST(FakeLidarMessageFactoryTest, InvalidScanContainsNoValidMeasurements)
{
  FakeLidarMessageFactory factory{FakeLidarConfig{}};
  const auto scan = factory.make_scan(FaultMode::INVALID_DATA, time_from_ns(5000000000LL));

  ASSERT_FALSE(scan.ranges.empty());
  for (const auto range : scan.ranges) {
    EXPECT_TRUE(std::isnan(range));
  }
}

TEST(FakeLidarMessageFactoryTest, StaleScanKeepsDataButBackdatesHeader)
{
  FakeLidarConfig config;
  config.stale_timestamp_offset = std::chrono::milliseconds{1500};
  FakeLidarMessageFactory factory{config};
  const auto scan = factory.make_scan(FaultMode::STALE_TIMESTAMP, time_from_ns(5000000000LL));

  EXPECT_EQ(scan.header.stamp.sec, 3);
  EXPECT_EQ(scan.header.stamp.nanosec, 500000000U);
  EXPECT_TRUE(std::isfinite(scan.ranges.front()));
}

}  // namespace
}  // namespace safety_test_tools

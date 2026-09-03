#include "safety_test_tools/fake_lidar.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace safety_test_tools
{
namespace
{
constexpr float kHalfPi = 1.57079632679489661923F;
constexpr float kPi = 3.14159265358979323846F;
}  // namespace

bool fault_mode_from_u8(std::uint8_t value, FaultMode & mode) noexcept
{
  switch (value) {
    case static_cast<std::uint8_t>(FaultMode::NORMAL):
      mode = FaultMode::NORMAL;
      return true;
    case static_cast<std::uint8_t>(FaultMode::STOP_PUBLISH):
      mode = FaultMode::STOP_PUBLISH;
      return true;
    case static_cast<std::uint8_t>(FaultMode::SLOW_PUBLISH):
      mode = FaultMode::SLOW_PUBLISH;
      return true;
    case static_cast<std::uint8_t>(FaultMode::INVALID_DATA):
      mode = FaultMode::INVALID_DATA;
      return true;
    case static_cast<std::uint8_t>(FaultMode::STALE_TIMESTAMP):
      mode = FaultMode::STALE_TIMESTAMP;
      return true;
    default:
      return false;
  }
}

const char * to_string(FaultMode mode) noexcept
{
  switch (mode) {
    case FaultMode::NORMAL:
      return "NORMAL";
    case FaultMode::STOP_PUBLISH:
      return "STOP_PUBLISH";
    case FaultMode::SLOW_PUBLISH:
      return "SLOW_PUBLISH";
    case FaultMode::INVALID_DATA:
      return "INVALID_DATA";
    case FaultMode::STALE_TIMESTAMP:
      return "STALE_TIMESTAMP";
  }
  return "UNKNOWN";
}

FakeLidarMessageFactory::FakeLidarMessageFactory(FakeLidarConfig config)
: config_(config)
{
  if (config_.normal_range_m <= config_.range_min_m ||
    config_.normal_range_m >= config_.range_max_m ||
    config_.range_min_m < 0.0F || config_.stale_timestamp_offset.count() <= 0)
  {
    throw std::invalid_argument("fake LiDAR ranges and stale_timestamp_offset must be valid");
  }
}

sensor_msgs::msg::LaserScan FakeLidarMessageFactory::make_scan(
  FaultMode mode, const builtin_interfaces::msg::Time & now) const
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = now;
  scan.header.frame_id = "laser_frame";
  scan.angle_min = -kHalfPi;
  scan.angle_max = kHalfPi;
  scan.angle_increment = kPi / 6.0F;
  scan.range_min = config_.range_min_m;
  scan.range_max = config_.range_max_m;
  scan.ranges.assign(7U, config_.normal_range_m);

  if (mode == FaultMode::INVALID_DATA) {
    scan.ranges.assign(7U, std::numeric_limits<float>::quiet_NaN());
  } else if (mode == FaultMode::STALE_TIMESTAMP) {
    const std::int64_t now_ns = static_cast<std::int64_t>(now.sec) * 1000000000LL + now.nanosec;
    const std::int64_t stale_ns = now_ns -
      std::chrono::duration_cast<std::chrono::nanoseconds>(config_.stale_timestamp_offset).count();
    const std::int64_t safe_ns = stale_ns > 0 ? stale_ns : 0;
    scan.header.stamp.sec = static_cast<std::int32_t>(safe_ns / 1000000000LL);
    scan.header.stamp.nanosec = static_cast<std::uint32_t>(safe_ns % 1000000000LL);
  }
  return scan;
}

}  // namespace safety_test_tools

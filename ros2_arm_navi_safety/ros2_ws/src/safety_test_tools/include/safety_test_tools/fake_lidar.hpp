#ifndef SAFETY_TEST_TOOLS__FAKE_LIDAR_HPP_
#define SAFETY_TEST_TOOLS__FAKE_LIDAR_HPP_

#include <chrono>
#include <cstdint>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace safety_test_tools
{

enum class FaultMode : std::uint8_t
{
  NORMAL = 0,
  STOP_PUBLISH = 1,
  SLOW_PUBLISH = 2,
  INVALID_DATA = 3,
  STALE_TIMESTAMP = 4
};

struct FakeLidarConfig
{
  float normal_range_m{2.0F};
  float range_min_m{0.05F};
  float range_max_m{10.0F};
  std::chrono::milliseconds stale_timestamp_offset{1000};
};

/// Convert a service value to a fault mode without accepting undefined modes.
[[nodiscard]] bool fault_mode_from_u8(std::uint8_t value, FaultMode & mode) noexcept;
[[nodiscard]] const char * to_string(FaultMode mode) noexcept;

/**
 * @brief Creates deterministic scans independently of ROS timers and services.
 *
 * The node owns timing and publication.  Keeping message generation here makes
 * the fault-data contract unit-testable without starting a ROS graph.
 */
class FakeLidarMessageFactory
{
public:
  explicit FakeLidarMessageFactory(FakeLidarConfig config);

  [[nodiscard]] sensor_msgs::msg::LaserScan make_scan(
    FaultMode mode, const builtin_interfaces::msg::Time & now) const;

private:
  FakeLidarConfig config_;
};

}  // namespace safety_test_tools

#endif  // SAFETY_TEST_TOOLS__FAKE_LIDAR_HPP_

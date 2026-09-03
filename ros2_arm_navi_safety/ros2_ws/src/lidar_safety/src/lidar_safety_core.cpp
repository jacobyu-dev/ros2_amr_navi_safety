#include "lidar_safety/lidar_safety_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lidar_safety
{
namespace
{
constexpr float kDegreesToRadians = 0.01745329251994329577F;
constexpr float kAngleToleranceRad = 1.0e-5F;
}  // namespace

LidarSafetyResult LidarSafetyCore::evaluate(
  const LidarScanData & scan,
  const LidarSafetyConfig & config) const
{
  if (config.stop_distance_m < 0.0F) {
    throw std::invalid_argument("stop_distance_m must be non-negative");
  }
  if (config.front_angle_deg <= 0.0F || config.front_angle_deg > 180.0F) {
    throw std::invalid_argument("front_angle_deg must be in (0, 180]");
  }

  LidarSafetyResult result{};
  result.min_distance_m = std::numeric_limits<float>::infinity();

  const float roi_half_angle_rad = config.front_angle_deg * kDegreesToRadians;
  const float scan_lower_angle = std::min(scan.angle_min_rad, scan.angle_max_rad);
  const float scan_upper_angle = std::max(scan.angle_min_rad, scan.angle_max_rad);

  for (std::size_t index = 0U; index < scan.ranges_m.size(); ++index) {
    const float angle_rad = scan.angle_min_rad +
      (static_cast<float>(index) * scan.angle_increment_rad);

    // angle_max is checked as well as calculated angles, so malformed scan data
    // cannot make an out-of-coverage point participate in the decision.
    if (angle_rad < scan_lower_angle - kAngleToleranceRad ||
      angle_rad > scan_upper_angle + kAngleToleranceRad ||
      std::fabs(angle_rad) > roi_half_angle_rad + kAngleToleranceRad)
    {
      continue;
    }

    const float range_m = scan.ranges_m[index];
    if (!std::isfinite(range_m) || range_m < scan.range_min_m || range_m > scan.range_max_m) {
      continue;
    }

    result.has_valid_measurement = true;
    result.min_distance_m = std::min(result.min_distance_m, range_m);
  }

  result.obstacle_detected = result.has_valid_measurement &&
    result.min_distance_m <= config.stop_distance_m;
  return result;
}

}  // namespace lidar_safety

#ifndef LIDAR_SAFETY__LIDAR_SAFETY_CORE_HPP_
#define LIDAR_SAFETY__LIDAR_SAFETY_CORE_HPP_

#include <vector>

namespace lidar_safety
{

struct LidarScanData
{
  float angle_min_rad{};
  float angle_max_rad{};
  float angle_increment_rad{};
  float range_min_m{};
  float range_max_m{};
  std::vector<float> ranges_m{};
};

struct LidarSafetyConfig
{
  float stop_distance_m{0.7F};
  float front_angle_deg{30.0F};
};

struct LidarSafetyResult
{
  bool obstacle_detected{false};
  bool has_valid_measurement{false};
  float min_distance_m{};
};

// Pure C++ decision logic. If there is no valid point in the ROI, this returns
// has_valid_measurement=false, obstacle_detected=false, and min_distance_m=+infinity.
class LidarSafetyCore
{
public:
  [[nodiscard]] LidarSafetyResult evaluate(
    const LidarScanData & scan,
    const LidarSafetyConfig & config) const;
};

}  // namespace lidar_safety

#endif  // LIDAR_SAFETY__LIDAR_SAFETY_CORE_HPP_

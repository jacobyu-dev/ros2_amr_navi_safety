#include "lidar_safety/lidar_safety_core.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace lidar_safety
{
namespace
{
constexpr float kPi = 3.14159265358979323846F;

LidarScanData make_scan(std::vector<float> ranges)
{
  return LidarScanData{
    -kPi / 2.0F, kPi / 2.0F, kPi / 6.0F, 0.05F, 10.0F, std::move(ranges)};
}

LidarSafetyConfig default_config()
{
  return LidarSafetyConfig{0.7F, 30.0F};
}

TEST(LidarSafetyCoreTest, NoObstacleInFrontRoi)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, 3.0F, 4.0F, 5.0F, 5.0F}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_TRUE(result.has_valid_measurement);
  EXPECT_FLOAT_EQ(result.min_distance_m, 2.0F);
}

TEST(LidarSafetyCoreTest, DetectsCloseObstacle)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, 0.5F, 3.0F, 5.0F, 5.0F}), default_config());
  EXPECT_TRUE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 0.5F);
}

TEST(LidarSafetyCoreTest, IgnoresObstacleOutsideRoi)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, 2.0F, 2.0F, 0.2F, 5.0F}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 2.0F);
}

TEST(LidarSafetyCoreTest, IgnoresNan)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, std::numeric_limits<float>::quiet_NaN(), 3.0F, 5.0F, 5.0F}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 2.0F);
}

TEST(LidarSafetyCoreTest, IgnoresInfinity)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, std::numeric_limits<float>::infinity(), 3.0F, 5.0F, 5.0F}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 2.0F);
}

TEST(LidarSafetyCoreTest, IgnoresBelowRangeMinimum)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, 0.04F, 3.0F, 5.0F, 5.0F}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 2.0F);
}

TEST(LidarSafetyCoreTest, IgnoresAboveRangeMaximum)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, 10.01F, 3.0F, 5.0F, 5.0F}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 2.0F);
}

TEST(LidarSafetyCoreTest, HandlesNoValidPoint)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const auto result = LidarSafetyCore{}.evaluate(make_scan({nan, nan, nan, nan, nan, nan, nan}), default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_FALSE(result.has_valid_measurement);
  EXPECT_TRUE(std::isinf(result.min_distance_m));
}

TEST(LidarSafetyCoreTest, PositiveInfinityMeansClearRay)
{
  const float infinity = std::numeric_limits<float>::infinity();
  const auto result = LidarSafetyCore{}.evaluate(
    make_scan({infinity, infinity, infinity, infinity, infinity, infinity, infinity}),
    default_config());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_TRUE(result.has_valid_measurement);
  EXPECT_TRUE(std::isinf(result.min_distance_m));
}

TEST(LidarSafetyCoreTest, TreatsStopDistanceAsObstacle)
{
  const auto result = LidarSafetyCore{}.evaluate(make_scan({5.0F, 5.0F, 2.0F, 0.7F, 3.0F, 5.0F, 5.0F}), default_config());
  EXPECT_TRUE(result.obstacle_detected);
  EXPECT_FLOAT_EQ(result.min_distance_m, 0.7F);
}

}  // namespace
}  // namespace lidar_safety

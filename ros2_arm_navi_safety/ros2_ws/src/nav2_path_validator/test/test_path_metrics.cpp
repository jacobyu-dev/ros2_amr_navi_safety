#include "nav2_path_validator/path_metrics.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"

namespace nav2_path_validator
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(PathMetricsTest, CalculatesPathAndTrajectoryLengthInXy)
{
  const std::vector<Point2D> points{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  EXPECT_DOUBLE_EQ(pathLength(points), 7.0);
}

TEST(PathMetricsTest, EmptyAndSinglePosePathsHaveZeroLength)
{
  EXPECT_DOUBLE_EQ(pathLength({}), 0.0);
  EXPECT_DOUBLE_EQ(pathLength({{2.0, 3.0}}), 0.0);
}

TEST(PathMetricsTest, CalculatesPointToSegmentUsingProjection)
{
  EXPECT_DOUBLE_EQ(pointToSegmentDistance({2.0, 3.0}, {0.0, 0.0}, {4.0, 0.0}), 3.0);
  EXPECT_DOUBLE_EQ(pointToSegmentDistance({-2.0, 0.0}, {0.0, 0.0}, {4.0, 0.0}), 2.0);
  EXPECT_DOUBLE_EQ(pointToSegmentDistance({1.0, 1.0}, {0.0, 0.0}, {0.0, 0.0}), std::sqrt(2.0));
}

TEST(PathMetricsTest, CrossTrackErrorUsesSegmentsRatherThanVertices)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<Point2D> trajectory{{2.0, 0.1}, {5.0, 0.2}, {8.0, 0.3}};
  const auto errors = crossTrackErrors(trajectory, path);
  ASSERT_TRUE(errors.valid);
  EXPECT_NEAR(errors.mean, 0.2, 1.0e-12);
  EXPECT_NEAR(errors.rmse, std::sqrt((0.01 + 0.04 + 0.09) / 3.0), 1.0e-12);
  EXPECT_NEAR(errors.maximum, 0.3, 1.0e-12);
}

TEST(PathMetricsTest, EmptyInputsAreHandledWithoutInventingMetrics)
{
  EXPECT_FALSE(errorStatistics({}).valid);
  EXPECT_FALSE(crossTrackErrors({{0.0, 0.0}}, {}).valid);
  EXPECT_TRUE(std::isinf(pointToPathDistance({0.0, 0.0}, {})));
  EXPECT_DOUBLE_EQ(pointToPathDistance({3.0, 4.0}, {{0.0, 0.0}}), 5.0);
}

TEST(PathMetricsTest, CalculatesGoalPositionError)
{
  EXPECT_DOUBLE_EQ(distance({0.0, 0.0}, {3.0, 4.0}), 5.0);
}

TEST(PathMetricsTest, NormalizesAnglesAndCalculatesShortestYawError)
{
  EXPECT_NEAR(normalizeAngle(3.0 * kPi), kPi, 1.0e-12);
  EXPECT_NEAR(yawError(179.0 * kPi / 180.0, -179.0 * kPi / 180.0),
    2.0 * kPi / 180.0, 1.0e-12);
}

TEST(PathMetricsTest, ReplanMetricIgnoresRemovedPrefixOfSameRoute)
{
  const std::vector<Point2D> original{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};
  const std::vector<Point2D> remaining{{4.0, 0.0}, {7.0, 0.0}, {10.0, 0.0}};
  EXPECT_NEAR(pathGeometryDeviation(original, remaining), 0.0, 1.0e-12);
}

TEST(PathMetricsTest, ReplanMetricDetectsMeaningfulGeometryChange)
{
  const std::vector<Point2D> original{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<Point2D> detour{{0.0, 0.0}, {5.0, 2.0}, {10.0, 0.0}};
  EXPECT_GT(pathGeometryDeviation(original, detour), 1.9);
}

}  // namespace
}  // namespace nav2_path_validator

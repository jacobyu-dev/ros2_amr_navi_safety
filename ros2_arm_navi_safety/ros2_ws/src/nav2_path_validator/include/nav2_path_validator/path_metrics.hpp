#ifndef NAV2_PATH_VALIDATOR__PATH_METRICS_HPP_
#define NAV2_PATH_VALIDATOR__PATH_METRICS_HPP_

#include <cstddef>
#include <vector>

namespace nav2_path_validator
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct ErrorStatistics
{
  bool valid{false};
  double mean{0.0};
  double rmse{0.0};
  double maximum{0.0};
};

double distance(Point2D first, Point2D second) noexcept;
double pathLength(const std::vector<Point2D> & path) noexcept;
double pointToSegmentDistance(Point2D point, Point2D start, Point2D end) noexcept;
double pointToPathDistance(Point2D point, const std::vector<Point2D> & path) noexcept;
ErrorStatistics errorStatistics(const std::vector<double> & errors) noexcept;
ErrorStatistics crossTrackErrors(
  const std::vector<Point2D> & trajectory, const std::vector<Point2D> & path) noexcept;
double normalizeAngle(double angle) noexcept;
double yawError(double actual, double goal) noexcept;

// Maximum directed distance from samples on candidate to reference. This
// intentionally ignores a removed prefix when a planner republishes the same
// route from the robot's newer position.
double pathGeometryDeviation(
  const std::vector<Point2D> & reference, const std::vector<Point2D> & candidate,
  std::size_t sample_count = 100U) noexcept;

}  // namespace nav2_path_validator

#endif  // NAV2_PATH_VALIDATOR__PATH_METRICS_HPP_

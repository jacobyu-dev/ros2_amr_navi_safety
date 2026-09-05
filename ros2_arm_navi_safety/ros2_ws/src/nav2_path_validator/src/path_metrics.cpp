#include "nav2_path_validator/path_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav2_path_validator
{
namespace
{

std::vector<Point2D> samplePath(const std::vector<Point2D> & path, std::size_t count)
{
  if (path.empty() || count == 0U) {
    return {};
  }
  if (path.size() == 1U || count == 1U) {
    return {path.front()};
  }
  const double length = pathLength(path);
  if (length <= std::numeric_limits<double>::epsilon()) {
    return std::vector<Point2D>(count, path.front());
  }

  std::vector<double> cumulative(path.size(), 0.0);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    cumulative[index] = cumulative[index - 1U] + distance(path[index - 1U], path[index]);
  }

  std::vector<Point2D> result;
  result.reserve(count);
  std::size_t segment = 1U;
  for (std::size_t sample = 0U; sample < count; ++sample) {
    const double target = length * static_cast<double>(sample) / static_cast<double>(count - 1U);
    while (segment + 1U < cumulative.size() && cumulative[segment] < target) {
      ++segment;
    }
    const double segment_length = cumulative[segment] - cumulative[segment - 1U];
    const double ratio = segment_length <= std::numeric_limits<double>::epsilon() ? 0.0 :
      (target - cumulative[segment - 1U]) / segment_length;
    result.push_back(Point2D{
          path[segment - 1U].x + ratio * (path[segment].x - path[segment - 1U].x),
          path[segment - 1U].y + ratio * (path[segment].y - path[segment - 1U].y)});
  }
  return result;
}

}  // namespace

double distance(Point2D first, Point2D second) noexcept
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

double pathLength(const std::vector<Point2D> & path) noexcept
{
  double result = 0.0;
  for (std::size_t index = 1U; index < path.size(); ++index) {
    result += distance(path[index - 1U], path[index]);
  }
  return result;
}

double pointToSegmentDistance(Point2D point, Point2D start, Point2D end) noexcept
{
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return distance(point, start);
  }
  const double projection = std::clamp(
    ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared, 0.0, 1.0);
  return distance(point, Point2D{start.x + projection * dx, start.y + projection * dy});
}

double pointToPathDistance(Point2D point, const std::vector<Point2D> & path) noexcept
{
  if (path.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (path.size() == 1U) {
    return distance(point, path.front());
  }
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1U; index < path.size(); ++index) {
    closest = std::min(closest, pointToSegmentDistance(point, path[index - 1U], path[index]));
  }
  return closest;
}

ErrorStatistics errorStatistics(const std::vector<double> & errors) noexcept
{
  if (errors.empty()) {
    return {};
  }
  double sum = 0.0;
  double sum_squared = 0.0;
  double maximum = 0.0;
  for (const double error : errors) {
    sum += error;
    sum_squared += error * error;
    maximum = std::max(maximum, error);
  }
  const double count = static_cast<double>(errors.size());
  return ErrorStatistics{true, sum / count, std::sqrt(sum_squared / count), maximum};
}

ErrorStatistics crossTrackErrors(
  const std::vector<Point2D> & trajectory, const std::vector<Point2D> & path) noexcept
{
  if (path.empty()) {
    return {};
  }
  std::vector<double> errors;
  errors.reserve(trajectory.size());
  for (const auto & point : trajectory) {
    errors.push_back(pointToPathDistance(point, path));
  }
  return errorStatistics(errors);
}

double normalizeAngle(double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawError(double actual, double goal) noexcept
{
  return std::abs(normalizeAngle(actual - goal));
}

double pathGeometryDeviation(
  const std::vector<Point2D> & reference, const std::vector<Point2D> & candidate,
  std::size_t sample_count) noexcept
{
  if (reference.empty() || candidate.empty() || sample_count == 0U) {
    return std::numeric_limits<double>::infinity();
  }
  double maximum = 0.0;
  for (const auto & point : samplePath(candidate, sample_count)) {
    maximum = std::max(maximum, pointToPathDistance(point, reference));
  }
  return maximum;
}

}  // namespace nav2_path_validator

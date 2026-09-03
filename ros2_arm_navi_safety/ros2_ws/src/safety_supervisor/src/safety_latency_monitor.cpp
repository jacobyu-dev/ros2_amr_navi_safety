#include "safety_supervisor/safety_latency_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace safety_supervisor
{
namespace
{
constexpr double kNanosecondsPerMillisecond = 1000000.0;

double elapsed_ms(std::int64_t end_ns, std::int64_t begin_ns) noexcept
{
  return end_ns >= begin_ns ? static_cast<double>(end_ns - begin_ns) / kNanosecondsPerMillisecond : 0.0;
}

double nearest_rank(const std::vector<double> & sorted, double percentile)
{
  if (sorted.empty()) {
    return 0.0;
  }
  const auto rank = static_cast<std::size_t>(std::ceil(percentile * sorted.size()));
  return sorted.at(std::max<std::size_t>(1U, rank) - 1U);
}
}  // namespace

double SafetyLatencySample::detection_latency_ms() const noexcept
{
  return elapsed_ms(detection_time_steady_ns, fault_time_steady_ns);
}

double SafetyLatencySample::dispatch_latency_ms() const noexcept
{
  return elapsed_ms(supervisor_start_time_steady_ns, detection_time_steady_ns);
}

double SafetyLatencySample::decision_latency_ms() const noexcept
{
  return elapsed_ms(decision_time_steady_ns, supervisor_start_time_steady_ns);
}

double SafetyLatencySample::total_latency_ms() const noexcept
{
  return elapsed_ms(decision_time_steady_ns, fault_time_steady_ns);
}

LatencyStatistics calculate_latency_statistics(const std::vector<double> & samples)
{
  LatencyStatistics result;
  if (samples.empty()) {
    return result;
  }
  std::vector<double> sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  result.count = sorted.size();
  result.min_ms = sorted.front();
  result.max_ms = sorted.back();
  const double sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
  result.mean_ms = sum / static_cast<double>(sorted.size());
  const std::size_t middle = sorted.size() / 2U;
  result.median_ms = sorted.size() % 2U == 0U ?
    (sorted[middle - 1U] + sorted[middle]) / 2.0 : sorted[middle];
  result.p95_ms = nearest_rank(sorted, 0.95);
  result.p99_ms = nearest_rank(sorted, 0.99);
  double squared_difference_sum = 0.0;
  for (const double value : sorted) {
    const double difference = value - result.mean_ms;
    squared_difference_sum += difference * difference;
  }
  result.standard_deviation_ms = std::sqrt(squared_difference_sum / static_cast<double>(sorted.size()));
  return result;
}

void SafetyLatencyMonitor::record_detection(
  std::uint64_t event_id, std::string fault_type, std::int64_t fault_time_steady_ns,
  std::int64_t detection_time_steady_ns)
{
  if (event_id == 0U || fault_time_steady_ns <= 0 || detection_time_steady_ns <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (completed_by_event_.count(event_id) != 0U) {
    return;
  }
  auto & pending = pending_[event_id];
  pending.sample.event_id = event_id;
  pending.sample.fault_type = std::move(fault_type);
  pending.sample.fault_time_steady_ns = fault_time_steady_ns;
  pending.sample.detection_time_steady_ns = detection_time_steady_ns;
  pending.has_detection = true;
}

void SafetyLatencyMonitor::record_supervisor_start(
  std::uint64_t event_id, std::string fault_type, std::int64_t fault_time_steady_ns,
  std::int64_t detection_time_steady_ns, std::int64_t supervisor_start_time_steady_ns)
{
  record_detection(event_id, std::move(fault_type), fault_time_steady_ns, detection_time_steady_ns);
  if (event_id == 0U || supervisor_start_time_steady_ns <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto pending = pending_.find(event_id);
  if (pending == pending_.end() || completed_by_event_.count(event_id) != 0U) {
    return;
  }
  pending->second.sample.supervisor_start_time_steady_ns = supervisor_start_time_steady_ns;
  pending->second.has_supervisor_start = true;
}

std::optional<SafetyLatencySample> SafetyLatencyMonitor::complete_decision(
  std::uint64_t event_id, std::int64_t decision_time_steady_ns, std::string result_state)
{
  if (event_id == 0U || decision_time_steady_ns <= 0) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (completed_by_event_.count(event_id) != 0U) {
    return std::nullopt;
  }
  const auto pending = pending_.find(event_id);
  if (pending == pending_.end() || !pending->second.has_detection || !pending->second.has_supervisor_start) {
    return std::nullopt;
  }
  SafetyLatencySample sample = pending->second.sample;
  sample.decision_time_steady_ns = decision_time_steady_ns;
  sample.result_state = std::move(result_state);
  completed_by_event_.emplace(event_id, sample);
  completed_.push_back(sample);
  pending_.erase(pending);
  return sample;
}

std::vector<SafetyLatencySample> SafetyLatencyMonitor::samples() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return completed_;
}

std::vector<SafetyLatencySample> SafetyLatencyMonitor::samples_for(const std::string & fault_type) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<SafetyLatencySample> result;
  for (const auto & sample : completed_) {
    if (sample.fault_type == fault_type) {
      result.push_back(sample);
    }
  }
  return result;
}

LatencyStatistics SafetyLatencyMonitor::total_statistics(const std::string & fault_type) const
{
  const auto selected = fault_type.empty() ? samples() : samples_for(fault_type);
  std::vector<double> totals;
  totals.reserve(selected.size());
  for (const auto & sample : selected) {
    totals.push_back(sample.total_latency_ms());
  }
  return calculate_latency_statistics(totals);
}

SafetyLatencySummary SafetyLatencyMonitor::summary(const std::string & fault_type) const
{
  const auto selected = fault_type.empty() ? samples() : samples_for(fault_type);
  std::vector<double> detection;
  std::vector<double> dispatch;
  std::vector<double> decision;
  std::vector<double> total;
  detection.reserve(selected.size());
  dispatch.reserve(selected.size());
  decision.reserve(selected.size());
  total.reserve(selected.size());
  for (const auto & sample : selected) {
    detection.push_back(sample.detection_latency_ms());
    dispatch.push_back(sample.dispatch_latency_ms());
    decision.push_back(sample.decision_latency_ms());
    total.push_back(sample.total_latency_ms());
  }
  return SafetyLatencySummary{
    calculate_latency_statistics(detection), calculate_latency_statistics(dispatch),
    calculate_latency_statistics(decision), calculate_latency_statistics(total)};
}

void SafetyLatencyMonitor::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.clear();
  completed_by_event_.clear();
  completed_.clear();
}

}  // namespace safety_supervisor

#ifndef SAFETY_SUPERVISOR__SAFETY_LATENCY_MONITOR_HPP_
#define SAFETY_SUPERVISOR__SAFETY_LATENCY_MONITOR_HPP_

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace safety_supervisor
{

/// A completed monotonic-clock measurement for one correlated safety fault.
struct SafetyLatencySample
{
  std::uint64_t event_id{0U};
  std::string fault_type;
  std::int64_t fault_time_steady_ns{0};
  std::int64_t detection_time_steady_ns{0};
  std::int64_t supervisor_start_time_steady_ns{0};
  std::int64_t decision_time_steady_ns{0};
  std::string result_state;

  [[nodiscard]] double detection_latency_ms() const noexcept;
  [[nodiscard]] double dispatch_latency_ms() const noexcept;
  [[nodiscard]] double decision_latency_ms() const noexcept;
  [[nodiscard]] double total_latency_ms() const noexcept;
};

struct LatencyStatistics
{
  std::size_t count{0U};
  double min_ms{0.0};
  double max_ms{0.0};
  double mean_ms{0.0};
  double median_ms{0.0};
  double p95_ms{0.0};
  double p99_ms{0.0};
  double standard_deviation_ms{0.0};
};

struct SafetyLatencySummary
{
  LatencyStatistics detection;
  LatencyStatistics dispatch;
  LatencyStatistics decision;
  LatencyStatistics total;
};

/// Compute descriptive statistics using the nearest-rank percentile convention.
[[nodiscard]] LatencyStatistics calculate_latency_statistics(const std::vector<double> & samples);

/**
 * @brief Thread-safe event correlation and completed-sample store.
 *
 * The monitor deliberately stores only monotonic nanoseconds supplied by the
 * caller. It neither samples ROS time nor performs ROS logging, keeping it
 * usable in deterministic unit tests and independent of ROS clock jumps.
 */
class SafetyLatencyMonitor
{
public:
  void record_detection(
    std::uint64_t event_id, std::string fault_type, std::int64_t fault_time_steady_ns,
    std::int64_t detection_time_steady_ns);
  void record_supervisor_start(
    std::uint64_t event_id, std::string fault_type, std::int64_t fault_time_steady_ns,
    std::int64_t detection_time_steady_ns, std::int64_t supervisor_start_time_steady_ns);

  /// Complete an event once. Repeated watchdog reports for an event are ignored.
  [[nodiscard]] std::optional<SafetyLatencySample> complete_decision(
    std::uint64_t event_id, std::int64_t decision_time_steady_ns, std::string result_state);

  [[nodiscard]] std::vector<SafetyLatencySample> samples() const;
  [[nodiscard]] std::vector<SafetyLatencySample> samples_for(const std::string & fault_type) const;
  [[nodiscard]] LatencyStatistics total_statistics(const std::string & fault_type = "") const;
  [[nodiscard]] SafetyLatencySummary summary(const std::string & fault_type = "") const;
  void clear();

private:
  struct PendingSample
  {
    SafetyLatencySample sample;
    bool has_detection{false};
    bool has_supervisor_start{false};
  };

  mutable std::mutex mutex_;
  std::unordered_map<std::uint64_t, PendingSample> pending_;
  std::unordered_map<std::uint64_t, SafetyLatencySample> completed_by_event_;
  std::vector<SafetyLatencySample> completed_;
};

}  // namespace safety_supervisor

#endif  // SAFETY_SUPERVISOR__SAFETY_LATENCY_MONITOR_HPP_

#ifndef SENSOR_WATCHDOG__WATCHDOG_MONITOR_HPP_
#define SENSOR_WATCHDOG__WATCHDOG_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace sensor_watchdog
{

enum class WatchdogState : std::uint8_t
{
  WAITING = 0,
  HEALTHY,
  TIMEOUT
};

struct WatchdogSourceConfig
{
  std::string source_name;
  std::chrono::milliseconds timeout{500};
  bool critical{true};
};

struct WatchdogEntry
{
  std::string source_name;
  std::int64_t last_update_time_ns{0};
  std::chrono::milliseconds timeout{500};
  bool received_once{false};
  bool critical{true};
  WatchdogState state{WatchdogState::WAITING};
};

struct WatchdogEvaluation
{
  WatchdogState overall_state{WatchdogState::WAITING};
  bool all_critical_sources_healthy{false};
  std::vector<WatchdogEntry> sources;
};

/**
 * @brief Thread-safe timeout policy independent of ROS callback plumbing.
 *
 * The caller supplies time values from its ROS clock in nanoseconds. This
 * class deliberately never samples a system or steady clock, which makes its
 * policy deterministic in unit tests and consistent with simulated ROS time.
 */
class WatchdogMonitor
{
public:
  WatchdogMonitor(
    std::vector<WatchdogSourceConfig> source_configs,
    std::chrono::milliseconds startup_grace_period,
    std::int64_t started_at_ns);

  /// Record a fresh ROS-topic update for one configured source.
  void record_update(const std::string & source_name, std::int64_t now_ns);

  /// Evaluate every source at the supplied ROS time and return a consistent snapshot.
  [[nodiscard]] WatchdogEvaluation evaluate(std::int64_t now_ns);

private:
  [[nodiscard]] static std::int64_t elapsed_ns(std::int64_t now_ns, std::int64_t then_ns) noexcept;

  const std::chrono::milliseconds startup_grace_period_;
  const std::int64_t started_at_ns_;
  std::mutex mutex_;
  std::unordered_map<std::string, WatchdogEntry> entries_;
};

[[nodiscard]] const char * to_string(WatchdogState state) noexcept;

}  // namespace sensor_watchdog

#endif  // SENSOR_WATCHDOG__WATCHDOG_MONITOR_HPP_

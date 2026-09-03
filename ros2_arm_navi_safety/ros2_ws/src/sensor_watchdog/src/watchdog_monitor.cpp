#include "sensor_watchdog/watchdog_monitor.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace sensor_watchdog
{

WatchdogMonitor::WatchdogMonitor(
  std::vector<WatchdogSourceConfig> source_configs,
  std::chrono::milliseconds startup_grace_period,
  std::int64_t started_at_ns)
: startup_grace_period_(startup_grace_period),
  started_at_ns_(started_at_ns)
{
  if (source_configs.empty()) {
    throw std::invalid_argument("at least one watchdog source is required");
  }
  if (startup_grace_period_.count() < 0) {
    throw std::invalid_argument("startup_grace_period must be non-negative");
  }
  for (const auto & config : source_configs) {
    if (config.source_name.empty() || config.timeout.count() <= 0) {
      throw std::invalid_argument("watchdog source names must be non-empty and timeouts positive");
    }
    const auto inserted = entries_.emplace(
      config.source_name,
      WatchdogEntry{config.source_name, 0, config.timeout, false, config.critical, WatchdogState::WAITING});
    if (!inserted.second) {
      throw std::invalid_argument("watchdog source names must be unique");
    }
  }
}

void WatchdogMonitor::record_update(const std::string & source_name, std::int64_t now_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & entry = entries_.at(source_name);
  entry.last_update_time_ns = now_ns;
  entry.received_once = true;
}

WatchdogEvaluation WatchdogMonitor::evaluate(std::int64_t now_ns)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto startup_elapsed = elapsed_ns(now_ns, started_at_ns_);
  const auto startup_grace_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    startup_grace_period_).count();

  WatchdogEvaluation evaluation;
  evaluation.all_critical_sources_healthy = true;
  bool critical_timeout = false;
  bool critical_waiting = false;
  evaluation.sources.reserve(entries_.size());

  for (auto & item : entries_) {
    auto & entry = item.second;
    if (!entry.received_once) {
      entry.state = startup_elapsed >= startup_grace_ns ?
        WatchdogState::TIMEOUT : WatchdogState::WAITING;
    } else {
      const auto age_ns = elapsed_ns(now_ns, entry.last_update_time_ns);
      const auto timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(entry.timeout).count();
      // The boundary is intentionally fail-safe: an age equal to the threshold is stale.
      entry.state = age_ns >= timeout_ns ? WatchdogState::TIMEOUT : WatchdogState::HEALTHY;
    }

    if (entry.critical && entry.state != WatchdogState::HEALTHY) {
      evaluation.all_critical_sources_healthy = false;
      critical_timeout = critical_timeout || entry.state == WatchdogState::TIMEOUT;
      critical_waiting = critical_waiting || entry.state == WatchdogState::WAITING;
    }
    evaluation.sources.push_back(entry);
  }

  if (critical_timeout) {
    evaluation.overall_state = WatchdogState::TIMEOUT;
  } else if (critical_waiting) {
    evaluation.overall_state = WatchdogState::WAITING;
  } else {
    evaluation.overall_state = WatchdogState::HEALTHY;
  }
  std::sort(
    evaluation.sources.begin(), evaluation.sources.end(),
    [](const WatchdogEntry & lhs, const WatchdogEntry & rhs) {
      return lhs.source_name < rhs.source_name;
    });
  return evaluation;
}

std::int64_t WatchdogMonitor::elapsed_ns(std::int64_t now_ns, std::int64_t then_ns) noexcept
{
  // A backwards ROS-time jump must not manufacture a timeout.
  return now_ns > then_ns ? now_ns - then_ns : 0;
}

const char * to_string(WatchdogState state) noexcept
{
  switch (state) {
    case WatchdogState::WAITING:
      return "WAITING";
    case WatchdogState::HEALTHY:
      return "HEALTHY";
    case WatchdogState::TIMEOUT:
      return "TIMEOUT";
  }
  return "WAITING";
}

}  // namespace sensor_watchdog

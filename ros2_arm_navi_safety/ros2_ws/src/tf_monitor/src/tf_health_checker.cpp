#include "tf_monitor/tf_health_checker.hpp"

#include <stdexcept>

namespace tf_monitor
{

TfHealthChecker::TfHealthChecker(TfMonitorConfig config)
: config_(config)
{
  validate_config(config_);
}

TfStatus TfHealthChecker::evaluate(const TfObservation & observation) const
{
  if (!observation.transform_available) {
    return TfStatus::MISSING;
  }
  if (config_.allow_static_transform && observation.is_static) {
    return TfStatus::OK;
  }
  // A small future timestamp can occur when clocks are synchronized imperfectly.
  // Treat it as a current transform rather than as a stale one.
  const auto age = observation.age.count() < 0 ? std::chrono::milliseconds{0} : observation.age;
  return age <= config_.stale_threshold ? TfStatus::OK : TfStatus::STALE;
}

const TfMonitorConfig & TfHealthChecker::config() const noexcept
{
  return config_;
}

void TfHealthChecker::validate_config(const TfMonitorConfig & config)
{
  if (config.check_period.count() <= 0) {
    throw std::invalid_argument("check_period must be positive");
  }
  if (config.lookup_timeout.count() < 0) {
    throw std::invalid_argument("lookup_timeout must be non-negative");
  }
  if (config.stale_threshold.count() < 0) {
    throw std::invalid_argument("stale_threshold must be non-negative");
  }
}

const char * to_string(TfStatus status) noexcept
{
  switch (status) {
    case TfStatus::OK:
      return "OK";
    case TfStatus::MISSING:
      return "MISSING";
    case TfStatus::TIMEOUT:
      return "TIMEOUT";
    case TfStatus::STALE:
      return "STALE";
  }
  return "UNKNOWN";
}

TfStatus status_from_lookup_failure(TfLookupFailure failure) noexcept
{
  return failure == TfLookupFailure::TIMEOUT ? TfStatus::TIMEOUT : TfStatus::MISSING;
}

}  // namespace tf_monitor

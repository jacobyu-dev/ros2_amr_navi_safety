#ifndef TF_MONITOR__TF_HEALTH_CHECKER_HPP_
#define TF_MONITOR__TF_HEALTH_CHECKER_HPP_

#include <chrono>

namespace tf_monitor
{

enum class TfStatus
{
  OK,
  MISSING,
  TIMEOUT,
  STALE
};

enum class TfLookupFailure
{
  MISSING,
  TIMEOUT
};

struct TfMonitorConfig
{
  std::chrono::milliseconds check_period{100};
  std::chrono::milliseconds lookup_timeout{100};
  std::chrono::milliseconds stale_threshold{500};
  bool allow_static_transform{true};
};

struct TfObservation
{
  bool transform_available{false};
  bool is_static{false};
  std::chrono::milliseconds age{0};
};

class TfHealthChecker
{
public:
  explicit TfHealthChecker(TfMonitorConfig config);

  [[nodiscard]] TfStatus evaluate(const TfObservation & observation) const;
  [[nodiscard]] const TfMonitorConfig & config() const noexcept;

  static void validate_config(const TfMonitorConfig & config);

private:
  TfMonitorConfig config_;
};

[[nodiscard]] const char * to_string(TfStatus status) noexcept;
[[nodiscard]] TfStatus status_from_lookup_failure(TfLookupFailure failure) noexcept;

}  // namespace tf_monitor

#endif  // TF_MONITOR__TF_HEALTH_CHECKER_HPP_

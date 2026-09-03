#ifndef SAFETY_SUPERVISOR__SAFETY_STATE_MACHINE_HPP_
#define SAFETY_SUPERVISOR__SAFETY_STATE_MACHINE_HPP_

#include <cstdint>
#include <string>

namespace safety_supervisor
{

// Independent from ROS message constants so the aggregation policy can be
// exercised in a plain C++ unit test.
enum class SafetyLevel : std::uint8_t
{
  UNKNOWN = 0,
  SAFE,
  WARNING,
  STOP,
  FAULT
};

enum class SystemSafetyState : std::uint8_t
{
  INIT = 0,
  SAFE,
  WARNING,
  STOP,
  FAULT
};

struct SafetyEvaluation
{
  bool sources_ready{false};
  SafetyLevel worst_level{SafetyLevel::UNKNOWN};
  std::string reason;
};

class SafetyStateMachine
{
public:
  [[nodiscard]] SystemSafetyState update(const SafetyEvaluation & evaluation) noexcept;
  [[nodiscard]] SystemSafetyState current_state() const noexcept;

private:
  SystemSafetyState current_state_{SystemSafetyState::INIT};
};

[[nodiscard]] SafetyLevel worst_safety_level(SafetyLevel lhs, SafetyLevel rhs) noexcept;
[[nodiscard]] const char * to_string(SafetyLevel level) noexcept;
[[nodiscard]] const char * to_string(SystemSafetyState state) noexcept;

}  // namespace safety_supervisor

#endif  // SAFETY_SUPERVISOR__SAFETY_STATE_MACHINE_HPP_

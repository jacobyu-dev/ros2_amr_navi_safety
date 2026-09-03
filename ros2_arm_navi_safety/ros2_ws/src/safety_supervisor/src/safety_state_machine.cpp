#include "safety_supervisor/safety_state_machine.hpp"

namespace safety_supervisor
{

SafetyLevel worst_safety_level(SafetyLevel lhs, SafetyLevel rhs) noexcept
{
  // UNKNOWN is not a normal operating state. Its presence is fail-safe FAULT.
  if (lhs == SafetyLevel::UNKNOWN || rhs == SafetyLevel::UNKNOWN) {
    return SafetyLevel::FAULT;
  }
  return static_cast<std::uint8_t>(lhs) >= static_cast<std::uint8_t>(rhs) ? lhs : rhs;
}

SystemSafetyState SafetyStateMachine::update(const SafetyEvaluation & evaluation) noexcept
{
  if (!evaluation.sources_ready) {
    current_state_ = SystemSafetyState::INIT;
    return current_state_;
  }

  switch (evaluation.worst_level) {
    case SafetyLevel::SAFE:
      current_state_ = SystemSafetyState::SAFE;
      break;
    case SafetyLevel::WARNING:
      current_state_ = SystemSafetyState::WARNING;
      break;
    case SafetyLevel::STOP:
      current_state_ = SystemSafetyState::STOP;
      break;
    case SafetyLevel::FAULT:
    case SafetyLevel::UNKNOWN:
      current_state_ = SystemSafetyState::FAULT;
      break;
  }
  return current_state_;
}

SystemSafetyState SafetyStateMachine::current_state() const noexcept
{
  return current_state_;
}

const char * to_string(SafetyLevel level) noexcept
{
  switch (level) {
    case SafetyLevel::UNKNOWN:
      return "UNKNOWN";
    case SafetyLevel::SAFE:
      return "SAFE";
    case SafetyLevel::WARNING:
      return "WARNING";
    case SafetyLevel::STOP:
      return "STOP";
    case SafetyLevel::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

const char * to_string(SystemSafetyState state) noexcept
{
  switch (state) {
    case SystemSafetyState::INIT:
      return "INIT";
    case SystemSafetyState::SAFE:
      return "SAFE";
    case SystemSafetyState::WARNING:
      return "WARNING";
    case SystemSafetyState::STOP:
      return "STOP";
    case SystemSafetyState::FAULT:
      return "FAULT";
  }
  return "INIT";
}

}  // namespace safety_supervisor

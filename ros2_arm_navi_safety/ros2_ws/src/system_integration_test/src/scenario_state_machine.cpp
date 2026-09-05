#include "system_integration_test/scenario_state_machine.hpp"

namespace system_integration_test
{

bool ScenarioStateMachine::handle(ScenarioEvent event) noexcept
{
  if (terminal()) {
    return false;
  }
  if (event == ScenarioEvent::STEP_TIMEOUT || event == ScenarioEvent::CRITICAL_FAILURE) {
    state_ = ScenarioState::FAILED;
    return true;
  }

  ScenarioState next = state_;
  switch (state_) {
    case ScenarioState::INITIALIZING:
      if (event == ScenarioEvent::SYSTEM_READY) {next = ScenarioState::WAITING_FOR_NAVIGATION;}
      break;
    case ScenarioState::WAITING_FOR_NAVIGATION:
      if (event == ScenarioEvent::MISSION_ACCEPTED) {next = ScenarioState::NORMAL_NAVIGATION;}
      break;
    case ScenarioState::NORMAL_NAVIGATION:
      if (event == ScenarioEvent::OBSTACLE_INJECTED) {next = ScenarioState::WAITING_FOR_OBSTACLE_STOP;}
      break;
    case ScenarioState::WAITING_FOR_OBSTACLE_STOP:
      if (event == ScenarioEvent::OBSTACLE_STOP_CONFIRMED) {
        next = ScenarioState::WAITING_FOR_OBSTACLE_RECOVERY;
      }
      break;
    case ScenarioState::WAITING_FOR_OBSTACLE_RECOVERY:
      if (event == ScenarioEvent::OBSTACLE_REMOVED) {
        // Removal is an observed action, not proof that recovery is stable.
        return true;
      }
      if (event == ScenarioEvent::OBSTACLE_RECOVERED) {
        next = ScenarioState::WAITING_FOR_OBSTACLE_RESUME;
      }
      break;
    case ScenarioState::WAITING_FOR_OBSTACLE_RESUME:
      if (event == ScenarioEvent::NAVIGATION_RESUMED) {
        next = ScenarioState::WAITING_FOR_SENSOR_STOP;
      }
      break;
    case ScenarioState::WAITING_FOR_SENSOR_STOP:
      if (event == ScenarioEvent::SENSOR_FAULT_INJECTED) {return true;}
      if (event == ScenarioEvent::SENSOR_STOP_CONFIRMED) {
        next = ScenarioState::WAITING_FOR_SENSOR_RECOVERY;
      }
      break;
    case ScenarioState::WAITING_FOR_SENSOR_RECOVERY:
      if (event == ScenarioEvent::SENSOR_RESTORED) {return true;}
      if (event == ScenarioEvent::SENSOR_RECOVERED) {
        next = ScenarioState::WAITING_FOR_SENSOR_RESUME;
      }
      break;
    case ScenarioState::WAITING_FOR_SENSOR_RESUME:
      if (event == ScenarioEvent::NAVIGATION_RESUMED) {next = ScenarioState::FINAL_NAVIGATION;}
      break;
    case ScenarioState::FINAL_NAVIGATION:
      if (event == ScenarioEvent::GOAL_REACHED) {next = ScenarioState::PASSED;}
      break;
    case ScenarioState::PASSED:
    case ScenarioState::FAILED:
      return false;
  }
  if (next == state_) {
    return false;
  }
  state_ = next;
  return true;
}

ScenarioState ScenarioStateMachine::state() const noexcept {return state_;}
bool ScenarioStateMachine::terminal() const noexcept
{
  return state_ == ScenarioState::PASSED || state_ == ScenarioState::FAILED;
}
bool ScenarioStateMachine::passed() const noexcept {return state_ == ScenarioState::PASSED;}

const char * to_string(ScenarioState state) noexcept
{
  switch (state) {
    case ScenarioState::INITIALIZING: return "INITIALIZING";
    case ScenarioState::WAITING_FOR_NAVIGATION: return "WAITING_FOR_NAVIGATION";
    case ScenarioState::NORMAL_NAVIGATION: return "NORMAL_NAVIGATION";
    case ScenarioState::WAITING_FOR_OBSTACLE_STOP: return "WAITING_FOR_OBSTACLE_STOP";
    case ScenarioState::WAITING_FOR_OBSTACLE_RECOVERY: return "WAITING_FOR_OBSTACLE_RECOVERY";
    case ScenarioState::WAITING_FOR_OBSTACLE_RESUME: return "WAITING_FOR_OBSTACLE_RESUME";
    case ScenarioState::WAITING_FOR_SENSOR_STOP: return "WAITING_FOR_SENSOR_STOP";
    case ScenarioState::WAITING_FOR_SENSOR_RECOVERY: return "WAITING_FOR_SENSOR_RECOVERY";
    case ScenarioState::WAITING_FOR_SENSOR_RESUME: return "WAITING_FOR_SENSOR_RESUME";
    case ScenarioState::FINAL_NAVIGATION: return "FINAL_NAVIGATION";
    case ScenarioState::PASSED: return "PASSED";
    case ScenarioState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace system_integration_test

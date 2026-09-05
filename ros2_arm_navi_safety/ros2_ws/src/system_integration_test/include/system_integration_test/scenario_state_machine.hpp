#ifndef SYSTEM_INTEGRATION_TEST__SCENARIO_STATE_MACHINE_HPP_
#define SYSTEM_INTEGRATION_TEST__SCENARIO_STATE_MACHINE_HPP_

#include <cstdint>

namespace system_integration_test
{

enum class ScenarioState : std::uint8_t
{
  INITIALIZING = 0,
  WAITING_FOR_NAVIGATION,
  NORMAL_NAVIGATION,
  WAITING_FOR_OBSTACLE_STOP,
  WAITING_FOR_OBSTACLE_RECOVERY,
  WAITING_FOR_OBSTACLE_RESUME,
  WAITING_FOR_SENSOR_STOP,
  WAITING_FOR_SENSOR_RECOVERY,
  WAITING_FOR_SENSOR_RESUME,
  FINAL_NAVIGATION,
  PASSED,
  FAILED
};

enum class ScenarioEvent : std::uint8_t
{
  SYSTEM_READY = 0,
  MISSION_ACCEPTED,
  NAVIGATION_STARTED,
  OBSTACLE_INJECTED,
  OBSTACLE_STOP_CONFIRMED,
  OBSTACLE_REMOVED,
  OBSTACLE_RECOVERED,
  NAVIGATION_RESUMED,
  SENSOR_FAULT_INJECTED,
  SENSOR_STOP_CONFIRMED,
  SENSOR_RESTORED,
  SENSOR_RECOVERED,
  GOAL_REACHED,
  STEP_TIMEOUT,
  CRITICAL_FAILURE
};

class ScenarioStateMachine
{
public:
  [[nodiscard]] bool handle(ScenarioEvent event) noexcept;
  [[nodiscard]] ScenarioState state() const noexcept;
  [[nodiscard]] bool terminal() const noexcept;
  [[nodiscard]] bool passed() const noexcept;

private:
  ScenarioState state_{ScenarioState::INITIALIZING};
};

[[nodiscard]] const char * to_string(ScenarioState state) noexcept;

}  // namespace system_integration_test

#endif  // SYSTEM_INTEGRATION_TEST__SCENARIO_STATE_MACHINE_HPP_

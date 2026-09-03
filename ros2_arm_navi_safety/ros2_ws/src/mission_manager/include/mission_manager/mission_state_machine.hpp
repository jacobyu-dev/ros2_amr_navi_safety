#ifndef MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_
#define MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_

#include <string>

#include "mission_manager/mission.hpp"

namespace mission_manager
{

enum class MissionState
{
  IDLE,
  RUNNING,
  PAUSED,
  COMPLETED,
  FAILED,
  CANCELED
};

enum class NavigationState
{
  IDLE,
  WAITING_FOR_GOAL,
  NAVIGATING,
  PAUSED,
  ARRIVED,
  FAILED,
  CANCELED
};

[[nodiscard]] const char * toString(MissionState state) noexcept;
[[nodiscard]] const char * toString(NavigationState state) noexcept;

/**
 * @brief Pure mission/navigation lifecycle policy.
 *
 * The ROS node owns action handles and synchronization.  This class owns only
 * legal state transitions, keeping the safety policy independently testable.
 */
class MissionStateMachine
{
public:
  [[nodiscard]] MissionState missionState() const noexcept;
  [[nodiscard]] NavigationState navigationState() const noexcept;

  [[nodiscard]] bool start(const Mission & mission) noexcept;
  [[nodiscard]] bool beginNavigation() noexcept;
  [[nodiscard]] bool goalSucceeded(Mission & mission) noexcept;
  [[nodiscard]] bool navigationFailed() noexcept;
  [[nodiscard]] bool pauseBySafety() noexcept;
  [[nodiscard]] bool cancel() noexcept;
  [[nodiscard]] bool resume(bool safety_motion_permitted) noexcept;

private:
  [[nodiscard]] bool canStart() const noexcept;

  MissionState mission_state_{MissionState::IDLE};
  NavigationState navigation_state_{NavigationState::IDLE};
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_

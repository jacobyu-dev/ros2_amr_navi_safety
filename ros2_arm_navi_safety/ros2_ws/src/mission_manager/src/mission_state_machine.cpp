#include "mission_manager/mission_state_machine.hpp"

namespace mission_manager
{

const char * toString(MissionState state) noexcept
{
  switch (state) {
    case MissionState::IDLE: return "IDLE";
    case MissionState::RUNNING: return "RUNNING";
    case MissionState::PAUSED: return "PAUSED";
    case MissionState::COMPLETED: return "COMPLETED";
    case MissionState::FAILED: return "FAILED";
    case MissionState::CANCELED: return "CANCELED";
  }
  return "UNKNOWN";
}

const char * toString(NavigationState state) noexcept
{
  switch (state) {
    case NavigationState::IDLE: return "IDLE";
    case NavigationState::WAITING_FOR_GOAL: return "WAITING_FOR_GOAL";
    case NavigationState::NAVIGATING: return "NAVIGATING";
    case NavigationState::PAUSED: return "PAUSED";
    case NavigationState::ARRIVED: return "ARRIVED";
    case NavigationState::FAILED: return "FAILED";
    case NavigationState::CANCELED: return "CANCELED";
  }
  return "UNKNOWN";
}

MissionState MissionStateMachine::missionState() const noexcept {return mission_state_;}
NavigationState MissionStateMachine::navigationState() const noexcept {return navigation_state_;}

bool MissionStateMachine::canStart() const noexcept
{
  return mission_state_ == MissionState::IDLE || mission_state_ == MissionState::COMPLETED ||
         mission_state_ == MissionState::FAILED || mission_state_ == MissionState::CANCELED;
}

bool MissionStateMachine::start(const Mission & mission) noexcept
{
  if (!canStart() || !mission.hasCurrentGoal()) {
    return false;
  }
  mission_state_ = MissionState::RUNNING;
  navigation_state_ = NavigationState::WAITING_FOR_GOAL;
  return true;
}

bool MissionStateMachine::beginNavigation() noexcept
{
  if (mission_state_ != MissionState::RUNNING || navigation_state_ != NavigationState::WAITING_FOR_GOAL) {
    return false;
  }
  navigation_state_ = NavigationState::NAVIGATING;
  return true;
}

bool MissionStateMachine::goalSucceeded(Mission & mission) noexcept
{
  if (mission_state_ != MissionState::RUNNING || navigation_state_ != NavigationState::NAVIGATING ||
    !mission.hasCurrentGoal())
  {
    return false;
  }
  navigation_state_ = NavigationState::ARRIVED;
  mission.advanceGoal();
  if (mission.hasCurrentGoal()) {
    navigation_state_ = NavigationState::WAITING_FOR_GOAL;
  } else {
    mission_state_ = MissionState::COMPLETED;
    navigation_state_ = NavigationState::IDLE;
  }
  return true;
}

bool MissionStateMachine::navigationFailed() noexcept
{
  if (mission_state_ != MissionState::RUNNING || navigation_state_ != NavigationState::NAVIGATING) {
    return false;
  }
  navigation_state_ = NavigationState::FAILED;
  mission_state_ = MissionState::FAILED;
  return true;
}

bool MissionStateMachine::pauseBySafety() noexcept
{
  if (mission_state_ != MissionState::RUNNING ||
    (navigation_state_ != NavigationState::NAVIGATING &&
    navigation_state_ != NavigationState::WAITING_FOR_GOAL))
  {
    return false;
  }
  mission_state_ = MissionState::PAUSED;
  navigation_state_ = NavigationState::PAUSED;
  return true;
}

bool MissionStateMachine::cancel() noexcept
{
  if ((mission_state_ != MissionState::RUNNING && mission_state_ != MissionState::PAUSED) ||
    (navigation_state_ != NavigationState::NAVIGATING && navigation_state_ != NavigationState::PAUSED &&
    navigation_state_ != NavigationState::WAITING_FOR_GOAL))
  {
    return false;
  }
  mission_state_ = MissionState::CANCELED;
  navigation_state_ = NavigationState::CANCELED;
  return true;
}

bool MissionStateMachine::resume(bool safety_motion_permitted) noexcept
{
  if (mission_state_ != MissionState::PAUSED || navigation_state_ != NavigationState::PAUSED ||
    !safety_motion_permitted)
  {
    return false;
  }
  mission_state_ = MissionState::RUNNING;
  navigation_state_ = NavigationState::WAITING_FOR_GOAL;
  return true;
}

}  // namespace mission_manager

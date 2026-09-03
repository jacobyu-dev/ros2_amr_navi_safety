#include "mission_manager/mission.hpp"

#include <stdexcept>

namespace mission_manager
{

bool Mission::hasCurrentGoal() const noexcept
{
  return current_goal_index < goals.size();
}

bool Mission::hasNextGoal() const noexcept
{
  return current_goal_index + 1U < goals.size();
}

const geometry_msgs::msg::PoseStamped & Mission::currentGoal() const
{
  if (!hasCurrentGoal()) {
    throw std::out_of_range("mission has no current goal");
  }
  return goals.at(current_goal_index);
}

void Mission::advanceGoal() noexcept
{
  if (hasCurrentGoal()) {
    ++current_goal_index;
  }
}

void Mission::reset() noexcept
{
  current_goal_index = 0U;
}

}  // namespace mission_manager

#ifndef MISSION_MANAGER__MISSION_HPP_
#define MISSION_MANAGER__MISSION_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace mission_manager
{

struct Mission
{
  std::uint64_t mission_id{0U};
  std::vector<geometry_msgs::msg::PoseStamped> goals;
  std::size_t current_goal_index{0U};

  [[nodiscard]] bool hasCurrentGoal() const noexcept;
  [[nodiscard]] bool hasNextGoal() const noexcept;
  [[nodiscard]] const geometry_msgs::msg::PoseStamped & currentGoal() const;
  void advanceGoal() noexcept;
  void reset() noexcept;
};

}  // namespace mission_manager

#endif  // MISSION_MANAGER__MISSION_HPP_

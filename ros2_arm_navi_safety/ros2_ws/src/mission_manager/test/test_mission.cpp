#include "gtest/gtest.h"
#include "mission_manager/mission.hpp"

namespace mission_manager
{
namespace
{

geometry_msgs::msg::PoseStamped pose(double x)
{
  geometry_msgs::msg::PoseStamped value;
  value.pose.position.x = x;
  return value;
}

TEST(MissionTest, TracksCurrentAndNextWaypoint)
{
  Mission mission{42U, {pose(1.0), pose(2.0)}, 0U};
  EXPECT_TRUE(mission.hasCurrentGoal());
  EXPECT_TRUE(mission.hasNextGoal());
  EXPECT_DOUBLE_EQ(mission.currentGoal().pose.position.x, 1.0);

  mission.advanceGoal();
  EXPECT_TRUE(mission.hasCurrentGoal());
  EXPECT_FALSE(mission.hasNextGoal());
  EXPECT_DOUBLE_EQ(mission.currentGoal().pose.position.x, 2.0);

  mission.advanceGoal();
  EXPECT_FALSE(mission.hasCurrentGoal());
  EXPECT_THROW(static_cast<void>(mission.currentGoal()), std::out_of_range);
}

TEST(MissionTest, ResetReturnsToFirstWaypoint)
{
  Mission mission{7U, {pose(1.0), pose(2.0)}, 0U};
  mission.advanceGoal();
  mission.reset();
  EXPECT_EQ(mission.current_goal_index, 0U);
  EXPECT_DOUBLE_EQ(mission.currentGoal().pose.position.x, 1.0);
}

}  // namespace
}  // namespace mission_manager

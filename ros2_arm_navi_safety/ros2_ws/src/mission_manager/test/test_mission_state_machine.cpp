#include "gtest/gtest.h"
#include "mission_manager/mission_state_machine.hpp"

namespace mission_manager
{
namespace
{

Mission twoGoalMission()
{
  Mission mission;
  mission.mission_id = 1U;
  mission.goals.resize(2U);
  return mission;
}

TEST(MissionStateMachineTest, InitialStateIsIdle)
{
  MissionStateMachine machine;
  EXPECT_EQ(machine.missionState(), MissionState::IDLE);
  EXPECT_EQ(machine.navigationState(), NavigationState::IDLE);
}

TEST(MissionStateMachineTest, StartAndSequentialGoalCompletion)
{
  MissionStateMachine machine;
  auto mission = twoGoalMission();
  ASSERT_TRUE(machine.start(mission));
  EXPECT_EQ(machine.missionState(), MissionState::RUNNING);
  EXPECT_EQ(machine.navigationState(), NavigationState::WAITING_FOR_GOAL);

  ASSERT_TRUE(machine.beginNavigation());
  ASSERT_TRUE(machine.goalSucceeded(mission));
  EXPECT_EQ(mission.current_goal_index, 1U);
  EXPECT_EQ(machine.missionState(), MissionState::RUNNING);
  EXPECT_EQ(machine.navigationState(), NavigationState::WAITING_FOR_GOAL);

  ASSERT_TRUE(machine.beginNavigation());
  ASSERT_TRUE(machine.goalSucceeded(mission));
  EXPECT_EQ(machine.missionState(), MissionState::COMPLETED);
  EXPECT_EQ(machine.navigationState(), NavigationState::IDLE);
}

TEST(MissionStateMachineTest, NavigationFailureFailsMission)
{
  MissionStateMachine machine;
  auto mission = twoGoalMission();
  ASSERT_TRUE(machine.start(mission));
  ASSERT_TRUE(machine.beginNavigation());
  EXPECT_TRUE(machine.navigationFailed());
  EXPECT_EQ(machine.missionState(), MissionState::FAILED);
  EXPECT_EQ(machine.navigationState(), NavigationState::FAILED);
}

TEST(MissionStateMachineTest, CancelTransitionsRunningMission)
{
  MissionStateMachine machine;
  auto mission = twoGoalMission();
  ASSERT_TRUE(machine.start(mission));
  ASSERT_TRUE(machine.beginNavigation());
  EXPECT_TRUE(machine.cancel());
  EXPECT_EQ(machine.missionState(), MissionState::CANCELED);
  EXPECT_EQ(machine.navigationState(), NavigationState::CANCELED);
}

TEST(MissionStateMachineTest, SafetyStopAndExplicitResumePreserveWaypoint)
{
  MissionStateMachine machine;
  auto mission = twoGoalMission();
  ASSERT_TRUE(machine.start(mission));
  ASSERT_TRUE(machine.beginNavigation());
  ASSERT_TRUE(machine.pauseBySafety());
  EXPECT_EQ(machine.missionState(), MissionState::PAUSED);
  EXPECT_EQ(machine.navigationState(), NavigationState::PAUSED);
  EXPECT_EQ(mission.current_goal_index, 0U);
  EXPECT_FALSE(machine.resume(false));
  EXPECT_TRUE(machine.resume(true));
  EXPECT_EQ(machine.missionState(), MissionState::RUNNING);
  EXPECT_EQ(machine.navigationState(), NavigationState::WAITING_FOR_GOAL);
  EXPECT_EQ(mission.current_goal_index, 0U);
}

TEST(MissionStateMachineTest, InvalidTransitionsAreRejected)
{
  MissionStateMachine machine;
  auto mission = twoGoalMission();
  EXPECT_FALSE(machine.resume(true));
  EXPECT_FALSE(machine.cancel());
  EXPECT_FALSE(machine.start(Mission{}));
  ASSERT_TRUE(machine.start(mission));
  EXPECT_FALSE(machine.start(mission));
  EXPECT_FALSE(machine.goalSucceeded(mission));
  ASSERT_TRUE(machine.beginNavigation());
  ASSERT_TRUE(machine.goalSucceeded(mission));
  EXPECT_FALSE(machine.resume(true));
}

}  // namespace
}  // namespace mission_manager

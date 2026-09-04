#include <gtest/gtest.h>

#include "system_integration_test/scenario_state_machine.hpp"

using system_integration_test::ScenarioEvent;
using system_integration_test::ScenarioState;
using system_integration_test::ScenarioStateMachine;

TEST(ScenarioStateMachine, CompletesExpectedSequence)
{
  ScenarioStateMachine machine;
  EXPECT_TRUE(machine.handle(ScenarioEvent::SYSTEM_READY));
  EXPECT_TRUE(machine.handle(ScenarioEvent::MISSION_ACCEPTED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::OBSTACLE_INJECTED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::OBSTACLE_STOP_CONFIRMED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::OBSTACLE_REMOVED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::OBSTACLE_RECOVERED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::NAVIGATION_RESUMED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::SENSOR_FAULT_INJECTED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::SENSOR_STOP_CONFIRMED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::SENSOR_RESTORED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::SENSOR_RECOVERED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::NAVIGATION_RESUMED));
  EXPECT_TRUE(machine.handle(ScenarioEvent::GOAL_REACHED));
  EXPECT_EQ(machine.state(), ScenarioState::PASSED);
  EXPECT_TRUE(machine.passed());
}

TEST(ScenarioStateMachine, TimeoutFailsEveryActiveState)
{
  ScenarioStateMachine machine;
  EXPECT_TRUE(machine.handle(ScenarioEvent::STEP_TIMEOUT));
  EXPECT_EQ(machine.state(), ScenarioState::FAILED);
  EXPECT_FALSE(machine.passed());
}

TEST(ScenarioStateMachine, UnexpectedEventDoesNotAdvance)
{
  ScenarioStateMachine machine;
  EXPECT_FALSE(machine.handle(ScenarioEvent::GOAL_REACHED));
  EXPECT_EQ(machine.state(), ScenarioState::INITIALIZING);
}

TEST(ScenarioStateMachine, TerminalStateRejectsFurtherEvents)
{
  ScenarioStateMachine machine;
  EXPECT_TRUE(machine.handle(ScenarioEvent::CRITICAL_FAILURE));
  EXPECT_FALSE(machine.handle(ScenarioEvent::SYSTEM_READY));
  EXPECT_EQ(machine.state(), ScenarioState::FAILED);
}

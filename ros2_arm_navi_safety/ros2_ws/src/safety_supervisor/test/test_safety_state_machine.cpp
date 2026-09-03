#include "gtest/gtest.h"
#include "safety_supervisor/safety_state_machine.hpp"

namespace safety_supervisor
{
namespace
{

SafetyEvaluation ready(SafetyLevel level)
{
  return SafetyEvaluation{true, level, "test"};
}

TEST(SafetyStateMachineTest, InitTransitionsToSafeWhenAllSourcesAreSafe)
{
  SafetyStateMachine machine;
  EXPECT_EQ(machine.update(ready(SafetyLevel::SAFE)), SystemSafetyState::SAFE);
}

TEST(SafetyStateMachineTest, SafeTransitionsToWarning)
{
  SafetyStateMachine machine;
  static_cast<void>(machine.update(ready(SafetyLevel::SAFE)));
  EXPECT_EQ(machine.update(ready(SafetyLevel::WARNING)), SystemSafetyState::WARNING);
}

TEST(SafetyStateMachineTest, WarningRecoversToSafe)
{
  SafetyStateMachine machine;
  static_cast<void>(machine.update(ready(SafetyLevel::WARNING)));
  EXPECT_EQ(machine.update(ready(SafetyLevel::SAFE)), SystemSafetyState::SAFE);
}

TEST(SafetyStateMachineTest, SafeTransitionsToStop)
{
  SafetyStateMachine machine;
  static_cast<void>(machine.update(ready(SafetyLevel::SAFE)));
  EXPECT_EQ(machine.update(ready(SafetyLevel::STOP)), SystemSafetyState::STOP);
}

TEST(SafetyStateMachineTest, StopRecoversToSafe)
{
  SafetyStateMachine machine;
  static_cast<void>(machine.update(ready(SafetyLevel::STOP)));
  EXPECT_EQ(machine.update(ready(SafetyLevel::SAFE)), SystemSafetyState::SAFE);
}

TEST(SafetyStateMachineTest, SafeTransitionsToFault)
{
  SafetyStateMachine machine;
  static_cast<void>(machine.update(ready(SafetyLevel::SAFE)));
  EXPECT_EQ(machine.update(ready(SafetyLevel::FAULT)), SystemSafetyState::FAULT);
}

TEST(SafetyStateMachineTest, FaultRecoversToSafe)
{
  SafetyStateMachine machine;
  static_cast<void>(machine.update(ready(SafetyLevel::FAULT)));
  EXPECT_EQ(machine.update(ready(SafetyLevel::SAFE)), SystemSafetyState::SAFE);
}

TEST(SafetyStateMachineTest, AggregatesWarningAndStopToStop)
{
  EXPECT_EQ(
    worst_safety_level(SafetyLevel::WARNING, SafetyLevel::STOP), SafetyLevel::STOP);
}

TEST(SafetyStateMachineTest, AggregatesStopAndFaultToFault)
{
  EXPECT_EQ(
    worst_safety_level(SafetyLevel::STOP, SafetyLevel::FAULT), SafetyLevel::FAULT);
}

TEST(SafetyStateMachineTest, UnknownInputTransitionsToFault)
{
  SafetyStateMachine machine;
  EXPECT_EQ(machine.update(ready(SafetyLevel::UNKNOWN)), SystemSafetyState::FAULT);
}

TEST(SafetyStateMachineTest, MissingSourcesRemainInitDuringStartup)
{
  SafetyStateMachine machine;
  EXPECT_EQ(machine.update(SafetyEvaluation{false, SafetyLevel::UNKNOWN, "waiting"}),
    SystemSafetyState::INIT);
}

}  // namespace
}  // namespace safety_supervisor

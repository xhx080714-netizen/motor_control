#include <gtest/gtest.h>

#include "sand_rake_control/safety_state_machine.hpp"

using sand_rake_control::SafetyState;
using sand_rake_control::SafetyStateMachine;

TEST(SafetyStateMachineTest, InitialStateIsReady)
{
  SafetyStateMachine machine;

  EXPECT_EQ(machine.get_state(), SafetyState::READY);
  EXPECT_STREQ(machine.get_state_name(), "READY");
  EXPECT_FALSE(machine.motion_allowed());
  EXPECT_TRUE(machine.command_is_zero());
  EXPECT_FALSE(machine.stop_condition_active());
  EXPECT_FALSE(machine.fault_condition_active());
}
TEST(SafetyStateMachineTest, NonZeroCommandStartsRunning)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);

  EXPECT_EQ(machine.get_state(), SafetyState::RUNNING);
  EXPECT_TRUE(machine.motion_allowed());
  EXPECT_FALSE(machine.command_is_zero());
}

TEST(SafetyStateMachineTest, ZeroCommandReturnsToReady)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);
  EXPECT_EQ(machine.get_state(), SafetyState::RUNNING);

  machine.update_motion_command(true);

  EXPECT_EQ(machine.get_state(), SafetyState::READY);
  EXPECT_FALSE(machine.motion_allowed());
  EXPECT_TRUE(machine.command_is_zero());
}

TEST(SafetyStateMachineTest, StopConditionLatchesStop)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);
  machine.update_stop_condition(true);

  EXPECT_EQ(machine.get_state(), SafetyState::STOP_LATCHED);
  EXPECT_TRUE(machine.stop_condition_active());
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachineTest, ClearingStopDoesNotAutomaticallyUnlock)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);
  machine.update_stop_condition(true);

  machine.update_stop_condition(false);

  EXPECT_EQ(machine.get_state(), SafetyState::STOP_LATCHED);
  EXPECT_FALSE(machine.stop_condition_active());
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachineTest, ResetFailsWhileStopConditionIsActive)
{
  SafetyStateMachine machine;

  machine.update_stop_condition(true);

  const bool result = machine.request_reset();

  EXPECT_FALSE(result);
  EXPECT_EQ(machine.get_state(), SafetyState::STOP_LATCHED);
}

TEST(SafetyStateMachineTest, ResetFailsWhileOldMotionCommandIsStillNonZero)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);
  machine.update_stop_condition(true);
  machine.update_stop_condition(false);

  const bool result = machine.request_reset();

  EXPECT_FALSE(result);
  EXPECT_EQ(machine.get_state(), SafetyState::STOP_LATCHED);
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachineTest, ResetSucceedsAfterStopClearedAndCommandZero)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);
  machine.update_stop_condition(true);

  machine.update_stop_condition(false);
  machine.update_motion_command(true);

  const bool result = machine.request_reset();

  EXPECT_TRUE(result);
  EXPECT_EQ(machine.get_state(), SafetyState::READY);
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachineTest, FaultEntersFaultState)
{
  SafetyStateMachine machine;

  machine.update_motion_command(false);
  machine.update_fault_condition(true);

  EXPECT_EQ(machine.get_state(), SafetyState::FAULT);
  EXPECT_TRUE(machine.fault_condition_active());
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachineTest, ClearingFaultDoesNotAutomaticallyUnlock)
{
  SafetyStateMachine machine;

  machine.update_fault_condition(true);
  machine.update_fault_condition(false);

  EXPECT_EQ(machine.get_state(), SafetyState::FAULT);
  EXPECT_FALSE(machine.fault_condition_active());
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachineTest, FaultCanResetAfterFaultClearedAndCommandZero)
{
  SafetyStateMachine machine;

  machine.update_fault_condition(true);
  machine.update_fault_condition(false);
  machine.update_motion_command(true);

  const bool result = machine.request_reset();

  EXPECT_TRUE(result);
  EXPECT_EQ(machine.get_state(), SafetyState::READY);
}

#include <gtest/gtest.h>

#include "sand_rake_control/safety_state_machine.hpp"

using sand_rake_control::SafetyState;
using sand_rake_control::SafetyStateMachine;

TEST(SafetyStateMachine, StartsReady)
{
  SafetyStateMachine machine;
  EXPECT_EQ(machine.get_state(), SafetyState::READY);
  EXPECT_FALSE(machine.motion_allowed());
}

TEST(SafetyStateMachine, NonzeroCommandRuns)
{
  SafetyStateMachine machine;
  machine.update_motion_command(false);
  EXPECT_EQ(machine.get_state(), SafetyState::RUNNING);
  EXPECT_TRUE(machine.motion_allowed());
}

TEST(SafetyStateMachine, StopLatchesUntilClearAndReset)
{
  SafetyStateMachine machine;
  machine.update_motion_command(false);
  machine.update_stop_condition(true);
  EXPECT_EQ(machine.get_state(), SafetyState::STOP_LATCHED);
  EXPECT_FALSE(machine.request_reset());

  machine.update_stop_condition(false);
  machine.update_motion_command(true);
  EXPECT_TRUE(machine.request_reset());
  EXPECT_EQ(machine.get_state(), SafetyState::READY);
}

TEST(SafetyStateMachine, ResetRejectsNonzeroCommand)
{
  SafetyStateMachine machine;
  machine.update_stop_condition(true);
  machine.update_stop_condition(false);
  machine.update_motion_command(false);
  EXPECT_FALSE(machine.request_reset());
  EXPECT_EQ(machine.get_state(), SafetyState::STOP_LATCHED);
}

TEST(SafetyStateMachine, FaultRequiresConditionClearAndReset)
{
  SafetyStateMachine machine;
  machine.update_fault_condition(true);
  EXPECT_EQ(machine.get_state(), SafetyState::FAULT);
  machine.update_fault_condition(false);
  EXPECT_TRUE(machine.request_reset());
  EXPECT_EQ(machine.get_state(), SafetyState::READY);
}

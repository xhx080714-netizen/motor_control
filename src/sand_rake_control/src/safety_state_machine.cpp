#include "sand_rake_control/safety_state_machine.hpp"

namespace sand_rake_control
{

SafetyStateMachine::SafetyStateMachine()
: state_(SafetyState::READY),
  stop_active_(false),
  fault_active_(false),
  command_is_zero_(true)
{
}

void SafetyStateMachine::update_motion_command(bool command_is_zero)
{
  command_is_zero_ = command_is_zero;

  if (state_ == SafetyState::STOP_LATCHED ||
    state_ == SafetyState::FAULT)
  {
    return;
  }

  if (fault_active_) {
    state_ = SafetyState::FAULT;
    return;
  }

  if (stop_active_) {
    state_ = SafetyState::STOP_LATCHED;
    return;
  }

  if (command_is_zero_) {
    state_ = SafetyState::READY;
  } else {
    state_ = SafetyState::RUNNING;
  }
}

void SafetyStateMachine::update_stop_condition(bool stop_active)
{
  stop_active_ = stop_active;

  if (stop_active_) {
    if (state_ != SafetyState::FAULT) {
      state_ = SafetyState::STOP_LATCHED;
    }
  }
}

void SafetyStateMachine::update_fault_condition(bool fault_active)
{
  fault_active_ = fault_active;

  if (fault_active_) {
    state_ = SafetyState::FAULT;
  }
}

bool SafetyStateMachine::request_reset()
{
  if (stop_active_ || fault_active_) {
    return false;
  }

  if (!command_is_zero_) {
    return false;
  }

  if (state_ == SafetyState::STOP_LATCHED ||
    state_ == SafetyState::FAULT)
  {
    state_ = SafetyState::READY;
    return true;
  }

  return false;
}

SafetyState SafetyStateMachine::get_state() const
{
  return state_;
}

const char * SafetyStateMachine::get_state_name() const
{
  switch (state_) {
    case SafetyState::READY:
      return "READY";

    case SafetyState::RUNNING:
      return "RUNNING";

    case SafetyState::STOP_LATCHED:
      return "STOP_LATCHED";

    case SafetyState::FAULT:
      return "FAULT";

    default:
      return "UNKNOWN";
  }
}

bool SafetyStateMachine::motion_allowed() const
{
  return
    state_ == SafetyState::RUNNING &&
    !stop_active_ &&
    !fault_active_ &&
    !command_is_zero_;
}

bool SafetyStateMachine::stop_condition_active() const
{
  return stop_active_;
}

bool SafetyStateMachine::fault_condition_active() const
{
  return fault_active_;
}

bool SafetyStateMachine::command_is_zero() const
{
  return command_is_zero_;
}

}  // namespace sand_rake_control

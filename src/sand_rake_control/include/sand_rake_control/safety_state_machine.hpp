#ifndef SAND_RAKE_CONTROL__SAFETY_STATE_MACHINE_HPP_
#define SAND_RAKE_CONTROL__SAFETY_STATE_MACHINE_HPP_

namespace sand_rake_control
{

enum class SafetyState
{
  READY,
  RUNNING,
  STOP_LATCHED,
  FAULT,
};

class SafetyStateMachine
{
public:
  SafetyStateMachine();

  void update_motion_command(bool command_is_zero);
  void update_stop_condition(bool stop_active);
  void update_fault_condition(bool fault_active);
  bool request_reset();

  SafetyState get_state() const;
  const char * get_state_name() const;
  bool motion_allowed() const;
  bool stop_condition_active() const;
  bool fault_condition_active() const;
  bool command_is_zero() const;

private:
  SafetyState state_;
  bool stop_active_;
  bool fault_active_;
  bool command_is_zero_;
};

}  // namespace sand_rake_control

#endif  // SAND_RAKE_CONTROL__SAFETY_STATE_MACHINE_HPP_

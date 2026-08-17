#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "sand_rake_control/safety_state_machine.hpp"
#include "sand_rake_interfaces/msg/safety_event.hpp"

using namespace std::chrono_literals;

class SafetyController : public rclcpp::Node
{
public:
  SafetyController()
  : Node("safety_controller"),
    received_cmd_(false),
    cmd_timeout_active_(false)
  {
    cmd_timeout_sec_ =
      this->declare_parameter<double>("cmd_timeout_sec", 0.5);
    if (cmd_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("cmd_timeout_sec must be positive");
    }
    watchdog_margin_sec_ =
      this->declare_parameter<double>("watchdog_margin_sec", 0.01);
    watchdog_margin_sec_ = std::clamp(
      watchdog_margin_sec_, 0.0, cmd_timeout_sec_ * 0.25);

    cmd_sub_ =
      this->create_subscription<geometry_msgs::msg::Twist>(
      "/teleop/cmd_vel",
      10,
      std::bind(
        &SafetyController::cmd_callback,
        this,
        std::placeholders::_1));

    safety_event_sub_ =
      this->create_subscription<sand_rake_interfaces::msg::SafetyEvent>(
      "/safety/event",
      10,
      std::bind(
        &SafetyController::safety_event_callback,
        this,
        std::placeholders::_1));

    cmd_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
      "/safety/cmd_vel",
      10);

    reset_service_ =
      this->create_service<std_srvs::srv::Trigger>(
      "/safety/reset",
      std::bind(
        &SafetyController::reset_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    const auto watchdog_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(cmd_timeout_sec_ - watchdog_margin_sec_));
    watchdog_timer_ = this->create_wall_timer(
      watchdog_period,
      std::bind(&SafetyController::watchdog_callback, this));
    watchdog_timer_->cancel();

    timer_ =
      this->create_wall_timer(
      50ms,
      std::bind(
        &SafetyController::timer_callback,
        this));

    RCLCPP_INFO(
      this->get_logger(),
      "Safety controller started.");

    RCLCPP_INFO(
      this->get_logger(),
      "Input: /teleop/cmd_vel, /safety/event");

    RCLCPP_INFO(
      this->get_logger(),
      "Output: /safety/cmd_vel at 20 Hz");

    RCLCPP_INFO(
      this->get_logger(),
      "Command timeout: %.3f s",
      cmd_timeout_sec_);
  }

private:
  bool is_zero_command(
    const geometry_msgs::msg::Twist & msg) const
  {
    constexpr double epsilon = 1e-6;

    return
      std::abs(msg.linear.x) < epsilon &&
      std::abs(msg.linear.y) < epsilon &&
      std::abs(msg.linear.z) < epsilon &&
      std::abs(msg.angular.x) < epsilon &&
      std::abs(msg.angular.y) < epsilon &&
      std::abs(msg.angular.z) < epsilon;
  }

  void cmd_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_ = *msg;

    received_cmd_ = true;
    watchdog_timer_->reset();

    if (cmd_timeout_active_) {
      cmd_timeout_active_ = false;

      RCLCPP_INFO(
        this->get_logger(),
        "Teleop command stream recovered.");
    }

    const bool command_is_zero =
      is_zero_command(*msg);

    state_machine_.update_motion_command(
      command_is_zero);
  }

  void safety_event_callback(
    const sand_rake_interfaces::msg::SafetyEvent::SharedPtr msg)
  {
    const auto previous_state =
      state_machine_.get_state();

    state_machine_.update_stop_condition(
      msg->stop);

    const auto current_state =
      state_machine_.get_state();

    RCLCPP_INFO(
      this->get_logger(),
      "Safety event received: stop=%s, reason=%u",
      msg->stop ? "true" : "false",
      static_cast<unsigned int>(msg->reason));

    if (current_state != previous_state) {
      RCLCPP_WARN(
        this->get_logger(),
        "Safety state changed: %s",
        state_machine_.get_state_name());
    }

    if (!msg->stop) {
      RCLCPP_INFO(
        this->get_logger(),
        "Stop condition cleared, "
        "but latched state requires manual reset.");
    }
  }

  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    const bool success =
      state_machine_.request_reset();

    response->success = success;

    if (success) {
      response->message =
        "Safety state reset to READY.";

      latest_cmd_ =
        geometry_msgs::msg::Twist();

      RCLCPP_INFO(
        this->get_logger(),
        "Safety reset succeeded. State: %s",
        state_machine_.get_state_name());
    } else {
      response->message =
        "Safety reset rejected: "
        "stop/fault condition may still be active "
        "or teleop command is not zero.";

      RCLCPP_WARN(
        this->get_logger(),
        "Safety reset rejected. State: %s",
        state_machine_.get_state_name());
    }
  }

  void timer_callback()
  {
    geometry_msgs::msg::Twist output_cmd;

    const bool command_timed_out =
      !received_cmd_ || cmd_timeout_active_;

    if (command_timed_out) {
      /*
       * A timeout invalidates the previous motion command.
       *
       * This also records command_is_zero=true inside the
       * SafetyStateMachine. If the state is STOP_LATCHED or
       * FAULT, the state itself remains latched.
       */
      state_machine_.update_motion_command(true);

      output_cmd = geometry_msgs::msg::Twist();
    } else if (state_machine_.motion_allowed()) {
      output_cmd = latest_cmd_;
    } else {
      output_cmd = geometry_msgs::msg::Twist();
    }

    cmd_pub_->publish(output_cmd);
  }

  void watchdog_callback()
  {
    if (!received_cmd_) {
      watchdog_timer_->cancel();
      return;
    }

    cmd_timeout_active_ = true;
    state_machine_.update_motion_command(true);

    // Publish immediately at the deadline instead of waiting for the next
    // 20 Hz arbitration tick (which could add almost another 50 ms).
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    watchdog_timer_->cancel();

    RCLCPP_WARN(
      this->get_logger(),
      "Teleop command timeout. Forcing zero velocity.");
  }

  sand_rake_control::SafetyStateMachine state_machine_;

  geometry_msgs::msg::Twist latest_cmd_;

  double cmd_timeout_sec_;
  double watchdog_margin_sec_;

  bool received_cmd_;
  bool cmd_timeout_active_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  rclcpp::Subscription<
    sand_rake_interfaces::msg::SafetyEvent>::SharedPtr
    safety_event_sub_;

  rclcpp::Publisher<
    geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Service<
    std_srvs::srv::Trigger>::SharedPtr reset_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<SafetyController>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

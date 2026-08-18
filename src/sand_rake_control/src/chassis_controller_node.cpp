#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sand_rake_control/diff_drive_kinematics.hpp"
#include "sand_rake_control/msg/wheel_rpm.hpp"

class ChassisControllerNode : public rclcpp::Node
{
public:
  ChassisControllerNode()
  : Node("chassis_controller")
  {
    const double wheel_radius_m =
      declare_parameter<double>("wheel_radius_m", 0.134665);
    const double effective_track_width_m =
      declare_parameter<double>("effective_track_width_m", 0.53907);
    const double gear_ratio = declare_parameter<double>("gear_ratio", 7.5);
    const double max_motor_rpm = declare_parameter<double>("max_motor_rpm", 1500.0);
    const double min_motor_rpm = declare_parameter<double>("min_motor_rpm", 30.0);

    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.5);
    max_linear_speed_mps_ =
      declare_parameter<double>("max_linear_speed_mps", 0.15);
    max_angular_speed_rps_ =
      declare_parameter<double>("max_angular_speed_rps", 0.8);
    max_linear_accel_mps2_ =
      declare_parameter<double>("max_linear_accel_mps2", 0.30);
    max_angular_accel_rps2_ =
      declare_parameter<double>("max_angular_accel_rps2", 1.5);

    if (!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0 ||
      !std::isfinite(cmd_timeout_sec_) || cmd_timeout_sec_ <= 0.0 ||
      !std::isfinite(max_linear_speed_mps_) || max_linear_speed_mps_ <= 0.0 ||
      !std::isfinite(max_angular_speed_rps_) || max_angular_speed_rps_ <= 0.0 ||
      !std::isfinite(max_linear_accel_mps2_) || max_linear_accel_mps2_ <= 0.0 ||
      !std::isfinite(max_angular_accel_rps2_) || max_angular_accel_rps2_ <= 0.0)
    {
      throw std::invalid_argument(
              "control rate, timeout, speed limits, and acceleration limits "
              "must be finite and positive");
    }

    kinematics_ = std::make_unique<sand_rake_control::DiffDriveKinematics>(
      wheel_radius_m,
      effective_track_width_m,
      gear_ratio,
      max_motor_rpm,
      min_motor_rpm);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/safety/cmd_vel", 10,
      std::bind(&ChassisControllerNode::cmd_callback, this, std::placeholders::_1));
    wheel_cmd_pub_ = create_publisher<sand_rake_control::msg::WheelRpm>(
      "/chassis/wheel_rpm_cmd", 10);
    limited_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/chassis/cmd_vel_limited", 10);

    last_cmd_time_ = std::chrono::steady_clock::now();
    last_update_time_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    timer_ = create_wall_timer(
      period, std::bind(&ChassisControllerNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "Chassis controller started at %.1f Hz (r=%.6f m, L_eff=%.5f m, G=%.2f).",
      control_rate_hz_, wheel_radius_m, effective_track_width_m, gear_ratio);
  }

private:
  static double ramp(double current, double target, double max_rate, double dt)
  {
    const double maximum_step = max_rate * dt;
    return current + std::clamp(target - current, -maximum_step, maximum_step);
  }

  void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected cmd_vel containing NaN or infinity.");
      return;
    }

    target_linear_mps_ = std::clamp(
      msg->linear.x, -max_linear_speed_mps_, max_linear_speed_mps_);
    target_angular_rps_ = std::clamp(
      msg->angular.z, -max_angular_speed_rps_, max_angular_speed_rps_);
    received_cmd_ = true;
    last_cmd_time_ = std::chrono::steady_clock::now();
  }

  void timer_callback()
  {
    const auto stamp = now();
    const auto update_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(update_time - last_update_time_).count();
    last_update_time_ = update_time;
    dt = std::clamp(dt, 0.0, 0.1);

    const bool timed_out =
      !received_cmd_ ||
      std::chrono::duration<double>(update_time - last_cmd_time_).count() >= cmd_timeout_sec_;
    if (timed_out) {
      target_linear_mps_ = 0.0;
      target_angular_rps_ = 0.0;
    }

    const bool stop_requested =
      std::abs(target_linear_mps_) < 1.0e-9 &&
      std::abs(target_angular_rps_) < 1.0e-9;

    // This node only publishes physical-wheel RPM. A hardware bridge owns the
    // final mapping and the choice of Coast/Brake for a zero command.
    if (stop_requested) {
      current_linear_mps_ = 0.0;
      current_angular_rps_ = 0.0;
    } else {
      current_linear_mps_ = ramp(
        current_linear_mps_, target_linear_mps_, max_linear_accel_mps2_, dt);
      current_angular_rps_ = ramp(
        current_angular_rps_, target_angular_rps_, max_angular_accel_rps2_, dt);
    }

    const auto rpm = kinematics_->command_to_motor_rpm(
      current_linear_mps_, current_angular_rps_);

    sand_rake_control::msg::WheelRpm wheel_msg;
    wheel_msg.header.stamp = stamp;
    wheel_msg.header.frame_id = "base_link";
    wheel_msg.left_front = rpm.left_front;
    wheel_msg.left_rear = rpm.left_rear;
    wheel_msg.right_front = rpm.right_front;
    wheel_msg.right_rear = rpm.right_rear;
    wheel_msg.direction_valid = true;
    wheel_cmd_pub_->publish(wheel_msg);

    geometry_msgs::msg::Twist limited_cmd;
    limited_cmd.linear.x = current_linear_mps_;
    limited_cmd.angular.z = current_angular_rps_;
    limited_cmd_pub_->publish(limited_cmd);
  }

  std::unique_ptr<sand_rake_control::DiffDriveKinematics> kinematics_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<sand_rake_control::msg::WheelRpm>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr limited_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::chrono::steady_clock::time_point last_cmd_time_;
  std::chrono::steady_clock::time_point last_update_time_;
  bool received_cmd_{false};

  double control_rate_hz_{50.0};
  double cmd_timeout_sec_{0.5};
  double max_linear_speed_mps_{0.15};
  double max_angular_speed_rps_{0.8};
  double max_linear_accel_mps2_{0.3};
  double max_angular_accel_rps2_{1.5};
  double target_linear_mps_{0.0};
  double target_angular_rps_{0.0};
  double current_linear_mps_{0.0};
  double current_angular_rps_{0.0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisControllerNode>());
  rclcpp::shutdown();
  return 0;
}

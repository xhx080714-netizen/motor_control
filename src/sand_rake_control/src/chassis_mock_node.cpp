#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "sand_rake_control/diff_drive_kinematics.hpp"
#include "sand_rake_control/msg/wheel_rpm.hpp"

namespace
{

struct PlanarState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

void integrate(
  PlanarState & state,
  const sand_rake_control::BodyTwist & twist,
  double dt)
{
  const double middle_yaw = state.yaw + twist.angular_rps * dt * 0.5;
  state.x += twist.linear_mps * std::cos(middle_yaw) * dt;
  state.y += twist.linear_mps * std::sin(middle_yaw) * dt;
  state.yaw = std::remainder(
    state.yaw + twist.angular_rps * dt, 2.0 * 3.14159265358979323846);
}

geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

}  // namespace

class ChassisMockNode : public rclcpp::Node
{
public:
  ChassisMockNode()
  : Node("chassis_mock"), random_engine_(std::random_device{}())
  {
    const double wheel_radius_m =
      declare_parameter<double>("wheel_radius_m", 0.135);
    const double effective_track_width_m =
      declare_parameter<double>("effective_track_width_m", 0.54);
    const double gear_ratio = declare_parameter<double>("gear_ratio", 7.5);
    const double max_motor_rpm = declare_parameter<double>("max_motor_rpm", 1500.0);
    const double min_motor_rpm = declare_parameter<double>("min_motor_rpm", 30.0);

    simulation_rate_hz_ = declare_parameter<double>("simulation_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.5);
    left_gain_ = declare_parameter<double>("left_gain", 1.0);
    right_gain_ = declare_parameter<double>("right_gain", 1.0);
    motor_time_constant_sec_ =
      std::max(0.0, declare_parameter<double>("motor_time_constant_sec", 0.0));
    rpm_noise_stddev_ =
      std::max(0.0, declare_parameter<double>("rpm_noise_stddev", 0.0));
    path_publish_divider_ = static_cast<int>(std::max<int64_t>(
        1, declare_parameter<int64_t>("path_publish_divider", 5)));
    max_path_points_ = static_cast<int>(std::max<int64_t>(
        2, declare_parameter<int64_t>("max_path_points", 5000)));
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    if (simulation_rate_hz_ <= 0.0 || command_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("simulation rate and command timeout must be positive");
    }

    kinematics_ = std::make_unique<sand_rake_control::DiffDriveKinematics>(
      wheel_radius_m,
      effective_track_width_m,
      gear_ratio,
      max_motor_rpm,
      min_motor_rpm);
    noise_distribution_ = std::normal_distribution<double>(0.0, rpm_noise_stddev_);

    wheel_cmd_sub_ = create_subscription<sand_rake_control::msg::WheelRpm>(
      "/chassis/wheel_rpm_cmd", 10,
      std::bind(&ChassisMockNode::wheel_cmd_callback, this, std::placeholders::_1));
    feedback_pub_ = create_publisher<sand_rake_control::msg::WheelRpm>(
      "/mock/wheel_rpm_feedback", 10);
    truth_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/mock/chassis_state", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/chassis/odom_raw", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/mock/path", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    path_.header.frame_id = odom_frame_;
    last_command_time_ = std::chrono::steady_clock::now();
    last_update_time_ = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / simulation_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&ChassisMockNode::update, this));

    RCLCPP_INFO(
      get_logger(),
      "Mock chassis started at %.1f Hz (left_gain=%.3f, right_gain=%.3f, tau=%.3f s).",
      simulation_rate_hz_, left_gain_, right_gain_, motor_time_constant_sec_);
  }

private:
  void wheel_cmd_callback(const sand_rake_control::msg::WheelRpm::SharedPtr msg)
  {
    command_rpm_ = {
      msg->left_front, msg->left_rear, msg->right_front, msg->right_rear};
    command_received_ = true;
    last_command_time_ = std::chrono::steady_clock::now();
  }

  double update_motor(double actual, double target, double dt) const
  {
    if (motor_time_constant_sec_ <= 1.0e-9) {
      return target;
    }
    const double alpha = 1.0 - std::exp(-dt / motor_time_constant_sec_);
    return actual + alpha * (target - actual);
  }

  nav_msgs::msg::Odometry make_odometry(
    const rclcpp::Time & stamp,
    const std::string & child_frame,
    const PlanarState & state,
    const sand_rake_control::BodyTwist & twist) const
  {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = child_frame;
    message.pose.pose.position.x = state.x;
    message.pose.pose.position.y = state.y;
    message.pose.pose.orientation = yaw_quaternion(state.yaw);
    message.twist.twist.linear.x = twist.linear_mps;
    message.twist.twist.angular.z = twist.angular_rps;
    message.pose.covariance[0] = 0.01;
    message.pose.covariance[7] = 0.01;
    message.pose.covariance[35] = 0.02;
    message.twist.covariance[0] = 0.02;
    message.twist.covariance[35] = 0.03;
    return message;
  }

  void update()
  {
    const auto stamp = now();
    const auto update_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(update_time - last_update_time_).count();
    last_update_time_ = update_time;
    dt = std::clamp(dt, 0.0, 0.1);

    const bool timed_out =
      !command_received_ ||
      std::chrono::duration<double>(update_time - last_command_time_).count() >=
      command_timeout_sec_;
    const sand_rake_control::WheelRpmValues safe_command =
      timed_out ? sand_rake_control::WheelRpmValues{} : command_rpm_;

    const sand_rake_control::WheelRpmValues gained_command{
      safe_command.left_front * left_gain_,
      safe_command.left_rear * left_gain_,
      safe_command.right_front * right_gain_,
      safe_command.right_rear * right_gain_};

    actual_rpm_.left_front = update_motor(
      actual_rpm_.left_front, gained_command.left_front, dt);
    actual_rpm_.left_rear = update_motor(
      actual_rpm_.left_rear, gained_command.left_rear, dt);
    actual_rpm_.right_front = update_motor(
      actual_rpm_.right_front, gained_command.right_front, dt);
    actual_rpm_.right_rear = update_motor(
      actual_rpm_.right_rear, gained_command.right_rear, dt);

    const auto true_twist = kinematics_->motor_rpm_to_body_twist(actual_rpm_);
    integrate(truth_state_, true_twist, dt);

    sand_rake_control::WheelRpmValues feedback_rpm{
      actual_rpm_.left_front + noise_distribution_(random_engine_),
      actual_rpm_.left_rear + noise_distribution_(random_engine_),
      actual_rpm_.right_front + noise_distribution_(random_engine_),
      actual_rpm_.right_rear + noise_distribution_(random_engine_)};
    const auto odom_twist = kinematics_->motor_rpm_to_body_twist(feedback_rpm);
    integrate(odom_state_, odom_twist, dt);

    sand_rake_control::msg::WheelRpm feedback;
    feedback.header.stamp = stamp;
    feedback.header.frame_id = base_frame_;
    feedback.left_front = feedback_rpm.left_front;
    feedback.left_rear = feedback_rpm.left_rear;
    feedback.right_front = feedback_rpm.right_front;
    feedback.right_rear = feedback_rpm.right_rear;
    feedback.direction_valid = true;
    feedback_pub_->publish(feedback);

    truth_pub_->publish(
      make_odometry(
        stamp, base_frame_ + "_truth", truth_state_, true_twist));
    const auto odom = make_odometry(
      stamp, base_frame_, odom_state_, odom_twist);
    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = odom_state_.x;
    transform.transform.translation.y = odom_state_.y;
    transform.transform.rotation = yaw_quaternion(odom_state_.yaw);
    tf_broadcaster_->sendTransform(transform);

    if (++path_counter_ >= path_publish_divider_) {
      path_counter_ = 0;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = odom.header;
      pose.pose = odom.pose.pose;
      path_.header.stamp = stamp;
      path_.poses.push_back(pose);
      if (path_.poses.size() > static_cast<std::size_t>(max_path_points_)) {
        path_.poses.erase(path_.poses.begin());
      }
      path_pub_->publish(path_);
    }
  }

  std::unique_ptr<sand_rake_control::DiffDriveKinematics> kinematics_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sand_rake_control::msg::WheelRpm>::SharedPtr wheel_cmd_sub_;
  rclcpp::Publisher<sand_rake_control::msg::WheelRpm>::SharedPtr feedback_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr truth_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sand_rake_control::WheelRpmValues command_rpm_;
  sand_rake_control::WheelRpmValues actual_rpm_;
  PlanarState truth_state_;
  PlanarState odom_state_;
  nav_msgs::msg::Path path_;
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point last_update_time_;
  bool command_received_{false};
  int path_counter_{0};

  double simulation_rate_hz_{50.0};
  double command_timeout_sec_{0.5};
  double left_gain_{1.0};
  double right_gain_{1.0};
  double motor_time_constant_sec_{0.0};
  double rpm_noise_stddev_{0.0};
  int path_publish_divider_{5};
  int max_path_points_{5000};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::mt19937 random_engine_;
  std::normal_distribution<double> noise_distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ChassisMockNode>());
  rclcpp::shutdown();
  return 0;
}

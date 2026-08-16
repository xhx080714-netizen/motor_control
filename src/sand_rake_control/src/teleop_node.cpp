#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
  TeleopNode()
  : Node("teleop_node")
  {
    // 1. 声明参数
    const double max_linear_speed = std::abs(
      this->declare_parameter<double>("max_linear_speed_mps", 0.15));
    const double max_angular_speed = std::abs(
      this->declare_parameter<double>("max_angular_speed_rps", 0.80));
    linear_speed_ = std::abs(
      this->declare_parameter<double>("linear_speed", 0.10));
    angular_speed_ = std::abs(
      this->declare_parameter<double>("angular_speed", 0.50));
    key_timeout_sec_ =
      this->declare_parameter<double>("key_timeout_sec", 0.5);

    if (max_linear_speed <= 0.0 || max_angular_speed <= 0.0 ||
      key_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "teleop speed limits and key timeout must be positive");
    }
    if (linear_speed_ > max_linear_speed) {
      RCLCPP_WARN(
        this->get_logger(),
        "linear_speed exceeds configured commissioning limit; clamping");
      linear_speed_ = max_linear_speed;
    }
    if (angular_speed_ > max_angular_speed) {
      RCLCPP_WARN(
        this->get_logger(),
        "angular_speed exceeds configured commissioning limit; clamping");
      angular_speed_ = max_angular_speed;
    }

    // 2. 创建 Publisher
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
      "/teleop/cmd_vel",
      10);

    // 3. 配置终端
    setup_terminal();

    // 4. 20 Hz 定时器
    timer_ =
      this->create_wall_timer(
      50ms,
      std::bind(&TeleopNode::timer_callback, this));

    last_key_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "Teleop node started.");

    RCLCPP_INFO(
      this->get_logger(),
      "W/S: forward/backward | A/D: arc turn | Q/E: rotate | Space: stop");
  }

  ~TeleopNode()
  {
    restore_terminal();
  }

private:
  void setup_terminal()
  {
    if (!isatty(STDIN_FILENO)) {
      RCLCPP_WARN(
        this->get_logger(),
        "stdin is not a terminal; run teleop_node directly in an interactive shell");
      return;
    }

    // 保存原始终端设置
    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
      RCLCPP_ERROR(this->get_logger(), "failed to read terminal settings");
      return;
    }

    struct termios raw = original_termios_;

    // 关闭规范模式和字符回显
    raw.c_lflag &= ~(ICANON | ECHO);

    // read() 不等待字符
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    // 保存原来的文件状态标志
    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ < 0 || tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      RCLCPP_ERROR(this->get_logger(), "failed to configure terminal");
      return;
    }

    // 设置 stdin 为非阻塞
    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      RCLCPP_ERROR(this->get_logger(), "failed to set nonblocking terminal input");
      return;
    }
    terminal_configured_ = true;
  }

  void restore_terminal()
  {
    if (!terminal_configured_) {
      return;
    }
    tcsetattr(
      STDIN_FILENO,
      TCSANOW,
      &original_termios_);

    fcntl(
      STDIN_FILENO,
      F_SETFL,
      original_flags_);
  }

  void timer_callback()
  {
    read_keyboard();

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_key_time_).count();

    if (elapsed > key_timeout_sec_) {
      current_cmd_.linear.x = 0.0;
      current_cmd_.angular.z = 0.0;
    }

    cmd_vel_pub_->publish(current_cmd_);
  }

  void read_keyboard()
  {
    char key;

    while (read(STDIN_FILENO, &key, 1) > 0) {
      if (process_key(key)) {
        last_key_time_ = std::chrono::steady_clock::now();
      }
    }
  }

  bool process_key(char key)
  {
    switch (key) {
      case 'w':
      case 'W':
        current_cmd_.linear.x = linear_speed_;
        current_cmd_.angular.z = 0.0;
        return true;

      case 's':
      case 'S':
        current_cmd_.linear.x = -linear_speed_;
        current_cmd_.angular.z = 0.0;
        return true;

      case 'a':
      case 'A':
        current_cmd_.linear.x = linear_speed_;
        current_cmd_.angular.z = angular_speed_;
        return true;

      case 'd':
      case 'D':
        current_cmd_.linear.x = linear_speed_;
        current_cmd_.angular.z = -angular_speed_;
        return true;

      case 'q':
      case 'Q':
        current_cmd_.linear.x = 0.0;
        current_cmd_.angular.z = angular_speed_;
        return true;

      case 'e':
      case 'E':
        current_cmd_.linear.x = 0.0;
        current_cmd_.angular.z = -angular_speed_;
        return true;

      case ' ':
        current_cmd_.linear.x = 0.0;
        current_cmd_.angular.z = 0.0;
        return true;

      default:
        return false;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_key_time_;

  geometry_msgs::msg::Twist current_cmd_;

  double linear_speed_;
  double angular_speed_;
  double key_timeout_sec_;

  struct termios original_termios_ {};
  int original_flags_{0};
  bool terminal_configured_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TeleopNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

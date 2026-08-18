#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sand_rake_interfaces/msg/safety_event.hpp"

class SafetyEventPublisher : public rclcpp::Node
{
public:
  SafetyEventPublisher()
  : Node("safety_event_cpp_publisher")
  {
    publisher_ =
      this->create_publisher<sand_rake_interfaces::msg::SafetyEvent>(
      "/safety/event", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        publish_event();
      });
  }

private:
  void publish_event()
  {
    sand_rake_interfaces::msg::SafetyEvent message;

    message.stamp = this->now();
    message.source_id = "safety_event_cpp_test";
    message.stop = true;
    message.reason =
      sand_rake_interfaces::msg::SafetyEvent::REASON_LASER;

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing safety event: stop=%s, reason=%u",
      message.stop ? "true" : "false",
      message.reason);

    publisher_->publish(message);
  }

  rclcpp::Publisher<
    sand_rake_interfaces::msg::SafetyEvent
  >::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<SafetyEventPublisher>());

  rclcpp::shutdown();

  return 0;
}

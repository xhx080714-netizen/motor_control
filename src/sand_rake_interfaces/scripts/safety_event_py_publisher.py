#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sand_rake_interfaces.msg import SafetyEvent


class SafetyEventPublisher(Node):

    def __init__(self):
        super().__init__('safety_event_py_publisher')

        self.publisher_ = self.create_publisher(
            SafetyEvent,
            '/safety/event',
            10
        )

        self.timer_ = self.create_timer(
            1.0,
            self.publish_event
        )

    def publish_event(self):
        message = SafetyEvent()

        message.stamp = self.get_clock().now().to_msg()
        message.stop = True
        message.reason = SafetyEvent.REASON_TIMEOUT

        self.get_logger().info(
            f'Publishing safety event: '
            f'stop={message.stop}, reason={message.reason}'
        )

        self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = SafetyEventPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

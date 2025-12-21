#!/usr/bin/env python3
"""
ROS 2 Node with Parameters
Configure behavior at runtime without code changes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters with default values
        self.declare_parameter('message_prefix', 'Data')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('message_count', 100)

        # Get parameter values
        self.prefix = self.get_parameter('message_prefix').value
        self.rate = self.get_parameter('publish_rate').value
        self.count = self.get_parameter('message_count').value

        # Log configuration
        self.get_logger().info(
            f'Configured: Prefix={self.prefix}, Rate={self.rate}Hz, Count={self.count}')

        # Create publisher
        self.publisher = self.create_publisher(String, 'topic', 10)

        # Create timer with configured rate
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Publish with configured prefix
        if self.counter < self.count:
            msg = String()
            msg.data = f'{self.prefix}: {self.counter}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')
            self.counter += 1
        else:
            self.get_logger().info(f'Reached message count limit ({self.count})')


def main(args=None):
    rclpy.init(args=args)
    parameterized_node = ParameterizedNode()
    rclpy.spin(parameterized_node)
    parameterized_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

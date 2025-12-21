#!/usr/bin/env python3
"""
Simple ROS 2 Lifecycle Node
Demonstrates state transitions: Unconfigured → Inactive → Active → Finalized
"""

import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.lifecycle import State
from std_msgs.msg import String


class MinimalLifecycleNode(Node):
    def __init__(self):
        # Use Node instead of LifecycleNode for simplicity in examples
        super().__init__('minimal_lifecycle_node')
        self.publisher = None
        self.get_logger().info('Node created (Unconfigured state)')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Called: Unconfigured → Inactive
        # Allocate resources here
        self.get_logger().info('Configuring...')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.get_logger().info('Configuration complete (now Inactive)')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Called: Inactive → Active
        # Start processing
        self.get_logger().info('Activating...')
        self.get_logger().info('Node is now Active and processing')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Called: Active → Inactive
        # Stop processing (but keep resources)
        self.get_logger().info('Deactivating...')
        self.get_logger().info('Node paused but not cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Called: Inactive → Unconfigured
        # Release resources
        self.get_logger().info('Cleaning up...')
        if self.publisher is not None:
            self.destroy_publisher(self.publisher)
        self.get_logger().info('Cleanup complete (now Unconfigured)')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    minimal_lifecycle_node = MinimalLifecycleNode()
    rclpy.spin(minimal_lifecycle_node)
    minimal_lifecycle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

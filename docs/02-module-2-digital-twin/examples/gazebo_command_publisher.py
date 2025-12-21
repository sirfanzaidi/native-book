#!/usr/bin/env python3
"""
Send Joint Commands to Gazebo
Control robot arm in simulation with smooth motion
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10)

        # Timer to send commands every 100ms
        self.timer = self.create_timer(0.1, self.publish_commands)
        self.time_step = 0
        self.get_logger().info('Joint command publisher started')

    def publish_commands(self):
        """Publish joint commands to control Gazebo robot"""
        # Create command message
        msg = Float64MultiArray()

        # Generate smooth sine wave motion
        angle = math.sin(self.time_step / 50.0) * math.pi / 2.0

        # Set joint positions (in radians)
        # 3 joints: base, elbow, wrist
        msg.data = [angle, angle / 2.0, 0.0]

        # Publish to Gazebo
        self.joint_cmd_pub.publish(msg)

        # Log command for verification
        self.get_logger().info(
            f'Command: joint1={angle:.3f}rad, '
            f'joint2={angle/2:.3f}rad, joint3=0.000rad')

        self.time_step += 1

        # Stop after 20 steps
        if self.time_step >= 20:
            self.get_logger().info('Motion sequence complete')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    publisher = JointCommandPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

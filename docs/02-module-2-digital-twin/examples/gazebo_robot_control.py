#!/usr/bin/env python3
"""
Gazebo Robot Arm Control
Publishes joint commands to move robot arm in simulation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time


class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')

        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10)

        # Timer to periodically send commands
        self.timer = self.create_timer(0.5, self.control_callback)
        self.step = 0
        self.get_logger().info('Robot arm controller started')

    def control_callback(self):
        """Send joint commands to move the arm"""
        # Create message with joint positions
        msg = Float64MultiArray()

        # Calculate sine wave motion (0 to π radians)
        angle = math.sin(self.step / 10.0) * math.pi / 2.0
        msg.data = [angle]  # joint1 position

        # Publish command
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(
            f'Joint command: joint1={angle:.2f} rad ({math.degrees(angle):.1f}°)')

        self.step += 1

        # Stop after 20 commands
        if self.step >= 20:
            self.get_logger().info('Motion complete')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

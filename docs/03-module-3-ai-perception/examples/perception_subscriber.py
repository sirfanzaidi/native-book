#!/usr/bin/env python3
"""
Perception Result Subscriber
Receives perception pipeline output and makes robot decisions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32


class PerceptionSubscriber(Node):
    def __init__(self):
        super().__init__('perception_subscriber')

        # Subscribe to perception pipeline results
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/detected_objects',
            self.pose_callback,
            10)

        self.id_sub = self.create_subscription(
            Int32,
            '/tracked_id',
            self.id_callback,
            10)

        # Publish motor/movement commands based on perception
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.current_target = None
        self.current_id = None
        self.get_logger().info('Perception subscriber started')

    def pose_callback(self, msg):
        """Receive detected object pose"""
        self.current_target = msg
        self.get_logger().info(
            f'Target detected at: x={msg.pose.position.x:.2f}m, '
            f'y={msg.pose.position.y:.2f}m, '
            f'z={msg.pose.position.z:.2f}m')

        # Make decision based on target location
        self._decide_action(msg)

    def id_callback(self, msg):
        """Receive tracked object ID"""
        self.current_id = msg.data
        self.get_logger().info(f'Tracking object ID: {self.current_id}')

    def _decide_action(self, target_pose):
        """Make robot decision based on perceived target"""
        # Extract target position
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        z = target_pose.pose.position.z

        # Simple decision logic:
        # If target in front and close → move toward it
        # If target to side → turn
        # If target above/below → move arm

        cmd = Twist()

        # Forward/backward motion (x-axis)
        if x > 0.7:
            cmd.linear.x = 0.2  # Move closer
        elif x < 0.3:
            cmd.linear.x = -0.1  # Move back
        else:
            cmd.linear.x = 0.0  # Stay

        # Lateral motion (y-axis)
        if y > 0.2:
            cmd.angular.z = 0.2  # Turn right
        elif y < -0.2:
            cmd.angular.z = -0.2  # Turn left
        else:
            cmd.angular.z = 0.0  # Face target

        # Publish command
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'Decision: forward={cmd.linear.x:.2f}m/s, '
            f'turn={cmd.angular.z:.2f}rad/s')


def main(args=None):
    rclpy.init(args=args)
    subscriber = PerceptionSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

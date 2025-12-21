#!/usr/bin/env python3
"""
Subscribe to Gazebo Sensor Data
Receive sensor readings (joint states, LiDAR) from simulated robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan


class SensorSubscriber(Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # Subscribe to joint states (published by Gazebo)
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Subscribe to LiDAR scan (if robot has LiDAR)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.lidar_callback,
            10)

        self.get_logger().info('Sensor subscriber started, waiting for data...')

    def joint_state_callback(self, msg):
        """Receive joint positions from Gazebo simulation"""
        # msg.name: list of joint names
        # msg.position: list of joint angles (radians)
        # msg.velocity: list of joint velocities
        joint_angles = ', '.join([f'{p:.3f}' for p in msg.position])
        self.get_logger().info(
            f'Joint positions: [{joint_angles}] rad')

    def lidar_callback(self, msg):
        """Receive LiDAR range data from Gazebo simulation"""
        # msg.ranges: array of distance measurements
        # Find closest obstacle
        if msg.ranges:
            closest_range = min(msg.ranges)
            self.get_logger().info(
                f'LiDAR: Closest obstacle {closest_range:.2f}m away')


def main(args=None):
    rclpy.init(args=args)
    subscriber = SensorSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

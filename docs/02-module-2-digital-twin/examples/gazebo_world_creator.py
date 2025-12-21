#!/usr/bin/env python3
"""
Simple Gazebo World Creation
Spawns a robot arm in a Gazebo world via ROS 2 service
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


class GazeboWorldCreator(Node):
    def __init__(self):
        super().__init__('gazebo_world_creator')

        # Wait for Gazebo spawn service to be available
        self.spawn_client = self.create_client(
            SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')

        self.get_logger().info('Gazebo ready! Spawning robot arm...')
        self.create_robot_arm()

    def create_robot_arm(self):
        """Create and spawn a simple 2-link robot arm"""
        # Simple robot arm URDF: base link + arm link + joint
        arm_urdf = '''<?xml version="1.0"?>
        <robot name="simple_arm">
            <link name="base">
                <inertial>
                    <mass value="1.0"/>
                    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
                </inertial>
                <visual>
                    <geometry>
                        <box size="0.1 0.1 0.1"/>
                    </geometry>
                </visual>
            </link>

            <link name="link1">
                <inertial>
                    <mass value="0.5"/>
                    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
                </inertial>
                <visual>
                    <geometry>
                        <cylinder length="0.3" radius="0.05"/>
                    </geometry>
                </visual>
            </link>

            <joint name="joint1" type="revolute">
                <parent link="base"/>
                <child link="link1"/>
                <axis xyz="0 0 1"/>
                <limit lower="0" upper="3.14" effort="10" velocity="1"/>
            </joint>
        </robot>'''

        # Create spawn request
        request = SpawnEntity.Request()
        request.name = 'simple_arm'
        request.xml = arm_urdf
        request.initial_pose = Pose()
        request.initial_pose.position.z = 0.5  # Above ground

        # Call spawn service
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        # Check result
        if future.result() is not None:
            self.get_logger().info('✓ Robot arm spawned successfully!')
            self.get_logger().info('  Name: simple_arm')
            self.get_logger().info('  Position: (0, 0, 0.5)')
        else:
            self.get_logger().error('✗ Failed to spawn robot arm')


def main(args=None):
    rclpy.init(args=args)
    world_creator = GazeboWorldCreator()
    rclpy.spin(world_creator)
    world_creator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

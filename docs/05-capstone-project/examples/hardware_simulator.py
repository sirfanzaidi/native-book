#!/usr/bin/env python3
"""
Hardware Simulator: Bridges ROS 2 Commands to Gazebo Physics
Simulates robot hardware response for testing without real robot
Integrates Modules 1 (ROS 2), 2 (Gazebo), and 3 (Perception feedback)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.timer import Timer
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Header, Float32
import numpy as np
import time
from typing import Dict, List, Tuple


class SimplePhysicsEngine:
    """
    Simplified physics simulation
    In production: Use Gazebo or CoppeliaSim
    """

    def __init__(self):
        # Robot state
        self.joint_angles = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow': 0.0,
            'wrist': 0.0,
            'gripper': 0.0
        }

        self.joint_velocities = {k: 0.0 for k in self.joint_angles}
        self.gripper_force = 0.0
        self.time_step = 0.01  # 10ms simulation step

    def update(self, cmd_vel: Twist, dt: float = 0.01):
        """
        Update robot state based on command
        Simple first-order dynamics
        """
        # Command limits
        max_angular_vel = 1.0  # rad/s
        max_linear_vel = 0.5   # m/s

        # Map Twist to joint velocities with acceleration limits
        cmd_shoulder_pan = np.clip(cmd_vel.linear.x, -max_angular_vel, max_angular_vel)
        cmd_shoulder_lift = np.clip(cmd_vel.linear.y, -max_angular_vel, max_angular_vel)
        cmd_elbow = np.clip(cmd_vel.linear.z, -max_angular_vel, max_angular_vel)
        cmd_wrist = np.clip(cmd_vel.angular.z, -max_angular_vel, max_angular_vel)

        # Apply acceleration constraints (smooth motion)
        max_accel = 0.5
        for joint, cmd_vel_val in [
            ('shoulder_pan', cmd_shoulder_pan),
            ('shoulder_lift', cmd_shoulder_lift),
            ('elbow', cmd_elbow),
            ('wrist', cmd_wrist)
        ]:
            # Clamp acceleration
            current_vel = self.joint_velocities[joint]
            accel = (cmd_vel_val - current_vel) / dt
            accel = np.clip(accel, -max_accel, max_accel)

            # Update velocity
            self.joint_velocities[joint] += accel * dt

        # Integrate velocities to positions (with angle wrapping)
        for joint in self.joint_angles:
            self.joint_angles[joint] += self.joint_velocities[joint] * dt
            # Wrap to [-pi, pi]
            self.joint_angles[joint] = np.arctan2(
                np.sin(self.joint_angles[joint]),
                np.cos(self.joint_angles[joint]))

    def get_end_effector_pose(self) -> Tuple[float, float, float]:
        """
        Forward kinematics: compute end effector position
        Simplified 3-link arm model
        """
        # Link lengths
        l1, l2, l3 = 0.5, 0.4, 0.3

        # Forward kinematics
        theta1 = self.joint_angles['shoulder_pan']
        theta2 = self.joint_angles['shoulder_lift']
        theta3 = self.joint_angles['elbow']

        x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
        y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2)
        z = l3 * np.sin(theta3)

        return (x, y, z)

    def check_collision_with_object(self, object_pos: Tuple[float, float, float]) -> bool:
        """Check if gripper collides with object"""
        ee_pos = self.get_end_effector_pose()
        distance = np.sqrt(sum((a - b) ** 2 for a, b in zip(ee_pos, object_pos)))

        # Collision threshold: 10cm
        collision_distance = 0.1
        return distance < collision_distance


class HardwareSimulator(Node):
    """
    Simulates robot hardware in ROS 2
    Bridges Module 1 (ROS 2 control) with Module 2 (Physics simulation)
    Provides feedback as if it were real hardware
    """

    def __init__(self):
        super().__init__('hardware_simulator')

        # Callback group for concurrent subscriptions
        self.cb_group = ReentrantCallbackGroup()

        # Subscription: Motion commands from task orchestrator (Module 1)
        self.motion_cmd_sub = self.create_subscription(
            Twist,
            '/robot/motion_cmd',
            self.on_motion_command,
            10,
            callback_group=self.cb_group)

        # Publications: Sensor feedback (Module 1 style topics)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)

        self.end_effector_pub = self.create_publisher(
            PoseStamped,
            '/end_effector/pose',
            10)

        # Simulation outputs (Module 3 style perception)
        self.detection_pub = self.create_publisher(
            PoseStamped,
            '/perception/detected_objects',
            10)

        # Camera image output (simulated)
        self.camera_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10)

        # Physics state
        self.physics = SimplePhysicsEngine()
        self.last_cmd = Twist()
        self.time_step = 0.01  # 10ms simulation step

        # Simulated scene objects
        self.simulated_parts = {
            'panel_left': {'pos': (0.5, -0.2, 0.8), 'id': 0},
            'panel_right': {'pos': (0.5, 0.2, 0.8), 'id': 1},
            'shelf_1': {'pos': (0.0, 0.0, 0.6), 'id': 2}
        }

        # Gripper state
        self.gripper_open = True
        self.grasped_object = None

        # Create timer for simulation loop
        self.sim_timer: Timer = self.create_timer(
            self.time_step,
            self.simulation_step,
            callback_group=self.cb_group)

        self.frame_count = 0
        self.last_print_time = time.time()

        self.get_logger().info('🎮 Hardware Simulator started (10ms timestep)')
        self.get_logger().info('   Simulating robotic arm with physics')

    def on_motion_command(self, msg: Twist):
        """Receive motion command from task orchestrator"""
        self.last_cmd = msg
        self.get_logger().debug(f'Command: vx={msg.linear.x:.2f} ωz={msg.angular.z:.2f}')

    def simulation_step(self):
        """
        Execute one simulation step
        Called at 100Hz (10ms timestep)
        """
        # Update physics with latest command
        self.physics.update(self.last_cmd, self.time_step)

        # Publish joint state feedback (Module 1: typical robot driver output)
        self.publish_joint_state()

        # Publish end effector pose (Module 1: forward kinematics output)
        self.publish_end_effector_pose()

        # Publish detected objects (Module 3: perception simulation)
        self.publish_detections()

        # Publish simulated camera image (Module 3: vision input)
        self.publish_camera_frame()

        # Periodic status logging
        self.frame_count += 1
        if time.time() - self.last_print_time > 1.0:
            fps = self.frame_count
            self.frame_count = 0
            self.last_print_time = time.time()

            # Log state periodically
            angles = self.physics.joint_angles
            self.get_logger().info(
                f'Sim: pan={angles["shoulder_pan"]:.2f} '
                f'lift={angles["shoulder_lift"]:.2f} '
                f'elbow={angles["elbow"]:.2f} | {fps}Hz')

    def publish_joint_state(self):
        """
        Publish joint state as real robot would
        Module 1: Standard ROS 2 joint state topic
        """
        msg = JointState()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='world')

        msg.name = list(self.physics.joint_angles.keys())
        msg.position = list(self.physics.joint_angles.values())
        msg.velocity = list(self.physics.joint_velocities.values())
        msg.effort = [0.0] * len(msg.name)  # Would be actual torques

        self.joint_state_pub.publish(msg)

    def publish_end_effector_pose(self):
        """
        Publish end effector pose from forward kinematics
        Module 1: Computed pose based on joint angles
        """
        ee_pos = self.physics.get_end_effector_pose()

        msg = PoseStamped()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='base_link')

        msg.pose.position.x = ee_pos[0]
        msg.pose.position.y = ee_pos[1]
        msg.pose.position.z = ee_pos[2]

        # Orientation (simplified)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        self.end_effector_pub.publish(msg)

    def publish_detections(self):
        """
        Publish detected objects in scene
        Module 3: Simulated perception pipeline output
        """
        ee_pos = self.physics.get_end_effector_pose()

        # Check which objects are "visible" to simulated camera
        # (within 1.5m and in front of robot)
        for part_name, part_info in self.simulated_parts.items():
            part_pos = part_info['pos']

            # Distance to end effector
            distance = np.sqrt(sum((a - b) ** 2 for a, b in zip(ee_pos, part_pos)))

            # Publish if within sensing range
            if distance < 1.5:
                msg = PoseStamped()
                msg.header = Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=part_name)

                msg.pose.position.x = part_pos[0]
                msg.pose.position.y = part_pos[1]
                msg.pose.position.z = part_pos[2]

                msg.pose.orientation.w = 1.0

                self.detection_pub.publish(msg)

    def publish_camera_frame(self):
        """
        Publish simulated camera frame
        Module 3: Vision input (usually processed by GPU perception)
        """
        # Create dummy image message (in real system: actual camera frames)
        msg = Image()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='camera_link')

        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 640 * 3
        msg.data = bytes(480 * 640 * 3)  # Dummy RGB data

        self.camera_pub.publish(msg)

    def check_grasp_stability(self) -> bool:
        """Check if grasped object will stay in gripper"""
        if not self.grasped_object:
            return False

        # Simulated grasp force check
        # In real system: Measure from force/torque sensor
        simulated_grasp_force = 50  # Newtons

        return simulated_grasp_force > 20  # Min force for stability


def main(args=None):
    rclpy.init(args=args)
    simulator = HardwareSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

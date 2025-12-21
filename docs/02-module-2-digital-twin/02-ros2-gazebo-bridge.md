---
id: 05-ros2-gazebo-bridge
title: "Chapter 5 - ROS 2 and Gazebo Integration"
sidebar_label: "Chapter 5: ROS 2 + Gazebo"
---

# Chapter 5: ROS 2 and Gazebo Integration

## Introduction

In Chapter 4, you learned how to create Gazebo worlds and robots. But how do you **control robots in simulation using ROS 2**?

This chapter shows you the critical bridge: connecting ROS 2 nodes to Gazebo robots via **plugins and middleware**.

### The Bridge Architecture

```
[ROS 2 Control Node]         [Gazebo Simulator]
        |                           |
        |--- Publish Commands ------|
        |   /joint_commands topic   |
        |                    [Robot Arm]
        |                    [Physics]
        |                    [Sensors]
        |<-- Publish Sensor Data ----|
        |   /joint_states topic      |
        |   /camera/image_raw topic  |
        |   /lidar/scan topic        |
```

The beauty: **Your ROS 2 code doesn't know or care if it's controlling a real robot or a simulated one.**

### Why This Matters

Being able to:
1. Develop control algorithms in simulation
2. Test them thoroughly
3. Deploy to real robots without code changes

This is the core workflow of professional robotics development.

**Source**: Open Robotics. (2024). *Gazebo and ROS 2 Integration*. Retrieved from https://gazebosim.org/docs/garden/ros2_integration/

## How ROS 2 - Gazebo Integration Works

### Three Key Components

1. **Gazebo Plugins**: Add ROS 2 publishing/subscribing to simulation
2. **ROS 2 Topics**: Bridge between control code and simulation
3. **Messages**: Standard formats for commands and sensor data

### Plugin System

Gazebo plugins allow you to:
- Publish sensor data to ROS 2 topics
- Subscribe to ROS 2 topics for commands
- Execute custom logic during simulation steps

```xml
<plugin filename="libgazebo_ros_joint_state_publisher.so"
        name="joint_state_publisher">
    <topic_name>joint_states</topic_name>
    <update_rate>10</update_rate>
</plugin>
```

This simple XML tells Gazebo to publish joint state information on the ROS 2 topic `joint_states` at 10 Hz.

## Publishing Sensor Data from Gazebo

Gazebo sensors (cameras, LiDAR, IMU) automatically publish to ROS 2 topics via plugins.

### Subscribing to Sensor Data

```python
#!/usr/bin/env python3
"""
Subscribe to Gazebo Sensor Data
Receive sensor readings from simulated robot
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import TransformStamped


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

        self.get_logger().info('Sensor subscriber started')

    def joint_state_callback(self, msg):
        # Receive joint positions from Gazebo
        self.get_logger().info(
            f'Joint positions: {[f"{p:.2f}" for p in msg.position]}')

    def lidar_callback(self, msg):
        # Receive LiDAR range data from Gazebo
        closest_range = min(msg.ranges)
        self.get_logger().info(
            f'Closest obstacle: {closest_range:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    subscriber = SensorSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What's happening**:
1. Create subscription to `/joint_states` topic (published by Gazebo)
2. In callback, receive sensor data from simulation
3. Log joint positions and LiDAR ranges
4. Same code works with real robots!

### Expected Output

**Terminal**:
```
[sensor_subscriber] Sensor subscriber started
[sensor_subscriber] Joint positions: ['0.50', '1.05', '0.00']
[sensor_subscriber] Closest obstacle: 2.34m
[sensor_subscriber] Joint positions: ['0.51', '1.04', '0.02']
[sensor_subscriber] Closest obstacle: 2.31m
```

## Sending Commands to Gazebo

To control the simulated robot, publish to ROS 2 command topics.

### Publishing Joint Commands

```python
#!/usr/bin/env python3
"""
Send Joint Commands to Gazebo
Control robot arm in simulation
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

        # Timer to send commands periodically
        self.timer = self.create_timer(0.1, self.publish_commands)
        self.time_step = 0
        self.get_logger().info('Joint command publisher started')

    def publish_commands(self):
        # Create command message
        msg = Float64MultiArray()

        # Sine wave motion for smooth robot movement
        angle = math.sin(self.time_step / 50.0) * math.pi / 2.0

        # Set joint positions (in radians)
        msg.data = [angle, angle / 2.0, 0.0]  # 3 joints

        # Publish to Gazebo
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(
            f'Sent command: joints=[{angle:.2f}, {angle/2:.2f}, 0.00] rad')

        self.time_step += 1


def main(args=None):
    rclpy.init(args=args)
    publisher = JointCommandPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What's happening**:
1. Create publisher to `/joint_commands` topic
2. Every 100ms, generate a command (sine wave motion)
3. Publish joint positions (in radians)
4. Gazebo receives and executes the commands
5. Robot arm moves realistically with physics

### Expected Output

**Terminal**:
```
[joint_command_publisher] Joint command publisher started
[joint_command_publisher] Sent command: joints=[0.00, 0.00, 0.00] rad
[joint_command_publisher] Sent command: joints=[0.06, 0.03, 0.00] rad
[joint_command_publisher] Sent command: joints=[0.12, 0.06, 0.00] rad
```

**Gazebo Window**:
- Robot arm smoothly moves
- Joint angles follow the sine wave
- Physics realistically simulates arm dynamics

## Real vs Simulated: The Same Code

This is the key insight:

**Controlling a real robot**:
```python
joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
msg.data = [angle, angle / 2.0, 0.0]
joint_cmd_pub.publish(msg)  # Talks to real robot motors
```

**Controlling simulated robot**:
```python
joint_cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
msg.data = [angle, angle / 2.0, 0.0]
joint_cmd_pub.publish(msg)  # Talks to Gazebo simulator
```

**Exactly the same code!** The only difference is which ROS 2 node is subscribed to `/joint_commands`:
- In simulation: Gazebo plugin receives the command
- On hardware: Robot driver receives the command

## Topics: Standard Message Types

Common topics for robotics control:

| Topic | Message Type | Direction | Purpose |
|-------|--------------|-----------|---------|
| `/joint_states` | JointState | Gazebo → Control | Read joint angles |
| `/joint_commands` | Float64MultiArray | Control → Gazebo | Command joint angles |
| `/cmd_vel` | Twist | Control → Gazebo | Command linear/angular velocity |
| `/sensor/imu` | Imu | Gazebo → Control | IMU acceleration/rotation |
| `/camera/image_raw` | Image | Gazebo → Control | Camera image |
| `/lidar/scan` | LaserScan | Gazebo → Control | LiDAR range data |

Using standard message types means code works seamlessly with real robots and other simulators.

**Source**: ROS 2 Messages. (2024). Retrieved from https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html

## Summary

In this chapter, you learned:

- **Bridge architecture**: How ROS 2 talks to Gazebo
- **Plugins**: Enable ROS 2 communication in simulation
- **Standard topics**: `/joint_states`, `/joint_commands`, sensor topics
- **Sensor data**: Subscribe to simulate sensor readings
- **Commands**: Publish to control simulated robots
- **Sim-real identity**: Same code works everywhere

**Key Takeaway**: With ROS 2 and Gazebo, you develop and test robot control algorithms without expensive hardware.

**Next steps**: In Chapter 6, we'll explore advanced simulation techniques: physics tuning, realistic sensors, and sim-to-real transfer.

---

**References**:

- Open Robotics. (2024). *Gazebo and ROS 2 Integration*. Retrieved from https://gazebosim.org/docs/garden/ros2_integration/
- ROS 2 Standard Interfaces. (2024). Retrieved from https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html
- Gazebo ROS Package. (2024). *gazebo_ros*. Retrieved from https://github.com/ros-simulation/gazebo_ros_pkg_v11
- ROS 2 Control Framework. (2024). Retrieved from https://github.com/ros-controls/ros2_control

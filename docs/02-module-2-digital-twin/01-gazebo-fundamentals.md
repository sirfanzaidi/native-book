---
id: 04-gazebo-fundamentals
title: "Chapter 4 - Gazebo Simulation Fundamentals"
sidebar_label: "Chapter 4: Gazebo Basics"
---

# Chapter 4: Gazebo Simulation Fundamentals

## Introduction

In Module 1, you learned how to control robots using ROS 2 nodes and messaging. But how do you **test and develop without a physical robot**?

The answer is **simulation**. Gazebo is the open-source 3D robotics simulator that works seamlessly with ROS 2.

### Why Simulation?

Simulating robots before deployment offers critical advantages:

- **Cost**: No risk of damaging expensive hardware during development
- **Speed**: Test algorithms 10x faster than real robots
- **Iteration**: Try thousands of ideas without physical constraints
- **Debugging**: Inspect internal robot state easily
- **Safety**: Test dangerous behaviors safely in simulation
- **Repeatability**: Identical test conditions every run

**Example**: Training a humanoid robot to walk takes weeks of simulation before running on hardware.

**Source**: Gazebo Official Documentation. (2024). *Why Gazebo?*. Retrieved from https://gazebosim.org/docs/

### What is Gazebo?

Gazebo is a **3D physics simulator** that models:

- **Physics**: Gravity, friction, collision detection
- **Sensors**: Cameras, LiDAR, IMU, distance sensors
- **Actuators**: Motors, joints, gripper forces
- **Environment**: Terrain, obstacles, lighting
- **Dynamics**: Realistic robot movement and interaction

It integrates with ROS 2 via plugins, allowing the same ROS 2 code to work in simulation or on real robots.

### Simulation vs Reality

```
[ROS 2 Control Code]
        ↓
    ┌───────────────────┐
    │  In Simulation    │    In Reality
    ├───────────────────┤
    │  Gazebo Physics   │    Physical Robot
    │  Virtual Robot    │    Real Motors
    │  Fake Sensors     │    Real Sensors
    └───────────────────┘
        ↓
    [Same ROS 2 Topics/Services]
```

The beauty of ROS 2: your control code doesn't change whether running in Gazebo or on real hardware.

## Understanding 3D Worlds

A Gazebo **world** contains:

1. **Models**: Robots, objects (URDF or SDF format)
2. **Physics Engine**: Simulates gravity and collisions
3. **Sensors**: Cameras, LiDAR that publish to ROS 2 topics
4. **Plugins**: Connect simulation to ROS 2

### World Structure

```
World
├── Robot (URDF model)
│   ├── Links (body parts)
│   ├── Joints (connections)
│   ├── Sensors
│   │   ├── Camera
│   │   ├── LiDAR
│   │   └── IMU
│   └── Actuators
│       ├── Motors
│       └── Grippers
├── Ground Plane
├── Objects
│   ├── Cube
│   └── Sphere
└── Physics
    ├── Gravity
    └── Friction
```

## Creating Your First Gazebo World

### Simple World with a Robot Arm

We'll create a world with a basic robot arm in Gazebo:

```python
#!/usr/bin/env python3
"""
Simple Gazebo World Creation
Spawns a robot arm in a Gazebo world
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import os


class GazeboWorldCreator(Node):
    def __init__(self):
        super().__init__('gazebo_world_creator')

        # Wait for Gazebo to be ready
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo spawn service...')

        self.get_logger().info('Gazebo ready!')
        self.create_robot_arm()

    def create_robot_arm(self):
        """Create and spawn a simple robot arm"""
        # Simple robot arm URDF (2 links, 1 joint)
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

        # Spawn the robot in Gazebo
        request = SpawnEntity.Request()
        request.name = 'simple_arm'
        request.xml = arm_urdf
        request.initial_pose = Pose()
        request.initial_pose.position.z = 0.5  # Above ground

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Robot arm spawned successfully!')
        else:
            self.get_logger().error('Failed to spawn robot arm')


def main(args=None):
    rclpy.init(args=args)
    world_creator = GazeboWorldCreator()
    rclpy.spin(world_creator)
    world_creator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What's happening**:
1. Create a Gazebo spawn client
2. Define a robot arm in URDF format (XML)
3. Spawn the robot at position (0, 0, 0.5)
4. Log success/failure

### Expected Output

**Terminal**:
```
[gazebo_world_creator] Waiting for Gazebo spawn service...
[gazebo_world_creator] Waiting for Gazebo spawn service...
[gazebo_world_creator] Gazebo ready!
[gazebo_world_creator] Robot arm spawned successfully!
```

**Gazebo Window**:
- 3D view opens
- Ground plane visible
- Robot arm appears at center

## URDF: Robot Definition Format

URDF (Unified Robot Description Format) is XML that defines:

```xml
<robot name="my_robot">
    <!-- Links are rigid bodies -->
    <link name="base">...</link>
    <link name="arm">...</link>

    <!-- Joints connect links -->
    <joint name="shoulder" type="revolute">
        <parent link="base"/>
        <child link="arm"/>
        <axis xyz="0 0 1"/>
        <limit lower="0" upper="3.14" effort="10" velocity="1"/>
    </joint>
</robot>
```

**Key concepts**:
- **Links**: Rigid bodies (have mass, geometry, visual shape)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Axes**: Rotation direction (x, y, or z)
- **Limits**: Joint angle ranges and torque limits

### Common Joint Types

| Type | Motion | Example |
|------|--------|---------|
| revolute | Rotating | Shoulder, elbow |
| prismatic | Linear | Gripper fingers, slide |
| fixed | No motion | Attached tools |
| continuous | Unlimited rotation | Wheels |

**Source**: ROS URDF Documentation. (2024). Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Introduction.html

## Running Gazebo with ROS 2

To launch Gazebo and create worlds:

```bash
# Start Gazebo with ROS 2 support
gazebo --verbose world.sdf

# In another terminal, spawn objects via ROS 2
ros2 service call /spawn_entity gazebo_msgs/SpawnEntity \
  "{name: 'my_robot', xml: '$(cat robot.urdf)'}"
```

## Summary

In this chapter, you learned:

- **Why simulation**: Cost, speed, safety, repeatability
- **What Gazebo simulates**: Physics, sensors, actuators, environments
- **How to create worlds**: Using URDF robot definitions
- **Spawning robots**: Via ROS 2 service calls
- **URDF basics**: Links, joints, axes, limits

**Key Takeaway**: Gazebo lets you develop and test robot control code without expensive hardware.

**Next steps**: In Chapter 5, we'll bridge Gazebo simulation with ROS 2 control, publishing sensor data and receiving commands.

---

**References**:

- Gazebo Official Documentation. (2024). *Gazebo Sim*. Retrieved from https://gazebosim.org/docs/
- ROS URDF Documentation. (2024). *URDF Introduction*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Introduction.html
- Open Robotics. (2024). *Gazebo Tutorials*. Retrieved from https://gazebosim.org/docs/garden/tutorials/
- ROS 2 + Gazebo Integration. (2024). Retrieved from https://github.com/ros-simulation/gazebo_ros_pkg_v11

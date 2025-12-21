---
id: 03-advanced-nodes
title: "Chapter 3 - Advanced ROS 2 Nodes"
sidebar_label: "Chapter 3: Advanced Nodes"
---

# Chapter 3: Advanced ROS 2 Nodes

## Introduction

In Chapters 1 and 2, you created simple nodes that run indefinitely. But real robots need more sophisticated node management:

- How do I cleanly start and stop nodes?
- How do I configure a node at runtime?
- How do I control message delivery guarantees?
- How do I run multiple instances of the same node?

In this chapter, you'll learn advanced node patterns that make ROS 2 systems robust and production-ready.

### Why Advanced Patterns?

Simple nodes work for tutorials, but production systems need:

- **Lifecycle Management**: Start, configure, activate, and deactivate nodes cleanly
- **Quality of Service (QoS)**: Control message reliability and history
- **Namespacing**: Run multiple robots or systems without conflicts
- **Parameters**: Configure nodes without code changes

**Source**: ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Node Lifecycles*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Node-Lifecycle.html

## Lifecycle Nodes

A **lifecycle node** goes through distinct states:

```
[Unconfigured] → [Inactive] → [Active] → [Finalized]
      ↑              ↑           ↑
  Created      Configured    Running
```

Each state transition can trigger initialization, cleanup, or error handling code.

### Why Lifecycles Matter

Imagine starting a robot:

1. **Unconfigured**: Node created but not ready (no resources allocated)
2. **Inactive**: Node configured but not processing (ready to start)
3. **Active**: Node running and processing data (controlling robot)
4. **Finalized**: Node cleanup complete (resources released)

This allows coordinated startup sequences. For example:
- Start all perception nodes first (Active)
- Then start control nodes (Active)
- If perception fails, disable control nodes (inactive) without crashing

### Simple Lifecycle Node

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Lifecycle Node
Demonstrates state transitions
"""

import rclpy
from rclpy.lifecycle import Node, TransitionCallbackReturn
from rclpy.lifecycle import State
from rclpy_lifecycle.msg import TransitionEvent


class MinimalLifecycleNode(Node):
    def __init__(self):
        # Use LifecycleNode instead of Node
        super().__init__('minimal_lifecycle_node')
        self.publisher = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Called when transitioning: Unconfigured → Inactive
        # Allocate resources here
        self.get_logger().info('Configuring...')
        self.publisher = self.create_publisher(String, 'topic', 10)
        self.get_logger().info('Configuration complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Called when transitioning: Inactive → Active
        # Start processing
        self.get_logger().info('Activating...')
        # Start timer or begin processing
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Called when transitioning: Active → Inactive
        # Stop processing (but keep resources allocated)
        self.get_logger().info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Called when transitioning: Inactive → Unconfigured
        # Release resources
        self.get_logger().info('Cleaning up...')
        # Destroy publisher, subscriptions, etc.
        return TransitionCallbackReturn.SUCCESS


def main(args=None):
    rclpy.init(args=args)
    minimal_lifecycle_node = MinimalLifecycleNode()
    rclpy.spin(minimal_lifecycle_node)
    minimal_lifecycle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What's happening**:
1. `on_configure()` is called when starting the node (allocate resources)
2. `on_activate()` is called when enabling the node (start processing)
3. `on_deactivate()` is called when pausing the node (stop processing)
4. `on_cleanup()` is called when stopping the node (release resources)

Each callback can return SUCCESS or FAILURE, allowing error handling.

## Parameters: Configurable Nodes

Instead of hardcoding values, use **parameters** to configure nodes:

```python
#!/usr/bin/env python3
"""
ROS 2 Node with Parameters
Configure behavior without code changes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('message_prefix', 'Data')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('message_count', 100)

        # Get parameter values
        self.prefix = self.get_parameter('message_prefix').value
        self.rate = self.get_parameter('publish_rate').value
        self.count = self.get_parameter('message_count').value

        self.get_logger().info(
            f'Prefix: {self.prefix}, Rate: {self.rate}Hz, Count: {self.count}')

        # Create publisher
        self.publisher = self.create_publisher(String, 'topic', 10)

        # Create timer with configured rate
        timer_period = 1.0 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Publish with configured prefix
        msg = String()
        msg.data = f'{self.prefix}: {self.counter}'
        self.publisher.publish(msg)
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    configurable_node = ConfigurableNode()
    rclpy.spin(configurable_node)
    configurable_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Running with custom parameters**:

```bash
# Use default parameters
ros2 run package configurable_node

# Override parameters at runtime
ros2 run package configurable_node \
  --ros-args \
  -p message_prefix:='Sensor' \
  -p publish_rate:=10.0
```

This allows configuring nodes without modifying code.

## Quality of Service (QoS)

**QoS** controls how messages are delivered:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Best effort (fast, may lose messages)
qos_fast = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

# Reliable (slower, guarantees delivery)
qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_ALL,
    depth=100
)

# Use when creating publishers/subscribers
self.create_publisher(String, 'topic', qos_reliable)
```

**When to use each**:
- **BEST_EFFORT**: High-frequency data (cameras, LiDAR) - losing a frame is OK
- **RELIABLE**: Critical data (commands, state changes) - must not lose messages

## Namespacing: Multiple Nodes

Run multiple robots without conflicts using **namespaces**:

```bash
# Robot 1
ros2 run package node --ros-args -n /robot1

# Robot 2
ros2 run package node --ros-args -n /robot2

# Now topics are:
# /robot1/topic
# /robot2/topic
```

This is how you run multiple robots on the same ROS 2 network.

## Expected Output

**Lifecycle Node**:
```
[minimal_lifecycle_node] Configuring...
[minimal_lifecycle_node] Configuration complete
[minimal_lifecycle_node] Activating...
[minimal_lifecycle_node] Running...
```

**Configurable Node** (with custom parameters):
```
[configurable_node] Prefix: Sensor, Rate: 10.0Hz, Count: 100
[configurable_node] Publishing: Sensor: 0
[configurable_node] Publishing: Sensor: 1
[configurable_node] Publishing: Sensor: 2
...
```

## Putting It All Together

A production robot node combines all three:

1. **Lifecycle**: Configure on startup, activate when ready
2. **Parameters**: Allow runtime configuration
3. **QoS**: Choose appropriate reliability for each topic

```python
class RobotControlNode(LifecycleNode):
    def on_configure(self, state: State):
        # Declare parameters
        self.declare_parameter('joint_speed', 0.5)

        # Create publishers with reliable QoS for commands
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(
            JointCommand, 'joint_commands', qos_reliable)

        # Create subscriptions with best effort for sensor data
        qos_fast = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sensor_sub = self.create_subscription(
            JointState, 'joint_states', self.sensor_callback, qos_fast)

        return TransitionCallbackReturn.SUCCESS
```

## Summary

In this chapter, you learned:

- **Lifecycle Nodes**: Coordinate startup and shutdown with state transitions
- **Parameters**: Configure nodes at runtime without code changes
- **Quality of Service**: Control message reliability and delivery
- **Namespacing**: Run multiple nodes without conflicts
- Production-ready systems combine all three patterns

**Key Takeaway**: Advanced nodes make ROS 2 systems scalable, reliable, and maintainable.

**Next steps**: You've completed Module 1! The next modules will build on these foundations with simulation, perception, and AI integration.

---

**References**:

- ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Node Lifecycles*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Node-Lifecycle.html
- ROS 2 Documentation Contributors. (2024). *Parameters in ROS 2*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters.html
- ROS 2 Documentation Contributors. (2024). *Quality of Service Settings*. Retrieved from https://docs.ros.org/en/humble/Concepts/Advanced/QoS-Settings.html
- ROS 2 Lifecycle Documentation. (2024). Retrieved from https://github.com/ros2-lifecycle

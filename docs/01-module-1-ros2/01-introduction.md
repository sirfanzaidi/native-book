---
id: 01-introduction
title: "Chapter 1 - ROS 2 Architecture: Nodes, Topics, Services"
sidebar_label: "Chapter 1: Architecture"
---

# Chapter 1: ROS 2 Architecture

## Introduction

Welcome to Module 1: The Robotic Nervous System. In this chapter, you'll learn the fundamental concepts of ROS 2 (Robot Operating System 2), the industry-standard middleware that enables distributed robotics control.

### Why ROS 2?

ROS 2 is the de facto standard for robotics software development. It provides:
- **Distributed communication**: Connect components across multiple machines
- **Modular architecture**: Build systems from reusable, testable components
- **Cross-platform support**: Run on Linux, Windows, and macOS
- **Real-time capabilities**: Predictable message delivery and timing

**Source**: ROS 2 Documentation Contributors. (2024). *ROS 2 Humble Distribution*. Retrieved from https://docs.ros.org/en/humble/

### What is Middleware?

Middleware is software that provides a bridge between different applications and systems. In robotics, middleware handles:
- Inter-process communication (IPC) between robot control modules
- Time synchronization across sensors and actuators
- Network communication for distributed systems
- Message serialization and deserialization

ROS 2 builds on industry-standard protocols like DDS (Data Distribution Service) to provide reliable, efficient communication.

### The Pub/Sub Pattern Explained

ROS 2's primary communication pattern is **Publish/Subscribe (Pub/Sub)**:

- **Publisher**: A node that sends data to a **Topic** (named communication channel)
- **Subscriber**: A node that receives data from a Topic
- **Topic**: A named bus where messages flow between publishers and subscribers

This decouples publishers from subscribers - they don't need to know about each other, making the system flexible and scalable.

```
Publisher A ---> Topic "sensor/lidar" <--- Subscriber B
               |                        <--- Subscriber C
```

### What You'll Learn in This Chapter

By the end of this chapter, you will:
- [ ] Understand ROS 2 nodes, topics, and pub/sub messaging
- [ ] Write a simple ROS 2 publisher in Python
- [ ] Write a simple ROS 2 subscriber in Python
- [ ] Run both nodes and verify message exchange
- [ ] Understand services and their use cases
- [ ] Know when to use topics vs. services

---

## ROS 2 Nodes & Topics

### Nodes

A **node** is the basic ROS 2 building block. It's a process that:
- Publishes to one or more topics
- Subscribes to one or more topics
- Provides or calls services
- Executes in parallel with other nodes

All nodes communicate through the ROS 2 middleware.

### Topics

A **topic** is a named communication channel. Nodes can publish data to topics or subscribe to receive data from them.

**Example topics in a humanoid robot**:
- `/joint_states` - publishes joint angles and velocities
- `/sensor/lidar/scan` - publishes LiDAR point cloud
- `/cmd_vel` - receives velocity commands

---

## Creating Your First ROS 2 Nodes

### Simple Publisher

Let's create a publisher that sends messages to a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What's happening**:
1. `MinimalPublisher` is a class that extends `Node`
2. `create_publisher()` creates a publisher on topic `topic` with message type `String`
3. `create_timer()` calls `timer_callback()` every 0.5 seconds
4. `publisher_.publish(msg)` sends the message to the topic
5. `rclpy.spin()` keeps the node running and processing callbacks

### Simple Subscriber

Now let's create a subscriber to receive those messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What's happening**:
1. `create_subscription()` subscribes to topic `topic` with message type `String`
2. `listener_callback()` is called every time a message is received
3. The callback prints the received message

### Running the Example

**Terminal 1** (Publisher):
```bash
python3 minimal_publisher.py
```

**Terminal 2** (Subscriber):
```bash
python3 minimal_subscriber.py
```

### Expected Output

**Terminal 1**:
```
[INFO] [minimal_publisher]: Publishing: Hello World: 0
[INFO] [minimal_publisher]: Publishing: Hello World: 1
[INFO] [minimal_publisher]: Publishing: Hello World: 2
```

**Terminal 2**:
```
[INFO] [minimal_subscriber]: I heard: Hello World: 0
[INFO] [minimal_subscriber]: I heard: Hello World: 1
[INFO] [minimal_subscriber]: I heard: Hello World: 2
```

### Explanation

The publisher sends a new message every 0.5 seconds. The subscriber receives each message and logs it. This is the fundamental ROS 2 communication pattern used throughout robotics applications.

---

## Summary

In this chapter, you learned:
- ROS 2 provides a distributed middleware for robotics
- The Pub/Sub pattern decouples components
- Nodes publish and subscribe to topics
- Python `rclpy` library provides simple APIs for ROS 2 programming

**Next steps**: In Chapter 2, we'll explore services (synchronous request/response) and more advanced node patterns.

---

**References**:
- ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Nodes*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Nodes.html
- ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Topics*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Topics.html
- ROS 2 Examples. (2024). Retrieved from https://github.com/ros2/examples/tree/humble/rclpy_pubsub

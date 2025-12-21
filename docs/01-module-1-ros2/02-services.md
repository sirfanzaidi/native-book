---
id: 02-services
title: "Chapter 2 - ROS 2 Services & Actions"
sidebar_label: "Chapter 2: Services"
---

# Chapter 2: ROS 2 Services & Actions

## Introduction

In Chapter 1, you learned about **Pub/Sub** messaging: a one-way, asynchronous communication pattern where publishers send data to topics and subscribers receive it.

But what happens when you need a **request-response** interaction? What if one node needs to ask another node a question and wait for an answer?

In this chapter, you'll learn about **Services**, the ROS 2 mechanism for synchronous, request-response communication.

### Why Services?

Services solve different problems than topics:

- **Topics (Pub/Sub)**: "I'm publishing sensor data continuously"
- **Services (Request-Response)**: "I need to calculate something right now and get the result"

Common use cases for services:
- Asking a robot to move to a specific position
- Requesting calculations (e.g., inverse kinematics)
- Setting configuration parameters
- Triggering one-time actions

**Source**: ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Services*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Services.html

### Services vs Topics: When to Use Each

| Aspect | Topics (Pub/Sub) | Services (Request-Response) |
|--------|------------------|---------------------------|
| **Pattern** | One-way streaming | Two-way synchronous |
| **Timing** | Asynchronous | Synchronous (wait for reply) |
| **Use Case** | Sensors, continuous data | Commands, calculations |
| **Example** | Publishing joint angles | Asking for IK solution |
| **Blocking** | Non-blocking | Caller waits for response |

## Understanding the Service Pattern

A **service** consists of three components:

1. **Service Server**: Node that provides a service (answers requests)
2. **Service Client**: Node that calls the service (asks questions)
3. **Service Definition**: The request and response data structures

### The Request-Response Flow

```
Client (Robot Arm)         Server (IK Solver)
      |                          |
      | -- Request: Target Pos -->|
      |   (x=1.0, y=2.0, z=0.5)  |
      |                          |
      |                     [Calculating...]
      |                          |
      | <-- Response: Joint Angles |
      |   (θ1=45°, θ2=30°, θ3=60°)|
      |                          |
```

The client blocks and waits for the server to respond. This is different from Pub/Sub, where the publisher doesn't know or care who receives the data.

### Creating Service Definitions

Services need a `.srv` file that defines the request and response:

```
# Example: Inverse Kinematics Service
float64[] target_position  # Request: desired (x, y, z)
---
float64[] joint_angles     # Response: calculated angles
```

For simplicity, this chapter uses standard message types (like `std_srvs.srv.Trigger`) that come with ROS 2.

## Creating Your First Service

### Simple Service Server

A service server listens for requests and sends responses:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Service Server
Provides a service that adds two numbers
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # Create service: name='add_two_ints', type=AddTwoInts, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback)
        self.get_logger().info('Service server started')

    def add_two_ints_callback(self, request, response):
        # request.a and request.b are the inputs
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: {request.a} + {request.b} = {response.sum}')
        return response  # Send back the response


def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What's happening**:
1. `create_service()` registers a service named `add_two_ints`
2. When a client calls this service, `add_two_ints_callback()` is invoked
3. The callback receives the `request` object, does computation, and returns `response`
4. The client receives the response and continues execution

### Simple Service Client

A service client calls the service and waits for the response:

```python
#!/usr/bin/env python3
"""
Simple ROS 2 Service Client
Calls the add_two_ints service
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.get_logger().info('Service found')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        # Call service and wait for response
        future = self.cli.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()

    # Send request: 5 + 3
    future = minimal_client.send_request(5, 3)

    # Wait for response
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()

    minimal_client.get_logger().info(
        f'Result of 5 + 3 = {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**What's happening**:
1. `create_client()` creates a client for the service
2. `wait_for_service()` waits until the service is available
3. `call_async()` sends the request asynchronously
4. `spin_until_future_complete()` waits for the response
5. The result is retrieved and printed

### Expected Output

**Server terminal**:
```
[minimal_service] Service server started
[minimal_service] Incoming request: 5 + 3 = 8
```

**Client terminal**:
[minimal_client] Waiting for service...
[minimal_client] Service found
[minimal_client] Result of 5 + 3 = 8
```

### Running Both Nodes

**Terminal 1** (Server):
```bash
python3 simple_service_server.py
```

**Terminal 2** (Client):
```bash
python3 simple_service_client.py
```

The client will wait for the server, call the service, and receive the response.

## Services vs Topics: A Robot Example

Imagine a robot arm controller:

**Using Topics (Wrong approach)**:
- Publisher sends "move to position" messages continuously
- Subscriber tries to move
- No confirmation of success
- No way to know if the move completed

**Using Services (Correct approach)**:
- Client sends "move to position" request
- Server confirms receipt and starts movement
- Server waits for movement to complete
- Server responds with "success: true/false"
- Client continues only after getting response

This is why robots use services for critical commands.

## Summary

In this chapter, you learned:

- Services provide **request-response** synchronous communication
- Services are used for **commands** and **calculations**
- Topics are used for **continuous streaming** data
- Service servers listen for requests and respond
- Service clients send requests and wait for responses
- Use services for critical, one-time operations
- Use topics for continuous, non-blocking data flow

**Key Takeaway**: Services make robots deterministic—you know when an operation completes.

**Next steps**: In Chapter 3, we'll explore advanced node patterns including lifecycle management and parameterized nodes.

---

**References**:

- ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Services*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Services.html
- ROS 2 Documentation Contributors. (2024). *Understanding ROS 2 Actions*. Retrieved from https://docs.ros.org/en/humble/Tutorials/Understanding-ROS2-Actions.html
- ROS 2 Examples. (2024). *Services Example*. Retrieved from https://github.com/ros2/examples/tree/humble/rclpy_services

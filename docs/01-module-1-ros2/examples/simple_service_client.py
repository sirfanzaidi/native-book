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

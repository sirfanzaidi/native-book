#!/usr/bin/env python3
"""
Voice Command Processor
Converts voice input to text commands for robot
Can use speech recognition or text input
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty
import sys


class VoiceCommandProcessor(Node):
    def __init__(self):
        super().__init__('voice_command_processor')

        # Publisher for processed commands
        self.command_pub = self.create_publisher(
            String,
            '/voice_command',
            10)

        # List of allowed commands (safety whitelist)
        self.allowed_commands = [
            'pick up',
            'walk to',
            'bring',
            'place',
            'observe',
            'stop',
            'wait'
        ]

        self.get_logger().info('Voice command processor started')
        self.get_logger().info('Listening for voice input...')
        self.get_logger().info('Allowed commands: ' + ', '.join(self.allowed_commands))

    def process_voice_input(self, raw_input):
        """
        Process raw voice/text input
        Validate against whitelist for safety
        """
        # Convert to lowercase for comparison
        command = raw_input.lower().strip()

        # Validate against whitelist
        is_valid = any(
            allowed in command
            for allowed in self.allowed_commands
        )

        if not is_valid:
            self.get_logger().warning(f'⚠️  Unknown command: "{command}"')
            self.get_logger().info('Try: ' + ', '.join(self.allowed_commands))
            return False

        # Command is valid - publish it
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        self.get_logger().info(f'✓ Published command: "{command}"')
        return True

    def receive_text_input(self):
        """Receive text command from user input"""
        try:
            user_input = input('\n🎤 Enter command: ')
            return self.process_voice_input(user_input)
        except KeyboardInterrupt:
            self.get_logger().info('Shutting down...')
            return False


def main(args=None):
    rclpy.init(args=args)
    processor = VoiceCommandProcessor()

    # Process commands from stdin
    print('\n' + '='*50)
    print('Voice Command Processor')
    print('='*50)
    print('Enter voice commands for the robot')
    print('Type Ctrl+C to exit\n')

    try:
        while rclpy.ok():
            # Non-blocking check for ROS2 events
            rclpy.spin_once(processor, timeout_sec=0.1)

            # Get user input
            processor.receive_text_input()

    except KeyboardInterrupt:
        processor.get_logger().info('Shutdown signal received')
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

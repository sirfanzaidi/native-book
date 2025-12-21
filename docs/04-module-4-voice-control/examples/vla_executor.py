#!/usr/bin/env python3
"""
Vision-Language-Action Executor
Combines vision perception + language understanding + robot execution
Complete autonomous system pipeline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
import time


class VLAExecutor(Node):
    def __init__(self):
        super().__init__('vla_executor')

        # Inputs
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.vision_callback,
            10)

        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.language_callback,
            10)

        self.poses_sub = self.create_subscription(
            PoseStamped,
            '/detected_objects',
            self.perception_callback,
            10)

        # Outputs
        self.action_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.status_pub = self.create_publisher(
            String,
            '/vla_status',
            10)

        # State
        self.current_image = None
        self.current_command = None
        self.current_perception = None

        self.get_logger().info('VLA Executor initialized')
        self.get_logger().info('Ready: vision + language + perception')

    def vision_callback(self, msg):
        """Receive camera image"""
        self.current_image = msg
        self.try_execute_vla()

    def language_callback(self, msg):
        """Receive language command"""
        self.current_command = msg.data
        self.get_logger().info(f'🎤 Command: {msg.data}')
        self.try_execute_vla()

    def perception_callback(self, msg):
        """Receive object perception results"""
        self.current_perception = msg
        self.get_logger().info(
            f'👁️  Detected object at ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')
        self.try_execute_vla()

    def try_execute_vla(self):
        """Execute VLA when vision + language + perception available"""
        if (self.current_image is None or
            self.current_command is None or
            self.current_perception is None):
            return

        # All inputs available - run VLA
        self.execute_vla()

        # Reset for next command
        self.current_image = None
        self.current_command = None

    def execute_vla(self):
        """Execute Vision-Language-Action pipeline"""
        self.get_logger().info('⚙️  Executing VLA pipeline...')
        self.publish_status('VLA executing')

        # Stage 1: Vision - detect objects
        objects = self.stage_vision()
        self.get_logger().info(f'✓ Vision detected {len(objects)} objects')

        # Stage 2: Language - understand intent
        intent = self.stage_language(self.current_command)
        self.get_logger().info(f'✓ Language understood intent: {intent}')

        # Stage 3: Grounding - link language to vision
        target = self.stage_grounding(intent, objects)
        self.get_logger().info(f'✓ Grounding linked to target')

        # Stage 4: Planning - generate motion
        plan = self.stage_planning(target)
        self.get_logger().info(f'✓ Plan generated: {len(plan)} steps')

        # Stage 5: Execution - run actions
        success = self.stage_execution(plan)
        self.get_logger().info(f'✓ Execution {"successful" if success else "failed"}')

        self.publish_status('VLA complete')

    def stage_vision(self):
        """Stage 1: Vision perception"""
        # In real system, runs GPU perception pipeline
        objects = [
            {'name': 'cube', 'position': self.current_perception.pose.position}
        ]
        return objects

    def stage_language(self, command):
        """Stage 2: Language understanding"""
        # Extract intent from command
        if 'pick' in command.lower():
            return 'pick_up'
        elif 'move' in command.lower():
            return 'move_to'
        elif 'place' in command.lower():
            return 'place_down'
        return 'unknown'

    def stage_grounding(self, intent, objects):
        """Stage 3: Ground language in vision"""
        # Link "the object" to detected object
        if objects:
            return objects[0]
        return None

    def stage_planning(self, target):
        """Stage 4: Generate motion plan"""
        # Simple plan: move toward object
        plan = [
            {'action': 'navigate', 'target': target},
            {'action': 'grasp', 'force': 50},
            {'action': 'lift', 'height': 0.2}
        ]
        return plan

    def stage_execution(self, plan):
        """Stage 5: Execute plan"""
        for step in plan:
            self.get_logger().info(f'  Executing: {step["action"]}')
            time.sleep(0.5)  # Simulate execution time

        return True

    def publish_status(self, status):
        """Publish system status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    executor = VLAExecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

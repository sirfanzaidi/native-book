#!/usr/bin/env python3
"""
LLM-Based Robot Controller
Converts natural language voice commands to robot actions
Uses OpenAI GPT-4 for planning
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class LLMRobotController(Node):
    def __init__(self):
        super().__init__('llm_robot_controller')

        # Subscribe to voice commands (text or speech-to-text output)
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.process_voice_command,
            10)

        # Publish planned actions to executor
        self.action_pub = self.create_publisher(
            String,
            '/robot_action',
            10)

        # System prompt defines robot capabilities
        self.system_prompt = """
You are a controller for a humanoid robot with these capabilities:
- Two arms: reach 1.5m, max grip force 100N
- Mobile base: max speed 1.5 m/s
- Head camera: 1920×1080 resolution, real-time perception
- LiDAR: 270° field of view, 10m range
- Gripper: parallel jaw with force feedback

Available robot actions:
1. navigate_to(location): Move to named location
2. grasp(object_name): Pick up object
3. place(object_name, location): Put down object
4. observe(): Use camera to detect objects
5. say(text): Speak to human

Safety constraints (NON-NEGOTIABLE):
- Never move faster than 1.5 m/s
- Keep arms in safe workspace (1.5m radius)
- Gripper max force: 100N
- Always check path for obstacles
- Report any execution failures

Plan every command step-by-step."""

        self.get_logger().info('LLM robot controller initialized')
        self.get_logger().info('Ready to receive voice commands')

    def process_voice_command(self, msg):
        """Receive and process voice command"""
        user_command = msg.data
        self.get_logger().info(f'📢 Voice Command: "{user_command}"')

        # In real implementation, would call OpenAI API:
        # response = openai.ChatCompletion.create(...)

        # For this example, simulate LLM planning
        plan = self.simulate_llm_planning(user_command)

        self.get_logger().info(f'🤖 LLM Plan:\n{plan}')

        # Execute the planned actions
        self.execute_plan(plan)

    def simulate_llm_planning(self, command):
        """Simulate LLM generating a plan for the command"""
        # In real system, this would call OpenAI GPT-4
        # For now, return a plausible plan

        plans = {
            'pick up': 'Step 1: Observe to locate object\nStep 2: Navigate within reach\nStep 3: Grasp object with controlled force',
            'walk': 'Step 1: Check LiDAR for obstacles\nStep 2: Plan path avoiding obstacles\nStep 3: Navigate at safe speed',
            'bring': 'Step 1: Observe to locate object\nStep 2: Navigate and grasp\nStep 3: Carry to destination\nStep 4: Place carefully',
        }

        for keyword, plan in plans.items():
            if keyword.lower() in command.lower():
                return plan

        return 'Step 1: Clarify request\nStep 2: Wait for user confirmation'

    def execute_plan(self, plan):
        """Execute LLM-generated plan by publishing actions"""
        # Break down plan into steps
        steps = [s.strip() for s in plan.split('\n') if s.strip()]

        for step in steps:
            # Publish action to executor node
            action_msg = String()
            action_msg.data = step
            self.action_pub.publish(action_msg)

            self.get_logger().info(f'⚙️  Executing: {step}')

            # Brief delay between actions (in real system, wait for feedback)
            time.sleep(0.5)

        self.get_logger().info('✅ Plan execution complete')


def main(args=None):
    rclpy.init(args=args)
    controller = LLMRobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

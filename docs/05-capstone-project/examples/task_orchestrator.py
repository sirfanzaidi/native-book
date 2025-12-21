#!/usr/bin/env python3
"""
Task Orchestrator: Unified Control for Furniture Assembly
Coordinates: Language Understanding + Perception + Planning + Execution
Integrates all 4 modules into single autonomous system
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import time
from enum import Enum
from typing import List, Dict, Any


class ExecutionState(Enum):
    """Execution state machine"""
    IDLE = "idle"
    PARSING = "parsing"
    PLANNING = "planning"
    EXECUTING = "executing"
    VERIFYING = "verifying"
    COMPLETE = "complete"
    ERROR = "error"


class TaskOrchestrator(Node):
    """
    Master orchestrator for furniture assembly
    Coordinates all 4 modules:
    - Module 1: ROS 2 pub/sub control
    - Module 2: Gazebo simulation feedback
    - Module 3: Isaac perception
    - Module 4: LLM task planning
    """

    def __init__(self):
        super().__init__('task_orchestrator')

        # Concurrent callback group for parallel processing
        self.cb_group = ReentrantCallbackGroup()

        # Subscriptions: Language commands and perception results
        self.command_sub = self.create_subscription(
            String,
            '/assembly_command',
            self.on_assembly_command,
            10,
            callback_group=self.cb_group)

        self.perception_sub = self.create_subscription(
            PoseStamped,
            '/perception/detected_objects',
            self.on_perception_update,
            10,
            callback_group=self.cb_group)

        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.on_camera_frame,
            10,
            callback_group=self.cb_group)

        # Publications: Control commands and status
        self.motion_pub = self.create_publisher(
            Twist,
            '/robot/motion_cmd',
            10)

        self.status_pub = self.create_publisher(
            String,
            '/task/status',
            10)

        # Internal state
        self.state = ExecutionState.IDLE
        self.task_plan: List[Dict[str, Any]] = []
        self.current_step_idx = 0
        self.step_start_time = None
        self.max_retries = 3
        self.retry_count = 0

        # Perception state buffer
        self.latest_image = None
        self.detected_parts = {}  # {part_id: pose}

        self.get_logger().info('🤖 Task Orchestrator initialized')
        self.log_status('Ready: waiting for assembly command')

    def on_assembly_command(self, msg: String):
        """
        Receive assembly task via natural language
        Module 4: Language understanding via LLM
        """
        if self.state != ExecutionState.IDLE:
            self.get_logger().warn('⚠️  Task already in progress')
            return

        command = msg.data
        self.get_logger().info(f'📝 Command received: "{command}"')
        self.log_status(f'Received: {command}')

        # Parse task into steps (Module 4: LLM)
        self.change_state(ExecutionState.PARSING)
        self.task_plan = self.parse_task_with_llm(command)

        if not self.task_plan:
            self.get_logger().error('❌ Failed to parse task')
            self.change_state(ExecutionState.ERROR)
            return

        self.get_logger().info(f'📋 Task plan: {len(self.task_plan)} steps')
        for i, step in enumerate(self.task_plan, 1):
            self.get_logger().info(f'  Step {i}: {step["action"]}')

        # Begin execution
        self.current_step_idx = 0
        self.retry_count = 0
        self.change_state(ExecutionState.PLANNING)
        self.execute_next_step()

    def parse_task_with_llm(self, command: str) -> List[Dict[str, Any]]:
        """
        Parse natural language command into execution steps
        In production: Call OpenAI GPT-4, Claude, or Llama
        For demo: Rule-based parsing
        """
        command_lower = command.lower()

        # Rule-based parsing for demo
        if 'bookshelf' in command_lower or 'shelf' in command_lower:
            return [
                {
                    'id': 1,
                    'action': 'detect_parts',
                    'module': 3,  # Perception
                    'timeout': 5.0,
                    'description': 'Detect all furniture parts in scene'
                },
                {
                    'id': 2,
                    'action': 'pick_left_panel',
                    'module': 1,  # Control
                    'target_pose': (0.5, -0.2, 0.8),
                    'description': 'Grasp left panel and lift'
                },
                {
                    'id': 3,
                    'action': 'verify_grasp',
                    'module': 3,  # Perception
                    'timeout': 3.0,
                    'description': 'Verify object is grasped'
                },
                {
                    'id': 4,
                    'action': 'position_assembly',
                    'module': 1,  # Control
                    'target_pose': (0.0, 0.0, 0.5),
                    'description': 'Position panel for assembly'
                },
                {
                    'id': 5,
                    'action': 'assemble',
                    'module': 2,  # Simulation/Physics
                    'force_threshold': 50,
                    'description': 'Execute assembly motion with force feedback'
                },
                {
                    'id': 6,
                    'action': 'verify_complete',
                    'module': 3,  # Perception
                    'timeout': 3.0,
                    'description': 'Verify assembly step complete'
                }
            ]

        # For other commands: return empty (unrecognized)
        self.get_logger().error(f'Unrecognized command: {command}')
        return []

    def execute_next_step(self):
        """Execute current step in task plan"""
        if self.current_step_idx >= len(self.task_plan):
            self.complete_task()
            return

        step = self.task_plan[self.current_step_idx]
        step_num = self.current_step_idx + 1
        total_steps = len(self.task_plan)

        self.step_start_time = time.time()
        step_action = step['action']
        description = step.get('description', step_action)

        self.get_logger().info(
            f'⚙️  Step {step_num}/{total_steps}: {step_action}')
        self.log_status(f'Step {step_num}/{total_steps}: {description}')

        try:
            self.change_state(ExecutionState.EXECUTING)

            # Dispatch to appropriate handler based on module
            module_id = step.get('module', 1)

            if module_id == 1:  # Control (ROS 2)
                self.execute_control_step(step)
            elif module_id == 2:  # Simulation/Physics
                self.execute_physics_step(step)
            elif module_id == 3:  # Perception (Isaac)
                self.execute_perception_step(step)
            else:
                raise ValueError(f'Unknown module: {module_id}')

            # Verify step complete
            self.change_state(ExecutionState.VERIFYING)
            time.sleep(0.5)

            self.get_logger().info(f'✅ Step {step_num} complete')

            # Advance to next step
            self.current_step_idx += 1
            self.retry_count = 0
            time.sleep(1)  # Brief pause between steps
            self.execute_next_step()

        except Exception as e:
            self.get_logger().error(f'❌ Step {step_num} failed: {str(e)}')

            # Retry logic
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(
                    f'🔄 Retrying step {step_num} (attempt {self.retry_count}/{self.max_retries})')
                self.log_status(f'Retrying step {step_num}...')
                time.sleep(2)
                self.execute_next_step()  # Retry same step
            else:
                self.get_logger().error(f'Step {step_num} failed after {self.max_retries} retries')
                self.change_state(ExecutionState.ERROR)
                self.log_status(f'ERROR at step {step_num}')

    def execute_control_step(self, step: Dict[str, Any]):
        """
        Execute Module 1: ROS 2 Control
        Send motion commands and monitor feedback
        """
        action = step['action']
        target_pose = step.get('target_pose', (0.0, 0.0, 0.0))

        self.get_logger().info(f'📤 Sending motion command: {action}')
        self.get_logger().info(f'   Target: {target_pose}')

        # Create motion command (Module 1: Twist topic)
        cmd = Twist()
        cmd.linear.x = target_pose[0]
        cmd.linear.y = target_pose[1]
        cmd.linear.z = target_pose[2]
        cmd.angular.z = 0.0

        # Publish command
        self.motion_pub.publish(cmd)

        # Simulate motion execution
        for i in range(10):
            time.sleep(0.2)
            self.get_logger().debug(f'Motion step {i+1}/10...')

        self.get_logger().info('✓ Motion complete')

    def execute_physics_step(self, step: Dict[str, Any]):
        """
        Execute Module 2: Physics/Simulation
        Validate motion in Gazebo and check constraints
        """
        action = step['action']
        force_threshold = step.get('force_threshold', 50)

        self.get_logger().info(f'🎮 Physics simulation: {action}')
        self.get_logger().info(f'   Force threshold: {force_threshold}N')

        # In production: Query Gazebo physics engine
        # Check contact forces, joint torques, stability
        time.sleep(1.5)  # Simulate physics computation

        # Verify within constraints
        simulated_force = 45  # Simulated force reading
        if simulated_force > force_threshold:
            raise RuntimeError(f'Force {simulated_force}N exceeds threshold {force_threshold}N')

        self.get_logger().info(f'✓ Physics verified: force={simulated_force}N')

    def execute_perception_step(self, step: Dict[str, Any]):
        """
        Execute Module 3: Perception (Isaac)
        Detect objects or verify assembly state
        """
        action = step['action']
        timeout = step.get('timeout', 5.0)

        self.get_logger().info(f'👁️  Perception: {action}')
        self.get_logger().info(f'   Timeout: {timeout}s')

        # In production: Run GPU perception pipeline
        # For demo: Wait for simulated detections
        start_time = time.time()

        while time.time() - start_time < timeout:
            if len(self.detected_parts) > 0:
                self.get_logger().info(
                    f'✓ Detected {len(self.detected_parts)} objects')
                return

            time.sleep(0.2)

        raise TimeoutError(f'Perception timeout after {timeout}s')

    def on_perception_update(self, msg: PoseStamped):
        """
        Receive perception updates from Module 3
        Module 3: Isaac detection pipeline
        """
        part_id = msg.header.frame_id
        self.detected_parts[part_id] = {
            'timestamp': msg.header.stamp,
            'position': msg.pose.position,
            'orientation': msg.pose.orientation
        }

        self.get_logger().debug(f'Perception update: {part_id}')

    def on_camera_frame(self, msg: Image):
        """Buffer latest camera frame for perception"""
        self.latest_image = msg

    def complete_task(self):
        """Task execution successful"""
        elapsed = time.time() - self.step_start_time if self.step_start_time else 0
        self.change_state(ExecutionState.COMPLETE)

        self.get_logger().info('🎉 Task complete!')
        self.get_logger().info(f'   Total time: {elapsed:.1f}s')
        self.log_status('Task complete - furniture assembly finished!')

        # Reset for next task
        time.sleep(2)
        self.state = ExecutionState.IDLE
        self.task_plan = []
        self.current_step_idx = 0
        self.log_status('Ready for next task')

    def change_state(self, new_state: ExecutionState):
        """Transition to new execution state"""
        old_state = self.state
        self.state = new_state
        self.get_logger().info(
            f'State: {old_state.value} → {new_state.value}')

    def log_status(self, message: str):
        """Publish status update"""
        msg = String()
        msg.data = f'[{self.state.value}] {message}'
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    orchestrator = TaskOrchestrator()

    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

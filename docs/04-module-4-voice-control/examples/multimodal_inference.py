#!/usr/bin/env python3
"""
Multimodal Inference Engine
Fuses vision, language, and robot state for action generation
Core of VLA system
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import time


class MultimodalInferenceEngine(Node):
    def __init__(self):
        super().__init__('multimodal_inference')

        # Input streams
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.on_image,
            10)

        self.perception_sub = self.create_subscription(
            PoseStamped,
            '/detected_objects',
            self.on_perception,
            10)

        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.on_command,
            10)

        # Output
        self.action_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # State buffers for fusion
        self.latest_image = None
        self.latest_perception = None
        self.latest_command = None
        self.latest_robot_state = None

        self.inference_count = 0
        self.get_logger().info('Multimodal Inference Engine started')

    def on_image(self, msg):
        """Buffer latest image"""
        self.latest_image = msg

    def on_perception(self, msg):
        """Buffer latest perception"""
        self.latest_perception = msg

    def on_command(self, msg):
        """Trigger inference on new command"""
        self.latest_command = msg.data

        # Fuse all available modalities
        if self.latest_image and self.latest_perception and self.latest_command:
            self.run_multimodal_inference()

    def run_multimodal_inference(self):
        """
        Run multimodal inference:
        Fuse vision + language + state → action
        """
        self.inference_count += 1
        t_start = time.time()

        # Step 1: Encode vision (GPU)
        vision_encoding = self.encode_vision(self.latest_image)

        # Step 2: Encode language (GPU)
        language_encoding = self.encode_language(self.latest_command)

        # Step 3: Encode perception (CPU/GPU)
        perception_encoding = self.encode_perception(self.latest_perception)

        # Step 4: Fuse encodings (GPU)
        fused_representation = self.fuse_modalities(
            vision_encoding,
            language_encoding,
            perception_encoding
        )

        # Step 5: Decode action (GPU)
        action = self.decode_action(fused_representation)

        # Execute action
        self.action_pub.publish(action)

        # Log timing
        elapsed = (time.time() - t_start) * 1000  # Convert to ms
        self.get_logger().info(
            f'Inference #{self.inference_count}: {elapsed:.1f}ms | '
            f'Action: linear={action.linear.x:.2f}, angular={action.angular.z:.2f}')

    def encode_vision(self, image):
        """
        Encode vision modality
        In real system: Run CNN on GPU to extract visual features
        """
        # Simulate: Extract image dimensions as basic features
        return {
            'width': image.width,
            'height': image.height,
            'modality': 'vision'
        }

    def encode_language(self, command):
        """
        Encode language modality
        In real system: Use BERT/GPT embeddings
        """
        # Simulate: Parse command into tokens
        tokens = command.lower().split()
        return {
            'tokens': tokens,
            'length': len(tokens),
            'modality': 'language'
        }

    def encode_perception(self, perception):
        """
        Encode perception modality (object detections)
        In real system: Extract CNN features from objects
        """
        pos = perception.pose.position
        return {
            'x': float(pos.x),
            'y': float(pos.y),
            'z': float(pos.z),
            'modality': 'perception'
        }

    def fuse_modalities(self, vision, language, perception):
        """
        Fuse all modalities into unified representation
        In real system: Trained multimodal fusion network
        """
        # Combine all modalities
        fused = {
            'vision': vision,
            'language': language,
            'perception': perception,
            'fused_timestamp': time.time()
        }
        return fused

    def decode_action(self, fused_representation):
        """
        Decode action from fused representation
        In real system: Neural network trained end-to-end
        """
        # Extract components
        lang = fused_representation['language']
        perc = fused_representation['perception']

        # Simple rule-based action (would be learned in real system)
        action = Twist()

        # Language-based decision
        if 'forward' in ' '.join(lang['tokens']):
            action.linear.x = 0.5
        elif 'backward' in ' '.join(lang['tokens']):
            action.linear.x = -0.3
        elif 'stop' in ' '.join(lang['tokens']):
            action.linear.x = 0.0
            action.angular.z = 0.0

        # Perception-based refinement
        if perc['x'] > 1.0:
            action.linear.x = min(action.linear.x, 0.2)

        return action


def main(args=None):
    rclpy.init(args=args)
    engine = MultimodalInferenceEngine()
    rclpy.spin(engine)
    engine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

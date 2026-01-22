#!/usr/bin/env python3
"""
Week 2: ROS2 Basics Lab - Joint State Subscriber

This lab introduces ROS2 concepts through a simple subscriber node.
You will implement a node that:
1. Subscribes to the /so101_follower/joint_states topic
2. Extracts joint position data from incoming messages
3. Prints the positions in a human-readable format

Learning Objectives:
- Understand ROS2 node structure
- Learn how to create subscribers
- Work with sensor_msgs/JointState messages
- Use callbacks to process incoming data

Run with:
    ros2 run so101_hw_interface ros2_basics_lab

Or directly:
    python3 ros2_basics_lab.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStateSubscriber(Node):
    """
    A simple ROS2 node that subscribes to joint states and prints them.

    The SO-101 robot publishes joint positions on /so101_follower/joint_states
    as a sensor_msgs/JointState message containing:
    - header: Timestamp and frame info
    - name: List of joint names ["1", "2", "3", "4", "5", "6"]
    - position: List of joint angles in radians
    - velocity: List of joint velocities (may be empty)
    - effort: List of joint efforts/torques (may be empty)
    """

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # TODO: Create a subscriber to /so101_follower/joint_states
        # Use:
        #   - Message type: JointState
        #   - Topic: '/so101_follower/joint_states'
        #   - Callback: self.joint_state_callback
        #   - QoS queue size: 10
        #
        # Hint: self.create_subscription(MessageType, 'topic_name', callback, queue_size)

        self.subscription = None  # Replace with actual subscription

        self.get_logger().info('Joint State Subscriber started!')
        self.get_logger().info('Waiting for messages on /so101_follower/joint_states...')

    def joint_state_callback(self, msg: JointState):
        """
        Callback function called whenever a new JointState message is received.

        Args:
            msg: The incoming JointState message

        TODO: Implement this callback to:
        1. Extract joint names from msg.name
        2. Extract joint positions from msg.position
        3. Convert positions from radians to degrees
        4. Print each joint's name and position

        Expected output format:
            --- Joint States ---
            Joint 1 (Base):         45.0°
            Joint 2 (Shoulder):     30.0°
            Joint 3 (Elbow):       -15.0°
            Joint 4 (Wrist Pitch):  10.0°
            Joint 5 (Wrist Roll):    0.0°
            Joint 6 (Gripper):      25.0°

        Hints:
        - Use math.degrees() to convert radians to degrees
        - msg.name is a list of strings
        - msg.position is a list of floats
        - Use zip() to iterate over both lists together
        """

        # Joint name mapping for nice display
        joint_labels = {
            "1": "Base",
            "2": "Shoulder",
            "3": "Elbow",
            "4": "Wrist Pitch",
            "5": "Wrist Roll",
            "6": "Gripper"
        }

        # TODO: Implement the callback
        # Your code here...
        pass


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = JointStateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# =============================================================================
# TESTS - Run these to verify your implementation
# =============================================================================

def test_radians_to_degrees():
    """Test that radians to degrees conversion works."""
    assert abs(math.degrees(0) - 0.0) < 0.001
    assert abs(math.degrees(math.pi) - 180.0) < 0.001
    assert abs(math.degrees(math.pi / 2) - 90.0) < 0.001
    print("✓ Radians to degrees conversion test passed!")


def test_joint_labels():
    """Test that all joint labels are defined."""
    expected_joints = ["1", "2", "3", "4", "5", "6"]
    joint_labels = {
        "1": "Base",
        "2": "Shoulder",
        "3": "Elbow",
        "4": "Wrist Pitch",
        "5": "Wrist Roll",
        "6": "Gripper"
    }
    for j in expected_joints:
        assert j in joint_labels, f"Missing label for joint {j}"
    print("✓ Joint labels test passed!")


if __name__ == '__main__':
    # Run tests if executed directly with --test flag
    import sys
    if '--test' in sys.argv:
        test_radians_to_degrees()
        test_joint_labels()
        print("\nAll tests passed!")
    else:
        main()

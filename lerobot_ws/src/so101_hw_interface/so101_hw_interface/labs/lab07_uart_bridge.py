#!/usr/bin/env python3
"""
Week 7: UART Bridge - Arduino to ROS2

This ROS2 node reads commands from an Arduino over UART and publishes
them as joint commands to control the robot arm.

Data Flow:
    Arduino → USB Serial → This Node → /so101_follower/joint_commands

The node:
1. Opens a serial connection to the Arduino
2. Reads incoming command strings
3. Parses the commands (e.g., "J1+" → joint 1, positive direction)
4. Publishes incremental joint position updates

Protocol:
    "J1+"  → Move joint 1 in positive direction
    "J1-"  → Move joint 1 in negative direction
    "STOP" → Emergency stop
    "HOME" → Return to home position

Learning Objectives:
- Serial communication in Python (pyserial)
- ROS2 publisher nodes
- Command parsing and validation
- Real-time control loops
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import serial
import threading
from typing import Optional, Tuple


# =============================================================================
# CONFIGURATION
# =============================================================================

# Serial port settings
DEFAULT_PORT = "/dev/ttyUSB1"  # Arduino port (separate from robot)
DEFAULT_BAUD = 115200

# Movement settings
STEP_SIZE = 0.05  # Radians per button press
MAX_POSITION = 3.14  # Maximum joint position (radians)
MIN_POSITION = -3.14  # Minimum joint position (radians)

# Joint names
JOINTS = ["1", "2", "3", "4", "5", "6"]


# =============================================================================
# UART BRIDGE NODE
# =============================================================================

class UartBridgeNode(Node):
    """
    ROS2 node that bridges UART commands to joint commands.
    """

    def __init__(self):
        super().__init__('uart_bridge')

        # Declare parameters
        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baud', DEFAULT_BAUD)
        self.declare_parameter('step_size', STEP_SIZE)

        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.step_size = self.get_parameter('step_size').get_parameter_value().double_value

        # Current joint positions (start at home)
        self.joint_positions = {j: 0.0 for j in JOINTS}

        # E-stop state
        self.estop_active = False

        # -----------------------------------------------------------------
        # TODO: Create publisher for joint commands
        # -----------------------------------------------------------------
        # Publish JointState messages to 'so101_follower/joint_commands'
        # Queue size: 10

        self.joint_pub = None  # TODO: Create publisher

        # -----------------------------------------------------------------
        # TODO: Create client for torque service
        # -----------------------------------------------------------------
        # This allows us to disable torque on E-stop
        # Service: 'so101_follower/set_torque', type: SetBool

        self.torque_client = None  # TODO: Create service client

        # -----------------------------------------------------------------
        # Serial port setup
        # -----------------------------------------------------------------
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.1  # Non-blocking read with short timeout
            )
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {port}: {e}")
            self.serial_port = None

        # Start the serial reading thread
        if self.serial_port:
            self.running = True
            self.serial_thread = threading.Thread(target=self._serial_read_loop)
            self.serial_thread.daemon = True
            self.serial_thread.start()

        self.get_logger().info("UART Bridge started!")

    def _serial_read_loop(self):
        """
        Background thread that continuously reads from serial port.
        """
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.get_logger().debug(f"Received: {line}")
                        self._process_command(line)
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def _process_command(self, command: str):
        """
        Process a command received from the Arduino.

        Args:
            command: Raw command string (e.g., "J1+", "STOP", "HOME")

        TODO: Implement this method

        Steps:
        1. Check for special commands first:
           - "STOP" → Call handle_estop()
           - "HOME" → Call handle_home()
           - "READY" → Clear E-stop, log message

        2. For joint commands (e.g., "J1+"):
           - Parse using parse_joint_command()
           - If valid and not in E-stop, update position and publish

        Hints:
        - Use command.upper() to handle case variations
        - Check self.estop_active before processing joint commands
        """

        # Your code here
        pass

    def parse_joint_command(self, command: str) -> Optional[Tuple[str, int]]:
        """
        Parse a joint command string.

        Args:
            command: Command string like "J1+", "J3-", "j2+"

        Returns:
            Tuple of (joint_name, direction) or None if invalid
            - joint_name: "1" through "6"
            - direction: +1 for positive, -1 for negative

        Examples:
            "J1+" → ("1", 1)
            "J3-" → ("3", -1)
            "j2+" → ("2", 1)
            "J7+" → None (invalid joint)
            "ABC" → None (invalid format)

        TODO: Implement this function

        Steps:
        1. Convert to uppercase
        2. Check format: must start with 'J', have digit, end with +/-
        3. Extract joint number and validate (1-6)
        4. Extract direction (+ or -)
        5. Return tuple or None

        Hints:
        - Use command[0] to check first character
        - Use command[1:-1] to extract joint number
        - Use command[-1] to get direction
        - Validate joint is in JOINTS list
        """

        # Your code here
        raise NotImplementedError("TODO: Implement parse_joint_command")

    def update_joint_position(self, joint: str, direction: int):
        """
        Update a joint's target position and publish.

        Args:
            joint: Joint name ("1" through "6")
            direction: +1 or -1

        TODO: Implement this function

        Steps:
        1. Calculate new position:
           new_pos = current_pos + (direction * step_size)

        2. Clamp to limits:
           new_pos = max(MIN_POSITION, min(MAX_POSITION, new_pos))

        3. Update self.joint_positions[joint]

        4. Publish the command using publish_joint_command()
        """

        # Your code here
        raise NotImplementedError("TODO: Implement update_joint_position")

    def publish_joint_command(self):
        """
        Publish current joint positions as a JointState message.

        TODO: Implement this function

        Steps:
        1. Create a JointState message
        2. Set header.stamp to current time
        3. Set name to list of joint names
        4. Set position to list of current positions
        5. Publish the message

        Hints:
        - js = JointState()
        - js.header.stamp = self.get_clock().now().to_msg()
        - js.name = list(self.joint_positions.keys())
        - js.position = list(self.joint_positions.values())
        """

        # Your code here
        raise NotImplementedError("TODO: Implement publish_joint_command")

    def handle_estop(self):
        """
        Handle emergency stop command.

        TODO: Implement this function

        Steps:
        1. Set self.estop_active = True
        2. Log warning message
        3. Call torque service to disable torque (if client available)

        Calling the torque service:
        - Check if self.torque_client is not None
        - Create request: req = SetBool.Request(); req.data = False
        - Call asynchronously: self.torque_client.call_async(req)
        """

        # Your code here
        raise NotImplementedError("TODO: Implement handle_estop")

    def handle_home(self):
        """
        Handle home command - return all joints to zero.

        TODO: Implement this function

        Steps:
        1. Set all joint positions to 0.0
        2. Publish the command
        3. Log info message
        """

        # Your code here
        raise NotImplementedError("TODO: Implement handle_home")

    def destroy_node(self):
        """Clean up on shutdown."""
        self.running = False
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = UartBridgeNode()

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
# TESTS
# =============================================================================

def run_tests():
    """Run unit tests (without ROS2)."""
    print("Running UART bridge tests...\n")

    # Create a mock node for testing parse function
    class MockNode:
        def __init__(self):
            self.step_size = 0.05
            self.joint_positions = {j: 0.0 for j in JOINTS}
            self.estop_active = False

        def parse_joint_command(self, command):
            # Copy the implementation here for testing
            command = command.upper()
            if len(command) < 3:
                return None
            if command[0] != 'J':
                return None
            joint = command[1:-1]
            if joint not in JOINTS:
                return None
            direction_char = command[-1]
            if direction_char == '+':
                return (joint, 1)
            elif direction_char == '-':
                return (joint, -1)
            return None

    node = MockNode()

    # Test 1: Valid positive command
    print("Test 1: Parse 'J1+'")
    result = node.parse_joint_command("J1+")
    assert result == ("1", 1), f"Expected ('1', 1), got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 2: Valid negative command
    print("Test 2: Parse 'J3-'")
    result = node.parse_joint_command("J3-")
    assert result == ("3", -1), f"Expected ('3', -1), got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 3: Lowercase handling
    print("Test 3: Parse 'j2+' (lowercase)")
    result = node.parse_joint_command("j2+")
    assert result == ("2", 1), f"Expected ('2', 1), got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 4: Invalid joint number
    print("Test 4: Parse 'J7+' (invalid joint)")
    result = node.parse_joint_command("J7+")
    assert result is None, f"Expected None, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 5: Invalid format
    print("Test 5: Parse 'ABC' (invalid format)")
    result = node.parse_joint_command("ABC")
    assert result is None, f"Expected None, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 6: Missing direction
    print("Test 6: Parse 'J1' (missing direction)")
    result = node.parse_joint_command("J1")
    assert result is None, f"Expected None, got {result}"
    print(f"  ✓ Result: {result}\n")

    print("=" * 50)
    print("All tests passed!")
    print("=" * 50)


if __name__ == '__main__':
    import sys
    if '--test' in sys.argv:
        run_tests()
    else:
        main()

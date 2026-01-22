#!/usr/bin/env python3
"""
Week 4: Motor Bridge Skeleton

This is the core ROS2 node that bridges between ROS2 topics and the physical
Feetech servos. It:
1. Subscribes to joint commands from the GUI/controller
2. Publishes current joint states from servo readings
3. Provides a torque control service

The node runs at 50Hz, alternating between reading and writing to avoid
bus contention (the serial bus is half-duplex).

Architecture:
                                    ┌─────────────────┐
    /joint_commands ───────────────▶│                 │
                                    │  motor_bridge   │──────▶ Feetech Bus
    /joint_states  ◀────────────────│                 │◀────── (USB Serial)
                                    └─────────────────┘

Learning Objectives:
- Understand ROS2 node lifecycle
- Implement timer callbacks
- Work with publishers and subscribers
- Use ROS2 services
- Handle hardware communication errors

IMPORTANT: This skeleton uses the lerobot FeetechMotorsBus library.
You do NOT need to implement raw serial communication.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import math
import yaml
import pathlib

# Assume these are available (provided by instructor)
from so101_hw_interface.motors.feetech.feetech import FeetechMotorsBus
from so101_hw_interface.motors import Motor, MotorNormMode

# Import your position conversion functions from Week 3
# from skeleton.week3.position_conversion import radians_to_ticks, ticks_to_radians


# =============================================================================
# CONFIGURATION - Do not modify
# =============================================================================

PORT_DEFAULT = "/dev/ttyACM0"

JOINTS = {
    "1": {"id": 1, "model": "sts3215"},  # Base
    "2": {"id": 2, "model": "sts3215"},  # Shoulder
    "3": {"id": 3, "model": "sts3215"},  # Elbow
    "4": {"id": 4, "model": "sts3215"},  # Wrist Pitch
    "5": {"id": 5, "model": "sts3215"},  # Wrist Roll
    "6": {"id": 6, "model": "sts3215"},  # Gripper
}

# Default home positions (tick values when joints are at 0 radians)
DEFAULT_HOME_POSITIONS = {
    "1": 2048,
    "2": 2048,
    "3": 2048,
    "4": 2048,
    "5": 2048,
    "6": 2048,
}


# =============================================================================
# MOTOR BRIDGE NODE
# =============================================================================

class MotorBridgeSkeleton(Node):
    """
    ROS2 node that bridges joint commands/states with Feetech servos.
    """

    def __init__(self):
        super().__init__('motor_bridge_skeleton')

        # -----------------------------------------------------------------
        # Parameter declaration (provided)
        # -----------------------------------------------------------------
        self.declare_parameter("port", PORT_DEFAULT)
        port = self.get_parameter("port").get_parameter_value().string_value

        # -----------------------------------------------------------------
        # Motor bus setup (provided)
        # -----------------------------------------------------------------
        motors = {
            name: Motor(cfg["id"], cfg["model"], MotorNormMode.DEGREES)
            for name, cfg in JOINTS.items()
        }
        self.bus = FeetechMotorsBus(port, motors)

        self.get_logger().info(f"Connecting to Feetech bus on {port}...")
        try:
            self.bus.connect()
            self.bus.configure_motors()
            self.bus.enable_torque()
        except Exception as exc:
            self.get_logger().error(f"Could not open motor bus: {exc}")
            raise
        self.get_logger().info("Motor bus connected!")

        # -----------------------------------------------------------------
        # State variables
        # -----------------------------------------------------------------
        self.current_commands: dict[str, float] = {}  # Target positions (radians)
        self._home_offsets = DEFAULT_HOME_POSITIONS.copy()
        self._torque_enabled = True
        self._is_read_turn = True  # Alternates between read/write

        # Conversion constant
        self._ticks_per_radian = 4096.0 / (2 * math.pi)

        # -----------------------------------------------------------------
        # TODO: Create publisher for joint states
        # -----------------------------------------------------------------
        # Create a publisher that publishes JointState messages to
        # 'so101_follower/joint_states' with queue size 10
        #
        # Hint: self.create_publisher(MessageType, 'topic_name', queue_size)

        self.joint_states_pub = None  # TODO: Replace with actual publisher

        # -----------------------------------------------------------------
        # TODO: Create subscriber for joint commands
        # -----------------------------------------------------------------
        # Create a subscriber that listens to JointState messages on
        # 'so101_follower/joint_commands' and calls self._command_callback
        #
        # Hint: self.create_subscription(MessageType, 'topic', callback, queue)

        self.joint_commands_sub = None  # TODO: Replace with actual subscriber

        # -----------------------------------------------------------------
        # TODO: Create service for torque control
        # -----------------------------------------------------------------
        # Create a service of type SetBool on 'so101_follower/set_torque'
        # that calls self._torque_callback
        #
        # Hint: self.create_service(ServiceType, 'service_name', callback)

        self.torque_srv = None  # TODO: Replace with actual service

        # -----------------------------------------------------------------
        # Timer for periodic read/write (provided)
        # -----------------------------------------------------------------
        self.timer = self.create_timer(0.02, self._timer_callback)  # 50 Hz
        self.get_logger().info("Motor bridge started with 50Hz timer.")

    # =====================================================================
    # CALLBACKS - Implement these
    # =====================================================================

    def _command_callback(self, msg: JointState):
        """
        Callback for incoming joint commands.

        This is called whenever a new JointState message arrives on the
        /so101_follower/joint_commands topic. Store the commanded positions
        so they can be written to the servos in the next write cycle.

        Args:
            msg: JointState message containing:
                - msg.name: List of joint names (e.g., ["1", "2", "3"])
                - msg.position: List of positions in radians

        TODO: Implement this callback

        Hints:
        - Use zip(msg.name, msg.position) to iterate over both lists
        - Only store positions for valid joint names (check if name in JOINTS)
        - Store in self.current_commands dict: self.current_commands[name] = position
        """

        # Your code here
        pass

    def _torque_callback(self, request, response):
        """
        Service callback for torque enable/disable.

        This allows the GUI to turn torque on/off so users can manually
        move the arm (torque off) or have it hold position (torque on).

        Args:
            request: SetBool.Request with request.data = True/False
            response: SetBool.Response to fill in

        Returns:
            The filled response object

        TODO: Implement this callback

        Steps:
        1. If request.data is True:
           - Call self.bus.enable_torque()
           - Set self._torque_enabled = True
           - Set response.success = True
           - Set response.message = "Torque enabled"
        2. If request.data is False:
           - Call self.bus.disable_torque()
           - Set self._torque_enabled = False
           - Set response.success = True
           - Set response.message = "Torque disabled"
        3. Wrap in try/except to handle errors
        4. Return response
        """

        # Your code here
        response.success = False
        response.message = "Not implemented"
        return response

    def _timer_callback(self):
        """
        Timer callback that alternates between reading and writing.

        Called every 20ms (50Hz). On even calls, reads servo positions.
        On odd calls, writes commanded positions to servos.

        This alternation prevents bus contention since the serial bus
        is half-duplex (can only send OR receive at once).
        """
        if self._is_read_turn:
            self._do_read()
        else:
            self._do_write()

        self._is_read_turn = not self._is_read_turn

    def _do_read(self):
        """
        Read current positions from servos and publish as JointState.

        TODO: Implement this method

        Steps:
        1. Read raw positions from servos:
           raw_positions = self.bus.sync_read("Present_Position", normalize=False)
           This returns a dict like {"1": 2048, "2": 2100, ...}

        2. Create a JointState message:
           js = JointState()
           js.header.stamp = self.get_clock().now().to_msg()
           js.name = list(JOINTS.keys())  # ["1", "2", "3", "4", "5", "6"]

        3. Convert each raw tick value to radians:
           For each joint, calculate:
           radians = (raw_ticks - homing_offset) * (2π / 4096)
           Store in js.position list

        4. Publish the message:
           self.joint_states_pub.publish(js)

        5. Wrap in try/except to handle communication errors

        Hints:
        - Use self._home_offsets[joint_name] to get homing offset
        - Use list comprehension to build js.position
        """

        # Your code here
        pass

    def _do_write(self):
        """
        Write commanded positions to servos.

        TODO: Implement this method

        Steps:
        1. If torque is disabled, return early (don't write)
           if not self._torque_enabled:
               return

        2. If no commands have been received, return early
           if not self.current_commands:
               return

        3. Convert each commanded position (radians) to ticks:
           For each joint in self.current_commands:
           raw_ticks = homing_offset + (radians * 4096 / 2π)
           Round to integer

        4. Write to servos:
           self.bus.sync_write("Goal_Position", raw_goals, normalize=False)
           Where raw_goals is a dict like {"1": 2048, "2": 2100, ...}

        5. Wrap in try/except to handle communication errors

        Hints:
        - Use self._ticks_per_radian for conversion
        - Use int() or round() to convert to integer ticks
        """

        # Your code here
        pass


# =============================================================================
# MAIN
# =============================================================================

def main():
    rclpy.init()
    node = MotorBridgeSkeleton()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.bus.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

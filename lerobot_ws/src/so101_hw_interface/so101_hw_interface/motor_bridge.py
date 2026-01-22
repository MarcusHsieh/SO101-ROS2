#!/usr/bin/env python3
"""ROS 2 node that bridges Feetech servos <-> topic_based_ros2_control.

It:
* subscribes to `/so101_follower/joint_commands` (sensor_msgs/JointState)
  and writes Goal_Position to the servos.
* publishes `/so101_follower/joint_states` every 20 ms using Present_Position.

Make sure the USB-to-UART adapter of the servo bus is accessible
(e.g. `/dev/ttyUSB0`) and you have permission to open it.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
import sys
import math
import yaml, pathlib
from ament_index_python.packages import get_package_share_directory

# -----------------------------------------------------------------------------
# Provide lightweight stubs for optional third-party dependencies that the
# Feetech SDK imports but are not strictly required to operate the motors.
# -----------------------------------------------------------------------------
try:
    import deepdiff  # type: ignore
except ImportError:  # create stub
    import types
    deepdiff_stub = types.ModuleType("deepdiff")
    class _DD(dict):
        def __init__(self, *args, **kwargs):
            # Accept any arguments but don't do anything - just a stub
            pass
    deepdiff_stub.DeepDiff = _DD  # type: ignore
    sys.modules["deepdiff"] = deepdiff_stub

try:
    import tqdm  # type: ignore
except ImportError:  # create stub
    import types
    tqdm_stub = types.ModuleType("tqdm")
    def _tqdm(iterable=None, **kwargs):
        return iterable if iterable is not None else []
    tqdm_stub.tqdm = _tqdm  # type: ignore
    sys.modules["tqdm"] = tqdm_stub

# -----------------------------------------------------------------------------

from so101_hw_interface.motors.feetech.feetech import FeetechMotorsBus
from so101_hw_interface.motors import Motor, MotorNormMode

PORT_DEFAULT = "/dev/ttyACM0"

JOINTS = {
    "1": {"id": 1, "model": "sts3215"},
    "2": {"id": 2, "model": "sts3215"},
    "3": {"id": 3, "model": "sts3215"},
    "4": {"id": 4, "model": "sts3215"},
    "5": {"id": 5, "model": "sts3215"},
    "6": {"id": 6, "model": "sts3215"},
}

# Default initial positions (used if no calibration file exists)
# These should match the calibration file's homing_offset values
DEFAULT_HOME_POSITIONS = {
     "1": 2091,
     "2": 2029,
     "3": 2081,
     "4": 1997,
     "5": 3051,
     "6": 2137,
}

CALIB_FILE = (
    pathlib.Path(get_package_share_directory("so101_hw_interface"))
    / "config/so101_calibration.yaml"
)

class MotorBridge(Node):
    def __init__(self):
        super().__init__("so101_motor_bridge")
        # Declare parameters so they can be overridden from launch/CLI
        self.declare_parameter("port", PORT_DEFAULT)
        self.declare_parameter("calib_file", str(CALIB_FILE))

        port = self.get_parameter("port").get_parameter_value().string_value
        if not port:
            port = PORT_DEFAULT

        # Build motor objects
        motors = {
            name: Motor(cfg["id"], cfg["model"], MotorNormMode.DEGREES)
            for name, cfg in JOINTS.items()
        }
        self.bus = FeetechMotorsBus(port, motors)

        self.get_logger().info(f"Connecting to Feetech bus on {port} …")
        try:
            self.bus.connect()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Could not open motor bus: {exc}")
            raise
        self.bus.configure_motors()
        self.bus.enable_torque()
        self.get_logger().info("Motor bus connected and configured.")

        # Publishers / Subscribers
        self.joint_states_pub = self.create_publisher(JointState, "so101_follower/joint_states", 10)
        self.joint_commands_sub = self.create_subscription(
            JointState,
            "so101_follower/joint_commands",
            self._command_cb,
            10,
        )

        # State variables
        self.current_commands: dict[str, float] = {}
        self._home_offsets: dict[str, int] | None = None
        self._limits: dict[str, tuple[int, int]] | None = None
        self._initial_move_done = False
        self._is_read_turn = True # Flag to alternate between read and write
        self._torque_enabled = True  # Track torque state

        self._steps_per_rad = 4096.0 / (2 * math.pi)

        # Service for torque control
        self.torque_srv = self.create_service(
            SetBool,
            'so101_follower/set_torque',
            self._torque_callback
        )

        # Load calibration
        calib_path = pathlib.Path(self.get_parameter("calib_file").get_parameter_value().string_value).expanduser()
        if calib_path.is_file():
            with open(calib_path, "r", encoding="utf-8") as fp:
                calib = yaml.safe_load(fp)
            self._limits = {j: (calib[j]["range_min"], calib[j]["range_max"]) for j in JOINTS if j in calib}
            # Load homing offsets from calibration file
            self._home_offsets = {j: calib[j]["homing_offset"] for j in JOINTS if j in calib}
            self.get_logger().info(f"Loaded calibration from {calib_path}")
            self.get_logger().info(f"  Homing offsets: {self._home_offsets}")
        else:
            self.get_logger().warn(f"Calibration file {calib_path} not found, using defaults.")
            self._home_offsets = DEFAULT_HOME_POSITIONS.copy()

        # Timer for periodic read/write (50 Hz)
        self.timer = self.create_timer(0.02, self._timer_cb)
        self.get_logger().info("Motor bridge node started with alternating read/write timer.")

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def _command_cb(self, msg: JointState):
        """Store desired joint positions from topic (rad)."""
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                self.current_commands[name] = pos

    def _torque_callback(self, request, response):
        """Handle torque enable/disable service calls."""
        try:
            if request.data:
                self.bus.enable_torque()
                self._torque_enabled = True
                response.success = True
                response.message = "Torque enabled"
                self.get_logger().info("Torque ENABLED")
            else:
                self.bus.disable_torque()
                self._torque_enabled = False
                response.success = True
                response.message = "Torque disabled"
                self.get_logger().info("Torque DISABLED - you can now move the arm manually")
        except Exception as exc:
            response.success = False
            response.message = f"Failed: {exc}"
            self.get_logger().error(f"Torque control failed: {exc}")
        return response

    def _timer_cb(self):
        # On each timer tick, we either do a read or a write, but never both.
        if self._is_read_turn:
            self._do_read()
        else:
            self._do_write()

        # Flip the flag for the next turn
        self._is_read_turn = not self._is_read_turn

    def _do_read(self):
        try:
            raw_positions = self.bus.sync_read("Present_Position", normalize=False)
        except Exception as exc:
            self.get_logger().warn(f"sync_read failed: {exc}")
            return

        # Publish current joint states (using calibrated home offsets)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINTS.keys())
        js.position = [
            (raw_positions.get(n, 0) - self._home_offsets.get(n, 2048)) * (2 * math.pi) / 4096.0
            for n in JOINTS.keys()
        ]
        self.joint_states_pub.publish(js)

    def _do_write(self):
        # Skip writing if torque is disabled (user is manually moving the arm)
        if not self._torque_enabled:
            return

        # On the first write cycle, command the motors to their calibrated home position.
        if not self._initial_move_done:
            initial_positions = {n: self._home_offsets.get(n, 2048) for n in JOINTS.keys()}
            self.get_logger().info(f"Commanding initial (home) pose: {initial_positions}")
            try:
                self.bus.sync_write("Goal_Position", initial_positions, normalize=False)
                self._initial_move_done = True
            except Exception as exc:
                self.get_logger().error(f"Failed to write initial pose: {exc}")
            return  # Skip regular command on this turn

        # On subsequent write cycles, send the latest commands from the topic.
        if not self.current_commands:
            return # Nothing to write

        try:
            raw_goals = {}
            for n, rad in self.current_commands.items():
                home = self._home_offsets.get(n, 0)
                raw = int(home + rad * self._steps_per_rad)
                if self._limits and n in self._limits:
                    low, high = self._limits[n]
                    raw = max(low, min(high, raw))
                raw_goals[n] = raw
            self.bus.sync_write("Goal_Position", raw_goals, normalize=False)
        except Exception as exc:
            self.get_logger().warn(f"sync_write failed: {exc}")


# -------------------------------------------------------------------------
# Main
# -------------------------------------------------------------------------

def main():  # noqa: D401
    """Entry-point."""
    rclpy.init()
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Disconnecting from motor bus...")
        node.bus.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
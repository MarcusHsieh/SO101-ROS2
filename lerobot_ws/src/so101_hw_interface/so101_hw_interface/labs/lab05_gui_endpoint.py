#!/usr/bin/env python3
"""
Week 5: GUI Endpoint Handler

This module handles HTTP requests from the web-based joint control GUI.
The GUI sends JSON requests to control joint positions and get robot state.

API Endpoints:
- POST /joint_command - Set target position for a joint
- POST /torque - Enable/disable torque
- GET /state - Get current joint positions

Learning Objectives:
- Handle HTTP requests in Python
- Parse and validate JSON data
- Design simple REST APIs
- Return appropriate error responses

This skeleton focuses on the request handling logic.
The HTTP server infrastructure is provided.
"""

import json
import math
from typing import Optional


# Joint configuration
JOINTS = ["1", "2", "3", "4", "5", "6"]

# Joint limits in radians (approximate safe range)
JOINT_LIMITS = {
    "1": (-math.pi, math.pi),       # Base: ±180°
    "2": (-math.pi/2, math.pi/2),   # Shoulder: ±90°
    "3": (-math.pi/2, math.pi/2),   # Elbow: ±90°
    "4": (-math.pi/2, math.pi/2),   # Wrist Pitch: ±90°
    "5": (-math.pi, math.pi),       # Wrist Roll: ±180°
    "6": (0, math.pi/2),            # Gripper: 0-90°
}


class JointCommandHandler:
    """
    Handles joint command requests from the GUI.

    This class processes incoming HTTP requests, validates data,
    and updates the robot's target positions.
    """

    def __init__(self):
        """Initialize the handler with empty command state."""
        # Current target positions for each joint (radians)
        self.target_positions: dict[str, float] = {j: 0.0 for j in JOINTS}

        # Torque state
        self.torque_enabled: bool = True

    def handle_joint_command(self, request_data: dict) -> dict:
        """
        Process a joint command request from the GUI.

        The GUI sends requests like:
        {
            "joint": "1",
            "position": 0.5
        }

        Args:
            request_data: Dictionary containing:
                - "joint": Joint name as string ("1" through "6")
                - "position": Target position in radians (float)

        Returns:
            Response dictionary with:
                - "success": bool
                - "message": str (description or error message)
                - "clamped": bool (True if position was clamped to limits)
                - "final_position": float (actual position set, after clamping)

        TODO: Implement this function

        Steps:
        1. Extract 'joint' from request_data
           - If missing, return error response

        2. Validate joint name is valid (in JOINTS list)
           - If invalid, return error response

        3. Extract 'position' from request_data
           - If missing, return error response
           - If not a number, return error response

        4. Clamp position to joint limits
           - Get limits from JOINT_LIMITS[joint]
           - Use max(min_limit, min(position, max_limit))
           - Track if clamping was needed

        5. Update self.target_positions[joint]

        6. Return success response with final position

        Example responses:
            Success: {"success": True, "message": "Joint 1 set to 0.5 rad", "clamped": False, "final_position": 0.5}
            Error:   {"success": False, "message": "Invalid joint name: 7"}
        """

        # Your code here
        raise NotImplementedError("TODO: Implement handle_joint_command")

    def handle_torque_request(self, request_data: dict) -> dict:
        """
        Process a torque enable/disable request.

        The GUI sends requests like:
        {
            "enable": true
        }

        Args:
            request_data: Dictionary containing:
                - "enable": bool (True to enable torque, False to disable)

        Returns:
            Response dictionary with:
                - "success": bool
                - "message": str
                - "torque_enabled": bool (current state after update)

        TODO: Implement this function

        Steps:
        1. Extract 'enable' from request_data
           - If missing, return error response
           - If not a boolean, return error response

        2. Update self.torque_enabled

        3. Return success response

        In a real implementation, this would also call the ROS2 service
        to actually enable/disable torque on the robot. For this skeleton,
        we just track the state.
        """

        # Your code here
        raise NotImplementedError("TODO: Implement handle_torque_request")

    def handle_get_state(self) -> dict:
        """
        Return current robot state for the GUI.

        Returns:
            Response dictionary with:
                - "success": bool
                - "joints": dict mapping joint names to positions (radians)
                - "torque_enabled": bool

        TODO: Implement this function

        This should return the current target_positions and torque state.
        """

        # Your code here
        raise NotImplementedError("TODO: Implement handle_get_state")

    def handle_home_request(self) -> dict:
        """
        Move all joints to home position (0 radians).

        Returns:
            Response dictionary with:
                - "success": bool
                - "message": str

        TODO: Implement this function

        Set all target_positions to 0.0.
        """

        # Your code here
        raise NotImplementedError("TODO: Implement handle_home_request")


def validate_joint_name(joint: str) -> bool:
    """
    Check if a joint name is valid.

    Args:
        joint: Joint name to validate

    Returns:
        True if valid, False otherwise

    TODO: Implement this helper function
    """

    # Your code here
    raise NotImplementedError("TODO: Implement validate_joint_name")


def clamp_to_limits(position: float, joint: str) -> tuple[float, bool]:
    """
    Clamp a position to the joint's limits.

    Args:
        position: Desired position in radians
        joint: Joint name (must be valid)

    Returns:
        Tuple of (clamped_position, was_clamped)

    TODO: Implement this helper function
    """

    # Your code here
    raise NotImplementedError("TODO: Implement clamp_to_limits")


# =============================================================================
# TESTS
# =============================================================================

def run_tests():
    """Run unit tests for the handler."""
    print("Running GUI endpoint tests...\n")

    handler = JointCommandHandler()

    # Test 1: Valid joint command
    print("Test 1: Valid joint command")
    response = handler.handle_joint_command({"joint": "1", "position": 0.5})
    assert response["success"] == True, f"Expected success, got {response}"
    assert response["final_position"] == 0.5, f"Expected 0.5, got {response['final_position']}"
    print(f"  ✓ Response: {response}\n")

    # Test 2: Invalid joint name
    print("Test 2: Invalid joint name")
    response = handler.handle_joint_command({"joint": "7", "position": 0.5})
    assert response["success"] == False, f"Expected failure, got {response}"
    print(f"  ✓ Response: {response}\n")

    # Test 3: Missing joint field
    print("Test 3: Missing joint field")
    response = handler.handle_joint_command({"position": 0.5})
    assert response["success"] == False, f"Expected failure, got {response}"
    print(f"  ✓ Response: {response}\n")

    # Test 4: Position clamping (exceeds max)
    print("Test 4: Position clamping (exceeds limit)")
    response = handler.handle_joint_command({"joint": "2", "position": 5.0})  # > π/2
    assert response["success"] == True, f"Expected success, got {response}"
    assert response["clamped"] == True, f"Expected clamped=True, got {response}"
    assert response["final_position"] <= math.pi/2 + 0.001, f"Should be clamped to π/2"
    print(f"  ✓ Response: {response}\n")

    # Test 5: Torque enable
    print("Test 5: Enable torque")
    response = handler.handle_torque_request({"enable": True})
    assert response["success"] == True, f"Expected success, got {response}"
    assert response["torque_enabled"] == True, f"Expected torque_enabled=True"
    print(f"  ✓ Response: {response}\n")

    # Test 6: Torque disable
    print("Test 6: Disable torque")
    response = handler.handle_torque_request({"enable": False})
    assert response["success"] == True, f"Expected success, got {response}"
    assert response["torque_enabled"] == False, f"Expected torque_enabled=False"
    print(f"  ✓ Response: {response}\n")

    # Test 7: Get state
    print("Test 7: Get state")
    response = handler.handle_get_state()
    assert response["success"] == True, f"Expected success, got {response}"
    assert "joints" in response, f"Expected 'joints' in response"
    assert len(response["joints"]) == 6, f"Expected 6 joints"
    print(f"  ✓ Response: {response}\n")

    # Test 8: Home request
    print("Test 8: Home request")
    handler.target_positions["1"] = 1.0  # Set non-zero first
    response = handler.handle_home_request()
    assert response["success"] == True, f"Expected success, got {response}"
    assert handler.target_positions["1"] == 0.0, f"Expected position reset to 0"
    print(f"  ✓ Response: {response}\n")

    # Test 9: validate_joint_name helper
    print("Test 9: validate_joint_name")
    assert validate_joint_name("1") == True
    assert validate_joint_name("6") == True
    assert validate_joint_name("7") == False
    assert validate_joint_name("") == False
    print("  ✓ All validation tests passed\n")

    # Test 10: clamp_to_limits helper
    print("Test 10: clamp_to_limits")
    pos, clamped = clamp_to_limits(0.5, "1")
    assert clamped == False, "Should not clamp 0.5 for joint 1"
    pos, clamped = clamp_to_limits(5.0, "1")
    assert clamped == True, "Should clamp 5.0 for joint 1"
    assert pos <= math.pi, f"Clamped value should be <= π, got {pos}"
    print("  ✓ Clamping tests passed\n")

    print("=" * 50)
    print("All tests passed!")
    print("=" * 50)


if __name__ == '__main__':
    run_tests()

#!/usr/bin/env python3
"""
Week 6: Calibration Capture

This module handles capturing and saving robot calibration data.
Calibration establishes the "home" position for each joint - the servo
tick value that corresponds to 0 radians.

Calibration Process:
1. Disable torque (so arm can be moved freely)
2. Physically position the arm at the home pose
3. Read current servo tick values
4. Save these as homing_offset values in YAML file

The calibration file format (YAML):
    1:
      homing_offset: 2091
      range_min: 1000
      range_max: 3000
    2:
      homing_offset: 2029
      range_min: 1000
      range_max: 3000
    ...

Learning Objectives:
- Work with YAML files in Python
- Understand robot calibration concepts
- Handle file I/O safely
- Design user-friendly calibration workflows
"""

import yaml
import pathlib
from typing import Optional
from datetime import datetime


# Default calibration values (approximate center positions)
DEFAULT_CALIBRATION = {
    "1": {"homing_offset": 2048, "range_min": 500, "range_max": 3500},
    "2": {"homing_offset": 2048, "range_min": 500, "range_max": 3500},
    "3": {"homing_offset": 2048, "range_min": 500, "range_max": 3500},
    "4": {"homing_offset": 2048, "range_min": 500, "range_max": 3500},
    "5": {"homing_offset": 2048, "range_min": 500, "range_max": 3500},
    "6": {"homing_offset": 2048, "range_min": 500, "range_max": 3500},
}


def load_calibration(filepath: str) -> dict:
    """
    Load calibration data from a YAML file.

    Args:
        filepath: Path to the calibration YAML file

    Returns:
        Dictionary with calibration data, or DEFAULT_CALIBRATION if file
        doesn't exist or is invalid

    TODO: Implement this function

    Steps:
    1. Convert filepath to a Path object
    2. Check if the file exists
       - If not, print a warning and return DEFAULT_CALIBRATION.copy()
    3. Open and read the file
    4. Parse YAML content using yaml.safe_load()
    5. Validate the structure (should have keys "1" through "6")
       - If invalid, print warning and return DEFAULT_CALIBRATION.copy()
    6. Return the loaded calibration

    Hints:
    - Use pathlib.Path(filepath) to create a Path object
    - Use path.is_file() to check existence
    - Use yaml.safe_load(file_handle) to parse YAML
    - Always use 'with open(...) as f:' for file handling
    """

    # Your code here
    raise NotImplementedError("TODO: Implement load_calibration")


def save_calibration(calibration: dict, filepath: str) -> bool:
    """
    Save calibration data to a YAML file.

    Args:
        calibration: Dictionary with calibration data
        filepath: Path to save the YAML file

    Returns:
        True if successful, False otherwise

    TODO: Implement this function

    Steps:
    1. Validate the calibration data structure
    2. Create parent directories if they don't exist
    3. Write the YAML file with nice formatting
    4. Return True on success, False on any error

    Hints:
    - Use pathlib.Path(filepath).parent.mkdir(parents=True, exist_ok=True)
    - Use yaml.dump(data, file, default_flow_style=False, sort_keys=False)
    - Wrap in try/except to catch write errors
    """

    # Your code here
    raise NotImplementedError("TODO: Implement save_calibration")


def capture_home_position(current_ticks: dict[str, int], calibration: dict) -> dict:
    """
    Update calibration with current tick values as new home positions.

    This is called when the user has physically positioned the arm at
    the desired home pose and wants to capture it.

    Args:
        current_ticks: Dictionary mapping joint names to current tick readings
                      e.g., {"1": 2091, "2": 2029, "3": 2081, ...}
        calibration: Existing calibration dictionary to update

    Returns:
        Updated calibration dictionary with new homing_offset values

    TODO: Implement this function

    Steps:
    1. Create a copy of the calibration dict (don't modify original)
    2. For each joint in current_ticks:
       - Update the homing_offset in the calibration copy
    3. Return the updated calibration

    Hints:
    - Use dict.copy() or copy.deepcopy() to avoid modifying original
    - Make sure to handle joints that might be missing from calibration
    """

    # Your code here
    raise NotImplementedError("TODO: Implement capture_home_position")


def capture_joint_limit(joint: str, current_ticks: int, limit_type: str, calibration: dict) -> dict:
    """
    Update calibration with a joint limit.

    This is called when the user has moved a joint to its physical limit
    and wants to record that limit.

    Args:
        joint: Joint name ("1" through "6")
        current_ticks: Current tick reading at the limit position
        limit_type: Either "min" or "max"
        calibration: Existing calibration dictionary to update

    Returns:
        Updated calibration dictionary with new limit value

    TODO: Implement this function

    Steps:
    1. Validate joint name is valid
    2. Validate limit_type is "min" or "max"
    3. Create a copy of calibration
    4. Update the appropriate limit (range_min or range_max)
    5. Return updated calibration

    Safety note: Add a small margin (e.g., 50 ticks) from the physical
    limit to prevent the servo from hitting hard stops.
    """

    SAFETY_MARGIN = 50  # Ticks to keep away from physical limits

    # Your code here
    raise NotImplementedError("TODO: Implement capture_joint_limit")


def validate_calibration(calibration: dict) -> tuple[bool, list[str]]:
    """
    Validate a calibration dictionary.

    Args:
        calibration: Calibration dictionary to validate

    Returns:
        Tuple of (is_valid: bool, errors: list[str])
        errors is a list of error messages, empty if valid

    TODO: Implement this function

    Validation checks:
    1. All 6 joints present ("1" through "6")
    2. Each joint has homing_offset, range_min, range_max
    3. homing_offset is within range_min and range_max
    4. range_min < range_max
    5. All values are integers
    6. Values are in reasonable range (0-4095 for tick values)
    """

    errors = []

    # Your code here
    raise NotImplementedError("TODO: Implement validate_calibration")


def format_calibration_report(calibration: dict) -> str:
    """
    Generate a human-readable calibration report.

    Args:
        calibration: Calibration dictionary

    Returns:
        Formatted string report

    TODO: Implement this function

    The report should look something like:
    ```
    === Robot Calibration Report ===
    Generated: 2024-01-15 10:30:00

    Joint 1 (Base):
      Home Position: 2091 ticks
      Range: 500 - 3500 ticks
      Range in degrees: -135° to +135°

    Joint 2 (Shoulder):
      ...
    ```

    Hints:
    - Use datetime.now().strftime("%Y-%m-%d %H:%M:%S") for timestamp
    - Convert ticks to degrees: degrees = (ticks - 2048) * 360 / 4096
    """

    # Your code here
    raise NotImplementedError("TODO: Implement format_calibration_report")


# =============================================================================
# TESTS
# =============================================================================

def run_tests():
    """Run unit tests."""
    import tempfile
    import os

    print("Running calibration tests...\n")

    # Test 1: Load non-existent file returns defaults
    print("Test 1: Load non-existent file")
    result = load_calibration("/nonexistent/path/calibration.yaml")
    assert result == DEFAULT_CALIBRATION, f"Expected defaults, got {result}"
    print("  ✓ Returns default calibration\n")

    # Test 2: Save and load calibration
    print("Test 2: Save and load calibration")
    with tempfile.TemporaryDirectory() as tmpdir:
        filepath = os.path.join(tmpdir, "test_calib.yaml")
        test_calib = DEFAULT_CALIBRATION.copy()
        test_calib["1"]["homing_offset"] = 2100

        success = save_calibration(test_calib, filepath)
        assert success == True, "Save should succeed"

        loaded = load_calibration(filepath)
        assert loaded["1"]["homing_offset"] == 2100, f"Expected 2100, got {loaded['1']['homing_offset']}"
    print("  ✓ Save and load working\n")

    # Test 3: Capture home position
    print("Test 3: Capture home position")
    current_ticks = {"1": 2091, "2": 2029, "3": 2081, "4": 1997, "5": 3051, "6": 2137}
    updated = capture_home_position(current_ticks, DEFAULT_CALIBRATION)

    # Original should be unchanged
    assert DEFAULT_CALIBRATION["1"]["homing_offset"] == 2048, "Original should not change"
    # Updated should have new values
    assert updated["1"]["homing_offset"] == 2091, f"Expected 2091, got {updated['1']['homing_offset']}"
    print("  ✓ Home position captured correctly\n")

    # Test 4: Capture joint limit
    print("Test 4: Capture joint limit")
    updated = capture_joint_limit("1", 3400, "max", DEFAULT_CALIBRATION)
    # Should have safety margin
    assert updated["1"]["range_max"] <= 3400, f"Should be <= 3400 (with safety margin)"
    print(f"  ✓ Limit captured: {updated['1']['range_max']}\n")

    # Test 5: Validate calibration
    print("Test 5: Validate calibration")
    is_valid, errors = validate_calibration(DEFAULT_CALIBRATION)
    assert is_valid == True, f"Default calibration should be valid, errors: {errors}"
    print("  ✓ Valid calibration passes\n")

    # Test 6: Invalid calibration detection
    print("Test 6: Detect invalid calibration")
    bad_calib = {"1": {"homing_offset": 2048}}  # Missing joints and fields
    is_valid, errors = validate_calibration(bad_calib)
    assert is_valid == False, "Should detect invalid calibration"
    assert len(errors) > 0, "Should have error messages"
    print(f"  ✓ Detected errors: {errors[:2]}...\n")

    # Test 7: Format report
    print("Test 7: Format calibration report")
    report = format_calibration_report(DEFAULT_CALIBRATION)
    assert "Joint 1" in report, "Report should mention joints"
    assert "Home Position" in report or "homing" in report.lower(), "Report should show home position"
    print(f"  ✓ Report generated ({len(report)} chars)\n")

    print("=" * 50)
    print("All tests passed!")
    print("=" * 50)


if __name__ == '__main__':
    run_tests()

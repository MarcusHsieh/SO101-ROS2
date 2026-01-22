#!/usr/bin/env python3
"""
Week 3: Position Conversion Functions

This module contains functions to convert between:
- Radians (ROS2 standard for joint angles)
- Servo ticks (Feetech STS3215 native units)

The STS3215 servo specifications:
- Resolution: 4096 ticks per full revolution (360°)
- This means: 4096 ticks = 2π radians = 360°
- Tick value 0-4095 represents position
- The "homing_offset" is the tick value when the joint is at 0 radians

Learning Objectives:
- Understand servo position encoding
- Implement bidirectional conversion functions
- Handle calibration offsets
- Write testable, pure functions

Example:
    If homing_offset = 2048 (servo center position):
    - When servo reads 2048 ticks → joint is at 0 radians
    - When servo reads 3072 ticks → joint is at +π/2 radians (90°)
    - When servo reads 1024 ticks → joint is at -π/2 radians (-90°)
"""

import math

# Constants
TICKS_PER_REVOLUTION = 4096
RADIANS_PER_REVOLUTION = 2 * math.pi


def radians_to_ticks(radians: float, homing_offset: int) -> int:
    """
    Convert a joint angle in radians to a servo tick value.

    The conversion formula:
        ticks = homing_offset + (radians * TICKS_PER_REVOLUTION / 2π)

    Args:
        radians: Joint angle in radians.
                 Positive = counterclockwise (when viewed from above)
                 Can be any float value (not limited to 0-2π)
        homing_offset: The tick value that corresponds to 0 radians.
                       This comes from the calibration file.
                       Typically around 2048 (center of range).

    Returns:
        Servo tick value as an integer (typically 0-4095, but may exceed
        this range if the input radians is large).

    Examples:
        >>> radians_to_ticks(0.0, 2048)
        2048
        >>> radians_to_ticks(math.pi, 2048)  # 180 degrees
        4096
        >>> radians_to_ticks(-math.pi/2, 2048)  # -90 degrees
        1024

    TODO: Implement this function

    Hints:
    - Calculate ticks_per_radian = TICKS_PER_REVOLUTION / (2 * math.pi)
    - Multiply radians by ticks_per_radian to get tick offset
    - Add homing_offset to get absolute tick position
    - Round to nearest integer
    """

    # Your code here
    raise NotImplementedError("TODO: Implement radians_to_ticks")


def ticks_to_radians(ticks: int, homing_offset: int) -> float:
    """
    Convert a servo tick value to a joint angle in radians.

    This is the inverse of radians_to_ticks().

    The conversion formula:
        radians = (ticks - homing_offset) * 2π / TICKS_PER_REVOLUTION

    Args:
        ticks: Raw servo position reading (typically 0-4095)
        homing_offset: The tick value that corresponds to 0 radians.

    Returns:
        Joint angle in radians as a float.

    Examples:
        >>> ticks_to_radians(2048, 2048)
        0.0
        >>> ticks_to_radians(4096, 2048)  # Should be ~π
        3.14159...
        >>> ticks_to_radians(1024, 2048)  # Should be ~-π/2
        -1.5707...

    TODO: Implement this function

    Hints:
    - Subtract homing_offset from ticks to get offset from zero
    - Divide by ticks_per_radian to convert to radians
    """

    # Your code here
    raise NotImplementedError("TODO: Implement ticks_to_radians")


def clamp_ticks(ticks: int, min_ticks: int, max_ticks: int) -> int:
    """
    Clamp a tick value to be within safe joint limits.

    This is a safety function to prevent commanding the servo
    outside its physical range of motion.

    Args:
        ticks: The desired tick value
        min_ticks: Minimum allowed tick value (from calibration)
        max_ticks: Maximum allowed tick value (from calibration)

    Returns:
        The tick value clamped to [min_ticks, max_ticks]

    Examples:
        >>> clamp_ticks(5000, 1000, 3000)
        3000
        >>> clamp_ticks(500, 1000, 3000)
        1000
        >>> clamp_ticks(2000, 1000, 3000)
        2000

    TODO: Implement this function

    Hints:
    - Use max() and min() functions
    - Or use: max(min_ticks, min(ticks, max_ticks))
    """

    # Your code here
    raise NotImplementedError("TODO: Implement clamp_ticks")


def radians_to_degrees(radians: float) -> float:
    """
    Convert radians to degrees.

    Args:
        radians: Angle in radians

    Returns:
        Angle in degrees

    TODO: Implement this function

    Hint: degrees = radians * 180 / π
    """

    # Your code here
    raise NotImplementedError("TODO: Implement radians_to_degrees")


def degrees_to_radians(degrees: float) -> float:
    """
    Convert degrees to radians.

    Args:
        degrees: Angle in degrees

    Returns:
        Angle in radians

    TODO: Implement this function

    Hint: radians = degrees * π / 180
    """

    # Your code here
    raise NotImplementedError("TODO: Implement degrees_to_radians")


# =============================================================================
# TESTS - Run with: python3 position_conversion.py
# =============================================================================

def run_tests():
    """Run all unit tests."""
    print("Running position conversion tests...\n")

    # Test 1: radians_to_ticks at zero position
    print("Test 1: radians_to_ticks(0, 2048) should equal 2048")
    result = radians_to_ticks(0.0, 2048)
    assert result == 2048, f"Expected 2048, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 2: radians_to_ticks at π radians (180°)
    print("Test 2: radians_to_ticks(π, 2048) should equal 4096")
    result = radians_to_ticks(math.pi, 2048)
    assert result == 4096, f"Expected 4096, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 3: radians_to_ticks at -π/2 radians (-90°)
    print("Test 3: radians_to_ticks(-π/2, 2048) should equal 1024")
    result = radians_to_ticks(-math.pi / 2, 2048)
    assert result == 1024, f"Expected 1024, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 4: ticks_to_radians at zero position
    print("Test 4: ticks_to_radians(2048, 2048) should equal 0.0")
    result = ticks_to_radians(2048, 2048)
    assert abs(result - 0.0) < 0.001, f"Expected 0.0, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 5: ticks_to_radians at 4096 ticks
    print("Test 5: ticks_to_radians(4096, 2048) should equal π")
    result = ticks_to_radians(4096, 2048)
    assert abs(result - math.pi) < 0.001, f"Expected π, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 6: Round-trip conversion
    print("Test 6: Round-trip - ticks_to_radians(radians_to_ticks(1.0, 2048), 2048) should equal 1.0")
    ticks = radians_to_ticks(1.0, 2048)
    result = ticks_to_radians(ticks, 2048)
    assert abs(result - 1.0) < 0.01, f"Expected 1.0, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 7: clamp_ticks
    print("Test 7: clamp_ticks(5000, 1000, 3000) should equal 3000")
    result = clamp_ticks(5000, 1000, 3000)
    assert result == 3000, f"Expected 3000, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 8: clamp_ticks (no clamping needed)
    print("Test 8: clamp_ticks(2000, 1000, 3000) should equal 2000")
    result = clamp_ticks(2000, 1000, 3000)
    assert result == 2000, f"Expected 2000, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 9: radians_to_degrees
    print("Test 9: radians_to_degrees(π) should equal 180.0")
    result = radians_to_degrees(math.pi)
    assert abs(result - 180.0) < 0.001, f"Expected 180.0, got {result}"
    print(f"  ✓ Result: {result}\n")

    # Test 10: degrees_to_radians
    print("Test 10: degrees_to_radians(90) should equal π/2")
    result = degrees_to_radians(90.0)
    assert abs(result - math.pi / 2) < 0.001, f"Expected π/2, got {result}"
    print(f"  ✓ Result: {result}\n")

    print("=" * 50)
    print("All tests passed! Great job!")
    print("=" * 50)


if __name__ == '__main__':
    run_tests()

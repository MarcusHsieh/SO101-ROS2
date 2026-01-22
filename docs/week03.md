# Week 3: Serial Protocol & Position Conversion

## Learning Objectives

By the end of this week, you will be able to:
- Understand the Feetech STS3215 servo memory map
- Convert between radians (ROS2) and ticks (servo native units)
- Explain the role of homing offsets in calibration
- Implement position conversion functions

## Prerequisites

- Completed Weeks 1-2
- Understanding of ROS2 topics and messages

---

## Part 1: Servo Fundamentals

### 1.1 The Feetech STS3215

Our robot uses Feetech STS3215 serial bus servos:

| Specification | Value |
|---------------|-------|
| Resolution | 4096 positions per revolution |
| Angle range | 0-360° (0-4095 ticks) |
| Communication | Half-duplex UART at 1Mbps |
| Protocol | Feetech serial protocol |

### 1.2 Position Encoding

The servo reports and accepts positions as **ticks** (0-4095):

```
          0 ticks
             │
    3072 ────┼──── 1024
             │
         2048 ticks
         (center)
```

- **4096 ticks = 360° = 2π radians**
- **1 tick ≈ 0.088° ≈ 0.00153 radians**

### 1.3 Why Ticks Instead of Degrees?

1. **Integer efficiency** - No floating point on servo microcontroller
2. **Higher precision** - 4096 steps vs 360 degrees
3. **Protocol compatibility** - Standard for servo communication

---

## Part 2: The Memory Map

### 2.1 What is a Memory Map?

Servos have internal registers that store settings and state. You read/write to specific addresses:

| Address | Name | Size | Description |
|---------|------|------|-------------|
| 5 | ID | 1 byte | Motor ID (1-6) |
| 40 | Torque_Enable | 1 byte | 0=off, 1=on |
| 42 | Goal_Position | 2 bytes | Target position |
| 56 | Present_Position | 2 bytes | Current position |

### 2.2 Our Memory Map File

Open `/ros2_ws/src/so101_hw_interface/so101_hw_interface/motors/feetech/tables.py`:

```python
STS_SMS_SERIES_CONTROL_TABLE = {
    "Torque_Enable": (40, 1),      # Address 40, 1 byte
    "Goal_Position": (42, 2),      # Address 42, 2 bytes
    "Present_Position": (56, 2),   # Address 56, 2 bytes
    # ... more entries
}
```

The format is `"Name": (address, size_in_bytes)`.

### 2.3 Reading and Writing

The motor driver abstracts this for us:

```python
# Read current positions (returns ticks)
positions = bus.sync_read("Present_Position", normalize=False)
# {"1": 2048, "2": 2100, "3": 1950, ...}

# Write goal positions (ticks)
bus.sync_write("Goal_Position", {"1": 2048, "2": 2100}, normalize=False)
```

---

## Part 3: Position Conversion

### 3.1 The Problem

- **ROS2 uses radians** (standard for robotics)
- **Servos use ticks** (0-4095)
- We need to convert between them!

### 3.2 The Math

**Ticks to Radians:**
```
radians = (ticks - homing_offset) × (2π / 4096)
```

**Radians to Ticks:**
```
ticks = homing_offset + (radians × 4096 / 2π)
```

### 3.3 What is Homing Offset?

The **homing offset** is the tick value when the joint is at 0 radians (home position).

Example for Joint 1:
- Physical home position: arm pointing forward
- Servo reads: 2091 ticks at this position
- So `homing_offset = 2091`

When the servo reads 2091 ticks → we report 0 radians
When the servo reads 3115 ticks → we report ~0.785 radians (45°)

### 3.4 Calibration File

Open `/ros2_ws/src/so101_hw_interface/config/so101_calibration.yaml`:

```yaml
"1":
  homing_offset: 2091
  range_min: 500
  range_max: 3500
"2":
  homing_offset: 2029
  range_min: 500
  range_max: 3500
# ...
```

---

## Part 4: Implement Conversion Functions

### 4.1 Open the Skeleton

Open `/ros2_ws/src/so101_hw_interface/labs/lab03_position_conversion.py`

### 4.2 Implement radians_to_ticks

```python
def radians_to_ticks(radians: float, homing_offset: int) -> int:
    """
    Convert radians to servo ticks.

    ticks = homing_offset + (radians × 4096 / 2π)
    """
    ticks_per_radian = 4096 / (2 * math.pi)
    ticks = homing_offset + (radians * ticks_per_radian)
    return int(round(ticks))
```

### 4.3 Implement ticks_to_radians

```python
def ticks_to_radians(ticks: int, homing_offset: int) -> float:
    """
    Convert servo ticks to radians.

    radians = (ticks - homing_offset) × (2π / 4096)
    """
    radians_per_tick = (2 * math.pi) / 4096
    radians = (ticks - homing_offset) * radians_per_tick
    return radians
```

### 4.4 Implement clamp_ticks

```python
def clamp_ticks(ticks: int, min_ticks: int, max_ticks: int) -> int:
    """
    Clamp ticks to safe range.
    """
    return max(min_ticks, min(ticks, max_ticks))
```

### 4.5 Run Tests

```bash
cd /ros2_ws/src/so101_hw_interface/labs
python3 lab03_position_conversion.py
```

Expected output:
```
Running position conversion tests...

Test 1: radians_to_ticks(0, 2048) should equal 2048
  ✓ Result: 2048

Test 2: radians_to_ticks(π, 2048) should equal 4096
  ✓ Result: 4096
...
All tests passed! Great job!
```

---

## Part 5: Trace Through the System

### 5.1 Reading Position (Servo → ROS2)

```
Servo reports: 2600 ticks
                  ↓
motor_bridge reads raw value
                  ↓
Subtract homing_offset (2091): 2600 - 2091 = 509
                  ↓
Convert to radians: 509 × (2π/4096) = 0.78 rad
                  ↓
Publish on /joint_states: position = [0.78, ...]
```

### 5.2 Writing Position (ROS2 → Servo)

```
GUI requests: 0.5 radians
                  ↓
Publish on /joint_commands
                  ↓
motor_bridge receives command
                  ↓
Convert to ticks: 0.5 × (4096/2π) = 326
                  ↓
Add homing_offset: 326 + 2091 = 2417
                  ↓
Write 2417 to servo Goal_Position
```

---

## Lab Exercise

### Task 1: Verify Your Understanding

1. If `homing_offset = 2048` and the servo reads `3072` ticks, what angle in degrees is that?

2. If you want to move a joint to +90° and `homing_offset = 2048`, what tick value should you send?

3. Why do we need separate homing offsets for each joint?

### Task 2: Complete the Skeleton

Implement all functions in `lab03_position_conversion.py`:
- `radians_to_ticks()`
- `ticks_to_radians()`
- `clamp_ticks()`
- `radians_to_degrees()`
- `degrees_to_radians()`

### Task 3: Edge Cases

What happens with your functions when:
- Radians is greater than 2π?
- Ticks is negative?
- Homing offset is 0?

Add test cases for these scenarios.

### Deliverables

- [ ] All tests passing in `lab03_position_conversion.py`
- [ ] Written answers to Task 1 questions
- [ ] Additional test cases for edge scenarios

---

## Key Formulas

| Conversion | Formula |
|------------|---------|
| Ticks → Radians | `(ticks - offset) × (2π / 4096)` |
| Radians → Ticks | `offset + (radians × 4096 / 2π)` |
| Radians → Degrees | `radians × 180 / π` |
| Degrees → Radians | `degrees × π / 180` |

## Constants

```python
TICKS_PER_REVOLUTION = 4096
RADIANS_PER_REVOLUTION = 2 * math.pi  # ≈ 6.283
TICKS_PER_RADIAN = 4096 / (2 * math.pi)  # ≈ 651.9
```

---

## What's Next?

In Week 4, you'll learn:
- How to build a complete motor bridge node
- Publisher and subscriber integration
- Timer callbacks for periodic updates
- Service implementation for torque control

---

## Reference

### Feetech Protocol Basics

```
Packet structure:
[0xFF][0xFF][ID][Length][Instruction][Param1][Param2]...[Checksum]

Instructions:
- 0x01: Ping
- 0x02: Read
- 0x03: Write
- 0x83: Sync Write (multiple motors)
```

### Useful Addresses

| Address | Name | R/W | Size |
|---------|------|-----|------|
| 5 | ID | RW | 1 |
| 40 | Torque_Enable | RW | 1 |
| 42 | Goal_Position | RW | 2 |
| 56 | Present_Position | R | 2 |
| 58 | Present_Velocity | R | 2 |
| 63 | Present_Temperature | R | 1 |

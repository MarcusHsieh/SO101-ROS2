# Week 6: Calibration & Safety

## Learning Objectives

By the end of this week, you will be able to:
- Explain why robot calibration is necessary
- Capture and save calibration data
- Implement joint limit safety checks
- Work with YAML configuration files

## Prerequisites

- Completed Weeks 3-5
- Understanding of position conversion

---

## Part 1: Why Calibration?

### 1.1 The Problem

When you assemble a robot:
- Servos don't know their absolute position
- "Zero" on one robot ≠ "Zero" on another
- Joint limits vary based on assembly

### 1.2 What is Calibration?

Calibration establishes:
1. **Homing offset** - Tick value at "zero" position
2. **Joint limits** - Safe min/max positions
3. **Direction** - Which way is "positive"

### 1.3 Without Calibration

```
You command: 0 radians (home position)
Robot thinks: tick 2048 is home
Actually: arm is tilted 15° because servo was mounted differently
Result: Wrong position!
```

### 1.4 With Calibration

```
You command: 0 radians
Calibration says: home = tick 2091 for this specific robot
Robot goes to: tick 2091
Result: Correct position!
```

---

## Part 2: The Calibration File

### 2.1 File Format

`/ros2_ws/src/so101_hw_interface/config/so101_calibration.yaml`:

```yaml
"1":
  homing_offset: 2091   # Tick value at 0 radians
  range_min: 500        # Minimum safe tick value
  range_max: 3500       # Maximum safe tick value
"2":
  homing_offset: 2029
  range_min: 1024
  range_max: 3072
# ... joints 3-6
```

### 2.2 What Each Field Means

| Field | Description |
|-------|-------------|
| `homing_offset` | Tick reading when joint is at 0 radians |
| `range_min` | Lowest tick value before hitting physical limit |
| `range_max` | Highest tick value before hitting physical limit |

### 2.3 How It's Used

In motor_bridge:
```python
# Reading: convert ticks to radians
radians = (ticks - homing_offset) * (2π / 4096)

# Writing: convert radians to ticks, then clamp
ticks = homing_offset + (radians * 4096 / 2π)
ticks = clamp(ticks, range_min, range_max)
```

---

## Part 3: Calibration Procedure

### 3.1 Overview

1. Disable torque (so you can move arm freely)
2. Physically position arm at "home" pose
3. Read current tick values
4. Save as new homing offsets

### 3.2 Define "Home" Position

Our home position:
- Base: pointing forward (0°)
- Shoulder: horizontal (0°)
- Elbow: straight (0°)
- Wrist pitch: level (0°)
- Wrist roll: centered (0°)
- Gripper: open

### 3.3 Step-by-Step Calibration

```bash
# 1. Start motor_bridge
ros2 run so101_hw_interface so101_motor_bridge

# 2. Disable torque via service
ros2 service call /so101_follower/set_torque std_srvs/srv/SetBool "{data: false}"

# 3. Physically move arm to home position

# 4. Read current positions
ros2 topic echo /so101_follower/joint_states --once
# Note: These are in RADIANS (already converted from ticks)

# 5. To get raw ticks, use the calibration capture tool
python3 lab06_calibration.py --capture-home
```

### 3.4 Capturing Joint Limits

For each joint:
1. Move to physical minimum (carefully!)
2. Record tick value + safety margin
3. Move to physical maximum
4. Record tick value - safety margin

**Safety margin:** 50-100 ticks to avoid hitting hard stops.

---

## Part 4: Implement Calibration Functions

### 4.1 Open the Skeleton

Open `/ros2_ws/src/so101_hw_interface/labs/lab06_calibration.py`

### 4.2 Implement load_calibration

```python
def load_calibration(filepath: str) -> dict:
    path = pathlib.Path(filepath)

    if not path.is_file():
        print(f"Warning: {filepath} not found, using defaults")
        return DEFAULT_CALIBRATION.copy()

    try:
        with open(path, 'r') as f:
            calibration = yaml.safe_load(f)

        # Validate structure
        for joint in ['1', '2', '3', '4', '5', '6']:
            if joint not in calibration:
                print(f"Warning: Joint {joint} missing, using default")
                calibration[joint] = DEFAULT_CALIBRATION[joint]

        return calibration

    except Exception as e:
        print(f"Error loading calibration: {e}")
        return DEFAULT_CALIBRATION.copy()
```

### 4.3 Implement save_calibration

```python
def save_calibration(calibration: dict, filepath: str) -> bool:
    try:
        path = pathlib.Path(filepath)
        path.parent.mkdir(parents=True, exist_ok=True)

        with open(path, 'w') as f:
            yaml.dump(calibration, f, default_flow_style=False, sort_keys=False)

        print(f"Calibration saved to {filepath}")
        return True

    except Exception as e:
        print(f"Error saving calibration: {e}")
        return False
```

### 4.4 Implement capture_home_position

```python
def capture_home_position(current_ticks: dict, calibration: dict) -> dict:
    # Make a copy to avoid modifying original
    new_calibration = {}
    for joint, data in calibration.items():
        new_calibration[joint] = data.copy()

    # Update homing offsets with current tick values
    for joint, ticks in current_ticks.items():
        if joint in new_calibration:
            new_calibration[joint]['homing_offset'] = ticks
            print(f"Joint {joint}: homing_offset = {ticks}")

    return new_calibration
```

### 4.5 Run Tests

```bash
cd /ros2_ws/src/so101_hw_interface/labs
python3 lab06_calibration.py
```

---

## Part 5: Safety Systems

### 5.1 Software Limits

Always clamp commanded positions:

```python
def safe_command(joint, ticks, calibration):
    limits = calibration[joint]
    min_ticks = limits['range_min']
    max_ticks = limits['range_max']

    if ticks < min_ticks:
        print(f"Warning: Joint {joint} clamped to min")
        return min_ticks
    if ticks > max_ticks:
        print(f"Warning: Joint {joint} clamped to max")
        return max_ticks

    return ticks
```

### 5.2 Torque Control

Torque OFF allows:
- Manual positioning for calibration
- Moving arm away if stuck
- Teaching demonstrations

**Important:** Re-enable torque before autonomous operation!

### 5.3 Emergency Stop

The GUI has STOP button that:
1. Immediately stops all movement
2. Can disable torque
3. Logs the event

---

## Lab Exercise

### Task 1: Complete the Skeleton

Implement all functions in `lab06_calibration.py`:
- [ ] `load_calibration`
- [ ] `save_calibration`
- [ ] `capture_home_position`
- [ ] `capture_joint_limit`
- [ ] `validate_calibration`
- [ ] `format_calibration_report`

### Task 2: Calibrate Your Robot

Perform a full calibration:
1. Capture home position
2. Capture min/max limits for at least 2 joints
3. Save to a new calibration file
4. Verify by commanding home position

### Task 3: Validation

Implement calibration validation that checks:
- All joints present
- homing_offset within range_min and range_max
- range_min < range_max
- Values are reasonable (0-4095)

### Deliverables

- [ ] Completed `lab06_calibration.py`
- [ ] Your robot's calibration file
- [ ] Calibration report showing all values
- [ ] Screenshot of robot at calibrated home position

---

## Common Mistakes

1. **Calibrating with torque ON** - Arm fights you, readings wrong
2. **Forgetting safety margin** - Robot hits hard stops
3. **Not backing up old calibration** - Lose previous values
4. **Modifying original dict** - Use `.copy()`

---

## What's Next?

In Week 7, you'll learn:
- Arduino programming basics
- Button debouncing
- UART communication
- Connecting hardware to ROS2

---

## Reference

### YAML in Python

```python
import yaml

# Load YAML file
with open('config.yaml', 'r') as f:
    data = yaml.safe_load(f)

# Save to YAML file
with open('config.yaml', 'w') as f:
    yaml.dump(data, f, default_flow_style=False)
```

### Calibration Commands

```bash
# Disable torque for manual positioning
ros2 service call /so101_follower/set_torque std_srvs/srv/SetBool "{data: false}"

# Enable torque
ros2 service call /so101_follower/set_torque std_srvs/srv/SetBool "{data: true}"

# Read current joint states
ros2 topic echo /so101_follower/joint_states --once
```

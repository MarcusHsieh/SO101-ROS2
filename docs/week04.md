# Week 4: Motor Bridge Implementation

## Learning Objectives

By the end of this week, you will be able to:
- Build a complete ROS2 node with publishers and subscribers
- Implement timer callbacks for periodic operations
- Create a ROS2 service for torque control
- Understand the read/write alternation pattern for serial buses

## Prerequisites

- Completed Week 3 (position conversion functions)
- Understanding of ROS2 nodes, topics, and messages

---

## Part 1: Motor Bridge Architecture

### 1.1 What Does motor_bridge Do?

The motor bridge is the central node that:
1. **Reads** servo positions and publishes to `/joint_states`
2. **Receives** commands from `/joint_commands` and writes to servos
3. **Provides** a torque control service

```
                    ┌─────────────────────────────┐
 /joint_commands ──▶│                             │
                    │       motor_bridge          │◀──▶ Feetech Servos
 /joint_states   ◀──│                             │     (USB Serial)
                    │                             │
 /set_torque     ──▶│  (service)                  │
                    └─────────────────────────────┘
```

### 1.2 The Read/Write Pattern

The serial bus is **half-duplex** - it can only send OR receive at once. Our solution: alternate between reading and writing.

```
Timer fires (50Hz)
        │
        ▼
    Is it read turn?
    /           \
   Yes           No
    │             │
    ▼             ▼
 Read from    Write to
  servos       servos
    │             │
    ▼             ▼
  Publish      (commands
 /joint_states  applied)
    │             │
    └──────┬──────┘
           ▼
    Flip the flag
    (next turn is opposite)
```

This gives us 25Hz effective read rate and 25Hz write rate.

---

## Part 2: Node Structure

### 2.1 Publishers and Subscribers

```python
class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        # Publisher: sends joint states to ROS2
        self.joint_states_pub = self.create_publisher(
            JointState,
            'so101_follower/joint_states',
            10
        )

        # Subscriber: receives commands from GUI/other nodes
        self.joint_commands_sub = self.create_subscription(
            JointState,
            'so101_follower/joint_commands',
            self._command_callback,
            10
        )
```

### 2.2 Services

```python
        # Service: allows torque control
        self.torque_srv = self.create_service(
            SetBool,
            'so101_follower/set_torque',
            self._torque_callback
        )
```

### 2.3 Timers

```python
        # Timer: fires every 20ms (50Hz)
        self.timer = self.create_timer(0.02, self._timer_callback)
```

---

## Part 3: Implement the Skeleton

### 3.1 Open the Skeleton File

Open `/ros2_ws/src/so101_hw_interface/labs/lab04_motor_bridge.py`

### 3.2 Implement the Publisher

Find the TODO for creating the publisher:

```python
# TODO: Create publisher for joint states
self.joint_states_pub = self.create_publisher(
    JointState,
    'so101_follower/joint_states',
    10
)
```

### 3.3 Implement the Subscriber

```python
# TODO: Create subscriber for joint commands
self.joint_commands_sub = self.create_subscription(
    JointState,
    'so101_follower/joint_commands',
    self._command_callback,
    10
)
```

### 3.4 Implement the Command Callback

```python
def _command_callback(self, msg: JointState):
    """Store commanded positions for next write cycle."""
    for name, position in zip(msg.name, msg.position):
        if name in JOINTS:
            self.current_commands[name] = position
```

### 3.5 Implement the Torque Callback

```python
def _torque_callback(self, request, response):
    """Handle torque enable/disable requests."""
    try:
        if request.data:
            self.bus.enable_torque()
            self._torque_enabled = True
            response.success = True
            response.message = "Torque enabled"
        else:
            self.bus.disable_torque()
            self._torque_enabled = False
            response.success = True
            response.message = "Torque disabled"
    except Exception as exc:
        response.success = False
        response.message = f"Failed: {exc}"
    return response
```

### 3.6 Implement _do_read

```python
def _do_read(self):
    """Read positions from servos and publish."""
    try:
        # Read raw tick values from servos
        raw_positions = self.bus.sync_read("Present_Position", normalize=False)

        # Create JointState message
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JOINTS.keys())

        # Convert ticks to radians for each joint
        js.position = []
        for joint_name in JOINTS.keys():
            ticks = raw_positions.get(joint_name, 0)
            offset = self._home_offsets.get(joint_name, 2048)
            radians = (ticks - offset) * (2 * math.pi) / 4096.0
            js.position.append(radians)

        # Publish
        self.joint_states_pub.publish(js)

    except Exception as exc:
        self.get_logger().warn(f"Read failed: {exc}")
```

### 3.7 Implement _do_write

```python
def _do_write(self):
    """Write commanded positions to servos."""
    # Skip if torque disabled or no commands
    if not self._torque_enabled:
        return
    if not self.current_commands:
        return

    try:
        raw_goals = {}
        for joint_name, radians in self.current_commands.items():
            offset = self._home_offsets.get(joint_name, 2048)
            ticks = int(offset + radians * self._ticks_per_radian)

            # Apply safety limits if available
            if self._limits and joint_name in self._limits:
                min_ticks, max_ticks = self._limits[joint_name]
                ticks = max(min_ticks, min(ticks, max_ticks))

            raw_goals[joint_name] = ticks

        self.bus.sync_write("Goal_Position", raw_goals, normalize=False)

    except Exception as exc:
        self.get_logger().warn(f"Write failed: {exc}")
```

---

## Part 4: Understanding the Flow

### 4.1 Startup Sequence

1. Node initializes, connects to servo bus
2. Enables torque on all servos
3. Loads calibration (homing offsets, limits)
4. Starts 50Hz timer

### 4.2 Runtime Loop

Every 20ms:
1. Timer fires
2. If read turn: read positions, convert, publish
3. If write turn: convert commands, write to servos
4. Flip the turn flag

### 4.3 Command Flow

1. User moves slider in GUI
2. GUI publishes to `/joint_commands`
3. `_command_callback` stores the position
4. Next write cycle sends it to servo

---

## Lab Exercise

### Task 1: Complete the Skeleton

Implement all TODO sections in `lab04_motor_bridge.py`:
- [ ] Create publisher
- [ ] Create subscriber
- [ ] Create service
- [ ] Implement `_command_callback`
- [ ] Implement `_torque_callback`
- [ ] Implement `_do_read`
- [ ] Implement `_do_write`

### Task 2: Add Logging

Add logging statements to track:
- When commands are received
- When positions are published
- When torque changes state

Example:
```python
self.get_logger().info(f"Received command for joint {name}: {position:.3f} rad")
```

### Task 3: Error Handling

What happens if:
- The serial connection is lost?
- An invalid joint name is received?
- The servo returns an unexpected value?

Add appropriate error handling for each case.

### Deliverables

- [ ] Completed `lab04_motor_bridge.py`
- [ ] Screenshot showing your node running alongside the working system
- [ ] Written explanation of the read/write alternation pattern

---

## Testing Your Implementation

### Option 1: Side-by-side Test

Run your lab implementation alongside the working motor_bridge:

```bash
# Terminal 1: Start the working system
./start_all.sh

# Terminal 2: Run your lab (different node name)
python3 lab04_motor_bridge.py
```

Your lab will publish to the same topics - watch with:
```bash
ros2 topic echo /so101_follower/joint_states
```

### Option 2: Run Your Code on the Real Robot

Use the `--lab` flag to run your implementation instead of the working version:

```bash
# This runs YOUR lab04_motor_bridge.py on the actual robot!
./start_all.sh --lab motor_bridge
```

Now when you move sliders in the GUI, YOUR code controls the robot. Check the log for errors:

```bash
docker exec so101_ros2 tail -f /tmp/motor_bridge.log
```

If something breaks, just restart without the flag to use the working code:

```bash
./start_all.sh  # Back to working version
```

---

## Common Mistakes

1. **Forgetting to convert units** - Always convert between ticks and radians
2. **Not handling empty commands** - Check if `current_commands` has data
3. **Blocking the executor** - Don't use `time.sleep()` in callbacks
4. **Missing error handling** - Serial communication can fail

---

## What's Next?

In Week 5, you'll learn:
- How to build HTTP APIs in Python
- JSON request/response handling
- The joint slider GUI architecture

---

## Reference

### ROS2 Node Methods

```python
# Create publisher
self.create_publisher(MessageType, 'topic_name', queue_size)

# Create subscriber
self.create_subscription(MessageType, 'topic', callback, queue)

# Create service
self.create_service(ServiceType, 'service_name', callback)

# Create timer
self.create_timer(period_seconds, callback)

# Get current time
self.get_clock().now().to_msg()

# Log messages
self.get_logger().info("message")
self.get_logger().warn("warning")
self.get_logger().error("error")
```

### Service Callback Pattern

```python
def _my_service_callback(self, request, response):
    # Process request
    # Fill in response
    response.success = True
    response.message = "Done"
    return response  # Must return response!
```

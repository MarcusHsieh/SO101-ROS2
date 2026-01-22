# Week 2: ROS2 Fundamentals & Visualization

## Learning Objectives

By the end of this week, you will be able to:
- Explain the ROS2 node/topic/message architecture
- Create a subscriber node that receives joint states
- Use Foxglove to visualize robot data
- Understand the `/joint_states` message format

## Prerequisites

- Completed Week 1 setup
- Robot connected and working

---

## Part 1: ROS2 Concepts

### 1.1 What is ROS2?

ROS2 (Robot Operating System 2) is a framework for building robot software. Key concepts:

- **Nodes**: Independent programs that do one thing well
- **Topics**: Named channels for data flow
- **Messages**: Structured data sent over topics
- **Services**: Request/response interactions
- **Publishers**: Nodes that send data to topics
- **Subscribers**: Nodes that receive data from topics

### 1.2 Our System Architecture

```
┌─────────────────┐         ┌─────────────────┐
│  motor_bridge   │────────▶│    /joint_states │
│     (Node)      │ publish │     (Topic)      │
└─────────────────┘         └────────┬─────────┘
                                     │ subscribe
                                     ▼
                            ┌─────────────────┐
                            │  Your Node!     │
                            │  (Subscriber)   │
                            └─────────────────┘
```

### 1.3 The JointState Message

The robot publishes joint positions using `sensor_msgs/JointState`:

```python
# sensor_msgs/msg/JointState
header:
  stamp: {sec: 1234567890, nanosec: 123456789}
  frame_id: ''
name: ['1', '2', '3', '4', '5', '6']
position: [0.0, 0.1, -0.2, 0.3, 0.0, 0.5]  # radians
velocity: []
effort: []
```

---

## Part 2: Explore the Running System

### 2.1 Start the Robot

```bash
cd /home/mj/SO101-ROS2/lerobot_ws
./start_all.sh
```

### 2.2 List Running Nodes

```bash
ros2 node list
```

Expected output:
```
/foxglove_bridge
/joint_slider_gui
/robot_state_publisher
/so101_motor_bridge
```

### 2.3 List Topics

```bash
ros2 topic list
```

Expected output:
```
/so101_follower/joint_states
/so101_follower/joint_commands
/tf
/tf_static
...
```

### 2.4 Watch Joint States

```bash
ros2 topic echo /so101_follower/joint_states
```

Move the robot (via GUI or manually with torque off) and watch the values change!

Press `Ctrl+C` to stop.

### 2.5 Get Topic Info

```bash
ros2 topic info /so101_follower/joint_states
```

Shows the message type and who's publishing/subscribing.

---

## Part 3: Your First Subscriber

### 3.1 Open the Skeleton File

Open `/ros2_ws/src/so101_hw_interface/labs/lab02_ros2_basics.py` in VS Code.

### 3.2 Understand the Structure

```python
class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')

        # TODO: Create subscriber here
        self.subscription = None

    def joint_state_callback(self, msg: JointState):
        # TODO: Process message here
        pass
```

### 3.3 Implement the Subscriber

**Step 1:** Create the subscription in `__init__`:

```python
self.subscription = self.create_subscription(
    JointState,                           # Message type
    '/so101_follower/joint_states',       # Topic name
    self.joint_state_callback,            # Callback function
    10                                    # Queue size
)
```

**Step 2:** Implement the callback:

```python
def joint_state_callback(self, msg: JointState):
    print("--- Joint States ---")
    for name, position in zip(msg.name, msg.position):
        degrees = math.degrees(position)
        label = joint_labels.get(name, name)
        print(f"Joint {name} ({label}): {degrees:6.1f}°")
    print("")
```

### 3.4 Test Your Implementation

```bash
cd /ros2_ws/src/so101_hw_interface/labs
python3 lab02_ros2_basics.py --test
```

If tests pass, run the actual node:

```bash
# In one terminal, make sure robot is running
# In another terminal:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
python3 lab02_ros2_basics.py
```

You should see joint positions printed whenever the robot moves!

---

## Part 4: Foxglove Visualization

### 4.1 Connect to Foxglove

1. Open https://app.foxglove.dev
2. Connect to `ws://localhost:8765`

### 4.2 Add Panels

**3D Panel:**
1. Click "Add panel" → "3D"
2. In the panel settings, enable `/tf`
3. You should see the robot model

**Plot Panel:**
1. Click "Add panel" → "Plot"
2. Add a series: `/so101_follower/joint_states.position[0]`
3. Move joint 1 and watch the plot update

**Raw Messages Panel:**
1. Click "Add panel" → "Raw Messages"
2. Select `/so101_follower/joint_states`
3. See the actual message data

### 4.3 Latency Fix

If visualization seems delayed:
1. Open panel settings (gear icon)
2. Change "Timestamp method" to "Receive time"

---

## Lab Exercise

### Task: Enhanced Joint Monitor

Extend your subscriber to:
1. Track the maximum and minimum position seen for each joint
2. Print a warning if any joint moves more than 0.5 radians in one update
3. Count total messages received

### Skeleton Enhancement

Add these to your node:

```python
def __init__(self):
    # ... existing code ...
    self.message_count = 0
    self.max_positions = {j: float('-inf') for j in ['1','2','3','4','5','6']}
    self.min_positions = {j: float('inf') for j in ['1','2','3','4','5','6']}
    self.last_positions = {}

def joint_state_callback(self, msg):
    self.message_count += 1

    # TODO: Track min/max for each joint
    # TODO: Detect large movements
    # TODO: Print nicely formatted output
```

### Deliverables

- [ ] Completed `lab02_ros2_basics.py` with passing tests
- [ ] Screenshot of your node running and printing joint states
- [ ] Screenshot of Foxglove with 3D and Plot panels
- [ ] Enhanced version with min/max tracking

---

## Key Concepts Summary

| Concept | Description |
|---------|-------------|
| Node | A single-purpose program in ROS2 |
| Topic | Named channel for publishing/subscribing |
| Publisher | Sends messages to a topic |
| Subscriber | Receives messages from a topic |
| Callback | Function called when message arrives |
| JointState | Standard message for robot joint data |

## Common Mistakes

1. **Forgetting to source setup.bash** - Always run `source /ros2_ws/install/setup.bash`
2. **Wrong topic name** - Use `ros2 topic list` to verify exact name
3. **Message type mismatch** - Check with `ros2 topic info <topic>`

---

## What's Next?

In Week 3, you'll learn:
- How the Feetech serial protocol works
- Position conversion between radians and servo ticks
- The role of calibration offsets

---

## Reference

### Useful Commands

```bash
# List all nodes
ros2 node list

# Get info about a node
ros2 node info /so101_motor_bridge

# List all topics
ros2 topic list

# Get info about a topic
ros2 topic info /so101_follower/joint_states

# Echo topic messages
ros2 topic echo /so101_follower/joint_states

# Check message type definition
ros2 interface show sensor_msgs/msg/JointState
```

### Message Types

```bash
# See what fields a message has
ros2 interface show sensor_msgs/msg/JointState

# Output:
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

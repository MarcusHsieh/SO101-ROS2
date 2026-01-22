# Week 9: Data Collection for Imitation Learning

## Learning Objectives

By the end of this week, you will be able to:
- Explain imitation learning fundamentals
- Set up leader-follower teleoperation
- Record high-quality demonstration data
- Understand the LeRobot dataset format

## Prerequisites

- Completed Weeks 1-8
- Access to both leader and follower arms

---

## Part 1: Imitation Learning Basics

### 1.1 What is Imitation Learning?

Instead of programming every movement, we:
1. **Demonstrate** the task by moving the robot
2. **Record** observations and actions
3. **Train** a neural network to mimic the demonstration
4. **Deploy** the trained model to do the task autonomously

### 1.2 Why Imitation Learning?

| Approach | Pros | Cons |
|----------|------|------|
| Manual Programming | Precise, predictable | Tedious, brittle |
| Reinforcement Learning | Learns from scratch | Needs reward function, slow |
| **Imitation Learning** | Natural teaching, fast | Needs good demos |

### 1.3 Key Concepts

**Observation (State):** What the robot perceives
- Joint positions
- Camera images (optional)
- Sensor readings

**Action:** What the robot does
- Joint commands
- Gripper position
- Velocity targets

**Episode:** One complete demonstration
- Pick up cube, place it → one episode
- Typically 3-10 seconds
- We need 50-100 episodes for training

---

## Part 2: Leader-Follower Setup

### 2.1 Hardware Configuration

```
┌─────────────────┐                ┌─────────────────┐
│     LEADER      │                │    FOLLOWER     │
│                 │                │                 │
│   (Torque OFF)  │  ──────────▶  │   (Torque ON)   │
│                 │  joint_states  │                 │
│   Human moves   │                │    Mirrors      │
│   this arm      │                │    leader       │
└─────────────────┘                └─────────────────┘
        │                                  │
        │                                  │
        ▼                                  ▼
   /leader/joint_states           /follower/joint_states
              │                           │
              └───────────┬───────────────┘
                          ▼
                   Data Recorder
                   (Records both)
```

### 2.2 Topic Mapping

| Arm | Topic | Purpose |
|-----|-------|---------|
| Leader | `/leader/joint_states` | Human demonstration input |
| Follower | `/so101_follower/joint_states` | Robot execution |
| Follower | `/so101_follower/joint_commands` | Mirror commands |

### 2.3 Starting the System

```bash
# Terminal 1: Start follower arm (our existing setup)
cd /home/mj/SO101-ROS2/lerobot_ws
./start_all.sh

# Terminal 2: Start leader arm driver
ros2 run so101_hw_interface so101_motor_bridge \
    --ros-args \
    -p port:=/dev/ttyUSB1 \
    -r so101_follower/joint_states:=/leader/joint_states \
    -r so101_follower/joint_commands:=/leader/joint_commands

# Terminal 3: Disable leader torque
ros2 service call /leader/set_torque std_srvs/srv/SetBool "{data: false}"

# Terminal 4: Start mirroring (leader → follower)
ros2 run so101_hw_interface leader_follower_bridge
```

---

## Part 3: Recording Episodes

### 3.1 What We Record

For each timestep (50Hz):
- **Observation:** Leader arm joint positions (what human did)
- **Action:** Follower arm commands (what robot should do)
- **Timestamp:** When this happened

### 3.2 Episode Format

```python
episode = {
    "observation.state": np.array([
        [0.0, 0.1, 0.2, 0.3, 0.0, 0.5],   # t=0
        [0.01, 0.11, 0.21, 0.31, 0.01, 0.51],  # t=0.02
        # ... more timesteps
    ]),  # Shape: (T, 6) where T = timesteps
    "action": np.array([
        [0.0, 0.1, 0.2, 0.3, 0.0, 0.5],
        [0.01, 0.11, 0.21, 0.31, 0.01, 0.51],
        # ... matches observation
    ]),  # Shape: (T, 6)
    "timestamp": np.array([0.0, 0.02, 0.04, ...]),  # Shape: (T,)
    "episode_index": 0,
}
```

### 3.3 Recording Workflow

```
1. Position task objects (cube at location A)
2. Press 'r' to start recording
3. Perform demonstration:
   - Move leader arm
   - Follower mirrors
   - Complete the task
4. Press 's' to stop and save
5. Reset objects
6. Repeat 50+ times
```

---

## Part 4: Implement Data Recorder

### 4.1 Open the Skeleton

Open `/ros2_ws/src/so101_hw_interface/labs/lab09_data_recorder.py`

### 4.2 Implement Callbacks

```python
def _leader_callback(self, msg: JointState):
    """Store latest leader arm position."""
    self.latest_leader_state = np.array(msg.position)

def _follower_callback(self, msg: JointState):
    """Store latest follower arm position."""
    self.latest_follower_state = np.array(msg.position)
```

### 4.3 Implement Recording Control

```python
def start_recording(self) -> bool:
    if self.is_recording:
        self.get_logger().warn("Already recording!")
        return False

    self.episode_buffer.reset()
    self.is_recording = True
    self.get_logger().info("Recording started")
    return True

def stop_recording(self) -> Optional[str]:
    if not self.is_recording:
        return None

    self.is_recording = False

    if self.episode_buffer.get_length() == 0:
        self.get_logger().warn("No data recorded")
        return None

    # Save episode
    episode_data = self.episode_buffer.to_dict()
    filepath = self.save_episode(episode_data)

    self.episode_count += 1
    self.episode_buffer.reset()

    return filepath
```

### 4.4 Implement Saving

```python
def save_episode(self, episode_data: dict) -> str:
    episode_data['episode_index'] = self.episode_count

    filename = f"episode_{self.episode_count:04d}.npz"
    filepath = self.output_dir / filename

    np.savez(filepath, **episode_data)

    length = len(episode_data['timestamp'])
    duration = episode_data['timestamp'][-1]
    self.get_logger().info(
        f"Saved {filename}: {length} frames, {duration:.1f}s"
    )

    return str(filepath)
```

---

## Part 5: Data Quality

### 5.1 Good vs Bad Demonstrations

**Good Demonstrations:**
- Smooth, deliberate movements
- Consistent task execution
- Complete task successfully
- Similar start/end positions

**Bad Demonstrations:**
- Jerky, hesitant movements
- Inconsistent approach
- Failed task attempts
- Random exploration

### 5.2 Quality Checklist

Before each recording:
- [ ] Objects in consistent starting position
- [ ] Leader arm at consistent start pose
- [ ] No obstructions in workspace

During recording:
- [ ] Smooth, confident movements
- [ ] Complete the task fully
- [ ] Don't pause or restart

After recording:
- [ ] Verify episode saved
- [ ] Check file size is reasonable
- [ ] Reset for next episode

### 5.3 Visualizing Data

```python
import numpy as np
import matplotlib.pyplot as plt

# Load episode
data = np.load("episode_0000.npz")

# Plot joint trajectories
plt.figure(figsize=(12, 6))
for i in range(6):
    plt.subplot(2, 3, i+1)
    plt.plot(data['timestamp'], data['observation.state'][:, i])
    plt.title(f"Joint {i+1}")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (rad)")
plt.tight_layout()
plt.savefig("episode_visualization.png")
```

---

## Lab Exercise

### Task 1: Set Up Leader-Follower

1. Connect both arms
2. Configure topic remapping
3. Test mirroring works
4. Verify torque states

### Task 2: Complete Data Recorder

Implement in `data_recorder.py`:
- [ ] `_leader_callback`
- [ ] `_follower_callback`
- [ ] `start_recording`
- [ ] `stop_recording`
- [ ] `save_episode`
- [ ] `load_episode`
- [ ] `get_dataset_info`

### Task 3: Record Training Data

Task: Pick and Place
- Pick up cube from position A
- Place at position B
- Record 20 episodes (minimum)

### Task 4: Validate Dataset

1. Count total episodes
2. Visualize 3 random episodes
3. Check for consistency
4. Remove bad episodes if needed

### Deliverables

- [ ] Working leader-follower setup
- [ ] Completed data_recorder.py
- [ ] 20+ recorded episodes
- [ ] Visualization of sample episodes
- [ ] Dataset info report

---

## The Pick-and-Place Task

### Setup

```
Workspace Layout:

    ┌─────────────────────────────┐
    │                             │
    │    [A]           [B]        │
    │    Cube          Target     │
    │    Start         Location   │
    │                             │
    │         [Robot]             │
    │           ▲                 │
    └───────────┼─────────────────┘
                │
              Base
```

### Demonstration Steps

1. **Approach:** Move gripper above cube at A
2. **Descend:** Lower gripper to cube
3. **Grasp:** Close gripper
4. **Lift:** Raise cube
5. **Transport:** Move to position B
6. **Lower:** Descend to placement height
7. **Release:** Open gripper
8. **Retract:** Move away from cube

### Tips for Good Demos

- Keep movements smooth and consistent
- Use similar approach angles each time
- Pause briefly when grasping (helps model learn)
- Return to consistent end position

---

## What's Next?

In Week 10, you'll learn:
- Upload dataset to HuggingFace
- Train models in Google Colab
- Deploy and evaluate on physical robot

---

## Reference

### Episode File Structure

```python
# Loading an episode
data = np.load("episode_0000.npz")

# Available arrays
data['observation.state']  # (T, 6) leader positions
data['action']            # (T, 6) follower commands
data['timestamp']         # (T,) time values
data['frame_index']       # (T,) frame numbers
data['episode_index']     # scalar, episode number
```

### Dataset Statistics

```python
def compute_dataset_stats(data_dir):
    episodes = list(Path(data_dir).glob("episode_*.npz"))

    total_frames = 0
    total_duration = 0

    for ep_file in episodes:
        data = np.load(ep_file)
        total_frames += len(data['timestamp'])
        total_duration += data['timestamp'][-1]

    print(f"Episodes: {len(episodes)}")
    print(f"Total frames: {total_frames}")
    print(f"Total duration: {total_duration:.1f}s")
    print(f"Avg episode length: {total_duration/len(episodes):.1f}s")
```

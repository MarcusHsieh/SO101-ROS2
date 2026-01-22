#!/usr/bin/env python3
"""
Week 9: Data Recorder for Imitation Learning

This module records robot demonstrations for training imitation learning
models using the LeRobot framework.

Data Collection Setup (Leader-Follower):
    ┌─────────────┐                    ┌─────────────┐
    │   LEADER    │   joint_states     │  FOLLOWER   │
    │    ARM      │ ─────────────────▶ │    ARM      │
    │  (torque    │                    │  (torque    │
    │    OFF)     │   Human moves      │    ON)      │
    │             │   this arm         │  Mirrors    │
    └──────┬──────┘                    └──────┬──────┘
           │                                  │
           │  /leader/joint_states            │  /follower/joint_states
           │                                  │
           └──────────────┬───────────────────┘
                          ▼
                   ┌─────────────┐
                   │   Record    │
                   │  Episode    │
                   │   Data      │
                   └─────────────┘

Episode Data Format (LeRobot compatible):
    {
        "observation.state": np.array of shape (T, 6),  # Leader positions
        "action": np.array of shape (T, 6),             # Follower commands
        "timestamp": np.array of shape (T,),            # Time values
        "episode_index": int,
        "frame_index": np.array of shape (T,),
    }

Learning Objectives:
- Understand imitation learning data requirements
- Work with HuggingFace datasets format
- Implement real-time data collection
- Handle synchronization between leader and follower
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import json
import os
from datetime import datetime
from typing import Optional, List, Dict
from pathlib import Path


# =============================================================================
# CONFIGURATION
# =============================================================================

# Number of joints
NUM_JOINTS = 6

# Recording frequency (Hz)
RECORD_RATE = 50

# Default output directory
DEFAULT_OUTPUT_DIR = "./recorded_episodes"

# Episode file format
EPISODE_FORMAT = "episode_{:04d}.npz"


# =============================================================================
# DATA STRUCTURES
# =============================================================================

class EpisodeBuffer:
    """
    Buffer to accumulate data during an episode recording.

    Stores observations (leader positions) and actions (follower commands)
    for each timestep.
    """

    def __init__(self):
        """Initialize empty episode buffer."""
        self.observations: List[np.ndarray] = []  # Leader joint positions
        self.actions: List[np.ndarray] = []        # Follower commands
        self.timestamps: List[float] = []          # Recording timestamps
        self.start_time: Optional[float] = None

    def reset(self):
        """Clear the buffer for a new episode."""
        self.observations = []
        self.actions = []
        self.timestamps = []
        self.start_time = None

    def add_frame(self, observation: np.ndarray, action: np.ndarray, timestamp: float):
        """
        Add a frame to the episode buffer.

        Args:
            observation: Leader joint positions (6,)
            action: Follower joint commands (6,)
            timestamp: Current ROS time
        """
        if self.start_time is None:
            self.start_time = timestamp

        self.observations.append(observation.copy())
        self.actions.append(action.copy())
        self.timestamps.append(timestamp - self.start_time)

    def get_length(self) -> int:
        """Return number of frames in the buffer."""
        return len(self.observations)

    def to_dict(self) -> Dict[str, np.ndarray]:
        """
        Convert buffer to dictionary format for saving.

        Returns:
            Dictionary with numpy arrays ready for saving
        """
        return {
            "observation.state": np.array(self.observations),
            "action": np.array(self.actions),
            "timestamp": np.array(self.timestamps),
            "frame_index": np.arange(len(self.observations)),
        }


# =============================================================================
# DATA RECORDER NODE
# =============================================================================

class DataRecorderNode(Node):
    """
    ROS2 node for recording demonstration episodes.
    """

    def __init__(self):
        super().__init__('data_recorder')

        # Parameters
        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
        self.declare_parameter('record_rate', RECORD_RATE)

        self.output_dir = Path(
            self.get_parameter('output_dir').get_parameter_value().string_value
        )
        self.record_rate = self.get_parameter('record_rate').get_parameter_value().integer_value

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # State
        self.episode_buffer = EpisodeBuffer()
        self.is_recording = False
        self.episode_count = self._count_existing_episodes()

        # Latest received data
        self.latest_leader_state: Optional[np.ndarray] = None
        self.latest_follower_state: Optional[np.ndarray] = None

        # -----------------------------------------------------------------
        # TODO: Create subscribers for leader and follower joint states
        # -----------------------------------------------------------------
        # Leader topic: '/leader/joint_states'
        # Follower topic: '/so101_follower/joint_states'
        # Both are JointState messages

        self.leader_sub = None  # TODO: Create subscriber
        self.follower_sub = None  # TODO: Create subscriber

        # Timer for recording (runs at record_rate Hz)
        self.record_timer = self.create_timer(
            1.0 / self.record_rate,
            self._record_timer_callback
        )

        self.get_logger().info(f"Data Recorder initialized")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Existing episodes: {self.episode_count}")

    def _count_existing_episodes(self) -> int:
        """Count existing episode files to continue numbering."""
        count = 0
        for f in self.output_dir.glob("episode_*.npz"):
            count += 1
        return count

    # =========================================================================
    # CALLBACKS - TODO: Implement
    # =========================================================================

    def _leader_callback(self, msg: JointState):
        """
        Callback for leader arm joint states.

        Args:
            msg: JointState message from leader arm

        TODO: Implement this callback

        Steps:
        1. Extract positions from msg.position
        2. Convert to numpy array
        3. Store in self.latest_leader_state

        Hints:
        - msg.position is a tuple/list of floats
        - Use np.array(msg.position) to convert
        """

        # Your code here
        pass

    def _follower_callback(self, msg: JointState):
        """
        Callback for follower arm joint states.

        Args:
            msg: JointState message from follower arm

        TODO: Implement this callback

        Same as _leader_callback but stores in self.latest_follower_state
        """

        # Your code here
        pass

    def _record_timer_callback(self):
        """
        Timer callback that records a frame if recording is active.

        Called at record_rate Hz. Only records if:
        1. Recording is active (self.is_recording)
        2. Both leader and follower states are available
        """
        if not self.is_recording:
            return

        if self.latest_leader_state is None or self.latest_follower_state is None:
            return

        # Get current timestamp
        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Add frame to buffer
        self.episode_buffer.add_frame(
            observation=self.latest_leader_state,
            action=self.latest_follower_state,
            timestamp=timestamp
        )

    # =========================================================================
    # RECORDING CONTROL - TODO: Implement
    # =========================================================================

    def start_recording(self) -> bool:
        """
        Start recording a new episode.

        Returns:
            True if recording started successfully, False otherwise

        TODO: Implement this function

        Steps:
        1. Check if already recording (return False if so)
        2. Reset the episode buffer
        3. Set is_recording = True
        4. Log info message
        5. Return True
        """

        # Your code here
        raise NotImplementedError("TODO: Implement start_recording")

    def stop_recording(self) -> Optional[str]:
        """
        Stop recording and save the episode.

        Returns:
            Path to saved episode file, or None if nothing to save

        TODO: Implement this function

        Steps:
        1. Check if currently recording (return None if not)
        2. Set is_recording = False
        3. Check if buffer has data (return None if empty)
        4. Save the episode using save_episode()
        5. Increment episode_count
        6. Reset the buffer
        7. Return the saved file path
        """

        # Your code here
        raise NotImplementedError("TODO: Implement stop_recording")

    def discard_recording(self):
        """
        Discard current recording without saving.

        TODO: Implement this function

        Steps:
        1. Set is_recording = False
        2. Reset the episode buffer
        3. Log warning message
        """

        # Your code here
        raise NotImplementedError("TODO: Implement discard_recording")

    # =========================================================================
    # FILE I/O - TODO: Implement
    # =========================================================================

    def save_episode(self, episode_data: Dict[str, np.ndarray]) -> str:
        """
        Save episode data to disk.

        Args:
            episode_data: Dictionary containing:
                - 'observation.state': np.ndarray of shape (T, 6)
                - 'action': np.ndarray of shape (T, 6)
                - 'timestamp': np.ndarray of shape (T,)
                - 'frame_index': np.ndarray of shape (T,)

        Returns:
            Path to saved file

        TODO: Implement this function

        Steps:
        1. Add episode_index to the data
        2. Generate filename using EPISODE_FORMAT and episode_count
        3. Create full path: self.output_dir / filename
        4. Save using np.savez(path, **episode_data)
        5. Log info message with file path and episode length
        6. Return the file path as string

        Hints:
        - episode_data['episode_index'] = self.episode_count
        - Use str(path) to convert Path to string
        """

        # Your code here
        raise NotImplementedError("TODO: Implement save_episode")

    def load_episode(self, episode_index: int) -> Optional[Dict[str, np.ndarray]]:
        """
        Load a previously saved episode.

        Args:
            episode_index: Index of episode to load

        Returns:
            Episode data dictionary, or None if not found

        TODO: Implement this function

        Steps:
        1. Generate filename using EPISODE_FORMAT
        2. Create full path
        3. Check if file exists (return None if not)
        4. Load using np.load(path)
        5. Convert to regular dict and return

        Hints:
        - Use path.exists() to check
        - np.load returns a NpzFile, convert with dict(npz_file)
        """

        # Your code here
        raise NotImplementedError("TODO: Implement load_episode")

    # =========================================================================
    # DATASET UTILITIES - TODO: Implement
    # =========================================================================

    def get_dataset_info(self) -> Dict:
        """
        Get information about the recorded dataset.

        Returns:
            Dictionary with dataset statistics

        TODO: Implement this function

        Return a dict containing:
        - 'num_episodes': Total number of recorded episodes
        - 'output_dir': Path to output directory
        - 'total_frames': Sum of frames across all episodes
        - 'episodes': List of episode summaries

        For each episode summary include:
        - 'index': Episode number
        - 'num_frames': Number of frames
        - 'duration': Duration in seconds
        """

        # Your code here
        raise NotImplementedError("TODO: Implement get_dataset_info")


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()

    # Simple command interface
    print("\nData Recorder Commands:")
    print("  'r' - Start recording")
    print("  's' - Stop recording and save")
    print("  'd' - Discard current recording")
    print("  'i' - Show dataset info")
    print("  'q' - Quit")
    print("")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # Check for user input (non-blocking)
            import sys
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().lower()
                if cmd == 'r':
                    if node.start_recording():
                        print("Recording started...")
                    else:
                        print("Already recording!")
                elif cmd == 's':
                    path = node.stop_recording()
                    if path:
                        print(f"Saved to: {path}")
                    else:
                        print("Nothing to save")
                elif cmd == 'd':
                    node.discard_recording()
                    print("Recording discarded")
                elif cmd == 'i':
                    info = node.get_dataset_info()
                    print(f"Episodes: {info['num_episodes']}")
                    print(f"Total frames: {info['total_frames']}")
                elif cmd == 'q':
                    break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# =============================================================================
# TESTS
# =============================================================================

def run_tests():
    """Run unit tests."""
    import tempfile
    import shutil

    print("Running data recorder tests...\n")

    # Test EpisodeBuffer
    print("Test 1: EpisodeBuffer operations")
    buffer = EpisodeBuffer()
    assert buffer.get_length() == 0

    obs = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5])
    act = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    buffer.add_frame(obs, act, 1.0)
    buffer.add_frame(obs * 2, act * 2, 1.02)
    assert buffer.get_length() == 2
    print(f"  ✓ Buffer length: {buffer.get_length()}\n")

    print("Test 2: Buffer to dict conversion")
    data = buffer.to_dict()
    assert 'observation.state' in data
    assert 'action' in data
    assert 'timestamp' in data
    assert data['observation.state'].shape == (2, 6)
    print(f"  ✓ Shapes: obs={data['observation.state'].shape}, act={data['action'].shape}\n")

    print("Test 3: Buffer reset")
    buffer.reset()
    assert buffer.get_length() == 0
    print("  ✓ Buffer reset successful\n")

    # Test file saving (mock)
    print("Test 4: Save and load episode")
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create test data
        test_data = {
            'observation.state': np.random.randn(100, 6),
            'action': np.random.randn(100, 6),
            'timestamp': np.arange(100) * 0.02,
            'frame_index': np.arange(100),
            'episode_index': 0,
        }

        # Save
        filepath = Path(tmpdir) / "test_episode.npz"
        np.savez(filepath, **test_data)

        # Load
        loaded = dict(np.load(filepath))
        assert loaded['observation.state'].shape == (100, 6)
        print(f"  ✓ Saved and loaded episode with {loaded['observation.state'].shape[0]} frames\n")

    print("=" * 50)
    print("All tests passed!")
    print("=" * 50)


if __name__ == '__main__':
    import sys
    if '--test' in sys.argv:
        run_tests()
    else:
        main()

# SO-101 Robotics Engineering Course

## Course Overview

| Phase | Weeks | Focus |
|-------|-------|-------|
| Foundation | 1-3 | Assembly, ROS2 basics, serial communication |
| Control | 4-6 | Motor drivers, web GUI, calibration |
| Embedded | 7-8 | Arduino integration, system testing |
| ML | 9-10 | Data collection, imitation learning |

## Quick Start

### 1. Fork & Clone

```bash
# Fork the repo on GitHub first, then:
git clone https://github.com/YOUR-USERNAME/SO101-ROS2.git
cd SO101-ROS2
```

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed git workflow.

### 2. Setup

No manual setup needed! The start script handles everything automatically.

### 3. Connect the Robot (USB)

**Windows (WSL2):**
```powershell
# In PowerShell as Administrator
usbipd list                       # Find BUSID
usbipd bind --busid 1-3           # First time only
usbipd attach --wsl --busid 1-3   # Each time you connect
```

**macOS / Linux:** USB appears automatically.

### 4. Run the Robot

```bash
cd lerobot_ws
./start_all.sh
```

This single command:
- Starts the Docker container (if not running)
- Detects your USB serial port automatically
- Launches all ROS2 nodes
- Starts the web GUI and Foxglove bridge

**To stop:**
```bash
docker compose -f docker/docker-compose.yml down
```

### 5. Control & Visualize

| What | URL |
|------|-----|
| Control GUI | http://localhost:8082 |
| Foxglove | https://app.foxglove.dev → connect to `ws://localhost:8765` |

**The robot should work immediately!** Move the sliders to control joints.

---

## Course Materials

### Weekly Guides

| Week | Topic | Guide | Lab |
|------|-------|-------|-----|
| 1 | Environment & Assembly | [week01.md](docs/week01.md) | Setup only |
| 2 | ROS2 Fundamentals | [week02.md](docs/week02.md) | `lab02_ros2_basics.py` |
| 3 | Serial Protocol | [week03.md](docs/week03.md) | `lab03_position_conversion.py` |
| 4 | Motor Bridge | [week04.md](docs/week04.md) | `lab04_motor_bridge.py` |
| 5 | Web GUI | [week05.md](docs/week05.md) | `lab05_gui_endpoint.py` |
| 6 | Calibration | [week06.md](docs/week06.md) | `lab06_calibration.py` |
| 7 | Arduino Integration | [week07.md](docs/week07.md) | `lab07_uart_bridge.py` + Arduino |
| 8 | Integration Testing | [week08.md](docs/week08.md) | *(Integration week - no new lab)* |
| 9 | Data Collection | [week09.md](docs/week09.md) | `lab09_data_recorder.py` |
| 10 | Model Training | [week10.md](docs/week10.md) | *(Colab notebook provided)* |

### Lab Files Location

Your exercises are in:
```
lerobot_ws/src/so101_hw_interface/so101_hw_interface/labs/
├── lab02_ros2_basics.py
├── lab03_position_conversion.py
├── lab04_motor_bridge.py
├── lab05_gui_endpoint.py
├── lab06_calibration.py
├── lab07_uart_bridge.py
└── lab09_data_recorder.py

hardware/arduino/button_controller/
└── button_controller.ino
```

### Running & Testing Labs

**Run tests locally:**
```bash
cd /ros2_ws/src/so101_hw_interface/so101_hw_interface/labs

# Run a lab (includes tests)
python3 lab03_position_conversion.py

# Expected output:
# Running tests...
# Test 1: ✓
# Test 2: ✓
# All tests passed!
```

**Run YOUR code on the real robot:**
```bash
# Test your motor bridge implementation
./start_all.sh --lab motor_bridge

# Test your GUI implementation
./start_all.sh --lab gui

# Test both at once
./start_all.sh --lab all

# Back to working code (if something breaks)
./start_all.sh
```

When you use `--lab`, YOUR code controls the actual robot!

---

## What's Provided vs What You Build

### Provided (Works Out of Box)

| Component | Purpose |
|-----------|---------|
| `motor_bridge.py` | Communicates with servos |
| `joint_slider_gui.py` | Web control interface |
| `start_all.sh` | Launches everything |
| Robot URDF | 3D model for visualization |
| Foxglove Bridge | Web-based visualization |

### You Implement (Labs)

| Lab | What You Learn | Runs on Robot? |
|-----|----------------|----------------|
| Lab 2 | ROS2 subscribers | Reads data |
| Lab 3 | Position math (radians ↔ ticks) | Tests only |
| Lab 4 | Building a ROS2 node | **Yes! `--lab motor_bridge`** |
| Lab 5 | HTTP request handling | **Yes! `--lab gui`** |
| Lab 6 | Calibration & YAML | Configures robot |
| Lab 7 | Arduino + UART | Controls robot |
| Lab 9 | Data recording for ML | Records demos |

Labs 4 and 5 can fully replace the working code - your implementation controls the real robot!

---

## Project Structure

```
SO101-ROS2/
├── README.md               # This file
├── CONTRIBUTING.md         # Git workflow for students
├── docs/                   # Weekly lab guides
│   ├── week01.md
│   └── ...
├── hardware/               # Arduino code & wiring
│   └── arduino/
│       ├── button_controller/
│       └── WIRING_DIAGRAM.md
└── lerobot_ws/             # ROS2 workspace
    ├── src/
    │   └── so101_hw_interface/
    │       └── so101_hw_interface/
    │           ├── motor_bridge.py      # PROVIDED
    │           ├── joint_slider_gui.py  # PROVIDED
    │           ├── labs/                # YOUR WORK
    │           └── motors/              # Feetech driver
    └── start_all.sh
```

---

## Hardware Reference

### SO-101 Robot Arm

- 6 degrees of freedom
- Feetech STS3215 servos
- 4096 ticks per revolution
- USB serial (CH340)

### Joints

| # | Name | Range |
|---|------|-------|
| 1 | Base | ±180° |
| 2 | Shoulder | ±90° |
| 3 | Elbow | ±90° |
| 4 | Wrist Pitch | ±90° |
| 5 | Wrist Roll | ±180° |
| 6 | Gripper | 0-90° |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "Device not found" | Windows: `usbipd attach --wsl --busid X-X` |
| "Permission denied" | `sudo chmod 666 /dev/ttyACM0` |
| Robot doesn't move | Check torque is ON in GUI |
| No 3D model in Foxglove | Restart `start_all.sh` |

See weekly guides for detailed troubleshooting.

---

## Getting Help

1. Read the weekly guide (`docs/weekXX.md`)
2. Check error messages carefully
3. Ask questions

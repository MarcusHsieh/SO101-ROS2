# Student Contribution Guide

This guide explains how to work with the course repository.

## Getting Started

### 1. Fork the Repository

1. Go to the course repository on GitHub
2. Click the **Fork** button (top right)
3. This creates a copy under your GitHub account

### 2. Clone Your Fork

```bash
# Clone YOUR fork (not the original)
git clone https://github.com/YOUR-USERNAME/SO101-ROS2.git
cd SO101-ROS2
```

### 3. Set Up Remotes

```bash
# Add the original repo as "upstream" for getting updates
git remote add upstream https://github.com/COURSE-ORG/SO101-ROS2.git

# Verify remotes
git remote -v
# origin    https://github.com/YOUR-USERNAME/SO101-ROS2.git (your fork)
# upstream  https://github.com/COURSE-ORG/SO101-ROS2.git (original)
```

## Weekly Workflow

### Before Each Week

Get any updates from the course repo:

```bash
git fetch upstream
git merge upstream/main
```

### Working on Labs

1. Create a branch for your work:
```bash
git checkout -b week03-lab
```

2. Do your lab work in the `labs/` folder:
```bash
code lerobot_ws/src/so101_hw_interface/so101_hw_interface/labs/lab03_position_conversion.py
```

3. Test your implementation:
```bash
cd lerobot_ws/src/so101_hw_interface/so101_hw_interface/labs
python3 lab03_position_conversion.py
```

4. Commit your work:
```bash
git add .
git commit -m "Complete lab 3: position conversion"
```

5. Push to your fork:
```bash
git push origin week03-lab
```

### Submitting Work (if required)

If your instructor requires pull requests:

1. Go to your fork on GitHub
2. Click "Pull requests" → "New pull request"
3. Select your branch
4. Add description of what you completed
5. Submit

## Repository Structure

```
SO101-ROS2/
├── README.md                 # Start here
├── CONTRIBUTING.md           # This file
├── docs/                     # Weekly lab guides
│   ├── week01.md
│   └── ...
├── hardware/                 # Arduino code
│   └── arduino/
│       └── button_controller/
├── lerobot_ws/              # ROS2 workspace
│   ├── src/
│   │   └── so101_hw_interface/
│   │       └── so101_hw_interface/
│   │           ├── motor_bridge.py      # PROVIDED - working
│   │           ├── joint_slider_gui.py  # PROVIDED - working
│   │           └── labs/                # YOUR WORK GOES HERE
│   │               ├── lab02_ros2_basics.py
│   │               ├── lab03_position_conversion.py
│   │               └── ...
│   └── start_all.sh
└── .instructor/             # NOT in student repo
```

## What's Provided vs What You Implement

### Provided (Working Out of Box)

These files are complete and working. Don't modify unless asked:

| File | Purpose |
|------|---------|
| `motor_bridge.py` | Talks to robot servos |
| `joint_slider_gui.py` | Web interface for control |
| `start_all.sh` | Launches everything |
| URDF files | Robot model |
| Docker config | Development environment |

### Your Labs (You Implement)

These are skeleton files with TODOs for you to complete:

| Lab | File | Week |
|-----|------|------|
| Lab 2 | `labs/lab02_ros2_basics.py` | Week 2 |
| Lab 3 | `labs/lab03_position_conversion.py` | Week 3 |
| Lab 4 | `labs/lab04_motor_bridge.py` | Week 4 |
| Lab 5 | `labs/lab05_gui_endpoint.py` | Week 5 |
| Lab 6 | `labs/lab06_calibration.py` | Week 6 |
| Lab 7 | `labs/lab07_uart_bridge.py` | Week 7 |
| Lab 9 | `labs/lab09_data_recorder.py` | Week 9 |

## Testing Your Work

Each lab file includes tests. Run them to verify your implementation:

```bash
# Navigate to labs folder
cd lerobot_ws/src/so101_hw_interface/so101_hw_interface/labs

# Run a specific lab's tests
python3 lab03_position_conversion.py

# Expected output when complete:
# Running position conversion tests...
# Test 1: ... ✓
# Test 2: ... ✓
# ...
# All tests passed!
```

## Common Git Commands

```bash
# See what files you changed
git status

# See your changes
git diff

# Stage all changes
git add .

# Commit with message
git commit -m "Your message here"

# Push to your fork
git push origin branch-name

# Create new branch
git checkout -b new-branch-name

# Switch branches
git checkout branch-name

# Get updates from course repo
git fetch upstream
git merge upstream/main
```

## Getting Help

1. Check the weekly guide in `docs/weekXX.md`
2. Read error messages carefully
3. Try the troubleshooting section in the guide


# Week 1: Environment Setup & Robot Assembly

## Learning Objectives

By the end of this week, you will be able to:
- Set up the Docker development environment
- Assemble the SO-101 robot arm
- Connect the robot via USB and verify communication
- Run the pre-built solution to see a working robot

## Prerequisites

- Laptop with Windows 10/11 (WSL2), macOS, or Ubuntu 22.04
- Docker Desktop installed and running
- Git installed

---

## Part 1: Environment Setup

### 1.1 Clone the Repository

```bash
git clone <repository-url>
cd SO101-ROS2
```

### 1.2 Verify Docker is Running

Make sure Docker Desktop is running. You can verify with:
```bash
docker --version
```

### 1.3 Test the Setup

The first time you run `start_all.sh`, it will:
- Build the Docker image (5-10 minutes first time)
- Start the container
- Build the ROS2 packages

```bash
cd lerobot_ws
./start_all.sh
```

**Checkpoint:** You should see "Startup complete!" with access points listed.

---

## Part 2: Robot Assembly

### 2.1 Unpack Components

Your kit should contain:
- [ ] 1x Base plate
- [ ] 6x Feetech STS3215 servo motors
- [ ] 1x Gripper assembly
- [ ] Structural brackets and screws
- [ ] 1x USB-to-Serial adapter (CH340)
- [ ] 1x 6V power supply
- [ ] Serial bus cables

### 2.2 Assembly Steps

Follow the official SO-ARM100 assembly guide:
https://github.com/TheRobotStudio/SO-ARM100

**Key points:**
1. Motors are numbered 1-6 (Base to Gripper)
2. Connect servo bus cables in daisy-chain configuration
3. Note the cable orientation (don't reverse polarity!)

### 2.3 Motor ID Verification

Each motor should already be programmed with its ID (1-6). If not, ask your instructor for help with the Feetech configuration tool.

| Motor Position | ID | Name |
|----------------|----|----- |
| Base rotation | 1 | Base |
| Shoulder | 2 | Shoulder |
| Elbow | 3 | Elbow |
| Wrist pitch | 4 | Wrist Pitch |
| Wrist roll | 5 | Wrist Roll |
| Gripper | 6 | Gripper |

---

## Part 3: USB Connection

### 3.1 Windows (WSL2) Users

USB devices don't automatically pass through to WSL2. You need `usbipd-win`.

**Install usbipd (one time, in PowerShell as Administrator):**
```powershell
winget install usbipd
```

**Attach USB (every time you connect the robot):**

1. Plug in the robot's USB cable
2. Open PowerShell as Administrator
3. List devices:
```powershell
usbipd list
```
4. Find your device (look for "CH340" or "USB Serial")
5. Note the BUSID (e.g., `1-3`)
6. Bind and attach:
```powershell
usbipd bind --busid 1-3
usbipd attach --wsl --busid 1-3
```

### 3.2 Linux Users

The device should appear automatically. Check with:
```bash
ls /dev/tty*
# Look for /dev/ttyUSB0 or /dev/ttyACM0
```

You may need to add your user to the `dialout` group:
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### 3.3 macOS Users

**Important:** Docker Desktop on macOS runs in a VM, so USB devices aren't directly accessible inside the container.

**Options:**
1. **Use serial-over-network** (e.g., `ser2net` on host)
2. **Run motor_bridge outside Docker** on your Mac
3. **Use a Linux/Windows machine** for physical robot work

For simulation and visualization (Foxglove, URDF), macOS works fine.

### 3.4 Verify Connection

In your container terminal:
```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

If you see the device, you're connected!

---

## Part 4: Run the Demo

### 4.1 Start All Services

From your host terminal (not inside a container):
```bash
cd SO101-ROS2/lerobot_ws
./start_all.sh
```

This single command handles everything:
- Starts the Docker container automatically (if not running)
- Detects your USB serial port
- Builds the ROS2 packages
- Launches all services (motor bridge, GUI, Foxglove)

You should see output like:
```
==========================================
SO-101 Robot Arm Startup Script
==========================================
Container not running. Starting with docker compose...
Container so101_ros2 is running

Detecting serial port...
Found serial port: /dev/ttyACM0

[1/7] Rebuilding packages...
...
Startup complete!

Access points:
  - Joint Control GUI: http://localhost:8082
  - Foxglove Bridge:   ws://localhost:8765
```

### 4.2 Open the Control GUI

1. Open a web browser
2. Go to: http://localhost:8082
3. You should see sliders for each joint

### 4.3 Test the Robot

1. Make sure torque is **ON** (green indicator)
2. Slowly move one slider
3. The robot should move!

**Safety:** Always move slowly and keep hands clear of the robot.

### 4.4 Connect Foxglove (Optional)

1. Open https://app.foxglove.dev
2. Click "Open connection"
3. Select "Foxglove WebSocket"
4. Enter: `ws://localhost:8765`
5. Add a 3D panel
6. You should see the robot model moving!

---

## Lab Exercise

### Task: Explore the System

1. Move each joint using the GUI and note its range of motion
2. Click "Torque OFF" and manually move the arm - watch Foxglove update
3. Click "Torque ON" and try the "Home" button

### Questions to Answer

1. What happens when you move a slider too fast?
2. What's the difference between torque ON and OFF?
3. Can you identify which physical joint corresponds to each slider?

### Deliverables

- [ ] Screenshot of the GUI with robot connected
- [ ] Screenshot of Foxglove showing the 3D model
- [ ] Answers to the questions above

---

## Troubleshooting

### "Device not found"

- Windows: Make sure you ran `usbipd attach`
- Check cable connection
- Try a different USB port

### "Permission denied"

```bash
sudo chmod 666 /dev/ttyACM0
```

### Robot doesn't move

1. Is power connected to the servo bus?
2. Is torque enabled in the GUI?
3. Check logs: `docker exec so101_ros2 cat /tmp/motor_bridge.log`

### Container won't build

1. Make sure Docker Desktop is running
2. Try: "Dev Containers: Rebuild Container"

---

## What's Next?

In Week 2, you'll learn:
- How ROS2 nodes communicate
- What topics and messages are
- How to write your first subscriber

---

## Reference

### Useful Commands

```bash
# Check what nodes are running
ros2 node list

# See what topics exist
ros2 topic list

# Watch joint states in real-time
ros2 topic echo /so101_follower/joint_states

# Check logs
docker exec so101_ros2 cat /tmp/motor_bridge.log
```

### File Locations

| What | Where |
|------|-------|
| Startup script | `lerobot_ws/start_all.sh` |
| Motor bridge code | `lerobot_ws/src/so101_hw_interface/so101_hw_interface/motor_bridge.py` |
| GUI code | `lerobot_ws/src/so101_hw_interface/so101_hw_interface/joint_slider_gui.py` |
| URDF | `lerobot_ws/src/lerobot_description/urdf/so101.urdf.xacro` |

### To Stop

```bash
cd lerobot_ws
docker compose -f docker/docker-compose.yml down
```

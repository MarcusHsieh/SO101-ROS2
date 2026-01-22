#!/bin/bash
# SO-101 Robot Arm - Start All Services
# This script starts all necessary ROS2 nodes for the robot arm
#
# Usage:
#   ./start_all.sh                    # Run with provided working code
#   ./start_all.sh --lab motor_bridge # Run with student's motor bridge lab
#   ./start_all.sh --lab gui          # Run with student's GUI lab
#   ./start_all.sh --lab all          # Run with all student labs

set -e

CONTAINER="so101_ros2"
USE_LAB_MOTOR_BRIDGE=false
USE_LAB_GUI=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --lab)
            shift
            case $1 in
                motor_bridge|motor-bridge)
                    USE_LAB_MOTOR_BRIDGE=true
                    ;;
                gui)
                    USE_LAB_GUI=true
                    ;;
                all)
                    USE_LAB_MOTOR_BRIDGE=true
                    USE_LAB_GUI=true
                    ;;
                *)
                    echo "Unknown lab: $1"
                    echo "Available labs: motor_bridge, gui, all"
                    exit 1
                    ;;
            esac
            shift
            ;;
        -h|--help)
            echo "Usage: ./start_all.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --lab motor_bridge   Use student's lab04_motor_bridge.py"
            echo "  --lab gui            Use student's lab05_gui_endpoint.py"
            echo "  --lab all            Use all student lab implementations"
            echo "  -h, --help           Show this help message"
            echo ""
            echo "Examples:"
            echo "  ./start_all.sh                      # Run working code (default)"
            echo "  ./start_all.sh --lab motor_bridge   # Test your motor bridge lab"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "SO-101 Robot Arm Startup Script"
echo "=========================================="

# Show which code will be used
if $USE_LAB_MOTOR_BRIDGE || $USE_LAB_GUI; then
    echo ""
    echo "LAB MODE ENABLED:"
    $USE_LAB_MOTOR_BRIDGE && echo "  - Motor Bridge: labs/lab04_motor_bridge.py"
    $USE_LAB_GUI && echo "  - GUI: labs/lab05_gui_endpoint.py"
    echo ""
fi

# Check if container is running, start if needed
if ! docker ps | grep -q $CONTAINER; then
    echo "Container not running. Starting with docker compose..."
    cd "$(dirname "$0")"
    docker compose -f docker/docker-compose.yml -f docker/docker-compose.override.yml up -d ros2 2>&1
    echo "Waiting for container to initialize..."
    sleep 5
fi

echo "Container $CONTAINER is running"

# Auto-detect serial port
echo ""
echo "Detecting serial port..."
SERIAL_PORT=$(docker exec $CONTAINER bash -c 'ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1')
if [ -z "$SERIAL_PORT" ]; then
    echo "WARNING: No serial device found in container!"
    echo "Make sure USB is connected and passed to Docker."
    echo "On WSL2: usbipd attach --wsl --busid <BUSID>"
    SERIAL_PORT="/dev/ttyACM0"  # fallback
else
    echo "Found serial port: $SERIAL_PORT"
fi

# Rebuild packages to ensure latest code
echo ""
echo "[1/7] Rebuilding packages..."
docker exec $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select so101_hw_interface 2>&1' | tail -5

# Kill any existing processes
echo ""
echo "[2/7] Cleaning up existing processes..."
docker exec $CONTAINER pkill -f "motor_bridge" 2>/dev/null || true
docker exec $CONTAINER pkill -f "joint_slider_gui" 2>/dev/null || true
docker exec $CONTAINER pkill -f "lab04_motor_bridge" 2>/dev/null || true
docker exec $CONTAINER pkill -f "lab05_gui" 2>/dev/null || true
docker exec $CONTAINER pkill -f "robot_state_publisher" 2>/dev/null || true
docker exec $CONTAINER pkill -f "foxglove_bridge" 2>/dev/null || true
docker exec $CONTAINER pkill -f "http.server" 2>/dev/null || true
sleep 2

# Generate URDF with HTTP URLs for meshes
echo ""
echo "[3/7] Generating URDF with HTTP mesh URLs..."
docker exec $CONTAINER bash -c '
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
xacro /ros2_ws/src/lerobot_description/urdf/so101.urdf.xacro > /tmp/so101_complete.urdf 2>&1
sed "s|package://lerobot_description/|http://localhost:9090/lerobot_description/|g" /tmp/so101_complete.urdf > /tmp/so101_http.urdf
echo "URDF generated: /tmp/so101_http.urdf"
'

# Create launch file
echo ""
echo "[4/7] Creating robot_state_publisher launch file..."
docker exec $CONTAINER bash -c 'cat > /tmp/rsp_launch2.py << "LAUNCH_EOF"
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    with open("/tmp/so101_http.urdf", "r") as f:
        urdf_content = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": urdf_content}],
            remappings=[("/joint_states", "/so101_follower/joint_states")],
            output="screen",
        ),
    ])
LAUNCH_EOF
echo "Launch file created: /tmp/rsp_launch2.py"
'

# Start mesh server
echo ""
echo "[5/7] Starting mesh HTTP server on port 9090..."
docker exec -d $CONTAINER bash -c 'cd /ros2_ws/install/lerobot_description/share && python3 -m http.server 9090 > /tmp/mesh_server.log 2>&1'
sleep 1

# Start all ROS2 nodes
echo ""
echo "[6/7] Starting ROS2 nodes..."

# Motor bridge - use lab or working version
if $USE_LAB_MOTOR_BRIDGE; then
    echo "  - Starting motor_bridge (LAB VERSION) on $SERIAL_PORT..."
    docker exec -d $CONTAINER bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/so101_hw_interface/so101_hw_interface/labs && python3 lab04_motor_bridge.py --ros-args -p port:=$SERIAL_PORT > /tmp/motor_bridge.log 2>&1"
else
    echo "  - Starting motor_bridge on $SERIAL_PORT..."
    docker exec -d $CONTAINER bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run so101_hw_interface so101_motor_bridge --ros-args -p port:=$SERIAL_PORT > /tmp/motor_bridge.log 2>&1"
fi
sleep 2

# Robot state publisher
echo "  - Starting robot_state_publisher..."
docker exec -d $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch /tmp/rsp_launch2.py > /tmp/rsp.log 2>&1'
sleep 1

# Foxglove bridge
echo "  - Starting foxglove_bridge..."
docker exec -d $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run foxglove_bridge foxglove_bridge > /tmp/foxglove.log 2>&1'
sleep 1

# Joint slider GUI - use lab or working version
if $USE_LAB_GUI; then
    echo "  - Starting joint_slider_gui (LAB VERSION)..."
    docker exec -d $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/so101_hw_interface/so101_hw_interface/labs && python3 lab05_gui_endpoint.py > /tmp/gui.log 2>&1'
else
    echo "  - Starting joint_slider_gui..."
    docker exec -d $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && cd /ros2_ws/src/so101_hw_interface/so101_hw_interface && python3 joint_slider_gui.py > /tmp/gui.log 2>&1'
fi
sleep 2

# Verify everything is running
echo ""
echo "[7/7] Verifying services..."
echo ""
echo "Running nodes:"
docker exec $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null' || echo "  waiting for nodes..."

echo ""
echo "Available services:"
docker exec $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && ros2 service list 2>/dev/null | grep -E "torque|joint"' || echo "  waiting for services..."

echo ""
echo "=========================================="
echo "Startup complete!"
echo ""
if $USE_LAB_MOTOR_BRIDGE || $USE_LAB_GUI; then
    echo "RUNNING IN LAB MODE"
    $USE_LAB_MOTOR_BRIDGE && echo "  - Motor Bridge: YOUR LAB CODE"
    $USE_LAB_GUI && echo "  - GUI: YOUR LAB CODE"
    echo ""
fi
echo "Access points:"
echo "  - Joint Control GUI: http://localhost:8082"
echo "  - Foxglove Bridge:   ws://localhost:8765"
echo "  - Mesh Server:       http://localhost:9090"
echo ""
echo "Log files [inside container]:"
echo "  - /tmp/motor_bridge.log"
echo "  - /tmp/rsp.log"
echo "  - /tmp/foxglove.log"
echo "  - /tmp/gui.log"
echo "  - /tmp/mesh_server.log"
echo ""
echo "To view logs: docker exec $CONTAINER cat /tmp/<logfile>"
echo "To see errors: docker exec $CONTAINER tail -20 /tmp/motor_bridge.log"
echo "=========================================="

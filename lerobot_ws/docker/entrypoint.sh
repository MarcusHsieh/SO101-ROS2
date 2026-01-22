#!/bin/bash
# SO-101 ROS2 Docker Entrypoint
# ==============================
# This script sets up the ROS2 environment and optionally starts services.

set -e

# Source ROS2 and workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

# Set default ROS_DOMAIN_ID if not set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Print welcome message
echo "=================================================="
echo "  SO-101 ROS2 Robotics Course Container"
echo "  ROS Distro: ${ROS_DISTRO}"
echo "  ROS Domain ID: ${ROS_DOMAIN_ID}"
echo "=================================================="
echo ""
echo "Available launch commands:"
echo "  # Visualization only (RViz)"
echo "  ros2 launch lerobot_description so101_display.launch.py"
echo ""
echo "  # Gazebo simulation (Linux with GPU)"
echo "  ros2 launch lerobot_description so101_gazebo.launch.py"
echo ""
echo "  # Controllers"
echo "  ros2 launch lerobot_controller so101_controller.launch.py"
echo ""
echo "  # MoveIt planning"
echo "  ros2 launch lerobot_moveit so101_moveit.launch.py"
echo ""
echo "  # Foxglove Bridge (for web visualization)"
echo "  ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
echo ""
echo "  # Hardware interface (real robot)"
echo "  ros2 launch so101_hw_interface so101_hw.launch.py"
echo ""
echo "=================================================="
echo ""

# Handle different startup modes
case "$1" in
    "foxglove")
        echo "Starting Foxglove Bridge..."
        exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml
        ;;
    "display")
        echo "Starting robot visualization..."
        exec ros2 launch lerobot_description so101_display.launch.py
        ;;
    "sim")
        echo "Starting Gazebo simulation..."
        exec ros2 launch lerobot_description so101_gazebo.launch.py
        ;;
    "moveit")
        echo "Starting MoveIt..."
        exec ros2 launch lerobot_moveit so101_moveit.launch.py
        ;;
    "all-sim")
        echo "Starting full simulation stack (Gazebo + Controllers + Foxglove)..."
        # Use tmux to run multiple processes
        tmux new-session -d -s ros2 'ros2 launch lerobot_description so101_gazebo.launch.py'
        sleep 5
        tmux new-window -t ros2 'ros2 launch lerobot_controller so101_controller.launch.py'
        sleep 2
        tmux new-window -t ros2 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
        echo "Started in tmux session 'ros2'. Attach with: tmux attach -t ros2"
        exec bash
        ;;
    *)
        # Default: run the provided command or bash
        exec "$@"
        ;;
esac

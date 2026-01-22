#!/bin/bash
# SO-101 ROS2 Setup Script for Linux
# ===================================
# This script sets up the development environment on native Linux.
#
# Usage:
#   chmod +x setup_linux.sh
#   ./setup_linux.sh

set -e

echo "=========================================="
echo "  SO-101 ROS2 Setup for Linux"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}Please do not run this script as root${NC}"
    exit 1
fi

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Step 1: Check Docker installation
echo ""
echo -e "${YELLOW}Step 1: Checking Docker installation...${NC}"
if command_exists docker; then
    DOCKER_VERSION=$(docker --version)
    echo -e "${GREEN}Docker is installed: $DOCKER_VERSION${NC}"
else
    echo -e "${RED}Docker is not installed.${NC}"
    echo "Please install Docker first:"
    echo "  https://docs.docker.com/engine/install/"
    exit 1
fi

# Check if user is in docker group
if groups | grep -q docker; then
    echo -e "${GREEN}User is in docker group${NC}"
else
    echo -e "${YELLOW}Adding user to docker group...${NC}"
    sudo usermod -aG docker $USER
    echo -e "${YELLOW}Please log out and back in for group changes to take effect${NC}"
fi

# Step 2: Check Docker Compose
echo ""
echo -e "${YELLOW}Step 2: Checking Docker Compose...${NC}"
if command_exists docker-compose || docker compose version >/dev/null 2>&1; then
    echo -e "${GREEN}Docker Compose is available${NC}"
else
    echo -e "${RED}Docker Compose is not installed${NC}"
    echo "Please install Docker Compose or update Docker Desktop"
    exit 1
fi

# Step 3: Check for USB serial devices
echo ""
echo -e "${YELLOW}Step 3: Checking USB serial devices...${NC}"
if ls /dev/ttyUSB* 2>/dev/null || ls /dev/ttyACM* 2>/dev/null; then
    echo -e "${GREEN}USB serial devices found${NC}"
    ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
else
    echo -e "${YELLOW}No USB serial devices found. Connect the robot to see devices.${NC}"
fi

# Step 4: Set up udev rules for USB access
echo ""
echo -e "${YELLOW}Step 4: Setting up udev rules for USB access...${NC}"
UDEV_RULE_FILE="/etc/udev/rules.d/99-so101-robot.rules"
if [ -f "$UDEV_RULE_FILE" ]; then
    echo -e "${GREEN}udev rules already exist${NC}"
else
    echo "Creating udev rules for USB serial access..."
    sudo tee "$UDEV_RULE_FILE" > /dev/null << 'EOF'
# SO-101 Robot Arm USB Serial Rules
# Feetech USB-to-Serial adapter
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", SYMLINK+="so101_robot"
# CH340 USB-to-Serial (common adapter)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7522", MODE="0666", GROUP="dialout"
# FTDI USB-to-Serial
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout"
# CP210x USB-to-Serial
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}udev rules created${NC}"
fi

# Ensure user is in dialout group
if groups | grep -q dialout; then
    echo -e "${GREEN}User is in dialout group${NC}"
else
    echo -e "${YELLOW}Adding user to dialout group...${NC}"
    sudo usermod -aG dialout $USER
    echo -e "${YELLOW}Please log out and back in for group changes to take effect${NC}"
fi

# Step 5: Install Python dependencies for motor_bridge.py
echo ""
echo -e "${YELLOW}Step 5: Installing Python dependencies...${NC}"
if command_exists pip3; then
    pip3 install --user pyserial numpy pyyaml
    # Try to install scservo-sdk
    pip3 install --user scservo-sdk 2>/dev/null || \
        echo -e "${YELLOW}scservo-sdk not found in PyPI, motor_bridge.py has built-in driver${NC}"
    echo -e "${GREEN}Python dependencies installed${NC}"
else
    echo -e "${YELLOW}pip3 not found, skipping Python dependencies${NC}"
fi

# Step 6: Build Docker image
echo ""
echo -e "${YELLOW}Step 6: Building Docker image...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

if [ -f "docker/Dockerfile" ]; then
    echo "Building Docker image (this may take a while)..."
    docker build -t so101-ros2:humble -f docker/Dockerfile .
    echo -e "${GREEN}Docker image built successfully${NC}"
else
    echo -e "${RED}Dockerfile not found at docker/Dockerfile${NC}"
    exit 1
fi

# Step 7: Create environment file
echo ""
echo -e "${YELLOW}Step 7: Creating environment configuration...${NC}"
ENV_FILE="$WORKSPACE_DIR/docker/.env"
cat > "$ENV_FILE" << EOF
# SO-101 Docker Environment Configuration
# Linux configuration

# Use host network for best ROS2 DDS performance
NETWORK_MODE=host

# ROS2 Domain ID (change if multiple robots on network)
ROS_DOMAIN_ID=0

# Display for GUI applications
DISPLAY=$DISPLAY

# Serial port for robot (adjust as needed)
SERIAL_PORT=/dev/ttyUSB0
EOF
echo -e "${GREEN}Environment file created at $ENV_FILE${NC}"

# Step 8: Test setup
echo ""
echo -e "${YELLOW}Step 8: Testing setup...${NC}"
if docker run --rm so101-ros2:humble ros2 --help >/dev/null 2>&1; then
    echo -e "${GREEN}Docker container test passed${NC}"
else
    echo -e "${RED}Docker container test failed${NC}"
    exit 1
fi

# Print summary
echo ""
echo "=========================================="
echo -e "${GREEN}  Setup Complete!${NC}"
echo "=========================================="
echo ""
echo "Quick Start Commands:"
echo ""
echo "  # Start ROS2 container interactively"
echo "  cd $WORKSPACE_DIR/docker"
echo "  docker compose up -d ros2"
echo "  docker exec -it so101_ros2 bash"
echo ""
echo "  # Start Foxglove visualization"
echo "  docker compose up -d foxglove"
echo "  # Open http://localhost:8765 in Foxglove Studio"
echo ""
echo "  # Run motor bridge for real robot"
echo "  python3 $WORKSPACE_DIR/scripts/motor_bridge.py --port /dev/ttyUSB0"
echo ""
echo "  # OR run everything in Docker with USB passthrough"
echo "  docker run -it --rm --network=host \\"
echo "    --device=/dev/ttyUSB0 \\"
echo "    so101-ros2:humble bash"
echo ""
echo "For GUI applications (RViz), allow X11 access first:"
echo "  xhost +local:docker"
echo ""

#!/bin/bash
# SO-101 ROS2 Setup Script for macOS
# ====================================
# This script sets up the development environment on macOS.
#
# Note: macOS cannot pass USB devices directly to Docker.
# This setup uses the hybrid architecture:
#   - ROS2 runs in Docker container
#   - motor_bridge.py runs natively on macOS for USB access
#   - Communication via rosbridge WebSocket
#
# Usage:
#   chmod +x setup_macos.sh
#   ./setup_macos.sh

set -e

echo "=========================================="
echo "  SO-101 ROS2 Setup for macOS"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Detect architecture
ARCH=$(uname -m)
echo "Detected architecture: $ARCH"
if [ "$ARCH" = "arm64" ]; then
    echo -e "${GREEN}Apple Silicon detected - using ARM64 Docker images${NC}"
else
    echo "Intel Mac detected - using AMD64 Docker images"
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Step 1: Check Homebrew
echo ""
echo -e "${YELLOW}Step 1: Checking Homebrew...${NC}"
if command_exists brew; then
    echo -e "${GREEN}Homebrew is installed${NC}"
else
    echo "Homebrew is not installed. Installing..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    echo -e "${GREEN}Homebrew installed${NC}"
fi

# Step 2: Check Docker Desktop
echo ""
echo -e "${YELLOW}Step 2: Checking Docker Desktop...${NC}"
if command_exists docker; then
    DOCKER_VERSION=$(docker --version)
    echo -e "${GREEN}Docker is installed: $DOCKER_VERSION${NC}"

    # Check if Docker is running
    if docker info >/dev/null 2>&1; then
        echo -e "${GREEN}Docker Desktop is running${NC}"
    else
        echo -e "${RED}Docker Desktop is not running. Please start it.${NC}"
        echo "Open Docker Desktop from Applications or run:"
        echo "  open -a Docker"
        exit 1
    fi
else
    echo -e "${RED}Docker is not installed${NC}"
    echo "Please install Docker Desktop for Mac:"
    echo "  https://docs.docker.com/desktop/install/mac-install/"
    echo ""
    echo "Or install via Homebrew:"
    echo "  brew install --cask docker"
    exit 1
fi

# Step 3: Install Python dependencies
echo ""
echo -e "${YELLOW}Step 3: Installing Python dependencies...${NC}"
if command_exists python3; then
    PYTHON_VERSION=$(python3 --version)
    echo "Python: $PYTHON_VERSION"

    # Install pip if needed
    if ! command_exists pip3; then
        echo "Installing pip..."
        python3 -m ensurepip --upgrade
    fi

    # Install dependencies
    echo "Installing Python packages..."
    pip3 install --user pyserial numpy pyyaml websocket-client

    # Try to install scservo-sdk
    pip3 install --user scservo-sdk 2>/dev/null || \
        echo -e "${YELLOW}scservo-sdk not in PyPI, motor_bridge.py has built-in driver${NC}"

    echo -e "${GREEN}Python dependencies installed${NC}"
else
    echo -e "${RED}Python 3 not found${NC}"
    echo "Install Python 3 via Homebrew:"
    echo "  brew install python"
    exit 1
fi

# Step 4: Check for USB serial devices
echo ""
echo -e "${YELLOW}Step 4: Checking USB serial devices...${NC}"
echo "Looking for USB serial devices..."

# macOS uses /dev/tty.* and /dev/cu.* for serial devices
SERIAL_DEVICES=$(ls /dev/tty.usb* /dev/cu.usb* /dev/tty.wchusbserial* /dev/cu.wchusbserial* 2>/dev/null || true)
if [ -n "$SERIAL_DEVICES" ]; then
    echo -e "${GREEN}USB serial devices found:${NC}"
    echo "$SERIAL_DEVICES"
else
    echo -e "${YELLOW}No USB serial devices found.${NC}"
    echo "Connect the SO-101 robot to see devices."
    echo "Common device names on macOS:"
    echo "  /dev/tty.usbserial-*     (FTDI adapter)"
    echo "  /dev/tty.wchusbserial*   (CH340 adapter)"
    echo "  /dev/cu.usbmodem*        (USB CDC device)"
fi

# Step 5: Install CH340 driver if needed
echo ""
echo -e "${YELLOW}Step 5: Checking CH340 USB driver...${NC}"
if [ "$ARCH" = "arm64" ]; then
    echo "Note: macOS on Apple Silicon usually has built-in CH340 support"
    echo "If your device is not detected, you may need the CH340 driver:"
    echo "  https://github.com/adrianmihalko/ch340g-ch34g-ch34x-mac-os-x-driver"
else
    echo "Intel Mac may need CH340 driver for some USB adapters"
    echo "Download from: https://www.wch.cn/downloads/CH341SER_MAC_ZIP.html"
fi

# Step 6: Build Docker image
echo ""
echo -e "${YELLOW}Step 6: Building Docker image...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

if [ -f "docker/Dockerfile" ]; then
    echo "Building Docker image (this may take a while on first run)..."
    # Enable BuildKit for faster builds
    DOCKER_BUILDKIT=1 docker build -t so101-ros2:humble -f docker/Dockerfile .
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
# macOS configuration

# Use bridge network (host network not supported on macOS)
NETWORK_MODE=bridge

# ROS2 Domain ID (change if multiple robots on network)
ROS_DOMAIN_ID=0

# Serial port (adjust based on your USB adapter)
# Run: ls /dev/tty.* /dev/cu.*
SERIAL_PORT=/dev/tty.usbserial-0001
EOF
echo -e "${GREEN}Environment file created at $ENV_FILE${NC}"

# Step 8: Create launcher script
echo ""
echo -e "${YELLOW}Step 8: Creating launcher scripts...${NC}"

# Create start script
cat > "$SCRIPT_DIR/start_macos.sh" << 'EOF'
#!/bin/bash
# start_macos.sh - Start SO-101 development environment on macOS
#
# This starts:
# 1. Docker container with ROS2
# 2. motor_bridge.py for USB communication (if serial device found)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  Starting SO-101 Development Environment"
echo "=========================================="
echo ""

# Start Docker containers
echo "Starting Docker containers..."
cd "$WORKSPACE_DIR/docker"
docker compose up -d ros2 foxglove
echo "Docker containers started"
echo ""

# Find serial device
SERIAL_PORT=""
for device in /dev/tty.usbserial* /dev/tty.wchusbserial* /dev/cu.usbserial* /dev/cu.wchusbserial*; do
    if [ -e "$device" ]; then
        SERIAL_PORT="$device"
        break
    fi
done

if [ -n "$SERIAL_PORT" ]; then
    echo "Found serial device: $SERIAL_PORT"
    echo "Starting motor bridge..."
    echo ""
    echo "Motor bridge output (Ctrl+C to stop):"
    echo "----------------------------------------"
    python3 "$SCRIPT_DIR/motor_bridge.py" --port "$SERIAL_PORT" --mode websocket
else
    echo "No serial device found. Robot not connected?"
    echo ""
    echo "To manually start the motor bridge later:"
    echo "  python3 $SCRIPT_DIR/motor_bridge.py --port /dev/tty.usbserial-XXX --mode websocket"
    echo ""
    echo "Docker containers are running. Access:"
    echo "  docker exec -it so101_ros2 bash"
    echo "  Foxglove: http://localhost:8765"
fi
EOF
chmod +x "$SCRIPT_DIR/start_macos.sh"
echo -e "${GREEN}Created: $SCRIPT_DIR/start_macos.sh${NC}"

# Create stop script
cat > "$SCRIPT_DIR/stop_macos.sh" << 'EOF'
#!/bin/bash
# stop_macos.sh - Stop SO-101 development environment

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "Stopping Docker containers..."
cd "$WORKSPACE_DIR/docker"
docker compose down
echo "Done!"
EOF
chmod +x "$SCRIPT_DIR/stop_macos.sh"
echo -e "${GREEN}Created: $SCRIPT_DIR/stop_macos.sh${NC}"

# Step 9: Test setup
echo ""
echo -e "${YELLOW}Step 9: Testing setup...${NC}"
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
echo "Quick Start:"
echo ""
echo "  # Start everything (Docker + motor bridge)"
echo "  ./scripts/start_macos.sh"
echo ""
echo "  # Or start manually:"
echo "  cd $WORKSPACE_DIR/docker"
echo "  docker compose up -d"
echo ""
echo "  # Run motor bridge (separate terminal)"
echo "  python3 $WORKSPACE_DIR/scripts/motor_bridge.py \\"
echo "    --port /dev/tty.usbserial-XXX --mode websocket"
echo ""
echo "Visualization:"
echo "  Open Foxglove Studio and connect to:"
echo "  ws://localhost:8765"
echo ""
echo "Interactive ROS2 shell:"
echo "  docker exec -it so101_ros2 bash"
echo ""
echo -e "${YELLOW}Note: macOS cannot pass USB directly to Docker.${NC}"
echo "The hybrid architecture uses motor_bridge.py running"
echo "natively on macOS for USB communication."
echo ""

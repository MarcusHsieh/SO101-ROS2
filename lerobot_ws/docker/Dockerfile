# SO-101 ROS2 Robotics Course Docker Image
# ==========================================
# Supports: amd64 (Intel/AMD) and arm64 (Apple Silicon)
#
# Build:
#   docker build -t so101-ros2:humble -f docker/Dockerfile .
#
# Run (Linux with GPU):
#   docker run -it --rm --network=host --device=/dev/ttyUSB0 so101-ros2:humble
#
# Run (macOS/Windows - no direct USB):
#   docker run -it --rm -p 8765:8765 -p 9090:9090 so101-ros2:humble

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

LABEL maintainer="SO-101 Robotics Course"
LABEL description="ROS2 environment for SO-101 robot arm with MoveIt, Foxglove, and cross-platform support"

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

# Set up locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    wget \
    curl \
    vim \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rosbridge-suite \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-teleop-twist-joy \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt* \
    && rm -rf /var/lib/apt/lists/*

# Install topic-based-ros2-control if available, or build from source
RUN apt-get update && \
    (apt-get install -y ros-${ROS_DISTRO}-topic-based-ros2-control 2>/dev/null || \
     echo "topic-based-ros2-control not available as apt package, will build from source") && \
    rm -rf /var/lib/apt/lists/*

# Install Python packages for motor driver
RUN pip3 install --no-cache-dir \
    pyserial \
    pyyaml \
    numpy \
    feetech-servo-sdk \
    deepdiff \
    tqdm

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY ./src /ros2_ws/src

# topic_based_ros2_control is installed via apt above, no need to build from source

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Source workspace on container start
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

# Expose ports for communication
# 8765: Foxglove Bridge WebSocket
# 9090: Rosbridge WebSocket
# 11311: ROS Master (legacy, not used in ROS2 but sometimes expected)
EXPOSE 8765 9090 11311

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

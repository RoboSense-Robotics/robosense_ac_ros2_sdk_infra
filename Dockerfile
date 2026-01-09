# Multi-arch Dockerfile for robosense_ac_ros2_sdk_infra
# Supports amd64 and arm64

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base AS builder

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libusb-1.0-0-dev \
    libuvc-dev \
    libopencv-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libavutil-dev \
    libavdevice-dev \
    libswresample-dev \
    libpostproc-dev \
    liblzma-dev \
    libnuma-dev \
    libbz2-dev \
    libasound2-dev \
    libxcb1-dev \
    libx11-dev \
    libxv-dev \
    libxext-dev \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-msgs \
    ros-${ROS_DISTRO}-diagnostic-updater \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws/src
COPY . .

# Build workspace
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ac_driver ac_codec

# Final stage
FROM ros:${ROS_DISTRO}-ros-core

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    libuvc0 \
    libopencv-imgcodecs406t64 \
    libopencv-imgproc406t64 \
    libavcodec60 \
    libavformat60 \
    libswscale7 \
    libavutil58 \
    libavdevice60 \
    libnuma1 \
    libasound2t64 \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-lifecycle \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY --from=builder /ros2_ws/install ./install

# Setup entrypoint
COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

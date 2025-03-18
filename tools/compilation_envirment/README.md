# Super Sensor SDK Docker Environment Management Tool

[README](README.md) | [中文文档](README_CN.md)

## 1. Introduction

This tool provides a convenient way to manage Docker containers for the Super Sensor SDK's cross-compilation and local compilation environments (only supports ROS 2 Humble). It includes container management, image management, and automated environment setup.

### Key Features
- **Cross-Compilation Environment** (ARM64 architecture support)
- **Local Compilation Environment** (Ubuntu 20.04 + ROS 2 Humble)
- Automatic Docker image management
- Container lifecycle management
- User permission mapping
- Development required volume mounts

## 2. Prerequisites

### 2.1 Install Docker
Follow the [official Docker installation guide](https://docs.docker.com/engine/install/) for your operating system.

### 2.2 Install multi-platform support
```bash
# install qemu-user-static
sudo apt-get install qemu-user-static
# enable binfmt support
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## 3. Installation & Setup

### 3.1 Docker Images

The tool uses the following pre-built images:

ARM compilation image: arm_sdk_compile_humble_based:latest
x86 compilation image: x86_sdk_compile_humble_based:latest

### 3.2 Image Management
On first use, the script will automatically:

Download the image package from CDN (~3GB) Extract and load the Docker image Verify image integrity Note: Image hosting address: https://cdn.robosense.cn/

## 4. Usage

### 4.1 Container Management

Use the start_container.sh script to manage containers:

```bash
cd src/super_sensor_sdk/sdk_infra/tools/cross_compile
./start_container.sh [option]
```

Available options:
- `arm`: Start ARM compilation environment
- `stop`: Stop all SDK compilation containers
- `status`: Show container status
- `help`: Display help information

### 4.2 Volume Mounts

The following paths are mounted in the containers:

- Project workspace: `$(pwd)/../../../../..` → `/workspace`
- System files:
    - `/etc/timezone`
    - `/etc/localtime`
    - `/etc/passwd`
    - `/etc/group`
- Devices: `/dev`
- X11 display: `/tmp/.X11-unix`

## 5. Examples

### ARM Compilation Process

```bash
./start_container.sh arm
# Inside the container
cd /workspace
colcon build
```
### x86 Compilation Process
```bash
./start_container.sh
# Inside the container
cd /workspace
colcon build
```

### Notes
- Without parameters, the script will start the compilation environment for the current architecture by default.
- On an x86 environment, using the `arm` parameter can utilize qemu to simulate starting the ARM image for cross-compilation. However, due to the overhead of the emulator, performance in this environment is relatively poor.

## 6. Troubleshooting

Common issues and solutions:

- **Slow image download:**
```bash
# Manually download and place in the script directory
wget https://cdn.robosense.cn/arm_sdk_compile_humble_based.tgz
wget https://cdn.robosense.cn/x86_sdk_compile_humble_based.tgz
```

- **Docker not found:**
Ensure Docker is installed and running
Add your user to the Docker group

- **Permission denied:**
Run with sudo if needed
Check user group membership

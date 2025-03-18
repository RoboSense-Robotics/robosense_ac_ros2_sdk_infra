# ros2_codec

[中文文档](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ros2_codec/README_CN.md)

## 1. Introduction

ros2_codec is the ROS middleware node for the codec, which is used to decode h265 video data.

## 2. Installation

### 2.1 Build (Linux + ROS 2)

Ensure you have a `ROS 2` distribution installed. This project has been developed and tested on `ROS 2 Humble`.

With your `ROS 2` environment ready, clone the repository into your workspace using the following commands:

```bash
# Using ssh
git clone git@github.com:RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
# Using http
git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
```

Then, enter the modules/ros2_codec directory, Run the following commands to compile:

```bash
colcon build
```
## 3. Usage

### 3.1 Prepare the ros2_codec environment
Refresh the bash profile of the workspace to ensure that the environment configuration of the components is ok.
Run the following commands:
```bash
source install/setup.bash
```

### 3.2 Run the ros2_codec Node
The ros2_codec node can be run using the ros2 run command.
```bash
ros2 run ros2_codec codec_node
```


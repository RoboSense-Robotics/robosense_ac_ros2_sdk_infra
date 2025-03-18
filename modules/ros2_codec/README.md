# ros2_codec

[README](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ros2_codec/README.md) | [中文文档](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/r2_codec/README_CN.md)

## 1. Introduction

ros2_codec is the ROS middleware node for the codec, which is used to decode h265 video data.

## 2. Installation

### 2.1 Build (Linux + ROS 2)

Ensure you have a `ROS 2` distribution installed. This project has been developed and tested on `ROS 2 Humble`.

With your `ROS 2` environment ready, clone the repository into your workspace using the following commands:

```bash
# Using ssh
git clone git@gitlab.robosense.cn:super_sensor_sdk/ros2_sdk/sdk_infra.git
# Using http
git clone http://gitlab.robosense.cn/super_sensor_sdk/ros2_sdk/sdk_infra.git
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
## 4. FAQ

For further details and troubleshooting, refer to the FAQ section.

[Create New Issue](http://gitlab.robosense.cn/super_sensor_sdk/sdk_middleware/issues/new)


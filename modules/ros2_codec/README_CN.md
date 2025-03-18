# ros2_codec

[README](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ros2_codec/README.md) | [中文文档](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ros2_codec/README_CN.md)

## 1. 简介

ros2_codec是解码H265的ROS中间件节点,用于接收H265的数据，解码成RGB数据发布出去.

## 2. 构建

### 2.1 编译 (Linux + ROS 2)

确保ROS2的编译环境已安装，整个工程的开发和测试基于的是ROS 2 Humble56版本。

ROS 2环境准备好后，使用以下命令将代码下载到工作目录下。

```bash
# Using ssh
git clone git@gitlab.robosense.cn:super_sensor_sdk/ros2_sdk/sdk_infra.git
# Using http
git clone http://gitlab.robosense.cn/super_sensor_sdk/ros2_sdk/sdk_infra.git
```

然后进入modules/ros2_codec目录, 使用以下命令进行编译:

```bash
colcon build
```
## 3. 运行

### 3.1 准备环境

刷新工作目录下的bash配置文件，确保组件的配置是完整的。
使用以下命令刷新:

```bash
source install/setup.bash
```

### 3.2 运行ros2_codec节点
使用以下命令运行ros2_codec节点

```bash
ros2 run ros2_codec codec_node
```
## 4 FAQ

更详细的内容及问题，请访问FAQ章节

[Create New Issue](http://gitlab.robosense.cn/super_sensor_sdk/sdk_middleware/issues/new)


# ros2_metas

[README](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ros_metas/README.md)

## 1. 简介

ros2_metas是MetaS传感器驱动的ROS中间件节点,用于接收传感器数据，整合和发布给其它节点使用。传感器数据包括摄像头，激光雷达和IMU。

## 2. 构建

### 2.1 编译 (Linux + ROS 2)

确保ROS2的编译环境已安装，整个工程的开发和测试基于的是ROS 2 Humble版本，[ROS2 Humble官方安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

ROS 2环境准备好后，使用以下命令将代码下载到工作目录下。

```bash
# Using ssh
git clone git@github.com:RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
# Using http
git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
```
Then, prepare system build environment base on Radxa and X86.
然后进行系统编译依赖的准备操作，如下基于Radxa开发板及X86环境.

#### 2.1.1 Radxa开发板
参考:https://docs.radxa.com/rock5/rock5b/app-development/rtsp?target=ffmpeg
安装ffmpeg-rockchip依赖库,执行以下命令:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libdrm-dev librga-dev librockchip-mpp-dev libsdl2*-dev libx264-dev libx265-dev pkg-config
```
注意如果无法访问源安装依赖库，请下载源码安装，参考:https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Compilation

安装ffmpeg-rockchip:

```bash
git clone https://github.com/nyanmisaka/ffmpeg-rockchip
pushd ffmpeg-rockchip/
./configure --prefix=/usr --enable-gpl --enable-version3 --enable-libdrm --enable-rkmpp --enable-rkrga --enable-libx264 --enable-libx265 --enable-ffplay
make -j$(nproc)
sudo make install
popd
```
在安装完成后，执行以下命令配置系统依赖库环境:

```bash
sudo ln -s /usr/lib/aarch64-linux-gnu/librga.so.2.1.0 /usr/lib/aarch64-linux-gnu/librga.so
sudo ln -s /usr/lib/aarch64-linux-gnu/libdrm.so.2.123.0 /usr/lib/aarch64-linux-gnu/libdrm.so
sudo rm /usr/lib/aarch64-linux-gnu/libavformat.*
sudo rm /usr/lib/aarch64-linux-gnu/libavutil.*
sudo rm /usr/lib/aarch64-linux-gnu/libswscale.*
sudo rm /usr/lib/aarch64-linux-gnu/libpostproc.*
sudo rm /usr/lib/aarch64-linux-gnu/libavdevice.*
sudo rm /usr/lib/aarch64-linux-gnu/libswresample.*
sudo rm /usr/lib/aarch64-linux-gnu/libavfilter.*
sudo rm /usr/lib/aarch64-linux-gnu/libavcodec*
```
#### 2.1.2 X86
安装依赖库,如下示例:
```bash
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

#### 2.1.3 编译
然后进入modules目录, 使用以下命令进行编译:

```bash
# 全量编译
colcon build

#或者单独编译
colcon build --symlink-install --packages-select robosense_msgs
colcon build --symlink-install --packages-select ros2_codec
colcon build --symlink-install --packages-select ros_metas
```
## 3. 运行

### 3.1 准备环境

刷新工作目录下的bash配置文件，确保组件的配置是完整的。
使用以下命令刷新:

```bash
source install/setup.bash
```

### 3.2 运行ros_metas节点
使用以下命令运行ros_metas节点

```bash
ros2 run metas_ros ms_node
```
### 3.3 查看发布的传感器数据

#### 3.3.1 通过界面来查看传感器数据

可以使用带界面的工具，如rviz2查看传感器输出的数据，以下是rviz2的安装步骤:

1. 安装rviz:
```bash
sudo apt-get install ros-<ros2-distro>-rviz2
```
2. 运行rviz:
```bash
rviz2
```
3. 配置rviz:
在rviz界面，需要增加特定显示的传感器节点，如下:
图像数据增加Image节点.
激光雷达数据选择PointCloud2.
IMU数据选择IMU.

4. 选择Topic:
选择ros2_metas节点发布的topic.

参照4.2 Topic章节选择topic显示，可使rviz显示ros2_metas节点发布的传感器数据.

#### 3.3.2 录制数据并查看
可通过ROS2的录制工具进行数据录制并回放查看数据，按以下步骤操作:

1. 录制
使用ros2 bag record命令进行录制:
录制所有topic可使用以下命令:
```bash
ros2 bag record -a
```

录制特定topic可使用以下命令:
```bash
ros2 bag record /topic1 /topic2
```
2. 回放
录制完成后，可以通过以下命令进行回放:
sh
```bash
ros2 bag play <bagfile>
```
替换<bagfile>为相应录制的路径文件

3. 查看数据:
可通过一些工具进行查看，例如rviz
对于录制和回放的更详细内容可参照ROS2文档的录制回放章节.


## 4. 特性
### 4.1  依赖
ros2_metas节点依赖以下关键的库和软件包:

#### 4.1.1 ROS2 Core 库:
* rclcpp: ROS2 C++ 客户端库, 提供ROS2的核心功能.
* sensor_msgs: 传感器的标准数据消息，包括images and point clouds.
* std_msgs: ROS2的标准消息.
#### 4.1.2 robosense_msgs:
为H.265定制的ROS2消息，用于传输metaS传感器的图像数据。
### 4.2 Topic 
1. camera RGB image topic:/rs_camera/rgb
2. lidar topic:/rs_lidar/points
3. imu topic:/rs_imu
4. camera h265 video topic:/rs_camera/compressed


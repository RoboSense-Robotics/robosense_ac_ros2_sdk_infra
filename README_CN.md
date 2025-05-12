# **[robosense_ac_ros2_sdk_infra](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra)**  

[README](./README.md)

## 1. 简介

​	robosense_ac_ros2_sdk_infra工程项目是基于ROS2的包工程，该仓作为AC1传感器的ROS2基础软件仓，包含AC1驱动、相机消息的压缩与解压缩功能以及ROS2 docker镜像的使用工具。

- AC1 ROS2驱动

用于接收传感器数据，整合和发布给其它节点使用。传感器数据包括摄像头，激光雷达和IMU，详情请查看当前目录下[modules/ac_driver/README_CN.md](modules/ac_driver/README_CN.md)。

- ros2_codec相机消息编解码

ros2_codec是解码H265的ROS中间件节点,用于接收H265的数据，解码成RGB数据发布出去，详情请查看当前目录下[modules/ac_codec/README_CN.md](modules/ac_codec/README_CN.md)。

- ROS2 docker镜像

该工具用于管理 Super Sensor SDK 的跨平台编译和本地编译环境的 Docker 容器（仅支持ROS 2 Humble）。它包含了容器管理、镜像管理以及自动化环境设置等功能，详情请查看当前目录下[tools/compilation_envirment/README_CN.md](/tools/compilation_envirment/README_CN.md)。


## 2. 前置依赖

- Ubuntu 20.04 (其他操作系统版本，需要适配)
- 操作系统匹配的ROS2版本即可，如ros2 humble安装，[ROS2 Humble官方安装文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)


## 3. 驱动安装和硬件连接

### 3.1 驱动安装(只需要安装一次) 

在当前工程目录下的scripts文件夹打开新的终端，安装如下图所示的命令执行脚本，执行安装驱动脚本，

```shell
# 进入源码下的脚本目录
cd /codepath/src/ac_driver/scripts
# 首次接入新设备执行（只需要一次）
sudo bash AC_usb_permission.sh
```

等待安装，安装完成后，重启电脑即可。 

### 3.2 硬件连接

由于Robosense提供的设备数据传输量较大，所以要求必须使用USB3.0接入，连接后，可以使用usbview 工具查看进行进一步确认。

## 4. 编译ac_driver

- 步骤1: 打开新的终端，将终端路径切换到源码根目录

- 步骤2:  执行如下编译命令

  ```shell
  # 设置已安装ros2 humble环境变量
  source /opt/ros/humble/setup.bash
  # 进入源码根目录
  cd /codepath
  colcon build
  ```

## 5. 运行ac_driver

- 步骤1: 对于编译好的包，执行source命令设置节点运行环境:

  ```shell
  source install/setup.bash
  ```

- 步骤2: 启动节点: 

  ```sh
  ros2 run ac_driver ms_node [--ros-args --param image_input_fps:=30 --param imu_input_fps:=200 --param enable_jpeg:=false]
  或 
  ros2 launch ac_driver start.py 
  ```



## 7. 话题名称及消息类型

- /rs_camera/rgb         ->  sensor_msgs::Image 
- /rs_lidar/points         -> sensor_msgs::PointCloud2, 其中点云的frame_id为"/rslidar"
- /rs_imu                      -> sensor_msgs::Imu  

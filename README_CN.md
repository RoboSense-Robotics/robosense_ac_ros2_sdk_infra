# sdk_infra

该仓作为AC1传感器的ROS2基础软件仓，包含AC1驱动、相机消息的压缩与解压缩功能以及ROS2 docker镜像的使用工具。

- AC1 ROS2驱动

用于接收传感器数据，整合和发布给其它节点使用。传感器数据包括摄像头，激光雷达和IMU，详情请查看当前目录下[modules/ros_metas/README_CN.md](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ros_metas/README_CN.md)。



- ros2_codec相机消息编解码

ros2_codec是解码H265的ROS中间件节点,用于接收H265的数据，解码成RGB数据发布出去，详情请查看当前目录下[modules/ros2_codec/README_CN.md](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ros2_codec/README_CN.md)。



- ROS2 docker镜像

该工具用于管理 Super Sensor SDK 的跨平台编译和本地编译环境的 Docker 容器（仅支持ROS 2 Humble）。它包含了容器管理、镜像管理以及自动化环境设置等功能，详情请查看当前目录下[tools/compilation_envirment/README_CN.md](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/tools/compilation_envirment/README_CN.md)。


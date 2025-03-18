# sdk_infra

It serves as the basic ROS2 software repository for the AC1 sensor and includes AC1 drivers, camera message compression and decompression functions, and tools for using ROS2 docker images.



- AC1 ROS2 driver

It is used to receive sensor data, integrate and publish it to other nodes. Sensor data includes camera, LiDAR, and IMU. For details, see "[modules/ros_metas/README.md](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ros_metas/README.md)".



- ros2_codec camera message codec

ros2_codec is the ROS middleware node that decodes H265, which is used to receive H265 data and decode it into RGB data for release. For details, please check "[modules/ros2_codec/README.md](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ros2_codec/README.md)" in the current directory.



- ROS2 docker image

This tool is used to manage Docker containers for cross-platform compilation and native compilation environments of the Super Sensor SDK (ROS 2 Humble only). It includes container management, image management, and automatic environment setting. For details, see "[tools/compilation_envirment/README.md](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/tools/compilation_envirment/README.md)" in the current directory.

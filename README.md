# [robosense_ac_ros2_sdk_infra](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra)

[中文文档](README_CN.md)

## 1. Overview

​	The robosense_ac_ros2_sdk_infra project is a package project based on ROS2. This warehouse is the basic ROS2 software warehouse of AC1 sensor, including AC1 driver, camera message compression and decompression functions, and ROS2 docker image using tools.

- AC1 ROS2 drive

It is used to receive sensor data, integrate and publish it to other nodes. Sensor data includes camera, LiDAR, and IMU. For details, see the current directory [modules/ac_driver/README.md](modules/ac_driver/README.md).

- ros2_codec camera message codec

ros2_codec is the ROS middleware node decoding H265, which is used to receive H265 data, decode it into RGB data and publish it. For details, see [modules/ac_codec/README.md](modules/ac_codec/README.md).

- ROS2 docker image

This tool is used to manage Docker containers for cross-platform compilation and native compilation environments of the Super Sensor SDK (ROS 2 Humble only). It includes container management, image management and automatic environment Settings and other functions. For details, view [tools/compilation_envirment/README.md](/tools/compilation_envirment/README.md) in the current directory.

## 2. Prerequisites 

- Ubuntu20.04 

- ROS2 version of the operating system matching, such as ROS2 humble installation, [ROS2 humble official installation document](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)


## 3. Drive Install And Hardware Connect 

### 3.1 Drive Install(Only Need Once)  

Open a new terminal in the scripts folder in the current project directory, install the command execution script as shown in the following figure, and run the installation driver script.

```shell
# Go to the script directory under source code
cd /codepath/src/ac_driver/scripts
# First access to a new device execution (only once required)
sudo bash AC_usb_permission.sh
```

Wait for the installation, and restart the computer.

### 3.2 Hardware Connect 

Due to the large amount of data transmission of the device provided by Robosense, it is required to use USB3.0 access. 

## 4. Build 

- Step 1: Open a new terminal and switch the terminal path to the source root directory

- Step2: Run ros build command 

  ```shell
  # Set the ros2 humble environment variable installed
  source /opt/ros/humble/setup.bash
  # Go to the source code root directory
  cd /codepath
  colcon build
  ```

## 5. Run 

- Step1: Setup the node's environment by source command: 

  ```shell
  source install/setup.bash
  ```

- Step2: Run node: 

  ```shell
  ros2 run ac_driver ms_node [--ros-args --param image_input_fps:=30 --param imu_input_fps:=200 --param enable_jpeg:=false]
  或 
  ros2 launch ac_driver start.py 
  ```


## 7. Topic Name And Data Type 

- /rs_camera/rgb     ->  sensor_msgs::Image 
- /rs_lidar/points     -> sensor_msgs::PointCloud2, the pointcloud frame_id is **"/rslidar"**
- /rs_imu                  -> sensor_msgs::Imu  

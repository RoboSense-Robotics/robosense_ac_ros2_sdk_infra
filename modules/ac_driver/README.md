# ac_driver

[中文文档](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ac_driver/README_CN.md)

## 1. Introduction

ac_driver is the ROS middleware node for the AC driver, which is used to receive sensor data from AC, integrate the data, and then publish it for use by other nodes. This includes data from three sources: camera, lidar, and IMU.

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
Then, prepare system build environment base on Radxa and X86.

#### 2.1.1 Radxa Development Board

install ffmpeg-rockchip depend libs, refer to https://docs.radxa.com/rock5/rock5b/app-development/rtsp?target=ffmpeg:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libdrm-dev librga-dev librockchip-mpp-dev libsdl2*-dev libx264-dev libx265-dev pkg-config
```
If you cannot access the apt source list to install the dependency libraries, please download the source code and install it. For reference, https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Compilation

install ffmpeg-rockchip:
```bash
git clone https://github.com/nyanmisaka/ffmpeg-rockchip
pushd ffmpeg-rockchip/
./configure --prefix=/usr --enable-gpl --enable-version3 --enable-libdrm --enable-rkmpp --enable-rkrga --enable-libx264 --enable-libx265 --enable-ffplay --enable-alsa --enable-sndio --enable-x11grab --enable-xv --extra-libs="-lasound -lx264 -lsndio -lX11 -lXv" --extra-cflags="-I/usr/include/alsa -I/usr/local/include" --extra-ldflags="-L/usr/lib/aarch64-linux-gnu -L/usr/local/lib"
make -j$(nproc)
sudo make install
popd
```

After the installation is complete, execute the following command to configure the system dependency library environment:
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
#### 2.1.2 X86 Board
install dependency library, for example:
```bash
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

#### 2.1.3 Jetson Orin Platform

When building and running on the Jetson Orin platform, install dependency library, for example:

```shell
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

ensure that the CUDA environment is properly installed. Follow these steps:

1. Install the necessary CUDA libraries:
```bash
sudo apt-get update
sudo apt-get install nvidia-cuda-toolkit
```

2. Verify the CUDA installation:
```bash
nvcc --version
```

3. Ensure that ROS2 and OpenCV dependencies are installed.

4. Follow the instructions in section 2.1.3 to build the project.

#### 2.1.4 Build
Then, enter the modules directory, Run the following commands to compile:

```bash
# Full build
colcon build

# Or build individually
colcon build --symlink-install --packages-select robosense_msgs
colcon build --symlink-install --packages-select ac_codec
colcon build --symlink-install --packages-select ac_driver
```

## 3. Usage

### 3.1 Prepare the ac_driver environment
Refresh the bash profile of the workspace to ensure that the environment configuration of the components is ok.
Run the following commands:
```bash
source install/setup.bash
```

**Note:** Before starting the driver, ensure that the `ROS_DOMAIN_ID` is set correctly. Failure to set this may result in abnormal driver processes or data publishing delays. You can set it as follows:
```bash
export ROS_DOMAIN_ID=<your_domain_id>
```
Replace `<your_domain_id>` with the appropriate domain ID for your ROS 2 environment.

### 3.2 Run the ac_driver Node
The ac_driver node can be run using the ros2 run command.
1. Non-zero-copy mode
```bash
ros2 run ac_driver ms_node [--ros-args --param image_input_fps:=30 --param imu_input_fps:=200 --param enable_jpeg:=false --param jpeg_quality:=70]
or 
ros2 launch ac_driver start.py 
```
2. Zero-copy mode (only for ROS2 Humble)
```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=ac_driver/conf/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
ros2 run ac_driver ms_node [--ros-args --param image_input_fps:=30 --param imu_input_fps:=200 --param enable_jpeg:=false --param jpeg_quality:=70]
or 
export FASTRTPS_DEFAULT_PROFILES_FILE=ac_driver/conf/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
ros2 launch ac_driver start.py
```

#### Parameter Description
- `image_input_fps`: The image sensor output frame rates, where the frame rates supported by images include: 10Hz/15Hz/30Hz. The default value is 30
  
- `imu_input_fps`:  The imu sensor output frame rates, where the frame rates supported by Imu include: 100Hz/200Hz. The default value is 200
  
- `enable_jpeg`: Whether to enable JPEG image compression. The default value is `false` (disabled). When enabled, it compresses images using JPEG to reduce bandwidth usage but increases CPU usage. You can enable it as follows:
  
  ```shell
  ros2 run ac_driver ms_node [--ros-args --param enable_jpeg:=true]
  ```
  
- `jpeg_quality`: The quality of JPEG image compression, default is 70

Depending on the method of starting the node, as mentioned in 1/2 above, if starting through the **ros2 run** command, parameters can be passed in through the **-- param** command; If using the **ros2 launch** startup command, modify the startup parameter settings in the **start.launch** file.

#### Format Description

- At present, image transcoding optimization has been done on rk3588 and jetson orin platforms, and the default driver is directly exported to nv12 and released into rgb24 format images through hardware;

- Other platforms use rgb24 by default and adopt cpu jepg compression. The compression function considers the impact of performance. The compression function is disabled by default and can be turned on by enable-jpeg switch.

### 3.3 View the published sensor data.

#### 3.3.1 View Published Sensor Data Through a User Interface

To view the published sensor data via a graphical interface, you can use tools like rviz2 in ROS2. Here are the steps:
1. Install rviz:
Ensure that rviz is installed:
```bash
sudo apt-get install ros-<ros2-distro>-rviz2
```
2. Launch rviz:
Start rviz to visualize the sensor data:
```bash
rviz2
```
3. Configure rviz:
In the rviz interface, add the necessary displays to visualize different types of sensor data:
For image data, add the Image display.
For point cloud data from lidar, add the PointCloud2 display.
For IMU data, add the IMU display.

4. Select Topics:
Configure the displays to subscribe to the appropriate topics published by the ac_driver node.

By following these steps, you can view the published sensor data from the ac_driver node using graphical interfaces like rviz.

#### 3.3.2 Recording and Viewing Data
You can use the built-in ROS2 bag recording tool to record and then play back the data for viewing. Here are the steps:

1. Record Data:
Use the ros2 bag record command to record data from specific topics. For example, to record data from all topics, you can use:
```bash
ros2 bag record -a
```

To record data from specific topics, specify the topic names:
```bash
ros2 bag record /topic1 /topic2
```
2. Play Back Data:
Once the recording is complete, you can play back the recorded data using the ros2 bag play command:
```bash
ros2 bag play <bagfile>
```
Replace <bagfile> with the path to your recorded bag file.

3. View Data:
While playing back the data, you can use tools like rviz to view the data.
For more detailed instructions on recording and playing back data, you can refer to the ROS2 documentation on recording and playing back data .


## 4. Features
### 4.1  Dependencies
The ac_driver node relies on several key libraries and packages to function properly. Here is a detailed list of the dependencies:

#### 4.1.1 ROS2 Core Libraries:
* rclcpp: The ROS2 C++ client library, providing the core functionality for ROS2 nodes.
* sensor_msgs: Provides standard message types for common sensor data, such as images and point clouds.
* std_msgs: Provides standard message types for basic data types, such as integers, floats, and strings.
#### 4.1.2 robosense_msgs:
This custom ROS2 package defines the message formats for H.265 compressed images and other sensor data specific to the AC sensors. It is essential for the ac_driver node to interpret and publish the sensor data correctly.

### 4.2 Topic 
1. camera RGB image topic:/rs_camera/rgb
   * Zero-copy mode msg (custom): robosense_msgs/msg/RsImage
   * Non-zero-copy mode msg (ROS2 standard): sensor_msgs/msg/Image
2. lidar topic:/rs_lidar/points
   * Zero-copy mode msg (custom): robosense_msgs/msg/RsPointCloud
   * Non-zero-copy mode msg (ROS2 standard): sensor_msgs/msg/PointCloud2
3. imu topic:/rs_imu
4. camera h265 video topic:/rs_camera/compressed

## 5. Limitations

### 5.1 Zero-Copy Usage Limitations

Compared to the ROS2 publisher/subscriber data transmission method, using zero-copy transmission has the following limitations:
* Currently only supports the Humble version. It is recommended to use RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT for QOS Reliability (it is recommended to directly use `rclcpp::SensorDataQoS()` to set QOS).
* QOS History only supports KEEPLAST, does not support KEEPALL, and KEEPLAST cannot be set too large due to memory limitations. Currently, it is set to a maximum of 256MB.
* The size of the transmitted message is fixed, i.e., the `sizeof` value of the message does not change. It cannot contain variable-length data types, such as strings or dynamic arrays.
* RMW_QOS_POLICY_RELIABILITY_RELIABLE has stability issues under multiple communication methods.
* Can only be used for inter-process communication on the same device and cannot be transmitted across devices.
* The publisher message must be obtained first and then assigned before sending. It must also check whether it was successfully obtained.
* The message received by the subscriber is only valid within the callback function and cannot be used outside the callback function.

### 5.2 Performance

* For images at 30fps, the CPU usage on Orin Nano is measured at 105%. If JPEG compression is disabled, CPU usage can be reduced by 40-50%. Disable it as needed by commenting out the following code:
```cpp
{
    std::lock_guard<std::mutex> lock(jpeg_mutex_);
    jpeg_queue_.push(msgPtr);
    jpeg_condition_.notify_one();
}
```



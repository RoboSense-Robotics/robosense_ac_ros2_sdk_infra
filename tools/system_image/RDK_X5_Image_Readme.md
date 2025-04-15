#  RDK X5  Development Guide

[RDK_X5_Image_Readme](RDK_X5_Image_Readme.md) | [中文文档](RDK_X5_Image_Readme_CN.md)

## 1. System Image Download and Flashing  
https://developer.d-robotics.cc/rdk_doc/Quick_start/install_os/rdk_x5  

## 2. Installing colcon and PCL Dependencies  
```bash
sudo apt install python3-colcon-common-extensions ros-humble-pcl-conversions
```

### 2.3 Robosense SDK Usage  

1. **Create a Workspace and Clone the Repository**:  
   ```bash
   mkdir -p ac_studio/src && cd ac_studio
   git clone https://github.com/RoboSense-Robotics/robosense_ac_studio.git
   vcs import src < ac_studio/ac_studio_ssh.repos
   ```

2. **Build All Modules**:  
   ```bash
   colcon build
   ```

3. **Build a Specific Module**:  
   ```bash
   colcon build --symlink-install --packages-select ac_codec
   ```
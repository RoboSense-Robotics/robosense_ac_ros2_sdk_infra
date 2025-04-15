# RDK X5 开发指南

[RDK_X5_Image_Readme](RDK_X5_Image_Readme.md) | [中文文档](RDK_X5_Image_Readme_CN.md)

## 1. 系统镜像下载与烧录
https://developer.d-robotics.cc/rdk_doc/Quick_start/install_os/rdk_x5


## 2. 安装colcon与pcl依赖库
```bash
sudo apt install python3-colcon-common-extensions ros-humble-pcl-conversions
```

### 2.3 Robosense SDK 使用

1. **创建工作空间并拉取代码**：
   ```bash
   mkdir -p ac_studio/src && cd ac_studio
   git clone https://github.com/RoboSense-Robotics/robosense_ac_studio.git
   vcs import src < ac_studio/ac_studio_ssh.repos
   ```

2. **编译所有模块**：
   ```bash
   colcon build
   ```

3. **编译单个模块**：
   ```bash
   colcon build --symlink-install --packages-select ac_codec
   ```
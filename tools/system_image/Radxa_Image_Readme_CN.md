# Radxa 开发指南

[Radxa_Image_Readme](Radxa_Image_Readme.md) | [中文文档](Radxa_Image_Readme_CN.md)

## 1. 系统镜像下载与解压
**下载地址**：  [rock-5b-plus_bookworm_kde.tar.gz](https://cdn.robosense.cn/AC_wiki/rock-5b-plus_bookworm_kde.tar.gz)

**解压命令**：
```bash
tar -zxvpf rock-5b-plus_bookworm_kde.tar.gz
```

---

## 2. 镜像烧录与 ROS2 环境使用

### 2.1 镜像烧录
**参考文档**：[Radxa Maskrom 烧录教程](https://docs.radxa.com/rock5/rock5b/low-level-dev/maskrom)  

**Ubuntu 采用 `rkdeveloptool`，烧录命令如下：**

1. **进入 Maskrom 烧录模式**：
   - 插上 **Type-C 烧录接口**
   - 按住 **Maskrom 按键**
  
2. **烧录 Loader**：
   ```bash
   sudo rkdeveloptool db rk3588_spl_loader_v1.15.113.bin
   ```


3. **烧录系统镜像**：
   ```bash
   sudo rkdeveloptool wl 0 rock-5b-plus_bookworm_kde.img
   ```

4. **重启设备**：
   ```bash
   sudo rkdeveloptool rd
   ```

---

### 2.2 ROS2 环境使用
```bash
source /opt/ros2_humble/setup.bash
```

---

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
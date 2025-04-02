# Radxa Development Board Guide  

[Radxa_Image_Readme](Radxa_Image_Readme.md) | [中文文档](Radxa_Image_Readme_CN.md)

## 1. Download and Extract the System Image  

**Download Link:**  [rock-5b-plus_bookworm_kde.tar.gz](https://cdn.robosense.cn/AC_wiki/rock-5b-plus_bookworm_kde.tar.gz)  

**Extract the Image:**  
```bash
tar -zxvpf rock-5b-plus_bookworm_kde.tar.gz
```  

---

## 2. Flash the Image and Use the ROS2 Environment  

### 2.1 Flashing the Image  

**Reference:** [Radxa Maskrom Flashing Guide](https://docs.radxa.com/rock5/rock5b/low-level-dev/maskrom)  

**Ubuntu uses `rkdeveloptool`, and the flashing commands are as follows:**  

1. **Enter Maskrom Mode:**  
   - Connect the **Type-C flashing port**  
   - Press and hold the **Maskrom button**  

2. **Flash the Loader:**  
   ```bash
   sudo rkdeveloptool db rk3588_spl_loader_v1.15.113.bin
   ```

3. **Flash the System Image:**  
   ```bash
   sudo rkdeveloptool wl 0 rock-5b-plus_bookworm_kde.img
   ```

4. **Reboot the Device:**  
   ```bash
   sudo rkdeveloptool rd
   ```

---

### 2.2 Using the ROS2 Environment  

```bash
source /opt/ros2_humble/setup.bash
```

---

### 2.3 Using the Robosense SDK  

1. **Create a workspace and clone the repository:**  
   ```bash
   mkdir -p ac_studio/src && cd ac_studio
   git clone https://github.com/RoboSense-Robotics/robosense_ac_studio.git
   vcs import src < ac_studio/ac_studio_ssh.repos
   ```

2. **Build all modules:**  
   ```bash
   colcon build
   ```

3. **Build a single module:**  
   ```bash
   colcon build --symlink-install --packages-select ac_codec
   ```
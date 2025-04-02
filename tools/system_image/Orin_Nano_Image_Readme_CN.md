# Jetson Orin Nano 开发指南

[Orin_Nano_Image_Readme](Orin_Nano_Image_Readme.md) | [中文文档](Orin_Nano_Image_Readme_CN.md)


### **Jetson Orin Nano 开发板使用指南**

#### **官方文档**  
[Jetson Orin Nano 官方入门文档](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)

---

### **Key Points**
1. **JetPack 6.1 (rev.1)** —— 2024 年 12 月后更新的固件，性能提升 **70%**  
2. **电源要求** —— 19V，2.37A 输入  

---

## **1. 安装系统镜像**
### **使用 [SDK Manager](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html)** 安装到 SSD
安装参考：https://www.jetson-ai-lab.com/initial_setup_jon_sdkm.html

> 如果在刷写过程中遇到 USB 连接问题，请参考官方 FAQ:  
> [Jetson AGX Orin FAQ](https://forums.developer.nvidia.com/t/jetson-agx-orin-faq/237459)

**解决 Flash 时报 USB 连接问题：**
```bash
sudo -s
echo -1 > /sys/module/usbcore/parameters/autosuspend
```

---

## **2. 安装 JetPack**
```bash
sudo apt install nvidia-jetpack -y
```

### **设置环境变量**
```bash
vim ~/.bashrc
```
在 `.bashrc` 文件末尾添加：
```bash
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export PATH=/usr/src/tensorrt/bin:$PATH
```
保存后，使其生效：
```bash
source ~/.bashrc
```

---

## **3. 安装 ROS 2 Humble**
> 由于 Jetson Orin Nano 运行 Ubuntu 22.04，可以直接通过 `apt` 安装 ROS 2 Humble。

### **3.1. 设置系统源**
先更新系统：
```bash
sudo apt update && sudo apt upgrade -y
```
添加 ROS 2 官方软件源：
```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
```

### **3.2. 添加 ROS 2 GPG 密钥**
```bash
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### **3.3. 添加 ROS 2 软件源**
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
更新软件包：
```bash
sudo apt update
```

### **3.4. 安装 ROS 2**
安装完整桌面版（包含 RViz、rosbag、demo 等）：
```bash
sudo apt install ros-humble-desktop -y
```
如果只需要基础版：
```bash
sudo apt install ros-humble-ros-base -y
```

### **3.5. 配置环境变量**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
或者每次使用时手动运行：
```bash
source /opt/ros/humble/setup.bash
```

### **3.6. 安装 `colcon` 构建工具**
```bash
sudo apt install python3-colcon-common-extensions -y
```
启用 `colcon` 命令自动补全：
```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

### **3.7. 测试 ROS 2**
检查 ROS 2 是否安装成功：
```bash
ros2 --version
```

运行 `talker` 和 `listener` 测试：
#### **启动 `talker`（发布消息）**
```bash
ros2 run demo_nodes_cpp talker
```
#### **启动 `listener`（订阅消息）**
打开另一个终端：
```bash
ros2 run demo_nodes_cpp listener
```
如果两个终端都打印出消息，说明 ROS 2 Humble 安装成功！🎉

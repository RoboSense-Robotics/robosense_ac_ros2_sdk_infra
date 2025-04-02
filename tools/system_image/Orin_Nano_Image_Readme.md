# **Jetson Orin Nano Development Board Guide**  

[Orin_Nano_Image_Readme](Orin_Nano_Image_Readme.md) | [中文文档](Orin_Nano_Image_Readme_CN.md)

#### **Official Documentation**  
[Jetson Orin Nano Official Getting Started Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)  

---

### **Key Points**  
1. **JetPack 6.1 (rev.1)** — Firmware update after **December 2024** provides a **70% performance boost**.  
2. **Power Requirement** — 19V, 2.37A input.  

---

## **1. Installing the System Image**  
### **Install to SSD Using [SDK Manager](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html)**  
Installation reference: https://www.jetson-ai-lab.com/initial_setup_jon_sdkm.html
> If you encounter USB connection issues during flashing, refer to the official FAQ:  
> [Jetson AGX Orin FAQ](https://forums.developer.nvidia.com/t/jetson-agx-orin-faq/237459)  

**Fixing USB Connection Issues During Flashing:**
```bash
sudo -s
echo -1 > /sys/module/usbcore/parameters/autosuspend
```

---

## **2. Installing JetPack**  
```bash
sudo apt install nvidia-jetpack -y
```

### **Set Environment Variables**  
```bash
vim ~/.bashrc
```
Append the following lines to `.bashrc`:
```bash
export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
export PATH=/usr/src/tensorrt/bin:$PATH
```
Apply the changes:
```bash
source ~/.bashrc
```

---

## **3. Installing ROS 2 Humble**  
> Since Jetson Orin Nano runs Ubuntu 22.04, you can directly install ROS 2 Humble using `apt`.

### **3.1. Set Up System Sources**  
Update the system:
```bash
sudo apt update && sudo apt upgrade -y
```
Add the official ROS 2 software source:
```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update
```

### **3.2. Add ROS 2 GPG Key**  
```bash
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### **3.3. Add ROS 2 Repository**  
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
Update the package list:
```bash
sudo apt update
```

### **3.4. Install ROS 2**  
Install the full desktop version (includes RViz, rosbag, demos, etc.):
```bash
sudo apt install ros-humble-desktop -y
```
If you only need the basic version:
```bash
sudo apt install ros-humble-ros-base -y
```

### **3.5. Configure Environment Variables**  
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Alternatively, manually source it before use:
```bash
source /opt/ros/humble/setup.bash
```

### **3.6. Install `colcon` Build Tool**  
```bash
sudo apt install python3-colcon-common-extensions -y
```
Enable command auto-completion for `colcon`:
```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

### **3.7. Test ROS 2 Installation**  
Check if ROS 2 is installed successfully:
```bash
ros2 --version
```

Run `talker` and `listener` test nodes:  
#### **Start `talker` (Publishes messages)**  
```bash
ros2 run demo_nodes_cpp talker
```
#### **Start `listener` (Subscribes to messages)**  
Open another terminal and run:
```bash
ros2 run demo_nodes_cpp listener
```
If both terminals print messages, **ROS 2 Humble is successfully installed! 🎉**


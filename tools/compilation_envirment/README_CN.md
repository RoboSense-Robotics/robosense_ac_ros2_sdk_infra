# Super Sensor SDK Docker 环境管理工具

[README](README.md) | [中文文档](README_zh.md)

## 1. 简介

该工具用于管理 Super Sensor SDK 的跨平台编译和本地编译环境的 Docker 容器（仅支持ROS 2 Humble）。它包含了容器管理、镜像管理以及自动化环境设置等功能。

### 主要特性
- **跨平台编译环境** (ARM64架构支持)
- **本地编译环境** (Ubuntu 22.04 + ROS 2 Humble)
- 自动化的 Docker 镜像管理
- 容器生命周期管理
- 用户权限映射
- 开发所需的卷挂载

## 2. 前置依赖

### 2.1 安装 Docker
根据您的操作系统选择 [Docker 官方安装指南](https://docs.docker.com/engine/install/) 中的对应内容进行安装。

### 2.2 安装多平台支持
```bash
# 安装qemu-user-static（交叉编译必需）
sudo apt-get install qemu-user-static
# 启用binfmt支持
sudo docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

## 3. 安装部署

### 3.1 Docker 镜像
工具使用以下预构建镜像：

ARM编译镜像：​ arm_sdk_compile_humble_based:latest
​x86编译镜像：​ x86_sdk_compile_humble_based:latest

### 3.2 镜像管理

首次使用时脚本会自动：

从CDN下载镜像包（约3GB）
解压并加载Docker镜像
验证镜像完整性

## 4. 使用方式

### 4.1 容器管理

使用 `start_container.sh` 脚本管理容器：

```bash
cd src/ac_studio/robosense_ac_ros2_sdk_infra/tools/compilation_envirment
./start_container.sh [选项]
```

可用选项：
- `arm`: 启动ARM编译环境
- `stop`: 停止所有SDK编译容器
- `status`: 显示容器状态
- `help`: 显示帮助信息

### 4.2 卷挂载

容器中挂载以下路径：
- 项目工作空间: `$(pwd)/../../../../..` → `/workspace`
- 系统文件:
  - `/etc/timezone`
  - `/etc/localtime`
  - `/etc/passwd`
  - `/etc/group`
- 设备: `/dev`
- X11 显示: `/tmp/.X11-unix`

## 5. 示例

### ARM编译流程
```bash
./start_container.sh arm
# 在容器内
cd /workspace
colcon build
```
### x86编译流程
```bash
./start_container.sh
# 在容器内
cd /workspace
colcon build
```

### 注意事项
- 不带参数时，默认启动当前架构的编译环境。
- 在X86环境下使用`arm`参数可以利用qemu模拟启动ARM镜像来进行交叉编译，但需要注意由于模拟器的损耗在该环境中性能较差。

## 6. 常见问题

常见问题及解决方案：

- **镜像下载缓慢:**
```bash
# 可手动下载后放入脚本同目录
wget https://cdn.robosense.cn/AC_wiki/arm_sdk_compile_humble_based.tgz
wget https://cdn.robosense.cn/AC_wiki/x86_sdk_compile_humble_based.tgz
```

- **找不到 Docker:**
  - 确保 Docker 已安装并运行
  - 将用户添加到 Docker 用户组

- **权限不足:**
  - 需要时使用 sudo 运行
  - 检查用户组成员身份


# 配置指南

## 目录

### 基础环境

1. [Windows 端配置指南](docs/md/windows.md)
   - NVIDIA 驱动安装与更新
   - 安装 CUDA 和 cuDNN
   - Anaconda 的安装
   - PyTorch 环境安装
   - PyCharm 安装
   - LabelImg 安装及使用
   - YOLOv5 / YOLOv8 项目克隆和环境依赖安装

2. [虚拟机端配置指南](docs/md/vm.md)
   - 安装 SDK Manager

3. [板卡端配置指南](docs/md/board.md)
   - M.2 挂载
   - rootOnNVMe
   - fishros
   - pip
   - jtop
   - 摄像头
   - swap

4. [其他工具安装](docs/md/tools.md)
   - NoMachine
   - ToDesk
   - VNC
   - Jupyter-lab

### 无人机

5. [无人机相关配置](docs/md/drone.md)
   - MAVROS
   - ROS 分布式通信

6. [Ego-Planner](docs/md/ego-planner.md)
   - 依赖安装
   - Ceres Solver 编译
   - IMU 话题频率配置

7. [Deepstream-YOLO](docs/md/deepstream-yolo.md)
   - PyTorch (aarch64)
   - torchvision
   - YOLOv5 (v6.2) / YOLOv8
   - TensorRTX (YOLOv5 v6.2)
   - Deepstream 6.0.1
   - Deepstream-python

### 感知与建图

8. [RealSense 安装指南](docs/md/realsense.md)
   - 编译安装（Ubuntu）
   - 安装（Jetson / RK3588）
   - RealSense-ROS

9. [RoboSense 激光雷达](docs/md/rslidar.md)
   - rslidar_sdk 安装
   - rs_to_velodyne 转换器

10. [FAST-LIO 激光惯性里程计](docs/md/fastlio.md)
    - Livox-SDK2 安装
    - livox_ros_driver2
    - FAST_LIO 编译

11. [FAST-LIVO2 激光视觉里程计](docs/md/fastlivo.md)
    - rpg_vikit 依赖
    - FAST-LIVO2 编译

12. [ORB-SLAM3 视觉 SLAM](docs/md/orbslam3.md)
    - Eigen / Pangolin 安装
    - CMakeLists.txt 修改
    - ORB-SLAM3 编译

13. [Cartographer 2D SLAM](docs/md/cartographer.md)
    - Stow / Ninja / abseil-cpp 安装
    - Cartographer 编译

### 依赖库

14. [Sophus 李群库](docs/md/sophus.md)
    - 编译安装 Sophus

### 自动驾驶

15. [Autoware.ai](docs/md/autoware.md)
    - Docker 镜像构建与运行
    - ROS workspace 编译

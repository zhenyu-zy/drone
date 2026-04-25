# RealSense 安装指南

### 一、编译安装（Ubuntu x86_64）

```bash
git clone -b v2.56.5 https://github.com/realsenseai/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../
sudo make uninstall && make clean && make -j8 && sudo make install
```

### 二、安装（Jetson）

```bash
sudo cp /home/nx/drone/docs/d435/libuvc_installation_jetson.sh /home/nx/libuvc_installation_jetson.sh
sudo chmod +x ./libuvc_installation_jetson.sh
./libuvc_installation_jetson.sh
sudo rm libuvc_installation_jetson.sh
```

### 三、安装（OrangePi5Max / RK3588）

```bash
sudo cp /home/orangepi/drone/docs/d435/libuvc_installation_rk3588.sh /home/orangepi/libuvc_installation_rk3588.sh
sudo chmod +x ./libuvc_installation_rk3588.sh
./libuvc_installation_rk3588.sh
sudo rm libuvc_installation_rk3588.sh
```

### 四、测试

```bash
realsense-viewer
```

注意：左上角显示的 USB 必须是 3.x。若为 2.x，可能是 USB 线为 2.0，或插在了 2.0 的口上（3.0 的线和口均为蓝色）。

### 五、RealSense-ROS

- 在已建立的 catkin 工作空间中安装：
  ```bash
  mkdir realsense_ws/src
  cd realsense_ws/src
  git clone -b ros1-legacy https://github.com/IntelRealSense/realsense-ros.git
  git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
  cd ..
  catkin_make
  sudo apt-get install ros-noetic-ddynamic-reconfigure
  sudo apt-get install ros-noetic-rgbd-launch
  sudo apt-get install ros-noetic-realsense2-camera-*
  ```

- 添加环境变量：
  ```bash
  echo "source ~/realsense_ws/devel/setup.bash --extend" >> ~/.bashrc
  source ~/.bashrc
  ```

- 测试：
  ```bash
  roslaunch realsense2_camera rs_camera.launch
  rostopic list
  ```

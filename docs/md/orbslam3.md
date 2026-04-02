# ORB-SLAM3 视觉 SLAM

### 一、安装依赖

```bash
sudo apt-get install libeigen3-dev
sudo apt install libgl1-mesa-dev libglew-dev libpython2.7-dev pkg-config
sudo apt install libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols -y
```

### 二、编译安装 Pangolin

```bash
git clone -b v0.6 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j8
sudo make install
```

- 测试（可选）：
  ```bash
  cd Pangolin/examples/HelloPangolin
  mkdir build && cd build
  cmake ..
  make -j8
  ./HelloPangolin
  ```
  运行后显示红绿蓝立方体，表示安装成功。

### 三、克隆 ORB-SLAM3

```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git
```

### 四、修改 CMakeLists.txt

- 修改 `ORB_SLAM3/CMakeLists.txt`：
  - 第 33 行，将 OpenCV 版本从 4.4 改为 4.2：
    ```cmake
    find_package(OpenCV 4.2)
    ```
  - 第 42 行，去掉 Eigen3 版本限制：
    ```cmake
    find_package(Eigen3 REQUIRED)
    ```

- 修改 `ORB_SLAM3/Thirdparty/DBoW2/CMakeLists.txt`，同样去掉 Eigen3 版本限制。

- 修改 `ORB_SLAM3/Thirdparty/g2o/CMakeLists.txt` 第 72 行：
  ```cmake
  FIND_PACKAGE(Eigen3 REQUIRED)
  ```

### 五、安装 RealSense2 ROS 驱动

```bash
sudo apt-get install ros-noetic-realsense2-camera-* -y
```

### 六、编译 ORB-SLAM3

```bash
cd ORB_SLAM3
sudo chmod +x build.sh
./build.sh
```

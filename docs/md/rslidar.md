# RoboSense 激光雷达

### 一、创建工作空间并克隆 rslidar_sdk

```bash
mkdir -p rslidar_ws/src
cd rslidar_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```

### 二、安装系统依赖

```bash
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev libpcap-dev
```

### 三、修改点云类型配置

- 修改 `rslidar_ws/src/rslidar_sdk/CMakeLists.txt` 第 8 行：
  ```cmake
  set(POINT_TYPE XYZIRT)
  ```

### 四、安装 rs_to_velodyne

```bash
cd ..
git clone https://github.com/HViktorTsoi/rs_to_velodyne.git
```

- 修改 `rslidar_ws/src/rs_to_velodyne/CMakeLists.txt` 第 4 行：
  ```cmake
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
  ```

### 五、编译

```bash
cd ~/rslidar_ws
catkin_make
```

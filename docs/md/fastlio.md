# FAST-LIO 激光惯性里程计

### 一、安装 Livox-SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j8
sudo make install
cd ../..
```

### 二、创建工作空间并克隆源码

```bash
mkdir -p ws_livox/src
cd ws_livox/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd ..
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

### 三、修改 FAST_LIO 源码以兼容 livox_ros_driver2

- 修改 `src/FAST_LIO/CMakeLists.txt` 第 54 行，添加依赖：
  ```cmake
  livox_ros_driver2
  ```

- 修改 `src/FAST_LIO/package.xml` 第 28、39 行，添加依赖：
  ```xml
  <depend>livox_ros_driver2</depend>
  ```

- 修改 `src/FAST_LIO/src/laserMapping.cpp`：
  - 第 59 行，替换 include：
    ```cpp
    #include <livox_ros_driver2/CustomMsg.h>
    ```
  - 第 302 行，替换回调函数签名：
    ```cpp
    void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
    ```

- 修改 `src/FAST_LIO/src/preprocess.cpp`：
  - 第 44 行：
    ```cpp
    void Preprocess::process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
    ```
  - 第 92 行：
    ```cpp
    void Preprocess::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
    ```

- 修改 `src/FAST_LIO/src/preprocess.h`：
  - 第 8 行：
    ```cpp
    #include <livox_ros_driver2/CustomMsg.h>
    ```
  - 第 94 行：
    ```cpp
    void process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
    ```
  - 第 110 行：
    ```cpp
    void avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg);
    ```

### 四、编译

```bash
cd ws_livox/src/livox_ros_driver2
./build.sh ROS1
```

### 五、配置环境变量

```bash
echo 'source ~/ws_livox/devel/setup.bash --extend' >> ~/.bashrc
source ~/.bashrc
```

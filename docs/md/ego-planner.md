# Ego-Planner

### 一、克隆项目

```bash
git clone https://github.com/ZJU-FAST-Lab/Fast-Drone-250
cd Fast-Drone-250
unzip 3rd_party.zip
```

### 二、安装依赖

```bash
cd glog
sudo chmod 777 *
./autogen.sh && ./configure && make && sudo make install
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
cd ..
```

### 三、编译 Ceres Solver

```bash
sudo mv ceres-solver-2.0.0rc1 ceres-solver
cd ceres-solver
mkdir build && cd build
cmake ..
sudo make -j8
sudo make install
cd ../..
```

### 四、编译工作空间

```bash
catkin_make
source devel/setup.bash
roslaunch ego_planner single_run_in_sim.launch
```

### 五、更改 IMU 话题频率

> 非常重要：如果没有更改，会导致 VINS 发散。

进入工作空间下 `shfiles` 文件夹，打开 `rspx4.sh`，将 `roslaunch mavros px4.launch` 之后的 `& sleep` 改为 `10`，然后在后面添加两行：

```bash
rosrun mavros mavcmd long 511 105 4000 0 0 0 0 0 & sleep 3
rosrun mavros mavcmd long 511 31 4000 0 0 0 0 0 & sleep 3
```

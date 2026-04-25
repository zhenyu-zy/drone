# Cartographer 2D 建图

## 一、安装依赖库

### 1. 安装 stow ninja
```bash
sudo apt install -y stow ninja-build
```

### 2. 克隆 cartographer
```bash
git clone https://github.com/zhenyu-zy/cartographer.git
```

### 3. 安装 abseil-cpp
```bash
cd src/cartographer/scripts
sudo chmod +x ./install_abseil.sh
sudo ./install_abseil.sh
```

## 二、工作空间编译

### 正常编译(需先安装ceres-solver)
```bash
catkin_make_isolated --install --use-ninja
```

### 出现 Python 版本报错时编译
```bash
catkin_make_isolated --install --use-ninja -DPYTHON_EXECUTABLE=/usr/bin/python3




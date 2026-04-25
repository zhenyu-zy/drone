# FAST-LIVO2 激光视觉里程计

### 一、创建工作空间

```bash
mkdir -p ws_livox2/src
cd ws_livox2/src
```

### 二、克隆依赖与源码

```bash
git clone https://github.com/xuankuzcr/rpg_vikit.git
git clone https://github.com/hku-mars/FAST-LIVO2.git
```

### 三、编译

```bash
cd ~/ws_livox2
catkin_make
```

### 四、配置环境变量

```bash
echo 'source ~/ws_livox2/devel/setup.bash --extend' >> ~/.bashrc
source ~/.bashrc
```

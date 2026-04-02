# Autoware.ai

### 一、安装 Docker

```bash
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl start docker
sudo docker run hello-world
```

### 二、构建并运行 Autoware.ai Docker 镜像

```bash
git clone https://github.com/autowarefoundation/autoware_ai_docker.git
cd autoware_ai_docker/generic
./build.sh
./run.sh -t local
```

### 三、初始化工作空间

```bash
cd shared_dir
mkdir -p autoware.ai/src
cd autoware.ai
wget -O autoware.ai.repos "https://raw.githubusercontent.com/autowarefoundation/autoware_ai/1.14.0/autoware.ai.repos"
vcs import src < autoware.ai.repos
```

### 四、安装 ROS 依赖并编译

```bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

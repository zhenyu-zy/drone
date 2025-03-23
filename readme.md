看起来你已经将之前的配置指南整合到了一个完整的Markdown文件中，并且添加了链接。以下是对你的内容进行微调后的版本，确保格式更加规范，同时修复了一些小问题（例如重复的内容和链接格式）：

---

# 虚拟机端配置指南

## 一、安装 SDK Manager

- 使用虚拟机 Ubuntu 18.04 系统，下载 SDK Manager。
- 使用前请先注册/登录 NVIDIA 账号。
- 下载的 `.deb` 文件：
  ```bash
  sudo dpkg -i sdkmanager*
  ```

# Jetson 端配置指南

## 一、M.2 挂载

- 如果板卡上的 eMMC 比较小，推荐外接一个 SSD。
- 部分板卡没有 eMMC，用的是 SD 卡，如果 SD 卡够用则不需要使用 SSD。
- 检查挂载情况：
  ```bash
  df -h
  sudo fdisk -l
  ```

## 二、rootOnNVMe

- 将 SD 卡转存到 SSD，并以 SSD 启动系统：
  ```bash
  git clone https://github.com/jetsonhacks/rootOnNVMe.git
  cd rootOnNVMe/
  ./copy-rootfs-ssd.sh
  ./setup-service.sh
  sudo reboot
  df -h
  ```

## 三、fishros

- 安装 fishros：
  ```bash
  wget http://fishros.com/install -O fishros && . fishros
  ```

## 四、pip

- 检查 Python 版本：
  ```bash
  python3 --version
  ```
- 安装相关依赖：
  ```bash
  sudo apt install python3-pip python3-dev build-essential libssl-dev libffi-dev python3-setuptools
  ```
- 检查 pip 版本并升级：
  ```bash
  pip3 --version
  sudo pip3 install --upgrade pip
  # sudo pip3 install pip==21.3.1
  ```

## 五、jtop

- jtop 是 Jetson 系列设备最佳设备状态监控软件，可以实时查看 CPU、GPU、内存等硬件设备使用情况，开发环境配置情况，同时可以直接在图形化界面设置运行功率和风扇转速。
- 安装步骤：
  ```bash
  sudo apt install python3-pip
  sudo -H pip3 install -U jetson-stats
  reboot
  jtop
  ```

## 六、输入法

- 安装输入法：
  ```bash
  sudo apt-get install ibus ibus-clutter ibus-gtk ibus-gtk3 ibus-qt4
  im-config -s ibus
  sudo apt-get install ibus-pinyin
  ibus-setup
  ```

## 七、摄像头

- 安装 v4l-utils：
  ```bash
  sudo apt install v4l-utils
  ```
- 检查摄像头设备：
  ```bash
  v4l2-ctl --list-devices
  v4l2-ctl --device=/dev/video0 --list-formats-ext
  ```

## 八、swap

- 新增 swapfile 文件大小自定义（6G）：
  ```bash
  sudo fallocate -l 6G /var/swapfile
  ```
- 配置文件权限：
  ```bash
  sudo chmod 600 /var/swapfile
  ```
- 建立交换分区：
  ```bash
  sudo mkswap /var/swapfile
  ```
- 启用交换分区：
  ```bash
  sudo swapon /var/swapfile
  ```
- 自启动启用：
  ```bash
  sudo bash -c 'echo "/var/swapfile swap swap defaults 0 0" >> /etc/fstab'
  ```

## 九、VNC

- 安装 VNC：
  ```bash
  sudo apt update
  sudo apt install vino
  ```
- 配置 VNC server：
  ```bash
  gsettings set org.gnome.Vino prompt-enabled false
  gsettings set org.gnome.Vino require-encryption false
  ```
- 编辑 org.gnome，恢复丢失的“enabled”参数：
  ```bash
  sudo vi /usr/share/glib-2.0/schemas/org.gnome.Vino.gschema.xml
  ```
  添加以下内容：
  ```xml
  <key name='enabled' type='b'>
      <summary>Enable remote access to the desktop</summary>
      <description>
           If true, allows remote access to the desktop via the RFB
           protocol. Users on remote machines may then connect to the
           desktop using a VNC viewer.
       </description>
       <default>false</default>
  </key>
  ```
- 设置为 Gnome 编译模式：
  ```bash
  sudo glib-compile-schemas /usr/share/glib-2.0/schemas
  ```
- 手动启动：
  ```bash
  /usr/lib/vino/vino-server
  ```
- 设置 VNC 登陆密码：
  ```bash
  gsettings set org.gnome.Vino authentication-methods "['vnc']"
  gsettings set org.gnome.Vino vnc-password $(echo -n 'thepassword'|base64)
  sudo reboot
  ```
- 开机自启动 VNC Server：
  ```bash
  gsettings set org.gnome.Vino enabled true
  ```
- 创建自启动文件：
  ```bash
  mkdir -p ~/.config/autostart
  vi ~/.config/autostart/vino-server.desktop
  ```
  添加以下内容：
  ```ini
  [Desktop Entry]
  Type=Application
  Name=Vino VNC server
  Exec=/usr/lib/vino/vino-server
  NoDisplay=true
  ```

## 十、Jupyter-lab

### 1. 安装

- 安装依赖：
  ```bash
  sudo apt install nodejs npm
  sudo apt install libffi-dev
  sudo pip3 install jupyter jupyterlab
  ```

### 2. 生成配置文件

- 生成配置文件：
  ```bash
  jupyter notebook --generate-config
  ```

### 3. 修改配置文件

- 修改配置文件：
  ```bash
  sudo gedit ~/.jupyter/jupyter_notebook_config.py
  ```
  修改以下内容：
  ```python
  c.NotebookApp.ip = '0.0.0.0'  # 第296行
  c.NotebookApp.open_browser = False  # 第400行
  c.NotebookApp.port = 8888  # 第424行，开放的端口号
  c.NotebookApp.notebook_dir = '/home/'  # 第392行，可以访问的目录
  ```

### 4. 设置访问密码

- 设置密码：
  ```bash
  jupyter notebook password
  ```

### 5. 启动 Jupyter Notebook

- 启动 Jupyter Notebook：
  ```bash
  jupyter notebook
  ```

### 6. 开机自启动

- 查找 Jupyter-lab 安装位置：
  ```bash
  which jupyter-lab
  ```
- 创建 `jupyter.service` 文件：
  ```bash
  sudo gedit /etc/systemd/system/jupyter.service
  ```
  添加以下内容：
  ```ini
  [Unit]
  Description=Jupyter Notebook

  [Service]
  Type=simple
  User=fs  # 需要更换为自己的用户名
  ExecStart=/home/fs/.local/bin/jupyter-lab --port 8888  # 更换为自己的路径
  WorkingDirectory=/home/  # notebook 启动时的目录

  [Install]
  WantedBy=default.target
  ```
- 启动服务：
  ```bash
  sudo systemctl enable jupyter
  sudo systemctl start jupyter
  ```
- 查看当前 IP：
  ```bash
  ifconfig
  ```
- 重启：
  ```bash
  sudo reboot
  ```
- 等待重启完成，在同局域网下，通过浏览器访问 Ubuntu：`ip:端口号/lab`

## 十一、Deepstream-YOLO

### 1. PyTorch

- 安装依赖：
  ```bash
  sudo apt-get install python3-pip libopenblas-base libopenmpi-dev
  pip3 install Cython
  pip3 install numpy==1.19.3
  pip3 install protobuf==3.3.0
  pip3 install torch-1.8.0-cp36-cp36m-linux_aarch64.whl
  ```

### 2. torchvision (v0.9.0)

- 安装依赖：
  ```bash
  sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
  python3 -m pip install --upgrade pillow
  ```
- 克隆并安装：
  ```bash
  git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
  cd torchvision
  export BUILD_VERSION=0.9.0
  python3 setup.py install --user
  ```

### 3. 清华源

- 使用清华源安装 Python 包：
  ```bash
  # pip install numpy -i https://pypi.tuna.tsinghua.edu.cn/simple
  ```

### 4. YOLOv5 (v6.2)

- 安装依赖：
  ```bash
  pip install testresources
  pip install launchpadlib
  ```
- 克隆并安装：
  ```bash
  git clone -b v6.2 https://github.com/ultralytics/yolov5.git
  pip install -r requirements.txt
  ```

### 5. TensorRTX (YOLOv5 v6.2)

- 克隆并编译：
  ```bash
  git clone -b yolov5-v6.2 https://github.com/wang-xinyu/tensorrtx.git
  cd tensorrtx
  mkdir build
  cd build
  cmake ..
  make
  # 修改 yololar.h 中的 class_num
  ```

### 6. torchvision (v0.9.0)

- 安装依赖：
  ```bash
  sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev
  python3 -m pip install --upgrade pillow
  ```
- 克隆并安装：
  ```bash
  git clone --branch v0.9.0 https://github.com/pytorch/vision torchvision
  cd torchvision
  export BUILD_VERSION=0.9.0
  python3 setup.py install --user
  ```

### 7. Deepstream 6.0.1

- 使用脚本生成权重文件：
  ```bash
  # gen_wts_yolov5.py
  ```
  [DeepStream-Yolo](https://github.com/marcoslucianops/DeepStream-Yolo/tree/e652ef4e394fbcee0b8b8652c4630802bec4eab3)

---

如果你需要进一步调整或添加其他内容，请告诉我！
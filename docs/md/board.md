# 板卡端配置指南

### 一、M.2 挂载
- 如果板卡上的 eMMC 比较小，推荐外接一个 SSD。
- 部分板卡没有 eMMC，用的是 SD 卡，如果 SD 卡够用则不需要使用 SSD。
- 检查挂载情况：
  ```bash
  df -h
  sudo fdisk -l
  ```

### 二、rootOnNVMe
- 将 SD 卡转存到 SSD，并以 SSD 启动系统：
  ```bash
  git clone https://github.com/jetsonhacks/rootOnNVMe.git
  cd rootOnNVMe/
  ./copy-rootfs-ssd.sh
  ./setup-service.sh
  sudo reboot
  df -h
  ```

### 三、fishros
- 安装 fishros：
  ```bash
  wget http://fishros.com/install -O fishros && . fishros
  ```

### 四、pip
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

### 五、jtop
- jtop 是 Jetson 系列设备最佳设备状态监控软件，可以实时查看 CPU、GPU、内存等硬件设备使用情况，开发环境配置情况，同时可以直接在图形化界面设置运行功率和风扇转速。
- 安装步骤：
  ```bash
  sudo apt install python3-pip
  sudo -H pip3 install -U jetson-stats
  reboot
  jtop
  ```

### 六、摄像头
- 安装 v4l-utils：
  ```bash
  sudo apt install v4l-utils
  ```
- 检查摄像头设备：
  ```bash
  v4l2-ctl --list-devices
  v4l2-ctl --device=/dev/video0 --list-formats-ext
  ```

### 七、swap
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
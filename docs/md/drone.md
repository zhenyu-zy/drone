# 无人机相关配置

### 1. MAVROS
- MAVROS 是一层 MAVLink 与 ROS 通信的封装，旨在方便无人机与机载电脑通信。[如果geographiclib不能正常下载](docs/mavros/README.md)
  ```bash
  sudo apt-get install ros-noetic-mavros-*
  sudo apt-get install ros-noetic-serial
  cd /opt/ros/noetic/lib/mavros
  sudo ./install_geographiclib_datasets.sh
  ```

- 修改 launch 文件：
  ```bash
  sudo chmod 777 /opt/ros/noetic/share/mavros/launch/px4.launch
  sudo gedit /opt/ros/noetic/share/mavros/launch/px4.launch
  ```
  将其中的：
  ```xml
  <arg name="fcu_url" default="/dev/ttyACM0:57600" />
  ```
  修改为：
  ```xml
  <arg name="fcu_url" default="/dev/ttyACM0:921600" />
  ```

- 插上飞控查看 MAVROS 对应的串口号：
  ```bash
  ls /dev/tty*
  ```
  给予相应的权限：
  ```bash
  sudo chmod 777 /dev/ttyACM0
  ```
  启动 MAVROS：
  ```bash
  roslaunch mavros px4.launch
  ```

### 2. ROS 分布式通信
- 安装依赖：
  ```bash
  sudo apt install net-tools
  ```

- 配置主机和从机的 IP 地址（以 OrangePi 为例）：

  - 在 OrangePi 和虚拟机终端上：
    ```bash
    ifconfig  # 查看 IP
    hostname  # 查看计算机名称
    ```

  - 在 OrangePi 上：
    ```bash
    ping 192.168.**.**
    sudo gedit /etc/hosts
    ```
    如果没有 `gedit`，请在文件管理器中找到该文件，然后运行以下命令：
    ```bash
    sudo chmod 777 hosts
    ```
    添加一行：
    ```plaintext
    192.168.**.** 虚拟机用户名
    ```
    测试：
    ```bash
    ping 虚拟机用户名
    ```

  - 在虚拟机上：
    ```bash
    ping 192.168.**.**  # 测试 OrangePi
    sudo vim /etc/hosts
    ```
    添加一行：
    ```plaintext
    192.168.**.** orangepi
    ```
    测试：
    ```bash
    ping orangepi
    ```

- 配置主机的 IP 地址：
  - 在 OrangePi 端：
    ```bash
    sudo vim ~/.bashrc
    ```
    在最后一行加入以下代码：
    ```bash
    export ROS_MASTER_URI=http://主机IP:11311
    export ROS_HOSTNAME=主机IP
    ```
    保存并退出：
    ```bash
    :wq
    ```
    应用更改：
    ```bash
    source ~/.bashrc
    ```

  - 在虚拟机端：
    ```bash
    sudo vim ~/.bashrc
    ```
    在最后一行加入以下代码：
    ```bash
    export ROS_MASTER_URI=http://主机IP:11311
    export ROS_HOSTNAME=从机IP
    ```
    保存并退出：
    ```bash
    :wq
    ```
    应用更改：
    ```bash
    source ~/.bashrc
    ```

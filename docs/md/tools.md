# 其他工具安装

### 1. NoMachine
- **ARMv8**  
  下载地址：[NoMachine ARMv8](https://downloads.nomachine.com/linux/?distro=Arm&id=30)  
  安装命令：
  ```bash
  cd docs/app
  sudo dpkg -i nomachine*
  ```

### 2. ToDesk
- **ARM64 & aarch64**  
  下载地址：[ToDesk Linux 版](https://www.todesk.com/linux.html)  
  安装命令：
  ```bash
  sudo apt-get install libappindicator3-1
  sudo apt-get install ./todesk*
  ```

### 3. VNC
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
- 编辑 org.gnome，恢复丢失的"enabled"参数：
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

### 4. Jupyter-lab
#### 1. 安装
- 安装依赖：
  ```bash
  sudo apt install nodejs npm
  sudo apt install libffi-dev
  sudo pip3 install jupyter jupyterlab
  ```

#### 2. 生成配置文件
- 生成配置文件：
  ```bash
  jupyter notebook --generate-config
  ```

#### 3. 修改配置文件
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

#### 4. 设置访问密码
- 设置密码：
  ```bash
  jupyter notebook password
  ```

#### 5. 启动 Jupyter Notebook
- 启动 Jupyter Notebook：
  ```bash
  jupyter notebook
  ```

#### 6. 开机自启动
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

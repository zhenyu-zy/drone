# 虚拟机端配置指南

### 一、安装 SDK Manager

- 使用虚拟机 Ubuntu 18.04/20.04 系统，下载 SDK Manager。
- 使用前请先注册/登录 NVIDIA 账号。
- 安装 `.deb` 文件：
  ```bash
  sudo dpkg -i sdkmanager*
  ```
- 安装完成后，点击右上角三个点 → 更新，或手动启动：
  ```bash
  sdkmanager
  ```
- 切换旧版本：
  ```bash
  sdkmanager --archived-versions
  ```

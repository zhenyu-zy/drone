#!/bin/bash

# 确保脚本以root权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "此脚本需要root权限，请使用sudo运行"
    exit 1
fi

# 安装MAVROS相关包
echo "===== 开始安装MAVROS包 ====="
apt-get update
apt-get install -y ros-noetic-mavros-*
apt-get install -y ros-noetic-serial

# 检查安装是否成功
if [ $? -ne 0 ]; then
    echo "错误：MAVROS包安装失败"
    exit 1
fi

# 安装地理数据集
echo "===== 开始安装地理数据集 ====="
cd /opt/ros/noetic/lib/mavros || {
    echo "错误：找不到mavros目录"
    exit 1
}

if [ -f "install_geographiclib_datasets.sh" ]; then
    ./install_geographiclib_datasets.sh
else
    echo "错误：未找到install_geographiclib_datasets.sh脚本"
    exit 1
fi

# 修改px4.launch文件中的波特率
echo "===== 开始修改波特率设置 ====="
LAUNCH_FILE="/opt/ros/noetic/share/mavros/launch/px4.launch"

# 检查文件是否存在
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "错误：文件 $LAUNCH_FILE 不存在"
    echo "请确认已正确安装mavros包"
    exit 1
fi

# 修改文件权限
chmod 777 "$LAUNCH_FILE"

# 替换波特率设置
sed -i 's/<arg name="fcu_url" default="\/dev\/ttyACM0:57600" \/>/<arg name="fcu_url" default="\/dev\/ttyACM0:921600" \/>/g' "$LAUNCH_FILE"

# 验证修改是否成功
if grep -q "921600" "$LAUNCH_FILE"; then
    echo "成功：已将波特率修改为921600"
else
    echo "警告：未检测到波特率修改，请手动检查文件"
fi
echo "===== MAVROS安装和配置完成 ====="


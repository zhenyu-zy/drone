#!/bin/bash

# 检查是否以root权限运行
if [ "$(id -u)" -ne 0 ]; then
    echo "请使用root权限运行此脚本 (sudo $0)"
    exit 1
fi

# 定义交换文件路径
SWAP_FILE="/var/swapfile"

# 提示用户输入交换文件大小，默认6G
read -p "请输入交换文件大小（例如 6G 或 1024M，默认6G）: " SWAP_SIZE

# 如果用户未输入，使用默认值6G
if [ -z "$SWAP_SIZE" ]; then
    SWAP_SIZE="6G"
fi

echo "开始处理${SWAP_SIZE}的交换文件..."

# 检查是否存在交换文件，如果存在则先删除
if [ -f "$SWAP_FILE" ]; then
    echo "发现已存在的交换文件，正在移除..."
    
    # 先关闭交换文件
    echo "1. 关闭当前交换文件..."
    swapoff $SWAP_FILE
    
    # 从fstab中移除条目
    echo "2. 从开机启动配置中移除..."
    sed -i "/$SWAP_FILE/d" /etc/fstab
    
    # 删除交换文件
    echo "3. 删除交换文件..."
    rm -f $SWAP_FILE
fi

echo "开始创建新的${SWAP_SIZE}交换文件..."

# 1. 创建指定大小的交换文件
echo "4. 创建${SWAP_SIZE}的交换文件..."
fallocate -l $SWAP_SIZE $SWAP_FILE

# 检查上一步是否成功
if [ $? -ne 0 ]; then
    echo "创建交换文件失败"
    exit 1
fi

# 2. 配置文件权限
echo "5. 配置文件权限..."
chmod 600 $SWAP_FILE

# 3. 建立交换分区
echo "6. 建立交换分区..."
mkswap $SWAP_FILE

# 4. 启用交换分区
echo "7. 启用交换分区..."
swapon $SWAP_FILE

# 5. 配置自启动
echo "8. 配置开机自启动..."
echo "$SWAP_FILE swap swap defaults 0 0" >> /etc/fstab

echo "交换文件配置完成！"
echo "当前交换分区状态："
swapon --show
    

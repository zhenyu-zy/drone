#!/bin/bash

# 设置 CPU 最高性能模式并锁定最高频率
for i in {0..7}; do
    echo "performance" | sudo tee /sys/devices/system/cpu/cpu$i/cpufreq/scaling_governor
    max_freq=$(cat /sys/devices/system/cpu/cpu$i/cpufreq/scaling_max_freq)
    echo $max_freq | sudo tee /sys/devices/system/cpu/cpu$i/cpufreq/scaling_setspeed
    echo "CPU $i 设置为最高性能模式 ($max_freq Hz)"
done

echo "CPU 频率已设置为最高性能模式."

# 设置 GPU 最高性能模式
if [ -d "/sys/class/devfreq" ]; then
    for gpu in /sys/class/devfreq/*gpu*; do
        echo "performance" | sudo tee $gpu/governor
        max_gpu_freq=$(cat $gpu/available_frequencies | awk '{print $NF}')
        echo $max_gpu_freq | sudo tee $gpu/max_freq
        echo "GPU 频率设置为最高 ($max_gpu_freq Hz)"
    done
else
    echo "未找到 GPU 设备, 跳过 GPU 频率设置."
fi

# 设置 DDR 内存最高频率
if [ -d "/sys/class/devfreq/dmc" ]; then
    max_ddr_freq=$(cat /sys/class/devfreq/dmc/available_frequencies | awk '{print $NF}')
    echo $max_ddr_freq | sudo tee /sys/class/devfreq/dmc/max_freq
    echo "DDR 频率设置为最高 ($max_ddr_freq Hz)"
else
    echo "未找到 DDR 控制器, 跳过 DDR 频率设置."
fi

# 关闭功耗限制
if [ -f "/sys/module/rockchip_pm/parameters/enable" ]; then
    echo 0 | sudo tee /sys/module/rockchip_pm/parameters/enable
    echo "功耗限制已关闭."
else
    echo "未找到功耗管理文件, 跳过此步骤."
fi

# 提高 I/O 读写速度
if [ -d "/sys/block" ]; then
    for dev in /sys/block/mmcblk0 /sys/block/sd*; do
        if [ -f "$dev/queue/scheduler" ]; then
            echo noop | sudo tee $dev/queue/scheduler
            echo "$dev I/O 调度器设置为 noop."
        fi
    done
fi
echo "RK3588 现已运行在最高性能模式 🚀"
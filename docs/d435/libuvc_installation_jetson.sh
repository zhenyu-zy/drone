#!/bin/bash -xe

#在本地抑制 stderr 以避免引发不相关的消息
exec 3>&2
exec 2> /dev/null
con_dev=$(ls /dev/video* | wc -l)
exec 2>&3

if [ $con_dev -ne 0 ];
then
	echo -e "\e[32m"
	read -p "卸下所有连接的实感摄像头。准备就绪后按任意键继续..."
	echo -e "\e[0m"
fi

lsb_release -a
echo "Kernel version $(uname -r)"
sudo apt-get update
cd ~/
sudo rm -rf ./librealsense

echo Installing Librealsense-required dev packages
sudo apt-get install git cmake libssl-dev freeglut3-dev libusb-1.0-0-dev pkg-config libgtk-3-dev unzip -y

git clone -b v2.56.5 https://github.com/IntelRealSense/librealsense.git
cd ./librealsense

echo Install udev-rules
sudo rm -f /etc/udev/rules.d/99-realsense-libusb.rules
sudo rm -f /etc/udev/rules.d/99-realsense-d4xx-mipi-dfu.rules
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo cp config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger 
mkdir build && cd build
cmake ../  -DCMAKE_BUILD_TYPE=release -DBUILD_WITH_CUDA=true -DFORCE_RSUSB_BACKEND=true
make -j8
sudo make install
echo -e "\e[92m\n\e[1mLibrealsense script completed.\n\e[0m"





#!/bin/bash

set -e

# Установка ROS Melodic
echo "Установка ROS Melodic..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-melodic-desktop-full

# Инициализация rosdep
echo "Инициализация rosdep..."
sudo rosdep init
rosdep update

# Настройка окружения ROS
echo "Настройка окружения ROS..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Установка необходимых инструментов
echo "Установка необходимых инструментов..."
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Создание рабочего пространства catkin
echo "Создание рабочего пространства catkin..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

# Установка пакетов для LIDAR Velodyne VLP-16
echo "Установка пакетов для LIDAR Velodyne VLP-16..."
sudo apt install -y ros-melodic-velodyne

# Установка пакетов для ODrive
echo "Установка пакетов для ODrive..."
cd ~/catkin_ws/src
git clone https://github.com/neomanic/odrive_ros.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

# Установка пакетов для XSens MTI-1 IMU
echo "Установка пакетов для XSens MTI-1 IMU..."
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/xsens_driver.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash

# Установка пакетов для работы с веб-камерой
echo "Установка пакетов для работы с веб-камерой..."
sudo apt install -y ros-melodic-usb-cam

# Настройка SSH
echo "Настройка SSH..."
if [ ! -f ~/.ssh/id_rsa ]; then
    ssh-keygen -t rsa -N "" -f ~/.ssh/id_rsa
fi
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
echo "Ваш публичный ключ SSH:"
cat ~/.ssh/id_rsa.pub

# Финальные шаги
echo "Настройка завершена."

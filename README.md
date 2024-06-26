Для проекта берется робот-пылесос с примером подключения трех датчиков: LIDAR Velodyne VLP-16, контроллера двигателей колёс ODrive и IMU-датчиком XSens MTI-1. Указанные датчики подключаются к вычислительному модулю NVIDIA Jetson, который выполняет функции вычислений, обработки данных, управления и коммуникации с внешними устройствами. Основные интерфейсы для подключения устройств: USB, Ethernet, UART, SPI, GPIO.

#### Схема подключения датчиков

Компоненты:
1. NVIDIA Jetson: Выполняет функции управления, обработки данных и коммуникации с внешними устройствами через ROS.
Подключение к интернету и внешним устройствам через Ethernet или USB.

2. Velodyne VLP-16 LIDAR: Использует ROS для получения данных о расстояниях и создания карты окружающей среды. Подключается к NVIDIA Jetson через Ethernet интерфейс для передачи данных LIDAR.

3. ODrive контроллеры двигателей: Отправляются команды управления двигателями и получаются обратные данные о положении и скорости.Каждый контроллер ODrive подключается к NVIDIA Jetson через UART интерфейс.

4. XSens MTI-1 IMU: Используется для оценки ориентации и мониторинга движения робота. Подключается к NVIDIA Jetson через UART или SPI интерфейс для передачи данных о поворотах и ускорениях.

#### Библиотеки и пакеты ROS для подключения датчиков

Для работы с вышеуказанными датчиками (LIDAR Velodyne VLP-16, контроллеры двигателей ODrive, IMU-датчик XSens MTI-1) в ROS на основе Ubuntu (Linux), потребуется установить следующие пакеты и библиотеки. Предполагается, что у вас уже установлен ROS Melodic или другая поддерживаемая версия ROS. 

Пакеты для работы с датчиками

Пакеты для работы с LIDAR Velodyne в ROS: velodyne
`sudo apt-get install ros-melodic-velodyne`

Пакеты для взаимодействия с контроллерами ODrive в ROS: ros-odrive
```
# Установка пакета через исходный код (например, если нет стандартного пакета)
cd ~/catkin_ws/src
git clone https://github.com/madcowswe/ODrive.git
cd ..
catkin_make
source devel/setup.bash
```

Пакет для работы с IMU от XSens в ROS: xsens_driver
```
# Установка через исходный код
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/xsens_driver.git
cd ..
catkin_make
source devel/setup.bash
```

#### Создание скрипта для автоматической установки всех необходимых пакетов и зависимостей

Скрипт устанавливает все необходимые пакеты и зависимости для работы с LIDAR Velodyne VLP-16, контроллерами двигателей ODrive и IMU-датчиком XSens MTI-1 в ROS Melodic на Ubuntu. Скрипт также настроит окружение ROS и SSH доступ.

Скрипт автоматической установки `install_robot_dependencies.sh` :

```
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
```

#### Создание ROS-пакета с LAUNCH-файлом

Для создания ROS-пакета и LAUNCH-файла используйте следующие команды:
```
cd ~/catkin_ws/src
catkin_create_pkg my_robot_control rospy std_msgs  # Примерный набор зависимостей
cd my_robot_control
mkdir launch
touch my_robot.launch
```

Скрипт `my_robot.launch`:
```
<launch>
    <arg name="rviz" default="false"/>
    <arg name="rqt" default="false"/>

    <node name="velodyne_node" pkg="velodyne" type="velodyne_node" output="screen"/>

    <group if="$(arg rviz)">
        <node name="rviz" pkg="rviz" type="rviz"/>
    </group>

    <group if="$(arg rqt)">
        <node name="rqt" pkg="rqt_gui" type="rqt_gui"/>
    </group>
</launch>
```

#### Подключение и проверка работы датчика (например, веб-камера)

Установка необходимых пакетов для работы с веб-камерой:
```
sudo apt install -y ros-melodic-camera-info-manager ros-melodic-image-transport ros-melodic-cv-camera
```

Пример использования веб-камеры в launch-файле:
```
<launch>
    <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="mjpeg"/>
        <param name="camera_frame_id" value="usb_cam"/>
    </node>

    <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
        <remap from="image" to="/usb_cam/image_raw"/>
    </node>
</launch>
```

#### Создание Dockerfile для сборки docker-образа

Создайте файл Dockerfile в корне вашего ROS Workspace:
```
# Базовый образ с Ubuntu 18.04 и ROS Melodic
FROM ros:melodic-ros-base-bionic

# Установка необходимых пакетов ROS
RUN apt-get update && apt-get install -y \
    ros-melodic-desktop-full \
    ros-melodic-velodyne \
    ros-melodic-usb-cam \
    ros-melodic-rviz \
    ros-melodic-rqt \
    && rm -rf /var/lib/apt/lists/*

# Установка пакетов для удалённого доступа по SSH
RUN apt-get update && apt-get install -y \
    openssh-server \
    && rm -rf /var/lib/apt/lists/*

# Настройка SSH доступа
RUN mkdir /var/run/sshd

# Добавление SSH ключа для пользователя root
COPY id_rsa.pub /root/.ssh/authorized_keys
RUN chmod 600 /root/.ssh/authorized_keys \
    && chown root:root /root/.ssh/authorized_keys

# Настройка SSH сервера
RUN echo "PermitRootLogin yes" >> /etc/ssh/sshd_config \
    && echo "PasswordAuthentication no" >> /etc/ssh/sshd_config

EXPOSE 22

CMD ["/usr/sbin/sshd", "-D"]
```

#### Инструкция по использованию Dockerfile

1. Создайте новый каталог для вашего проекта Docker, если его еще нет:
```
mkdir ~/my_robot_docker
cd ~/my_robot_docker
```

2. Сохраните Dockerfile в этом каталоге.

3. Создайте или скопируйте ваш публичный SSH ключ в файл id_rsa.pub в том же каталоге. Если у вас еще нет SSH ключей, вы можете создать их с помощью следующей команды:
```
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

4. Постройте Docker-образ:
```
docker build -t my_robot_image .
```

5. Запустите Docker-контейнер:
```
docker run -d -p 2222:22 --name my_robot_container my_robot_image
```

6. Подключитесь к контейнеру по SSH:
```
ssh -p 2222 root@localhost
```

Убедитесь, что ваш SSH агент настроен для использования ключа, или используйте флаг -i для указания файла ключа:
```
ssh -i ~/.ssh/id_rsa -p 2222 root@localhost
```

### Конец



# Базовый образ с ROS Melodic
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

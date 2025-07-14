FROM arm64v8/ros:humble-ros-core

# 更换ubuntu ubuntu-ports源
RUN curl -sSL https://linuxmirrors.cn/main.sh -o /tmp/mirror.sh && \
    bash /tmp/mirror.sh \
      --source mirrors.tuna.tsinghua.edu.cn \
      --protocol http \
      --use-intranet-source false \
      --install-epel false \
      --backup true \
      --upgrade-software false \
      --clean-cache false \
      --ignore-backup-tips

# 更换ros2源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt upgrade -y

# Install some essential packages
RUN DEBIAN_FRONTEND=noninteractive apt install -y --no-install-recommends --no-install-suggests \
    ros-dev-tools wget gnupg2 git vim unzip

# Set cyclonedds as default rmw implementation
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 安装Nav2
RUN apt install -y \
    ros-humble-nav2-bringup ros-humble-navigation2










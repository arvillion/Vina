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

# Install Foxglove Bridge
RUN apt install -y ros-humble-foxglove-bridge

RUN apt install -y ros-humble-geometry2 ros-humble-tf2-ros ros-humble-nav2-msgs ros-humble-map-msgs ros-humble-dwb-msgs ros-humble-bond

# 安装woosh agent
COPY ros-humble-woosh-robot-agent_0.0.1-0jammy_arm64.run /tmp/woosh-agent.run
RUN chmod +x /tmp/woosh-agent.run && \
    /tmp/woosh-agent.run && \
    rm -rf /tmp/woosh-agent.run










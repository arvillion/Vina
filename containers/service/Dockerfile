FROM arm64v8/ros:humble-ros-core

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


RUN apt-get update && apt upgrade -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends --no-install-suggests \
    git \
    unzip \
    python3-colcon-common-extensions \
    python3-pip \
    python3-dev \
    build-essential \
    libprotobuf-dev \
    libgl1 \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 更换pypi源
RUN pip3 config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple

RUN pip3 install \
    requests \
    numpy \
    scipy \
    pyyaml \
    opencv-python-headless \
    openai \
    flask \
    open3d \
    pyrealsense2 \
    hydra-core \
    && rm -rf /root/.cache/pip

# 更换ros2源
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt upgrade -y

# Set cyclonedds as default rmw implementation
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY ./resources/woosh_agent/ros-humble-woosh-robot-agent_0.0.2-0jammy_arm64.run /tmp/woosh-robot-agent.run
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends --no-install-suggests sudo 
RUN chmod +x /tmp/woosh-robot-agent.run && \ 
    cd /tmp && ./woosh-robot-agent.run && \
    rm -f woosh-robot-agent.run 

WORKDIR /ros2_ws

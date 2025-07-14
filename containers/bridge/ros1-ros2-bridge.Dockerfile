FROM ros:foxy-ros-base-focal
# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ca-certificates \
    curl \
    && rm -rf /var/lib/apt/lists/*


# setup keys
RUN set -eux; \
       key='4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros1-snapshots-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

# setup sources.list
RUN echo "deb [ signed-by=/usr/share/keyrings/ros1-snapshots-archive-keyring.gpg ] http://snapshots.ros.org/noetic/final/ubuntu focal main" > /etc/apt/sources.list.d/ros1-snapshots.list

ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=foxy

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros1-bridge \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

COPY ros_bridge.yaml /ros_bridge.yaml
COPY start_bridge.sh /start_bridge.sh
RUN chmod +x /start_bridge.sh

# 设置入口点
ENTRYPOINT ["/start_bridge.sh"]

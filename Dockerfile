FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
    xfce4 \
    xfce4-terminal \
    tigervnc-standalone-server \
    tigervnc-common \
    tigervnc-tools \
    novnc \
    python3-websockify \
    dbus-x11 \
    x11-utils \
    sudo \
    curl \
    wget \
    git \
    git-lfs \
    nano \
    net-tools \
    mesa-utils \
    libgl1 \
    libglu1-mesa \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    python3-yaml \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-ros2controlcli \
    ros-jazzy-moveit \
    ros-jazzy-control-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN mkdir -p /root/.vnc \
    && echo "ros" | vncpasswd -f > /root/.vnc/passwd \
    && chmod 600 /root/.vnc/passwd

COPY docker/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh \
    && bash -lc "source /opt/ros/jazzy/setup.bash && colcon build"

EXPOSE 6080 5901 3000
CMD ["/usr/local/bin/docker-entrypoint.sh"]

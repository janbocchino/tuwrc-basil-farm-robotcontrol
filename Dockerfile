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

# VNC password + session. Both are independent of the workspace sources, so
# they stay cached when only src/ changes.
RUN mkdir -p /root/.vnc \
    && echo "ros" | vncpasswd -f > /root/.vnc/passwd \
    && chmod 600 /root/.vnc/passwd

COPY docker/xstartup /root/.vnc/xstartup
COPY docker/docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /root/.vnc/xstartup /usr/local/bin/docker-entrypoint.sh

# Sources go in last so that editing them only invalidates this one cheap layer.
# The workspace is NOT built here on purpose: docker-compose bind-mounts the
# host repo over /workspace, so any install/ produced at image build time is
# replaced at runtime anyway. The entrypoint builds instead, incrementally,
# into the persistent build/install volumes.
COPY . /workspace

EXPOSE 6080 5901 3000
CMD ["/usr/local/bin/docker-entrypoint.sh"]

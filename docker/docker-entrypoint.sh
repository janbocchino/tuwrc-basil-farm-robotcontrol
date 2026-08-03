#!/bin/bash
set -euo pipefail

rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true
export USER="${USER:-root}"
export HOME="${HOME:-/root}"

vncserver :1 -geometry 1920x1080 -depth 24 -localhost no
websockify --web /usr/share/novnc/ 6080 localhost:5901 &

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM=1

# ROS setup scripts reference optional unset vars; disable nounset while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd /workspace

# Bind-mounting the host repo hides the image build. Always rebuild into the
# Docker volumes so source edits and install stay in sync (fast for this repo).
echo "Building workspace inside container…"
colcon build
# shellcheck disable=SC1091
source /workspace/install/setup.bash
set -u

MODE="${TUWRC_MODE:-view}"
echo "READY: noVNC http://localhost:6080/vnc.html (password: ros)"
echo "READY: GUI http://localhost:3000"
echo "Starting robot bringup in mode=${MODE}"

exec ros2 launch tuwrc_bringup robot.launch.py \
  mode:="${MODE}" \
  use_rviz:=true \
  use_moveit:=true \
  use_gui:=true \
  gui_port:=3000

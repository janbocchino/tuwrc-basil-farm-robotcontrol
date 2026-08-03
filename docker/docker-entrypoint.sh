#!/bin/bash
set -euo pipefail

rm -f /tmp/.X1-lock /tmp/.X11-unix/X1 2>/dev/null || true
export USER="${USER:-root}"
export HOME="${HOME:-/root}"

# Everything below renders on the CPU (llvmpipe): there is no GPU to pass
# through on macOS/Windows Docker. Both the framebuffer size and the number of
# rasterizer threads are therefore direct CPU-load knobs.
VNC_GEOMETRY="${TUWRC_VNC_GEOMETRY:-1600x900}"
export TUWRC_VNC_SESSION="${TUWRC_VNC_SESSION:-light}"

vncserver :1 -geometry "${VNC_GEOMETRY}" -depth 24 -localhost no
websockify --web /usr/share/novnc/ 6080 localhost:5901 &

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
# Cap llvmpipe's rasterizer threads. Unset it defaults to one per core, which
# lets a single RViz window saturate every CPU the Docker VM was given.
export LP_NUM_THREADS="${LP_NUM_THREADS:-4}"
export QT_X11_NO_MITSHM=1

# ROS setup scripts reference optional unset vars; disable nounset while sourcing.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd /workspace

# Bind-mounting the host repo hides the image build. Always rebuild into the
# Docker volumes so source edits and install stay in sync (fast for this repo).
# --symlink-install keeps colcon from re-copying the ~16 MB of meshes into
# install/ on every single start; it matches what tools/run does natively.
echo "Building workspace inside container…"
colcon build --symlink-install
# shellcheck disable=SC1091
source /workspace/install/setup.bash
set -u

MODE="${TUWRC_MODE:-view}"
USE_RVIZ="${TUWRC_USE_RVIZ:-true}"
USE_MOVEIT="${TUWRC_USE_MOVEIT:-true}"
USE_GUI="${TUWRC_USE_GUI:-true}"
MOCK_RATE="${TUWRC_MOCK_RATE:-20.0}"

if [[ "${USE_RVIZ}" == "true" ]]; then
  echo "READY: noVNC http://localhost:6080/vnc.html (password: ros)"
fi
if [[ "${USE_GUI}" == "true" ]]; then
  echo "READY: GUI http://localhost:3000"
fi
echo "Starting robot bringup in mode=${MODE} (rviz=${USE_RVIZ} moveit=${USE_MOVEIT} gui=${USE_GUI})"

exec ros2 launch tuwrc_bringup robot.launch.py \
  mode:="${MODE}" \
  use_rviz:="${USE_RVIZ}" \
  use_moveit:="${USE_MOVEIT}" \
  use_gui:="${USE_GUI}" \
  mock_rate:="${MOCK_RATE}" \
  gui_port:=3000

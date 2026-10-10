# tuwrc-basil-farm-robotcontrol

GitHub: [https://github.com/janbocchino/tuwrc-basil-farm-robotcontrol](https://github.com/janbocchino/tuwrc-basil-farm-robotcontrol)

One shared ROS 2 project for the TUWRC basil-farm robot: Max’s measured SO-101 arm on the prismatic X-rail, with RViz, MoveIt, a browser GUI, mock view mode, and real arm hardware support on Linux.

| Mode | What you get | Where it runs |
| --- | --- | --- |
| **view** | Simulated joint motion in RViz and the browser GUI | macOS, Windows/WSL, Linux |
| **hardware** | Real six-servo arm over USB. Rail held at 0 m | Native Ubuntu 24.04 |

Gazebo is not part of the MVP. The physical rail motor is not driven yet.

## View mode

View mode runs a mock arm and a movable rail (±0.5 m). On macOS and Windows it runs in Docker. On Ubuntu you can use Docker or a native ROS 2 install.

### Shared setup

- Git
- Git LFS (`git lfs install`)
- Docker Desktop or Docker Engine (required on macOS and Windows; optional on Linux)

Clone the whole repository, then pull the meshes:

```bash
git lfs install
git clone git@github.com:janbocchino/tuwrc-basil-farm-robotcontrol.git
cd tuwrc-basil-farm-robotcontrol
git lfs pull
```

### macOS

1. Install Docker Desktop and start it.
2. From the repository root:

```bash
./tools/run --mode view
```

3. Open:
   - RViz desktop: [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html) (password: `ros`)
   - Browser GUI: [http://localhost:3000](http://localhost:3000)

Stop with `Ctrl+C`, or:

```bash
docker compose --profile view down
```

### Windows (WSL2)

1. Install Docker Desktop with WSL2 integration, or Docker Engine inside WSL.
2. Clone the repository inside WSL and run `git lfs install && git lfs pull`.
3. From the repository root:

```bash
./tools/run --mode view
```

4. Open the same URLs as macOS from the Windows browser (`localhost:6080` and `localhost:3000`).

USB hardware through WSL (`usbipd-win`) is experimental. Use native Ubuntu for the real arm.

### Ubuntu

View mode without Docker:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
./tools/run --mode view --runtime native
```

RViz opens as a normal Linux window. The GUI is at [http://localhost:3000](http://localhost:3000).

Docker view mode (`./tools/run --mode view`) also works on Ubuntu if ROS is not installed.

### If the machine gets hot

Docker on macOS and Windows has no GPU passthrough, so RViz renders on the CPU. It redraws continuously even when the robot is still and even when no browser is connected to noVNC.

Measured on an M-series Mac (10-CPU Docker VM), idle and stationary:

| Command | Container CPU |
| --- | --- |
| `./tools/run --mode view` | ~140 % |
| `./tools/run --mode view --no-moveit` | ~57 % |
| `./tools/run --mode view --no-rviz` | ~10 % |

`--no-rviz` keeps the browser GUI fully working: joint control, the end-effector pose readout, and MoveIt IK. You lose only the 3D view.

Optional environment variables:

| Variable | Default | Effect |
| --- | --- | --- |
| `TUWRC_VNC_GEOMETRY` | `1600x900` | noVNC desktop size; smaller means less to rasterize and encode |
| `LP_NUM_THREADS` | `4` | Caps the software renderer's threads |
| `TUWRC_MOCK_RATE` | `20.0` | `/joint_states` publish rate in Hz |
| `TUWRC_VNC_SESSION` | `light` | `full` gives the complete XFCE desktop inside noVNC |

`./tools/run --geometry 1280x720` sets `TUWRC_VNC_GEOMETRY` for you.

### Try a movement

With bringup already running:

1. Wait until the GUI status shows controllers ready.
2. Click **Fill current**.
3. Change one arm joint by a few degrees.
4. Click **Send**.
5. Watch RViz update.

Scripted motion, from a second terminal. `./tools/example` sets up the Docker or native ROS environment:

```bash
./tools/example small_arm_motion --return-home
./tools/example leaf_pick_imitation --move-rail --return-home
./tools/example leaf_pick_imitation --scale 0.5 --return-home
```

`leaf_pick_imitation` is a jaw open/close snip gesture. `--move-rail` works in view mode only.

## Hardware mode

Hardware mode runs on the Ubuntu computer that has the arm plugged in. It drives the six servos over USB and holds the rail at 0 m. Docker refuses this mode.

### One-time setup

Ubuntu 24.04 with ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-moveit \
  ros-jazzy-control-msgs \
  python3-serial \
  python3-yaml \
  python3-colcon-common-extensions

sudo usermod -aG dialout "$USER"
```

Log out and back in after the `dialout` change so the shell can open the USB serial port.

In the repository:

```bash
git lfs install
git lfs pull
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Connect the arm

Plug the arm in with a data USB cable, then see which port appeared:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

The usual port is `/dev/ttyACM0`. Pass a different path with `--port` if the name differs.

Keep people and objects clear of the arm before the next step.

### Start

Leave this terminal running:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./tools/run --mode hardware --runtime native --port /dev/ttyACM0
```

RViz opens as a normal Linux window. The GUI is at [http://localhost:3000](http://localhost:3000). Wait until the status says the controllers are ready.

### First movements

Start in the GUI, one joint only:

1. Click **Fill current**.
2. Change one arm joint by a few degrees.
3. Click **Send**.
4. Confirm the real arm and RViz both move.

Then, from a second terminal, with bringup still running:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./tools/example small_arm_motion --hardware --allow-hardware --return-home
```

That nudges joints 1–4 and 6 by small amounts (about 0.08–0.15 rad) and returns to the start pose. The script refuses to move the real arm without `--allow-hardware`.

A smaller scripted move:

```bash
./tools/example small_arm_motion --hardware --allow-hardware --delta-deg 5 --return-home
```

`--delta-deg` must stay within ±20 degrees.

After that looks right, the jaw open/close gesture:

```bash
./tools/example leaf_pick_imitation --hardware --allow-hardware --scale 0.5 --return-home
```

Leave `--move-rail` off on the real robot. The rail stays fixed at 0 m until a rail driver exists.

### Calibration and safety

- Measured joint limits and zero positions live in `src/six_motor_driver/config/six_motor_calibration.yaml`.
- Change calibration or URDF joint limits only after a documented physical check. See [CONTRIBUTING.md](CONTRIBUTING.md).
- Use tiny deltas first.
- Hardware scripts require `--allow-hardware`.

## Depth camera

The Orbbec Gemini 2 runs on the same native Ubuntu 24.04 computer as the arm, using the driver pinned in `dependencies/orbbec.repos`.

- **Ubuntu robot computer:** arm driver, camera driver, TF, MoveIt, and point-cloud consumers.
- **macOS development computer:** edit the repo and use Docker view mode. Replay recorded camera data there for later perception work.

The Gemini 2 needs a direct USB connection on Ubuntu. Docker Desktop on macOS cannot pass the camera through with a normal `--device` mapping, because containers run inside a Linux VM. The Orbbec ROS 2 wrapper documents Linux as the supported platform.

One-time source and system setup, after the hardware setup above:

```bash
sudo apt update
sudo apt install python3-vcstool libgflags-dev nlohmann-json3-dev \
  libdw-dev libssl-dev libgoogle-glog-dev \
  ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
  ros-jazzy-compressed-image-transport ros-jazzy-image-publisher \
  ros-jazzy-camera-info-manager ros-jazzy-diagnostic-updater \
  ros-jazzy-diagnostic-msgs ros-jazzy-statistics-msgs \
  ros-jazzy-backward-ros

vcs import . < dependencies/orbbec.repos
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Required once for non-root USB access, then unplug and reconnect the camera.
sudo bash src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Use a data-capable USB 3 cable and update the Gemini 2 to Orbbec firmware `1.4.98`. Check discovery before starting the arm:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run orbbec_camera list_devices_node
```

Start the real arm and the depth point cloud:

```bash
./tools/run --mode hardware --runtime native --camera

# Optional RGB-colored registered cloud:
./tools/run --mode hardware --runtime native --colored-point-cloud
```

Expected topics:

- `/camera/color/image_raw` and `/camera/color/camera_info`
- `/camera/depth/image_raw` and `/camera/depth/camera_info`
- `/camera/depth/points`
- `/camera/depth_registered/points` when `--colored-point-cloud` is used

In RViz, add a `PointCloud2` display for `/camera/depth/points` and set the fixed frame to `world`. If the cloud follows the gripper but sits offset or rotated, replace the provisional `camera_mount_*` values near the top of `src/lerobot_description/urdf/so101_base.xacro` with the measured `gripper` to `camera_link` transform. The current zeros are placeholders.

Camera troubleshooting:

- `Permission denied` or no device: rerun the udev setup, reconnect the camera, and start the ROS node as your user.
- USB 3 detection or missing streams: change the cable or port and plug in directly.
- Point cloud exists but RViz cannot transform it: confirm `gripper` → `camera_link` and the driver's optical frames are in TF.
- High bandwidth or CPU: leave the colored cloud off and lower resolution or FPS in `tuwrc_bringup/launch/gemini2.launch.py`.

## Control contract

Both modes use the same actions:

- Arm and gripper: `/six_motor_controller/follow_joint_trajectory` (joints `1`–`6`)
- Rail: `/rail_controller/follow_joint_trajectory` (joint `rail_joint`)
- Feedback: `/joint_states`
- Pose IK: MoveIt `/compute_ik`, group `arm`, frames `world` → `tcp`

| Mode | Arm controller | Rail controller |
| --- | --- | --- |
| view | `tuwrc_mock_hardware` | mock (movable ±0.5 m) |
| hardware | `six_motor_driver` | `rail_hold` (fixed 0 m) |

## Repository map

```text
tuwrc-basil-farm-robotcontrol/
├── README.md
├── CONTRIBUTING.md
├── dependencies/             ← pinned third-party ROS source manifests
├── Dockerfile                ← ROS 2 Jazzy + noVNC image
├── docker-compose.yml
├── docker/                   ← container entrypoint
├── tools/run                 ← start script
├── tools/example             ← motion example runner
├── tests/
└── src/
    ├── lerobot_description/  ← URDF/xacro, meshes, RViz config
    ├── six_motor_driver/     ← ST3215 arm driver + calibration
    ├── six_motor_moveit_config/
    ├── lerobot_gui/          ← browser UI (port 3000)
    ├── tuwrc_bringup/        ← launch: view | hardware
    ├── tuwrc_mock_hardware/
    └── tuwrc_motion_examples/
```

| Path | Purpose |
| --- | --- |
| `src/lerobot_description/urdf/so101_base.xacro` | Rail + measured arm model |
| `src/lerobot_description/meshes/` | STL meshes (Git LFS) |
| `src/six_motor_driver/config/six_motor_calibration.yaml` | Real-robot calibration |
| `src/tuwrc_bringup/launch/robot.launch.py` | Main launch file |
| `src/lerobot_gui/lerobot_gui/joint_state_gui.py` | Browser GUI |
| `tools/run` | OS-aware launcher |

## Git

Push the whole repository. Docker files, `tools/`, this README, and the manifests are required on every OS. Branch and pull-request rules are in [CONTRIBUTING.md](CONTRIBUTING.md).

Commit source, launch files, calibration, meshes (via Git LFS), tests, docs, Docker, and `tools/`. Leave `build/`, `install/`, `log/`, IDE settings, and `.env` untracked.

If RViz is missing meshes, run `git lfs pull`.

## Later

- Gazebo simulation parity
- Rail-to-arm geometry from the assembled CAD model (mount args in `so101_base.xacro` are provisional)
- Real rail motor, encoder, homing, and E-stop driver
- Visual seating of the arm on the slider

## Troubleshooting

| Problem | Fix |
| --- | --- |
| Docker not found | Install and start Docker Desktop |
| Port 6080 or 3000 busy | `docker compose --profile view down` |
| Missing meshes in RViz | `git lfs pull` |
| No `/dev/ttyACM*` | Check the USB cable. On Linux, join `dialout` and log in again |
| Hardware refused in Docker | Run hardware mode on native Ubuntu |
| Controllers not ready in the GUI | Wait for bringup and check the terminal logs |
| Mac fans spin up in view mode | Use `--no-rviz`. See [If the machine gets hot](#if-the-machine-gets-hot) |
| `colcon` or ROS missing on Mac | Use Docker view mode |

## License

Apache-2.0. Arm description based on [SO-ARM100 / LeRobot SO-101](https://github.com/TheRobotStudio/SO-ARM100).

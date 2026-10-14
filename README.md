# TUWRC Basil Farm Robot Control

ROS 2 Jazzy workspace for the TUWRC basil-farm robot: a measured SO-101
six-servo arm on a 71.5 cm prismatic rail, with MoveIt, RViz, a browser GUI,
mock view mode, and native Ubuntu hardware control.

Repository: [janbocchino/tuwrc-basil-farm-robotcontrol](https://github.com/janbocchino/tuwrc-basil-farm-robotcontrol)

## Next steps

### Team Robot Control

- [ ]

### Team Machine Vision

- [ ]

## Quick start

### Choose a mode

- **View mode:** mock arm and rail movement in RViz and the browser GUI. Runs
  through Docker on macOS/Windows, or Docker/native ROS on Linux.
- **Hardware mode:** real six-servo arm and Arduino rail. Runs only on native
  Ubuntu 24.04 with ROS 2 Jazzy.

Hardware mode is intentionally refused in Docker Desktop. Use native Ubuntu
for USB control.

### 1. Get the repository

Install Git and Git LFS, then clone the complete workspace:

```bash
git lfs install
git clone git@github.com:janbocchino/tuwrc-basil-farm-robotcontrol.git
cd tuwrc-basil-farm-robotcontrol
git lfs pull
```

The LFS step downloads the robot meshes used by RViz.

### 2. Prepare the computer

#### macOS or Windows/WSL: Docker view mode

Install and start Docker Desktop. On Windows, enable WSL2 integration and
clone the repository inside WSL.

No local ROS installation is required.

#### Ubuntu 24.04: native view or hardware mode

Use the checked-in setup script instead of copying the package-install and
build commands one by one:

```bash
# Native mock/view setup
./tools/setup-linux --view

# Or real arm + rail setup
./tools/setup-linux --hardware
```

The script validates Ubuntu 24.04, installs ROS 2 Jazzy and workspace
dependencies, initializes Git LFS, resolves ROS dependencies, and builds with
`colcon`. It is safe to rerun after pulling dependency changes.

Useful setup options:

```bash
./tools/setup-linux --hardware --check   # report missing setup only
./tools/setup-linux --view --dry-run     # print commands without changing anything
./tools/setup-linux --hardware --yes     # skip the confirmation prompt
```

The hardware profile also installs the Python ST3215 library and adds the
current user to `dialout`. Log out and back in after the first `dialout`
change.

### 3. Start view mode

From the repository root:

```bash
./tools/run --mode view
```

On macOS and Windows this starts Docker. Open:

- RViz/noVNC desktop: [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html)
  (password: `ros`)
- Browser control GUI: [http://localhost:3000](http://localhost:3000)

On native Ubuntu:

```bash
./tools/run --mode view --runtime native
```

Stop with `Ctrl+C`. To stop Docker explicitly:

```bash
docker compose --profile view down
```

If Docker view mode uses too much CPU, keep the browser controls but disable
the 3D view:

```bash
./tools/run --mode view --no-rviz
```

Other useful options are `--no-moveit` and `--geometry 1280x720`.

### 4. Make the first view movement

Wait until the browser GUI reports that both controllers are ready:

1. Click **Fill current**.
2. Change one arm joint by a few degrees.
3. Click **Send**.
4. Confirm the model moves in RViz.

Then try a bounded scripted movement from a second terminal:

```bash
./tools/example small_arm_motion --return-home
./tools/example arm_reach_grip --release --return-home
./tools/example rail_reach_grip --rail-m 0.2 --release --return-home
```

`./tools/example` automatically uses the running Docker container or the
native ROS workspace.

### 5. Connect the real arm and rail

Hardware mode requires the `--hardware` setup profile above.

Connect the arm servo board and rail Arduino with data-capable USB cables,
then identify stable device paths:

```bash
ls -l /dev/serial/by-id/
```

The arm is usually `/dev/ttyACM*`; the rail is usually `/dev/ttyUSB*`.
Short device numbers can change after rebooting. Prefer the
`/dev/serial/by-id/...` paths.

Keep people and objects clear, then start bringup:

```bash
./tools/run --mode hardware --runtime native \
  --port /dev/serial/by-id/<arm-device> \
  --rail-port /dev/serial/by-id/<rail-device>
```

If more than one serial device is visible, the arm driver fails safely until
an explicit `--port` is supplied.

### 6. Calibrate the rail

Calibrate after every new bringup. The Arduino forgets its position whenever
it reconnects, and the driver rejects rail goals until calibration succeeds.

From a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /tuwrc_rail_driver/calibrate std_srvs/srv/Trigger
```

The rail moves to the left limit switch, measures the right limit, briefly
checks the middle, and returns to the left home position. Normal measured
travel is about 71.5 cm.

### What the `source` commands mean

```bash
source /opt/ros/jazzy/setup.bash
```

Loads the system ROS 2 Jazzy environment into the current terminal. It makes
commands such as `ros2` and the installed ROS packages available.

```bash
source install/setup.bash
```

Overlays the packages built from this repository. Run it after the Jazzy
setup command and after `colcon build`. Each new terminal needs these
environment settings; `tools/run` and `tools/example` source them
automatically.

### 7. Home the arm

After bringup and rail calibration, move the arm to the saved home pose:

```bash
./tools/example go_home --hardware --allow-hardware
```

Real motion examples require both `--hardware` and `--allow-hardware`. The
second flag is an explicit operator confirmation.

### 8. Make the first hardware movement

Start in the browser GUI with one joint and a small change. Confirm that the
real arm and RViz agree.

Then run:

```bash
./tools/example small_arm_motion \
  --hardware --allow-hardware --delta-deg 5 --return-home

./tools/example arm_reach_grip \
  --hardware --allow-hardware --release --return-home
```

Only command the rail after calibration:

```bash
./tools/example rail_reach_grip \
  --hardware --allow-hardware --rail-m 0.2 --release --return-home
```

## Important information and guides

### Home position

The canonical arm home pose is `HOME_ARM` in
`src/tuwrc_motion_examples/tuwrc_motion_examples/go_home.py`:

```text
1:  0.0107 rad
2: -1.5616 rad
3:  1.5447 rad
4:  0.0123 rad
5:  2.3363 rad
6: -0.6995 rad
```

It was measured from the physical arm using the current
`six_motor_calibration.yaml`. Re-measure it after changing servo calibration:

```bash
ros2 topic echo --once --field position /joint_states
```

Rail home is the left limit switch (`rail_joint = 0.0 m`) established by the
calibration service.

### Calibration and safety

- Arm calibration lives in
  `src/six_motor_driver/config/six_motor_calibration.yaml`.
- Model/planning limits live in `so101_base.xacro` and
  `six_motor_moveit_config/config/joint_limits.yaml`.
- Do not change measured limits, zero positions, or directions without a
  documented physical check. See [CONTRIBUTING.md](CONTRIBUTING.md).
- Begin with one joint, a small delta, and a clear workspace.
- The arm driver honors trajectory timing and rejects out-of-calibration
  targets.
- The GUI validates joint names, limits, and durations, but the hardware
  driver remains the final safety boundary.

### Rail calibration and model

- `rail_joint = 0.0 m` is the left switch (`lm_left`).
- Positive movement goes toward the right switch (`lm_right`).
- Expected travel is 0–0.715 m.
- The rail firmware is `firmware/rail_stepper/rail_stepper.ino`.
- Firmware and driver communicate at 115200 baud using `ST ...` status lines.
- Restart bringup after changing model limits; RViz and MoveIt read them at
  startup.
- Rail-to-arm mount offsets in `so101_base.xacro` remain provisional until
  checked against assembled CAD.

Rail status topics:

- `/rail/position_cm` — current position in centimetres
- `/rail/lm_left` and `/rail/lm_right` — limit switch state
- `/joint_states` — arm joints and `rail_joint`

### Control contract

Both modes expose the same ROS interfaces:

- Arm and gripper action:
  `/six_motor_controller/follow_joint_trajectory`
- Rail action: `/rail_controller/follow_joint_trajectory`
- Joint feedback: `/joint_states`
- MoveIt IK: `/compute_ik`, group `arm`, chain `world` → `tcp`

View mode provides mock action servers. Hardware mode uses the ST3215 arm
driver and Arduino rail driver.

### Repository map

```text
tuwrc-basil-farm-robotcontrol/
├── CONTRIBUTING.md                PR checks and calibration rules
├── Dockerfile
├── docker-compose.yml
├── firmware/rail_stepper/         Arduino rail firmware
├── requirements-hardware.txt      physical arm Python dependency
├── tests/                         offline contracts and helper tests
├── tools/
│   ├── setup-linux                one-time Ubuntu setup
│   ├── run                        canonical bringup command
│   └── example                    motion example runner
└── src/
    ├── lerobot_description/       URDF/xacro, meshes, RViz
    ├── lerobot_gui/               browser GUI
    ├── six_motor_driver/          ST3215 driver + calibration
    ├── six_motor_moveit_config/   MoveIt/SRDF/controller config
    ├── tuwrc_bringup/             unified view/hardware launch
    ├── tuwrc_mock_hardware/       mock controllers + rail driver
    └── tuwrc_motion_examples/     bounded movement examples
```

The canonical launch is
`src/tuwrc_bringup/launch/robot.launch.py`; use it through `tools/run`.

### Troubleshooting

- **Docker is missing:** install/start Docker Desktop or Docker Engine.
- **Port 6080 or 3000 is busy:** run
  `docker compose --profile view down`.
- **RViz meshes are missing:** run `git lfs pull`.
- **ROS/colcon is missing on Ubuntu:** rerun
  `./tools/setup-linux --view` or `--hardware`.
- **USB permission denied:** verify `dialout` membership and log out/in.
- **Serial device is ambiguous:** pass stable `--port` and `--rail-port`
  paths from `/dev/serial/by-id/`.
- **Rail goal is rejected:** call the calibration service after bringup.
- **Controllers are not ready:** inspect the bringup terminal before sending
  movement.
- **Docker view mode is hot:** use `--no-rviz`.

## Git workflow

Push the whole repository, not only `src/`. Commit source, launch/config,
tests, documentation, Docker files, firmware, tools, and meshes through Git
LFS. Do not commit `build/`, `install/`, `log/`, IDE settings, or `.env`.

Before opening a pull request, run the checks in
[CONTRIBUTING.md](CONTRIBUTING.md). Use focused branches such as
`feature/<name>`, `fix/<name>`, or `docs/<name>`.

## Update log

### 2026-10-14 — repository cleanup

- Reorganized this README around setup, calibration, home, and first motion.
- Added `tools/setup-linux` for repeatable Ubuntu view/hardware setup.
- Fixed merged arm/rail joint-state handling and hardware trajectory timing.
- Added offline helper tests.
- Removed superseded rail-hold, three-motor, Gazebo, and duplicate MoveIt
  paths; unified bringup remains canonical.

### 2025-10-03 — first hardware bringup

- Connected the SO-101 arm and Arduino rail over USB.
- Measured 1000 rail steps per centimetre.
- Added the six-motor hardware driver and real rail driver.
- Added explicit arm/rail serial-port selection.
- Changed the MoveIt arm chain to `world` → `tcp` so planning includes the
  rail.

## License

Apache-2.0. Arm description based on
[SO-ARM100 / LeRobot SO-101](https://github.com/TheRobotStudio/SO-ARM100).

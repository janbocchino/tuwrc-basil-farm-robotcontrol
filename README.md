# TUWRC Basil Farm Robot Control

ROS 2 Jazzy workspace for the TUWRC basil-farm robot: a measured SO-101
six-servo arm on a 71.5 cm prismatic rail, with MoveIt, RViz, a browser GUI,
mock view mode, and native Ubuntu hardware control.

Repository: [janbocchino/tuwrc-basil-farm-robotcontrol](https://github.com/janbocchino/tuwrc-basil-farm-robotcontrol)

## 1. Get the repository

Install Git and Git LFS, then clone the complete workspace:

```bash
git lfs install
git clone git@github.com:janbocchino/tuwrc-basil-farm-robotcontrol.git
cd tuwrc-basil-farm-robotcontrol
git lfs pull
```

The LFS step downloads the robot meshes used by RViz.

## 2. One-Time-Setup for Linux without docker (Ubuntu only)

Choose the command according to rather you will connect to hardware or not with your device.

IMPORTANT: After the first `--hardware` run, log out and back in so `dialout` applies.

```bash
./tools/setup-linux --view       # Use if you have Linux and will never connect to hardware
./tools/setup-linux --hardware   # Use if you connect to real hardware (e.g. our PC at the hub)
```

The script installs ROS 2 Jazzy and the workspace dependencies, initializes
Git LFS, and builds with `colcon`.

| Flag           | Meaning                                                         |
| -------------- | --------------------------------------------------------------- |
| `--view`       | Native mock controllers.                                        |
| `--hardware`   | View packages, plus the ST3215 library and the `dialout` group. |
| `--check`      | Report missing packages. Changes nothing.                       |
| `--dry-run`    | Prints needed install commands (doesn't execute them)           |
| `--yes`        | Skip the confirmation prompt.                                   |
| `-h`, `--help` | Shows this comand list list.                                    |

## 3.1 View mode

Mock arm and rail in RViz and the browser

Runs through Docker on macOS, Windows or Linux, or through native ROS on Ubuntu. No USB hardware.

### Prepare

On macOS or Windows, install and start Docker Desktop. On Windows, enable
WSL2 integration and clone the repository inside WSL. No local ROS install
is required.

On Ubuntu 24.04, run `./tools/setup-linux --view` from the table above.

### Start

From the repository root:

```bash
./tools/run --mode view
```

On macOS and Windows have Docker Desktop open before running. Open:

- RViz/noVNC desktop: [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html)
(password: `ros`)
- Browser control GUI: [http://localhost:3000](http://localhost:3000)

Stop with `Ctrl+C`. To stop Docker explicitly:

```bash
docker compose --profile view down
```

| Flag                      | Default                              | Meaning                                        |
| ------------------------- | ------------------------------------ | ---------------------------------------------- |
| `--mode view`             | `view`                               | Mock arm and rail.                             |
| `--runtime native/docker` | Docker on macOS/WSL, native on Linux | chooses the environment                        |
| `--no-rviz`               | RViz on                              | Skip RViz. Largest CPU saving in Docker.       |
| `--no-moveit`             | MoveIt on                            | Skip MoveIt. RViz shows the plain robot model. |
| `--no-gui`                | GUI on                               | Skip the browser GUI.                          |
| `--geometry WxH`          | `1600x900`                           | noVNC desktop size. Docker only.               |
| `-h`, `--help`            |                                      | Show the full `./tools/run` list.              |

### First movement

Wait until the browser GUI reports that both controllers are ready:

1. Click **Fill current**.
2. Change one arm joint by a few degrees.
3. Click **Send**.
4. Confirm the model moves in RViz.

Then run a command from [Motion examples](#motion-examples) in a second
terminal.

## 3.2 Hardware mode

Real arm and rail on native Ubuntu 24.04 only. Hardware mode is refused in Docker and on macOS.

### Prepare

Run `./tools/setup-linux --hardware`. See [Ubuntu setup](#ubuntu-setup).

Connect the arm servo board and rail Arduino with data-capable USB cables, then identify their device paths:

```bash
ls -l /dev/serial/by-id/
```

The arm is usually `/dev/ttyACM*` and the rail is usually `/dev/ttyUSB*`.
Short device numbers can change after rebooting. Prefer the
`/dev/serial/by-id/...` paths.

### Start

Start bringup with the following command and add the correct ports to it:

```bash
./tools/run --mode hardware # default bringup

# or with new ports
./tools/run --mode hardware \
  --port /dev/serial/by-id/<arm-device> \
  --rail-port /dev/serial/by-id/<rail-device>
```

| Flag                    | Default         | Meaning                                                |
| ----------------------- | --------------- | ------------------------------------------------------ |
| `--mode hardware`       |                 | Real six-servo arm and Arduino rail.                   |
| `--runtime native`      | native on Linux | Required for USB. Docker is refused.                   |
| `--port PATH`           | `/dev/ttyACM0`  | Arm serial device.                                     |
| `--rail-port PATH`      | `/dev/ttyUSB1`  | Rail Arduino.                                          |
| `--camera`              | off             | Gripper-mounted Gemini 2. Requires hardware mode.      |
| `--colored-point-cloud` | off             | Also publish the RGB point cloud. Turns `--camera` on. |

 `--no-rviz`, `--no-moveit`, `--no-gui`, and `--runtime` from [View mode](#view-mode) work here too.

Parsed in `tools/run` and forwarded to
`src/tuwrc_bringup/launch/robot.launch.py`.

### Calibrate the rail

Calibrate after every new bringup. The Arduino forgets its position whenever
it reconnects, and the driver rejects rail goals until calibration succeeds.

`tools/run` and `tools/example` load ROS on their own. This service call is
typed directly, so the terminal needs the two `source` lines first. The
first loads system ROS 2 Jazzy. The second overlays this workspace. Each new
terminal needs both, in this order, after `colcon build`.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /tuwrc_rail_driver/calibrate std_srvs/srv/Trigger
```

The arm moves to the left and right limit switches on the rail, measures the distance and coordinates, briefly checks the middle, and returns to the left home position. Normal measured travel is about 71.5 cm.

### First movement

Start in the browser GUI with one joint and a small change. Confirm that
the real arm and RViz agree. Then use the same commands as
[Motion examples](#motion-examples).

## 4. Motion examples

Bringup must already be running. `./tools/example` uses the running Docker
container or the native ROS workspace, then runs a script from
`tuwrc_motion_examples`. The script moves the mock robot or the real robot
according to the mode `./tools/run` started.

```bash
./tools/example small_arm_motion --return-home
./tools/example arm_reach_grip --release --return-home
./tools/example rail_reach_grip --rail-m 0.2 --release --return-home
./tools/example go_home
```

| Script | Motion | Flags |
| --- | --- | --- |
| `go_home` | Arm to the saved home pose. | `--duration` (default 4 s) |
| `small_arm_motion` | Small nudge from the current joints. | `--duration` (2 s), `--delta-deg` (±20°), `--return-home` |
| `arm_reach_grip` | Reach forward and close the gripper. Rail stays put. | `--scale` (0.5), `--duration-scale`, `--release`, `--return-home` |
| `rail_reach_grip` | Move the rail, then the same reach and grip. | `--rail-m` required (0–0.715), plus the `arm_reach_grip` flags |
| `leaf_pick_imitation` | Front reach and snip. Add `--move-rail` to move the rail first. | `--rail-m` (0.15), `--scale` (1.0), `--duration-scale`, `--return-home` |

Shared flags: `--scale` (0.2–1.0) shrinks joints 1–5 toward zero, `--return-home` goes back to the pose at the start of the script, and `--release` opens the gripper after the retract. `0` on `--rail-m` is the left switch. `./tools/example <script> --help` prints that script's flags. Sources are in `src/tuwrc_motion_examples/tuwrc_motion_examples/`.

## Repository map

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
    └── tuwrc_motion_examples/     movement examples
```

The canonical launch is
`src/tuwrc_bringup/launch/robot.launch.py`; use it through `tools/run`.

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
calibration service. `./tools/example go_home` moves the arm to this pose.

### Calibration and safety

- Arm calibration lives in
`src/six_motor_driver/config/six_motor_calibration.yaml`.
- Model/planning limits live in `so101_base.xacro` and  
`six_motor_moveit_config/config/joint_limits.yaml`.

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

## License

Apache-2.0. Arm description based on
[SO-ARM100 / LeRobot SO-101](https://github.com/TheRobotStudio/SO-ARM100).
# tuwrc-basil-farm-robotcontrol

GitHub: [https://github.com/janbocchino/tuwrc-basil-farm-robotcontrol](https://github.com/janbocchino/tuwrc-basil-farm-robotcontrol)

One shared ROS 2 project for the TUWRC basil-farm robot: Max’s measured SO-101 arm on the prismatic X-rail, with RViz, MoveIt, a browser GUI, mock view mode, and real arm hardware support on Linux.

## What this project does


| Mode         | What you get                                   | Who should use it                    |
| ------------ | ---------------------------------------------- | ------------------------------------ |
| **view**     | Simulated joint motion in RViz + browser GUI   | Everyone (macOS, Windows/WSL, Linux) |
| **hardware** | Real six-servo arm over USB + rail held at 0 m | Native Ubuntu (recommended)          |


Gazebo is intentionally **not** required for the MVP. The physical rail motor is also **not** supported yet; in hardware mode the rail stays fixed at zero.

## Prerequisites



### All platforms

- Git
- Git LFS (`git lfs install`)
- Docker Desktop or Docker Engine (**required on macOS / Windows**; optional on Linux)



### Native Ubuntu 24.04 (hardware and native view)

- ROS 2 **Jazzy**
- Packages used by this workspace: `rviz2`, `xacro`, `robot-state-publisher`, `moveit`, `control-msgs`, `python3-serial`, `python3-yaml`, `python3-colcon-common-extensions`
- User in the `dialout` group for USB serial



## Quick start



### macOS

1. Install Docker Desktop and start it.
2. Clone this repository (whole repo, not only `src/`).
3. Install Git LFS once: `git lfs install && git lfs pull`
4. Start view mode:

```bash
cd tuwrc-basil-farm-robotcontrol
./tools/run --mode view
```

1. Open:
  - RViz desktop: [http://localhost:6080/vnc.html](http://localhost:6080/vnc.html) (password: `ros`)
  - Browser GUI: [http://localhost:3000](http://localhost:3000)

Stop with `Ctrl+C` in the terminal, or:

```bash
docker compose --profile view down
```

#### If your machine gets hot

Docker on macOS and Windows has no GPU to pass through, so RViz is rendered
entirely on the CPU. RViz is by far the most expensive process in view mode —
it redraws continuously even when the robot is still and even when no browser
is connected to noVNC. The other nodes together use only a few percent.

Measured on an M-series Mac (10-CPU Docker VM), idle and stationary:

| Command                                | Container CPU |
| -------------------------------------- | ------------- |
| `./tools/run --mode view`              | ~140 %        |
| `./tools/run --mode view --no-moveit`  | ~57 %         |
| `./tools/run --mode view --no-rviz`    | ~10 %         |

`--no-rviz` keeps the browser GUI at [http://localhost:3000](http://localhost:3000)
fully working — joint control, the end-effector pose readout and MoveIt IK all
still function. You only lose the 3D view. This is the recommended way to work
when you do not need to watch the robot.

Further knobs (environment variables, all optional):

| Variable              | Default     | Effect                                                     |
| --------------------- | ----------- | ---------------------------------------------------------- |
| `TUWRC_VNC_GEOMETRY`  | `1600x900`  | noVNC desktop size; smaller = less to rasterize and encode |
| `LP_NUM_THREADS`      | `4`         | Caps the software renderer's threads                       |
| `TUWRC_MOCK_RATE`     | `20.0`      | `/joint_states` publish rate in Hz (view mode)             |
| `TUWRC_VNC_SESSION`   | `light`     | `full` gives the complete XFCE desktop inside noVNC        |

`--geometry` on `./tools/run` sets `TUWRC_VNC_GEOMETRY` for you.



### Windows (WSL2)

1. Install Docker Desktop with WSL2 integration, or Docker Engine inside WSL.
2. Clone the repository inside WSL.
3. `git lfs install && git lfs pull`
4. Run:

```bash
./tools/run --mode view
```

1. Open the same URLs as macOS from the Windows browser (`localhost:6080` and `:3000`).

USB hardware through WSL (`usbipd-win`) is **experimental** and not the default path.

### Native Ubuntu 24.04

View mode without Docker:

```bash
source /opt/ros/jazzy/setup.bash
cd tuwrc-basil-farm-robotcontrol
colcon build --symlink-install
source install/setup.bash
./tools/run --mode view --runtime native
```

Hardware mode (real arm):

```bash
# one-time
sudo usermod -aG dialout "$USER"   # then log out/in
ls /dev/ttyACM* /dev/ttyUSB*

./tools/run --mode hardware --runtime native --port /dev/ttyACM0
```

Then open the GUI at [http://localhost:3000](http://localhost:3000). RViz opens as a normal Linux window.

## Safe first movements

In **view** mode, use the browser GUI:

1. Wait until status shows controllers ready.
2. Click **Fill current**.
3. Change one arm joint by a few degrees.
4. Click **Send**.
5. Watch RViz update.

Optional scripted motion (with bringup already running):

```bash
# macOS / Windows (Docker view mode)
docker compose exec robot bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && \
   ros2 run tuwrc_motion_examples small_arm_motion -- --return-home'

# Native Ubuntu — view / mock
ros2 run tuwrc_motion_examples small_arm_motion -- --return-home

# Native Ubuntu — real hardware (extra confirmation required)
ros2 run tuwrc_motion_examples small_arm_motion -- \
  --hardware --allow-hardware --return-home
```



## Repository map

```text
tuwrc-basil-farm-robotcontrol/
├── README.md                 ← you are here
├── CONTRIBUTING.md           ← branch/PR rules
├── Dockerfile                ← ROS 2 Jazzy + noVNC image
├── docker-compose.yml
├── docker/                   ← container entrypoint
├── tools/run                 ← cross-platform start script
├── tests/                    ← offline contract tests
└── src/
    ├── lerobot_description/  ← URDF/xacro, meshes, RViz display config
    ├── six_motor_driver/     ← real ST3215 arm driver + calibration
    ├── six_motor_moveit_config/ ← MoveIt planning / controllers / RViz
    ├── lerobot_gui/          ← browser UI (port 3000)
    ├── tuwrc_bringup/        ← unified launch: view | hardware
    ├── tuwrc_mock_hardware/  ← mock arm+rail (view) and rail_hold (hardware)
    └── tuwrc_motion_examples/← small_arm_motion example
```

Important files:


| Path                                                     | Purpose                   |
| -------------------------------------------------------- | ------------------------- |
| `src/lerobot_description/urdf/so101_base.xacro`          | Rail + measured arm model |
| `src/lerobot_description/meshes/`                        | STL meshes (Git LFS)      |
| `src/six_motor_driver/config/six_motor_calibration.yaml` | Real-robot calibration    |
| `src/tuwrc_bringup/launch/robot.launch.py`               | Main launch file          |
| `src/lerobot_gui/lerobot_gui/joint_state_gui.py`         | Browser GUI               |
| `tools/run`                                              | OS-aware launcher         |




## Modes and control contract

Both modes use the same actions:

- Arm/gripper: `/six_motor_controller/follow_joint_trajectory` (joints `1`–`6`)
- Rail: `/rail_controller/follow_joint_trajectory` (joint `rail_joint`)
- Feedback: `/joint_states`
- Pose IK (optional): MoveIt `/compute_ik`, group `arm`, frames `world` → `tcp`


| Mode     | Arm controller        | Rail controller         |
| -------- | --------------------- | ----------------------- |
| view     | `tuwrc_mock_hardware` | mock (movable ±0.5 m)   |
| hardware | `six_motor_driver`    | `rail_hold` (fixed 0 m) |




## Git and GitHub workflow

**Push the whole repository**, not only `src/`. Docker files, `tools/`, README, and manifests are required for teammates on other OSes.

### Clone

```bash
git lfs install
git clone git@github.com:janbocchino/tuwrc-basil-farm-robotcontrol.git
cd tuwrc-basil-farm-robotcontrol
git lfs pull
```



### Daily work

```bash
git checkout main
git pull
git checkout -b feature/short-description
# edit files…
git status
git add -A
git commit -m "Describe why the change exists"
git push -u origin HEAD
```

Then open a pull request on GitHub.

### What to commit / ignore

Commit:

- source under `src/`
- launch/config/calibration
- meshes (via Git LFS)
- tests, docs, Docker, `tools/`

Do **not** commit (already in `.gitignore`):

- `build/`, `install/`, `log/`
- local IDE settings, `.env`, temporary files



### Git LFS

Meshes are large STL files. After cloning on a new machine:

```bash
git lfs install
git lfs pull
```

If RViz shows missing meshes, you probably forgot `git lfs pull`.

## Calibration and safety

- Measured joint limits and zero positions live in `six_motor_calibration.yaml`.
- Do **not** change calibration or URDF joint limits without a documented physical verification.
- Use tiny deltas first.
- Hardware scripts require `--allow-hardware`.
- Rail physical motion is disabled until a real rail driver exists.



## Deferred / later TODOs

- Gazebo simulation parity
- Accurate rail-to-arm geometry from the fully assembled CAD model (current rail placement is provisional; mount args are in `so101_base.xacro`)
- Real rail motor/encoder/homing/E-stop driver
- Perfect visual seating of the arm on the slider



## Troubleshooting


| Problem                       | Fix                                                                         |
| ----------------------------- | --------------------------------------------------------------------------- |
| Docker not found              | Install/start Docker Desktop                                                |
| Port 6080/3000 busy           | Stop old containers: `docker compose --profile view down`                   |
| Missing meshes in RViz        | `git lfs pull`                                                              |
| No `/dev/ttyACM*`             | Check USB cable; on WSL use `usbipd`; on Linux join `dialout`               |
| Hardware refused in Docker    | Expected — use native Ubuntu                                                |
| Controllers not ready in GUI  | Wait for bringup; check terminal logs                                       |
| Mac fans spin up in view mode | Expected — RViz renders on the CPU. Use `--no-rviz`; see "If your machine gets hot" |
| `colcon` / ROS missing on Mac | Use Docker view mode; do not install ROS natively on macOS for this project |




## License

Apache-2.0. Arm description based on [SO-ARM100 / LeRobot SO-101](https://github.com/TheRobotStudio/SO-ARM100).
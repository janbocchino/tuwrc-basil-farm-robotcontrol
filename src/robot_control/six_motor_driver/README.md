# Six-motor ST3215 driver

ROS 2 hardware driver and read-only utilities for the six SO-101 bus servos.
The canonical startup path is the repository-level bringup:

```bash
./tools/setup-linux --hardware
./tools/run --mode hardware --runtime native \
  --port /dev/serial/by-id/<arm-device> \
  --rail-port /dev/serial/by-id/<rail-device>
```

The driver exposes
`/six_motor_controller/follow_joint_trajectory`, publishes arm joints on
`/joint_states`, validates goals against
`config/six_motor_calibration.yaml`, and honors trajectory
`time_from_start`.

## Utilities

- `show_six_positions`: print raw servo positions without commanding motion.
- `six_motor_position_reader`: publish passive position readings.
- `configure_servo_id`: assign an ID with one servo connected.
- `calibrate_six_endpoints`: assist endpoint measurement.
- `print_moveit_limits`: print model limits derived from calibration.

Source ROS and the workspace before calling utilities directly:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run six_motor_driver show_six_positions \
  --port /dev/serial/by-id/<arm-device>
```

Do not edit calibration values without a documented physical measurement.
See the root [README](../../../README.md) and
[CONTRIBUTING](../../../CONTRIBUTING.md).

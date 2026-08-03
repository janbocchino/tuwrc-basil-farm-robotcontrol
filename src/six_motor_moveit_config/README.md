# MoveIt configuration for the rail-mounted SO-101

Prefer the unified bringup from the repository root:

```bash
./tools/run --mode view
# or on native Ubuntu with the real arm:
./tools/run --mode hardware --runtime native --port /dev/ttyACM0
```

Direct MoveIt launch (legacy helper):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch six_motor_moveit_config moveit.launch.py port:=/dev/ttyACM0
```

## Important files

- `config/so101.srdf`: planning group `arm` (`rail_link` → `tcp`) plus gripper
- `config/joint_limits.yaml`: measured arm limits + provisional rail limits
- `config/controllers.yaml` / `controllers_real.yaml`: action controller wiring
- `launch/moveit.launch.py`: model, optional hardware driver, `move_group`, RViz

See the root [README.md](../../README.md) for OS-specific instructions.

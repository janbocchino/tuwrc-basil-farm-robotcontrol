# MoveIt configuration for the rail-mounted SO-101

Prefer the unified bringup from the repository root:

```bash
./tools/run --mode view
# or on native Ubuntu with the real arm:
./tools/run --mode hardware --runtime native \
  --port /dev/serial/by-id/<arm-device> \
  --rail-port /dev/serial/by-id/<rail-device>
```

## Important files

- `config/so101.srdf`: planning group `arm` (`world` → `tcp`) plus gripper
- `config/joint_limits.yaml`: measured arm limits + 0–0.715 m rail limits
- `config/controllers.yaml`: shared action-controller wiring
- `config/kinematics.yaml`: inverse-kinematics configuration
- `config/moveit.rviz`: MoveIt RViz layout

See the root [README.md](../../../README.md) for OS-specific instructions.

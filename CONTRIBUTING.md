# Contributing

## Branch naming

- `feature/<short-name>` for new work
- `fix/<short-name>` for bug fixes
- `docs/<short-name>` for documentation-only changes

Always branch from an up-to-date `main`.

## Pull requests

Before opening a PR:

1. `python3 tests/test_model_contract.py`
2. `python3 tests/test_gui_actions_contract.py`
3. On Ubuntu/Jazzy (or in Docker): `colcon build --symlink-install`
4. Smoke-test view mode with `./tools/run --mode view`

PR description should say:

- what changed
- how to test
- whether calibration/URDF measurements were touched

## Measurements and calibration

Do **not** change these without documented physical verification:

- joint origins / limits in `src/lerobot_description/urdf/so101_base.xacro` (arm section)
- `src/six_motor_driver/config/six_motor_calibration.yaml`
- MoveIt joint limits derived from those values

Rail mount offsets (`rail_mount_*` args) may be adjusted for visualization, but note that they are provisional until CAD data arrives.

## Meshes / Git LFS

- STL/mesh binaries are tracked with Git LFS (see `.gitattributes`)
- Never replace meshes with huge uncompressed CAD dumps without team agreement
- After clone: `git lfs pull`

## What belongs in the repository

Push the **entire** repository layout (README, Docker, tools, `src/`, tests).  
Do **not** push only `src/`.  
Do **not** commit `build/`, `install/`, or `log/`.

## Code style

- Prefer small, focused commits
- Keep the shared action names:
  - `/six_motor_controller/follow_joint_trajectory`
  - `/rail_controller/follow_joint_trajectory`
- Prefer the browser GUI + action clients over one-off topic publishers

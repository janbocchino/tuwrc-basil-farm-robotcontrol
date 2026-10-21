# Contributing

## Workstream ownership

Use the workstream folders to keep responsibilities clear:

| Workstream | Folder | Owns |
| --- | --- | --- |
| Machine Vision & Perception | `src/perception/` | camera processing, point clouds, plant readiness, cut-coordinate estimation |
| Robot Control & Motion Planning | `src/robot_control/` | drivers, rail/arm movement, and planning to a pre-cut pose |
| Manipulation | `src/manipulation/` | teleoperation, learned policies, final approach, and cutting execution |
| Shared system | `src/shared/` | robot description, ROS interfaces, and full-system bringup |

If a change crosses workstream boundaries, discuss the interface before
implementing it. Changes in `src/shared/` should be treated as high-impact
because they can affect all teams.

The first-level folders under `src/` organize ownership. Actual ROS packages
live below them and contain `package.xml`.

## Branch naming

Always branch from an up-to-date `main`.

Use short-lived branches with a clear prefix:

- `perception/<short-name>` for Machine Vision & Perception work
- `robot-control/<short-name>` for Robot Control & Motion Planning work
- `manipulation/<short-name>` for Manipulation work
- `fix/<short-name>` for bug fixes
- `docs/<short-name>` for documentation-only changes

Examples:

- `perception/stem-cutpoint-detector`
- `robot-control/precut-pose-planner`
- `manipulation/teleop-policy-runner`
- `docs/pr-workflow`

## Pull requests

All completed work should be merged through a pull request into `main`.
Do not push directly to `main`.

Before opening a PR:

1. `python3 tests/test_model_contract.py`
2. `python3 tests/test_gui_actions_contract.py`
3. `python3 tests/test_camera_contract.py`
4. `python3 tests/test_runtime_helpers.py`
5. On Ubuntu/Jazzy (or in Docker): `colcon build --symlink-install`
6. Smoke-test view mode with `./tools/run --mode view`

PR description should say:

- what changed
- how to test
- which workstream owns the change
- whether `src/shared/` changed
- whether calibration/URDF measurements were touched

Keep PRs focused. A perception detector, a robot-control planner, and a
manipulation policy should usually be separate PRs unless they are part of the
same small integration change.

Use squash merge by default so `main` stays readable.

## Review and merge rules

- `main` should always build and be usable by the team.
- Require at least one approval from the repository owner before merging.
- Require passing tests/build checks before merging when CI is available.
- Request review from the affected workstream lead when a PR touches their
  folder.
- Request extra review for `src/shared/`, calibration, or hardware-control
  changes.
- Prefer small PRs over large mixed changes.

## Measurements and calibration

Do **not** change these without documented physical verification:

- joint origins / limits in
  `src/shared/lerobot_description/urdf/so101_base.xacro` (arm section)
- `src/robot_control/six_motor_driver/config/six_motor_calibration.yaml`
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
Do **not** commit large bag files, datasets, point-cloud dumps, or model
weights directly. Use Git LFS or external storage with documented download
steps.

## Code style

- Prefer small, focused commits
- Keep the shared action names:
  - `/six_motor_controller/follow_joint_trajectory`
  - `/rail_controller/follow_joint_trajectory`
- Prefer the browser GUI + action clients over one-off topic publishers

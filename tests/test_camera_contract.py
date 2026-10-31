#!/usr/bin/env python3
"""Offline contract checks for the physical Gemini 2 integration."""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CAMERA_LAUNCH = ROOT / "src/shared/tuwrc_bringup/launch/gemini2.launch.py"
ROBOT_LAUNCH = ROOT / "src/shared/tuwrc_bringup/launch/robot.launch.py"
PACKAGE_XML = ROOT / "src/shared/tuwrc_bringup/package.xml"
RUNNER = ROOT / "tools/run"
REPOS = ROOT / "dependencies/orbbec.repos"
CAMERA_URDF = (
    ROOT / "src/shared/lerobot_description/urdf/gemini2_camera.xacro"
)
ROBOT_URDF = ROOT / "src/shared/lerobot_description/urdf/so101_base.xacro"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def require_tokens(text: str, tokens: tuple[str, ...], label: str) -> None:
    for token in tokens:
        if token not in text:
            fail(f"{label} missing expected token: {token}")


def main() -> None:
    camera_launch = CAMERA_LAUNCH.read_text(encoding="utf-8")
    robot_launch = ROBOT_LAUNCH.read_text(encoding="utf-8")
    runner = RUNNER.read_text(encoding="utf-8")
    camera_urdf = CAMERA_URDF.read_text(encoding="utf-8")
    robot_urdf = ROBOT_URDF.read_text(encoding="utf-8")

    # Syntax checks do not require ROS Python packages to be installed.
    ast.parse(camera_launch, filename=str(CAMERA_LAUNCH))
    ast.parse(robot_launch, filename=str(ROBOT_LAUNCH))

    require_tokens(
        camera_launch,
        (
            "gemini2.launch.py",
            '"enable_point_cloud": "true"',
            '"depth_registration": "true"',
            '"enable_colored_point_cloud"',
            '"publish_tf": "true"',
        ),
        "Gemini 2 launch wrapper",
    )
    if "gazebo" in camera_launch.lower() or "ros_gz" in camera_launch:
        fail("physical camera launch must not depend on Gazebo")

    require_tokens(
        camera_urdf,
        (
            'link name="camera_link"',
            'joint name="gripper_to_camera" type="fixed"',
            '<parent link="${parent}"',
            '<child link="camera_link"',
            '<box size="0.025 0.090 0.030"',
        ),
        "Gemini 2 model",
    )
    require_tokens(
        robot_urdf,
        (
            "gemini2_camera.xacro",
            "camera_mount_x",
            "camera_mount_y",
            "camera_mount_z",
            'parent="gripper"',
        ),
        "robot camera mount",
    )

    require_tokens(
        robot_launch,
        (
            '"use_camera"',
            "use_camera=true requires mode=hardware",
            '"camera_serial"',
            '"camera_usb_port"',
        ),
        "unified robot launch",
    )
    require_tokens(
        runner,
        (
            "--camera",
            "--colored-point-cloud",
            'use_camera:="$USE_CAMERA"',
            'enable_colored_point_cloud:="$ENABLE_COLORED_POINT_CLOUD"',
        ),
        "tools/run",
    )

    package_xml = PACKAGE_XML.read_text(encoding="utf-8")
    if "<exec_depend>orbbec_camera</exec_depend>" not in package_xml:
        fail("tuwrc_bringup must declare its orbbec_camera runtime dependency")

    repos = REPOS.read_text(encoding="utf-8")
    require_tokens(repos, ("OrbbecSDK_ROS2.git", "version:"), "Orbbec repos file")
    match = re.search(r"version:\s*([0-9a-f]{40})\s*$", repos, re.MULTILINE)
    if not match:
        fail("Orbbec source must be pinned to an immutable 40-character commit")

    print("OK: Gemini 2 camera contracts passed")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Offline checks that do not require a live ROS graph."""

from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "src/lerobot_description/urdf/so101_base.xacro"
SRDF = ROOT / "src/six_motor_moveit_config/config/so101.srdf"
LIMITS = ROOT / "src/six_motor_moveit_config/config/joint_limits.yaml"
CALIB = ROOT / "src/six_motor_driver/config/six_motor_calibration.yaml"


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def main() -> None:
    text = URDF.read_text(encoding="utf-8")
    for token in (
        'name="rail_joint"',
        'name="tcp"',
        'name="carriage_link"',
        'name="rail_link"',
        'rail_mount_x',
        'filename="package://lerobot_description/meshes/rail/rail.stl"',
        'filename="package://lerobot_description/meshes/rail/plate.stl"',
    ):
        if token not in text:
            fail(f"URDF missing expected token: {token}")

    # Max measured limits for joint 1 must remain.
    if "lower=\"-2.06780610\"" not in text or "upper=\"2.06780610\"" not in text:
        fail("Measured joint 1 limits from Max are missing")

    # Ensure arm is mounted on carriage, not directly on world.
    if re.search(r'<joint name="base_joint"[^>]*>\s*<parent link="world"', text):
        fail("base_joint must attach to carriage_link, not world")
    if 'parent link="carriage_link"' not in text:
        fail("base_joint parent carriage_link missing")

    srdf = SRDF.read_text(encoding="utf-8")
    if 'base_link="rail_link"' not in srdf or 'tip_link="tcp"' not in srdf:
        fail("SRDF arm chain must be rail_link → tcp")
    if 'name="rail_joint"' not in srdf:
        fail("SRDF must mention rail_joint")

    limits = LIMITS.read_text(encoding="utf-8")
    if "rail_joint:" not in limits:
        fail("joint_limits.yaml missing rail_joint")

    calib = CALIB.read_text(encoding="utf-8")
    for name in ("'1'", "'2'", "'3'", "'4'", "'5'", "'6'"):
        if name not in calib:
            fail(f"calibration missing joint name {name}")

    # Mesh presence
    for relative in (
        "src/lerobot_description/meshes/rail/rail.stl",
        "src/lerobot_description/meshes/rail/plate.stl",
        "src/lerobot_description/meshes/so101/base_so101_v2.stl",
    ):
        path = ROOT / relative
        if not path.is_file() or path.stat().st_size < 100:
            fail(f"Missing or tiny mesh: {relative}")

    # Ensure package manifests parse as XML.
    for package_xml in ROOT.glob("src/*/package.xml"):
        ET.parse(package_xml)

    print("OK: model contract checks passed")


if __name__ == "__main__":
    main()

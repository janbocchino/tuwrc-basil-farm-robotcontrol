#!/usr/bin/env python3
"""Print URDF and MoveIt limits derived from cyclic servo calibration."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

from .cyclic_calibration import CyclicJointCalibration


def default_config_path() -> Path:
    source_path = (
        Path.home()
        / "ros2_ws"
        / "src"
        / "six_motor_system"
        / "six_motor_driver"
        / "config"
        / "six_motor_calibration.yaml"
    )
    if source_path.is_file():
        return source_path
    from ament_index_python.packages import get_package_share_directory

    return (
        Path(get_package_share_directory("six_motor_driver"))
        / "config"
        / "six_motor_calibration.yaml"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Berechnet lineare URDF-/MoveIt-Grenzen aus den zyklischen "
            "ST3215-Rohwertgrenzen."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
    )
    args = parser.parse_args()

    document = yaml.safe_load(args.config.read_text())
    node_config = document.get("/**") or document.get(
        "six_motor_position_reader"
    )
    parameters = node_config["ros__parameters"]
    names = parameters["names"]
    calibrations = [
        CyclicJointCalibration(lower, upper, zero, direction)
        for lower, upper, zero, direction in zip(
            parameters["lower_limits_steps"],
            parameters["upper_limits_steps"],
            parameters["zero_positions_steps"],
            parameters["directions"],
        )
    ]

    print("# URDF-Grenzen")
    for name, calibration in zip(names, calibrations):
        lower, upper = calibration.angle_limits()
        print(
            f'<limit lower="{lower:.8f}" upper="{upper:.8f}" '
            'effort="4.0" velocity="0.8"/>'
            f"  <!-- {name} -->"
        )

    print("\n# MoveIt joint_limits.yaml")
    print("joint_limits:")
    for name, calibration in zip(names, calibrations):
        lower, upper = calibration.angle_limits()
        print(f"  {name}:")
        print("    has_position_limits: true")
        print(f"    min_position: {lower:.8f}")
        print(f"    max_position: {upper:.8f}")
        print("    has_velocity_limits: true")
        print("    max_velocity: 0.8")
        print("    has_acceleration_limits: true")
        print("    max_acceleration: 1.5")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

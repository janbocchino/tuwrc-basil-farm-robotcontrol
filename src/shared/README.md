# Shared System Packages

This directory contains contracts and integration packages used across
Machine Vision & Perception, Robot Control & Motion Planning, and
Manipulation.

- `lerobot_description/` defines shared geometry, coordinate frames, joint
  names, meshes, and camera/tool mounting.
- `tuwrc_interfaces/` is reserved for shared ROS messages, services, and
  actions.
- `tuwrc_bringup/` launches and integrates the complete system.

Changes here can affect every workstream and need cross-team review.

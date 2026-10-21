# TUWRC Basil Farm Robot — Team Onboarding

Welcome to the TUWRC basil-farm robot project. This document gives new team
members a high-level view of the project: what we are building, what already
works, where the main parts live, and what needs to happen next.

For installation commands and detailed operating instructions, use the
[README](README.md). For development rules and required checks, use
[CONTRIBUTING.md](CONTRIBUTING.md).

## The goal

We are building an autonomous basil farm. The long-term system should monitor
the growing environment, understand plant condition, decide when work is
needed, carry out that work, verify the result, and keep records with minimal
human intervention.

The rail-mounted robot in this repository is one part of that larger system.
It provides the movement, sensing, planning, and manipulation platform needed
to inspect and care for plants. Individual actions may include monitoring,
targeted treatment, pruning, or harvesting, depending on the farm task being
developed.

The platform combines:

- a rail that moves the robot along the growing area;
- a six-servo SO-101 arm;
- a gripper at the end of the arm;
- an eye-in-hand Gemini 2 depth camera;
- motion-planning and control software built on ROS 2;
- a browser interface and 3D view for operators and developers.

The long-term objective is a closed autonomous farming cycle:

1. observe the plants and growing environment;
2. assess plant state and identify required work;
3. choose and plan an appropriate task;
4. perform the task safely;
5. verify and record the result;
6. continue monitoring and adapt future decisions.

Today, this repository provides much of the robot movement and hardware
foundation for that cycle. It does not yet provide farm-level decision-making
or a complete autonomous plant-care workflow.

## Where we are now

The project currently has a working robot-control stack for development and
hardware bringup. This is an enabling subsystem of the autonomous farm rather
than the complete farm system.

### What works

- The arm and rail are represented together as one robot.
- The robot can be viewed and moved without physical hardware.
- The real arm and rail can be controlled on an Ubuntu computer.
- The rail can calibrate itself using its limit switches.
- Arm and rail positions are reported back to the rest of the system.
- MoveIt can plan motions for the rail-mounted arm.
- A browser GUI can monitor the robot and send bounded movements.
- Example motions cover homing, small arm movements, reaching, gripping, and
a leaf-pick imitation.
- Safety checks reject invalid joint targets and require explicit confirmation
before examples command real hardware.
- The Gemini 2 camera driver and robot model integration are present for
hardware use.
- Offline contract tests protect important model, camera, GUI, and runtime
assumptions.

### What is still incomplete

- Farm-level monitoring, scheduling, task selection, and outcome recording are
not yet implemented.
- The camera mount position and orientation are placeholders until measured on
the assembled robot.
- The rail-to-arm mount geometry is provisional until checked against CAD or
physical measurements.
- The software does not yet detect and select basil leaves.
- Camera observations are not yet converted into arm targets.
- There is no complete autonomous sense-plan-act workflow.
- The current leaf-pick example follows fixed waypoints; it does not react to
a real plant.
- The gripper currently stands in for a cutting or harvesting tool. A final
end-effector task and design still need to be agreed and validated.
- Repeatability, collision clearance, and plant interaction need structured
testing on the real platform.

## How the system fits together

At farm level, autonomy should form a continuous loop:

1. collect plant and environmental observations;
2. maintain a current view of farm and plant health;
3. decide whether and where intervention is needed;
4. assign a task to the robot;
5. execute and verify the task;
6. store the result for later decisions.

Within an assigned robot task, information and commands move like this:

1. The camera observes the plants.
2. Future vision software identifies a target and estimates its 3D position.
3. MoveIt calculates a safe motion for the rail and arm.
4. The arm and rail drivers send commands to the physical devices.
5. Position feedback updates the model, GUI, and planning system.
6. The operator supervises the system and can test movements in small steps.

The same control interfaces are used in both operating modes:

- **View mode** uses mock hardware. It is intended for learning, interface
development, and checking motions in RViz. It is a visualization and control
environment, not a full physics simulation.
- **Hardware mode** controls the real servos, rail, and optional camera. It
requires native Ubuntu 24.04 with ROS 2 Jazzy.

This shared structure allows most control work to begin without access to the  
physical robot.

## Team workstreams

| Subteam | What they do |
| --- | --- |
| **Machine Vision & Perception** | Teaches the robot to see: detects when a plant is ready to harvest and exactly where to cut. |
| **Robot Control & Motion Planning** | Moves arm and rail through free space to a pre-cut pose right in front of the stem. |
| **Manipulation** (new team) | Owns the last centimeters: takes over at the pre-cut pose and makes the cut, primarily with a learned policy trained from teleoperated demonstrations. |

## How we work

 [CONTRIBUTING.md](CONTRIBUTING.md)

## Key terms

- **ROS 2** — the communication framework connecting the robot's software
components.
- **MoveIt** — the motion-planning system used to calculate arm movements.
- **RViz** — the 3D visualization used to inspect the robot and its frames.
- **Joint** — one controllable movement of the arm or rail.
- **Pose** — a position and orientation in 3D space.
- **Coordinate frame** — a named reference used to describe where something
is.
- **Transform** — the measured relationship between two coordinate frames.
- **Calibration** — measured values that connect software positions to the
physical mechanism.
- **Bringup** — starting the set of components needed to operate the robot.
- **Point cloud** — depth-camera data represented as 3D points.

## Where to look next

- [README.md](README.md) — setup, startup, calibration, and troubleshooting.
- [CONTRIBUTING.md](CONTRIBUTING.md) — branch, review, testing, and measurement
rules.
- `src/robot_control/tuwrc_motion_examples/` — understandable examples of
  robot tasks.
- `src/shared/tuwrc_bringup/launch/robot.launch.py` — the main system startup
  path.

If you are unsure where a change belongs, begin with the user-visible behavior
you want, identify which workstream owns it, and discuss the interface before
editing calibration or shared control code.
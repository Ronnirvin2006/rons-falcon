# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 workspace for a 6-DOF robotic arm (Falcon) with gripper. Supports three hardware backends: Gazebo simulation, real STM32 hardware via serial, and mock/fake hardware for testing.

## Build & Launch Commands

```bash
# Build entire workspace
colcon build

# Build single package
colcon build --packages-select falcon_arm_hardware

# Source workspace (required before launching)
source install/setup.bash

# Fake hardware (no physical arm)
ros2 launch falcon_arm_bringup falcon_arm_bringup.launch.py hardware_type:=fake

# Gazebo simulation
ros2 launch falcon_arm_bringup falcon_arm_sim.launch.py

# Real hardware
ros2 launch falcon_arm_bringup falcon_arm_bringup.launch.py hardware_type:=real serial_port:=/dev/ttyACM0

# MoveIt2 motion planning
ros2 launch falcon_arm_moveit_config move_group.launch.py hardware_type:=fake

# MoveIt2 demo (fake hardware + RViz + interactive markers)
ros2 launch falcon_arm_moveit_config demo.launch.py

# Gazebo + MoveIt2 (plan & execute in simulation)
ros2 launch falcon_arm_bringup falcon_arm_moveit_sim.launch.py

# Gazebo + MoveIt Servo + Joystick (real-time IK control)
ros2 launch falcon_arm_bringup falcon_arm_servo_sim.launch.py
ros2 launch falcon_arm_bringup falcon_arm_servo_sim.launch.py joy_dev:=/dev/input/js1

# Teleoperation (GUI sliders)
ros2 launch falcon_arm_teleop slider_teleop.launch.py use_sim_time:=true
```

## Architecture

All packages use `ament_cmake`. The system has layered architecture:

1. **Robot Description** (`falcon_robotic_arm_description`) — URDF/Xacro defining 5 arm joints + 1 gripper (mimic pair). Hardware backend is selected via xacro args (`sim_gazebo`, `hardware_type`).

2. **Hardware Interface** (`falcon_arm_hardware`) — ros2_control plugin (`FalconArmHardwareInterface`) communicating with STM32 over serial at 115200 baud. Protocol: `J1:deg1,J2:deg2,...,J6:deg6\n`. Converts radians (ROS2 side) ↔ degrees (firmware side).

3. **Controllers** — Canonical names are `arm_controller` (JointTrajectoryController, joints 1-5) and `gripper_controller` (gripper_joint_1/2). Bringup config at 50 Hz, Gazebo description config at 1000 Hz, MoveIt config at 100 Hz.

4. **Motion Planning** (`falcon_arm_moveit_config`) — MoveIt2 config with KDL kinematics. Planning group `falcon_arm` (joints 1-5), end effector `falcon_gripper` (gripper_joint_1). SRDF: `falcon_robotic_arm_description.srdf`.

5. **MoveIt Servo** (`falcon_arm_moveit_config/config/servo.yaml`) — Real-time IK streaming via joystick. Publishes to `/arm_controller/joint_trajectory`. Uses custom `joy_servo.py` node (in `falcon_arm_teleop`) with: START=toggle enable, left stick=XY, right stick=Z+yaw, LB/RB=rotate base joint via JointJog, LT/RT=gripper close/open, X=singularity recovery (stops servo, moves to safe pose, restarts servo). KDL kinematics uses `position_only_ik: true` for the 5-DOF arm.

6. **Simulation** (`falcon_arm_gazebo`) — Gazebo with `gz_ros2_control/GazeboSimSystem` and ros_gz_bridge.

7. **Teleoperation** (`falcon_arm_teleop`) — Python nodes: `joy_teleop.py` (gamepad), `slider_teleop.py` (tkinter GUI), and `joy_servo.py` (joystick → MoveIt Servo bridge).

8. **Serial Library** (`serial`) — Vendored cross-platform UART library used by the hardware interface.

## Key Files

- `src/falcon_robotic_arm_description/urdf/falcon_robotic_arm.xacro` — Main robot URDF
- `src/falcon_robotic_arm_description/urdf/falcon_robotic_arm.ros2control.xacro` — Hardware interface selector
- `src/falcon_arm_moveit_config/config/servo.yaml` — MoveIt Servo configuration
- `src/falcon_arm_moveit_config/config/moveit_controllers.yaml` — MoveIt controller mapping
- `src/falcon_arm_bringup/config/falcon_controllers.yaml` — Bringup controller config (50 Hz)
- `src/falcon_arm_teleop/src/joy_servo.py` — Joystick → MoveIt Servo bridge (Cosmic Byte Ares / Xbox layout)
- `src/falcon_arm_hardware/SERIAL_PROTOCOL.md` — STM32 serial protocol spec

## Important: Gazebo + MoveIt Launch Pattern

Do NOT use `demo.launch.py` with Gazebo — it starts its own `ros2_control_node` which conflicts with Gazebo's built-in controller manager. Instead, launch `move_group.launch.py` + `moveit_rviz.launch.py` separately alongside `sim_bringup.launch.py`. The combined launch files (`falcon_arm_moveit_sim.launch.py`, `falcon_arm_servo_sim.launch.py`) handle this correctly.

## Xacro Arg/Property Pattern

The ros2control xacro (`falcon_robotic_arm.ros2control.xacro`) uses `${hardware_type}` Python expressions, which require a `<xacro:property>` — not just a `<xacro:arg>`. Any xacro that includes the description must declare the required args (`sim_gazebo`, `hardware_type`, `serial_port`) so they propagate through. The MoveIt config's `falcon_robotic_arm.urdf.xacro` relies on the description's ros2_control block (not its own separate one).

## Gazebo Mesh Resolution

Gazebo converts `package://` URIs to `model://` internally. The `sim_bringup.launch.py` sets `IGN_GAZEBO_RESOURCE_PATH` to the install share parent directory so meshes resolve correctly in both Gazebo and RViz. Do NOT use absolute mesh paths in xacro — this breaks RViz's resource retriever.

## Spawn Orientation

The URDF `world_fixed` joint already applies -90° roll to stand the arm upright. Spawn commands must use `R=0.0` — adding another roll causes the robot to lie flat.

## Robot Kinematics

All joints are revolute, ±180° range (±3.14159 rad) for full 360° flexibility testing. Gripper joints range -0.7 to 0 rad. The gripper uses a mimic joint (gripper_joint_2 mirrors gripper_joint_1). A fixed `world` frame is offset -90° from `base_link`.

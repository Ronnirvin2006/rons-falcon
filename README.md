# Falcon Robot Arm

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Platform](https://img.shields.io/badge/Platform-Linux-orange)

A 6-DOF robot arm with servo gripper, built with ROS2 Humble, MoveIt2, and Gazebo Fortress. Features STM32 microcontroller for hardware control.

![Falcon Robot Arm Concept](images/Falcon_Arm_prototype_concept_image.png)

## Table of Contents

- [Architecture](#architecture)
- [Team Responsibilities](#team-responsibilities)
- [Repository Structure](#repository-structure)
- [Quick Start](#quick-start)
- [Development Workflow](#development-workflow)
- [Resources](#resources)
- [License](#license)

## Architecture

For the complete system architecture diagram, see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

**High-Level Overview:**
```
User Input (Joystick/RViz)
         │
         ▼
    MoveIt2 (Motion Planning + IK)
         │
         ▼
    ros2_control (Controller Manager)
         │
    ┌────┼────┬────────────┐
    ▼    ▼    ▼            ▼
  Mock  Gazebo  STM32 ──► Motors
```

## Team Responsibilities

| Team | Focus Area | Primary Package |
|------|------------|-----------------|
| CAD | Fusion 360 design, STEP exports | `fusion_cad/` |
| URDF | Robot description, ros2_control config | `src/falcon_robotic_arm_description/` |
| Simulation | Gazebo worlds, spawning | `src/falcon_arm_gazebo/` |
| MoveIt2 | Motion planning, IK, joystick teleop | `src/falcon_arm_moveit_config/`, `src/falcon_arm_teleop/` |
| Hardware Interface | ros2_control hardware driver | `src/falcon_arm_hardware/` |
| Firmware | STM32 motor control code | `firmware/` |

## Repository Structure

```
falcon_robot_arm/
├── README.md                    # This file
├── CLAUDE.md                    # AI assistant guidance
├── LICENSE
├── .gitattributes               # Git LFS configuration
│
├── .claude/commands/            # Claude Code slash commands
│   ├── git-push.md
│   └── create-pr.md
│
├── fusion_cad/                  # CAD Team workspace (NOT a ROS package)
│   ├── fusion/                  # Native Fusion 360 files
│   ├── step/                    # STEP exports
│   └── exports/                 # fusion2urdf exports
│
├── firmware/                    # Firmware Team (NOT a ROS package)
│   ├── include/                 # Header files
│   └──  src/                     # Source files
│
├── src/                         # ROS2 Workspace
│   ├── falcon_robotic_arm_description/  # URDF, meshes, visualization
│   ├── falcon_arm_gazebo/       # Gazebo worlds and models
│   ├── falcon_arm_moveit_config/# MoveIt2 configuration
│   ├── falcon_arm_hardware/     # ros2_control hardware interface
│   ├── falcon_arm_bringup/      # Launch files and configs
│   └── falcon_arm_teleop/       # Joystick teleoperation
│
└── docs/                        # Documentation
    └── ARCHITECTURE.md          # System architecture
```

## Quick Start

### Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Fortress
- Git LFS

### Clone and Build

```bash
# Clone repository
git clone https://github.com/YOUR_ORG/falcon_robot_arm.git
cd falcon_robot_arm

# Pull LFS files (CAD, meshes)
git lfs pull

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Launch Options

```bash
# RViz + Mock Hardware (for URDF testing without physics)
ros2 launch falcon_arm_bringup robot_rviz.launch.py

# Gazebo Simulation (physics-based testing)
ros2 launch falcon_arm_bringup robot_sim.launch.py

# Real STM32 Hardware
ros2 launch falcon_arm_bringup robot_hardware.launch.py serial_port:=/dev/ttyACM0

# Gazebo + MoveIt2 (motion planning development)
ros2 launch falcon_arm_bringup robot_moveit_sim.launch.py

# Hardware + MoveIt2 (final deployment)
ros2 launch falcon_arm_bringup robot_moveit_hardware.launch.py
```

## Development Workflow

### Phase 1: Foundation
- [ ] Set up repository structure and Git LFS
- [ ] CAD team starts Fusion 360 design
- [ ] Create placeholder URDF with boxes

### Phase 2: URDF & Visualization
- [ ] Export URDF from Fusion using fusion2urdf
- [ ] Organize meshes (visual/collision)
- [ ] Complete URDF with ros2_control xacro
- [ ] Test visualization in RViz

### Phase 3: Simulation
- [ ] Create Gazebo world
- [ ] Add gz_ros2_control plugin
- [ ] Configure controllers
- [ ] Test in Gazebo

### Phase 4: MoveIt2 Integration
- [ ] Run MoveIt Setup Assistant
- [ ] Test motion planning in RViz
- [ ] Integrate with Gazebo

### Phase 5: Teleop
- [ ] Create MoveIt Servo integration
- [ ] Test joystick control
- [ ] Implement pick-and-place demo

### Phase 6: Hardware
- [ ] Develop STM32 firmware
- [ ] Create hardware interface
- [ ] Test with real hardware
- [ ] Full system integration

## Resources

### ROS2 & MoveIt2
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/)
- [MoveIt2 Setup Assistant Tutorial](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- [ros2_control Documentation](https://control.ros.org/humble/)

### Gazebo
- [Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress)
- [gz_ros2_control](https://github.com/ros-controls/gz_ros2_control)
- [ros_gz Bridge](https://github.com/gazebosim/ros_gz)

### Hardware
- [PlatformIO Documentation](https://docs.platformio.org/)
- [STM32 Arduino Core](https://github.com/stm32duino/Arduino_Core_STM32)

### CAD
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

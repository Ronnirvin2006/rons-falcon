# Fusion 360 CAD Workspace

This folder contains all CAD-related files for the Falcon Robot Arm.

## Folder Structure

```
fusion_cad/
├── README.md          # This file
├── fusion/            # Native Fusion 360 files (.f3d, .f3z)
├── step/              # STEP exports for CAD interchange
└── exports/           # fusion2urdf raw exports (reference only)
```

## Workflow

### 1. Design in Fusion 360

- Create robot components following the naming convention
- Define joints between components
- Set proper joint types and limits

### 2. Export URDF with fusion2urdf

1. Install [fusion2urdf](https://github.com/syuntoku14/fusion2urdf) add-in
2. Run the add-in from Tools → Add-Ins
3. Export to `exports/` folder

### 3. Process for ROS2

After export, follow the workflow in [docs/CAD_WORKFLOW.md](../docs/CAD_WORKFLOW.md):
- Copy meshes to `src/falcon_robotic_arm_description/meshes/`
- Convert URDF to xacro
- Add ros2_control configuration

## Component Naming Convention

```
falcon_arm (root assembly)
├── base_link
├── link_1
├── link_2
├── link_3
├── link_4
├── link_5
├── link_6
├── gripper_base
├── gripper_left_finger
└── gripper_right_finger
```

## Joint Naming Convention

| Joint | Type | Connects |
|-------|------|----------|
| `joint_1` | Revolute | base_link → link_1 |
| `joint_2` | Revolute | link_1 → link_2 |
| `joint_3` | Revolute | link_2 → link_3 |
| `joint_4` | Revolute | link_3 → link_4 |
| `joint_5` | Revolute | link_4 → link_5 |
| `joint_6` | Revolute | link_5 → link_6 |
| `gripper_joint` | Prismatic | link_6 → gripper_base |

## Git LFS

Large CAD files are tracked by Git LFS. Ensure you have Git LFS installed:

```bash
git lfs install
git lfs pull
```

## STEP Export

When sharing designs outside Fusion 360, export STEP files to the `step/` folder.

## Notes

- Keep Fusion 360 files in `fusion/`
- Export STEP periodically for backup/interchange
- `exports/` contains raw fusion2urdf output (reference only)
- Final URDF lives in `src/falcon_robotic_arm_description/`

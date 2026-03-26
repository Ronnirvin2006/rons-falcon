# Quick Start: Integrating Hardware Interface with Falcon Robot Arm

This guide shows how to integrate the `falcon_arm_hardware` interface into your existing Falcon Robot Arm URDF and launch files.

## Step 1: Build the Hardware Interface Package

```bash
cd ~/falcon_edutech/falcon_robot_arm

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build just the hardware package
colcon build --packages-select falcon_arm_hardware

# Source the workspace
source install/setup.bash
```

## Step 2: Update Your URDF Xacro File

Your existing `falcon_robotic_arm.xacro` file needs to include the hardware interface configuration.

### Option A: Include the Example Configuration

If you want to use the exact controller setup provided:

```xml
<?xml version="1.0" ?>
<robot name="falcon_robotic_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include standard materials -->
  <xacro:include filename="$(find falcon_robotic_arm_description)/urdf/materials.xacro" />
  
  <!-- Include the hardware interface configuration -->
  <xacro:include filename="$(find falcon_arm_hardware)/example_ros2_control_config.xacro" />

  <!-- Your existing links and joints -->
  <link name="base_link">
    <!-- ... existing link content ... -->
  </link>

  <!-- ... rest of your URDF ... -->

</robot>
```

### Option B: Manual Configuration

Replace your existing `<ros2_control>` section with this:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <ros2_control name="FalconArmSystem" type="system">
    <hardware>
      <plugin>falcon_arm_hardware/FalconArmHardwareInterface</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
      <param name="baud_rate">115200</param>
    </hardware>

    <!-- Joints 1-5: Arm -->
    <joint name="joint_1">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="joint_2">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="joint_3">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="joint_4">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    
    <joint name="joint_5">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>

    <!-- Joint 6: Gripper -->
    <joint name="joint_6">
      <command_interface name="position"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

</robot>
```

## Step 3: Create/Update Launch File

Create a new launch file `src/falcon_arm_bringup/launch/robot_hardware.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package paths
    robot_desc_pkg = FindPackageShare('falcon_robotic_arm_description')
    robot_desc_path = robot_desc_pkg.find('falcon_robotic_arm_description')
    
    # Define launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 connection'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time'
    )
    
    # Get configuration
    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Load URDF
    urdf_file = os.path.join(robot_desc_path, 'urdf', 'falcon_robotic_arm.xacro')
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('robot_description', 'robot_description'),
        ],
        output='screen'
    )
    
    # Joint State Broadcaster
    jsc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    
    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen'
    )
    
    # Gripper Controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '-c', '/controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        serial_port_arg,
        use_sim_time_arg,
        rsp_node,
        controller_manager_node,
        jsc_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
    ])
```

## Step 4: Update Controller Configuration

The controller configuration from your earlier request should go in:
`src/falcon_robotic_arm_description/config/falcon_controllers.yaml`

For reference, your configuration maps to:
- **arm_controller** → Controls joints 1-5 (arm)
- **gripper_controller** → Controls joint 6 (gripper)
- **joint_state_broadcaster** → Publishes `/joint_states`

## Step 5: USB Permission Setup (Linux)

For non-root access to the serial port:

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, then verify
id
# You should see 'dialout' in the groups
```

## Step 6: Test the Setup

### Verify Hardware Interface is Found

```bash
# List available hardware interfaces
ros2 run controller_manager list_hardware_interfaces

# Output should include:
# - "FalconArmSystem: Unclaimed" or "Claimed"
```

### Check Serial Connection

```bash
# Verify serial port exists
ls -la /dev/ttyACM*

# Optional: Test with minicom
minicom -D /dev/ttyACM0 -b 115200
```

### Launch the Hardware Interface

```bash
# Start with default serial port
ros2 launch falcon_arm_bringup robot_hardware.launch.py

# Or with custom serial port
ros2 launch falcon_arm_bringup robot_hardware.launch.py serial_port:=/dev/ttyUSB0
```

### Monitor Joint States

In another terminal:

```bash
# Watch joint states being published
ros2 topic echo /joint_states

# Output should show positions updating from STM32
```

### Send Test Command

```bash
# Use ROS2 CLI to send a trajectory goal
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  trajectory_msgs/JointTrajectory \
  "{header: {frame_id: base_link}, joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5],
    points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 5}}]}"
```

## Step 7: Integration with MoveIt2

If using MoveIt2, update your controller configuration to include:

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
    
  gripper_controller:
    type: GripperCommand
    joints:
      - joint_6
```

Then launch MoveIt:

```bash
ros2 launch falcon_arm_moveit_config move_group.launch.py hardware_protocol:=real
```

## Complete File Structure After Integration

```
falcon_robot_arm/
├── src/
│   ├── falcon_robotic_arm_description/
│   │   ├── urdf/
│   │   │   └── falcon_robotic_arm.xacro  (updated with hardware interface)
│   │   └── config/
│   │       └── falcon_controllers.yaml   (your controller config)
│   │
│   ├── falcon_arm_hardware/              (NEW - fully implemented)
│   │   ├── include/falcon_arm_hardware/
│   │   │   └── falcon_arm_hardware_interface.hpp
│   │   ├── src/
│   │   │   └── falcon_arm_hardware_interface.cpp
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── falcon_arm_hardware_plugins.xml
│   │   ├── README.md
│   │   ├── SERIAL_PROTOCOL.md
│   │   └── example_ros2_control_config.xacro
│   │
│   ├── falcon_arm_bringup/
│   │   └── launch/
│   │       └── robot_hardware.launch.py  (NEW - launch file from Step 3)
│   │
│   └── ... other packages ...
```

## Troubleshooting

### Hardware Interface Plugin Not Found

```bash
Error: Could not find class "falcon_arm_hardware/FalconArmHardwareInterface"
```

**Solution:**
1. Rebuild: `colcon build --packages-select falcon_arm_hardware`
2. Re-source: `source install/setup.bash`
3. Verify plugin XML: Check `falcon_arm_hardware_plugins.xml` exists

### Serial Port Permission Denied

```bash
Error: Could not open serial port /dev/ttyACM0: Permission denied
```

**Solution:**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Communication Errors

```bash
Failed to send commands (error 1/10)
Failed to read feedback (error 2/10)
```

**Solution:**
1. Check cable connection
2. Verify baud rate matches STM32 firmware (115200)
3. Check STM32 is powered and has firmware
4. Monitor serial with minicom to see raw data

### Controllers Won't Start

```bash
ERROR: Could not load controller 'arm_controller'
```

**Solution:**
1. Verify all 5 arm joints in URDF
2. Check controller YAML has correct joint names
3. Ensure hardware interface is claimed: `ros2 service call /controller_manager/list_hardware_interfaces`

## Next Steps

1. **Write STM32 Firmware** - Implement serial message parsing and motor control
2. **Integrate with MoveIt2** - Add motion planning
3. **Add Teleoperation** - Joystick control via falcon_arm_teleop
4. **Test with Gazebo** - Simulate before real hardware testing

## Support Files

- **SERIAL_PROTOCOL.md** - Detailed protocol specification for STM32 firmware
- **example_ros2_control_config.xacro** - Complete controller configuration example
- **README.md** - Detailed hardware interface documentation

Good luck with your robot arm!

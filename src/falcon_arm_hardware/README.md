# Falcon Arm Hardware Interface

This package contains the ros2_control hardware interface plugin for communicating with the Falcon Robot Arm's STM32 microcontroller.

## Overview

The hardware interface acts as a bridge between ROS2/MoveIt2 (which uses **radians**) and the STM32 firmware (which uses **degrees**). It:

- Receives joint position commands in radians from MoveIt2
- Converts them to degrees
- Sends commands via serial port (115200 baud) to the STM32
- Reads feedback from the microcontroller
- Converts feedback from degrees back to radians
- Makes both position and velocity available to controllers

## Hardware Capabilities

| Feature | Details |
|---------|---------|
| **Joints Controlled** | 5 arm joints + 1 gripper servo (6 total) |
| **Command Interface** | Position (radians) |
| **State Interfaces** | Position (radians) and Velocity (rad/s) |
| **Serial Protocol** | 115200 baud, 8 data bits, 1 stop bit, no parity |
| **Default Port** | `/dev/ttyACM0` (configurable) |
| **Update Rate** | 100 Hz (configurable via controller manager) |

## COM Protocol

### Message Format (STM32 → Hardware Interface)
```
J1:45.5,J2:30.0,J3:-15.2,J4:60.0,J5:45.0,J6:0.0\n
```
**Meaning:** Joint positions in degrees (comma-separated, newline-terminated)

### Message Format (Hardware Interface → STM32)
```
J1:45.5,J2:30.0,J3:-15.2,J4:60.0,J5:45.0,J6:0.0\n
```
**Meaning:** Joint position commands in degrees (sent at 100 Hz)

## Configuration

### URDF/Xacro Configuration

Add this to your `robot.ros2control` xacro file:

```xml
<ros2_control name="FalconRoboticArm" type="system">
  <hardware>
    <plugin>falcon_arm_hardware/FalconArmHardwareInterface</plugin>
    <!-- Configuration parameters -->
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
  </hardware>

  <!-- Define joints - example for 6 joints -->
  <joint name="joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- ... repeat for joints 2-6 ... -->
</ros2_control>
```

### Launch File Configuration

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare serial port as a launch argument (can be overridden)
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for STM32 connection'
    )
    
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description],
        remappings=[
            ('robot_description', 'robot_description'),
        ],
        output='screen',
    )
    
    return LaunchDescription([
        serial_port_arg,
        controller_manager_node,
        # ... other launch actions ...
    ])
```

### Launch Command

```bash
# With default serial port (/dev/ttyACM0)
ros2 launch falcon_arm_bringup robot_hardware.launch.py

# With custom serial port
ros2 launch falcon_arm_bringup robot_hardware.launch.py serial_port:=/dev/ttyUSB0
```

## Building

```bash
# Install dependencies
cd ~/falcon_edutech/falcon_robot_arm
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select falcon_arm_hardware

# Source the workspace
source install/setup.bash
```

## Debugging

### Enable Verbose Logging

```bash
# Run with debug-level logging
ROS_LOG_LEVEL=DEBUG ros2 launch falcon_arm_bringup robot_hardware.launch.py
```

### Check Serial Connection

```bash
# List available serial ports
ls -la /dev/ttyACM* /dev/ttyUSB*

# Test serial connection manually
minicom -D /dev/ttyACM0 -b 115200
```

### Monitor Serial Traffic

Use a serial monitor tool to see messages being sent/received:

```bash
# Install socat (if not already installed)
sudo apt install socat

# Create a virtual serial port pair for testing
socat -d -d PTY,raw,echo=0 PTY,raw,echo=0

# Monitor the connection
cat < /dev/ttyUSB0  # View outgoing data
```

## Code Structure

### Header File: `falcon_arm_hardware_interface.hpp`
- **Constants**: Defines number of joints, conversion factors
- **Class Definition**: `FalconArmHardwareInterface` extending `SystemInterface`
- **Lifecycle Methods**: `on_init`, `on_configure`, `on_activate`, `on_deactivate`, `on_cleanup`
- **Operation Methods**: `read()` and `write()`
- **Helper Methods**: Serial communication, message parsing

### Implementation File: `falcon_arm_hardware_interface.cpp`
- **Full implementation** with detailed comments on each line
- **Serial port management** (open/close)
- **Message construction** (convert rad→deg)
- **Message parsing** (convert deg→rad)
- **Error handling** with retry logic

## Conversion Details

### Radians to Degrees (what happens in `write()`)
```cpp
// Controller sends: 1.5708 radians (90°)
// Hardware interface converts:
double command_degrees = command_radians * (180.0 / 3.14159);  // = 90.0 degrees
// STM32 receives: "J1:90.0,J2:..."
```

### Degrees to Radians (what happens in `read()`)
```cpp
// STM32 sends: "J1:90.0,J2:..."
// Hardware interface converts:
double feedback_radians = feedback_degrees * (3.14159 / 180.0);  // = 1.5708 rad
// Controller/MoveIt receives: 1.5708 radians
```

## Lifecycle

The hardware interface follows this lifecycle:

1. **on_init()** - Read configuration from URDF
2. **on_configure()** - Open serial port
3. **on_activate()** - Enable command transmission
4. **read() & write()** - Called repeatedly (100 Hz default)
5. **on_deactivate()** - Stop command transmission
6. **on_cleanup()** - Close serial port

## Troubleshooting

### Serial Port Permission Denied
```bash
# Grant user access to serial ports
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### Hardware Interface Not Found
- Ensure package is built: `colcon build --packages-select falcon_arm_hardware`
- Check plugin registration in `falcon_arm_hardware_plugins.xml`
- Verify plugin entry in `package.xml`

### Communication Errors
- Check serial cable connection
- Verify baud rate matches STM32 firmware: 115200
- Monitor serial port output with minicom or similar tool
- Check for TX/RX line swap in cable

### Joint Not Moving
- Verify MoveIt is sending commands (monitor with `ros2 topic echo`)
- Check hardware is activated: `ros2 service call /controller_manager/list_controllers`
- Verify degree values are reasonable (within motor limits)
- Check STM32 firmware parses commands correctly

## References

- [ros2_control Documentation](https://control.ros.org/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube.html)

## Contributing

When modifying the hardware interface:
- Keep all comments detailed and descriptive
- Follow the conversion factor patterns
- Add error handling for serial failures
- Log all significant events for debugging
- Test lifecycle transitions thoroughly

## License

Apache License 2.0 - See LICENSE file for details

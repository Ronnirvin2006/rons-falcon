#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       falcon_arm_hw.launch.py
# *  Description:    Top-level bringup for the Falcon Robotic Arm on REAL hardware.
# *                  Starts the ros2_control node with either the real STM32 serial
# *                  hardware interface or a mock interface, then spawns all controllers.
# *
# *                  Use this file when the physical arm is connected over USB-UART
# *                  (default /dev/ttyACM0) or when testing with mock hardware.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    # Mock hardware (no physical arm, for RViz / controller testing):
# *    ros2 launch falcon_arm_bringup falcon_arm_hw.launch.py
# *
# *    # Real hardware (STM32 connected on default port):
# *    ros2 launch falcon_arm_bringup falcon_arm_hw.launch.py hardware_type:=real
# *
# *    # Real hardware on a custom port:
# *    ros2 launch falcon_arm_bringup falcon_arm_hw.launch.py \
# *         hardware_type:=real serial_port:=/dev/ttyUSB0
# *
# *  Launch arguments:
# *    hardware_type  : 'fake' (default) | 'real'
# *    serial_port    : serial device path, default /dev/ttyACM0
# *
# *  What starts:
# *    • robot_state_publisher   — URDF → TF
# *    • ros2_control_node       — controller manager (host process for controllers)
# *    • joint_state_broadcaster — publishes /joint_states
# *    • arm_controller          — JointTrajectoryController for joints 1–5
# *    • gripper_controller      — JointTrajectoryController for gripper joints
# *
# *  Hardware note:
# *    'real' mode uses the falcon_arm_hardware/FalconArmHardwareInterface plugin
# *     which communicates with the STM32 Nucleo over UART at 115200 baud.
# *    'fake' mode uses mock_components/GenericSystem — no physical connection needed.
# *
# *  Does NOT start Gazebo — use falcon_arm_sim.launch.py for simulation.
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# ROS2 launch core imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

# ---------------------------------------------------------------------------
# ROS2 launch_ros imports
# ---------------------------------------------------------------------------
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build and return the real-hardware (or mock) bringup LaunchDescription."""

    # -----------------------------------------------------------------------
    # CLI argument: hardware_type
    #   'fake' — mock_components/GenericSystem (safe for RViz / CI testing)
    #   'real' — FalconArmHardwareInterface over serial (physical arm)
    # -----------------------------------------------------------------------
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='fake',
        choices=['fake', 'real'],
        description=(
            "'fake' uses mock ros2_control (no physical arm needed). "
            "'real' connects to the STM32 over UART."
        ),
    )

    # -----------------------------------------------------------------------
    # CLI argument: serial_port
    #   Only meaningful when hardware_type=real.
    #   /dev/ttyACM0 is the default USB-CDC enumeration for the STM32 Nucleo.
    # -----------------------------------------------------------------------
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial device for real hardware  (ignored when hardware_type=fake).',
    )

    # -----------------------------------------------------------------------
    # CLI argument: use_sim_time
    #   Always false for hardware mode — wall clock is the correct time source.
    # -----------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (keep false for real hardware).',
    )

    hardware_type  = LaunchConfiguration('hardware_type')
    serial_port    = LaunchConfiguration('serial_port')
    use_sim_time   = LaunchConfiguration('use_sim_time')

    # -----------------------------------------------------------------------
    # Expand the xacro URDF at launch time.
    #   sim_gazebo is NOT set → selects fake/real hardware plugin based on
    #   hardware_type and serial_port arguments.
    # -----------------------------------------------------------------------
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('falcon_robotic_arm_description'),
                 'urdf', 'falcon_robotic_arm.xacro']
            ),
            ' hardware_type:=', hardware_type,
            ' serial_port:=',   serial_port,
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Path to controller configuration YAML (arm + gripper + JSB definitions)
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('falcon_arm_bringup'), 'config', 'falcon_controllers.yaml']
    )

    # -----------------------------------------------------------------------
    # Robot State Publisher (RSP)
    #   Parses URDF and publishes TF2 transforms.  Wall-clock mode (no sim time).
    # -----------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # -----------------------------------------------------------------------
    # ros2_control_node  (controller manager host)
    #   Loads the hardware plugin (fake or real) and manages all controllers.
    #   In simulation mode this node is NOT used — Gazebo owns the CM instead.
    # -----------------------------------------------------------------------
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[robot_description, robot_controllers, {'use_sim_time': use_sim_time}],
    )

    # -----------------------------------------------------------------------
    # joint_state_broadcaster spawner
    #   Publishes /joint_states from controller_manager.
    # -----------------------------------------------------------------------
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # -----------------------------------------------------------------------
    # arm_controller + gripper_controller spawners
    #   Both start only after joint_state_broadcaster is live (OnProcessExit).
    # -----------------------------------------------------------------------
    spawn_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_arm_controller',
                    output='screen',
                    arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                )
            ],
        )
    )

    spawn_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_gripper_controller',
                    output='screen',
                    arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                )
            ],
        )
    )

    return LaunchDescription([
        hardware_type_arg,         # 1. Register CLI args
        serial_port_arg,
        use_sim_time_arg,
        robot_state_publisher,     # 2. Publish URDF / TF
        control_node,              # 3. Start controller manager with hardware plugin
        spawn_jsb,                 # 4. Spawn joint_state_broadcaster
        spawn_arm,                 # 5. (after JSB) spawn arm_controller
        spawn_gripper,             # 6. (after JSB) spawn gripper_controller
    ])

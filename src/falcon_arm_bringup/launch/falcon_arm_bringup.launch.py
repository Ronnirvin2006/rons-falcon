#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       falcon_arm_bringup.launch.py
# *  Description:    Core hardware/mock bringup for the Falcon Robotic Arm.
# *                  Starts robot_state_publisher, ros2_control_node, and all
# *                  controller spawners for either fake or real hardware mode.
# *
# *                  This is the foundational launch file used by all other bringup
# *                  variants. For simulation, use falcon_arm_sim.launch.py instead.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    # Fake hardware (default — no physical arm needed):
# *    ros2 launch falcon_arm_bringup falcon_arm_bringup.launch.py
# *
# *    # Real hardware:
# *    ros2 launch falcon_arm_bringup falcon_arm_bringup.launch.py \
# *         hardware_type:=real serial_port:=/dev/ttyACM0
# *
# *  Launch arguments:
# *    hardware_type : 'fake' (default) | 'real'
# *    serial_port   : UART device path (default /dev/ttyACM0)
# *    use_sim_time  : bool (default false)
# *
# *  What starts:
# *    • robot_state_publisher    — URDF → TF
# *    • ros2_control_node        — controller manager + hardware plugin
# *    • joint_state_broadcaster  — publishes /joint_states
# *    • arm_controller           — JTC for joints 1–5
# *    • gripper_controller       — JTC for gripper joints
# *
# *  Copyright 2024 Falcon-Edutech (Apache License 2.0)
# *****************************************************************************************
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='fake',
        choices=['fake', 'real'],
        description='Use fake (simulated) or real hardware interface'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for real hardware (only used when hardware_type=real)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time'
    )

    # Get launch configurations
    hardware_type = LaunchConfiguration('hardware_type')
    serial_port = LaunchConfiguration('serial_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("falcon_robotic_arm_description"), "urdf", "falcon_robotic_arm.xacro"]
            ),
            " hardware_type:=",
            hardware_type,
            " serial_port:=",
            serial_port,
        ]
    )
    
    # Ensure the expanded xacro output is passed as a string parameter
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("falcon_arm_bringup"),
            "config",
            "falcon_controllers.yaml",
        ]
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Controller Manager (ros2_control)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time}],
        output="both",
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Arm Controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Gripper Controller spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Delay arm_controller and gripper_controller after joint_state_broadcaster
    delay_arm_gripper_spawner_after_jsc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    nodes = [
        hardware_type_arg,
        serial_port_arg,
        use_sim_time_arg,
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_arm_gripper_spawner_after_jsc,
    ]

    return LaunchDescription(nodes)

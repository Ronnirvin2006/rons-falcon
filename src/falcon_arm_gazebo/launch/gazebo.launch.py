#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       gazebo.launch.py  (falcon_arm_gazebo package)
# *  Description:    Spawn Falcon Robot Arm in the falcon_arm_world workbench scene
# *  Author:         BARGAVAN R
# *
# *  Usage:
# *    ros2 launch falcon_arm_gazebo gazebo.launch.py
# *
# *  What this does:
# *    1. Starts Gazebo with worlds/falcon_arm_world.sdf (table + coloured blocks)
# *    2. Publishes robot description (xacro → URDF, sim_gazebo:=true)
# *    3. Spawns robot on top of workbench (z = 0.77 m, roll = -90°)
# *    4. Bridges /clock and /joint_states from Gazebo to ROS
# *    5. Spawns controllers:  joint_state_broadcaster
# *                            arm_controller  (after JSB)
# *                            gripper_controller  (after JSB)
# *****************************************************************************************
'''

import os
from os.path import join

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    AppendEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------------------
    # Package paths
    # ---------------------------
    pkg_description = get_package_share_directory('falcon_robotic_arm_description')
    pkg_gazebo      = get_package_share_directory('falcon_arm_gazebo')
    pkg_gz_sim      = get_package_share_directory('ros_gz_sim')

    urdf_file   = os.path.join(pkg_description, 'urdf', 'falcon_robotic_arm.xacro')
    bridge_yaml = os.path.join(pkg_description, 'config', 'bridge.yaml')
    world_file  = os.path.join(pkg_gazebo,      'worlds', 'falcon_arm_world.sdf')

    # ---------------------------
    # Set Gazebo plugin search path
    # ---------------------------
    set_plugin_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        '/opt/ros/humble/lib',
    )

    # ---------------------------
    # World argument (override world from CLI if needed)
    # ---------------------------
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='Path to Gazebo world SDF',
    )
    world_config = LaunchConfiguration('world_file')

    # ---------------------------
    # Robot description
    # sim_gazebo:=true  → selects gz_ros2_control/GazeboSimSystem in ros2control.xacro
    # ---------------------------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_gazebo:=true']),
        value_type=str,
    )

    # ---------------------------
    # Gazebo simulation
    # IMPORTANT: controller_manager is started inside Gazebo via the
    # gz_ros2_control plugin declared in falcon_robotic_arm.gazebo.
    # Do NOT add a second ros2_control_node here.
    # ---------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_config, ' -r -v 3'],
        }.items(),
    )

    # ---------------------------
    # Robot State Publisher
    # ---------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
    )

    # ---------------------------
    # Spawn robot on top of workbench
    # Table surface is at z = 0.755 m.
    # Robot spawns at z = 0.77 m  (15 mm clearance above table top).
    # Roll = -1.5708 stands the arm upright (corrects URDF base orientation).
    # ---------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_falcon_arm',
        output='screen',
        arguments=[
            '-name',           'falcon_robotic_arm',
            '-topic',          'robot_description',
            '-allow_renaming', 'true',
            '-x',  '0.05',
            '-y',  '0.0',
            '-z',  '0.77',      # on workbench top
            '-R',  '-1.5708',   # 90 deg roll → arm stands upright
            '-P',  '0.0',
            '-Y',  '3.14159',   # 180 deg yaw to face the target blocks
        ],
    )

    # ---------------------------
    # ROS ↔ Gazebo topic bridge  (/clock, /joint_states)
    # ---------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}',
        ],
    )

    # ---------------------------
    # joint_state_broadcaster  (3 s delay → waits for Gazebo plugin)
    # ---------------------------
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # arm_controller  (starts after JSB exits successfully)
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

    # gripper_controller  (also starts after JSB exits)
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

    delayed_jsb = TimerAction(period=3.0, actions=[spawn_jsb])

    # ---------------------------
    # Launch sequence
    # ---------------------------
    return LaunchDescription([
        set_plugin_path,
        world_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        delayed_jsb,
        spawn_arm,
        spawn_gripper,
    ])

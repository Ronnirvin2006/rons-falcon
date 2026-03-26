#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       ros2_control.launch.py
# *  Description:    ros2_control bringup for the Falcon Robotic Arm in mock/fake mode.
# *                  Starts robot_state_publisher, controller_manager (ros2_control_node),
# *                  all controller spawners, and RViz2 for real-time visualization.
# *
# *                  This launch file is used for controller testing, RViz inspection,
# *                  and CI verification without a physical arm or Gazebo.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_robotic_arm_description ros2_control.launch.py
# *
# *  What starts:
# *    • robot_state_publisher      — URDF → TF + /robot_description
# *    • ros2_control_node          — controller manager (fake hardware)
# *    • joint_state_broadcaster    — publishes /joint_states
# *    • arm_controller             — JTC for joints 1–5
# *    • gripper_controller         — JTC for gripper joints
# *    • RViz2                      — visualization with display.rviz config
# *
# *  Spawner order:
# *    JSB first → arm_controller after JSB exits → gripper_controller after arm exits.
# *
# *  Note:
# *    Uses fake/mock hardware (no serial port, no Gazebo).
# *    For simulation, use falcon_arm_sim.launch.py instead.
# *****************************************************************************************
'''

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package and paths
    pkg_share = FindPackageShare('falcon_robotic_arm_description').find('falcon_robotic_arm_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'falcon_robotic_arm.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'falcon_controllers.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command(['xacro ', urdf_file])},
            controller_config
        ],
        output='screen'
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Arm Controller Spawner
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Gripper Controller Spawner
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Delay arm controller spawner after joint state broadcaster
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    # Delay gripper controller spawner after arm controller
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
        rviz_node
    ])
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       display.launch.py
# *  Description:    URDF visualisation launch for the Falcon Robotic Arm.
# *                  Starts robot_state_publisher, joint_state_publisher_gui (interactive
# *                  sliders), and RViz2 so the robot model can be inspected and manually
# *                  posed without any controllers or simulation.
# *
# *                  Intended for URDF validation and mesh inspection only.
# *                  This is a development/test tool — not used in production.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_robotic_arm_description display.launch.py
# *
# *  What starts:
# *    • robot_state_publisher    — parses URDF, publishes /robot_description + TF
# *    • joint_state_publisher_gui — tkinter sliders to manually set joint angles
# *    • RViz2                    — 3-D visualization with display.rviz config
# *
# *  Note:
# *    No controllers or Gazebo are needed.  This uses wall-clock time by default.
# *    Set use_sim_time:=true only if inspecting alongside a running simulator.
# *****************************************************************************************
'''

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('falcon_robotic_arm_description').find('falcon_robotic_arm_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'falcon_robotic_arm.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'display.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
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
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
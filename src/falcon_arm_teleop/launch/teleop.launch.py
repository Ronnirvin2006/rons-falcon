#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       teleop.launch.py
# *  Description:    Launch the Falcon Arm joystick teleoperation stack.
# *                  Starts the joy_node (reads from physical gamepad/joystick) and
# *                  the joy_teleop.py node (converts joystick events to arm commands).
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_arm_teleop teleop.launch.py
# *    ros2 launch falcon_arm_teleop teleop.launch.py joy_dev:=/dev/input/js1
# *
# *  Launch arguments:
# *    joy_dev : path to joystick device (default /dev/input/js0)
# *
# *  What starts:
# *    • joy_node       — reads raw joystick events and publishes sensor_msgs/Joy
# *    • joy_teleop.py  — maps Joy messages to FollowJointTrajectory goals
# *
# *  Prerequisites:
# *    The following must already be running before launching this:
# *      - joint_state_broadcaster  (publishes /joint_states)
# *      - arm_controller           (/arm_controller/follow_joint_trajectory)
# *      - gripper_controller       (/gripper_controller/follow_joint_trajectory)
# *
# *  Gamepad defaults:
# *    Left stick X/Y  →  joint_1 / joint_2
# *    Right stick X/Y →  joint_3 / joint_4
# *    D-pad X         →  joint_5
# *    LB button       →  close gripper
# *    RB button       →  open gripper
# *****************************************************************************************
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"dev": LaunchConfiguration("joy_dev")}],
    )

    teleop_node = Node(
        package="falcon_arm_teleop",
        executable="joy_teleop.py",
        name="falcon_joy_teleop",
        output="screen",
    )

    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])

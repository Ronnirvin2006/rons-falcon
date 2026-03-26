#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       falcon_arm_teleop_sim.launch.py
# *  Description:    Simulation + GUI teleop bringup for the Falcon Robotic Arm.
# *                  Starts Gazebo with the full arm world, activates all controllers,
# *                  then launches the slider-teleop GUI so the operator can manually
# *                  position every joint from a single terminal command.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_arm_bringup falcon_arm_teleop_sim.launch.py
# *
# *  What starts:
# *    1. Gazebo Fortress        — falcon_arm_world.sdf (workbench + 3 blocks)
# *    2. robot_state_publisher  — URDF → TF (sim time)
# *    3. ros_gz_bridge          — /clock and /joint_states
# *    4. joint_state_broadcaster, arm_controller, gripper_controller
# *    5. slider_teleop.py       — tkinter GUI with per-joint +/− step buttons
# *                                (delayed 6 s to ensure controllers are ready)
# *
# *  GUI note:
# *    Each joint has a read-only position indicator and +/- buttons (8°/step).
# *    Gripper has Open/Close buttons only.
# *    The GUI exits cleanly with the window's X button.
# *
# *  After launch:
# *    A tkinter window will appear automatically on the host display.
# *    Ensure DISPLAY is set (e.g.  export DISPLAY=:0  for local sessions).
# *
# *  Simulation only — uses sim_time internally for all timestamp sync.
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# ROS2 launch core imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ---------------------------------------------------------------------------
# ROS2 launch_ros imports
# ---------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Build LaunchDescription: Gazebo sim + slider teleop GUI."""

    # -----------------------------------------------------------------------
    # Resolve package share directories
    # -----------------------------------------------------------------------
    pkg_gazebo = get_package_share_directory('falcon_arm_gazebo')
    # ^^^ install/share/falcon_arm_gazebo/ — contains sim_bringup.launch.py

    pkg_teleop = get_package_share_directory('falcon_arm_teleop')
    # ^^^ install/share/falcon_arm_teleop/ — contains slider_teleop.launch.py

    # -----------------------------------------------------------------------
    # Include sim_bringup.launch.py
    #   Starts Gazebo + RSP + ros_gz_bridge + all controllers.
    #   Default world: falcon_arm_world.sdf
    # -----------------------------------------------------------------------
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, '/launch/sim_bringup.launch.py']
        ),
    )

    # -----------------------------------------------------------------------
    # Include slider_teleop.launch.py  (delayed 6 s)
    #   Delay ensures arm_controller and gripper_controller are fully active
    #   before the teleop node tries to connect to them.
    #   use_sim_time:=true keeps slider node timestamps in sync with Gazebo.
    # -----------------------------------------------------------------------
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_teleop, '/launch/slider_teleop.launch.py']
        ),
        launch_arguments={
            'use_sim_time': 'true',   # sync with Gazebo /clock
        }.items(),
    )

    delayed_teleop = TimerAction(
        period=6.0,               # 6 s: Gazebo + controllers need ~5 s to start
        actions=[teleop_launch],  # start GUI after controllers are ready
    )

    return LaunchDescription([
        sim_bringup,      # 1. Start full simulation stack
        delayed_teleop,   # 2. After 6 s — open teleop GUI
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       falcon_arm_sim.launch.py
# *  Description:    Top-level bringup: Falcon Robotic Arm in Gazebo simulation.
# *                  Starts Gazebo Fortress with the pick-and-place world, spawns the
# *                  robot on the workbench, and activates all ros2_control controllers.
# *
# *                  This is the single-command entry point for simulation.
# *                  Delegates world/robot/controller setup to sim_bringup.launch.py
# *                  in falcon_arm_gazebo so all Gazebo-specific configuration
# *                  stays in that package.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_arm_bringup falcon_arm_sim.launch.py
# *    ros2 launch falcon_arm_bringup falcon_arm_sim.launch.py \
# *         world_file:=/path/to/custom.sdf
# *
# *  What starts:
# *    • Gazebo Fortress        — physics simulation with falcon_arm_world.sdf
# *    • robot_state_publisher  — URDF → TF, synced with Gazebo /clock
# *    • ros_gz_bridge          — /clock and /joint_states forwarded to ROS2
# *    • joint_state_broadcaster — publishes /joint_states
# *    • arm_controller         — JointTrajectoryController for joints 1–5
# *    • gripper_controller     — JointTrajectoryController for gripper joints
# *
# *  After launch:
# *    Verify:  ros2 topic echo /joint_states
# *    Control: ros2 launch falcon_arm_teleop slider_teleop.launch.py use_sim_time:=true
# *
# *  Simulation only — no real hardware interface is used.
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# ROS2 launch core imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription                       # root descriptor
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------------------------
# ROS2 launch_ros imports
# ---------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory  # package path lookup


def generate_launch_description():
    """Build and return the simulation bringup LaunchDescription."""

    # -----------------------------------------------------------------------
    # Resolve package share directory for falcon_arm_gazebo
    # -----------------------------------------------------------------------
    pkg_gazebo = get_package_share_directory('falcon_arm_gazebo')
    # ^^^ install/share/falcon_arm_gazebo/  — contains worlds/ and launch/

    # -----------------------------------------------------------------------
    # CLI argument: world_file
    #   Passed through to sim_bringup.launch.py so users can override the
    #   world SDF from a single top-level command.
    # -----------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world_file',
        default_value='',           # empty → sim_bringup uses its own default
        description=(
            'Absolute path to a custom Gazebo world SDF. '
            'Leave blank to use falcon_arm_world.sdf.'
        ),
    )

    world_config = LaunchConfiguration('world_file')

    # -----------------------------------------------------------------------
    # Include sim_bringup.launch.py from falcon_arm_gazebo.
    #   That file handles:
    #     - IGN_GAZEBO_SYSTEM_PLUGIN_PATH extension
    #     - Gazebo server start
    #     - robot_state_publisher (sim_gazebo:=true)
    #     - Robot spawn on workbench
    #     - ros_gz_bridge (/clock, /joint_states)
    #     - Controller spawning (JSB → arm → gripper)
    # -----------------------------------------------------------------------
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, '/launch/sim_bringup.launch.py']
        ),
        launch_arguments={
            'world_file': world_config,   # forward user-supplied world path
        }.items(),
    )

    return LaunchDescription([
        world_arg,       # 1. Register world_file CLI argument
        sim_bringup,     # 2. Start full simulation stack
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       falcon_arm_moveit_sim.launch.py
# *  Description:    Simulation + MoveIt2 motion planning bringup.
# *                  Starts Gazebo with the full arm world, activates all ros2_control
# *                  controllers, then launches the MoveIt2 move_group server and RViz
# *                  with the MoveIt2 motion-planning plugin.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_arm_bringup falcon_arm_moveit_sim.launch.py
# *
# *  What starts:
# *    1. Gazebo Fortress        — falcon_arm_world.sdf (workbench + 3 blocks)
# *    2. robot_state_publisher  — URDF → TF (sim time)
# *    3. ros_gz_bridge          — /clock and /joint_states
# *    4. joint_state_broadcaster, arm_controller, gripper_controller
# *    5. move_group             — MoveIt2 planning server (after 8 s delay)
# *    6. RViz2                  — MoveIt2 motion-planning GUI (after move_group)
# *
# *  MoveIt2 note:
# *    motion planning uses the configuration in falcon_arm_moveit_config.
# *    The SRDF planning group 'arm' covers joints 1–5; 'gripper' covers
# *    gripper_joint_1 and gripper_joint_2.
# *
# *  After launch:
# *    Use the RViz2 MotionPlanning panel to set a goal, plan, and execute.
# *    The arm will execute the planned trajectory via the Gazebo simulation.
# *
# *  Simulation only.
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
    """Build LaunchDescription: Gazebo sim + MoveIt2 move_group + RViz2."""

    # -----------------------------------------------------------------------
    # Resolve package share directories
    # -----------------------------------------------------------------------
    pkg_gazebo  = get_package_share_directory('falcon_arm_gazebo')
    pkg_moveit  = get_package_share_directory('falcon_arm_moveit_config')

    # -----------------------------------------------------------------------
    # 1. Full simulation stack (Gazebo + robot + controllers)
    # -----------------------------------------------------------------------
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_gazebo, '/launch/sim_bringup.launch.py']
        ),
    )

    # -----------------------------------------------------------------------
    # 2. MoveIt2 move_group (planning server)
    #   Delayed 8 s: Gazebo + controllers need ~5 s; extra margin for
    #   move_group to find the active controllers before trying to connect.
    #   NOTE: Do NOT use demo.launch.py here — it starts its own
    #   ros2_control_node which conflicts with Gazebo's controller_manager.
    # -----------------------------------------------------------------------
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_moveit, '/launch/move_group.launch.py']
        ),
    )

    delayed_move_group = TimerAction(
        period=8.0,
        actions=[move_group],
    )

    # -----------------------------------------------------------------------
    # 3. RViz2 with MoveIt2 motion-planning plugin
    # -----------------------------------------------------------------------
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_moveit, '/launch/moveit_rviz.launch.py']
        ),
    )

    delayed_rviz = TimerAction(
        period=9.0,
        actions=[moveit_rviz],
    )

    return LaunchDescription([
        sim_bringup,           # 1. Start Gazebo simulation
        delayed_move_group,    # 2. After 8 s — start MoveIt2 move_group
        delayed_rviz,          # 3. After 9 s — start RViz2
    ])

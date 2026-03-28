#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       gazebo.launch.py
# *  Description:    Development/legacy Gazebo launch for the Falcon Robotic Arm.
# *                  Spawns the robot in a configurable world (default: empty.sdf),
# *                  starts ros_gz_bridge, and activates all ros2_control controllers.
# *
# *                  NOTE: For production use, prefer falcon_arm_sim.launch.py in the
# *                  falcon_arm_bringup package — it provides cleaner argument handling
# *                  and uses the custom falcon_arm_world.sdf by default.
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Usage:
# *    ros2 launch falcon_robotic_arm_description gazebo.launch.py
# *    ros2 launch falcon_robotic_arm_description gazebo.launch.py \
# *         world_file:=/path/to/world.sdf
# *
# *  Launch arguments:
# *    world_file : path to world SDF (default empty.sdf)
# *
# *  What starts:
# *    • Gazebo Fortress            — with specified world
# *    • robot_state_publisher      — URDF → TF (sim_gazebo:=true)
# *    • ros_gz_bridge              — /clock, /joint_states
# *    • joint_state_broadcaster    — after 3 s delay
# *    • arm_controller             — after JSB exits
# *    • gripper_controller         — after JSB exits
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
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------------------
    # Package paths
    # ---------------------------
    pkg_share = get_package_share_directory("falcon_robotic_arm_description")
    urdf_file   = os.path.join(pkg_share, "urdf",   "falcon_robotic_arm.xacro")
    bridge_yaml = os.path.join(pkg_share, "config", "bridge.yaml")

    # ---------------------------
    # World argument
    # ---------------------------
    world_arg = DeclareLaunchArgument(
        "world_file",
        default_value="empty.sdf",
        description="Gazebo world file to load",
    )
    world_file = LaunchConfiguration("world_file")

    # ---------------------------
    # Robot description (xacro -> URDF string)
    # Pass absolute mesh path to fix Gazebo model:// resolution
    # ---------------------------
    meshes_dir = os.path.join(get_package_share_directory('falcon_robotic_arm_description'), 'meshes')
    robot_description = ParameterValue(
        Command([
            "xacro ", urdf_file,
            " sim_gazebo:=true",
            " mesh_path:=" + meshes_dir
        ]),
        value_type=str,
    )

    # ---------------------------
    # Gazebo simulation
    #
    # IMPORTANT: Do NOT add a separate ros2_control_node here.
    # The controller_manager is started automatically inside Gazebo
    # via GazeboSimROS2ControlPlugin declared in your URDF/xacro.
    # Adding a second ros2_control_node causes duplicate robots.
    # ---------------------------
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"]),
        }.items(),
    )

    # ---------------------------
    # Robot State Publisher
    # Publishes /robot_description topic and TF tree
    # ---------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # ---------------------------
    # Spawn robot into Gazebo
    # Orientation handled by world_fixed joint rpy in URDF
    # ---------------------------
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_falcon_arm",
        output="screen",
        arguments=[
            "-name",           "falcon_robotic_arm",
            "-topic",          "robot_description",
            "-allow_renaming", "true",
            "-x",  "0.0",
            "-y",  "0.0",
            "-z",  "0.03",   # slight z offset to prevent initial collision with ground
            "-R",  "0.0",   # 90 deg roll to stand upright
            "-P",  "0.0",
            "-Y",  "0.0",
        ],
    )

    # ---------------------------
    # ROS <-> Gazebo topic bridge
    # Bridges /clock, /joint_states, etc.
    # ---------------------------
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_yaml}",
        ],
    )

    # ---------------------------
    # Spawner: joint_state_broadcaster
    # Delayed 3s to wait for Gazebo plugin to start controller_manager
    # ---------------------------
    spawn_jsb = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
    )

    # ---------------------------
    # Spawner: arm_controller
    # Uses OnProcessExit so it only starts after JSB spawner finishes.
    # This is more reliable than hardcoded timer delays.
    # ---------------------------
    spawn_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="spawner_arm_controller",
                    output="screen",
                    arguments=[
                        "arm_controller",
                        "--controller-manager", "/controller_manager",
                    ],
                )
            ],
        )
    )

    # ---------------------------
    # Spawner: gripper_controller
    # Also starts after JSB spawner finishes
    # ---------------------------
    spawn_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    name="spawner_gripper_controller",
                    output="screen",
                    arguments=[
                        "gripper_controller",
                        "--controller-manager", "/controller_manager",
                    ],
                )
            ],
        )
    )

    # Wrap JSB spawner in a 3s delay
    delayed_spawn_jsb = TimerAction(
        period=3.0,
        actions=[spawn_jsb],
    )

    # ---------------------------
    # Launch sequence:
    # 1. Gazebo  (starts controller_manager via URDF plugin)
    # 2. robot_state_publisher  (publishes URDF + TF)
    # 3. spawn_robot  (creates model in Gazebo)
    # 4. ros_gz_bridge  (bridges topics)
    # 5. [3s delay] joint_state_broadcaster spawner
    # 6. [after JSB] arm_controller spawner
    # 7. [after JSB] gripper_controller spawner
    # ---------------------------
    return LaunchDescription([
        world_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        delayed_spawn_jsb,
        spawn_arm,
        spawn_gripper,
    ])

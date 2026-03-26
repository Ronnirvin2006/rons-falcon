#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       sim_bare.launch.py
# *  Description:    Launch the Falcon Arm custom Gazebo world (workbench + coloured blocks)
# *                  WITHOUT spawning the robot model or any controllers.
# *                  Useful for world design, lighting inspection, and SDF debugging.
# *  Author:         BARGAVAN R - MIT
# *
# *  Usage:
# *    ros2 launch falcon_arm_gazebo sim_bare.launch.py
# *
# *  Launch sequence:
# *    1. Append gz plugin search path  (gz_ros2_control discovery)
# *    2. Start Gazebo with falcon_arm_world.sdf
# *         - Physics, lighting, workbench table, coloured manipulation blocks
# *         - No robot, no controllers, no ROS bridges
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# Standard library imports
# ---------------------------------------------------------------------------
import os                              # used for path resolution
from os.path import join               # shorthand for os.path.join

# ---------------------------------------------------------------------------
# ROS2 launch imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription                          # root descriptor
from launch.actions import (
    AppendEnvironmentVariable,         # set/extend an environment variable
    DeclareLaunchArgument,             # expose a CLI argument to the user
    IncludeLaunchDescription,          # embed another launch file
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,     # load a Python-based launch file
)
from launch.substitutions import LaunchConfiguration          # read a declared arg at runtime
from ament_index_python.packages import get_package_share_directory  # pkg path lookup


def generate_launch_description():
    """Build and return the LaunchDescription that starts only the Falcon Arm world."""

    # -----------------------------------------------------------------------
    # Resolve package share directories
    # -----------------------------------------------------------------------
    pkg_gazebo = get_package_share_directory('falcon_arm_gazebo')
    # ^^^ Points to   install/share/falcon_arm_gazebo/
    #     Contains worlds/ subdirectory with falcon_arm_world.sdf

    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    # ^^^ Contains gz_sim.launch.py which wraps the 'gz sim' command

    # -----------------------------------------------------------------------
    # Absolute path to the custom world SDF
    # -----------------------------------------------------------------------
    world_file = os.path.join(
        pkg_gazebo, 'worlds', 'falcon_arm_world.sdf'
    )
    # ^^^ Full path example:
    #     .../install/share/falcon_arm_gazebo/worlds/falcon_arm_world.sdf
    #     The world contains: directional sun, overhead warm/cool lights,
    #     a wooden workbench at z=0.755 m, and 3 coloured manipulation blocks.

    # -----------------------------------------------------------------------
    # CLI argument: world_file
    #   Allows the user to override the world at launch time:
    #     ros2 launch falcon_arm_gazebo world_only.launch.py \
    #          world_file:=/path/to/my_world.sdf
    # -----------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world_file',                   # argument name (used on CLI and substitution)
        default_value=world_file,        # default: the falcon_arm_world
        description='Absolute path to the Gazebo world SDF file to load',
    )

    # LaunchConfiguration resolves 'world_file' to its runtime value
    world_config = LaunchConfiguration('world_file')

    # -----------------------------------------------------------------------
    # Extend gz plugin search path so Physics, UserCommands, and
    # SceneBroadcaster plugins declared in the SDF are found at startup.
    # -----------------------------------------------------------------------
    set_plugin_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',   # env var Gazebo reads for plugin lookup
        '/opt/ros/humble/lib',             # where ROS2 Humble gz plugins live
    )

    # -----------------------------------------------------------------------
    # Start Gazebo with the custom world.
    #   gz_args is forwarded verbatim to 'gz sim <gz_args>':
    #     world_config  →  full path to the SDF file
    #     '-r'          →  start simulation immediately (not paused)
    #     '-v 3'        →  verbosity 3 for informational plugin messages
    # -----------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')   # ros_gz_sim wrapper
        ),
        launch_arguments={
            'gz_args': [world_config, ' -r -v 3'],   # path + run flags
        }.items(),
    )

    # -----------------------------------------------------------------------
    # Final launch description — minimal: just env var + Gazebo
    # No RSP, no robot spawn, no bridge, no controller spawners.
    # -----------------------------------------------------------------------
    return LaunchDescription([
        set_plugin_path,   # configure env before Gazebo reads it
        world_arg,         # register CLI argument
        gz_sim,            # start Gazebo with the custom world only
    ])

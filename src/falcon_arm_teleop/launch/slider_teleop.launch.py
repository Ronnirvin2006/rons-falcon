#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       slider_teleop.launch.py
# *  Description:    Launch the Falcon Arm GUI slider teleop node.
# *                  Starts slider_teleop.py which opens a tkinter window with
# *                  independent sliders for all 7 joints (5 arm + 2 gripper).
# *
# *  Author:         BARGAVAN R - MIT
# *
# *  Usage:
# *    ros2 launch falcon_arm_teleop slider_teleop.launch.py
# *    ros2 launch falcon_arm_teleop slider_teleop.launch.py use_sim_time:=true
# *
# *  Prerequisites:
# *    The following controllers must already be running before launching this:
# *      - joint_state_broadcaster
# *      - arm_controller          (/arm_controller/follow_joint_trajectory)
# *      - gripper_controller      (/gripper_controller/follow_joint_trajectory)
# *
# *  Typical usage with simulation:
# *    Terminal 1: ros2 launch falcon_arm_gazebo robot_world.launch.py
# *    Terminal 2: ros2 launch falcon_arm_teleop  slider_teleop.launch.py use_sim_time:=true
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# ROS2 launch imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription          # root descriptor
from launch.actions import DeclareLaunchArgument  # expose CLI arguments
from launch.substitutions import LaunchConfiguration  # resolve args at runtime
from launch_ros.actions import Node           # launch a ROS2 node


def generate_launch_description():
    """Build and return the LaunchDescription for the slider teleop GUI."""

    # -----------------------------------------------------------------------
    # CLI argument: use_sim_time
    #   Set to 'true' when running with Gazebo so the slider node's timestamps
    #   are synchronised with the simulation clock (/clock topic).
    #   Set to 'false' (default) for real hardware operation.
    # -----------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',              # argument name
        default_value='false',       # default: wall clock (real hardware mode)
        description=(
            'Use simulation time from /clock. '
            'Set true when running alongside Gazebo.'
        ),
    )

    # -----------------------------------------------------------------------
    # slider_teleop node
    #   package    : falcon_arm_teleop
    #   executable : slider_teleop.py  (installed to lib/falcon_arm_teleop/)
    #   output     : screen — shows ROS2 logging (connection status, errors)
    #   emulate_tty: True   — preserves ANSI colours in terminal log output
    # -----------------------------------------------------------------------
    slider_teleop_node = Node(
        package='falcon_arm_teleop',          # ROS2 package name
        executable='slider_teleop.py',        # script installed as executable
        name='falcon_slider_teleop',          # ROS2 node name in the graph
        output='screen',                      # log to terminal
        emulate_tty=True,                     # preserve log colour codes
        parameters=[
            {
                # Sync node clock with Gazebo /clock when in simulation
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # -----------------------------------------------------------------------
    # Return the complete launch description
    # -----------------------------------------------------------------------
    return LaunchDescription([
        use_sim_time_arg,     # register CLI argument first
        slider_teleop_node,   # then start the GUI node
    ])

#!/usr/bin/env python3
"""
Full simulation + MoveIt Servo + Joystick launch.

Starts:
  1. Gazebo + robot + controllers          (sim_bringup)
  2. MoveIt move_group                     (delayed 8s)
  3. RViz with MoveIt plugin               (delayed 9s)
  4. MoveIt Servo + JoyToServoPub + joy    (delayed 12s)

Usage:
  ros2 launch falcon_arm_bringup falcon_arm_servo_sim.launch.py
  ros2 launch falcon_arm_bringup falcon_arm_servo_sim.launch.py joy_dev:=/dev/input/js1
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('falcon_arm_gazebo')
    pkg_moveit = get_package_share_directory('falcon_arm_moveit_config')

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev", default_value="/dev/input/js0",
        description="Joystick device path",
    )

    # 1. Gazebo simulation (robot + controllers)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_gazebo + '/launch/sim_bringup.launch.py'
        ),
    )

    # 2. MoveIt move_group (needs controllers active)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_moveit + '/launch/move_group.launch.py'
        ),
    )
    delayed_move_group = TimerAction(period=8.0, actions=[move_group])

    # 3. RViz with MoveIt plugin
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_moveit + '/launch/moveit_rviz.launch.py'
        ),
    )
    delayed_rviz = TimerAction(period=9.0, actions=[moveit_rviz])

    # 4. MoveIt Servo + joystick (needs move_group's planning scene)
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_moveit + '/launch/servo.launch.py'
        ),
        launch_arguments={"joy_dev": LaunchConfiguration("joy_dev")}.items(),
    )
    delayed_servo = TimerAction(period=12.0, actions=[servo])

    return LaunchDescription([
        joy_dev_arg,
        sim_bringup,
        delayed_move_group,
        delayed_rviz,
        delayed_servo,
    ])

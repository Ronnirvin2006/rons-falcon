#!/usr/bin/env python3
"""
Launch MoveIt Servo + custom JoyServo + joy_node for real-time joystick IK control.

Prerequisites (must already be running):
  - Gazebo with gz_ros2_control (arm_controller active)
  - robot_state_publisher
  - MoveIt move_group

Usage:
  ros2 launch falcon_arm_moveit_config servo.launch.py
"""
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Build MoveIt config (robot_description, SRDF, kinematics)
    moveit_config = (
        MoveItConfigsBuilder(
            "falcon_robotic_arm_description",
            package_name="falcon_arm_moveit_config",
        )
        .robot_description(mappings={"sim_gazebo": "true"})
        .to_moveit_configs()
    )

    # Load servo parameters
    servo_yaml = load_yaml("falcon_arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev",
        default_value="/dev/input/js0",
        description="Joystick device path",
    )

    # MoveIt Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # Custom joy-to-servo bridge (replaces JoyToServoPub which has hardcoded Panda frames)
    joy_servo_node = Node(
        package="falcon_arm_teleop",
        executable="joy_servo.py",
        name="joy_servo",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # Joy node for reading the gamepad
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {"dev": LaunchConfiguration("joy_dev")},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription([
        joy_dev_arg,
        servo_node,
        joy_servo_node,
        joy_node,
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       sim_robot.launch.py
# *  Description:    Spawn Falcon Robotic Arm in an empty Gazebo world
# *                  (sun + ground plane only, no custom scene).
# *                  Intended for URDF validation, controller tuning, and
# *                  isolated robot testing without environment clutter.
# *  Author:         BARGAVAN R - MIT
# *
# *  Usage:
# *    ros2 launch falcon_arm_gazebo sim_robot.launch.py
# *
# *  Launch sequence:
# *    1. Append gz plugin search path so gz_ros2_control is discoverable
# *    2. Start Gazebo Fortress with the built-in empty world (sun + ground plane)
# *    3. Start robot_state_publisher with URDF (sim_gazebo:=true)
# *    4. Spawn the Falcon arm at ground level (z = 0.01 m)
# *    5. Start ros_gz_bridge for /clock and /joint_states
# *    6. Spawn joint_state_broadcaster  (3 s delay → Gazebo plugin must be up)
# *    7. Spawn arm_controller           (after JSB exits cleanly)
# *    8. Spawn gripper_controller       (after JSB exits cleanly)
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# Standard library imports
# ---------------------------------------------------------------------------
import os                              # os.path helpers for resolving file paths
from os.path import join               # join() shorthand used for pkg paths

# ---------------------------------------------------------------------------
# ROS2 launch core imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription                           # top-level descriptor
from launch.actions import (
    AppendEnvironmentVariable,         # prepend/append a value to an env variable
    DeclareLaunchArgument,             # declare a runtime CLI argument
    IncludeLaunchDescription,          # nest another launch file
    RegisterEventHandler,              # hook into process lifecycle events
    TimerAction,                       # delay an action by N seconds
)
from launch.event_handlers import OnProcessExit               # fires when a process ends
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,     # resolve a .py launch file path at runtime
)
from launch.substitutions import Command, LaunchConfiguration  # runtime substitutions

# ---------------------------------------------------------------------------
# ROS2 launch_ros imports
# ---------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory  # locate installed pkgs
from launch_ros.actions import Node                           # launch a ROS2 node
from launch_ros.parameter_descriptions import ParameterValue  # typed parameter wrapper


def generate_launch_description():
    """Build and return the complete LaunchDescription for robot-only simulation."""

    # -----------------------------------------------------------------------
    # Resolve installed package share directories
    # -----------------------------------------------------------------------
    pkg_description = get_package_share_directory('falcon_robotic_arm_description')
    # ^^^ Points to   install/share/falcon_robotic_arm_description/
    #     Contains URDF, meshes, configs

    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    # ^^^ Points to   install/share/ros_gz_sim/
    #     Contains gz_sim.launch.py that we reuse

    # -----------------------------------------------------------------------
    # Build absolute paths for key resource files
    # -----------------------------------------------------------------------
    urdf_file = os.path.join(
        pkg_description, 'urdf', 'falcon_robotic_arm.xacro'
    )
    # ^^^ Main robot xacro file — defines all links, joints, and hardware plugin

    bridge_yaml = os.path.join(
        pkg_description, 'config', 'bridge.yaml'
    )
    # ^^^ YAML config listing Gazebo ↔ ROS topic bridges (/clock, /joint_states)

    # -----------------------------------------------------------------------
    # Extend IGN_GAZEBO_SYSTEM_PLUGIN_PATH so Gazebo can find gz_ros2_control
    # This must be done before gz_sim starts; AppendEnvironmentVariable handles that.
    # -----------------------------------------------------------------------
    set_plugin_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',   # environment variable name
        '/opt/ros/humble/lib',             # ROS2 Humble library path containing gz plugins
    )

    # -----------------------------------------------------------------------
    # Process the xacro file into a URDF string at launch time.
    #   sim_gazebo:=true  → selects gz_ros2_control/GazeboSimSystem in
    #                        falcon_robotic_arm.ros2control.xacro so Gazebo
    #                        drives the controller_manager (no separate node).
    # ParameterValue wraps the Command substitution so it stays as a string
    # when passed as a ROS2 parameter.
    # -----------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_gazebo:=true']),
        value_type=str,
    )

    # -----------------------------------------------------------------------
    # Launch Gazebo Fortress with the built-in empty world.
    #   'empty.sdf'  →  Gazebo's built-in empty world: directional sun light
    #                   and a grey infinite ground plane.  No custom objects.
    #   '-r'         →  Run simulation immediately (no paused start)
    #   '-v 3'       →  Verbosity level 3 for useful debug output
    # NOTE: The gz_ros2_control plugin declared inside the URDF starts the
    # controller_manager automatically inside Gazebo.  Do NOT add a second
    # ros2_control_node here — it would conflict.
    # -----------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')   # upstream launch file
        ),
        launch_arguments={
            'gz_args': 'empty.sdf -r -v 3',   # empty world: sun + ground plane
        }.items(),
    )

    # -----------------------------------------------------------------------
    # Robot State Publisher (RSP)
    #   Parses robot_description URDF and broadcasts TF transforms for every
    #   fixed and moving joint.  use_sim_time=True makes it consume /clock
    #   from Gazebo instead of wall-clock so transforms stay synchronised.
    # -----------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},  # full URDF string
            {'use_sim_time': True},                    # synchronise with Gazebo /clock
        ],
    )

    # -----------------------------------------------------------------------
    # Spawn the robot into the running Gazebo world.
    #   -name            unique model name inside Gazebo
    #   -topic           reads URDF from this ROS topic (RSP publishes it)
    #   -allow_renaming  lets Gazebo add suffix if name conflicts
    #   -x, -y, -z       spawn position in world frame
    #                    z = 0.01 m → 10 mm above ground plane (prevents clipping)
    #   -R               roll  = -1.5708 rad  → corrects URDF base orientation;
    #                    the model stands upright on the ground
    #   -P               pitch = 0 (no tilt)
    #   -Y               yaw   = 0 (face +X axis)
    # -----------------------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_falcon_arm',
        output='screen',
        arguments=[
            '-name',           'falcon_robotic_arm',   # Gazebo model name
            '-topic',          'robot_description',    # URDF source topic
            '-allow_renaming', 'true',                 # avoid name collision
            '-x',  '0.0',                              # world-frame X position (m)
            '-y',  '0.0',                              # world-frame Y position (m)
            '-z',  '0.01',                             # 10 mm above ground plane
            '-R',  '-1.5708',                          # -90 deg roll → arm upright
            '-P',  '0.0',                              # no pitch
            '-Y',  '0.0',                              # yaw = 0 → face +X
        ],
    )

    # -----------------------------------------------------------------------
    # ROS ↔ Gazebo topic bridge
    #   Translates Gazebo internal topics to ROS2 topics.
    #   Config in bridge.yaml maps:
    #     /clock       (gz.msgs.Clock   → rosgraph_msgs/msg/Clock)
    #     /joint_states (gz.msgs.Model  → sensor_msgs/msg/JointState)
    # -----------------------------------------------------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}',   # path to bridge YAML config
        ],
    )

    # -----------------------------------------------------------------------
    # joint_state_broadcaster (JSB)
    #   Reads joint states from controller_manager and publishes /joint_states.
    #   Delayed by 3 s so Gazebo's gz_ros2_control plugin has time to start the
    #   controller_manager before the spawner tries to connect to it.
    # -----------------------------------------------------------------------
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',           # controller name from YAML
            '--controller-manager', '/controller_manager',  # CM namespace
        ],
    )

    # -----------------------------------------------------------------------
    # arm_controller (JointTrajectoryController — joints 1-5)
    #   Registered to start AFTER joint_state_broadcaster spawner exits
    #   successfully.  This guarantees JSB is active and /joint_states is
    #   publishing before the arm controller subscribes to it.
    # -----------------------------------------------------------------------
    spawn_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,            # watch for JSB spawner to finish
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_arm_controller',
                    output='screen',
                    arguments=[
                        'arm_controller',                        # controller name
                        '--controller-manager', '/controller_manager',
                    ],
                )
            ],
        )
    )

    # -----------------------------------------------------------------------
    # gripper_controller (JointGroupPositionController — gripper_joint_1/2)
    #   Also starts after JSB exits.  Runs in parallel with arm_controller
    #   startup since both just need JSB to be ready.
    # -----------------------------------------------------------------------
    spawn_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,            # same trigger: JSB spawner done
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_gripper_controller',
                    output='screen',
                    arguments=[
                        'gripper_controller',                    # controller name
                        '--controller-manager', '/controller_manager',
                    ],
                )
            ],
        )
    )

    # Wrap JSB spawner in a 3-second timer so Gazebo plugin is ready first
    delayed_jsb = TimerAction(
        period=3.0,          # seconds to wait after Gazebo starts
        actions=[spawn_jsb],  # execute JSB spawner after the delay
    )

    # -----------------------------------------------------------------------
    # Final launch sequence — order matters:
    #   1. Set env var          (before Gazebo starts)
    #   2. Gazebo               (starts sim + controller_manager via plugin)
    #   3. RSP                  (publishes URDF immediately)
    #   4. Spawn robot          (reads URDF from RSP topic)
    #   5. ROS-Gz bridge        (connects /clock and /joint_states)
    #   6. Delayed JSB          (waits 3 s then spawns JSB)
    #   7. arm + gripper        (event-driven, start after JSB is live)
    # -----------------------------------------------------------------------
    return LaunchDescription([
        set_plugin_path,       # must be first so env var is set before Gazebo
        gz_sim,                # start Gazebo empty world
        robot_state_publisher, # publish URDF transforms
        spawn_robot,           # create robot model in Gazebo
        ros_gz_bridge,         # bridge /clock + /joint_states
        delayed_jsb,           # wait 3 s → spawn JSB
        spawn_arm,             # arm_controller starts after JSB
        spawn_gripper,         # gripper_controller starts after JSB
    ])

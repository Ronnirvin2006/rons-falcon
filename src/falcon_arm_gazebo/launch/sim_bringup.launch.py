#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       sim_bringup.launch.py
# *  Description:    Full simulation launch — Falcon Robotic Arm spawned inside the
# *                  custom Falcon Arm World (workbench + coloured blocks) with all
# *                  ros2_control controllers active.
# *                  This is the primary launch file for manipulation tasks and
# *                  end-to-end testing in simulation.
# *  Author:         BARGAVAN R - MIT
# *
# *  Usage:
# *    ros2 launch falcon_arm_gazebo sim_bringup.launch.py
# *    ros2 launch falcon_arm_gazebo sim_bringup.launch.py \
# *         world_file:=/path/to/custom.sdf
# *
# *  Launch sequence:
# *    1. Append gz plugin search path  (gz_ros2_control discovery)
# *    2. Start Gazebo with falcon_arm_world.sdf
# *         (workbench at z=0.755 m, coloured blocks, custom lighting)
# *    3. Start robot_state_publisher  (URDF → TF, sim_gazebo:=true)
# *    4. Spawn Falcon arm on top of workbench  (z=0.77 m, R=-90°, Y=180°)
# *    5. Start ros_gz_bridge  (/clock, /joint_states)
# *    6. Spawn joint_state_broadcaster  (3 s delay)
# *    7. Spawn arm_controller           (after JSB exits)
# *    8. Spawn gripper_controller       (after JSB exits)
# *****************************************************************************************
'''

# ---------------------------------------------------------------------------
# Standard library imports
# ---------------------------------------------------------------------------
import os                              # file path resolution
from os.path import join               # convenience alias for os.path.join

# ---------------------------------------------------------------------------
# ROS2 launch core imports
# ---------------------------------------------------------------------------
from launch import LaunchDescription                          # root descriptor object
from launch.actions import (
    AppendEnvironmentVariable,         # extend an OS environment variable at launch
    DeclareLaunchArgument,             # expose a user-configurable CLI argument
    IncludeLaunchDescription,          # compose launch files by nesting
    RegisterEventHandler,              # attach a callback to a process lifecycle event
    TimerAction,                       # delay execution of an action
)
from launch.event_handlers import OnProcessExit               # triggers when a process ends
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,     # reference to a Python launch file
)
from launch.substitutions import Command, LaunchConfiguration  # runtime value resolution

# ---------------------------------------------------------------------------
# ROS2 launch_ros imports
# ---------------------------------------------------------------------------
from ament_index_python.packages import get_package_share_directory  # installed pkg paths
from launch_ros.actions import Node                           # wrapper for a ROS2 node
from launch_ros.parameter_descriptions import ParameterValue  # typed parameter for nodes


def generate_launch_description():
    """Build and return the full simulation LaunchDescription (robot + world + controllers)."""

    # -----------------------------------------------------------------------
    # Resolve package share directories at import time
    # -----------------------------------------------------------------------
    pkg_description = get_package_share_directory('falcon_robotic_arm_description')
    # ^^^ install/share/falcon_robotic_arm_description/ — URDF, meshes, bridge config

    pkg_gazebo = get_package_share_directory('falcon_arm_gazebo')
    # ^^^ install/share/falcon_arm_gazebo/ — world SDF files

    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    # ^^^ install/share/ros_gz_sim/ — contains gz_sim.launch.py wrapper

    # -----------------------------------------------------------------------
    # Build absolute paths for resources
    # -----------------------------------------------------------------------
    urdf_file = os.path.join(
        pkg_description, 'urdf', 'falcon_robotic_arm.xacro'
    )
    # ^^^ Main robot description file; xacro macro expands at launch via Command()

    bridge_yaml = os.path.join(
        pkg_description, 'config', 'bridge.yaml'
    )
    # ^^^ Maps Gazebo internal topics to ROS2 topics:
    #     /clock       gz.msgs.Clock   → rosgraph_msgs/msg/Clock
    #     /joint_states gz.msgs.Model  → sensor_msgs/msg/JointState

    world_file = os.path.join(
        pkg_gazebo, 'worlds', 'falcon_arm_world.sdf'
    )
    # ^^^ Custom world: wooden workbench at z=0.755 m, 3 coloured blocks,
    #     directional sun + warm/cool fill lights, 1 ms physics step

    # -----------------------------------------------------------------------
    # CLI argument: world_file
    #   Users can substitute a different world SDF without editing this file:
    #     ros2 launch ... robot_world.launch.py world_file:=/tmp/my_world.sdf
    # -----------------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world_file',                   # argument key
        default_value=world_file,        # default: falcon_arm_world.sdf
        description='Absolute path to the Gazebo world SDF to load',
    )

    # Substitution that resolves to the runtime value of 'world_file'
    world_config = LaunchConfiguration('world_file')

    # -----------------------------------------------------------------------
    # Extend Gazebo plugin search path before the simulator starts.
    #   gz_ros2_control is compiled into /opt/ros/humble/lib; Gazebo must find
    #   it to start controller_manager inside the simulator process.
    # -----------------------------------------------------------------------
    set_plugin_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH',   # Ignition plugin path env variable
        '/opt/ros/humble/lib',             # ROS2 Humble system library directory
    )

    # Tell Ignition Gazebo where to find package:// meshes.
    # Gazebo converts package://pkg_name/... to model://pkg_name/... and searches
    # IGN_GAZEBO_RESOURCE_PATH. We point it at the install share directory so
    # model://falcon_robotic_arm_description/meshes/... resolves correctly.
    set_resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_description, '..'),  # parent of share/falcon_robotic_arm_description
    )

    # -----------------------------------------------------------------------
    # Expand xacro to URDF string.
    #   sim_gazebo:=true → selects gz_ros2_control/GazeboSimSystem plugin in
    #   falcon_robotic_arm.ros2control.xacro.
    #   The controller_manager is then owned by Gazebo — do NOT launch a
    #   separate ros2_control_node alongside this setup.
    # -----------------------------------------------------------------------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim_gazebo:=true']),
        value_type=str,
    )

    # -----------------------------------------------------------------------
    # Launch Gazebo Fortress with the custom world.
    #   world_config  → resolved path to falcon_arm_world.sdf
    #   '-r'          → run immediately (no pause at startup)
    #   '-v 3'        → verbosity 3 (shows plugin load info)
    # -----------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')   # wrapper launch file
        ),
        launch_arguments={
            'gz_args': [world_config, ' -r -v 3'],   # SDF path + run flags
        }.items(),
    )

    # -----------------------------------------------------------------------
    # Robot State Publisher (RSP)
    #   Reads robot_description parameter and publishes TF2 transforms for all
    #   fixed joints plus the moving joints as they are updated by /joint_states.
    #   use_sim_time=True → RSP uses /clock from Gazebo so TF is time-consistent.
    # -----------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},  # full expanded URDF
            {'use_sim_time': True},                    # sync with Gazebo clock
        ],
    )

    # -----------------------------------------------------------------------
    # Spawn the Falcon arm into Gazebo.
    #   Position matches the workbench geometry in falcon_arm_world.sdf:
    #     x = 0.05 m   → slight forward offset from workbench centre
    #     y = 0.0  m   → centred laterally
    #     z = 0.77 m   → workbench surface at 0.755 m + 15 mm clearance
    #   Orientation:
    #     R/P = 0  → URDF world_fixed joint already applies -90° roll
    #     Y = 3.14159 → 180 deg yaw; arm faces the coloured manipulation blocks
    # -----------------------------------------------------------------------
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_falcon_arm',
        output='screen',
        arguments=[
            '-name',           'falcon_robotic_arm',   # Gazebo model identifier
            '-topic',          'robot_description',    # URDF published by RSP
            '-allow_renaming', 'true',                 # avoid name clash on restart
            '-x',  '0.05',                             # 5 cm forward on workbench
            '-y',  '0.0',                              # centred on workbench
            '-z',  '0.77',                             # on top of workbench surface
            '-R',  '0.0',                              # URDF handles orientation
            '-P',  '0.0',                              # no pitch
            '-Y',  '3.14159',                          # 180 deg yaw → faces blocks
        ],
    )

    # -----------------------------------------------------------------------
    # ROS ↔ Gazebo bridge
    #   Forwards two topics from Gazebo's internal bus to ROS2:
    #     /clock       — needed so all ROS nodes can use sim time
    #     /joint_states — real-time feedback from gz_ros2_control joint plugin
    # -----------------------------------------------------------------------
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_yaml}',   # path resolved at build time above
        ],
    )

    # -----------------------------------------------------------------------
    # joint_state_broadcaster (JSB) spawner
    #   JSB reads every joint's state from controller_manager and re-publishes
    #   on /joint_states.  RSP listens to /joint_states to update TF.
    #   Delayed 3 s: gz_ros2_control inside Gazebo needs time to initialise
    #   controller_manager before a client can register a controller.
    # -----------------------------------------------------------------------
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',                  # matches key in controllers YAML
            '--controller-manager', '/controller_manager',  # full CM node path
        ],
    )

    # -----------------------------------------------------------------------
    # arm_controller (JointTrajectoryController)
    #   Controls joints 1-5 using position commands.
    #   Started via event handler so it waits for JSB to finish spawning
    #   (OnProcessExit fires when the 'spawner' process for JSB exits with code 0).
    # -----------------------------------------------------------------------
    spawn_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,            # wait until JSB spawner process exits
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_arm_controller',
                    output='screen',
                    arguments=[
                        'arm_controller',                        # JTC for joints 1-5
                        '--controller-manager', '/controller_manager',
                    ],
                )
            ],
        )
    )

    # -----------------------------------------------------------------------
    # gripper_controller (JointGroupPositionController)
    #   Controls gripper_joint_1 and gripper_joint_2 with position commands.
    #   Also starts after JSB exits — runs concurrently with arm_controller startup.
    # -----------------------------------------------------------------------
    spawn_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,            # same trigger as arm_controller
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    name='spawner_gripper_controller',
                    output='screen',
                    arguments=[
                        'gripper_controller',                    # position controller for gripper
                        '--controller-manager', '/controller_manager',
                    ],
                )
            ],
        )
    )

    # Wrap JSB in a 6-second timer — gives Gazebo time to fully load
    # gz_ros2_control and start the physics loop before controller activation
    delayed_jsb = TimerAction(
        period=6.0,           # delay in seconds from launch start
        actions=[spawn_jsb],  # action to fire after the delay
    )

    # -----------------------------------------------------------------------
    # Assemble and return the complete LaunchDescription.
    # Execution order within a LaunchDescription is top-down except that
    # TimerAction and RegisterEventHandler are asynchronous.
    # -----------------------------------------------------------------------
    return LaunchDescription([
        set_plugin_path,        # 1. Set env var before Gazebo reads it
        set_resource_path,      # 1b. Set mesh resource path for Gazebo
        world_arg,              # 2. Register world_file CLI argument
        gz_sim,                 # 3. Start Gazebo with custom world
        robot_state_publisher,  # 4. Start RSP (publishes robot_description)
        spawn_robot,            # 5. Spawn arm on workbench
        ros_gz_bridge,          # 6. Bridge /clock + /joint_states
        delayed_jsb,            # 7. After 3 s → spawn JSB
        spawn_arm,              # 8. After JSB → spawn arm_controller
        spawn_gripper,          # 9. After JSB → spawn gripper_controller
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
# *****************************************************************************************
# *  Filename:       joy_teleop.py
# *  Description:    Joystick teleoperation node for the Falcon Robotic Arm.
# *                  Maps gamepad/joystick axes to arm joint velocity increments and
# *                  buttons to gripper open/close commands, sending goals via the
# *                  FollowJointTrajectory action interface.
# *
# *                  Joint layout:
# *                    Joint 1-5    : arm joints — mapped to joystick axes
# *                    Gripper 1-2  : gripper — mapped to LB (close) / RB (open)
# *
# *                  Default axis mapping (configurable in __init__):
# *                    axis 0 → joint_1  (base rotation)
# *                    axis 1 → joint_2  (shoulder)
# *                    axis 3 → joint_3  (elbow)
# *                    axis 4 → joint_4  (wrist pitch)
# *                    axis 6 → joint_5  (wrist roll)
# *
# *  Author:         BARGAVAN R - MIT
# *  Co-Author:      RON NIRVIN J
# *
# *  Dependencies:
# *    ROS2 packages : control_msgs, trajectory_msgs, sensor_msgs, joy, rclpy
# *    Python stdlib  : time (monotonic clock for rate limiting)
# *
# *  Usage:
# *    ros2 launch falcon_arm_teleop teleop.launch.py
# *    — or —
# *    ros2 run falcon_arm_teleop joy_teleop.py
# *
# *  Prerequisites:
# *    • joy_node running  (publishes /joy from connected joystick/gamepad)
# *    • arm_controller and gripper_controller active
# *    • joint_state_broadcaster publishing /joint_states
# *
# *  Notes:
# *    • Deadzone 0.12 — joystick noise below this threshold is ignored.
# *    • arm_step = 0.08 rad per tick at full axis deflection.
# *    • Rate-limited to one goal send per 150 ms.
# *****************************************************************************************
'''

import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FalconJoyTeleop(Node):
    def __init__(self):
        super().__init__("falcon_joy_teleop")

        self.arm_joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
        self.gripper_joint_names = ["gripper_joint_1", "gripper_joint_2"]

        # Axis map for arm joints. Tune these indices for your joystick.
        self.axis_mapping = [0, 1, 3, 4, 6]
        self.deadzone = 0.12
        self.arm_step = 0.08
        self.gripper_step = 0.04
        self.send_interval_sec = 0.15

        # Joint limits from Falcon URDF.
        self.arm_limits = [(-1.5708, 1.5708)] * 5
        self.gripper_limits = [(-0.7, 0.0), (-0.7, 0.0)]

        # Buttons for gripper control (LB close, RB open by default).
        self.close_button = 4
        self.open_button = 5

        self.current_arm = [0.0] * len(self.arm_joint_names)
        self.current_gripper = [0.0] * len(self.gripper_joint_names)
        self.have_joint_states = False
        self.last_send_time = 0.0

        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

        self.create_subscription(JointState, "/joint_states", self._joint_states_cb, 20)
        self.create_subscription(Joy, "/joy", self._joy_cb, 20)

        self.get_logger().info("Falcon joystick teleop ready. Waiting for /joint_states...")

    def _joint_states_cb(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        found_all = True
        for i, joint in enumerate(self.arm_joint_names):
            idx = name_to_index.get(joint)
            if idx is None:
                found_all = False
                continue
            self.current_arm[i] = msg.position[idx]

        for i, joint in enumerate(self.gripper_joint_names):
            idx = name_to_index.get(joint)
            if idx is None:
                found_all = False
                continue
            self.current_gripper[i] = msg.position[idx]

        if found_all and not self.have_joint_states:
            self.have_joint_states = True
            self.get_logger().info("Joint states received. Teleop active.")

    def _joy_cb(self, msg: Joy) -> None:
        if not self.have_joint_states:
            return

        now = time.monotonic()
        if now - self.last_send_time < self.send_interval_sec:
            return

        arm_goal = list(self.current_arm)
        gripper_goal = list(self.current_gripper)
        arm_changed = False
        gripper_changed = False

        for joint_idx, axis_idx in enumerate(self.axis_mapping):
            if axis_idx >= len(msg.axes):
                continue
            axis_val = msg.axes[axis_idx]
            if abs(axis_val) < self.deadzone:
                continue
            lower, upper = self.arm_limits[joint_idx]
            arm_goal[joint_idx] = self._clamp(
                arm_goal[joint_idx] + axis_val * self.arm_step, lower, upper
            )
            arm_changed = True

        close_pressed = self._button_pressed(msg, self.close_button)
        open_pressed = self._button_pressed(msg, self.open_button)

        if close_pressed and not open_pressed:
            for i, (lower, upper) in enumerate(self.gripper_limits):
                gripper_goal[i] = self._clamp(gripper_goal[i] + self.gripper_step, lower, upper)
            gripper_changed = True
        elif open_pressed and not close_pressed:
            for i, (lower, upper) in enumerate(self.gripper_limits):
                gripper_goal[i] = self._clamp(gripper_goal[i] - self.gripper_step, lower, upper)
            gripper_changed = True

        if arm_changed:
            self._send_goal(self.arm_client, self.arm_joint_names, arm_goal, 0.20)
        if gripper_changed:
            self._send_goal(self.gripper_client, self.gripper_joint_names, gripper_goal, 0.25)

        if arm_changed or gripper_changed:
            self.last_send_time = now

    @staticmethod
    def _button_pressed(msg: Joy, index: int) -> bool:
        return index < len(msg.buttons) and msg.buttons[index] == 1

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))

    def _send_goal(
        self,
        client: ActionClient,
        joint_names: list[str],
        positions: list[float],
        duration_sec: float,
    ) -> None:
        if not client.server_is_ready() and not client.wait_for_server(timeout_sec=0.1):
            return

        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points = [point]
        goal.trajectory = traj
        client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = FalconJoyTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

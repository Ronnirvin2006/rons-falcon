#!/usr/bin/env python3
"""
Joystick → MoveIt Servo bridge for Falcon Arm.

Converts sensor_msgs/Joy to geometry_msgs/TwistStamped and publishes
to MoveIt Servo's delta_twist_cmds topic.  Also controls the gripper
via the gripper_controller and can recover from singularities.

Controller: Cosmic Byte Ares (Xbox-compatible layout)
  Press START to enable joystick control (toggle on/off)

  While enabled:
    Left stick Y  → linear.x  (forward / back)
    Left stick X  → linear.y  (left / right)
    Right stick Y → linear.z  (up / down)
    Right stick X → angular.z (yaw)
    LT            → close gripper
    RT            → open gripper
    X             → recover to safe "ready" position (singularity escape)
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


# Xbox-compatible axis indices
LX, LY, LT, RX, RY, RT = 0, 1, 2, 3, 4, 5

# Xbox-compatible button indices
A, B, X, Y_BTN = 0, 1, 2, 3
LB, RB = 4, 5
VIEW, MENU = 6, 7

# Safe "ready" pose for singularity recovery (matches SRDF ready state)
SAFE_POSITIONS = [-0.5988, -0.3905, -0.1996, 0.4252, 0.1996]
ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']


class JoyServo(Node):
    def __init__(self):
        super().__init__('joy_servo')

        # Parameters
        self.declare_parameter('base_frame', 'world')
        self.declare_parameter('ee_frame', 'link_5')
        self.declare_parameter('linear_scale', 0.3)
        self.declare_parameter('angular_scale', 0.6)

        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        # Command frame (always base frame for now)
        self.command_frame = self.base_frame

        # Enable state (toggled by START button)
        self.enabled = False
        self._prev_start = False

        # Gripper state
        self.gripper_position = 0.0  # 0.0 = open, -0.7 = closed
        self.gripper_step = 0.02     # increment per joy callback

        # Twist publisher for MoveIt Servo
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )

        # Gripper command publisher (JointGroupPositionController)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10
        )

        # Arm action client for singularity recovery
        self.arm_action = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Recovery flag to avoid spamming
        self._recovering = False

        # Call start_servo service
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.get_logger().info('Waiting for /servo_node/start_servo service...')
        self.cli.wait_for_service(timeout_sec=30.0)
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self._start_cb)

        self.get_logger().info(
            'JoyServo ready — press START to enable, sticks to move EE, '
            'LT/RT for gripper, X for safe position'
        )

    def _start_cb(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'start_servo: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'start_servo failed: {e}')

    def _publish_gripper(self):
        """Publish current gripper position to gripper_controller."""
        msg = JointTrajectory()
        msg.joint_names = ['gripper_joint_1']
        pt = JointTrajectoryPoint()
        pt.positions = [self.gripper_position]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 0.1s
        msg.points = [pt]
        self.gripper_pub.publish(msg)

    def _send_safe_position(self):
        """Send arm to safe ready pose via arm_controller action."""
        if self._recovering:
            return
        if not self.arm_action.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('arm_controller action server not available')
            return

        self._recovering = True
        self.get_logger().info('Recovering to safe position...')

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = SAFE_POSITIONS
        pt.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [pt]

        future = self.arm_action.send_goal_async(goal)
        future.add_done_callback(self._recovery_response_cb)

    def _recovery_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Recovery goal rejected')
            self._recovering = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._recovery_done_cb)

    def _recovery_done_cb(self, future):
        self._recovering = False
        self.get_logger().info('Recovery to safe position complete')

    def joy_cb(self, msg: Joy):
        # Toggle enable with START button (rising edge)
        start_pressed = len(msg.buttons) > MENU and msg.buttons[MENU]
        if start_pressed and not self._prev_start:
            self.enabled = not self.enabled
            state = 'ENABLED' if self.enabled else 'DISABLED'
            self.get_logger().info(f'Joystick control {state}')
        self._prev_start = start_pressed

        if not self.enabled:
            return

        axes = msg.axes

        # X button → recover to safe position
        if len(msg.buttons) > X and msg.buttons[X]:
            self._send_safe_position()
            return

        # LT/RT → gripper control
        # Triggers default to 1.0 (released) → -1.0 (fully pressed)
        if len(axes) > RT:
            lt_val = (1.0 - axes[LT]) / 2.0 if len(axes) > LT else 0.0  # 0→1
            rt_val = (1.0 - axes[RT]) / 2.0 if len(axes) > RT else 0.0   # 0→1
            if lt_val > 0.1:  # LT pressed → close
                self.gripper_position = max(-0.7, self.gripper_position - self.gripper_step * lt_val)
                self._publish_gripper()
            elif rt_val > 0.1:  # RT pressed → open
                self.gripper_position = min(0.0, self.gripper_position + self.gripper_step * rt_val)
                self._publish_gripper()

        # Twist command for MoveIt Servo
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame

        # Left stick → linear X / Y
        twist.twist.linear.x = axes[LY] * self.linear_scale if len(axes) > LY else 0.0
        twist.twist.linear.y = axes[LX] * self.linear_scale if len(axes) > LX else 0.0

        # Right stick Y → linear Z (up/down)
        twist.twist.linear.z = axes[RY] * self.linear_scale if len(axes) > RY else 0.0

        # Right stick X → angular Z (yaw)
        twist.twist.angular.z = axes[RX] * self.angular_scale if len(axes) > RX else 0.0

        self.twist_pub.publish(twist)


def main():
    rclpy.init()
    node = JoyServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

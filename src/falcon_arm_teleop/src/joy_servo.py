#!/usr/bin/env python3
"""
Joystick → MoveIt Servo bridge for Falcon Arm.

Converts sensor_msgs/Joy to geometry_msgs/TwistStamped and publishes
to MoveIt Servo's delta_twist_cmds topic.

Controller: Cosmic Byte Ares (Xbox-compatible layout)
  Hold LB (deadman) + sticks to move:
    Left stick Y  → linear.x  (forward / back)
    Left stick X  → linear.y  (left / right)
    Right stick Y → linear.z  (up / down)
    Right stick X → angular.z (yaw)
    LT / RT       → angular.y / angular.x (pitch / roll)

  MENU (Start)  → switch to EE frame
  VIEW (Back)   → switch to base frame
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


# Xbox-compatible axis indices
LX, LY, LT, RX, RY, RT = 0, 1, 2, 3, 4, 5

# Xbox-compatible button indices
A, B, X, Y = 0, 1, 2, 3
LB, RB = 4, 5
VIEW, MENU = 6, 7


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

        # Start in base frame
        self.command_frame = self.base_frame

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Call start_servo service
        self.cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.get_logger().info('Waiting for /servo_node/start_servo service...')
        self.cli.wait_for_service(timeout_sec=30.0)
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self._start_cb)

        self.get_logger().info(
            'JoyServo ready — hold LB + use sticks to control end effector'
        )

    def _start_cb(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'start_servo: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'start_servo failed: {e}')

    def joy_cb(self, msg: Joy):
        # Safety: must hold LB (deadman switch)
        if len(msg.buttons) <= LB or not msg.buttons[LB]:
            return

        # Frame switching
        if len(msg.buttons) > MENU:
            if msg.buttons[VIEW]:
                if self.command_frame != self.base_frame:
                    self.command_frame = self.base_frame
                    self.get_logger().info(f'Switched to base frame: {self.base_frame}')
            elif msg.buttons[MENU]:
                if self.command_frame != self.ee_frame:
                    self.command_frame = self.ee_frame
                    self.get_logger().info(f'Switched to EE frame: {self.ee_frame}')

        axes = msg.axes
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

        # Triggers for pitch/roll (triggers default to 1.0 when unpressed on some drivers)
        if len(axes) > RT:
            # LT → angular.y (pitch), RT → angular.x (roll)
            # Triggers go from 1.0 (released) to -1.0 (fully pressed)
            lt = (1.0 - axes[LT]) / 2.0 if len(axes) > LT else 0.0
            rt = (1.0 - axes[RT]) / 2.0 if len(axes) > RT else 0.0
            twist.twist.angular.y = (lt - rt) * self.angular_scale

        self.twist_pub.publish(twist)


def main():
    rclpy.init()
    node = JoyServo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

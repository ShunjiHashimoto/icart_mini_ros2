#!/usr/bin/env python3
import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoystickFollowMeTeleop(Node):
    def __init__(self):
        super().__init__('joystick_follow_me_teleop')

        self.axis_linear = int(self.declare_parameter('axis_linear', 1).value)
        self.axis_angular = int(self.declare_parameter('axis_angular', 0).value)
        self.linear_scale_robot = float(self.declare_parameter('linear_scale_robot', 0.4).value)
        self.angular_scale_robot = float(self.declare_parameter('angular_scale_robot', 1.0).value)
        self.linear_scale_person = float(self.declare_parameter('linear_scale_person', 0.25).value)
        self.angular_scale_person = float(self.declare_parameter('angular_scale_person', 0.8).value)
        self.deadzone = float(self.declare_parameter('deadzone', 0.08).value)

        self.start_button = int(self.declare_parameter('start_button', 7).value)
        self.stop_button = int(self.declare_parameter('stop_button', 6).value)
        self.emergency_button = int(self.declare_parameter('emergency_button', 5).value)
        self.clear_emergency_button = int(self.declare_parameter('clear_emergency_button', 4).value)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.person_cmd_publisher = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.person_control_publisher = self.create_publisher(String, '/person/control', 10)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.mode = 'robot'
        self.previous_buttons: List[int] = []
        self.get_logger().info(
            'Joystick teleop ready: robot mode before follow, person mode after follow start.'
        )

    def joy_callback(self, msg: Joy):
        if not self.previous_buttons:
            self.previous_buttons = [0] * len(msg.buttons)

        if self.button_pressed(msg, self.start_button):
            self.publish_string(self.person_control_publisher, 'start')
            self.mode = 'person'
            self.publish_zero(self.cmd_vel_publisher)
            self.get_logger().info(
                'Start button pressed; leg tracker handles follow start from /joy, joystick mode switched to person.'
            )
        elif self.button_pressed(msg, self.stop_button):
            self.publish_string(self.person_control_publisher, 'stop')
            self.mode = 'robot'
            self.publish_zero(self.person_cmd_publisher)
            self.publish_zero(self.cmd_vel_publisher)
            self.get_logger().info(
                'Stop button pressed; leg tracker handles follow stop from /joy, joystick mode switched to robot.'
            )
        elif self.button_pressed(msg, self.emergency_button):
            self.publish_zero(self.person_cmd_publisher)
            self.publish_zero(self.cmd_vel_publisher)
            self.get_logger().warn('Emergency button pressed; leg tracker handles emergency stop from /joy.')
        elif self.button_pressed(msg, self.clear_emergency_button):
            self.get_logger().info('Clear emergency button pressed; leg tracker handles clear from /joy.')

        twist = self.make_twist(msg)
        if self.mode == 'robot':
            self.cmd_vel_publisher.publish(twist)
        else:
            self.person_cmd_publisher.publish(twist)

        self.previous_buttons = list(msg.buttons)

    def make_twist(self, msg: Joy) -> Twist:
        linear_axis = self.axis_value(msg.axes, self.axis_linear)
        angular_axis = self.axis_value(msg.axes, self.axis_angular)
        linear_axis = self.apply_deadzone(linear_axis)
        angular_axis = self.apply_deadzone(angular_axis)

        twist = Twist()
        if self.mode == 'robot':
            twist.linear.x = linear_axis * self.linear_scale_robot
            twist.angular.z = angular_axis * self.angular_scale_robot
        else:
            twist.linear.x = linear_axis * self.linear_scale_person
            twist.angular.z = angular_axis * self.angular_scale_person
        return twist

    def button_pressed(self, msg: Joy, index: int) -> bool:
        if index < 0 or index >= len(msg.buttons):
            return False
        previous = self.previous_buttons[index] if index < len(self.previous_buttons) else 0
        return msg.buttons[index] == 1 and previous == 0

    def axis_value(self, axes, index: int) -> float:
        if index < 0 or index >= len(axes):
            return 0.0
        return float(axes[index])

    def apply_deadzone(self, value: float) -> float:
        if math.fabs(value) < self.deadzone:
            return 0.0
        return value

    def publish_string(self, publisher, data: str):
        msg = String()
        msg.data = data
        publisher.publish(msg)

    def publish_zero(self, publisher):
        publisher.publish(Twist())


def main():
    rclpy.init()
    node = JoystickFollowMeTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

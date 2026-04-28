#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float):
    half_yaw = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class MovingLegPairController(Node):
    def __init__(self):
        super().__init__('moving_leg_pair_controller')

        self.model_name = self.declare_parameter('model_name', 'leg_pair').value
        self.reference_frame = self.declare_parameter('reference_frame', 'world').value
        self.path_mode = self.declare_parameter('path_mode', 'straight').value
        self.auto_start = as_bool(self.declare_parameter('auto_start', True).value)
        self.initial_x = float(self.declare_parameter('initial_x', 0.5).value)
        self.initial_y = float(self.declare_parameter('initial_y', 0.0).value)
        self.initial_z = float(self.declare_parameter('initial_z', 0.0).value)
        self.initial_yaw = float(self.declare_parameter('initial_yaw', 0.0).value)
        self.update_rate = float(self.declare_parameter('update_rate', 30.0).value)
        self.straight_speed = float(self.declare_parameter('straight_speed', 0.12).value)
        self.straight_min_x = float(self.declare_parameter('straight_min_x', 0.45).value)
        self.straight_max_x = float(self.declare_parameter('straight_max_x', 1.5).value)
        self.circle_center_x = float(self.declare_parameter('circle_center_x', 0.8).value)
        self.circle_center_y = float(self.declare_parameter('circle_center_y', 0.0).value)
        self.circle_radius = float(self.declare_parameter('circle_radius', 0.45).value)
        self.circle_angular_speed = float(self.declare_parameter('circle_angular_speed', 0.35).value)
        self.manual_timeout = float(self.declare_parameter('manual_timeout', 0.5).value)

        self.x = self.initial_x
        self.y = self.initial_y
        self.z = self.initial_z
        self.yaw = self.initial_yaw
        self.circle_phase = 0.0
        self.straight_direction = 1.0
        self.paused = not self.auto_start
        self.last_time = self.get_clock().now()
        self.last_cmd_time: Optional[rclpy.time.Time] = None
        self.manual_cmd = Twist()
        self.pending_request = None

        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        self.cmd_subscriber = self.create_subscription(Twist, '/person/cmd_vel', self.cmd_callback, 10)
        self.control_subscriber = self.create_subscription(String, '/person/control', self.control_callback, 10)
        self.event_publisher = self.create_publisher(String, '/person/motion_event', 10)

        timer_period = 1.0 / max(self.update_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publish_event('started' if self.auto_start else 'paused')
        self.get_logger().info(
            f"Moving leg pair controller ready: model={self.model_name}, mode={self.path_mode}"
        )

    def cmd_callback(self, msg: Twist):
        self.manual_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def control_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command in ('start', 'resume'):
            self.paused = False
            self.publish_event('resumed')
        elif command in ('stop', 'pause'):
            self.paused = True
            self.publish_event('paused')
        elif command == 'reset':
            self.reset_pose()
            self.publish_event('reset')
        elif command in ('mode:straight', 'straight'):
            self.path_mode = 'straight'
            self.publish_event('mode:straight')
        elif command in ('mode:circle', 'circle', 'curve'):
            self.path_mode = 'circle'
            self.publish_event('mode:circle')
        elif command in ('mode:manual', 'manual'):
            self.path_mode = 'manual'
            self.publish_event('mode:manual')
        else:
            self.get_logger().warn(f"Unknown /person/control command: {msg.data}")

    def reset_pose(self):
        self.x = self.initial_x
        self.y = self.initial_y
        self.z = self.initial_z
        self.yaw = self.initial_yaw
        self.circle_phase = 0.0
        self.straight_direction = 1.0
        self.send_state()

    def publish_event(self, data: str):
        msg = String()
        msg.data = data
        self.event_publisher.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 0.0)
        self.last_time = now

        if self.pending_request is not None and not self.pending_request.done():
            return

        if not self.set_state_client.service_is_ready():
            self.set_state_client.wait_for_service(timeout_sec=0.0)
            return

        if not self.paused:
            self.update_pose(dt, now)
        self.send_state()

    def update_pose(self, dt: float, now):
        if self.is_manual_active(now):
            self.integrate_manual(dt)
        elif self.path_mode == 'circle':
            self.integrate_circle(dt)
        elif self.path_mode == 'straight':
            self.integrate_straight(dt)

    def is_manual_active(self, now) -> bool:
        if self.last_cmd_time is None:
            return False
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        return age <= self.manual_timeout

    def integrate_manual(self, dt: float):
        linear_x = self.manual_cmd.linear.x
        linear_y = self.manual_cmd.linear.y
        angular_z = self.manual_cmd.angular.z
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (linear_x * cos_yaw - linear_y * sin_yaw) * dt
        self.y += (linear_x * sin_yaw + linear_y * cos_yaw) * dt
        self.yaw += angular_z * dt

    def integrate_straight(self, dt: float):
        self.x += self.straight_direction * self.straight_speed * dt
        if self.x > self.straight_max_x:
            self.x = self.straight_max_x
            self.straight_direction = -1.0
            self.yaw = math.pi
        elif self.x < self.straight_min_x:
            self.x = self.straight_min_x
            self.straight_direction = 1.0
            self.yaw = 0.0

    def integrate_circle(self, dt: float):
        self.circle_phase += self.circle_angular_speed * dt
        self.x = self.circle_center_x + self.circle_radius * math.cos(self.circle_phase)
        self.y = self.circle_center_y + self.circle_radius * math.sin(self.circle_phase)
        self.yaw = self.circle_phase + math.pi / 2.0

    def send_state(self):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        request.state.reference_frame = self.reference_frame
        request.state.pose.position.x = self.x
        request.state.pose.position.y = self.y
        request.state.pose.position.z = self.z
        quat = yaw_to_quaternion(self.yaw)
        request.state.pose.orientation.x = quat['x']
        request.state.pose.orientation.y = quat['y']
        request.state.pose.orientation.z = quat['z']
        request.state.pose.orientation.w = quat['w']
        self.pending_request = self.set_state_client.call_async(request)


def main():
    rclpy.init()
    node = MovingLegPairController()
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

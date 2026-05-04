#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
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


class InvertedPendulumBipedController(Node):
    def __init__(self):
        super().__init__('inverted_pendulum_biped_controller')

        self.left_leg_name = self.declare_parameter('left_leg_name', 'biped_left_leg').value
        self.right_leg_name = self.declare_parameter('right_leg_name', 'biped_right_leg').value
        self.direction_marker_name = self.declare_parameter(
            'direction_marker_name', 'biped_direction_marker'
        ).value
        self.pose_service = self.declare_parameter(
            'pose_service', '/world/follow_me_empty/set_pose'
        ).value
        self.path_mode = self.declare_parameter('path_mode', 'straight').value
        self.auto_start = as_bool(self.declare_parameter('auto_start', True).value)
        self.initial_x = float(self.declare_parameter('initial_x', 0.5).value)
        self.initial_y = float(self.declare_parameter('initial_y', 0.0).value)
        self.initial_z = float(self.declare_parameter('initial_z', 0.0).value)
        self.initial_yaw = float(self.declare_parameter('initial_yaw', 0.0).value)
        self.update_rate = float(self.declare_parameter('update_rate', 30.0).value)
        self.straight_speed = float(self.declare_parameter('straight_speed', 0.12).value)
        self.straight_max_x = float(self.declare_parameter('straight_max_x', 1.5).value)
        self.step_length = float(self.declare_parameter('step_length', 0.24).value)
        self.step_width = float(self.declare_parameter('step_width', 0.22).value)
        self.step_frequency = float(self.declare_parameter('step_frequency', 1.2).value)
        self.manual_timeout = float(self.declare_parameter('manual_timeout', 0.5).value)
        self.motion_epsilon = float(self.declare_parameter('motion_epsilon', 0.02).value)
        self.startup_delay = float(self.declare_parameter('startup_delay', 1.0).value)

        self.x = self.initial_x
        self.y = self.initial_y
        self.z = self.initial_z
        self.yaw = self.initial_yaw
        self.phase = 0.0
        self.paused = not self.auto_start
        self.straight_finished = False
        self.last_time = self.get_clock().now()
        self.last_cmd_time: Optional[rclpy.time.Time] = None
        self.manual_cmd = Twist()
        self.pending_requests = []
        self.start_time = self.get_clock().now()
        self.next_failure_log_time = self.start_time

        self.set_pose_client = self.create_client(SetEntityPose, self.pose_service)
        self.cmd_subscriber = self.create_subscription(Twist, '/person/cmd_vel', self.cmd_callback, 10)
        self.control_subscriber = self.create_subscription(String, '/person/control', self.control_callback, 10)
        self.event_publisher = self.create_publisher(String, '/person/motion_event', 10)

        timer_period = 1.0 / max(self.update_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.publish_event('started' if self.auto_start else 'paused')
        self.get_logger().info(
            'Inverted pendulum biped controller ready: '
            f'left={self.left_leg_name}, right={self.right_leg_name}, '
            f'pose_service={self.pose_service}'
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
        self.phase = 0.0
        self.straight_finished = False
        self.send_states()

    def publish_event(self, data: str):
        msg = String()
        msg.data = data
        self.event_publisher.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds * 1e-9, 0.0)
        self.last_time = now

        self.collect_finished_requests(now)
        if self.pending_requests:
            return

        if not self.set_pose_client.service_is_ready():
            self.set_pose_client.wait_for_service(timeout_sec=0.0)
            return

        # Gazebo Sim では spawn と controller 起動が並行するため、初回だけ少し待つ。
        # 待たないと entity 生成前の set_pose が失敗し、起動時ログが読みづらくなる。
        if (now - self.start_time).nanoseconds * 1e-9 < self.startup_delay:
            return

        if not self.paused:
            self.update_pose(dt, now)
        self.send_states()

    def update_pose(self, dt: float, now):
        should_step = False
        if self.is_manual_active(now):
            should_step = self.manual_speed() > self.motion_epsilon
            self.integrate_manual(dt)
        elif self.path_mode == 'straight':
            should_step = not self.straight_finished and abs(self.straight_speed) > self.motion_epsilon
            self.integrate_straight(dt)
        if should_step:
            self.phase += 2.0 * math.pi * self.step_frequency * dt

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

    def manual_speed(self) -> float:
        return math.hypot(self.manual_cmd.linear.x, self.manual_cmd.linear.y)

    def integrate_straight(self, dt: float):
        if self.straight_finished:
            return
        self.x += abs(self.straight_speed) * dt
        if self.x >= self.straight_max_x:
            self.x = self.straight_max_x
            self.straight_finished = True
            self.paused = True
            self.publish_event('straight_reached_max')

    def send_states(self):
        left_forward = 0.5 * self.step_length * math.sin(self.phase)
        right_forward = -left_forward

        self.call_set_state(
            self.left_leg_name,
            self.x + left_forward,
            self.y + self.step_width / 2.0,
            self.z,
            self.yaw,
        )
        self.call_set_state(
            self.right_leg_name,
            self.x + right_forward,
            self.y - self.step_width / 2.0,
            self.z,
            self.yaw,
        )
        if self.direction_marker_name:
            self.call_set_state(
                self.direction_marker_name,
                self.x,
                self.y,
                self.z,
                self.yaw,
            )

    def call_set_state(self, name: str, x: float, y: float, z: float, yaw: float):
        request = SetEntityPose.Request()
        request.entity.name = name
        request.entity.type = Entity.MODEL
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        quat = yaw_to_quaternion(yaw)
        request.pose.orientation.x = quat['x']
        request.pose.orientation.y = quat['y']
        request.pose.orientation.z = quat['z']
        request.pose.orientation.w = quat['w']
        self.pending_requests.append(self.set_pose_client.call_async(request))

    def collect_finished_requests(self, now):
        active_requests = []
        for request in self.pending_requests:
            if not request.done():
                active_requests.append(request)
                continue
            try:
                response = request.result()
            except Exception as exc:
                self.log_pose_failure(now, f'set_pose service call failed: {exc}')
                continue
            if not response.success:
                self.log_pose_failure(now, 'set_pose service returned success=false')
        self.pending_requests = active_requests

    def log_pose_failure(self, now, message: str):
        if now < self.next_failure_log_time:
            return
        self.get_logger().warn(message)
        self.next_failure_log_time = now + Duration(seconds=2.0)


def main():
    rclpy.init()
    node = InvertedPendulumBipedController()
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

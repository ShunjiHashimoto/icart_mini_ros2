#!/usr/bin/env python3
from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class MotionSegment:
    duration: float
    linear_x: float
    linear_y: float
    angular_z: float


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class ActorDebugMotionRunner(Node):
    def __init__(self):
        super().__init__('actor_debug_motion_runner')

        self.motion_scenario = self.declare_parameter('motion_scenario', 'straight').value
        self.start_delay = float(self.declare_parameter('start_delay', 6.0).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)
        self.linear_speed = float(self.declare_parameter('linear_speed', 0.25).value)
        self.duration = float(self.declare_parameter('duration', 10.0).value)
        self.turn_angular_speed = float(self.declare_parameter('turn_angular_speed', 0.35).value)
        self.stop_hold_duration = float(self.declare_parameter('stop_hold_duration', 1.0).value)
        self.stop_when_done = as_bool(self.declare_parameter('stop_when_done', True).value)

        self.person_cmd_pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.follow_control_pub = self.create_publisher(String, '/follow_me/control', 10)
        self.person_control_pub = self.create_publisher(String, '/person/control', 10)

        self.segments = self.make_segments()
        self.total_motion_duration = sum(segment.duration for segment in self.segments)
        self.start_time = self.get_clock().now()
        self.finish_requested = False
        self.done = False
        self.control_published = False

        timer_period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            'Actor debug motion runner ready: '
            f'scenario={self.motion_scenario}, speed={self.linear_speed:.3f}, '
            f'duration={self.total_motion_duration:.3f}, start_delay={self.start_delay:.3f}'
        )

    def make_segments(self) -> List[MotionSegment]:
        if self.motion_scenario == 'straight':
            return [MotionSegment(self.duration, self.linear_speed, 0.0, 0.0)]
        if self.motion_scenario == 'stop_restart_turn_left':
            return [
                MotionSegment(3.0, self.linear_speed, 0.0, 0.0),
                MotionSegment(2.0, 0.0, 0.0, 0.0),
                MotionSegment(4.0, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(3.0, self.linear_speed, 0.0, 0.0),
            ]
        if self.motion_scenario == 'orbit_left':
            return [MotionSegment(self.duration, self.linear_speed, 0.0, self.turn_angular_speed)]
        if self.motion_scenario == 'left_orbit_straight_right_orbit':
            return [
                MotionSegment(self.duration, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(3.0, self.linear_speed, 0.0, 0.0),
                MotionSegment(self.duration, self.linear_speed, 0.0, -self.turn_angular_speed),
            ]
        if self.motion_scenario == 'left_orbit_straight_right_orbit_goal_straight':
            return [
                MotionSegment(self.duration, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(3.0, self.linear_speed, 0.0, 0.0),
                MotionSegment(self.duration, self.linear_speed, 0.0, -self.turn_angular_speed),
                MotionSegment(20.0, self.linear_speed, 0.0, 0.0),
            ]
        if self.motion_scenario == 'diagonal_walk':
            # Actor pluginは横速度を使わないため、短く旋回してから直進し、斜め歩行を再現する。
            return [
                MotionSegment(2.0, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(self.duration, self.linear_speed, 0.0, 0.0),
            ]
        if self.motion_scenario == 'front_crossing':
            # ロボット前方を軽く横切るよう、近めの初期位置から左旋回しながら通過する。
            return [
                MotionSegment(self.duration, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(2.0, self.linear_speed, 0.0, 0.0),
            ]
        if self.motion_scenario == 'turning_forward_walk':
            # 前進しながら左右へ向きを変え、脚の見え方が変わる状態を継続的に作る。
            return [
                MotionSegment(4.0, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(4.0, self.linear_speed, 0.0, -self.turn_angular_speed),
                MotionSegment(4.0, self.linear_speed, 0.0, self.turn_angular_speed),
                MotionSegment(4.0, self.linear_speed, 0.0, 0.0),
            ]
        self.get_logger().warn(
            f'Unknown motion_scenario [{self.motion_scenario}], falling back to straight.'
        )
        return [MotionSegment(self.duration, self.linear_speed, 0.0, 0.0)]

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        if elapsed < self.start_delay:
            return

        motion_elapsed = elapsed - self.start_delay
        if not self.control_published:
            self.publish_control('start')
            self.control_published = True

        if motion_elapsed <= self.total_motion_duration:
            self.publish_twist(self.segment_at(motion_elapsed))
            return

        self.publish_zero()
        if motion_elapsed > self.total_motion_duration + self.stop_hold_duration:
            if not self.finish_requested:
                self.finish_requested = True
                self.get_logger().info('Actor debug motion completed.')
            if self.stop_when_done:
                self.done = True

    def publish_control(self, command: str):
        msg = String()
        msg.data = command
        self.follow_control_pub.publish(msg)
        self.person_control_pub.publish(msg)

    def segment_at(self, elapsed: float) -> MotionSegment:
        cursor = 0.0
        for segment in self.segments:
            cursor += segment.duration
            if elapsed <= cursor:
                return segment
        return self.segments[-1]

    def publish_twist(self, segment: MotionSegment):
        msg = Twist()
        msg.linear.x = segment.linear_x
        msg.linear.y = segment.linear_y
        msg.angular.z = segment.angular_z
        self.person_cmd_pub.publish(msg)

    def publish_zero(self):
        self.person_cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ActorDebugMotionRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_zero()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

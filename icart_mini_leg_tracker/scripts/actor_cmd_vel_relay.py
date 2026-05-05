#!/usr/bin/env python3
"""Gazebo Fortress 移行中だけ使う Actor cmd_vel relay。

`/person/cmd_vel` は脚プロキシも購読するため、Actorだけ速度感を変えたい場合に
このrelayで `/person_actor/cmd_vel` へ中継しながら倍率を掛ける。

Fortress への完全移行で Actor 側の速度スケールを標準設定にできたら、
このスクリプト、launch からの起動、actor_bridge の relay topic 設定は削除予定。
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ActorCmdVelRelay(Node):
    def __init__(self):
        super().__init__('actor_cmd_vel_relay')

        self.input_topic = self.declare_parameter('input_topic', '/person/cmd_vel').value
        self.output_topic = self.declare_parameter('output_topic', '/person_actor/cmd_vel').value
        self.linear_scale = float(self.declare_parameter('linear_scale', 1.0).value)
        self.angular_scale = float(self.declare_parameter('angular_scale', 1.0).value)

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)

        self.get_logger().info(
            f'Actor cmd_vel relay ready: {self.input_topic} -> {self.output_topic}, '
            f'linear_scale={self.linear_scale:.2f}, angular_scale={self.angular_scale:.2f}'
        )

    def cmd_callback(self, msg: Twist):
        scaled = Twist()
        # 円柱プロキシとActorの速度感が大きく違うため、Actorへ渡す直前だけ倍率を掛ける。
        scaled.linear.x = msg.linear.x * self.linear_scale
        scaled.linear.y = msg.linear.y * self.linear_scale
        scaled.linear.z = msg.linear.z * self.linear_scale
        scaled.angular.x = msg.angular.x
        scaled.angular.y = msg.angular.y
        scaled.angular.z = msg.angular.z * self.angular_scale
        self.publisher.publish(scaled)


def main():
    rclpy.init()
    node = ActorCmdVelRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            # 終了処理中に追加のCtrl+Cが入っても、不要なtracebackを出さずに終了する。
            pass


if __name__ == '__main__':
    main()

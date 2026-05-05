#!/usr/bin/env python3
"""Gazebo Fortress の scoped frame_id を既存ノード向けに整える relay。

Gazebo Fortress が出す `icart_mini/odom` などのスコープ付き frame を、
追従ノードやRVizが前提とする `odom` / `base_footprint` / `laser` に直して
RViz と既存ノードから同じ名前で見えるようにする。
"""

import copy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class FortressFrameRelay(Node):
    def __init__(self):
        super().__init__('fortress_frame_relay')

        self.raw_odom_topic = self.declare_parameter('raw_odom_topic', '/fortress/odom_raw').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.raw_scan_topic = self.declare_parameter('raw_scan_topic', '/fortress/scan_raw').value
        self.scan_topic = self.declare_parameter('scan_topic', '/scan').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.scan_frame = self.declare_parameter('scan_frame', 'laser').value
        self.publish_tf = bool(self.declare_parameter('publish_tf', True).value)

        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.scan_publisher = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Odometry, self.raw_odom_topic, self.odom_callback, 10)
        self.create_subscription(LaserScan, self.raw_scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            'Fortress frame relay ready: '
            f'{self.raw_odom_topic} -> {self.odom_topic} '
            f'({self.odom_frame} -> {self.base_frame}), '
            f'{self.raw_scan_topic} -> {self.scan_topic} ({self.scan_frame})'
        )

    def odom_callback(self, msg: Odometry):
        odom = copy.deepcopy(msg)
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        self.odom_publisher.publish(odom)

        if self.tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def scan_callback(self, msg: LaserScan):
        scan = copy.deepcopy(msg)
        scan.header.frame_id = self.scan_frame
        self.scan_publisher.publish(scan)


def main():
    rclpy.init()
    node = FortressFrameRelay()
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LED status monitor for icart_mini_leg_tracker.

This node listens to `/leg_tracker/is_lost_target` and drives two LEDs:
- Green LED (GPIO25) indicates normal tracking
- Red LED (GPIO26) indicates the target is lost

If GPIO access is unavailable (e.g. running on a desktop), the node keeps
running but logs warnings.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from gpiozero import LED
except ImportError:  # pragma: no cover - hardware dependency
    LED = None  # type: ignore

GREEN_LED_PIN = 25
RED_LED_PIN = 26


class LedStatusNode(Node):
    """ROS2 node that reflects tracking status on GPIO LEDs."""

    def __init__(self) -> None:
        super().__init__('led_status_node')
        self.get_logger().info('Starting LED status monitor')

        self._green_led = self._init_led(GREEN_LED_PIN, 'green')
        self._red_led = self._init_led(RED_LED_PIN, 'red')
        self._last_state = None  # type: ignore

        self._subscription = self.create_subscription(
            Bool,
            '/leg_tracker/is_lost_target',
            self._callback,
            10,
        )

        if self._green_led is None or self._red_led is None:
            self.get_logger().warn(
                'GPIO access not available; LED control disabled. '
                'Install gpiozero and ensure the script runs on the target hardware.'
            )
        else:
            self._apply_state(False)

    def _init_led(self, pin: int, color: str):
        if LED is None:
            return None
        try:
            led = LED(pin)
        except Exception as exc:  # pragma: no cover - hardware dependency
            self.get_logger().error(f'Failed to initialize {color} LED on GPIO{pin}: {exc}')
            return None
        return led

    def _callback(self, msg: Bool) -> None:
        self._apply_state(bool(msg.data))

    def _apply_state(self, lost: bool) -> None:
        if self._last_state == lost:
            return
        self._last_state = lost

        if self._green_led is None or self._red_led is None:
            state = 'lost' if lost else 'tracking'
            self.get_logger().info(f'Status changed to {state}, but LEDs are disabled.')
            return

        if lost:
            self._red_led.on()
            self._green_led.off()
            self.get_logger().info('Target lost: red LED ON, green LED OFF')
        else:
            self._green_led.on()
            self._red_led.off()
            self.get_logger().info('Tracking normal: green LED ON, red LED OFF')

    def destroy_node(self) -> bool:
        try:
            if self._green_led:
                self._green_led.off()
            if self._red_led:
                self._red_led.off()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LedStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LED status monitor interrupted, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

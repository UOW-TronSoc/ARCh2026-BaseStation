#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from custom_msgs.msg import BatteryFeedback  # Replace with your actual package
import random
import time
import threading
from datetime import datetime


class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')

        self.publisher = self.create_publisher(BatteryFeedback, '/battery_feedback', 10)

        # Simulated starting values
        self.charge_pct = 100.0
        self.current_draw = 5.0  # amps
        self.temperature = 30.0  # degrees Celsius

        # Start publishing loop
        self.timer = self.create_timer(2.0, self.publish_battery_data)  # every second

    def publish_battery_data(self):
        if self.charge_pct <= 0:
            self.charge_pct = 100.0  # reset battery after full drain

        msg = BatteryFeedback()
        msg.charge_pct = self.charge_pct
        msg.current_draw = self.current_draw + random.uniform(-1.0, 1.0)  # small variation
        msg.temperature = self.temperature + random.uniform(-0.5, 0.5)
        msg.timestamp = int(time.time())  # UNIX timestamp

        self.publisher.publish(msg)
        self.get_logger().info(
            f"🔋 Published battery: {msg.charge_pct:.1f}%, {msg.current_draw:.1f}A, {msg.temperature:.1f}°C"
        )

        self.charge_pct -= 1.0  # simulate 1% drain


def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import time

from custom_msgs.msg import RadioFeedback  # Replace with your actual package + message

class RadioTestPublisher(Node):
    def __init__(self):
        super().__init__('radio_test_publisher')
        self.publisher = self.create_publisher(RadioFeedback, '/radio_feedback', 10)
        self.timer = self.create_timer(0.2, self.publish_fake_radio_feedback)  # 200ms

        self.get_logger().info("Fake RadioFeedback publisher started (200ms interval)")

    def publish_fake_radio_feedback(self):
        msg = RadioFeedback()
        msg.signal_strength = float(random.randint(-90, -30))  # e.g. -67 dBm
        msg.ping_ms = random.randint(10, 100)  # Simulated ping in ms
        msg.rx_bytes = f"{round(random.uniform(0.5, 5.0), 2)} MB"
        msg.tx_bytes = f"{round(random.uniform(0.2, 2.0), 2)} MB"

        self.publisher.publish(msg)
        self.get_logger().info(
            f" Sent → Strength: {msg.signal_strength} dBm | Ping: {msg.ping_ms} ms | RX: {msg.rx_bytes} | TX: {msg.tx_bytes}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RadioTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

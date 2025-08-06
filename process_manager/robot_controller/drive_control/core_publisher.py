#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import CoreFeedback  # Replace with your actual package
import time
import random


class CoreFeedbackTestPublisher(Node):
    def __init__(self):
        super().__init__('core_feedback_test_publisher')
        self.publisher = self.create_publisher(CoreFeedback, '/core_feedback', 10)
        self.timer = self.create_timer(0.1, self.publish_feedback)  # 10 Hz

        self.value = -100
        self.increment = 2

    def publish_feedback(self):
        msg = CoreFeedback()
        msg.epoch_time = int(time.time() * 1000)

        # Simulated wheel values ramping from -100 to 100 and back
        wheel_array = [float(self.value)] * 4
        msg.wheel_position = wheel_array
        msg.wheel_velocity = [v * 0.5 for v in wheel_array]
        msg.wheel_torque = [v * 0.2 for v in wheel_array]

        # Simulated pitch and roll with slight noise
        msg.pitch = float(round(random.uniform(-5, 5) + self.value * 0.05, 2))
        msg.roll = float(round(random.uniform(-5, 5) - self.value * 0.03, 2))

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published: val={self.value}, pitch={msg.pitch}, roll={msg.roll}"
        )

        # Increment and loop
        self.value += self.increment
        if self.value >= 100 or self.value <= -100:
            self.increment *= -1


def main(args=None):
    rclpy.init(args=args)
    node = CoreFeedbackTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

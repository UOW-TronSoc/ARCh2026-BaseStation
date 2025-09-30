#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import DrivetrainFeedback  # your actual message
import time

class TestDrivetrainPublisher(Node):
    def __init__(self):
        super().__init__('test_drivetrain_publisher')
        self.publisher = self.create_publisher(DrivetrainFeedback, '/drivetrain_feedback', 10)
        self.timer = self.create_timer(0.2, self.publish_mock_data)  # 200ms = 5Hz
        self.start_time = time.time()
        self.value = 0
        self.direction = 1  # 1 = up, -1 = down

    def publish_mock_data(self):
        now = int(time.time())
        
        # Update simulated velocity
        self.value += 10 * self.direction

        # Clamp and reverse at bounds
        if self.value >= 100:
            self.direction = -1
        elif self.value <= -100:
            self.direction = 1

        # Use symmetrical values for left/right
        val_l = float(self.value)
        val_r = float(self.value)

        msg = DrivetrainFeedback()
        msg.epoch_time = now
        msg.wheel_velocity = [val_l, val_l, val_r, val_r]
        msg.wheel_position = [0.0] * 4
        msg.wheel_torque = [0.0] * 4

        self.publisher.publish(msg)
        self.get_logger().info(f"Published → L: {val_l}, R: {val_r}")

def main(args=None):
    rclpy.init(args=args)
    node = TestDrivetrainPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
import random
import time

from std_msgs.msg import String, Bool, Int32, Float64
from custom_msgs.msg import ScienceFeedback  # <-- Replace with your actual message package

class MockSciencePublisher(Node):
    def __init__(self):
        super().__init__('mock_science_publisher')
        self.publisher_ = self.create_publisher(ScienceFeedback, '/science_feedback', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # every 1 second

    def timer_callback(self):
        msg = ScienceFeedback()
        msg.rfid = random.choice(['Tag-A1', 'Tag-B2', 'Tag-C3', ''])  # simulate detection
        msg.moisture = round(random.uniform(5.0, 25.0), 2)
        msg.potentiometer = random.randint(0, 1023)
        msg.limit = random.choice([True, False])
        msg.height = round(random.uniform(0.0, 15.0), 2)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published mock feedback: {msg}')


def main(args=None):
    rclpy.init(args=args)
    node = MockSciencePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

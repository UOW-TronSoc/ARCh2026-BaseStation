import rclpy
from rclpy.node import Node
from custom_msgs.msg import ScienceFeedback  # Ensure this matches your package

import random
import time

class ScienceFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('science_feedback_publisher')
        self.publisher = self.create_publisher(ScienceFeedback, '/science_feedback', 10)
        self.timer = self.create_timer(2.0, self.publish_feedback)  # Publish every 2 seconds

    def publish_feedback(self):
        """Generate random sensor data and publish it."""
        msg = ScienceFeedback()
        msg.water_percent = round(random.uniform(30, 70), 2)
        msg.temperature = round(random.uniform(10, 40), 2)
        msg.ilmenite_percent = round(random.uniform(5, 20), 2)
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published Science Feedback: {msg.water_percent}% water, {msg.temperature}°C, {msg.ilmenite_percent}% ilmenite')

def main(args=None):
    rclpy.init(args=args)
    node = ScienceFeedbackPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

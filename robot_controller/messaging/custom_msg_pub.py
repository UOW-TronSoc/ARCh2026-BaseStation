import rclpy
from rclpy.node import Node
from custom_msgs.msg import CustomMessage  # Import the custom message

class CustomMessagePublisher(Node):
    def __init__(self):
        super().__init__('custom_message_publisher')
        self.publisher = self.create_publisher(CustomMessage, 'example_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every second
        self.get_logger().info("CustomMessage Publisher Node Initialized.")

    def publish_message(self):
        msg = CustomMessage()
        msg.epoch_time = self.get_clock().now().to_msg().sec  # Current time in seconds
        msg.data = "Hello from Django integration!"
        msg.flag = True

        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = CustomMessagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

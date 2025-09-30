import rclpy
from rclpy.node import Node
from custom_msgs.msg import CustomMessage  # Import the custom message

class CustomMessageSubscriber(Node):
    def __init__(self):
        super().__init__('custom_message_subscriber')
        self.subscription = self.create_subscription(
            CustomMessage,
            'example_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info("CustomMessage Subscriber Node Initialized.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = CustomMessageSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

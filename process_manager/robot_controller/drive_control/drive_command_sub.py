import rclpy
from rclpy.node import Node
from custom_msgs.msg import CustomMessage

class DriveCommandSubscriber(Node):
    def __init__(self):
        super().__init__('drive_command_subscriber')
        self.subscription = self.create_subscription(
            CustomMessage,
            'drive_commands',
            self.listener_callback,
            10
        )
        self.get_logger().info("Drive Command Subscriber Node Initialized.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received Drive Command: "
                               f"Left Drive: {msg.left_drive}, "
                               f"Right Drive: {msg.right_drive}, "
                               f"Data: {msg.data}, Flag: {msg.flag}")


def main(args=None):
    rclpy.init(args=args)
    node = DriveCommandSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

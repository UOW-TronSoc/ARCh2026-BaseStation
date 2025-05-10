import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera_0/image_raw',
            self.image_callback,
            10)
        self.current_frame = None

    def image_callback(self, msg):
        self.get_logger().info("Received an image!")
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_frame = np_arr.reshape((msg.height, msg.width, -1))

        # Display Image
        cv2.imshow("Camera Feed", self.current_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

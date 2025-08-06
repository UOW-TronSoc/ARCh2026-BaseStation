#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import argparse
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class VideoPublisher(Node):
    def __init__(self, camera_id: int, video_path: str):
        super().__init__(f'camera_{camera_id}_publisher')
        topic = f'/camera_{camera_id}/image_compressed'
        self.publisher = self.create_publisher(CompressedImage, topic, 10)
        self.get_logger().info(f'Publishing on {topic} from {video_path}')

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video file {video_path}')

        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            # loop back to start
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-id', type=int, required=True,
                        help='Which camera index to publish on')
    parser.add_argument('--video-path', type=str, required=True,
                        help='Path to the .mp4 file')
    args = parser.parse_args()

    rclpy.init()
    node = VideoPublisher(args.camera_id, args.video_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

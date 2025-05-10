import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import random
import time

class TestArmFeedbackPublisher(Node):
    def __init__(self):
        super().__init__('test_arm_feedback_publisher')
        self.publisher = self.create_publisher(JointState, '/arm_feedback', 10)
        self.timer = self.create_timer(1.0, self.publish_feedback)  # every 1 sec

        self.joint_names = ["theta1", "theta2", "theta3", "theta4", "theta5", "EE"]
        self.get_logger().info("🟢 Publishing fake arm feedback on /arm/feedback")

    def publish_feedback(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = [round(random.uniform(-180, 180), 2) for _ in self.joint_names]
        msg.velocity = [round(random.uniform(0, 90), 2) for _ in self.joint_names]

        self.publisher.publish(msg)
        self.get_logger().info(f"📤 Sent joint feedback: {msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = TestArmFeedbackPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

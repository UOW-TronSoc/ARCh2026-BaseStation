# fake_integrator.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class FakeJointIntegrator(Node):
    def __init__(self):
        super().__init__("fake_joint_integrator")
        # start with zeroed positions and velocities
        self.positions = [0.0] * 6
        self.velocities = [0.0] * 6
        self.last_time = self.get_clock().now()
        # subscribe to velocity commands
        self.sub = self.create_subscription(
            JointState,
            "/arm_velocity_command",
            self.vel_callback,
            10
        )
        # republish position on /arm_command so Django’s feedback subscriber sees it
        self.pub = self.create_publisher(JointState, "/arm_command", 10)
        # spin a timer at 20 Hz (50 ms) to integrate
        self.timer = self.create_timer(0.05, self.timer_callback)

    def vel_callback(self, msg: JointState):
        # Copy up to 6 velocities from the command, pad with zeros if fewer
        self.velocities = list(msg.velocity)[:6] + [0.0] * max(0, 6 - len(msg.velocity))

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        # Integrate: new_position = old_position + velocity * dt
        for i in range(6):
            self.positions[i] += self.velocities[i] * dt

        out = JointState()
        out.header.stamp = now.to_msg()
        out.name = [f"joint_{i}" for i in range(6)]
        out.position = self.positions[:6]
        out.velocity = self.velocities[:6]
        out.effort = []
        self.pub.publish(out)
        # Optional: log for debugging
        # self.get_logger().info(f"FakeIntegrator pos: {self.positions}")

def main(args=None):
    rclpy.init(args=args)
    node = FakeJointIntegrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

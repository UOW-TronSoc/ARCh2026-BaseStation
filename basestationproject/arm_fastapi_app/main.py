from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, field_validator
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import threading
import time
from typing import List, Optional

print("Arm FastAPI server starting...")

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ─── Pydantic Models ────────────────────────────────────────────────────────────

class ArmVelocityCommand(BaseModel):
    joint_velocities: List[float] = Field(default_factory=list)

    @field_validator('joint_velocities', mode='after')
    @classmethod
    def pad_velocities(cls, v):
        v = list(v)
        if len(v) == 5:
            v.append(0.0)
        while len(v) < 6:
            v.append(0.0)
        return v[:6]


class ArmEECommand(BaseModel):
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    j1_velocity: float = 0.0
    j5_velocity: float = 0.0
    j6_velocity: float = 0.0


class ArmModeCommand(BaseModel):
    mode: str

    @field_validator('mode', mode='after')
    @classmethod
    def validate_mode(cls, v):
        v = v.lower()
        if v not in ('joint', 'ee'):
            raise ValueError("mode must be 'joint' or 'ee'")
        return v


# ─── ROS2 Node Manager ──────────────────────────────────────────────────────────

class ROS2Manager:
    """Manages ROS2 lifecycle and spins nodes in a background thread."""
    def __init__(self):
        self.executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._nodes: List[Node] = []
        self._lock = threading.Lock()

    def init(self):
        rclpy.init()
        self.executor = MultiThreadedExecutor(num_threads=4)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while rclpy.ok():
            self.executor.spin_once(timeout_sec=0.01)

    def add_node(self, node: Node):
        with self._lock:
            self._nodes.append(node)
            self.executor.add_node(node)

    def shutdown(self):
        for node in self._nodes:
            node.destroy_node()
        if self.executor:
            self.executor.shutdown()
        rclpy.shutdown()


ros2_manager = ROS2Manager()


# ─── Arm Control Nodes ──────────────────────────────────────────────────────────

class ArmVelocityPublisher(Node):
    _J6_PWM_SPEED = 30.0
    _J6_MIN_ANGLE = 0.0
    _J6_MAX_ANGLE = 180.0
    _J6_START_ANGLE = 180.0

    def __init__(self):
        super().__init__('arm_velocity_publisher')
        self._j6_position = self._J6_START_ANGLE
        self._j6_last_time: Optional[float] = None
        self.publisher = self.create_publisher(JointState, '/kanga_arm/joint_control', 100)
        self.get_logger().info("ArmVelocityPublisher ready on /kanga_arm/joint_control")

    def publish_velocity(self, velocity_list: List[float]):
        vel = [max(-1.0, min(1.0, float(v))) for v in velocity_list[:5]]
        while len(vel) < 5:
            vel.append(0.0)
        j6_vel = float(velocity_list[5]) if len(velocity_list) > 5 else 0.0
        j6_vel = max(-1.0, min(1.0, j6_vel))
        vel.append(j6_vel)

        now = time.monotonic()
        dt = 0.0 if self._j6_last_time is None else (now - self._j6_last_time)
        self._j6_last_time = now

        self._j6_position += j6_vel * self._J6_PWM_SPEED * max(dt, 0.0)
        self._j6_position = max(self._J6_MIN_ANGLE, min(self._J6_MAX_ANGLE, self._j6_position))

        msg = JointState()
        msg.velocity = vel
        msg.name = ["j1", "j2", "j3", "j4", "j5", "j6"]
        msg.position = [0.0] * 6
        msg.position[5] = self._j6_position
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


class ArmEEPublisher(Node):
    def __init__(self):
        super().__init__('arm_ee_publisher')
        self.publisher = self.create_publisher(Twist, 'kanga_arm/ee_state_control', 100)
        self.get_logger().info("ArmEEPublisher ready on kanga_arm/ee_state_control")

    def publish_ee(self, linear_y: float, linear_z: float, angular_x: float):
        msg = Twist()
        msg.linear.x = float(linear_y)
        msg.linear.y = 0.0
        msg.linear.z = float(linear_z)
        msg.angular.x = 0.0
        msg.angular.y = float(angular_x)
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def publish_zero(self):
        self.publisher.publish(Twist())


class ArmModePublisher(Node):
    def __init__(self):
        super().__init__('arm_mode_publisher')
        self.publisher = self.create_publisher(Bool, 'kanga_arm/control_mode_joint', 100)
        self.get_logger().info("ArmModePublisher ready on kanga_arm/control_mode_joint")

    def publish_mode(self, is_joint: bool):
        msg = Bool()
        msg.data = is_joint
        self.publisher.publish(msg)


class ArmFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('arm_feedback_subscriber')
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self._callback, 10
        )
        self.latest_feedback = {}
        self.get_logger().info("ArmFeedbackSubscriber listening on /joint_states")

    def _callback(self, msg: JointState):
        positions_rad = list(msg.position)[:5]
        positions_deg = [float(p * 180.0 / 3.141592653589793) for p in positions_rad]

        velocities_rad = list(msg.velocity)[:5] if msg.velocity else [0.0] * len(positions_rad)
        velocities_deg = [float(v * 180.0 / 3.141592653589793) for v in velocities_rad]

        names = list(msg.name)[:5] if msg.name and len(msg.name) >= 5 else [f"J{i+1}" for i in range(5)]

        while len(positions_deg) < 5:
            positions_deg.append(0.0)
        while len(velocities_deg) < 5:
            velocities_deg.append(0.0)
        while len(names) < 5:
            names.append(f"J{len(names)+1}")

        self.latest_feedback = {
            "joint_positions": positions_deg,
            "joint_velocities": velocities_deg,
            "joint_names": names
        }


# ─── Global Node Instances ──────────────────────────────────────────────────────

arm_velocity_node: Optional[ArmVelocityPublisher] = None
arm_ee_node: Optional[ArmEEPublisher] = None
arm_mode_node: Optional[ArmModePublisher] = None
arm_feedback_node: Optional[ArmFeedbackSubscriber] = None


@app.on_event("startup")
async def startup_event():
    global arm_velocity_node, arm_ee_node, arm_mode_node, arm_feedback_node

    ros2_manager.init()

    arm_velocity_node = ArmVelocityPublisher()
    ros2_manager.add_node(arm_velocity_node)

    arm_ee_node = ArmEEPublisher()
    ros2_manager.add_node(arm_ee_node)

    arm_mode_node = ArmModePublisher()
    ros2_manager.add_node(arm_mode_node)

    arm_feedback_node = ArmFeedbackSubscriber()
    ros2_manager.add_node(arm_feedback_node)

    print("Arm ROS2 nodes initialized")


@app.on_event("shutdown")
async def shutdown_event():
    ros2_manager.shutdown()


# ─── Arm Control Endpoints ──────────────────────────────────────────────────────

@app.post("/arm/velocity")
async def arm_velocity(cmd: ArmVelocityCommand):
    if arm_velocity_node is None:
        raise HTTPException(status_code=503, detail="ROS2 not initialized")
    try:
        arm_velocity_node.publish_velocity(cmd.joint_velocities)
        return {"status": "velocity sent"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/arm/ee")
async def arm_ee(cmd: ArmEECommand):
    if arm_ee_node is None or arm_velocity_node is None:
        raise HTTPException(status_code=503, detail="ROS2 not initialized")
    try:
        arm_ee_node.publish_ee(cmd.linear_y, cmd.linear_z, cmd.angular_x)
        arm_velocity_node.publish_velocity([cmd.j1_velocity, 0, 0, 0, cmd.j5_velocity, cmd.j6_velocity])
        return {"status": "ee command sent"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/arm/mode")
async def arm_mode(cmd: ArmModeCommand):
    if arm_mode_node is None or arm_ee_node is None:
        raise HTTPException(status_code=503, detail="ROS2 not initialized")
    try:
        is_joint = cmd.mode == "joint"
        arm_mode_node.publish_mode(is_joint)
        if is_joint:
            arm_ee_node.publish_zero()
        return {"status": f"mode set to {cmd.mode}"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/arm/feedback")
async def arm_feedback():
    if arm_feedback_node is None:
        raise HTTPException(status_code=503, detail="ROS2 not initialized")
    try:
        feedback = arm_feedback_node.latest_feedback
        if not feedback or "joint_positions" not in feedback:
            return {"joints": []}

        positions = feedback["joint_positions"]
        velocities = feedback.get("joint_velocities", [0.0] * len(positions))
        names = feedback.get("joint_names", [f"J{i+1}" for i in range(len(positions))])

        joints = [
            {"name": names[i], "position": positions[i], "velocity": velocities[i]}
            for i in range(len(positions))
        ]
        return {"joints": joints}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

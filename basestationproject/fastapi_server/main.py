from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, field_validator
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
import threading
from typing import List, Optional

print("FastAPI server starting...")

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ─── Pydantic Models ────────────────────────────────────────────────────────────

class TwistVector(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class TwistCommand(BaseModel):
    linear: TwistVector = Field(default_factory=TwistVector)
    angular: TwistVector = Field(default_factory=TwistVector)

    @field_validator('linear', 'angular', mode='before')
    @classmethod
    def parse_from_flat(cls, v, info):
        if v is not None:
            return v
        return TwistVector()


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
        self.executor = MultiThreadedExecutor(num_threads=2)
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


# ─── Rover Cmd Vel Publisher ────────────────────────────────────────────────────

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('fastapi_cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 100)
        self.get_logger().info("CmdVelPublisher ready on /cmd_vel")

    def publish_twist(self, command: TwistCommand):
        msg = Twist()
        msg.linear.x = command.linear.x
        msg.linear.y = command.linear.y
        msg.linear.z = command.linear.z
        msg.angular.x = command.angular.x
        msg.angular.y = command.angular.y
        msg.angular.z = command.angular.z
        self.publisher.publish(msg)


# ─── Global Node Instance ───────────────────────────────────────────────────────

cmd_vel_node: Optional[CmdVelPublisher] = None


@app.on_event("startup")
async def startup_event():
    global cmd_vel_node

    ros2_manager.init()

    cmd_vel_node = CmdVelPublisher()
    ros2_manager.add_node(cmd_vel_node)

    print("ROS2 cmd_vel node initialized")


@app.on_event("shutdown")
async def shutdown_event():
    ros2_manager.shutdown()


# ─── Rover Command Endpoint ─────────────────────────────────────────────────────

@app.post("/command")
async def handle_command(command: TwistCommand):
    if cmd_vel_node is None:
        raise HTTPException(status_code=503, detail="ROS2 not initialized")
    try:
        cmd_vel_node.publish_twist(command)
        return {"status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

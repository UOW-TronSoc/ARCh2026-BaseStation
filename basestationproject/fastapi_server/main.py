from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, root_validator
import rclpy
from rclpy.node import Node
# Standard ROS2 message; custom msgs come from ARCH2026-Kanga/src/kanga_interfaces (no kanga_interfaces equivalent for cmd_vel)
from geometry_msgs.msg import Twist

print("starting?")

# Initialize the FastAPI app
app = FastAPI()

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic model
class TwistVector(BaseModel):
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class TwistCommand(BaseModel):
    linear: TwistVector = Field(default_factory=TwistVector)
    angular: TwistVector = Field(default_factory=TwistVector)

    @root_validator(pre=True)
    def allow_flat_payload(cls, values):
        """Support both nested and flat payload formats from the frontend."""
        raw = dict(values)

        if 'linear' not in raw:
            linear_axes = {axis: raw.get(f'linear_{axis}') for axis in ('x', 'y', 'z')}
            if any(v is not None for v in linear_axes.values()):
                raw['linear'] = {
                    axis: float(linear_axes[axis]) if linear_axes[axis] is not None else 0.0
                    for axis in ('x', 'y', 'z')
                }

        if 'angular' not in raw:
            angular_axes = {axis: raw.get(f'angular_{axis}') for axis in ('x', 'y', 'z')}
            if any(v is not None for v in angular_axes.values()):
                raw['angular'] = {
                    axis: float(angular_axes[axis]) if angular_axes[axis] is not None else 0.0
                    for axis in ('x', 'y', 'z')
                }

        return raw

# ROS2 client using Twist publisher
CMD_VELOCITY_TOPIC = '/cmd_vel'


class ROS2Client:
    def __init__(self):
        try:
            rclpy.init()
            self.node = rclpy.create_node('fastapi_ros2_client')
            self.publisher = self.node.create_publisher(Twist, CMD_VELOCITY_TOPIC, 10)
            self.node.get_logger().info(
                f"ROS2Client initialized. Publishing Twist messages on {CMD_VELOCITY_TOPIC}"
            )
        except Exception as e:
            print(f"Error initializing ROS2Client: {e}")
            raise

    def publish_twist(self, command: TwistCommand):
        try:
            msg = Twist()
            msg.linear.x = command.linear.x
            msg.linear.y = command.linear.y
            msg.linear.z = command.linear.z
            msg.angular.x = command.angular.x
            msg.angular.y = command.angular.y
            msg.angular.z = command.angular.z
            self.publisher.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"Failed to publish Twist command: {e}")
            raise

# Initialize ROS2 client
ros2_client = ROS2Client()

@app.post("/command")
async def handle_command(command: TwistCommand):
    try:
        ros2_client.publish_twist(command)
        return {
            "status": "success",
            "message": "Twist command sent successfully",
        }
    except Exception as e:
        print(f"Error handling /command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from custom_msgs.msg import CoreControl  # 🔁 updated import
import time

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
class DriveCommand(BaseModel):
    left_drive: int
    right_drive: int

# ROS2 Client using CoreControl
class ROS2Client:
    def __init__(self):
        try:
            rclpy.init()
            self.node = rclpy.create_node('fastapi_ros2_client')
            self.publisher = self.node.create_publisher(CoreControl, 'core_control', 10)
            self.node.get_logger().info("ROS2Client initialized with CoreControl!")
        except Exception as e:
            print(f"Error initializing ROS2Client: {e}")
            raise

    def publish_drive_command(self, left_drive: int, right_drive: int):
        try:
            msg = CoreControl()
            msg.epoch_time = int(time.time() * 1000)

            if not (-128 <= left_drive <= 127):
                raise ValueError(f"left_drive value {left_drive} is out of int8 range")
            if not (-128 <= right_drive <= 127):
                raise ValueError(f"right_drive value {right_drive} is out of int8 range")

            # Assign same values to both sides (front & back) for now
            msg.lf_drive = left_drive
            msg.lb_drive = left_drive
            msg.rf_drive = right_drive
            msg.rb_drive = right_drive

            self.publisher.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"Failed to publish CoreControl: {e}")
            raise

# Initialize ROS2 client
ros2_client = ROS2Client()

@app.post("/command")
async def handle_command(command: DriveCommand):
    try:
        if not (-100 <= command.left_drive <= 100) or not (-100 <= command.right_drive <= 100):
            raise HTTPException(status_code=400, detail="Drive values must be between -100 and 100")

        ros2_client.publish_drive_command(command.left_drive, command.right_drive)
        return {"status": "success", "message": "CoreControl command sent successfully"}
    except Exception as e:
        print(f"Error handling /command: {e}")
        raise HTTPException(status_code=500, detail=str(e))

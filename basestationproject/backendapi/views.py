# Django Imports
from django.http import JsonResponse, StreamingHttpResponse, HttpResponse
from django.views.decorators.csrf import csrf_exempt
from django.shortcuts import render
from django.core.cache import cache

# Django Rest Framework Imports
from rest_framework.response import Response
from rest_framework.views import APIView
from rest_framework.decorators import api_view
from rest_framework import status

# ROS 2 and Message Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import logging

try:
    from sensor_msgs.msg import Image, CompressedImage, JointState
    from custom_msgs.msg import DrivetrainFeedback, ScienceFeedback, ScienceControl, RadioFeedback, BatteryFeedback, CoreFeedback, ArmIncrementCommand
    from std_msgs.msg import String, Bool, Empty

    ROS_IMPORTS_AVAILABLE = True
    
except ImportError as e:
    ROS_IMPORTS_AVAILABLE = False


    logging.warning(f"ROS message imports failed: {e}. Some features may not work.")



# Third-party Imports
import cv2
import numpy as np
import httpx
import json
import time
import threading
from asgiref.sync import async_to_sync

# Custom Imports
from . import models


fps = 15


class ROS2Manager:
    # Singleton ROS2 Manager to handle initialization and node management
    _instance = None
    _lock = threading.Lock()  # Ensure thread-safe singleton initialization

    def __init__(self):
        if ROS2Manager._instance is not None:
            raise Exception("This is a singleton class. Use get_instance() instead.")

        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()

        self.executor = MultiThreadedExecutor(context=rclpy.get_default_context())  # Ensure proper context
        self.nodes = {}  # Stores all nodes

        # Start executor thread
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    @classmethod
    def get_instance(cls):
        # Thread-safe singleton instance creation
        with cls._lock:
            if cls._instance is None:
                cls._instance = ROS2Manager()
        return cls._instance

    def add_node(self, node):
        # Add a new node to the executor and manage it
        self.nodes[node.get_name()] = node
        self.executor.add_node(node)


# Initialize Singleton ROS2 Manager
ros_manager = ROS2Manager.get_instance()


# ----------------------------

# Camera Feedback streams

# ----------------------------

class MultiCameraSubscriber(Node):
    # Subscriber to multiple camera topics dynamically
    def __init__(self, camera_id):
        super().__init__(f'camera_stream_subscriber_{camera_id}')
        self.camera_id = camera_id
        self.subscription = self.create_subscription(
            CompressedImage, f'/camera_{camera_id}/image_compressed', self.image_callback, fps)
        self.current_frame = None
        self.lock = threading.Lock()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        """frame = np_arr.reshape((msg.height, msg.width, 3))
        with self.lock:
            self.current_frame = frame"""
        if frame is not None:
            with self.lock:
                self.current_frame = frame
    """def image_callback(self, msg):
        # Process received image frames.
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = np_arr.reshape((msg.height, msg.width, 3))
        with self.lock:
            self.current_frame = frame"""

no_of_cams = 5  # Cameras 0 to 4
def initialize_cameras():
    # Initialize all camera subscribers dynamically
    camera_nodes = {}
    for cam_id in range(no_of_cams):
        camera_node = MultiCameraSubscriber(cam_id)
        ros_manager.add_node(camera_node)
        camera_nodes[cam_id] = camera_node
    logging.info("✅ All camera nodes initialized!")
    return camera_nodes


camera_nodes = initialize_cameras()
    

def get_frame(request, camera_id):
    # Serve a single frame from a specific camera
    camera_id = int(camera_id)
    if camera_id in camera_nodes and camera_nodes[camera_id].current_frame is not None:
        with camera_nodes[camera_id].lock:
            success, jpeg = cv2.imencode('.jpg', camera_nodes[camera_id].current_frame)
            if not success:
                return HttpResponse(status=500)
        return HttpResponse(jpeg.tobytes(), content_type="image/jpeg")
    return HttpResponse(status=204)  # No content if no frame is available


# ----------------------------

# Drivetrain Feedback(only)

# ----------------------------

class DrivetrainFeedbackSubscriber(Node):
    # Subscriber to Drivetrain Feedback topic
    def __init__(self):
        super().__init__('drivetrain_feedback_subscriber')
        try:
            self.subscription = self.create_subscription(
                DrivetrainFeedback, '/drivetrain_feedback', self.feedback_callback, 10
            )
            self.latest_feedback = {}
        except Exception as e:
            logging.error(f"Error initializing drivetrain subscriber: {e}")

    def feedback_callback(self, msg):
        # Process drivetrain feedback messages
        try:
            self.latest_feedback = {
                "epoch_time": msg.epoch_time,
                "wheel_position": msg.wheel_position,
                "wheel_velocity": msg.wheel_velocity,
                "wheel_torque": msg.wheel_torque,
            }
        except Exception as e:
            logging.error(f"Error processing drivetrain feedback: {e}")



def get_drivetrain_feedback(request):
    # Retrieve the latest drivetrain feedback data
    global feedback_node  # Ensure we use the initialized feedback subscriber

    try:
        if feedback_node.latest_feedback:
            # Convert NumPy arrays to Python lists
            formatted_feedback = {
                "epoch_time": feedback_node.latest_feedback["epoch_time"],
                "wheel_position": feedback_node.latest_feedback["wheel_position"].tolist()
                if isinstance(feedback_node.latest_feedback["wheel_position"], np.ndarray)
                else feedback_node.latest_feedback["wheel_position"],
                "wheel_velocity": feedback_node.latest_feedback["wheel_velocity"].tolist()
                if isinstance(feedback_node.latest_feedback["wheel_velocity"], np.ndarray)
                else feedback_node.latest_feedback["wheel_velocity"],
                "wheel_torque": feedback_node.latest_feedback["wheel_torque"].tolist()
                if isinstance(feedback_node.latest_feedback["wheel_torque"], np.ndarray)
                else feedback_node.latest_feedback["wheel_torque"],
            }

            return JsonResponse(formatted_feedback)
        else:
            return JsonResponse({"error": "No feedback available"}, status=204)
    except Exception as e:
        logging.error(f"Error retrieving drivetrain feedback: {e}")
        return JsonResponse({"error": "Failed to retrieve feedback"}, status=500)


# Initialize drivetrain subscriber
feedback_node = DrivetrainFeedbackSubscriber()
ros_manager.add_node(feedback_node)



# ----------------------------

# Core Feedback

# ----------------------------

class CoreFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('core_feedback_subscriber')
        try:
            self.subscription = self.create_subscription(
                CoreFeedback, '/core_feedback', self.feedback_callback, 10
            )
            self.latest_feedback = {}
        except Exception as e:
            logging.error(f"Error initializing core feedback subscriber: {e}")

    def feedback_callback(self, msg):
        try:
            self.latest_feedback = {
                "epoch_time": int(msg.epoch_time),

                "wheel_position": [float(x) for x in msg.wheel_position],
                "wheel_velocity": [float(x) for x in msg.wheel_velocity],
                "wheel_torque": [float(x) for x in msg.wheel_torque],

                "pitch": round(float(msg.pitch), 2),
                "roll": round(float(msg.roll), 2),
            }
        except Exception as e:
            logging.error(f"Error processing core feedback: {e}")


core_feedback_node = CoreFeedbackSubscriber()
ros_manager.add_node(core_feedback_node)


def get_core_feedback(request):
    try:
        if core_feedback_node.latest_feedback:
            return JsonResponse(core_feedback_node.latest_feedback)
        return JsonResponse({"error": "No feedback available"}, status=204)
    except Exception as e:
        logging.error(f"Error retrieving core feedback: {e}")
        return JsonResponse({"error": f"Failed to retrieve feedback: {e}"}, status=500)


# ----------------------------

# Science Control

# ----------------------------

class ScienceFeedbackSubscriber(Node):
    # Subscriber to Science Feedback topic
    def __init__(self):
        super().__init__('science_feedback_subscriber')
        self.subscription = self.create_subscription(
            ScienceFeedback, '/science_feedback', self.feedback_callback, 10
        )
        self.latest_feedback = {}

    def feedback_callback(self, msg):
        self.latest_feedback = {
            "rfid": msg.rfid,
            "moisture": msg.moisture,
            "potentiometer": msg.potentiometer,
            "limit": msg.limit,
            "height": msg.height,
        }

# Science Control Publisher
class ScienceControlPublisher(Node):
    # Publisher to Science Control topic
    def __init__(self):
        super().__init__('science_control_publisher')
        self.publisher = self.create_publisher(ScienceControl, '/science_control', 10)

    def publish_control(self, data):
        msg = ScienceControl()
        msg.linear_actuator = data.get("linear_actuator", 0)
        msg.req_height = data.get("req_height", False)
        msg.req_nir = data.get("req_nir", False)
        self.publisher.publish(msg)


# Initialize Science Feedback Subscriber
science_feedback_node = ScienceFeedbackSubscriber()
ros_manager.add_node(science_feedback_node)

# Initialize Science Control Publisher
science_control_node = ScienceControlPublisher()
ros_manager.add_node(science_control_node)


def get_science_feedback(request):
    # Retrieve the latest science feedback data
    if science_feedback_node.latest_feedback:
        return JsonResponse(science_feedback_node.latest_feedback)
    return JsonResponse({"error": "No science feedback available"}, status=204)


@csrf_exempt
def set_science_control(request):
    # Set science control settings via a ROS2 publisher
    if request.method == "POST":
        try:
            data = json.loads(request.body)
            science_control_node.publish_control(data)
            return JsonResponse({"status": "success", "message": "Science control command sent!"})
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON received: {e}")
            return JsonResponse({"error": "Invalid JSON"}, status=400)
        except Exception as e:
            logging.error(f"Unexpected error processing science control: {e}")
            return JsonResponse({"error": "Internal server error"}, status=500)
    
    return JsonResponse({"error": "Invalid request method"}, status=405)


# ----------------------------

# Log Pub-sub

# ----------------------------

class RoverLogsSubscriber(Node):
    # Subscriber for the Rover_Logs topic
    def __init__(self):
        super().__init__('rover_logs_subscriber')
        self.subscription = self.create_subscription(
            String, 'rover_logs', self.log_callback, 10
        )
        self.latest_logs = []

    def log_callback(self, msg):
        # Store received log messages
        log_message = msg.data
        self.latest_logs.append(log_message)
        self.get_logger().info(f"Received log: {log_message}")

# Initialize the log subscriber and add to ROS2 Manager
rover_logs_node = RoverLogsSubscriber()
ros_manager.add_node(rover_logs_node)

# Django API Endpoint to Fetch Logs
def get_rover_logs(request):
    # Retrieve logs from the Rover_Logs topic
    try:
        if rover_logs_node.latest_logs:
            return JsonResponse({"logs": rover_logs_node.latest_logs})
        return JsonResponse({"logs": [], "message": "No logs received yet."}, status=204)
    except Exception as e:
        logging.error(f"Error retrieving rover logs: {e}")
        return JsonResponse({"error": "Failed to retrieve logs"}, status=500)



# ----------------------------

# ROS2 Arm Feedback Subscriber

# ----------------------------

ARM_COMMAND_TOPIC = "arm_command"
# GRIPPER_COMMAND_TOPIC = "gripper_command"
# ARM_FEEDBACK_TOPIC = "armgripper"
# ARM_FEEDBACK_TOPIC = "arm_feedback" # dont need 


class ArmFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('arm_feedback_subscriber')
        try:
            self.subscription = self.create_subscription(
                JointState, ARM_COMMAND_TOPIC, self.feedback_callback, 10
            )
            self.latest_feedback = {}
        except Exception as e:
            logging.error(f"Error initializing arm feedback subscriber: {e}")

    def feedback_callback(self, msg):
      try:
          positions = list(msg.position)
          velocities = list(msg.velocity)
          names = list(msg.name)

          while len(positions) < 6:
              positions.append(0.0)
          while len(velocities) < 6:
              velocities.append(0.0)
          while len(names) < 6:
              names.append(f"θ{len(names)+1}")

          self.latest_feedback["joint_positions"] = positions
          self.latest_feedback["joint_velocities"] = velocities
          self.latest_feedback["joint_names"] = names

      except Exception as e:
          logging.error(f"Error processing arm feedback: {e}")




# --- Arm Command Publisher ---
class ArmCommandPublisher(Node):
    def __init__(self):
        super().__init__('arm_command_publisher')
        try:
            self.publisher = self.create_publisher(JointState, ARM_COMMAND_TOPIC, 10)
            # self.gripper_publisher = self.create_publisher(Bool, GRIPPER_COMMAND_TOPIC, 10)
        except Exception as e:
            logging.error(f"Error initializing arm command publisher: {e}")

    def publish_arm_command(self, joint_positions):
        try:
            float_positions = [float(p) if isinstance(p, (int, float)) else 0.0 for p in joint_positions]

            arm_msg = JointState()
            arm_msg.position = float_positions
            arm_msg.name = [f"joint_{i}" for i in range(len(float_positions))]
            arm_msg.header.stamp = self.get_clock().now().to_msg()

            self.publisher.publish(arm_msg)
            self.get_logger().info(f"Published to /arm_command: {float_positions}")

        except Exception as e:
            logging.error(f"Error publishing arm command: {e}")


# --- Initialize and Register Nodes ---
arm_feedback_node = ArmFeedbackSubscriber()
arm_command_node = ArmCommandPublisher()

ros_manager.add_node(arm_feedback_node)
ros_manager.add_node(arm_command_node)

# --- API: Get Arm Feedback ---
def get_arm_feedback(request):
    try:
        feedback = arm_feedback_node.latest_feedback

        if "joint_positions" not in feedback:
            return JsonResponse({"joints": []}, status=204)

        positions = feedback["joint_positions"]
        names = feedback.get("joint_names", [f"θ{i+1}" for i in range(len(positions))])
        velocities = feedback.get("joint_velocities", [0.0] * len(positions))

        joints = [
            {"name": names[i], "position": positions[i], "velocity": velocities[i]}
            for i in range(len(positions))
        ]

        return JsonResponse({"joints": joints})

    except Exception as e:
        logging.error(f"Error retrieving arm feedback: {e}")
        return JsonResponse({"error": "Failed to retrieve feedback"}, status=500)

# --- API: Send Arm Command ---
@csrf_exempt
def send_arm_command(request):
    if request.method == "POST":
        try:
            data = json.loads(request.body)
            joint_positions = data.get("joint_positions", [])

            if len(joint_positions) != 6:
                return JsonResponse({"error": "Expected 6 joint positions (including gripper)"}, status=400)

            arm_command_node.publish_arm_command(joint_positions)
            return JsonResponse({"message": "Command sent successfully!"})
        except json.JSONDecodeError:
            logging.error("Invalid JSON received.")
            return JsonResponse({"error": "Invalid JSON"}, status=400)
        except Exception as e:
            logging.error(f"Unexpected error processing arm command: {e}")
            return JsonResponse({"error": "Internal server error"}, status=500)

    return JsonResponse({"error": "Invalid request method"}, status=405)


class ArmPresetPublisher(Node):
    def __init__(self):
        super().__init__('arm_preset_publisher')
        self.publisher = self.create_publisher(String, "/arm/preset_command", 10)

    def publish_preset(self, preset):
        self.publisher.publish(String(data=preset))

@csrf_exempt
def arm_preset_command_view(request, preset):
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)
    arm_preset_node.publish_preset(preset)
    return JsonResponse({"status": "ok", "preset": preset})


# =======================
class ArmPitchLockPublisher(Node):
    def __init__(self):
        super().__init__('arm_pitch_lock_publisher')
        self.publisher = self.create_publisher(Bool, "/arm/lock_pitch", 10)

    def publish_lock(self, locked):
        self.publisher.publish(Bool(data=locked))

@csrf_exempt
def lock_pitch_view(request):
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)

    try:
        data = json.loads(request.body.decode())
        locked = data.get("locked", False)
    except Exception:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    arm_pitch_lock_node.publish_lock(locked)
    return JsonResponse({"status": "ok", "locked": locked})


# =======================


@csrf_exempt
def horizontal_pitch_view(request):
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)

    try:
        data = json.loads(request.body.decode())
        names = data.get("name", [])
        positions = data.get("position", [])
    except Exception as e:
        logging.error("JSON decode failed:", str(e))
        logging.error("Raw body:", request.body.decode(errors="replace"))
        return JsonResponse({"error": "Invalid JSON", "details": str(e)}, status=400)

    if not names or not positions or len(names) != len(positions):
        return JsonResponse({"error": "Mismatched name/position arrays"}, status=400)

    msg = JointState()
    msg.name = names
    msg.position = [float(p) for p in positions]
    msg.velocity = []
    msg.effort = []

    arm_command_node.publisher.publish(msg)
    return JsonResponse({"status": "ok", "published": {"name": names, "position": list(msg.position)}})





# =======================
class ArmIncrementCommandPublisher(Node):
    def __init__(self):
        super().__init__('arm_increment_publisher')
        self.publisher = self.create_publisher(ArmIncrementCommand, "/arm/increment_command", 10)

    def publish_increment(self, mode, target, value):
        msg = ArmIncrementCommand()
        msg.mode = mode
        msg.target = target
        msg.value = value
        self.publisher.publish(msg)

@csrf_exempt
def arm_increment_command_view(request):
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)

    try:
        data = json.loads(request.body.decode())
        mode = data.get("mode")
        target = data.get("target")
        value = float(data.get("value"))
    except Exception as e:
        return JsonResponse({"error": "Invalid payload", "details": str(e)}, status=400)

    arm_increment_node.publish_increment(mode, target, value)

    return JsonResponse({
        "status": "ok",
        "sent": {"mode": mode, "target": target, "value": value}
    })


arm_preset_node = ArmPresetPublisher()
arm_pitch_lock_node = ArmPitchLockPublisher()
arm_increment_node = ArmIncrementCommandPublisher()

ros_manager.add_node(arm_preset_node)
ros_manager.add_node(arm_pitch_lock_node)
ros_manager.add_node(arm_increment_node)


# ----------------------------

# Radio Feedback & Sub

# ----------------------------

class RadioFeedbackSubscriber(Node):
    # Subscriber to /radio_feedback topic
    def __init__(self):
        super().__init__('radio_feedback_subscriber')
        try:
            self.subscription = self.create_subscription(
                RadioFeedback, '/radio_feedback', self.feedback_callback, 10
            )
            self.latest_feedback = {}
        except Exception as e:
            logging.error(f"Error initializing radio feedback subscriber: {e}")

    def feedback_callback(self, msg):
        try:
            self.latest_feedback = {
                "connection": "Connected",
                "strength": f"{msg.signal_strength:.1f} dBm",
                "ping": msg.ping_ms,
                "received": msg.rx_bytes,
                "sent": msg.tx_bytes
            }
        except Exception as e:
            logging.error(f"Error processing radio feedback: {e}")


radio_feedback_node = RadioFeedbackSubscriber()
ros_manager.add_node(radio_feedback_node)

def get_radio_feedback(request):
    try:
        if radio_feedback_node.latest_feedback:
            return JsonResponse(radio_feedback_node.latest_feedback)
        return JsonResponse({"error": "No radio feedback available"}, status=204)
    except Exception as e:
        logging.error(f"Error retrieving radio feedback: {e}")
        return JsonResponse({"error": "Failed to retrieve radio feedback"}, status=500)



# ----------------------------

# Battery Feedback & Sub

# ----------------------------
class BatteryFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('battery_feedback_subscriber')
        try:
            self.subscription = self.create_subscription(
                BatteryFeedback,
                '/battery_feedback',
                self.feedback_callback,
                10
            )
            self.latest_feedback = {}
        except Exception as e:
            logging.error(f"Error initializing battery feedback subscriber: {e}")

    def feedback_callback(self, msg):
        try:
            self.latest_feedback = {
                "charge_pct": round(msg.charge_pct, 2),
                "current_draw": round(msg.current_draw, 2),
                "temperature": round(msg.temperature, 2),
                "timestamp": msg.timestamp,
            }
        except Exception as e:
            logging.error(f"Error processing battery feedback: {e}")


battery_feedback_node = BatteryFeedbackSubscriber()
ros_manager.add_node(battery_feedback_node)


def get_battery_feedback(request):
    try:
        if battery_feedback_node.latest_feedback:
            return JsonResponse(battery_feedback_node.latest_feedback)
        return JsonResponse({"error": "No battery data available"}, status=204)
    except Exception as e:
        logging.error(f"Error retrieving battery data: {e}")
        return JsonResponse({"error": "Failed to retrieve battery data"}, status=500)

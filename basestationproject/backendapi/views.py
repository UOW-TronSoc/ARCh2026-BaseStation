# Django Imports
from django.http import JsonResponse, StreamingHttpResponse, HttpResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_GET
from django.views.decorators import gzip
from django.shortcuts import render
from django.core.cache import cache
from django.conf import settings

# Django Rest Framework Imports
from rest_framework.response import Response
from rest_framework.views import APIView
from rest_framework.decorators import api_view
from rest_framework import viewsets

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
import os
import numpy as np
import httpx
import json
import time
import threading
from asgiref.sync import async_to_sync

# Custom Imports
from .models import *
from .serialisers import ChecklistGroupSerializer, ChecklistTaskSerializer


class ChecklistGroupViewSet(viewsets.ReadOnlyModelViewSet):
    queryset = ChecklistGroup.objects.all()
    serializer_class = ChecklistGroupSerializer

class ChecklistTaskViewSet(viewsets.ModelViewSet):
    queryset = ChecklistTask.objects.all()
    serializer_class = ChecklistTaskSerializer


def get_checklist_group(request, group_id):
    group = ChecklistGroup.objects.get(id=group_id)
    tasks = group.tasks.all().values("id", "text", "completed", "value")
    return JsonResponse({"group": group.name, "tasks": list(tasks)})


@csrf_exempt
def update_checklist_task(request, task_id):
    if request.method == "POST":
        data = json.loads(request.body)
        task = ChecklistTask.objects.get(id=task_id)
        task.completed = data.get("completed", task.completed)
        task.value = data.get("value", task.value)
        task.save()
        return JsonResponse({"status": "success"})


class ROS2Manager:
    """
    Singleton manager responsible for:
      1. Initializing the ROS2 client library (rclpy).
      2. Holding a MultiThreadedExecutor to spin all registered nodes.
      3. Exposing a thread-safe method to add new nodes at runtime.
    """

    # Holds the sole instance of ROS2Manager
    _instance = None

    # Lock to prevent race conditions when creating the singleton
    _lock = threading.Lock()

    def __init__(self):
        """
        PRIVATE: Only called once under get_instance().
        - Checks no other instance exists (enforces singleton).
        - Initializes rclpy if it hasn’t been already.
        - Creates and starts a background executor thread.
        """
        # Prevent direct instantiation if instance already exists
        if ROS2Manager._instance is not None:
            raise Exception("This is a singleton class. Use get_instance() instead.")

        # Initialize the ROS2 Python client library, if needed
        if not rclpy.ok():
            rclpy.init()

        # Create a multithreaded executor tied to the default context
        self.executor = MultiThreadedExecutor(context=rclpy.get_default_context())
        # Dictionary to map node names to node instances
        self.nodes = {}

        # Launch the executor’s spin loop on a daemon thread
        # so it lives for the lifetime of the program
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True
        )
        self.executor_thread.start()

    @classmethod
    def get_instance(cls):
        """
        Returns the single ROS2Manager instance, creating it if needed.
        Uses a lock to ensure thread-safe, one-time construction.
        """
        with cls._lock:
            if cls._instance is None:
                cls._instance = ROS2Manager()
        return cls._instance

    def add_node(self, node):
        """
        Register a ROS2 node with this manager:
          1. Store it in the internal dict by its name.
          2. Add it to the executor so its callbacks fire.

        Args:
            node (rclpy.node.Node): an instantiated ROS2 node.
        """
        # Save for bookkeeping and potential future teardown
        self.nodes[node.get_name()] = node
        # Hook it into the executor loop
        self.executor.add_node(node)


# On import, immediately instantiate the singleton manager
ros_manager = ROS2Manager.get_instance()



# ----------------------------

# Camera Feedback streams

# ----------------------------

class MultiCameraSubscriber(Node):
    """
    A ROS2 node that subscribes to a compressed image topic for one camera ID,
    decodes incoming JPEG payloads into OpenCV frames, and stores the latest
    frame safely behind a thread lock.
    """

    def __init__(self, camera_id):
        """
        Initialize the subscriber node.
        
        Args:
            camera_id (int): Numeric identifier for the camera. Used both to
                             name the node and to form the subscription topic.
        """
        # Name this node uniquely based on camera_id
        super().__init__(f'camera_stream_subscriber_{camera_id}')
        
        # Store camera ID and prepare storage for the latest frame
        self.camera_id = camera_id
        self.current_frame = None
        
        # Lock to synchronize access to current_frame across threads
        self.lock = threading.Lock()
        
        # Subscribe to the ROS2 topic publishing CompressedImage for this camera
        #   - Topic name: /camera_<camera_id>/image_compressed
        #   - QoS depth: 10
        self.create_subscription(
            CompressedImage,
            f'/camera_{camera_id}/image_compressed',
            self.image_callback,
            10
        )

    def image_callback(self, msg: CompressedImage):
        """
        Callback executed whenever a new CompressedImage message arrives.
        Decodes the raw JPEG bytes into an OpenCV BGR image and updates
        self.current_frame under thread lock.
        
        Args:
            msg (CompressedImage): ROS2 message containing JPEG-compressed data.
        """
        # Convert ROS2 message byte buffer into a NumPy array
        arr = np.frombuffer(msg.data, np.uint8)
        
        # Decode JPEG bytes to OpenCV BGR image
        frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        
        # If decoding succeeded, update the shared frame safely
        if frame is not None:
            with self.lock:
                self.current_frame = frame


# -----------------------------------------------------------------------------
# Initialize and Register Camera Subscriber Nodes
# -----------------------------------------------------------------------------
no_of_cams = 5  # Total number of cameras we expect (IDs 0 through 4)

def initialize_cameras():
    """
    Dynamically create and register MultiCameraSubscriber nodes for each camera ID.
    
    Returns:
        dict: Mapping from camera_id to its subscriber node instance.
    """
    camera_nodes = {}
    for cam_id in range(no_of_cams):
        # Instantiate subscriber and add it to the global ROS2 manager
        camera_node = MultiCameraSubscriber(cam_id)
        ros_manager.add_node(camera_node)
        camera_nodes[cam_id] = camera_node
    
    logging.info("✅ All camera nodes initialized!")
    return camera_nodes

# Create subscribers at module load
camera_nodes = initialize_cameras()


# -----------------------------------------------------------------------------
# Single-Frame HTTP View
# -----------------------------------------------------------------------------
def get_frame(request, camera_id):
    """
    HTTP endpoint that returns the most recent frame from one camera as a
    single JPEG image. If no frame is yet available, returns 204 No Content.
    
    Args:
        request (HttpRequest): Django request object.
        camera_id (str|int): ID of the camera to fetch (parsed to int).
    
    Returns:
        HttpResponse: JPEG image or 204/500 status.
    """
    camera_id = int(camera_id)
    
    # Check that we have a subscriber and that it has at least one frame
    if camera_id in camera_nodes and camera_nodes[camera_id].current_frame is not None:
        # Encode the stored OpenCV frame back to JPEG
        with camera_nodes[camera_id].lock:
            success, jpeg = cv2.imencode('.jpg', camera_nodes[camera_id].current_frame)
        if not success:
            # Encoding failed; internal error
            return HttpResponse(status=500)
        
        # Serve the raw JPEG bytes with correct MIME type
        return HttpResponse(jpeg.tobytes(), content_type="image/jpeg")
    
    # No frame available yet
    return HttpResponse(status=204)


# -----------------------------------------------------------------------------
# Continuous MJPEG Streaming View
# -----------------------------------------------------------------------------
@gzip.gzip_page  # Optional gzip compression for the multipart stream
def mjpeg_stream(request, camera_id):
    """
    HTTP endpoint that streams an MJPEG (multipart/x-mixed-replace) response
    for a given camera, pushing new frames as they arrive.
    
    Args:
        request (HttpRequest): Django request object.
        camera_id (str|int): ID of the camera to stream (parsed to int).
    
    Returns:
        StreamingHttpResponse: Keeps connection open, sending frames until
        client disconnects.
    """
    camera_id = int(camera_id)
    if camera_id not in camera_nodes:
        # Unknown camera ID
        return HttpResponse(status=404)

    boundary = "--frame"
    
    # Create a streaming response that yields JPEG frames in a loop
    response = StreamingHttpResponse(
        _frame_generator(camera_nodes[camera_id], boundary),
        content_type=f'multipart/x-mixed-replace; boundary={boundary}'
    )
    
    # Prevent caching and ensure the client/proxy closes the socket on exit
    response['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response['Pragma']        = 'no-cache'
    response['Expires']       = '0'
    response['Connection']    = 'close'
    
    return response


def _frame_generator(camera_node, boundary):
    """
    MJPEG generator that yields *every* new frame immediately.
    """
    last_ts = 0.0
    try:
        while True:
            # grab the frame under lock
            with camera_node.lock:
                frame = camera_node.current_frame

            if frame is None:
                # nothing yet—briefly wait
                time.sleep(0.01)
                continue

            # we can encode and yield immediately
            success, jpeg = cv2.imencode('.jpg', frame)
            if not success:
                continue

            # yield boundary + JPEG
            yield (
                f"{boundary}\r\n"
                "Content-Type: image/jpeg\r\n\r\n"
            ).encode('utf-8') + jpeg.tobytes() + b"\r\n"

            # reset last_ts so we don't wait for frame-rate
            last_ts = time.time()

            # and loop right back to check for the next new frame
    except (BrokenPipeError, ConnectionResetError, GeneratorExit):
        # client closed—exit quietly
        return





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


from django.http import JsonResponse

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


# ─── ArmCommandPublisher Node ────────────────────────────────────────────────────
class ArmCommandPublisher(Node):
    def __init__(self):
        super().__init__('arm_command_publisher')
        # Publish JointState messages on /arm_command
        self.publisher = self.create_publisher(JointState, ARM_COMMAND_TOPIC, 10)
        self.get_logger().info("ArmCommandPublisher initialized, publishing to /arm_command")

    def publish_arm_command(self, joint_positions):
        """
        joint_positions: list of 6 floats
        Publishes a sensor_msgs/JointState on /arm_command.
        """
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # Name joints "joint_0" ... "joint_5"
            msg.name = [f"joint_{i}" for i in range(len(joint_positions))]
            msg.position = [float(x) for x in joint_positions]
            # velocities/effort left empty
            self.publisher.publish(msg)
            self.get_logger().info(f"Published JointState: {msg.position}")
        except Exception as e:
            self.get_logger().error(f"Error publishing arm command: {e}")


# ─── ArmFeedbackSubscriber Node ─────────────────────────────────────────────────
class ArmFeedbackSubscriber(Node):
    def __init__(self):
        super().__init__('arm_feedback_subscriber')
        # Subscribe to the real robot/simulator topic for joint states.
        # If your simulator or robot publishes to '/joint_states', keep this.
        # Otherwise, change to whatever feedback topic you actually use.
        self.subscription = self.create_subscription(
            JointState,
            '/arm_command',
            self.feedback_callback,
            10
        )
        self.latest_feedback = {}  # will hold keys: "joint_positions", "joint_velocities", "joint_names"
        self.get_logger().info("ArmFeedbackSubscriber initialized, subscribed to /joint_states")

    def feedback_callback(self, msg: JointState):
        try:
            positions = list(msg.position)[:6]
            velocities = list(msg.velocity)[:6] if msg.velocity else [0.0] * len(positions)
            names = list(msg.name)[:6] if msg.name else [f"joint_{i}" for i in range(len(positions))]

            # Pad arrays to length 6 if shorter
            while len(positions) < 6:
                positions.append(0.0)
            while len(velocities) < 6:
                velocities.append(0.0)
            while len(names) < 6:
                names.append(f"joint_{len(names)}")

            self.latest_feedback = {
                "joint_positions": positions,
                "joint_velocities": velocities,
                "joint_names": names
            }
        except Exception as e:
            self.get_logger().error(f"Error processing feedback: {e}")


# ─── ArmVelocityPublisher Node ───────────────────────────────────────────────
class ArmVelocityPublisher(Node):
    def __init__(self):
        super().__init__('arm_velocity_publisher')
        try:
            # Publish to /arm_velocity_command with JointState
            self.publisher = self.create_publisher(JointState, '/arm_velocity_command', 10)
            self.get_logger().info("ArmVelocityPublisher initialized, publishing to /arm_velocity_command")
        except Exception as e:
            logging.error(f"Error initializing ArmVelocityPublisher: {e}")

    def publish_velocity(self, velocity_list):
        try:
            # velocity_list should be a list of 6 floats (one per joint, including EE)
            vel_msg = JointState()
            vel_msg.velocity = [float(v) for v in velocity_list]
            # Fill in names for clarity (must match the arm’s joint names)
            vel_msg.name = [f"joint_{i}" for i in range(len(velocity_list))]
            # We do not set vel_msg.position (or effort) here—only velocities matter
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(vel_msg)
            self.get_logger().info(f"Published velocity to /arm_velocity_command: {velocity_list}")
        except Exception as e:
            self.get_logger().error(f"Error publishing velocity command: {e}")

# ─── Instantiate and register the velocity publisher with your ROS2Manager ────────
# (Assuming you already have ros_manager = ROS2Manager.get_instance())
arm_velocity_node = ArmVelocityPublisher()
ros_manager.add_node(arm_velocity_node)


# 1) Create nodes
arm_command_node = ArmCommandPublisher()
arm_feedback_node = ArmFeedbackSubscriber()

# 2) Add them to the ROS2Manager so they start spinning in the background
ros_manager.add_node(arm_command_node)
ros_manager.add_node(arm_feedback_node)


# ─── Django Views ────────────────────────────────────────────────────────────────
@csrf_exempt
def send_arm_command(request):
    """
    POST /api/arm-command/
    Expects JSON body: { "joint_positions": [float0, float1, ..., float5] }
    Publishes those 6 floats as a JointState on /arm_command via arm_command_node.
    """
    if request.method != "POST":
        return JsonResponse({"error": "Invalid request method"}, status=405)

    try:
        data = json.loads(request.body)
        joint_positions = data.get("joint_positions", [])

        if not isinstance(joint_positions, list) or len(joint_positions) != 6:
            return JsonResponse(
                {"error": "Expected 'joint_positions' as a list of 6 floats"},
                status=400
            )

        # Publish to ROS2:
        arm_command_node.publish_arm_command(joint_positions)
        return JsonResponse({"message": "Command sent successfully!"})
    except json.JSONDecodeError:
        logging.error("Invalid JSON in send_arm_command")
        return JsonResponse({"error": "Invalid JSON"}, status=400)
    except Exception as e:
        logging.error(f"Error in send_arm_command: {e}")
        return JsonResponse({"error": "Internal server error"}, status=500)


# =================================


@csrf_exempt
def send_arm_velocity(request):
    """
    Expects a POST with JSON body:
      { "joint_velocities": [v0, v1, v2, v3, v4, v5] }
    Publishes those six floats into JointState.velocity and sends on /arm_velocity_command.
    """
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)

    try:
        payload = json.loads(request.body)
        velocities = payload.get("joint_velocities", [])
    except json.JSONDecodeError:
        logging.error("Invalid JSON in send_arm_velocity.")
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    if not isinstance(velocities, list) or len(velocities) != 6:
        return JsonResponse(
            {"error": "Expected 'joint_velocities' as a list of 6 numbers."},
            status=400
        )

    try:
        # Publish to the ROS2 topic
        arm_velocity_node.publish_velocity(velocities)
        return JsonResponse({"status": "velocity command sent"})
    except Exception as e:
        logging.error(f"Unexpected error in send_arm_velocity: {e}")
        return JsonResponse({"error": "Failed to publish velocity"}, status=500)

# =================================
def get_arm_feedback(request):
    """
    GET /api/arm-feedback/
    Returns latest cached feedback as:
      { "joints": [ {"name": "...", "position": X, "velocity": V}, ... ] }
    If no feedback available yet, returns HTTP 204 with an empty body.
    """
    if request.method != "GET":
        return JsonResponse({"error": "Invalid request method"}, status=405)

    try:
        feedback = arm_feedback_node.latest_feedback
        if not feedback or "joint_positions" not in feedback:
            # No data yet
            return JsonResponse({"joints": []}, status=204)

        positions = feedback["joint_positions"]
        velocities = feedback.get("joint_velocities", [0.0] * len(positions))
        names = feedback.get("joint_names", [f"joint_{i}" for i in range(len(positions))])

        joints = []
        for i in range(len(positions)):
            joints.append({
                "name": names[i],
                "position": positions[i],
                "velocity": velocities[i]
            })

        return JsonResponse({"joints": joints})
    except Exception as e:
        logging.error(f"Error in get_arm_feedback: {e}")
        return JsonResponse({"error": "Failed to retrieve feedback"}, status=500)



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



# ----------------------------

# Logger

# ----------------------------


# ───────────────────────────────────────────────────────────────────────────────
# Point LOG_DIR at your logger’s output folder.
# logger.py lives in: …/robot_controller/log/logger.py
# It writes into a subfolder “logs” right next to itself, i.e.:
#
#   …/robot_controller/log/logs/<topic>_<timestamp>.csv
#
# Your Django “BASE_DIR” is (…)…/ARCh2026-BaseStation/basestationproject
# So the full path to CSVs is:
#
#   BASE_DIR/robot_controller/log/logs
# ───────────────────────────────────────────────────────────────────────────────
LOG_DIR = os.path.join(settings.BASE_DIR, "..", "robot_controller", "log", "logs")

@require_GET
def list_logs(request):
    """
    GET /api/list-logs/
    Returns JSON: { "files": ["arm_command_20250603_150102.csv", ...] }
    """
    try:
        if not os.path.isdir(LOG_DIR):
            return JsonResponse({"files": []})
        files = sorted(f for f in os.listdir(LOG_DIR) if f.endswith(".csv"))
        return JsonResponse({"files": files})
    except Exception as e:
        logging.error(f"Error listing logs in {LOG_DIR}: {e}")
        return JsonResponse({"files": []}, status=500)


@require_GET
def get_log_file(request, filename):
    """
    GET /api/get-log/<filename>/
    Reads logs/<filename> from LOG_DIR and returns:
      { "content": "<entire CSV as one string>" }
    """
    safe_name = os.path.basename(filename)  # prevent path traversal
    full_path = os.path.join(LOG_DIR, safe_name)
    if not os.path.isfile(full_path):
        return JsonResponse({"error": "File not found"}, status=404)

    try:
        with open(full_path, "r") as f:
            data = f.read()
        return JsonResponse({"content": data})
    except Exception as e:
        logging.error(f"Error reading log file {full_path}: {e}")
        return JsonResponse({"error": "Failed to read file"}, status=500)
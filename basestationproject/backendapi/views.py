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
# Custom messages: use only /home/kanga/kanga/ARCH2026-Kanga/src/kanga_interfaces (single source of truth).
# Standard ROS messages (sensor_msgs, std_msgs) used only where kanga_interfaces has no equivalent.
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import logging

try:
    from sensor_msgs.msg import Image, CompressedImage, JointState
    from std_msgs.msg import String, Bool, Empty
    from kanga_interfaces.msg import (
        # ScienceFeedback,
        # ScienceControl,
        # RadioFeedback,
        # CoreFeedback,
        BmsStatus,
        BatteryInfo,
        # RoverLog,
    )
    try:
        from can_msgs.msg import Frame as CanFrame
        CAN_MSGS_AVAILABLE = True
    except ImportError:
        CanFrame = None
        CAN_MSGS_AVAILABLE = False

    ROS_IMPORTS_AVAILABLE = True

except ImportError as e:
    ROS_IMPORTS_AVAILABLE = False
    CAN_MSGS_AVAILABLE = False
    CanFrame = None
    logging.warning(f"ROS message imports failed: {e}. Some features may not work.")



# Third-party Imports
import cv2
import os
import numpy as np
import httpx
import json
import time
import threading
import traceback
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


@require_GET
def status_view(request):
    """GET /api/status/ — backend health/connectivity check for the navbar."""
    return JsonResponse({"status": "ok", "connected": True})


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

# Camera Feedback streams (direct from system: RTSP + USB)

# ----------------------------
# IP cameras: RTSP streams at fixed addresses
# Use rtsp_transport=tcp for more reliable connections (avoids UDP packet loss)
IP_CAMERAS = [
    {"name": "ip_1", "url": "rtsp://10.0.0.5:554/1?rtsp_transport=tcp"},
    {"name": "ip_2", "url": "rtsp://10.0.0.6:554/1?rtsp_transport=tcp"},
]

# USB camera indices to probe when plugged in (e.g. /dev/video0, /dev/video1)
USB_CAMERA_INDICES = [0, 1, 2]


class DirectCameraSource:
    """
    Captures frames directly from the system: either an RTSP stream or a USB device.
    Runs a background thread to continuously read frames. Thread-safe.
    """

    def __init__(self, name, source, source_type="rtsp"):
        """
        Args:
            name (str): Display name for this camera.
            source: For rtsp: URL string. For usb: device index (int).
            source_type (str): "rtsp" or "usb".
        """
        self.name = name
        self.source = source
        self.source_type = source_type
        self.current_frame = None
        self.frame_version = 0  # incremented on each new frame (for cache invalidation)
        self.lock = threading.Lock()
        self._cap = None
        self._running = False
        self._thread = None

    def _open_capture(self):
        """Open the VideoCapture. Returns True if successful."""
        try:
            if self.source_type == "rtsp":
                self._cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
            else:
                self._cap = cv2.VideoCapture(self.source)
            if self._cap and self._cap.isOpened():
                # Reduce buffer to 1 frame for lower latency (especially RTSP)
                self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                return True
        except Exception as e:
            logging.warning(f"Failed to open {self.source_type} source {self.source}: {e}")
        return False

    def _capture_loop(self):
        """Background thread: continuously read frames."""
        while self._running and self._cap and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self.lock:
                    self.current_frame = frame.copy()
                    self.frame_version += 1
            else:
                # Reconnect on failure (e.g. RTSP drop)
                self._cap.release()
                self._cap = None
                if self._running and self._open_capture():
                    continue
                time.sleep(0.5)
        if self._cap:
            self._cap.release()
            self._cap = None

    def start(self):
        """Start the capture thread."""
        if self._running:
            return
        if not self._open_capture():
            logging.warning(f"Camera {self.name} could not be opened.")
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logging.info(f"Camera {self.name} ({self.source_type}) started.")

    def stop(self):
        """Stop the capture thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None


def _probe_usb_cameras():
    """Probe for available USB cameras. Returns list of {name, index} for working devices."""
    found = []
    for idx in USB_CAMERA_INDICES:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                found.append({"name": f"usb_{idx}", "index": idx})
        cap.release()
    return found


def initialize_cameras():
    """
    Initialize direct camera sources: IP cameras (RTSP) + detected USB cameras.
    Returns dict mapping camera_name -> DirectCameraSource.
    """
    camera_sources = {}

    # IP cameras (RTSP)
    for cfg in IP_CAMERAS:
        src = DirectCameraSource(cfg["name"], cfg["url"], source_type="rtsp")
        src.start()
        camera_sources[cfg["name"]] = src

    # USB cameras (direct from system)
    for cfg in _probe_usb_cameras():
        src = DirectCameraSource(cfg["name"], cfg["index"], source_type="usb")
        src.start()
        camera_sources[cfg["name"]] = src

    logging.info(f"Cameras initialized: {list(camera_sources.keys())}")
    return camera_sources


camera_nodes = initialize_cameras()


def get_camera_list(request):
    """Return the list of camera names (IP + detected USB)."""
    return JsonResponse({"cameras": list(camera_nodes.keys())})


@require_GET
def camera_debug(request):
    """
    GET /api/camera-debug/
    Returns status of each camera for debugging RTSP/USB streams:
    - has_frame: whether a frame has been received
    - source_type: rtsp or usb
    - source: URL or device index
    """
    status = {}
    for name, src in camera_nodes.items():
        with src.lock:
            has_frame = src.current_frame is not None
            frame_shape = list(src.current_frame.shape) if has_frame else None
        status[name] = {
            "has_frame": has_frame,
            "frame_shape": frame_shape,
            "source_type": src.source_type,
            "source": str(src.source) if src.source_type == "rtsp" else src.source,
        }
    return JsonResponse({"cameras": status})


# -----------------------------------------------------------------------------
# Link latency (antenna / network RTT)
# Frontend times the round-trip to this endpoint to show latency between
# basestation (e.g. 10.0.0.1) and the device viewing the site (e.g. 10.0.0.2).
# -----------------------------------------------------------------------------
@require_GET
def link_latency(request):
    """
    GET /api/link-latency/
    Returns minimal JSON so the client can measure RTT.
    Client measures: (time when response received) - (time when request sent).
    """
    client_ip = request.META.get("REMOTE_ADDR", "")
    x_forwarded = request.META.get("HTTP_X_FORWARDED_FOR")
    if x_forwarded:
        client_ip = x_forwarded.split(",")[0].strip()
    return JsonResponse({
        "ok": True,
        "ts": int(time.time() * 1000),
        "client_ip": client_ip,
    })


# -----------------------------------------------------------------------------
# Single-Frame HTTP View
# -----------------------------------------------------------------------------
def get_frame(request, camera_name):
    """
    HTTP endpoint that returns the most recent frame from one camera as a
    single JPEG image. If no frame is yet available, returns 204 No Content.

    Args:
        request (HttpRequest): Django request object.
        camera_name (str): Name of the camera (e.g. "top", "back").

    Returns:
        HttpResponse: JPEG image or 204/500 status.
    """
    if camera_name not in camera_nodes:
        return HttpResponse(status=404)
    node = camera_nodes[camera_name]
    if node.current_frame is None:
        return HttpResponse(status=204)
    with node.lock:
        success, jpeg = cv2.imencode('.jpg', node.current_frame)
    if not success:
        return HttpResponse(status=500)
    return HttpResponse(jpeg.tobytes(), content_type="image/jpeg")


# -----------------------------------------------------------------------------
# Continuous MJPEG Streaming View
# -----------------------------------------------------------------------------
def mjpeg_stream(request, camera_name):
    """
    HTTP endpoint that streams an MJPEG (multipart/x-mixed-replace) response
    for a given camera, pushing new frames as they arrive.
    Use ?single=1 to get one JPEG frame (avoids streaming issues).
    """
    if camera_name not in camera_nodes:
        return HttpResponse(status=404)
    node = camera_nodes[camera_name]

    # Single-frame mode: return one JPEG with compression (smaller = faster)
    if request.GET.get("single"):
        try:
            quality = min(95, max(30, int(request.GET.get("q", _DEFAULT_JPEG_QUALITY))))
        except (ValueError, TypeError):
            quality = _DEFAULT_JPEG_QUALITY
        try:
            max_width = min(1920, max(160, int(request.GET.get("w", _DEFAULT_MAX_WIDTH))))
        except (ValueError, TypeError):
            max_width = _DEFAULT_MAX_WIDTH
        with node.lock:
            frame = node.current_frame.copy() if node.current_frame is not None else None
            frame_version = node.frame_version
        # Cache hit: same frame, same params → return cached JPEG (avoids 25 req/s encode)
        cached = _frame_cache.get(camera_name)
        if frame is None:
            _frame_cache.pop(camera_name, None)  # invalidate when no signal
            jpeg_bytes = _get_no_signal_jpeg()
        elif cached and cached[0] == frame_version and cached[2] == quality and cached[3] == max_width:
            jpeg_bytes = cached[1]
        else:
            jpeg_bytes = _compress_frame(frame, quality=quality, max_width=max_width)
            if jpeg_bytes is None:
                jpeg_bytes = _get_no_signal_jpeg()
            else:
                _frame_cache[camera_name] = (frame_version, jpeg_bytes, quality, max_width)
        r = HttpResponse(jpeg_bytes, content_type="image/jpeg")
        r['Cache-Control'] = 'no-cache'
        return r

    boundary = "--frame"
    response = StreamingHttpResponse(
        _frame_generator(node, boundary),
        content_type=f'multipart/x-mixed-replace; boundary={boundary}'
    )
    response['Cache-Control'] = 'no-cache, no-store, must-revalidate'
    response['Pragma'] = 'no-cache'
    response['Expires'] = '0'
    response['Connection'] = 'close'
    return response


# Compression defaults for single-frame mode (smaller = faster transfer)
_DEFAULT_JPEG_QUALITY = 65
_DEFAULT_MAX_WIDTH = 640

# Per-camera cache: camera_name -> (frame_version, jpeg_bytes, quality, max_width)
# Avoids re-encoding the same frame 25x/sec when client polls at 25 fps
_frame_cache = {}


def _compress_frame(frame, quality=_DEFAULT_JPEG_QUALITY, max_width=_DEFAULT_MAX_WIDTH):
    """
    Downscale and encode frame for faster transfer.
    Returns JPEG bytes or None on failure.
    """
    if frame is None:
        return None
    try:
        h, w = frame.shape[:2]
        if w > max_width:
            scale = max_width / w
            new_w = max_width
            new_h = int(h * scale)
            frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
        success, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not success or jpeg is None:
            return None
        return jpeg.tobytes()
    except Exception:
        return None


# Minimal "no signal" placeholder: small gray frame for when camera is unreachable
_NO_SIGNAL_JPEG = None

def _get_no_signal_jpeg():
    global _NO_SIGNAL_JPEG
    if _NO_SIGNAL_JPEG is None:
        gray = np.zeros((90, 160, 3), dtype=np.uint8)
        gray[:] = (48, 48, 48)
        _, jpeg = cv2.imencode('.jpg', gray, [cv2.IMWRITE_JPEG_QUALITY, 85])
        _NO_SIGNAL_JPEG = jpeg.tobytes()
    return _NO_SIGNAL_JPEG


def _frame_generator(camera_node, boundary):
    """
    MJPEG generator that yields *every* new frame immediately.
    When camera has no frames (e.g. RTSP unreachable), sends a placeholder every 2s.
    """
    last_ts = 0.0
    last_placeholder_ts = 0.0
    try:
        while True:
            try:
                # grab the frame under lock
                with camera_node.lock:
                    frame = camera_node.current_frame

                if frame is None:
                    # Camera unreachable—send placeholder every 2s so client gets something
                    now = time.time()
                    if now - last_placeholder_ts >= 2.0:
                        jpeg_bytes = _get_no_signal_jpeg()
                        yield (
                            f"{boundary}\r\n"
                            "Content-Type: image/jpeg\r\n\r\n"
                        ).encode('utf-8') + jpeg_bytes + b"\r\n"
                        last_placeholder_ts = now
                    time.sleep(0.01)
                    continue

                # encode with slightly lower quality for faster streaming (85 vs default 95)
                success, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not success or jpeg is None:
                    continue

                # yield boundary + JPEG
                yield (
                    f"{boundary}\r\n"
                    "Content-Type: image/jpeg\r\n\r\n"
                ).encode('utf-8') + jpeg.tobytes() + b"\r\n"

                # reset last_ts so we don't wait for frame-rate
                last_ts = time.time()

            except (BrokenPipeError, ConnectionResetError, GeneratorExit):
                # client closed—exit quietly
                return
            except Exception as e:
                # Log but don't crash—avoids 500 when frame is corrupted or cv2 fails
                logging.debug("MJPEG frame encode error: %s", e)
                time.sleep(0.05)
                continue
    except (BrokenPipeError, ConnectionResetError, GeneratorExit):
        # client closed—exit quietly
        return





# ----------------------------

# Drivetrain Feedback(only)

# ----------------------------
"""
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

"""

# ----------------------------

# Core Feedback (commented out for now)

# ----------------------------
# class CoreFeedbackSubscriber(Node):
#     def __init__(self):
#         super().__init__('core_feedback_subscriber')
#         try:
#             self.subscription = self.create_subscription(
#                 CoreFeedback, '/core_feedback', self.feedback_callback, 10
#             )
#             self.latest_feedback = {}
#         except Exception as e:
#             logging.error(f"Error initializing core feedback subscriber: {e}")
#
#     def feedback_callback(self, msg):
#         try:
#             self.latest_feedback = {
#                 "epoch_time": int(msg.epoch_time),
#
#                 "wheel_position": [float(x) for x in msg.wheel_position],
#                 "wheel_velocity": [float(x) for x in msg.wheel_velocity],
#                 "wheel_torque": [float(x) for x in msg.wheel_torque],
#
#                 "pitch": round(float(msg.pitch), 2),
#                 "roll": round(float(msg.roll), 2),
#             }
#         except Exception as e:
#             logging.error(f"Error processing core feedback: {e}")
#
#
# core_feedback_node = CoreFeedbackSubscriber()
# ros_manager.add_node(core_feedback_node)
#
#
# def get_core_feedback(request):
#     try:
#         if core_feedback_node.latest_feedback:
#             return JsonResponse(core_feedback_node.latest_feedback)
#         return JsonResponse({"error": "No feedback available"}, status=204)
#     except Exception as e:
#         logging.error(f"Error retrieving core feedback: {e}")
#         return JsonResponse({"error": f"Failed to retrieve feedback: {e}"}, status=500)


# ----------------------------

# Science CAN Subscriber / Publisher (CAN/can1/receive, CAN/can1/transmit)

# ----------------------------
# CAN protocol: 0x100 temps, 0x101 ultrasonic, 0x102 current, 0x103-0x10B spectrophotometer
# Control TX: 0x200 (drill, linear_actuator, heating, cooling, nir, servo)
import struct

SCIENCE_CAN_RX_TOPIC = "CAN/can1/receive"
SCIENCE_CAN_TX_TOPIC = "CAN/can1/transmit"


def _default_science_feedback():
    """Mock data when no CAN frames received."""
    return {
        "temperatures": [22.1, 23.5, 24.0],
        "ultrasonic_cm": 45.2,
        "current_amps": 0.35,
        "spectrophotometer": [0.1 + 0.05 * i for i in range(18)],
        "heating_on": False,
        "cooling_on": False,
        "nir_on": False,
        "drill_state": "stopped",
        "linear_actuator_state": "stopped",
        "servo_angle": 90,
    }


class ScienceCANSubscriber(Node):
    """Subscribes to CAN/can1/receive, decodes frames into latest_science_feedback."""

    def __init__(self):
        super().__init__('science_can_subscriber')
        self._lock = threading.Lock()
        self._latest = _default_science_feedback()
        self._last_can_time = 0
        if CAN_MSGS_AVAILABLE and CanFrame is not None:
            self.subscription = self.create_subscription(
                CanFrame, SCIENCE_CAN_RX_TOPIC, self._can_callback, 10
            )
            self.get_logger().info(f"Subscribed to {SCIENCE_CAN_RX_TOPIC}")
        else:
            self.get_logger().warning("can_msgs not available, using mock data only")

    def _can_callback(self, msg):
        try:
            can_id = msg.id
            data = bytes(msg.data[: msg.dlc])
            with self._lock:
                self._last_can_time = time.time()
                if can_id == 0x100 and len(data) >= 4:
                    temps = []
                    for i in range(0, min(10, len(data)), 2):
                        if i + 2 <= len(data):
                            val = struct.unpack_from("<h", data, i)[0] / 10.0
                            temps.append(round(val, 1))
                    if temps:
                        self._latest["temperatures"] = temps[:5]
                elif can_id == 0x101 and len(data) >= 2:
                    self._latest["ultrasonic_cm"] = round(struct.unpack_from("<H", data, 0)[0] / 10.0, 1)
                elif can_id == 0x102 and len(data) >= 4:
                    self._latest["current_amps"] = round(struct.unpack_from("<f", data, 0)[0], 3)
                elif 0x103 <= can_id <= 0x10B and len(data) >= 8:
                    idx = (can_id - 0x103) * 2
                    v0, v1 = struct.unpack_from("<ff", data, 0)
                    arr = self._latest.get("spectrophotometer", [0.0] * 18)
                    while len(arr) < 18:
                        arr.append(0.0)
                    arr[idx] = round(v0, 4)
                    if idx + 1 < 18:
                        arr[idx + 1] = round(v1, 4)
                    self._latest["spectrophotometer"] = arr
        except Exception as e:
            self.get_logger().error(f"CAN decode error: {e}")

    def get_feedback(self):
        with self._lock:
            return dict(self._latest)


class ScienceCANPublisher(Node):
    """Publishes control frames to CAN/can1/transmit."""

    def __init__(self):
        super().__init__('science_can_publisher')
        self._lock = threading.Lock()
        self._state = {
            "drill": "stopped",
            "linear_actuator": "stopped",
            "heating_on": False,
            "cooling_on": False,
            "nir_on": False,
            "servo_angle": 90,
        }
        if CAN_MSGS_AVAILABLE and CanFrame is not None:
            self.publisher = self.create_publisher(CanFrame, SCIENCE_CAN_TX_TOPIC, 10)
            self.get_logger().info(f"Publishing to {SCIENCE_CAN_TX_TOPIC}")
        else:
            self.publisher = None

    def publish_control(self, data):
        """Encode control dict into CAN frame 0x200 and publish."""
        with self._lock:
            drill = data.get("drill", self._state["drill"])
            linact = data.get("linear_actuator", self._state["linear_actuator"])
            self._state["drill"] = drill if drill in ("left", "right", "stopped") else "stopped"
            self._state["linear_actuator"] = linact if linact in ("up", "down", "stopped") else "stopped"
            self._state["heating_on"] = data.get("heating_on", self._state["heating_on"])
            self._state["cooling_on"] = data.get("cooling_on", self._state["cooling_on"])
            self._state["nir_on"] = data.get("nir_on", self._state["nir_on"])
            servo = int(data.get("servo_angle", self._state["servo_angle"]))
            self._state["servo_angle"] = max(0, min(180, servo))

        if self.publisher is None:
            return
        try:
            msg = CanFrame()
            msg.id = 0x200
            msg.dlc = 8
            msg.is_extended = False
            msg.is_rtr = False
            msg.is_error = False
            drill_map = {"stopped": 0, "left": 1, "right": 2}
            linact_map = {"stopped": 0, "up": 1, "down": 2}
            msg.data = [
                drill_map.get(self._state["drill"], 0),
                linact_map.get(self._state["linear_actuator"], 0),
                1 if self._state["heating_on"] else 0,
                1 if self._state["cooling_on"] else 0,
                1 if self._state["nir_on"] else 0,
                self._state["servo_angle"] & 0xFF,
                0,
                0,
            ]
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"CAN publish error: {e}")


science_can_subscriber = None
science_can_publisher = None
if ROS_IMPORTS_AVAILABLE:
    try:
        science_can_subscriber = ScienceCANSubscriber()
        science_can_publisher = ScienceCANPublisher()
        ros_manager.add_node(science_can_subscriber)
        ros_manager.add_node(science_can_publisher)
    except Exception as e:
        logging.warning(f"Science CAN nodes failed to init: {e}. Science page will use mock data.")


def get_science_feedback(request):
    """GET /api/science-feedback/ - latest science sensor data (from CAN or mock)."""
    try:
        fb = _default_science_feedback()
        if science_can_subscriber is not None:
            fb = science_can_subscriber.get_feedback()
        # Merge last commanded actuator state from publisher
        if science_can_publisher is not None:
            with science_can_publisher._lock:
                fb["drill_state"] = science_can_publisher._state["drill"]
                fb["linear_actuator_state"] = science_can_publisher._state["linear_actuator"]
                fb["heating_on"] = science_can_publisher._state["heating_on"]
                fb["cooling_on"] = science_can_publisher._state["cooling_on"]
                fb["nir_on"] = science_can_publisher._state["nir_on"]
                fb["servo_angle"] = science_can_publisher._state["servo_angle"]
        return JsonResponse(fb)
    except Exception as e:
        logging.error(f"Science feedback error: {e}")
        return JsonResponse(_default_science_feedback())


@csrf_exempt
def set_science_control(request):
    """POST /api/science-control/ - send control commands via CAN."""
    if request.method != "POST":
        return JsonResponse({"error": "Invalid request method"}, status=405)
    try:
        data = json.loads(request.body)
        if science_can_publisher is not None:
            science_can_publisher.publish_control(data)
        return JsonResponse({"status": "success", "message": "Science control command sent!"})
    except json.JSONDecodeError as e:
        logging.error(f"Invalid JSON: {e}")
        return JsonResponse({"error": "Invalid JSON"}, status=400)
    except Exception as e:
        logging.error(f"Science control error: {e}")
        return JsonResponse({"error": "Internal server error"}, status=500)


# (old kanga_interfaces Science nodes removed)
# class _ScienceFeedbackSubscriber(Node):
#     # Subscriber to Science Feedback topic
#     def __init__(self):
#         super().__init__('science_feedback_subscriber')
#         self.subscription = self.create_subscription(
#             ScienceFeedback, '/science_feedback', self.feedback_callback, 10
#         )
#         self.latest_feedback = {}
#
#     def feedback_callback(self, msg):
#         self.latest_feedback = {
#             "rfid": msg.rfid,
#             "moisture": msg.moisture,
#             "potentiometer": msg.potentiometer,
#             "limit": msg.limit,
#             "height": msg.height,
#         }
#
# # Science Control Publisher
# class ScienceControlPublisher(Node):
#     # Publisher to Science Control topic
#     def __init__(self):
#         super().__init__('science_control_publisher')
#         self.publisher = self.create_publisher(ScienceControl, '/science_control', 10)
#
#     def publish_control(self, data):
#         msg = ScienceControl()
#         msg.linear_actuator = data.get("linear_actuator", 0)
#         msg.req_height = data.get("req_height", False)
#         msg.req_nir = data.get("req_nir", False)
#         self.publisher.publish(msg)
#
#
# # Initialize Science Feedback Subscriber
# science_feedback_node = ScienceFeedbackSubscriber()
# ros_manager.add_node(science_feedback_node)
#
# # Initialize Science Control Publisher
# science_control_node = ScienceControlPublisher()
# ros_manager.add_node(science_control_node)
#
#
# def get_science_feedback(request):
#     # Retrieve the latest science feedback data
#     if science_feedback_node.latest_feedback:
#         return JsonResponse(science_feedback_node.latest_feedback)
#     return JsonResponse({"error": "No science feedback available"}, status=204)
#
#
# @csrf_exempt
# def set_science_control(request):
#     # Set science control settings via a ROS2 publisher
#     if request.method == "POST":
#         try:
#             data = json.loads(request.body)
#             science_control_node.publish_control(data)
#             return JsonResponse({"status": "success", "message": "Science control command sent!"})
#         except json.JSONDecodeError:
#             logging.error(f"Invalid JSON received: {e}")
#             return JsonResponse({"error": "Invalid JSON"}, status=400)
#         except Exception as e:
#             logging.error(f"Unexpected error processing science control: {e}")
#             return JsonResponse({"error": "Internal server error"}, status=500)
#
#     return JsonResponse({"error": "Invalid request method"}, status=405)


# ----------------------------

# Log Pub-sub (RoverLogs commented out for now)

# ----------------------------
# class RoverLogsSubscriber(Node):
#     """Subscriber for rover logs using ARCH2026-Kanga/src/kanga_interfaces msg (single source of truth)."""
#     def __init__(self):
#         super().__init__('rover_logs_subscriber')
#         self.subscription = self.create_subscription(
#             RoverLog, 'rover_logs', self.log_callback, 10
#         )
#         self.latest_logs = []
#
#     def log_callback(self, msg: RoverLog):
#         entry = {
#             "timestamp": getattr(msg, "timestamp", 0),
#             "topic_type": getattr(msg, "topic_type", ""),
#             "topic_name": getattr(msg, "topic_name", ""),
#             "topic_message": getattr(msg, "topic_message", ""),
#         }
#         self.latest_logs.append(entry)
#         self.get_logger().info(f"Received log: {entry.get('topic_message', '')}")
#
# # Initialize the log subscriber and add to ROS2 Manager
# rover_logs_node = RoverLogsSubscriber()
# ros_manager.add_node(rover_logs_node)
#
# # Django API Endpoint to Fetch Logs (RoverLog from kanga_interfaces)
# def get_rover_logs(request):
#     try:
#         if rover_logs_node.latest_logs:
#             return JsonResponse({"logs": rover_logs_node.latest_logs})
#         return JsonResponse({"logs": [], "message": "No logs received yet."}, status=204)
#     except Exception as e:
#         logging.error(f"Error retrieving rover logs: {e}")
#         return JsonResponse({"error": "Failed to retrieve logs"}, status=500)


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
        # Subscribe to /joint_states topic (standard ROS2 topic for joint feedback)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.feedback_callback,
            10
        )
        self.latest_feedback = {}  # will hold keys: "joint_positions", "joint_velocities", "joint_names"
        self.get_logger().info("ArmFeedbackSubscriber initialized, subscribed to /joint_states")

    def feedback_callback(self, msg: JointState):
        try:
            # Extract positions (in radians) - expect 5 joints
            positions_rad = list(msg.position)[:5]
            # Convert radians to degrees for frontend display
            positions_deg = [float(p * 180.0 / 3.141592653589793) for p in positions_rad]
            
            # Extract velocities (in rad/s) - expect 5 joints
            velocities_rad = list(msg.velocity)[:5] if msg.velocity and len(msg.velocity) > 0 else [0.0] * len(positions_rad)
            # Convert rad/s to deg/s for frontend
            velocities_deg = [float(v * 180.0 / 3.141592653589793) for v in velocities_rad]
            
            # Use joint names from message if available, otherwise default to J1-J5
            if msg.name and len(msg.name) >= 5:
                names = list(msg.name)[:5]
            else:
                # Default joint names matching URDF: J1, J2, J3, J4, J5
                names = [f"J{i+1}" for i in range(len(positions_deg))]

            # Ensure we have exactly 5 joints
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


# ─── Send commands to interface ────────────────────────────────────────────────────────────────
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

    if not isinstance(velocities, list):
        return JsonResponse(
            {"error": "Expected 'joint_velocities' as a list of numbers."},
            status=400
        )

    # For now, only command the first 5 joints.
    # If a 6th element (gripper) is provided, it is ignored.
    if len(velocities) == 6:
        # Commented out, but kept for reference:
        # full_velocities = velocities
        velocities = velocities[:5]
    elif len(velocities) == 5:
        # Already only joints 1–5.
        pass
    else:
        return JsonResponse(
            {"error": "Expected 'joint_velocities' as a list of 5 or 6 numbers."},
            status=400
        )

    try:
        # Publish to the ROS2 topic with only the first 5 joints.
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

# Radio Feedback & Sub (commented out for now)

# ----------------------------
# class RadioFeedbackSubscriber(Node):
#     # Subscriber to /radio_feedback topic
#     def __init__(self):
#         super().__init__('radio_feedback_subscriber')
#         try:
#             self.subscription = self.create_subscription(
#                 RadioFeedback, '/radio_feedback', self.feedback_callback, 10
#             )
#             self.latest_feedback = {}
#         except Exception as e:
#             logging.error(f"Error initializing radio feedback subscriber: {e}")
#
#     def feedback_callback(self, msg):
#         try:
#             self.latest_feedback = {
#                 "connection": "Connected",
#                 "strength": f"{msg.signal_strength:.1f} dBm",
#                 "ping": msg.ping_ms,
#                 "received": msg.rx_bytes,
#                 "sent": msg.tx_bytes
#             }
#         except Exception as e:
#             logging.error(f"Error processing radio feedback: {e}")
#
#
# radio_feedback_node = RadioFeedbackSubscriber()
# ros_manager.add_node(radio_feedback_node)
#
# def get_radio_feedback(request):
#     try:
#         if radio_feedback_node.latest_feedback:
#             return JsonResponse(radio_feedback_node.latest_feedback)
#         return JsonResponse({"error": "No radio feedback available"}, status=204)
#     except Exception as e:
#         logging.error(f"Error retrieving radio feedback: {e}")
#         return JsonResponse({"error": "Failed to retrieve radio feedback"}, status=500)


# ----------------------------

# Battery Feedback & Sub

# ----------------------------

if ROS_IMPORTS_AVAILABLE:

    class BatteryInfoSubscriber(Node):
        def __init__(self):
            super().__init__("battery_info_subscriber")
            self.latest_msg = None
            self.subscription = self.create_subscription(
                BatteryInfo,
                "/battery/battery_info",
                self.listener_callback,
                10,
            )

        def listener_callback(self, msg):
            self.latest_msg = msg
            self.get_logger().info("BatteryInfo updated")

        def get_latest_data(self):
            return self.latest_msg


    class BmsStatusSubscriber(Node):
        def __init__(self):
            super().__init__("bms_status_subscriber")
            self.latest_msg = None
            self.subscription = self.create_subscription(
                BmsStatus,
                "/battery/bms_status",
                self.listener_callback,
                10,
            )

        def listener_callback(self, msg):
            self.latest_msg = msg
            self.get_logger().info("BmsStatus updated")

        def get_latest_data(self):
            return self.latest_msg


    battery_info_sub = BatteryInfoSubscriber()
    bms_status_sub = BmsStatusSubscriber()

    ros_manager.add_node(battery_info_sub)
    ros_manager.add_node(bms_status_sub)

else:
    logging.warning(
        "ROS message imports unavailable; battery telemetry subscribers disabled."
    )
    battery_info_sub = None
    bms_status_sub = None


def battery_feedback_view(request):
    if not ROS_IMPORTS_AVAILABLE or not battery_info_sub or not bms_status_sub:
        placeholder = {
            "charge_pct": 0.0,
            "current_draw": 0.0,
            "temperature": 0.0,
            "temperature_max": 0.0,
            "temperature_min": 0.0,
            "temps": [],
            "timestamp": int(time.time()),
            "source_timestamp": None,
            "total_voltage": 0.0,
            "measured_voltage": 0.0,
            "capacity": 0,
            "cell_voltages": [],
            "cell_voltages_v": [],
            "charge_state": 0,
            "fault_bits": [],
            "data_status": "unavailable",
        }
        return JsonResponse(placeholder)

    battery_msg = battery_info_sub.get_latest_data()
    bms_msg = bms_status_sub.get_latest_data()

    if battery_msg is None or bms_msg is None:
        placeholder = {
            "charge_pct": 0.0,
            "current_draw": 0.0,
            "temperature": 0.0,
            "temperature_max": 0.0,
            "temperature_min": 0.0,
            "temps": [],
            "timestamp": int(time.time()),
            "source_timestamp": None,
            "total_voltage": 0.0,
            "measured_voltage": 0.0,
            "capacity": 0,
            "cell_voltages": [],
            "cell_voltages_v": [],
            "charge_state": 0,
            "fault_bits": [],
            "data_status": "pending",
        }
        return JsonResponse(placeholder)

    try:
        temps = [int(t) for t in bms_msg.temps]
        cell_voltages_mv = [int(v) for v in bms_msg.cell_voltages]
        fault_bits = [int(f) for f in bms_msg.fault_bits]

        # Derived metrics
        average_temp = sum(temps) / len(temps) if temps else 0.0
        max_temp = max(temps) if temps else 0
        min_temp = min(temps) if temps else 0
        cell_voltages_v = [round(v / 1000.0, 3) for v in cell_voltages_mv]

        header_stamp = getattr(battery_msg, "header", None)
        if header_stamp:
            stamp = header_stamp.stamp
            source_timestamp = stamp.sec + stamp.nanosec / 1e9
        else:
            source_timestamp = None

        data = {
            "charge_pct": float(battery_msg.soc),
            "current_draw": float(battery_msg.current),
            "temperature": float(average_temp),
            "temperature_max": float(max_temp),
            "temperature_min": float(min_temp),
            "temps": temps,
            "timestamp": int(time.time()),
            "source_timestamp": source_timestamp,
            "total_voltage": float(battery_msg.total_voltage),
            "measured_voltage": float(battery_msg.measured_voltage),
            "capacity": int(battery_msg.capacity),
            "cell_voltages": cell_voltages_mv,
            "cell_voltages_v": cell_voltages_v,
            "charge_state": int(bms_msg.charge_state),
            "fault_bits": fault_bits,
        }

        return JsonResponse(data)

    except Exception as e:
        traceback.print_exc()
        return JsonResponse(
            {"error": f"Exception occurred: {str(e)}"}, status=500
        )


"""class BatteryFeedbackSubscriber(Node):
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

"""

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


# ----------------------------
# Django server logs (in-memory buffer)
# ----------------------------
from backendapi.log_buffer import get_django_log_lines


@require_GET
def get_django_logs(request):
    """
    GET /api/django-logs/
    Returns JSON: { "lines": ["...", ...] } from the in-memory Django log buffer.
    """
    try:
        lines = get_django_log_lines()
        return JsonResponse({"lines": lines})
    except Exception as e:
        logging.error(f"Error retrieving Django logs: {e}")
        return JsonResponse({"error": "Failed to retrieve logs", "lines": []}, status=500)

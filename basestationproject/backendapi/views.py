# Django Imports
from django.http import JsonResponse, StreamingHttpResponse, HttpResponse
from django.views.decorators.csrf import csrf_exempt
from django.views.decorators.http import require_GET, require_POST
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

# Core ROS packages (arm, science bridge) — do not bundle with kanga_interfaces: if that
# workspace fails to build/source, battery breaks but arm + kanga_science must still run.
ROS_IMPORTS_AVAILABLE = False
try:
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Bool, Empty, Int32, Float32, Float32MultiArray
    from geometry_msgs.msg import Twist
    ROS_IMPORTS_AVAILABLE = True
except ImportError as e:
    logging.warning(
        "ROS standard message imports failed (arm/science disabled): %s",
        e,
    )

KANGA_INTERFACES_AVAILABLE = False
try:
    from kanga_interfaces.msg import BmsStatus, BatteryInfo
    KANGA_INTERFACES_AVAILABLE = True
except ImportError as e:
    logging.warning(
        "kanga_interfaces not importable (battery topics disabled): %s",
        e,
    )



# Third-party Imports
import cv2
import contextlib
import glob
import os
import re
import subprocess
import sys
import numpy as np
import httpx
import json
import math
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

# Camera feedback: IP cams via RTSP; USB cams via Linux V4L2 (/dev/videoN) — not ROS topics.

# ----------------------------
# IP cameras: RTSP streams at fixed addresses
# Use rtsp_transport=tcp for more reliable connections (avoids UDP packet loss)
IP_CAMERAS = [
    {"name": "ip_1", "url": "rtsp://10.0.0.5:554/1?rtsp_transport=tcp"},
    {"name": "ip_2", "url": "rtsp://10.0.0.6:554/1?rtsp_transport=tcp"},
]

# USB: discovered from /dev/video* at startup and when /api/cameras/ is hit (hotplug).
# IMPORTANT: Multiple UVC cameras on the same USB 2.0 bus share ~480 Mbit/s.
# YUYV 800x600@20fps = ~230 Mbit/s per cam → 2 cams max.
# MJPEG 800x600@15fps = ~15-30 Mbit/s per cam → 6+ cams possible.
# We force MJPEG and limit FPS to avoid bandwidth starvation.
USB_STREAM_WIDTH = 800
USB_STREAM_HEIGHT = 600
USB_STREAM_FPS = 15
USB_INTER_CAMERA_DELAY_S = 0.8  # delay between starting each USB cam to avoid driver race

# FFmpeg / libavformat options for OpenCV's CAP_FFMPEG (low RTSP latency vs default ~1s buffer).
# Syntax: key;value pairs separated by | — applied when the capture is opened.
_RTSP_FFMPEG_OPTS = (
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;250000|reorder_queue_size;0"
)

_usb_camera_registry_lock = threading.Lock()
_last_usb_hotplug_sync = 0.0
_USB_HOTPLUG_MIN_INTERVAL_S = 2.5
_usb_initial_scan_done = False

# USB "valid stream" checks (see _probe_usb_cameras / _try_probe_usb_index):
# 1) Node exists as /dev/videoN; 2) If v4l2-ctl is installed, sysfs/--all must list
#    a "Video Capture" capability (drops metadata-only nodes). 3) OpenCV can open
#    the device and read() at least one frame (short bounded loop — no MJPEG/800x600
#    negotiation during probe to avoid driver QBUF churn). Full format is applied
#    when DirectCameraSource starts. USB scan runs lazily on first camera API use
#    so manage.py check/runserver does not block on every video node at import.


def _opencv_set_log_level_error():
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except AttributeError:
        pass


_opencv_set_log_level_error()


@contextlib.contextmanager
def _silence_stderr():
    try:
        stderr_fd = sys.stderr.fileno()
    except (AttributeError, OSError):
        yield
        return
    devnull = open(os.devnull, "w")
    try:
        old = os.dup(stderr_fd)
        os.dup2(devnull.fileno(), stderr_fd)
        yield
    finally:
        try:
            os.dup2(old, stderr_fd)
            os.close(old)
        except OSError:
            pass
        devnull.close()


def _discover_usb_video_indices():
    """Return sorted OpenCV indices for every /dev/videoN device node (Linux)."""
    if not sys.platform.startswith("linux"):
        return []
    indices = []
    for path in glob.glob("/dev/video[0-9]*"):
        base = os.path.basename(path)
        suffix = base[5:] if base.startswith("video") else ""
        if suffix.isdigit():
            indices.append(int(suffix))
    return sorted(set(indices))


def _v4l2_node_supports_video_capture(dev_path: str) -> bool:
    """
    Skip V4L2 nodes that are metadata-only (e.g. /dev/video9) so we do not register usb_N junk.
    If v4l2-ctl is not installed, do not filter.
    """
    try:
        r = subprocess.run(
            ["v4l2-ctl", "-d", dev_path, "--all"],
            capture_output=True,
            text=True,
            timeout=2.0,
        )
    except FileNotFoundError:
        return True
    except subprocess.TimeoutExpired:
        return True
    blob = (r.stdout or "") + (r.stderr or "")
    if r.returncode != 0:
        return False
    return re.search(r"^\s*Video Capture", blob, re.MULTILINE) is not None


def _usb_video_capture(index: int):
    """
    Open local UVC/V4L2 only (no CAP_FFMPEG for /dev/video* — mixing FFmpeg+V4L2 on the
    same node causes VIDIOC_QBUF / bad-fd noise and stuck drivers on Jetson).
    """
    if not sys.platform.startswith("linux"):
        return cv2.VideoCapture(index)
    idx = int(index)
    path = f"/dev/video{idx}"
    if not os.path.exists(path):
        return cv2.VideoCapture()

    makers = (
        lambda: cv2.VideoCapture(path),
        lambda: cv2.VideoCapture(path, cv2.CAP_V4L2),
        lambda: cv2.VideoCapture(idx, cv2.CAP_V4L2),
        lambda: cv2.VideoCapture(idx),
    )
    with _silence_stderr():
        for make in makers:
            cap = make()
            if cap.isOpened():
                return cap
            cap.release()
    return cv2.VideoCapture()


def _v4l2_warmup_grab(cap, max_attempts: int = 12, delay_s: float = 0.04) -> bool:
    """Bounded read loop — avoids hanging startup if a node opens but never delivers frames."""
    for _ in range(max_attempts):
        ret, _ = cap.read()
        if ret:
            return True
        time.sleep(delay_s)
    return False


def _v4l2_apply_low_latency(cap) -> None:
    """Best-effort V4L2 tuning; drivers may ignore BUFFERSIZE."""
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass


def _configure_usb_capture(cap, log_name: str = "") -> dict:
    """
    Request MJPEG 800×600@15fps from UVC devices (critical for multi-cam USB 2.0 bandwidth).
    Returns dict with actual negotiated format for diagnostics.
    """
    actual = {}
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, USB_STREAM_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, USB_STREAM_HEIGHT)
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_FPS, USB_STREAM_FPS)
    except Exception:
        pass
    _v4l2_apply_low_latency(cap)

    # Read back actual values
    try:
        fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
        fourcc_str = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4))
        actual["fourcc"] = fourcc_str
        actual["width"] = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual["height"] = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual["fps"] = round(cap.get(cv2.CAP_PROP_FPS), 1)
    except Exception:
        pass

    if log_name and actual:
        logging.info(
            "USB %s format: %s %dx%d @ %.1f fps",
            log_name,
            actual.get("fourcc", "?"),
            actual.get("width", 0),
            actual.get("height", 0),
            actual.get("fps", 0),
        )
    return actual


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
        self.actual_format = {}  # populated after USB open: fourcc, width, height, fps

    def _open_capture(self):
        """Open the VideoCapture. Returns True if successful."""
        try:
            if self.source_type == "rtsp":
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _RTSP_FFMPEG_OPTS
                self._cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
            else:
                self._cap = _usb_video_capture(int(self.source))
            if self._cap and self._cap.isOpened():
                if self.source_type == "rtsp":
                    self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                else:
                    self.actual_format = _configure_usb_capture(self._cap, log_name=self.name)
                return True
        except Exception as e:
            logging.warning(f"Failed to open {self.source_type} source {self.source}: {e}")
        return False

    def _capture_loop(self):
        """Background thread: continuously read frames."""
        while self._running and self._cap and self._cap.isOpened():
            if self.source_type == "rtsp":
                # Drop queued frames so we show latest (FFmpeg may still queue briefly).
                self._cap.grab()
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
        usb_retries = 25 if self.source_type == "usb" else 1
        opened = False
        for attempt in range(usb_retries):
            if self._open_capture():
                opened = True
                break
            if self.source_type == "usb":
                time.sleep(0.15)
        if not opened:
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


def _try_probe_usb_index(idx: int):
    """
    Quick validation: device opens, we set MJPEG (the format we'll actually stream),
    and get at least one frame. Probing at the real format avoids "works in probe,
    fails in stream" due to bandwidth or format mismatch.
    """
    cap = _usb_video_capture(idx)
    try:
        if not cap.isOpened():
            return None
        _configure_usb_capture(cap, log_name=f"probe usb_{idx}")
        if not _v4l2_warmup_grab(cap):
            logging.warning("USB probe usb_%s: opened but no frames", idx)
            return None
        return {"name": f"usb_{idx}", "index": idx}
    finally:
        cap.release()


def _probe_usb_cameras():
    """
    Return {name, index} for each usable USB capture device.
    Filters: v4l2-ctl Video Capture line (when v4l2-ctl exists), then OpenCV open + short read().
    Probes are serialized with short delays to avoid hammering USB bus.
    """
    found = []
    indices = _discover_usb_video_indices()
    for i, idx in enumerate(indices):
        dev = f"/dev/video{idx}"
        if sys.platform.startswith("linux") and not _v4l2_node_supports_video_capture(dev):
            logging.debug("USB %s skipped (no Video Capture cap)", dev)
            continue
        cfg = _try_probe_usb_index(idx)
        if cfg:
            found.append(cfg)
        if i < len(indices) - 1:
            time.sleep(0.3)  # brief pause between probes
    return found


def _sync_new_usb_cameras():
    """Attach any newly appeared USB capture devices (no Django restart)."""
    global _last_usb_hotplug_sync
    now = time.monotonic()
    if now - _last_usb_hotplug_sync < _USB_HOTPLUG_MIN_INTERVAL_S:
        return
    _last_usb_hotplug_sync = now
    with _usb_camera_registry_lock:
        new_cams = []
        for idx in _discover_usb_video_indices():
            name = f"usb_{idx}"
            if name in camera_nodes:
                continue
            dev = f"/dev/video{idx}"
            if sys.platform.startswith("linux") and not _v4l2_node_supports_video_capture(dev):
                continue
            cfg = _try_probe_usb_index(idx)
            if not cfg:
                continue
            new_cams.append(cfg)
        for i, cfg in enumerate(new_cams):
            name = cfg["name"]
            src = DirectCameraSource(name, cfg["index"], source_type="usb")
            src.start()
            camera_nodes[name] = src
            logging.info("Registered new USB camera %s (index %s)", name, cfg["index"])
            if i < len(new_cams) - 1:
                time.sleep(USB_INTER_CAMERA_DELAY_S)


def initialize_cameras():
    """
    Start IP (RTSP) cameras only. USB is registered lazily on first camera API call
    so Django import / system checks are not blocked by V4L probing.
    """
    camera_sources = {}

    # IP cameras (RTSP)
    for cfg in IP_CAMERAS:
        src = DirectCameraSource(cfg["name"], cfg["url"], source_type="rtsp")
        src.start()
        camera_sources[cfg["name"]] = src

    logging.info(f"IP cameras initialized: {list(camera_sources.keys())}")
    return camera_sources


camera_nodes = initialize_cameras()


def _register_usb_cameras_if_needed():
    """One-time scan of /dev/video* and start DirectCameraSource for working nodes."""
    global _usb_initial_scan_done, _last_usb_hotplug_sync
    with _usb_camera_registry_lock:
        if _usb_initial_scan_done:
            return
        usb_cams = _probe_usb_cameras()
        for i, cfg in enumerate(usb_cams):
            name = cfg["name"]
            if name in camera_nodes:
                continue
            src = DirectCameraSource(name, cfg["index"], source_type="usb")
            src.start()
            camera_nodes[name] = src
            # Stagger USB camera starts to avoid bandwidth spike / driver race
            if i < len(usb_cams) - 1:
                time.sleep(USB_INTER_CAMERA_DELAY_S)
        _usb_initial_scan_done = True
        _last_usb_hotplug_sync = time.monotonic()
        logging.info("USB cameras registered: %s", [k for k in camera_nodes if k.startswith("usb_")])


def _usb_device_path_for_name(name: str):
    if name.startswith("usb_") and name[4:].isdigit():
        return f"/dev/video{name[4:]}"
    return None


def get_camera_list(request):
    """Return the list of camera names (IP + detected USB); rescans /dev/video* for hotplug."""
    _register_usb_cameras_if_needed()
    _sync_new_usb_cameras()
    names = list(camera_nodes.keys())
    usb_device_paths = {}
    for n in names:
        p = _usb_device_path_for_name(n)
        if p:
            usb_device_paths[n] = p
    return JsonResponse({
        "cameras": names,
        "usb_device_paths": usb_device_paths,
    })


@require_GET
def camera_debug(request):
    """
    GET /api/camera-debug/
    Returns status of each camera for debugging RTSP/USB streams:
    - has_frame: whether a frame has been received
    - source_type: rtsp or usb
    - source: URL or device index
    """
    _register_usb_cameras_if_needed()
    _sync_new_usb_cameras()
    status = {}
    for name, src in camera_nodes.items():
        with src.lock:
            has_frame = src.current_frame is not None
            frame_shape = list(src.current_frame.shape) if has_frame else None
        entry = {
            "has_frame": has_frame,
            "frame_shape": frame_shape,
            "source_type": src.source_type,
            "source": str(src.source) if src.source_type == "rtsp" else src.source,
        }
        if src.source_type == "usb":
            dev = _usb_device_path_for_name(name)
            if dev:
                entry["device"] = dev
            if src.actual_format:
                entry["actual_format"] = src.actual_format
        status[name] = entry
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


# Serialise servo demo runs — concurrent GPIO access would fail or misbehave.
_servo_demo_lock = threading.Lock()


@csrf_exempt
@require_POST
def run_servo_demo(request):
    """
    POST /api/servo-demo/
    Runs scripts/servo_controller.py under the Django project root (basestation host, e.g. Jetson).
    Returns stdout/stderr and ok flag so the dashboard can show whether it actually ran.
    """
    script_path = os.path.abspath(
        os.path.join(settings.BASE_DIR, "scripts", "servo_controller.py")
    )
    if not os.path.isfile(script_path):
        return JsonResponse(
            {
                "ok": False,
                "error": "servo_controller.py not found",
                "path": script_path,
            },
            status=404,
        )

    if not _servo_demo_lock.acquire(blocking=False):
        return JsonResponse(
            {
                "ok": False,
                "error": "Servo demo is already running; wait for it to finish.",
            },
            status=409,
        )

    try:
        r = subprocess.run(
            [sys.executable, script_path],
            capture_output=True,
            text=True,
            timeout=120.0,
            cwd=os.path.dirname(script_path),
        )
        out = (r.stdout or "").strip()
        err = (r.stderr or "").strip()
        success = r.returncode == 0
        # Keep payloads bounded for JSON responses
        if len(out) > 8000:
            out = "…\n" + out[-7990:]
        if len(err) > 4000:
            err = "…\n" + err[-3990:]
        return JsonResponse(
            {
                "ok": success,
                "returncode": r.returncode,
                "stdout": out,
                "stderr": err,
                "message": (
                    "Servo routine finished successfully."
                    if success
                    else "Servo script exited with a non-zero status."
                ),
            }
        )
    except subprocess.TimeoutExpired as e:
        o = e.stdout or ""
        er = e.stderr or ""
        if isinstance(o, bytes):
            o = o.decode(errors="replace")
        if isinstance(er, bytes):
            er = er.decode(errors="replace")
        o, er = o.strip(), er.strip()
        return JsonResponse(
            {
                "ok": False,
                "error": "Servo script timed out (exceeded 120s).",
                "stdout": o[-4000:] if len(o) > 4000 else o,
                "stderr": er[-2000:] if len(er) > 2000 else er,
            },
            status=504,
        )
    except Exception as e:
        logging.exception("run_servo_demo failed")
        return JsonResponse(
            {"ok": False, "error": str(e)},
            status=500,
        )
    finally:
        _servo_demo_lock.release()


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
    _register_usb_cameras_if_needed()
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
    _register_usb_cameras_if_needed()
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

# Science: ROS2 only (no CAN). Subscribes sensor topics; publishes control topics.

# ----------------------------
# Sub (from rover / science stack): std_msgs
#   kanga_science/temperatures        Float32MultiArray  (up to 5 values, °C)
#   kanga_science/ultrasonic_cm       Float32
#   kanga_science/current_amps        Float32
#   kanga_science/spectrophotometer   Float32MultiArray  (up to 18 values)
# Pub (from basestation UI → API): std_msgs
#   kanga_science/linear_actuator_speed  Int32  (-255..255)
#   kanga_science/heating                Bool
#   kanga_science/cooling                Bool


def _default_science_feedback():
    """Fallback when the kanga_science ROS bridge node did not start."""
    return {
        "temperatures": [],
        "ultrasonic_cm": None,
        "current_amps": None,
        "spectrophotometer": [],
        "heating_on": False,
        "cooling_on": False,
        "linear_actuator_speed": 0,
    }


class KangaScienceBridgeNode(Node):
    """Single node: subscribe sensor topics, publish control topics, cache for HTTP API."""

    def __init__(self):
        super().__init__("kanga_science_bridge")
        self._lock = threading.Lock()
        self._sensors = {
            "temperatures": [],
            "ultrasonic_cm": None,
            "current_amps": None,
            "spectrophotometer": [],
        }
        self._state = {
            "linear_actuator_speed": 0,
            "heating_on": False,
            "cooling_on": False,
        }

        self.create_subscription(
            Float32MultiArray, "kanga_science/temperatures", self._cb_temperatures, 10
        )
        self.create_subscription(
            Float32, "kanga_science/ultrasonic_cm", self._cb_ultrasonic, 10
        )
        self.create_subscription(
            Float32, "kanga_science/current_amps", self._cb_current, 10
        )
        self.create_subscription(
            Float32MultiArray,
            "kanga_science/spectrophotometer",
            self._cb_spectrophotometer,
            10,
        )

        self.linear_pub = self.create_publisher(Int32, "kanga_science/linear_actuator_speed", 10)
        self.heating_pub = self.create_publisher(Bool, "kanga_science/heating", 10)
        self.cooling_pub = self.create_publisher(Bool, "kanga_science/cooling", 10)

        self.get_logger().info(
            "kanga_science_bridge: subs temperatures, ultrasonic_cm, current_amps, spectrophotometer; "
            "pubs linear_actuator_speed, heating, cooling"
        )

    def _cb_temperatures(self, msg: Float32MultiArray):
        try:
            data = list(msg.data)[:5]
            with self._lock:
                self._sensors["temperatures"] = [round(float(x), 1) for x in data]
        except Exception as e:
            self.get_logger().error(f"temperatures callback: {e}")

    def _cb_ultrasonic(self, msg: Float32):
        try:
            with self._lock:
                self._sensors["ultrasonic_cm"] = round(float(msg.data), 1)
        except Exception as e:
            self.get_logger().error(f"ultrasonic callback: {e}")

    def _cb_current(self, msg: Float32):
        try:
            with self._lock:
                self._sensors["current_amps"] = round(float(msg.data), 3)
        except Exception as e:
            self.get_logger().error(f"current_amps callback: {e}")

    def _cb_spectrophotometer(self, msg: Float32MultiArray):
        try:
            data = list(msg.data)[:18]
            with self._lock:
                self._sensors["spectrophotometer"] = [round(float(x), 4) for x in data]
        except Exception as e:
            self.get_logger().error(f"spectrophotometer callback: {e}")

    def publish_linear_speed(self, speed):
        speed = max(-255, min(255, int(speed)))
        with self._lock:
            self._state["linear_actuator_speed"] = speed
        m = Int32()
        m.data = speed
        self.linear_pub.publish(m)

    def publish_heating(self, on):
        with self._lock:
            self._state["heating_on"] = bool(on)
        m = Bool()
        m.data = bool(on)
        self.heating_pub.publish(m)

    def publish_cooling(self, on):
        with self._lock:
            self._state["cooling_on"] = bool(on)
        m = Bool()
        m.data = bool(on)
        self.cooling_pub.publish(m)

    def get_feedback(self):
        with self._lock:
            return {
                "temperatures": list(self._sensors["temperatures"]),
                "ultrasonic_cm": self._sensors["ultrasonic_cm"],
                "current_amps": self._sensors["current_amps"],
                "spectrophotometer": list(self._sensors["spectrophotometer"]),
                "linear_actuator_speed": self._state["linear_actuator_speed"],
                "heating_on": self._state["heating_on"],
                "cooling_on": self._state["cooling_on"],
            }


# Registered with ros_manager after arm nodes (see below) so all bridge nodes start together.
kanga_science_node = None


def get_science_feedback(request):
    """GET /api/science-feedback/ — cached values from kanga_science ROS topics + last commands."""
    try:
        if kanga_science_node is not None:
            return JsonResponse(kanga_science_node.get_feedback())
        return JsonResponse(_default_science_feedback())
    except Exception as e:
        logging.error(f"Science feedback error: {e}")
        return JsonResponse(_default_science_feedback())


@csrf_exempt
def set_science_control(request):
    """POST /api/science-control/ - publish control commands to ROS2 topics."""
    if request.method != "POST":
        return JsonResponse({"error": "Invalid request method"}, status=405)
    try:
        data = json.loads(request.body)
        if kanga_science_node is not None:
            if "linear_actuator_speed" in data:
                kanga_science_node.publish_linear_speed(data["linear_actuator_speed"])
            if "heating" in data:
                kanga_science_node.publish_heating(data["heating"])
            if "cooling" in data:
                kanga_science_node.publish_cooling(data["cooling"])
        return JsonResponse({"status": "success"})
    except json.JSONDecodeError as e:
        logging.error(f"Invalid JSON: {e}")
        return JsonResponse({"error": "Invalid JSON"}, status=400)
    except Exception as e:
        logging.error(f"Science control error: {e}")
        return JsonResponse({"error": "Internal server error"}, status=500)


_nir_servo_demo_lock = threading.Lock()


@csrf_exempt
@require_POST
def run_nir_servo_demo(request):
    """POST /api/nir-servo-demo/ - runs gpio_scripts/nir_servo_pin15.py on the host."""
    script_path = os.path.abspath(
        os.path.join(settings.BASE_DIR, "..", "gpio_scripts", "nir_servo_pin15.py")
    )
    if not os.path.isfile(script_path):
        return JsonResponse(
            {"ok": False, "error": "nir_servo_pin15.py not found", "path": script_path},
            status=404,
        )

    if not _nir_servo_demo_lock.acquire(blocking=False):
        return JsonResponse(
            {"ok": False, "error": "NIR servo demo is already running; wait for it to finish."},
            status=409,
        )

    try:
        r = subprocess.run(
            [sys.executable, script_path],
            capture_output=True,
            text=True,
            timeout=120.0,
            cwd=os.path.dirname(script_path),
        )
        out = (r.stdout or "").strip()
        err = (r.stderr or "").strip()
        success = r.returncode == 0
        if len(out) > 8000:
            out = "…\n" + out[-7990:]
        if len(err) > 4000:
            err = "…\n" + err[-3990:]
        return JsonResponse({
            "ok": success,
            "returncode": r.returncode,
            "stdout": out,
            "stderr": err,
            "message": (
                "NIR servo routine finished."
                if success
                else "NIR servo script exited with a non-zero status."
            ),
        })
    except subprocess.TimeoutExpired as e:
        o = e.stdout or ""
        er = e.stderr or ""
        if isinstance(o, bytes):
            o = o.decode(errors="replace")
        if isinstance(er, bytes):
            er = er.decode(errors="replace")
        o, er = o.strip(), er.strip()
        return JsonResponse(
            {
                "ok": False,
                "error": "NIR servo script timed out (exceeded 120s).",
                "stdout": o[-4000:] if len(o) > 4000 else o,
                "stderr": er[-2000:] if len(er) > 2000 else er,
            },
            status=504,
        )
    except Exception as e:
        logging.exception("run_nir_servo_demo failed")
        return JsonResponse({"ok": False, "error": str(e)}, status=500)
    finally:
        _nir_servo_demo_lock.release()


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
    # J6 position-increment parameters (mirror joy_to_hybrid_control defaults)
    _J6_PWM_SPEED = 30.0    # deg/s per unit of j6 velocity (-1..1)
    _J6_MIN_ANGLE = 0.0     # degrees
    _J6_MAX_ANGLE = 180.0   # degrees
    _J6_START_ANGLE = 180.0  # degrees

    def __init__(self):
        super().__init__('arm_velocity_publisher')
        self._j6_position = self._J6_START_ANGLE
        self._j6_last_time = None  # wall-clock time of last publish (None = uninitialized)
        try:
            self.publisher = self.create_publisher(JointState, '/kanga_arm/joint_control', 100)
            self.get_logger().info("ArmVelocityPublisher initialized, publishing to /kanga_arm/joint_control")
        except Exception as e:
            logging.error(f"Error initializing ArmVelocityPublisher: {e}")

    def publish_velocity(self, velocity_list):
        try:
            # J1-J6: all velocities normalized -1..1 (no deg/rad conversion)
            vel = [max(-1.0, min(1.0, float(v))) for v in velocity_list[:5]]
            while len(vel) < 5:
                vel.append(0.0)
            j6_vel = float(velocity_list[5]) if len(velocity_list) > 5 else 0.0
            j6_vel = max(-1.0, min(1.0, j6_vel))
            vel.append(j6_vel)

            # Compute dt for J6 position integration (degrees 0-180)
            now = time.monotonic()
            dt = 0.0 if self._j6_last_time is None else (now - self._j6_last_time)
            self._j6_last_time = now

            # Accumulate J6 position (deg) based on velocity and increment rate, clamp 0-180
            self._j6_position += j6_vel * self._J6_PWM_SPEED * max(dt, 0.0)
            self._j6_position = max(self._J6_MIN_ANGLE, min(self._J6_MAX_ANGLE, self._j6_position))

            msg = JointState()
            msg.velocity = vel
            msg.name = ["j1", "j2", "j3", "j4", "j5", "j6"]
            msg.position = [0.0] * 6
            msg.position[5] = self._j6_position
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            self.get_logger().info(
                f"Published velocity to /kanga_arm/joint_control: {vel[:5]} "
                f"j6_vel={j6_vel} j6_pos={self._j6_position:.2f}deg")
        except Exception as e:
            self.get_logger().error(f"Error publishing velocity command: {e}")

# ─── ArmEEPublisher Node (Twist to kanga_arm/ee_state_control) ─────────────────
class ArmEEPublisher(Node):
    def __init__(self):
        super().__init__('arm_ee_publisher')
        try:
            self.publisher = self.create_publisher(Twist, 'kanga_arm/ee_state_control', 100)
            self.get_logger().info("ArmEEPublisher initialized, publishing to kanga_arm/ee_state_control")
        except Exception as e:
            logging.error(f"Error initializing ArmEEPublisher: {e}")

    def publish_ee(self, linear_y, linear_z, angular_x):
        try:
            msg = Twist()
            msg.linear.x = float(linear_y)
            msg.linear.y = 0.0
            msg.linear.z = float(linear_z)
            msg.angular.x = 0.0
            msg.angular.y = float(angular_x)
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info(
                f"Published EE: Vx={msg.linear.x} Vz={msg.linear.z} Wy={msg.angular.y}")
        except Exception as e:
            self.get_logger().error(f"Error publishing EE command: {e}")

    def publish_zero(self):
        try:
            msg = Twist()
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing zero twist: {e}")


# ─── ArmModePublisher Node (Bool to kanga_arm/control_mode_joint) ──────────────
class ArmModePublisher(Node):
    def __init__(self):
        super().__init__('arm_mode_publisher')
        try:
            self.publisher = self.create_publisher(Bool, 'kanga_arm/control_mode_joint', 100)
            self.get_logger().info("ArmModePublisher initialized, publishing to kanga_arm/control_mode_joint")
        except Exception as e:
            logging.error(f"Error initializing ArmModePublisher: {e}")

    def publish_mode(self, is_joint: bool):
        try:
            msg = Bool()
            msg.data = is_joint
            self.publisher.publish(msg)
            self.get_logger().info(f"Published mode: {'joint' if is_joint else 'ee'}")
        except Exception as e:
            self.get_logger().error(f"Error publishing mode: {e}")


# ─── Instantiate and register all arm nodes ────────────────────────────────────
arm_velocity_node = ArmVelocityPublisher()
ros_manager.add_node(arm_velocity_node)

arm_ee_node = ArmEEPublisher()
ros_manager.add_node(arm_ee_node)

arm_mode_node = ArmModePublisher()
ros_manager.add_node(arm_mode_node)

arm_feedback_node = ArmFeedbackSubscriber()
ros_manager.add_node(arm_feedback_node)

# Science bridge (same process as arm publishers — topics appear in `ros2 topic list` on this host)
if ROS_IMPORTS_AVAILABLE:
    try:
        kanga_science_node = KangaScienceBridgeNode()
        ros_manager.add_node(kanga_science_node)
        logging.info(
            "KangaScienceBridgeNode registered — expect /kanga_science/linear_actuator_speed, "
            "/kanga_science/heating, /kanga_science/cooling (+ sensor subs)."
        )
    except Exception:
        logging.exception(
            "KangaScienceBridgeNode failed to init; Science API will use empty feedback. "
            "Check journal for traceback."
        )
        kanga_science_node = None


# ─── Send velocity commands to interface ────────────────────────────────────────────
@csrf_exempt
def send_arm_velocity(request):
    """
    Expects a POST with JSON body:
      { "joint_velocities": [v0, v1, v2, v3, v4, v5] }
    Publishes JointState to /kanga_arm/joint_control (deg/s converted to rad/s).
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

    if len(velocities) == 5:
        velocities = list(velocities) + [0.0]
    elif len(velocities) != 6:
        return JsonResponse(
            {"error": "Expected 'joint_velocities' as a list of 5 or 6 numbers."},
            status=400
        )

    try:
        arm_velocity_node.publish_velocity(velocities)
        return JsonResponse({"status": "velocity command sent"})
    except Exception as e:
        logging.error(f"Unexpected error in send_arm_velocity: {e}")
        return JsonResponse({"error": "Failed to publish velocity"}, status=500)


# ─── EE command endpoint (Twist Vy/Vz/Wx + joint_control J1/J5/J6) ───────────
@csrf_exempt
def send_arm_ee_command(request):
    """
    POST /api/arm-ee-command/
    Body: {
      "linear_y": float (Vy, EE frame),
      "linear_z": float (Vz, EE frame),
      "angular_x": float (Wx, EE frame),
      "j1_velocity": float (optional, deg/s),
      "j5_velocity": float (optional, deg/s),
      "j6_velocity": float (optional, -1..1)
    }
    Publishes Twist (Vy, Vz, Wx) to kanga_arm/ee_state_control and
    JointState (J1/J5/J6 only) to /kanga_arm/joint_control.
    """
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)

    try:
        p = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    try:
        ly = float(p.get("linear_y", 0))
        lz = float(p.get("linear_z", 0))
        wx = float(p.get("angular_x", 0))
        j1 = float(p.get("j1_velocity", 0))
        j5 = float(p.get("j5_velocity", 0))
        j6 = float(p.get("j6_velocity", 0))
    except (TypeError, ValueError):
        return JsonResponse({"error": "All values must be numbers"}, status=400)

    try:
        arm_ee_node.publish_ee(ly, lz, wx)
        arm_velocity_node.publish_velocity([j1, 0, 0, 0, j5, j6])
        return JsonResponse({"status": "ee command sent"})
    except Exception as e:
        logging.error(f"Error in send_arm_ee_command: {e}")
        return JsonResponse({"error": "Failed to publish EE command"}, status=500)


# ─── Mode toggle endpoint ────────────────────────────────────────────────────────
@csrf_exempt
def set_arm_mode(request):
    """
    POST /api/arm-mode/
    Body: { "mode": "joint" | "ee" }
    Publishes Bool to kanga_arm/control_mode_joint (true = joint).
    On switch to joint mode, a zero Twist is also published to clear EE.
    """
    if request.method != "POST":
        return JsonResponse({"error": "POST required"}, status=405)

    try:
        p = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "Invalid JSON"}, status=400)

    mode = p.get("mode", "").lower()
    if mode not in ("joint", "ee"):
        return JsonResponse({"error": "mode must be 'joint' or 'ee'"}, status=400)

    is_joint = mode == "joint"
    try:
        arm_mode_node.publish_mode(is_joint)
        if is_joint:
            arm_ee_node.publish_zero()
        return JsonResponse({"status": f"mode set to {mode}"})
    except Exception as e:
        logging.error(f"Error in set_arm_mode: {e}")
        return JsonResponse({"error": "Failed to publish mode"}, status=500)


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

if KANGA_INTERFACES_AVAILABLE:

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
        "kanga_interfaces unavailable; battery telemetry subscribers disabled."
    )
    battery_info_sub = None
    bms_status_sub = None


def battery_feedback_view(request):
    if not KANGA_INTERFACES_AVAILABLE or not battery_info_sub or not bms_status_sub:
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

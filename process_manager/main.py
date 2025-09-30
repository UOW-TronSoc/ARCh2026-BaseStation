#!/usr/bin/env python3
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import subprocess, os, signal, shlex
from pathlib import Path


def resolve_ros_install_root() -> Path:
    """Return best-guess path to the ROS 2 workspace install directory."""
    env_hint = os.environ.get("ROS_INSTALL_PREFIX")
    if env_hint:
        hint = Path(env_hint)
        if hint.exists():
            return hint

    local_ws = (Path(__file__).resolve().parent / ".." /
                "basestationproject" / "ros2_ws" / "install").resolve()
    if local_ws.exists():
        return local_ws

    container_ws = Path("/ros2_ws/install")
    if container_ws.exists():
        return container_ws

    return local_ws  # fall back to local path even if missing


ROS_DISTRO = os.environ.get("ROS_DISTRO", "humble")
ROS_INSTALL_ROOT = resolve_ros_install_root()
CUSTOM_LIB_PATH = ROS_INSTALL_ROOT / "custom_msgs" / "lib"


def build_launch_command(script_cmd: str) -> str:
    """Compose shell command that sources ROS setup files before running."""
    ros_setup_candidates = [
        Path(f"/opt/ros/{ROS_DISTRO}/setup.bash"),
        ROS_INSTALL_ROOT / "setup.bash",
    ]

    parts = []
    for setup in ros_setup_candidates:
        if setup.exists():
            parts.append(f"source {setup}")

    if CUSTOM_LIB_PATH.exists():
        parts.append(
            f"export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{CUSTOM_LIB_PATH}"
        )
        parts.append(
            f"export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:{CUSTOM_LIB_PATH}"
        )

    parts.append(f"python3 {script_cmd}")
    return "bash -c \"" + " && ".join(parts) + "\""

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten up in prod
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- dynamically discover all camera videos ---
camera_videos_dir = Path(
    os.path.abspath("robot_controller/camera/videos")
)
video_files = sorted(camera_videos_dir.glob("video*.mp4"))

scripts = {}
# one entry per video file
for idx, video in enumerate(video_files):
    name = f"Camera {idx}"
    publisher_py = Path(
        os.path.abspath("robot_controller/camera/camera_video_publisher.py")
    )
    scripts[name] = " ".join([
        shlex.quote(str(publisher_py)),
        "--camera-id",
        str(idx),
        "--video-path",
        shlex.quote(str(video)),
    ])

# then all your other publishers
static = {
    "Battery": "robot_controller/battery/battery_publisher.py",
    "Radio": "robot_controller/radio/radio_feedback_pub.py",
    "Core pub": "robot_controller/drive_control/core_publisher.py",
    "Arm Feedback": "robot_controller/arm/test_arm_feedback_publisher.py",
    "Fake Joint Integrator": "robot_controller/arm/fake_integrator.py",
    "Logger": "robot_controller/log/logger.py",
}
for k, v in static.items():
    scripts[k] = shlex.quote(os.path.abspath(v))

processes = {}

@app.get("/status")
def get_status():
    return {
        name: ("running" if name in processes and processes[name].poll() is None
               else "stopped")
        for name in scripts
    }

@app.post("/start-script/{script_name}")
def start_script(script_name: str):
    if script_name not in scripts:
        raise HTTPException(status_code=404, detail="Unknown script")
    if script_name in processes and processes[script_name].poll() is None:
        return {"status": "already running"}

    cmd = build_launch_command(scripts[script_name])
    processes[script_name] = subprocess.Popen(
        cmd,
        shell=True,
        executable='/bin/bash',
        preexec_fn=os.setsid
    )


    return {"status": "started"}

@app.post("/stop-script/{script_name}")
def stop_script(script_name: str):
    if script_name not in processes:
        return {"status": "not running"}
    p = processes[script_name]
    # send SIGINT to the whole process group
    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    p.wait()
    del processes[script_name]
    return {"status": "stopped"}

@app.post("/start-all")
def start_all():
    for name in scripts:
        if name not in processes or processes[name].poll() is not None:
            start_script(name)
    return {"status": "all started"}

@app.post("/stop-all")
def stop_all():
    for name in list(processes):
        stop_script(name)
    return {"status": "all stopped"}

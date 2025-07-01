#!/usr/bin/env python3
import os
import signal
import subprocess
from pathlib import Path

import yaml
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

# where your custom ROS 2 msgs live
custom_lib_path = os.path.abspath("../basestationproject/ros2_ws/install/custom_msgs/lib")

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten up in prod
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- dynamically discover all camera videos ---
camera_videos_dir = Path(os.path.abspath("../robot_controller/camera/videos"))
video_files = sorted(camera_videos_dir.glob("video*.mp4"))

scripts = {}
# one entry per video file
for idx, video in enumerate(video_files):
    name = f"Camera {idx}"
    # absolute path to your publisher script
    publisher_py = os.path.abspath("../robot_controller/camera/camera_video_publisher.py")
    scripts[name] = f"{publisher_py} --camera-id {idx} --video-path {video}"

# then all your other publishers
static = {
    "Battery": "../robot_controller/battery/battery_publisher.py",
    "Radio": "../robot_controller/radio/radio_feedback_pub.py",
    "Core pub": "../robot_controller/drive_control/core_publisher.py",
    "Arm Feedback": "../robot_controller/arm/test_arm_feedback_publisher.py",
    "Fake Joint Integrator": "../robot_controller/arm/fake_integrator.py",
    "Logger": "../robot_controller/log/logger.py",
}
for k, v in static.items():
    scripts[k] = v

processes = {}

# Load setup.yaml at startup
with open(os.path.abspath("../setup.yaml"), "r") as f:
    config = yaml.safe_load(f)

# Example: Access controller mappings
controller_mappings = config.get("controllerKeyMappings", {})

# Example: Use a value from YAML in your endpoint or logic
# print(controller_mappings)


@app.get("/status")
def get_status():
    return {
        name: ("running" if name in processes and processes[name].poll() is None else "stopped") for name in scripts
    }


@app.post("/start-script/{script_name}")
def start_script(script_name: str):
    if script_name not in scripts:
        raise HTTPException(status_code=404, detail="Unknown script")
    if script_name in processes and processes[script_name].poll() is None:
        return {"status": "already running"}

    cmd = f"bash -c 'export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:{custom_lib_path} && python3 {scripts[script_name]}'"
    processes[script_name] = subprocess.Popen(cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid)

    return {"status": "started"}


@app.post("/stop-script/{script_name}")
def stop_script(script_name: str):
    if script_name not in processes:
        return {"status": "not running"}
    p = processes[script_name]
    # send SIGINT to the whole process group
    os.killpg(os.getpgid(p.pid), signal.SIGINT)
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

"""
Local service probes and host metrics for the Log Viewer / ops dashboard.
Probes use loopback — intended for basestation services co-located with Django.
"""
from __future__ import annotations

import glob
import os
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional

import httpx

DEFAULT_PROBE_HOST = os.environ.get("BASESTATION_PROBE_HOST", "127.0.0.1")
ARM_FASTAPI_PORT = int(os.environ.get("BASESTATION_ARM_FASTAPI_PORT", "8001"))
DRIVE_FASTAPI_PORT = int(os.environ.get("BASESTATION_DRIVE_FASTAPI_PORT", "8080"))


def _probe_get(url: str, timeout: float = 1.0) -> Dict[str, Any]:
    t0 = time.perf_counter()
    try:
        with httpx.Client(timeout=timeout) as client:
            r = client.get(url)
        ms = round((time.perf_counter() - t0) * 1000, 2)
        ok = 200 <= r.status_code < 300
        err = None if ok else (r.text[:300] if r.text else f"HTTP {r.status_code}")
        return {
            "running": ok,
            "http_status": r.status_code,
            "latency_ms": ms,
            "error": err,
        }
    except Exception as e:
        ms = round((time.perf_counter() - t0) * 1000, 2)
        return {
            "running": False,
            "http_status": None,
            "latency_ms": ms,
            "error": str(e),
        }


def probe_local_services(
    host: Optional[str] = None,
    arm_port: Optional[int] = None,
    drive_port: Optional[int] = None,
) -> Dict[str, Any]:
    host = host or DEFAULT_PROBE_HOST
    ap = ARM_FASTAPI_PORT if arm_port is None else arm_port
    dp = DRIVE_FASTAPI_PORT if drive_port is None else drive_port
    arm_url = f"http://{host}:{ap}/arm/feedback"
    drive_url = f"http://{host}:{dp}/openapi.json"
    return {
        "arm_fastapi": {
            **_probe_get(arm_url),
            "probe": "GET /arm/feedback",
            "port": ap,
        },
        "drive_fastapi": {
            **_probe_get(drive_url),
            "probe": "GET /openapi.json",
            "port": dp,
        },
    }


def _read_cpu_percent(sample_interval: float = 0.12) -> Optional[float]:
    """Overall CPU usage from /proc/stat (all cores)."""

    def jiffies() -> Optional[tuple]:
        try:
            with open("/proc/stat", "r") as f:
                line = f.readline()
            if not line.startswith("cpu "):
                return None
            parts = line.split()
            nums = [int(x) for x in parts[1:8]]
            idle = nums[3] + nums[4]
            total = sum(nums)
            return idle, total
        except (OSError, ValueError, IndexError):
            return None

    a = jiffies()
    if not a:
        return None
    time.sleep(sample_interval)
    b = jiffies()
    if not b:
        return None
    idle_d = b[0] - a[0]
    total_d = b[1] - a[1]
    if total_d <= 0:
        return None
    return round(100.0 * (1.0 - idle_d / total_d), 1)


def _read_memory() -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "total_bytes": None,
        "used_bytes": None,
        "available_bytes": None,
        "used_percent": None,
    }
    try:
        mem_total_kb = 0
        mem_avail_kb = 0
        with open("/proc/meminfo", "r") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    mem_total_kb = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    mem_avail_kb = int(line.split()[1])
                    break
        if mem_total_kb <= 0:
            return out
        total = mem_total_kb * 1024
        avail = mem_avail_kb * 1024
        used = total - avail
        out["total_bytes"] = total
        out["available_bytes"] = avail
        out["used_bytes"] = used
        out["used_percent"] = round(100.0 * used / total, 1)
    except (OSError, ValueError, IndexError):
        pass
    return out


def _read_thermal_zones() -> List[Dict[str, Any]]:
    zones: List[Dict[str, Any]] = []
    base = "/sys/class/thermal"
    if not os.path.isdir(base):
        return zones
    for name in sorted(os.listdir(base)):
        if not name.startswith("thermal_zone"):
            continue
        zpath = os.path.join(base, name)
        try:
            with open(os.path.join(zpath, "type"), "r") as tf:
                typ = tf.read().strip()
            with open(os.path.join(zpath, "temp"), "r") as tf:
                millic = int(tf.read().strip())
            zones.append({"id": name, "type": typ, "celsius": round(millic / 1000.0, 1)})
        except (OSError, ValueError):
            continue
    return zones


def _nvidia_smi_gpu() -> Optional[Dict[str, Any]]:
    if sys.platform == "win32":
        return None
    try:
        r = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            text=True,
            timeout=2.5,
        )
        if r.returncode != 0 or not (r.stdout or "").strip():
            return None
        line = r.stdout.strip().split("\n")[0]
        parts = [p.strip() for p in line.split(",")]
        if len(parts) < 4:
            return None

        def parse_field(s: str) -> Optional[float]:
            u = s.strip().upper()
            if u in ("N/A", "[N/A]", ""):
                return None
            try:
                return float(s.strip().split()[0])
            except (ValueError, IndexError):
                return None

        util = parse_field(parts[0])
        mem_u = parse_field(parts[1])
        mem_t = parse_field(parts[2])
        temp = parse_field(parts[3])
        if util is None and mem_u is None and mem_t is None and temp is None:
            return {"source": "nvidia-smi", "available": False}
        return {
            "source": "nvidia-smi",
            "available": True,
            "utilization_percent": util,
            "memory_used_mb": mem_u,
            "memory_total_mb": mem_t,
            "temperature_c": temp,
        }
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return None


def _jetson_gpu_clock_mhz() -> Optional[Dict[str, Any]]:
    for path in sorted(glob.glob("/sys/class/devfreq/*gpu*/cur_freq")):
        try:
            with open(path, "r") as f:
                hz = int(f.read().strip())
            return {"cur_freq_mhz": round(hz / 1_000_000, 1), "sysfs": path}
        except (OSError, ValueError):
            continue
    return None


def collect_system_metrics() -> Dict[str, Any]:
    zones = _read_thermal_zones()
    max_c = max((z["celsius"] for z in zones), default=None)
    nv = _nvidia_smi_gpu()
    jclock = _jetson_gpu_clock_mhz()

    load1: Optional[float] = None
    try:
        load1 = round(os.getloadavg()[0], 2)
    except (OSError, AttributeError):
        pass

    return {
        "cpu_percent": _read_cpu_percent(),
        "loadavg_1m": load1,
        "memory": _read_memory(),
        "temperature_c_max": max_c,
        "thermal_zones": zones,
        "gpu": {
            "nvidia_smi": nv,
            "jetson_clock_mhz": jclock["cur_freq_mhz"] if jclock else None,
            "jetson_sysfs": jclock["sysfs"] if jclock else None,
        },
    }

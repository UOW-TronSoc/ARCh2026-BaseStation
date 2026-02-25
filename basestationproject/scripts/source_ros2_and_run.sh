#!/usr/bin/env bash
# Sources the ROS2 kanga_interfaces workspace and runs the given command.
# Optional: set BASESTATION_VENV to a venv path to activate before running.
# Usage: source_ros2_and_run.sh <command...>

set -e
# User-installed Python packages (pip install --user) and optional venv
export PATH="${HOME}/.local/bin:${PATH:-/usr/bin:/bin}"
if [[ -n "${BASESTATION_VENV}" && -f "${BASESTATION_VENV}/bin/activate" ]]; then
  source "${BASESTATION_VENV}/bin/activate"
fi
KANGA_WS="${KANGA_ROS2_WS:-/home/kanga/kanga/ARCH2026-Kanga}"
if [[ -f "$KANGA_WS/install/setup.bash" ]]; then
  source "$KANGA_WS/install/setup.bash"
fi
exec "$@"

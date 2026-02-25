# Basestation startup services (no Docker)

These systemd services start Django, FastAPI, and the frontend (Vite) on boot. They run natively so ROS2 and your `kanga_interfaces` workspace work normally.

## Prerequisites

- **Django:** Dependencies must be visible to the service. Either:
  - `pip3 install --user -r requirements.txt` (so `~/.local/bin` and user site-packages are used), or
  - Create a venv, install there, and set `BASESTATION_VENV` in the Django service to the venv path (e.g. `Environment=BASESTATION_VENV=/home/kanga/kanga/ARCh2026-BaseStation/basestationproject/venv`).
- **FastAPI:** Same as Django if needed (or ensure `python3` used by the script can import FastAPI/uvicorn).
- **Frontend:** Node/npm (e.g. via nvm). The service uses `scripts/run_npm_dev.sh` to load nvm and run `npm run dev`. Run `cd frontend && npm install` once.
- **ROS2** and `kanga_interfaces` built at `KANGA_ROS2_WS`. See [ROS2_SETUP.md](ROS2_SETUP.md).

## Paths and user

- **Project root:** `/home/kanga/kanga/ARCh2026-BaseStation/basestationproject`
- **ROS2 workspace:** `/home/kanga/kanga/ARCH2026-Kanga`
- **Service user:** `kanga`

If your paths or user differ, edit the `WorkingDirectory`, `ExecStart`, and `User` lines in the `.service` files and the path in `scripts/source_ros2_and_run.sh` (or set `KANGA_ROS2_WS` in the service `Environment`).

## Install and enable on boot

```bash
# Make scripts executable
chmod +x /home/kanga/kanga/ARCh2026-BaseStation/basestationproject/scripts/source_ros2_and_run.sh
chmod +x /home/kanga/kanga/ARCh2026-BaseStation/basestationproject/scripts/run_npm_dev.sh

# Copy service files (requires sudo)
sudo cp systemd/basestation-django.service   /etc/systemd/system/
sudo cp systemd/basestation-fastapi.service  /etc/systemd/system/
sudo cp systemd/basestation-frontend.service /etc/systemd/system/

# Reload systemd and enable to start on boot
sudo systemctl daemon-reload
sudo systemctl enable basestation-django basestation-fastapi basestation-frontend

# Start now (optional)
sudo systemctl start basestation-django basestation-fastapi basestation-frontend
```

## Useful commands

```bash
# Status
sudo systemctl status basestation-django basestation-fastapi basestation-frontend

# Logs (last 50 lines)
sudo journalctl -u basestation-django -n 50 -f
sudo journalctl -u basestation-fastapi -n 50 -f
sudo journalctl -u basestation-frontend -n 50 -f

# Stop / start / restart
sudo systemctl stop basestation-django basestation-fastapi basestation-frontend
sudo systemctl start basestation-django basestation-fastapi basestation-frontend
sudo systemctl restart basestation-django
```

## Ports

- **Django:** 8000 (API, video feeds, etc.)
- **FastAPI:** 8080 (e.g. `/command` for drive)
- **Frontend (Vite):** 5173

## Disable on boot

```bash
sudo systemctl disable basestation-django basestation-fastapi basestation-frontend
```

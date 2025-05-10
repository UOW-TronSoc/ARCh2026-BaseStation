#!/bin/bash

# Navigate to the base station project directory
cd ~/Documents/ARCh2025-BaseStation/basestationproject

# Source the ROS 2 environment
source ros2_ws/install/setup.bash

# Launch FastAPI server
gnome-terminal --title="FastAPI Server" --tab -- bash -c "cd fastapi_server; uvicorn main:app --host 127.0.0.1 --port 8080 --reload; exec bash"

# Launch React frontend
gnome-terminal --title="React Frontend" --tab -- bash -c "cd frontend; npm start; exec bash"

# Launch Django server
gnome-terminal --title="Django Server" --tab -- bash -c "python3 manage.py runserver; exec bash"

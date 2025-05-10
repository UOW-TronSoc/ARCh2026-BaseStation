# Base Station Project

This repository contains the setup and instructions for running the base station project, which consists of a Django backend, a FastAPI service, and a React frontend.

## Prerequisites

Before starting, ensure you have the following installed:
- ROS 2
- Python 3
- Django
- FastAPI
- Uvicorn
- Node.js & npm

## Setup Instructions

### 0. Source ROS 2 Environment
Before running any services, you need to source the ROS 2 environment:
```sh
source ros2_ws/install/setup.bash
```

### 1. Run Django Server
Navigate to the Django project directory and start the server:
```sh
cd basestationproject/
python3 manage.py runserver
```
This will start the Django development server on `http://127.0.0.1:8000/`.

### 2. Run FastAPI Server for Process Management
Open a new terminal, source ROS 2 again, and start the FastAPI server:
```sh
source ros2_ws/install/setup.bash
cd process_manager
uvicorn main:app --reload --host 0.0.0.0 --port 8080
```
This will start the FastAPI service on `http://0.0.0.0:8080/`.

### 3. Run React Frontend
Open another new terminal and start the React frontend:
```sh
cd basestationproject/frontend
npm run dev
```
This will start the frontend development server on `http://localhost:5173/`.
Links to look out for -
- [/ (Dashboard)](http://localhost:5173/)
- [/arm-control (ArmControl)](http://localhost:5173/arm-control)
- [/script-manager (Process manager)](http://localhost:5173/script-manager)

### 4. Run FastAPI Server (Optional/Required for drivetrain Feedback)
Open a new terminal, source ROS 2 again, and start the FastAPI server:
```sh
source ros2_ws/install/setup.bash
cd basestationproject/fastapi
uvicorn main:app --host 127.0.0.1 --port 8080 --reload

## Notes
- Ensure each service runs in a separate terminal.
- The `--reload` flag for FastAPI allows for automatic reloading during development.
- Modify `.env` files or configuration settings as needed.

## Contributing
If you'd like to contribute to this project, please submit an issue or pull request.

## License
This project is licensed under the MIT License.


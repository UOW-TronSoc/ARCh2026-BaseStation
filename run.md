# ARCh2025 Base Station - Docker Setup

## Overview
This repository contains the Dockerized setup for the ARCh2025 Base Station, which consists of three services:
- **React Frontend** (Port: `3000`)
- **Django Backend** (Port: `8000`)
- **FastAPI Service** (Port: `8080`)

Each service runs in its own container and communicates with the others as needed.

---

## Prerequisites
Ensure you have the following installed:
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

---

## Setup & Usage
### 1. Clone the Repository
```sh
 git clone https://github.com/your-repo/ARCh2025-BaseStation.git
 cd ARCh2025-BaseStation
```

### 2. Build and Run the Containers
Run the following command to build and start the services:
```sh
  docker build -f basestationproject/ros2_ws_docker/Dockerfile -t arch2025/ros2-ws .
  docker-compose up --build
```

This will:
- Build and run the **React frontend** on port `3000`.
- Build and run the **Django backend** on port `8000`.
- Build and run the **FastAPI service** on port `8080`.

To run the services in detached mode:
```sh
 docker-compose up -d --build
```

### 3. Stopping the Containers
To stop all running services:
```sh
 docker-compose down
```

To stop individual services:
```sh
 docker stop react_frontend
 docker stop django_server
 docker stop fastapi_server
```

### 4. Restarting Containers
To restart the services:
```sh
 docker-compose restart
```

---

## Container Details
| Container Name       | Image Name                        | Exposed Port |
|---------------------|--------------------------------|-------------|
| react_frontend      | arch2025-basestation_frontend   | 3000        |
| django_server      | arch2025-basestation_django     | 8000        |
| fastapi_server     | arch2025-basestation_fastapi    | 8080        |

---

## Environment Variables
Ensure you define any required environment variables in a `.env` file before running the containers.
Example:
```env
DJANGO_SECRET_KEY=your_secret_key
FASTAPI_ENV=development
```

---

## Logs & Debugging
To check logs for any specific container:
```sh
 docker logs -f react_frontend
 docker logs -f django_server
 docker logs -f fastapi_server
```

To open an interactive shell inside a running container:
```sh
 docker exec -it django_server /bin/bash
```

---

## Additional Notes
- Make sure ports `3000`, `8000`, and `8080` are available before running the setup.
- If needed, modify `docker-compose.yml` to update configurations.
- If any dependency changes, rebuild the images using:
  ```sh
  docker-compose up --build --force-recreate
  ```

---

## Contact
For any issues, reach out via [your contact method].


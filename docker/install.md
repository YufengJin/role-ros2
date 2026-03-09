# Docker Environment Setup and Usage

This document is the single reference for building, running, and troubleshooting the role-ros2 Docker environments. All paths are relative to the role-ros2 repository root unless noted.

## Overview

The `docker/` directory provides three kinds of setups:

1. **Camera/GPU only**: ROS2 Humble + CUDA 11.8 + ZED SDK for GPU or ZED camera use.
2. **Robot only**: ROS2 Foxy + Polymetis + libfranka for Franka robot control (no GPU).
3. **Camera + robot**: Both containers run together for the full stack.

When to use which:

- Camera/GPU only → **ros2_cu118**
- Robot only → **ros2_franka_libfranka_0.14.x** or **ros2_franka_libfranka_0.18.x**
- Camera + robot → **ros2_cu118_franka_0.14.x** or **ros2_cu118_franka_0.18.x**

## Subfolders

### ros2_cu118

- **Purpose**: Single image — ROS2 Humble + CUDA 11.8 + ZED SDK (camera/GPU).
- **Base image**: `nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04`
- **Files**: `Dockerfile`, `docker-compose.yaml`, `entrypoint.sh`, `ros2_cu118.env`, `install_nvidia_toolkit.sh`

### ros2_franka_libfranka_0.14.x

- **Purpose**: Single image — ROS2 Foxy + Polymetis + libfranka 0.14.x (robot control, Ubuntu 20.04, C++14, robot system 5.7.1).
- **Files**: `Dockerfile`, `docker-compose.yaml`, `entrypoint.sh`, `polymetis_ros2.env`

### ros2_franka_libfranka_0.18.x

- **Purpose**: Single image — same as 0.14.x but libfranka 0.18.x (C++17, requires fmt, robot system 5.9.0+).
- **Files**: `Dockerfile`, `docker-compose.yaml`, `entrypoint.sh`, `polymetis_ros2.env`

### ros2_cu118_franka_0.14.x

- **Purpose**: Compose stack — runs both ros2_cu118 and ros2_polymetis (libfranka 0.14.x). Build context is the **repository root**.
- **Files**: `docker-compose.yaml` (references `docker/ros2_cu118` and `docker/ros2_franka_libfranka_0.14.x`)

### ros2_cu118_franka_0.18.x

- **Purpose**: Compose stack — runs both ros2_cu118 and ros2_polymetis (libfranka 0.18.x).
- **Files**: `docker-compose.yaml`

## Prerequisites

- **Docker**: Docker and Docker Compose (v2: `docker compose`, or v1: `docker-compose`) installed.
- **NVIDIA Container Toolkit** (optional): Required for ZED camera or GPU in ros2_cu118. See “When NVIDIA Container Toolkit is not installed” if you skip it.
- **X11** (optional): Required for GUI apps (e.g. rviz2, rqt) inside containers. See “X11 / xhost setup”.

## Building images

### Camera only (ros2_cu118)

```bash
cd docker/ros2_cu118
docker compose build
```

### Robot only (ros2_franka_libfranka_0.14.x or 0.18.x)

```bash
cd docker/ros2_franka_libfranka_0.14.x
docker compose build
```

For 0.18.x:

```bash
cd docker/ros2_franka_libfranka_0.18.x
docker compose build
```

### Full stack (camera + robot)

Compose uses the **repository root** as build context. Run from the repo root:

```bash
docker compose -f docker/ros2_cu118_franka_0.14.x/docker-compose.yaml build
```

For 0.18.x:

```bash
docker compose -f docker/ros2_cu118_franka_0.18.x/docker-compose.yaml build
```

## X11 / xhost setup

For GUI apps (e.g. rviz2, rqt) inside containers, allow the container to use the host X server:

```bash
export DISPLAY=:0
xhost +local:root
```

(or `xhost +local:` if preferred)

- Ensure an X server is running on the host (e.g. desktop session).
- For SSH, use X11 forwarding: `ssh -X user@host`.
- Containers that mount `/tmp/.X11-unix` and set `DISPLAY` need this on the host.

## Running containers (compose up)

### ros2_cu118 only

```bash
cd docker/ros2_cu118
docker compose up -d
```

Enter the container:

```bash
docker exec -it ros2_cu118_container bash
```

One-off shell (new container, removed on exit):

```bash
docker compose run --rm ros2_cu118 bash
```

Without GPU (see next section): edit `docker-compose.yaml` and comment out or remove the `runtime: nvidia` line for the service, then run `docker compose up -d`.

### ros2_polymetis only (0.14.x or 0.18.x)

```bash
cd docker/ros2_franka_libfranka_0.14.x
docker compose up -d
docker exec -it ros2_polymetis_container bash
```

One-off:

```bash
docker compose run --rm ros2_polymetis bash
```

### Full stack (ros2_cu118 + ros2_polymetis)

From the **repository root**:

```bash
docker compose -f docker/ros2_cu118_franka_0.14.x/docker-compose.yaml up -d
```

For 0.18.x:

```bash
docker compose -f docker/ros2_cu118_franka_0.18.x/docker-compose.yaml up -d
```

Enter containers:

- Camera: `docker exec -it ros2_cu118_container bash`
- Robot: `docker exec -it ros2_polymetis_container bash`

## When NVIDIA Container Toolkit is not installed

- **ros2_cu118 only**: Comment out or remove the `runtime: nvidia` line in `docker/ros2_cu118/docker-compose.yaml`, then run `docker compose up -d`. ZED camera will not work; other functionality is unchanged.

- **ros2_cu118_franka_0.14.x / 0.18.x**: The compose file sets `runtime: nvidia` for the ros2_cu118 service. Either:
  1. Install the toolkit (recommended if you use ZED), or
  2. Edit the compose file to remove or comment out `runtime: nvidia` for the ros2_cu118 service so the stack runs without GPU (ZED will not work).

To install NVIDIA Container Toolkit:

```bash
cd docker/ros2_cu118
sudo ./install_nvidia_toolkit.sh
```

Restart Docker (or the daemon) and any affected containers after installation.

## Building the workspace (inside container)

Inside the relevant container:

```bash
cd /app/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

To build only role_ros2:

```bash
colcon build --packages-select role_ros2 --symlink-install
```

The role-ros2 source is mounted as a volume; after code changes, rebuild inside the container. No need to rebuild the image.

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| ROS_DOMAIN_ID | 0 | ROS2 domain ID (must match across containers) |
| RMW_IMPLEMENTATION | rmw_fastrtps_cpp | DDS implementation |
| ROS_LOCALHOST_ONLY | 0 | Restrict to localhost (0 = allow remote) |
| ROBOT_IP | 172.17.0.2 | Franka robot IP (polymetis) |
| DISPLAY | - | X11 display for GUI containers |

See `docker/ros2_cu118/ros2_cu118.env` and `docker/ros2_franka_libfranka_0.14.x/polymetis_ros2.env` (and 0.18.x) for more variables.

## Troubleshooting

### Container fails to start: could not select device driver "nvidia" with capabilities: [[gpu]]

NVIDIA Container Toolkit is not installed or not configured.

- **With ZED/GPU**: Install the toolkit: `cd docker/ros2_cu118 && sudo ./install_nvidia_toolkit.sh`, then restart Docker and run `docker compose up -d` again.
- **Without GPU**: Remove or comment out `runtime: nvidia` in the compose file for the ros2_cu118 service, then start again.

### ros2 topic list / ros2 node list hangs or does not respond

Usually ROS2 daemon or DDS configuration.

- Restart the container: `docker compose restart <service_name>` (from the directory of the compose file you used).
- Check environment: `docker exec <container> bash -c "echo \$ROS_DOMAIN_ID \$RMW_IMPLEMENTATION"`. RMW should be `rmw_fastrtps_cpp`.

### GUI: cannot connect to X server

- On the host, ensure `echo $DISPLAY` shows a value (e.g. `:0`).
- Run `xhost +local:root` (or `xhost +local:`) on the host.
- Ensure the container has `/tmp/.X11-unix` mounted and `DISPLAY` set (compose files that use GUI typically do this).
- Over SSH, use `ssh -X` or `ssh -Y`.

### docker-compose command not found

Use the v2 plugin: `docker compose` (space, not hyphen). If only v1 is installed: `docker-compose`. Install Docker Compose if neither works.

### colcon build warning: The path '/app/ros2_ws/install/xxx' doesn't exist

Common after removing `install/`. Usually safe to ignore; colcon will recreate it. To clean up: open a new shell in the container or run `unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH`, then re-source the ROS2 and workspace setup and run `colcon build` again.

### robotpkg / pinocchio build fails (during image build)

Dockerfiles pin robotpkg versions (e.g. pinocchio, casadi, eigenpy). If the build still fails, check network access and build logs; see Dockerfile comments for pinned versions and dependency order.

### Two containers in the full stack cannot communicate

- Ensure both use the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION` (e.g. `rmw_fastrtps_cpp`).
- Compose files use `network_mode: host` so both share the host network.

### Link errors when building image (e.g. undefined reference to cuvidDestroyVideoParser)

NVIDIA driver libraries are resolved at runtime. The Dockerfile uses flags so these symbols are allowed to be unresolved during build. If the image runs and the host has the correct NVIDIA driver and nvidia-container-toolkit, this is expected. If the container fails at runtime, check the host driver and toolkit.

## References

- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [ZED SDK](https://www.stereolabs.com/docs/)
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/)

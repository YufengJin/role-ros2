# Docker Setup for ROS2 Foxy with Polymetis

This directory contains Docker configuration files for running ROS2 Foxy with Polymetis robot control.

## Quick Start

```bash
# Build
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose build ros2_polymetis

# Run
docker compose run --rm ros2_polymetis bash
```

## Features

- **ROS2 Foxy** with CycloneDDS
- **Polymetis** robot control framework (built from source)
- **libfranka 0.14.1** for Franka robot support
- **Micromamba** isolated Python 3.8 environment

## Files

| File | Description |
|------|-------------|
| `Dockerfile.ros2_polymetis` | Docker image definition |
| `docker-compose.yaml` | Docker Compose configuration |
| `entrypoint.sh` | Container entrypoint script |
| `cyclonedds.xml` | CycloneDDS configuration |
| `fastrtps_profile.xml` | FastRTPS fallback configuration |
| `KNOWN_ISSUES.md` | Known issues and solutions |

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `172.17.0.2` | Franka robot IP |
| `POLYMETIS_IP` | `127.0.0.1` | Polymetis gRPC server IP |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS implementation |

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `LIBFRANKA_VERSION` | `0.14.1` | libfranka version |
| `ROBOT_TYPE` | `fr3` | Robot type (fr3 or panda) |

## Cross-Container Communication

For ROS2 communication between container (Foxy) and host (Humble), see [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## Container Structure

```
/app/
├── ros2_ws/
│   ├── src/role-ros2/          # role_ros2 package
│   └── install/                 # Built ROS2 packages
/opt/
├── ros/foxy/                    # ROS2 Foxy installation
└── conda/                       # Micromamba installation
```

## Troubleshooting

```bash
# Check ROS2
ros2 --version

# Check Polymetis
python -c "from polymetis import RobotInterface; print('OK')"

# Test ROS2 publishing
python /app/ros2_ws/src/role-ros2/scripts/test_publish.py
```

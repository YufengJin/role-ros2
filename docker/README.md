# Docker Setup for ROS2 Foxy with Polymetis

This directory contains Docker configuration files for running ROS2 Foxy with Polymetis robot control in a containerized environment.

## ⚠️ Known Issues

**Important**: There are currently two unresolved issues that prevent full functionality:

1. **Polymetis-Local and ROS2 Environment Incompatibility**: The `polymetis_bridge.py` node cannot run due to spdlog symbol conflicts between conda and system environments.

2. **Cross-Container ROS2 Communication Problems**: While topics can be discovered, messages are not being received between container and host.

See [KNOWN_ISSUES.md](KNOWN_ISSUES.md) for detailed analysis and root causes.

## Overview

The Docker setup provides:
- **ROS2 Foxy** - Robot Operating System 2 (Foxy Fitzroy distribution)
- **Python 3.8** - Required Python version for Polymetis
- **Polymetis** - Robot control framework (built from source)
- **libfranka 0.14.1** - Franka Emika robot library
- **Micromamba Environment** - Isolated Python environment with all dependencies

## Files

- `Dockerfile.ros2_polymetis` - Main Docker image definition
- `docker-compose.yaml` - Docker Compose configuration
- `entrypoint.sh` - Container entrypoint script
- `fastrtps_profile.xml` - FastRTPS DDS configuration for network discovery
- `README.md` - This file
- `README_CROSS_CONTAINER.md` - Cross-container ROS2 communication setup guide
- `KNOWN_ISSUES.md` - Known issues and root cause analysis

## Prerequisites

- Docker and Docker Compose installed
- Access to the Franka robot network (if connecting to hardware)
- Sufficient disk space (~10GB for the image)

## Building the Image

### Using Docker Compose (Recommended)

```bash
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose -f docker-compose.yaml build ros2_polymetis
```

Or using the default filename (docker-compose.yaml):
```bash
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose build ros2_polymetis
```

### Using Docker Directly

```bash
cd /path/to/ros2_ws
docker build \
  -f src/role-ros2/docker/Dockerfile.ros2_polymetis \
  --build-arg LIBFRANKA_VERSION=0.14.1 \
  --build-arg ROBOT_TYPE=fr3 \
  --build-arg ROBOT_IP=172.17.0.2 \
  --build-arg NUC_IP=172.17.0.1 \
  -t ros2_polymetis:latest \
  .
```

## Running the Container

### Using Docker Compose

```bash
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose -f docker-compose.yaml up -d ros2_polymetis
```

To run interactively:
```bash
docker compose -f docker-compose.yaml run --rm ros2_polymetis bash
```

Or using the default filename:
```bash
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose up -d ros2_polymetis
docker compose run --rm ros2_polymetis bash
```

### Using Docker Directly

```bash
docker run -it --rm \
  --network host \
  -e ROBOT_IP=172.17.0.2 \
  -e POLYMETIS_IP=127.0.0.1 \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=${DISPLAY} \
  ros2_polymetis:latest \
  bash
```

## Configuration

### Environment Variables

The following environment variables can be configured in `docker-compose.yaml`:

- `ROBOT_IP` - IP address of the Franka robot (default: `172.17.0.2`)
- `POLYMETIS_IP` - IP address for Polymetis gRPC server (default: `127.0.0.1`)
- `ROS_DOMAIN_ID` - ROS2 domain ID for DDS communication (default: `0`)
- `RMW_IMPLEMENTATION` - ROS2 middleware implementation (default: `rmw_fastrtps_cpp`)

### Build Arguments

When building the image, you can customize:

- `LIBFRANKA_VERSION` - Version of libfranka to build (default: `0.14.1`)
- `ROBOT_TYPE` - Robot type: `fr3` or `panda` (default: `fr3`)
- `ROBOT_IP` - Robot IP address (default: `172.17.0.2`)
- `NUC_IP` - NUC IP address (default: `172.17.0.1`)

## Usage Examples

### Launch Polymetis Server

```bash
# Inside the container
conda activate polymetis-local
python -m polymetis.scripts.launch_server
```

### Run ROS2 Nodes

```bash
# Inside the container
source /opt/ros/foxy/setup.bash
source /app/ros2_ws/install/setup.bash

# Run polymetis manager node
ros2 run role_ros2 polymetis_manager
```

### Interactive Development

```bash
# Start interactive container
docker compose -f docker-compose.yaml run --rm ros2_polymetis bash

# All ROS2 and Polymetis tools are available
ros2 --help
python -c "import polymetis; print(polymetis.__version__)"
```

## Network Configuration

The container uses `network_mode: host` to allow direct communication with the robot. Ensure:

1. The host machine can reach the robot network
2. ROS2 DDS settings match between container and host
3. Firewall rules allow necessary ports

## Real-time Scheduling (Optional)

For better robot control performance, you can enable real-time scheduling by uncommenting in `docker-compose.yaml`:

```yaml
privileged: true
# OR
cap_add:
  - SYS_NICE
ulimits:
  rtprio: 99
  memlock: -1
```

**Warning**: Using `privileged: true` grants extensive permissions. Use with caution.

## Troubleshooting

### Build Issues

1. **Out of disk space**: Clean up unused Docker images
   ```bash
   docker system prune -a
   ```

2. **CMake errors**: Ensure all dependencies are installed in the base image

3. **Conda environment creation fails**: Check internet connectivity and conda channels

### Runtime Issues

1. **Cannot connect to robot**: 
   - Verify `ROBOT_IP` is correct
   - Check network connectivity: `ping ${ROBOT_IP}`
   - Ensure robot is powered on and FCI is running

2. **ROS2 nodes not communicating**:
   - Verify `ROS_DOMAIN_ID` matches between containers/host
   - Check `RMW_IMPLEMENTATION` is consistent
   - Ensure firewall allows DDS traffic

3. **Polymetis import errors**:
   - Activate conda environment: `conda activate polymetis-local`
   - Verify installation: `python -c "import polymetis"`

### Common Commands

```bash
# Check ROS2 installation
ros2 --version

# List available ROS2 packages
ros2 pkg list

# Check Polymetis installation
conda activate polymetis-local
python -c "import polymetis; print(polymetis.__version__)"

# View container logs
docker compose -f docker-compose.yaml logs ros2_polymetis

# Restart container
docker compose -f docker-compose.yaml restart ros2_polymetis
```

## Directory Structure

**Docker directory:**
```
docker/
├── Dockerfile.ros2_polymetis    # Main Docker image definition
├── docker-compose.yaml          # Docker Compose configuration
├── entrypoint.sh                 # Container entrypoint script
├── fastrtps_profile.xml          # FastRTPS DDS configuration
├── README.md                     # This file
├── README_CROSS_CONTAINER.md     # Cross-container communication guide
└── KNOWN_ISSUES.md               # Known issues and root cause analysis
```

**Inside the container:**
- `/app` - Project root directory
- `/app/ros2_ws` - ROS2 workspace
- `/app/ros2_ws/src/role-ros2` - role_ros2 package
- `/app/ros2_ws/src/role-ros2/role_ros2/fairo/polymetis` - Polymetis source code
- `/opt/ros/foxy` - ROS2 Foxy installation
- `/opt/conda` - Micromamba/Conda installation

## Notes

- The image is based on Ubuntu 20.04 (Focal), required for ROS2 Foxy
- Python 3.8 is used throughout (required by Polymetis)
- Polymetis is built from source to ensure compatibility
- libfranka 0.14.1 is built from source with the specified version
- The micromamba environment `polymetis-local` contains all Python dependencies
- **Known Issues**: See [KNOWN_ISSUES.md](KNOWN_ISSUES.md) for unresolved problems

## References

- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [Polymetis Documentation](https://facebookresearch.github.io/fairo/polymetis/)
- [libfranka Documentation](https://frankaemika.github.io/docs/)


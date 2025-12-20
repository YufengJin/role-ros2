# Known Issues - Docker Container for ROS2 + Polymetis

## Overview

This document summarizes the current status of the Docker container setup for ROS2 Foxy and Polymetis integration.

---

## Issue 1: Polymetis-Local and ROS2 Environment Incompatibility - RESOLVED ✓

### Problem

The `polymetis_bridge.py` node requires both Polymetis SDK and ROS2 (rclpy) to work simultaneously. Previously, they had library conflicts due to both using `spdlog`.

### Solution

Resolved by:

1. **Removing spdlog from conda environment** in Dockerfile:
   ```dockerfile
   RUN rm -f /opt/conda/envs/polymetis-local/lib/libspdlog* && \
       rm -rf /opt/conda/envs/polymetis-local/include/spdlog && \
       rm -rf /opt/conda/envs/polymetis-local/lib/cmake/spdlog && \
       rm -rf /opt/conda/envs/polymetis-local/lib/pkgconfig/spdlog.pc
   ```

2. **Configuring Polymetis to use system spdlog**:
   ```dockerfile
   cmake .. -Dspdlog_DIR=/usr/lib/x86_64-linux-gnu/cmake/spdlog
   ```

3. **Adding git safe directory** in entrypoint.sh for version detection

4. **Disabling bind mount** in docker-compose.yaml to preserve build artifacts

**Status**: ✅ Resolved - Polymetis and ROS2 can now be imported together.

---

## Issue 2: Cross-Container ROS2 Communication

### Problem

ROS2 topics may not be discoverable between container (ROS2 Foxy) and host system (ROS2 Humble) due to different DDS implementations and versions.

### Current Configuration

The container uses CycloneDDS for better cross-version compatibility:

- **Container**: ROS2 Foxy with `rmw_cyclonedds_cpp`
- **Host**: ROS2 Humble (requires matching CycloneDDS configuration)

### Host System Setup (Required)

To enable cross-version communication, configure the host system:

```bash
# Install CycloneDDS
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Create config file
mkdir -p ~/.ros
cat > ~/.ros/cyclonedds.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
    <Domain id="any">
        <General>
            <AllowMulticast>true</AllowMulticast>
            <EnableMulticastLoopback>true</EnableMulticastLoopback>
        </General>
    </Domain>
</CycloneDDS>
EOF

# Add to ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export CYCLONEDDS_URI=file://$HOME/.ros/cyclonedds.xml' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=0' >> ~/.bashrc
source ~/.bashrc
```

### Testing Communication

**Terminal 1 (Container - Publish)**:
```bash
cd /home/yjin/repos/ros2_ws/src/role-ros2/docker
docker compose run --rm ros2_polymetis bash
# Inside container:
python3 /app/ros2_ws/src/role-ros2/scripts/test_publish.py
```

**Terminal 2 (Host - Subscribe)**:
```bash
ros2 topic echo /test_topic
```

**Status**: 🔧 Configuration provided - requires host setup and testing.

---

## Related Files

| File | Description |
|------|-------------|
| `Dockerfile.ros2_polymetis` | Docker image build (spdlog fix applied) |
| `docker-compose.yaml` | Container configuration |
| `entrypoint.sh` | Environment setup |
| `cyclonedds.xml` | CycloneDDS configuration |
| `fastrtps_profile.xml` | FastRTPS fallback configuration |
| `polymetis_bridge.py` | Main ROS2-Polymetis bridge node |

---

**Last Updated**: 2025-12-20

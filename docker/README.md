# Docker Setup for ROS2 Foxy with Polymetis

This directory contains Docker configuration files for running ROS2 Foxy with Polymetis robot control.

## Quick Start

### First Time Setup

```bash
# 1. Build Docker image (one-time, installs all dependencies)
cd /path/to/ros2_ws/src/role-ros2/docker
docker compose build ros2_polymetis

# 2. Enter container (auto-configures environment)
docker compose run --rm ros2_polymetis bash

# 3. Build ROS2 workspace FIRST (inside container)
#    This generates ROS2 messages and services
cd /app/ros2_ws
colcon build --symlink-install

# 4. Source ROS2 workspace (makes generated messages available)
source install/setup.bash

# 5. Source conda environment (if not already sourced by entrypoint)
#    The entrypoint.sh should handle this, but you can manually source if needed:
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

**Note**: The entrypoint script (`entrypoint.sh`) automatically sources the environment, but if you skip it with `SKIP_AUTO_ENV=true`, you must manually source in the correct order.

### Rebuilding ROS2 Workspace (Recommended for Multiple Rebuilds)

**Important**: Always build the workspace BEFORE sourcing the conda environment.

```bash
# 1. Enter container (skip auto-configuration for faster startup)
SKIP_AUTO_ENV=true docker compose run --rm ros2_polymetis bash

# 2. Build workspace FIRST (generates ROS2 messages/services)
cd /app/ros2_ws
colcon build --symlink-install

# 3. Source ROS2 workspace (makes generated messages available)
source install/setup.bash

# 4. Source conda environment (activates polymetis-local and sets library paths)
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

**Why this order matters:**
- `colcon build` generates Python modules for ROS2 messages (`role_ros2.msg`, `role_ros2.srv`)
- `source install/setup.bash` makes these generated modules available to Python
- `source polymetis_ros2.env` activates conda environment and sets library paths
- If you source conda first, Python might find conda packages before ROS2-generated ones, causing import errors

### Quick Commands

```bash
# Enter container with auto-configuration
docker compose run --rm ros2_polymetis bash

# Enter container without auto-configuration (manual setup)
SKIP_AUTO_ENV=true docker compose run --rm ros2_polymetis bash
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

**Note**: The ROS2 workspace is NOT built during docker build. This allows you to rebuild it multiple times without rebuilding the docker image.

## Features

- **ROS2 Foxy** with CycloneDDS
- **Polymetis** robot control framework (built from source)
- **libfranka 0.14.1** for Franka robot support
- **Micromamba** isolated Python 3.8 environment
- **✅ Resolved**: Polymetis and ROS2 compatibility (spdlog conflict fixed)

## Files

| File | Description |
|------|-------------|
| `Dockerfile.ros2_polymetis` | Docker image definition |
| `docker-compose.yaml` | Docker Compose configuration |
| `entrypoint.sh` | Container entrypoint script |
| `cyclonedds.xml` | CycloneDDS configuration |
| `fastrtps_profile.xml` | FastRTPS fallback configuration |
| `polymetis_ros2.env` | Environment setup script (source this to configure environment) |

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOT_IP` | `172.17.0.2` | Franka robot IP |
| `POLYMETIS_IP` | `127.0.0.1` | Polymetis gRPC server IP |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS implementation |

You can override these before sourcing the environment script:

```bash
export ROBOT_IP=192.168.1.100
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

### Build Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `LIBFRANKA_VERSION` | `0.14.1` | libfranka version |
| `ROBOT_TYPE` | `fr3` | Robot type (fr3 or panda) |

## Environment Setup

### Environment Configuration Script

The `polymetis_ros2.env` script configures:

- **Git configuration**: Sets safe directory (required for Polymetis version detection)
- **ROS2 Foxy**: Sources `/opt/ros/foxy/setup.bash`
- **ROS2 workspace**: Sources `/app/ros2_ws/install/setup.bash` (if built)
- **Micromamba environment**: Activates `polymetis-local` conda environment
- **Library paths**: Configures LD_LIBRARY_PATH (resolves spdlog conflicts)
- **Build paths**: Configures CMAKE_PREFIX_PATH and PKG_CONFIG_PATH
- **ROS2 DDS**: Configures CycloneDDS or FastRTPS
- **Environment variables**: ROBOT_IP, POLYMETIS_IP, ROS_DOMAIN_ID, etc.

### ⚠️ Important: Environment Configuration Order

**For Debug/Testing Mode, always follow this order:**

1. **Build workspace FIRST** → `colcon build --symlink-install`
2. **Source ROS2 workspace** → `source install/setup.bash`
3. **Source conda environment** → `source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env`

**Why this order matters:**
- `colcon build` generates Python modules for ROS2 messages/services (`role_ros2.msg`, `role_ros2.srv`)
- `source install/setup.bash` makes these generated modules available to Python (sets PYTHONPATH)
- `source polymetis_ros2.env` activates conda environment and sets library paths
- **If you source conda first**, Python might find conda packages before ROS2-generated ones, causing:
  - `ImportError: No module named 'role_ros2.msg'`
  - `ImportError: No module named 'role_ros2.srv'`

**Note**: The `entrypoint.sh` script sources workspace first (if it exists), then activates conda. This is correct, but you must build the workspace before entering the container for the first time.

### Manual Environment Configuration

For rebuilding ros2_ws multiple times (recommended):

**Correct Order** (build first, then source):

```bash
# Enter container without auto-configuration
SKIP_AUTO_ENV=true docker compose run --rm ros2_polymetis bash

# 1. Build ros2_ws FIRST (generates messages/services)
cd /app/ros2_ws
colcon build --symlink-install

# 2. Source ROS2 workspace (makes generated messages available)
source install/setup.bash

# 3. Source conda environment (activates polymetis-local)
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

**Why this order?**
- Building first ensures ROS2 messages/services are generated before Python tries to import them
- Sourcing ROS2 workspace makes generated modules available to Python
- Sourcing conda environment provides runtime dependencies (Polymetis, PyTorch) and sets library paths
- If you source conda first, Python might find conda packages before ROS2-generated ones, causing `ImportError: No module named 'role_ros2.msg'`

**Tip**: Add `source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env` to `~/.bashrc` for automatic configuration.

## Common Workflows

### Scenario 1: First Time Use

```bash
# 1. Build image
docker compose build ros2_polymetis

# 2. Enter container (auto-configuration)
docker compose run --rm ros2_polymetis bash

# 3. Build workspace FIRST (generates messages/services)
cd /app/ros2_ws
colcon build --symlink-install

# 4. Source ROS2 workspace (makes generated messages available)
source install/setup.bash

# 5. Source conda environment (if not already sourced by entrypoint)
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

**Note**: The entrypoint script automatically sources the environment, but after building, you may need to re-source to ensure everything is up to date.

### Scenario 2: Rebuild After Code Changes

```bash
# 1. Enter container (manual configuration, faster)
SKIP_AUTO_ENV=true docker compose run --rm ros2_polymetis bash

# 2. Build FIRST (generates messages/services)
cd /app/ros2_ws
colcon build --symlink-install

# 3. Source ROS2 workspace (makes generated messages available)
source install/setup.bash

# 4. Source conda environment (activates polymetis-local)
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

**Important**: Always build before sourcing conda environment to avoid import errors.

### Scenario 3: Clean Rebuild

```bash
# Inside container
cd /app/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## Cross-Container Communication

The container uses **CycloneDDS** for cross-version compatibility between ROS2 Foxy (container) and ROS2 Humble (host).

### Host System Setup (Required for Cross-Container Communication)

To enable communication between container and host:

```bash
# Install CycloneDDS on host
sudo apt install ros-humble-rmw-cyclonedds-cpp

# Create CycloneDDS config
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

## Important Notes

### ✅ Resolved: Polymetis-ROS2 Compatibility

The container has been configured to resolve library conflicts between Polymetis and ROS2:

- **spdlog conflict resolved**: Conda environment's spdlog removed, Polymetis uses system spdlog
- **Both can be imported**: Polymetis SDK and ROS2 (rclpy) work simultaneously
- **Verified**: `polymetis_bridge.py` can import both without conflicts

**Solution applied**:
- Removed spdlog from conda environment in Dockerfile
- Configured Polymetis to use system spdlog via CMake
- Added git safe directory configuration for version detection

### Python Package Installation

- **role_ros2** is installed to ROS2 install directory: `/app/ros2_ws/install/role_ros2/lib/python3.8/site-packages`
- All modules including `robot_ik` are properly installed via `colcon build`
- Conda environment (`polymetis-local`) provides Python dependencies (dm-control, dm-robotics, etc.)

## Verification

Run these commands to verify the environment is correctly configured:

```bash
# Check ROS2
ros2 --version
ros2 pkg list | grep role_ros2

# Check Polymetis
python -c "from polymetis import RobotInterface; print('✅ Polymetis OK')"

# Check ROS2 + Polymetis compatibility (should work without errors)
python -c "import rclpy; from polymetis import RobotInterface; print('✅ Both OK')"

# Verify role_ros2 installation
python -c "from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver; print('✅ robot_ik OK')"

# Check environment variables
echo "ROBOT_IP: $ROBOT_IP"
echo "CONDA_PREFIX: $CONDA_PREFIX"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
```

## Troubleshooting

### Problem: ros2 command not found

**Solution**: Ensure ROS2 environment is sourced
```bash
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

### Problem: role_ros2 module not found

**Solution**: Ensure workspace is built and sourced
```bash
cd /app/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Problem: ImportError for new messages/services (e.g., GripperCommand, ControllerStatus)

**Symptom:**
```
ImportError: cannot import name 'GripperCommand' from 'role_ros2.msg'
[ERROR] [polymetis_bridge-1]: process has died [pid XXX, exit code -11]
```

**Cause**: New `.msg` or `.srv` files were added but workspace was not rebuilt.

**Solution**: Rebuild workspace to generate new message/service modules
```bash
cd /app/ros2_ws
colcon build --symlink-install
source install/setup.bash
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env

# Verify imports work
python3 -c "from role_ros2.msg import GripperCommand, ControllerStatus; print('✓ Messages imported successfully')"
```

### Problem: URDF file not found

**Symptom:**
```
FileNotFoundError: URDF file not found: .../fr3.urdf
```

**Solution**: 
- **In mock mode**: The launch file will automatically use an empty robot description
- **In real robot mode**: Ensure URDF file exists at `/app/ros2_ws/src/role-ros2/role_ros2/robot_ik/franka/fr3.urdf`
- Check if file exists: `ls /app/ros2_ws/src/role-ros2/role_ros2/robot_ik/franka/fr3.urdf`
- If missing, the launch file will try to generate it from XML automatically

### Problem: Conda environment not activated

**Solution**: Manually activate
```bash
eval "$(micromamba shell hook --shell bash)"
micromamba activate polymetis-local
```

### Problem: Library path conflicts

**Solution**: Re-source environment script
```bash
source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
```

### Problem: Cross-container ROS2 communication not working

**Solution**: 
- Ensure host system has CycloneDDS installed and configured (see Cross-Container Communication section)
- Verify both container and host use the same ROS_DOMAIN_ID
- Check that ROS_LOCALHOST_ONLY=0 on both sides

### Problem: polymetis_bridge process crashes (exit code -11, SIGSEGV)

**Symptom:**
```
[ERROR] [polymetis_bridge-1]: process has died [pid XXX, exit code -11]
```

**Causes and Solutions**:
1. **New messages not compiled**: Rebuild workspace (see "ImportError for new messages/services" above)
2. **Library version mismatch**: Ensure conda environment is activated and library paths are correct
   ```bash
   source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env
   echo $CONDA_PREFIX  # Should show /opt/conda/envs/polymetis-local
   ```
3. **Memory issue**: Check system resources and restart container if needed

## Tips

- Use `--symlink-install` for faster builds (symbolic links instead of copying)
- If only Python code is modified, you can reinstall the package: `pip install -e /app/ros2_ws/src/role-ros2/role_ros2`
- Add `source /app/ros2_ws/src/role-ros2/docker/polymetis_ros2.env` to `~/.bashrc` for automatic environment configuration

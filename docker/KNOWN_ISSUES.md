# Known Issues - Docker Container for ROS2 + Polymetis

## Overview

This document summarizes the current known issues with the Docker container setup for ROS2 Foxy and Polymetis integration. These issues prevent full functionality but have been documented with root cause analysis.

## Issue 1: Polymetis-Local and ROS2 Environment Incompatibility

### Problem Description

The `polymetis_bridge.py` node requires both Polymetis SDK and ROS2 (rclpy) to work simultaneously, but they are installed in incompatible environments:

- **Polymetis**: Installed in `micromamba` conda environment (`polymetis-local`)
- **ROS2**: Installed in system environment, uses system Python (`/usr/bin/python3`)

### Root Cause Analysis

1. **Library Symbol Conflicts**:
   - Both Polymetis (via conda) and ROS2 (via system) depend on `spdlog` library
   - Conda's `spdlog` and system's `spdlog` have different symbol versions
   - When `polymetis_bridge.py` tries to import both `polymetis` (from conda) and `rclpy` (from ROS2), it causes symbol conflicts:
     ```
     ImportError: /opt/ros/foxy/lib/librcl_logging_spdlog.so: undefined symbol: 
     _ZN6spdlog5sinks15basic_file_sinkISt5mutexEC1ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEb
     ```

2. **Python Interpreter Mismatch**:
   - Polymetis requires conda Python (with PyTorch, etc.)
   - ROS2 rclpy works best with system Python
   - `polymetis_bridge.py` needs to run in conda environment to access Polymetis, but ROS2 nodes expect system Python

3. **Current Workaround Limitations**:
   - Current `entrypoint.sh` sets `ROS2_PYTHON=/usr/bin/python3` to force ROS2 to use system Python
   - However, when running `polymetis_bridge.py`, it must use conda Python to import Polymetis
   - This creates a catch-22 situation: cannot use both simultaneously

### Impact

- **Cannot run `polymetis_bridge.py` in the container**: The node fails to start due to spdlog symbol conflicts
- **Workaround**: Only simple ROS2 nodes (like `test_publish.py`) can run using system Python, but they cannot access Polymetis

### Potential Solutions (Not Implemented)

1. **Rebuild Polymetis with System Libraries**:
   - Build Polymetis from source using system Python and system libraries
   - This would eliminate conda dependency but requires significant build configuration changes

2. **Use ROS2 from Conda**:
   - Install ROS2 Foxy in the conda environment
   - This is complex and may not be officially supported

3. **Separate Processes**:
   - Run Polymetis interface in one process (conda Python)
   - Run ROS2 bridge in another process (system Python)
   - Use inter-process communication (e.g., shared memory, sockets)
   - This adds complexity and latency

4. **Docker Multi-Stage Build**:
   - Create separate containers for Polymetis and ROS2
   - Use ROS2 bridge between containers
   - This requires network communication and adds latency

5. **Static Linking**:
   - Statically link spdlog in Polymetis or ROS2
   - This may require rebuilding both from source

### Technical Details

**Current Environment Setup** (from `entrypoint.sh`):
```bash
# Activate conda environment (for Polymetis)
micromamba activate polymetis-local

# Set ROS2 to use system Python (to avoid spdlog conflicts)
export ROS2_PYTHON=/usr/bin/python3
export RCUTILS_LOGGING_SEVERITY=ERROR
```

**Problematic Code Path** (from `polymetis_bridge.py`):
```python
# This requires conda Python (for Polymetis)
from polymetis import RobotInterface, GripperInterface

# This requires system Python (for ROS2, to avoid spdlog conflicts)
import rclpy
from rclpy.node import Node
```

**Conflict Point**: Both imports happen in the same Python process, causing spdlog symbol conflicts.

---

## Issue 2: Cross-Container ROS2 Communication Problems

### Problem Description

While ROS2 topics can be discovered between container (ROS2 Foxy) and host system (ROS2 Humble), messages are not being received:

- **Topic Discovery**: ✅ Works - `/test_topic` appears in `ros2 topic list` on both sides
- **Message Reception**: ❌ Fails - `ros2 topic echo /test_topic` shows no messages

### Root Cause Analysis

1. **DDS Discovery vs. Data Transmission**:
   - FastRTPS DDS discovery mechanism works (topics are visible)
   - However, actual data transmission may be failing due to:
     - Network interface binding issues
     - Multicast configuration problems
     - Port mapping issues despite `network_mode: host`

2. **Publisher Process Status**:
   - Publisher may be starting but not actually publishing messages
   - Container logs may show Publisher started, but messages not reaching network layer
   - Possible causes:
     - Publisher process crashes silently after initialization
     - Publisher is blocked or waiting for something
     - Publisher is publishing but messages are not serialized/transmitted correctly

3. **FastRTPS Configuration**:
   - `fastrtps_profile.xml` is configured but may not be loaded correctly
   - Multicast discovery works, but unicast data transmission may fail
   - Network interface selection may be incorrect

4. **ROS2 Daemon State**:
   - ROS2 daemon may be in inconsistent state
   - Daemon restart may not fully clear previous state
   - Multiple daemon instances may be running

5. **Version Compatibility**:
   - ROS2 Foxy (container) and ROS2 Humble (host) use different FastRTPS versions
   - While discovery protocol is compatible, data serialization/deserialization may have issues
   - Message type compatibility between versions

### Impact

- **Cannot receive messages from container**: Topics are visible but no data flows
- **One-way communication only**: Discovery works, but actual message transmission fails
- **Testing is limited**: Can verify topic discovery but cannot test actual data flow

### Observed Behavior

1. **Topic List Shows Topic**:
   ```bash
   $ ros2 topic list
   /parameter_events
   /rosout
   /test_topic  # ✅ Visible
   ```

2. **Topic Info Shows Publisher**:
   ```bash
   $ ros2 topic info /test_topic
   Type: std_msgs/msg/String
   Publisher count: 1  # ✅ Publisher detected
   Subscription count: 0
   ```

3. **Topic Echo Shows Nothing**:
   ```bash
   $ ros2 topic echo /test_topic
   # ❌ No output, hangs or times out
   ```

4. **Topic Hz Shows No Rate**:
   ```bash
   $ ros2 topic hz /test_topic
   # ❌ No rate information
   ```

### Potential Solutions (Not Implemented)

1. **Debug FastRTPS Network Layer**:
   - Enable FastRTPS verbose logging
   - Check network packet capture (tcpdump/wireshark)
   - Verify multicast/unicast traffic

2. **Verify Publisher Process**:
   - Add more logging to `test_publish.py`
   - Verify publisher is actually calling `publish()`
   - Check if messages are being serialized

3. **Network Interface Configuration**:
   - Explicitly bind FastRTPS to specific network interface
   - Verify `network_mode: host` is working correctly
   - Check firewall rules for DDS ports (7400-7500)

4. **Message Type Verification**:
   - Verify message serialization format compatibility
   - Check if Foxy and Humble use same message encoding
   - Test with simpler message types

5. **Alternative DDS Implementation**:
   - Try CycloneDDS instead of FastRTPS
   - However, this requires both sides to use same DDS implementation

6. **Direct Network Testing**:
   - Test raw UDP communication between container and host
   - Verify network connectivity at lower level
   - Check if Docker networking is interfering

### Technical Details

**Current Network Configuration**:
- Container: `network_mode: host` (should share host network stack)
- DDS: FastRTPS with multicast discovery
- Ports: UDP 7400-7500

**FastRTPS Profile** (`fastrtps_profile.xml`):
- Discovery protocol: SIMPLE
- Multicast enabled
- Port base: 7400

**Environment Variables**:
```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
```

**Expected vs. Actual**:
- Expected: Messages flow from container Publisher to host Subscriber
- Actual: Topics are discovered but messages do not flow

---

## Summary

### Issue 1: Environment Incompatibility
- **Status**: Blocking - Prevents `polymetis_bridge.py` from running
- **Severity**: High - Core functionality unavailable
- **Workaround**: None - Cannot use Polymetis and ROS2 together in same process

### Issue 2: Communication Failure
- **Status**: Partial - Discovery works but data transmission fails
- **Severity**: Medium - Can test topic discovery but not data flow
- **Workaround**: None - Cannot receive messages from container

### Recommendations

1. **For Issue 1**: Consider architectural changes:
   - Separate Polymetis and ROS2 into different processes/containers
   - Use inter-process communication
   - Or rebuild Polymetis with system libraries

2. **For Issue 2**: Deep debugging needed:
   - Network packet analysis
   - FastRTPS verbose logging
   - Publisher process debugging
   - Message serialization verification

3. **Alternative Approach**: 
   - Consider using ROS2 Humble in container instead of Foxy
   - This may improve compatibility with host system
   - But requires rebuilding Docker image

---

## Related Files

- `entrypoint.sh`: Environment setup and spdlog conflict workarounds
- `polymetis_bridge.py`: Node that requires both Polymetis and ROS2
- `fastrtps_profile.xml`: DDS network configuration
- `docker-compose.yaml`: Container network configuration
- `test_publish.py`: Simple test publisher (works with system Python only)

---

**Last Updated**: 2025-12-19  
**Status**: Issues documented, solutions not implemented


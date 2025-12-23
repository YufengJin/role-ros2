#!/usr/bin/env python3
"""
Polymetis Dependencies Test

This script tests all dependencies required by:
1. polymetis_bridge_node.py - ROS2 node that bridges Polymetis with ROS2
2. franka_robot.launch.py - Launch file for Franka robot

Tests include:
- PyTorch and Polymetis imports
- ROS2 core imports
- role_ros2 custom messages and services
- IK solver and transformations
- Configuration files
- URDF files
- Launch file dependencies
- Executables availability
- Environment setup

Author: Role-ROS2 Team
"""

import sys
import os
import subprocess
import yaml
from pathlib import Path
from typing import Optional, Tuple, List

# Colors for output
GREEN = '\033[0;32m'
RED = '\033[0;31m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
CYAN = '\033[0;36m'
NC = '\033[0m'  # No Color

def print_header(text):
    print(f"\n{CYAN}{'='*70}{NC}")
    print(f"{CYAN}{text}{NC}")
    print(f"{CYAN}{'='*70}{NC}")

def print_success(text):
    print(f"{GREEN}✓ {text}{NC}")

def print_error(text):
    print(f"{RED}✗ {text}{NC}")

def print_warning(text):
    print(f"{YELLOW}⚠ {text}{NC}")

def print_info(text):
    print(f"{BLUE}ℹ {text}{NC}")


def test_import(module_name: str, description: Optional[str] = None, required: bool = True) -> bool:
    """Test if a module can be imported"""
    try:
        module = __import__(module_name)
        desc = description or module_name
        print_success(f"{desc} imported successfully")
        
        # Try to get version if available
        if hasattr(module, '__version__'):
            print_info(f"  Version: {module.__version__}")
        
        return True
    except ImportError as e:
        desc = description or module_name
        if required:
            print_error(f"{desc} import failed: {e}")
        else:
            print_warning(f"{desc} import failed: {e} (optional)")
        return False
    except Exception as e:
        desc = description or module_name
        if required:
            print_error(f"{desc} import error: {e}")
        else:
            print_warning(f"{desc} import error: {e} (optional)")
        return False


def test_pytorch_imports() -> bool:
    """Test PyTorch imports (required for Polymetis)"""
    print_header("1. PyTorch Dependencies")
    
    results = []
    
    # Test torch
    try:
        import torch
        print_success("PyTorch (torch) imported successfully")
        print_info(f"  Version: {torch.__version__}")
        print_info(f"  CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print_info(f"  CUDA version: {torch.version.cuda}")
            print_info(f"  GPU count: {torch.cuda.device_count()}")
        results.append(True)
    except ImportError as e:
        print_error(f"PyTorch not available: {e}")
        print_warning("  PyTorch is required for Polymetis. Install via conda/micromamba.")
        results.append(False)
    except Exception as e:
        print_error(f"PyTorch import error: {e}")
        results.append(False)
    
    return all(results)


def test_polymetis_imports() -> bool:
    """Test Polymetis imports"""
    print_header("2. Polymetis Dependencies")
    
    results = []
    
    # Test RobotInterface
    try:
        from polymetis import RobotInterface
        print_success("Polymetis RobotInterface imported successfully")
        results.append(True)
    except ImportError as e:
        print_error(f"Polymetis RobotInterface not available: {e}")
        print_warning("  Install Polymetis: pip install -e /opt/polymetis/polymetis")
        results.append(False)
    except (IndexError, Exception) as e:
        if "list index out of range" in str(e) or "version" in str(e).lower() or "Cannot locate Polymetis version" in str(e):
            print_warning(f"Polymetis version detection failed: {e}")
            print_info("  (This is OK if polymetis is not in a git repo)")
            print_info("  (Polymetis functionality may still work)")
            # Try to import again, ignoring version
            try:
                import sys
                import polymetis
                # Temporarily patch _version to avoid error
                if hasattr(polymetis, '_version'):
                    polymetis._version.__version__ = "0.0.0-unknown"
                from polymetis import RobotInterface
                print_success("Polymetis RobotInterface imported successfully (version detection bypassed)")
                results.append(True)
            except:
                results.append(False)  # Version detection is not critical, but import failed
        else:
            print_error(f"Polymetis RobotInterface import error: {e}")
            results.append(False)
    
    # Test GripperInterface
    try:
        from polymetis import GripperInterface
        print_success("Polymetis GripperInterface imported successfully")
        results.append(True)
    except ImportError as e:
        print_error(f"Polymetis GripperInterface not available: {e}")
        results.append(False)
    except (IndexError, Exception) as e:
        if "list index out of range" in str(e) or "version" in str(e).lower() or "Cannot locate Polymetis version" in str(e):
            print_warning(f"Polymetis version detection failed: {e}")
            print_info("  (This is OK if polymetis is not in a git repo)")
            print_info("  (Polymetis functionality may still work)")
            # Try to import again, ignoring version
            try:
                import sys
                import polymetis
                # Temporarily patch _version to avoid error
                if hasattr(polymetis, '_version'):
                    polymetis._version.__version__ = "0.0.0-unknown"
                from polymetis import GripperInterface
                print_success("Polymetis GripperInterface imported successfully (version detection bypassed)")
                results.append(True)
            except:
                results.append(False)
        else:
            print_error(f"Polymetis GripperInterface import error: {e}")
            results.append(False)
    
    return all(results)


def test_ros2_core_imports() -> bool:
    """Test ROS2 core imports"""
    print_header("3. ROS2 Core Dependencies")
    
    results = []
    results.append(test_import('rclpy', 'rclpy'))
    results.append(test_import('rclpy.node', 'rclpy.node'))
    results.append(test_import('rclpy.qos', 'rclpy.qos'))
    results.append(test_import('sensor_msgs.msg', 'sensor_msgs.msg'))
    results.append(test_import('geometry_msgs.msg', 'geometry_msgs.msg'))
    results.append(test_import('std_msgs.msg', 'std_msgs.msg'))
    
    return all(results)


def test_role_ros2_messages() -> bool:
    """Test role_ros2 custom messages"""
    print_header("4. role_ros2 Message Dependencies")
    
    results = []
    
    try:
        from role_ros2.msg import (
            PolymetisCommand, PolymetisRobotState, PolymetisRobotCommand,
            PolymetisGripperState, GripperCommand, ControllerStatus
        )
        print_success("role_ros2.msg imported successfully")
        print_info(f"  Messages: PolymetisCommand, PolymetisRobotState, PolymetisRobotCommand, "
                   f"PolymetisGripperState, GripperCommand, ControllerStatus")
        results.append(True)
    except ImportError as e:
        print_error(f"role_ros2.msg import failed: {e}")
        print_warning("  Build ROS2 workspace: cd /app/ros2_ws && colcon build")
        results.append(False)
    
    return all(results)


def test_role_ros2_services() -> bool:
    """Test role_ros2 custom services"""
    print_header("5. role_ros2 Service Dependencies")
    
    results = []
    
    try:
        from role_ros2.srv import (
            Reset, Home, StartCartesianImpedance, StartJointImpedance,
            TerminatePolicy, MoveToJointPositions, MoveToEEPose,
            SolveIK, ComputeFK, ComputeTimeToGo
        )
        print_success("role_ros2.srv imported successfully")
        print_info(f"  Services: Reset, Home, StartCartesianImpedance, StartJointImpedance, "
                   f"TerminatePolicy, MoveToJointPositions, MoveToEEPose, "
                   f"SolveIK, ComputeFK, ComputeTimeToGo")
        results.append(True)
    except ImportError as e:
        print_error(f"role_ros2.srv import failed: {e}")
        print_warning("  Build ROS2 workspace: cd /app/ros2_ws && colcon build")
        results.append(False)
    
    return all(results)


def test_ik_solver_and_transformations() -> Tuple[bool, bool]:
    """Test IK solver and transformations (optional but recommended)"""
    print_header("6. IK Solver and Transformations (Optional)")
    
    ik_available = False
    transforms_available = False
    
    # Test IK solver
    try:
        from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
        print_success("RobotIKSolver imported successfully")
        print_info("  High-level control commands (cartesian/joint position/velocity) will work")
        ik_available = True
    except ImportError as e:
        print_warning(f"RobotIKSolver not available: {e}")
        print_warning("  High-level control commands will NOT work")
        print_warning("  Only low-level impedance control will be available")
    
    # Test transformations
    try:
        from role_ros2.misc.transformations import (
            add_poses, euler_to_quat, pose_diff, quat_to_euler
        )
        print_success("Transformations imported successfully")
        print_info("  Functions: add_poses, euler_to_quat, pose_diff, quat_to_euler")
        transforms_available = True
    except ImportError as e:
        print_warning(f"Transformations not available: {e}")
        print_warning("  Some pose conversion functions may not work")
    
    return ik_available, transforms_available


def test_config_file() -> Tuple[bool, Optional[dict]]:
    """Test configuration file (franka_robot_config.yaml)"""
    print_header("7. Configuration File")
    
    config = None
    
    # Try to find config file
    config_paths = []
    
    # Path 1: Package share directory (installed)
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('role_ros2')
        config_paths.append(Path(package_share_dir) / 'config' / 'franka_robot_config.yaml')
    except Exception:
        pass
    
    # Path 2: Source directory
    config_paths.extend([
        Path('/app/ros2_ws/src/role-ros2/config/franka_robot_config.yaml'),
        Path(__file__).parent.parent / 'config' / 'franka_robot_config.yaml',
    ])
    
    config_file = None
    for path in config_paths:
        if path.exists():
            config_file = path
            break
    
    if config_file is None:
        print_error(f"Config file not found in any of these locations:")
        for path in config_paths:
            print_error(f"  - {path}")
        print_warning("  Create config file at: config/franka_robot_config.yaml")
        return False, None
    
    print_success(f"Config file found: {config_file}")
    
    # Try to load and validate config
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        if config is None:
            print_error("Config file is empty or invalid YAML")
            return False, None
        
        # Validate required fields
        required_fields = ['arm_id', 'arm_joints', 'gripper_joints', 'robot_ip']
        missing_fields = [field for field in required_fields if field not in config]
        
        if missing_fields:
            print_error(f"Config file missing required fields: {missing_fields}")
            return False, config
        
        print_success("Config file is valid")
        print_info(f"  arm_id: {config.get('arm_id')}")
        print_info(f"  robot_ip: {config.get('robot_ip')}")
        print_info(f"  arm_joints: {len(config.get('arm_joints', []))} joints")
        print_info(f"  gripper_joints: {len(config.get('gripper_joints', []))} joints")
        
        # Validate joint names format
        arm_id = config.get('arm_id', '')
        arm_joints = config.get('arm_joints', [])
        gripper_joints = config.get('gripper_joints', [])
        
        invalid_arm_joints = [j for j in arm_joints if not j.startswith(f'{arm_id}_')]
        invalid_gripper_joints = [j for j in gripper_joints if not j.startswith(f'{arm_id}_')]
        
        if invalid_arm_joints:
            print_error(f"Invalid arm joint names (must start with '{arm_id}_'): {invalid_arm_joints}")
            return False, config
        
        if invalid_gripper_joints:
            print_error(f"Invalid gripper joint names (must start with '{arm_id}_'): {invalid_gripper_joints}")
            return False, config
        
        print_success("Joint names format is valid")
        return True, config
        
    except yaml.YAMLError as e:
        print_error(f"Failed to parse config file: {e}")
        return False, None
    except Exception as e:
        print_error(f"Error reading config file: {e}")
        return False, None


def test_urdf_file(config: Optional[dict] = None) -> bool:
    """Test URDF file existence"""
    print_header("8. URDF File")
    
    if config is None:
        print_warning("Cannot check URDF file without config")
        return False
    
    arm_id = config.get('arm_id', 'fr3')
    
    # Try to find URDF file
    urdf_paths = []
    
    # Path 1: Package share directory (installed)
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('role_ros2')
        urdf_paths.append(Path(package_share_dir) / 'robot_ik' / 'franka' / f'{arm_id}.urdf')
    except Exception:
        pass
    
    # Path 2: Source directory
    urdf_paths.extend([
        Path(f'/app/ros2_ws/src/role-ros2/role_ros2/robot_ik/franka/{arm_id}.urdf'),
        Path(__file__).parent.parent / 'role_ros2' / 'robot_ik' / 'franka' / f'{arm_id}.urdf',
    ])
    
    urdf_file = None
    for path in urdf_paths:
        if path.exists():
            urdf_file = path
            break
    
    if urdf_file is None:
        print_error(f"URDF file not found for arm_id='{arm_id}'")
        print_error("  Searched in:")
        for path in urdf_paths:
            print_error(f"    - {path}")
        print_warning("  Generate URDF file:")
        print_warning(f"    python3 scripts/mujoco_to_urdf.py \\")
        print_warning(f"        role_ros2/robot_ik/franka/{arm_id}.xml \\")
        print_warning(f"        role_ros2/robot_ik/franka/{arm_id}.urdf \\")
        print_warning(f"        {arm_id}")
        return False
    
    print_success(f"URDF file found: {urdf_file}")
    
    # Check if file is readable
    try:
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
            if len(urdf_content) < 100:
                print_warning("URDF file seems too short (may be invalid)")
                return False
            print_success("URDF file is readable and has content")
            return True
    except Exception as e:
        print_error(f"Error reading URDF file: {e}")
        return False


def test_launch_file_dependencies() -> bool:
    """Test launch file dependencies"""
    print_header("9. Launch File Dependencies")
    
    results = []
    results.append(test_import('launch', 'launch'))
    results.append(test_import('launch.actions', 'launch.actions'))
    results.append(test_import('launch.conditions', 'launch.conditions'))
    results.append(test_import('launch.substitutions', 'launch.substitutions'))
    results.append(test_import('launch_ros.actions', 'launch_ros.actions'))
    results.append(test_import('ament_index_python.packages', 'ament_index_python.packages'))
    results.append(test_import('yaml', 'yaml'))
    
    return all(results)


def test_executables() -> bool:
    """Test if executables are available"""
    print_header("10. Executables Availability")
    
    results = []
    
    # Check robot_state_publisher (ROS2 executable, not in PATH but accessible via ros2 run)
    try:
        # Method 1: Check if ros2 command is available and can find the package
        result = subprocess.run(
            ['ros2', 'pkg', 'exec', 'robot_state_publisher', '--', 'robot_state_publisher', '--help'],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0 or 'robot_state_publisher' in result.stderr or 'robot_state_publisher' in result.stdout:
            print_success("robot_state_publisher found (via ros2 pkg exec)")
            results.append(True)
        else:
            # Method 2: Check if package is installed
            result2 = subprocess.run(
                ['ros2', 'pkg', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if 'robot_state_publisher' in result2.stdout:
                print_success("robot_state_publisher package found (via ros2 pkg list)")
                results.append(True)
            else:
                # Method 3: Check direct path
                import os
                ros_lib_path = '/opt/ros/foxy/lib/robot_state_publisher'
                if os.path.exists(ros_lib_path):
                    print_success(f"robot_state_publisher found at: {ros_lib_path}")
                    print_info("  Use 'ros2 run robot_state_publisher robot_state_publisher' to run")
                    results.append(True)
                else:
                    print_error("robot_state_publisher not found")
                    print_warning("  Install: apt-get install ros-foxy-robot-state-publisher")
                    print_warning("  Or use: ros2 run robot_state_publisher robot_state_publisher")
                    results.append(False)
    except FileNotFoundError:
        print_error("ros2 command not found")
        print_warning("  Source ROS2: source /opt/ros/foxy/setup.bash")
        results.append(False)
    except Exception as e:
        print_error(f"Error checking robot_state_publisher: {e}")
        results.append(False)
    
    # Check polymetis_bridge executable
    try:
        from ament_index_python.packages import get_package_share_directory
        package_share_dir = get_package_share_directory('role_ros2')
        # Correct path: install/role_ros2/share/role_ros2 -> install/role_ros2/lib/role_ros2
        lib_dir = Path(package_share_dir).parent.parent / 'lib' / 'role_ros2'
        exe_path = lib_dir / 'polymetis_bridge'
        
        if exe_path.exists():
            print_success(f"polymetis_bridge found: {exe_path}")
            results.append(True)
        else:
            print_error(f"polymetis_bridge not found at: {exe_path}")
            
            # Try alternative paths
            alt_paths = [
                Path('/app/ros2_ws/src/role-ros2/scripts/polymetis_bridge_node.py'),
                Path(__file__).parent.parent / 'scripts' / 'polymetis_bridge_node.py',
            ]
            
            found_alt = False
            for alt_path in alt_paths:
                if alt_path.exists():
                    print_warning(f"  Found source file: {alt_path}")
                    print_warning("  Build ROS2 workspace to install executable:")
                    print_warning("    cd /app/ros2_ws && colcon build")
                    found_alt = True
                    break
            
            if not found_alt:
                print_error("  Source file also not found")
            
            results.append(False)
    except Exception as e:
        print_error(f"Error checking polymetis_bridge: {e}")
        results.append(False)
    
    return all(results)


def test_environment_variables() -> bool:
    """Test environment variables"""
    print_header("11. Environment Variables")
    
    results = []
    
    # Check ROS2 environment
    ros_distro = os.environ.get('ROS_DISTRO', '')
    if ros_distro:
        print_success(f"ROS_DISTRO: {ros_distro}")
        results.append(True)
    else:
        print_warning("ROS_DISTRO not set")
        print_warning("  Source ROS2: source /opt/ros/foxy/setup.bash")
        results.append(False)
    
    # Check conda environment
    conda_env = os.environ.get('CONDA_DEFAULT_ENV', '')
    if conda_env:
        print_success(f"CONDA_DEFAULT_ENV: {conda_env}")
        if conda_env == 'polymetis-local':
            print_success("  Correct conda environment (polymetis-local)")
        else:
            print_warning(f"  Expected 'polymetis-local', got '{conda_env}'")
        results.append(True)
    else:
        print_warning("CONDA_DEFAULT_ENV not set")
        print_warning("  Activate conda: micromamba activate polymetis-local")
        results.append(False)
    
    # Check ROBOT_IP
    robot_ip = os.environ.get('ROBOT_IP', '')
    if robot_ip:
        print_success(f"ROBOT_IP: {robot_ip}")
        results.append(True)
    else:
        print_warning("ROBOT_IP not set (using default: 172.17.0.2)")
        results.append(True)  # Not critical, has default
    
    return all(results)


def main():
    """Main test function"""
    print_header("Polymetis Dependencies Test")
    print_info("Testing dependencies for polymetis_bridge_node.py and franka_robot.launch.py")
    print()
    
    all_results = []
    
    # Core dependencies (required)
    all_results.append(("PyTorch", test_pytorch_imports()))
    all_results.append(("Polymetis", test_polymetis_imports()))
    all_results.append(("ROS2 Core", test_ros2_core_imports()))
    all_results.append(("role_ros2 Messages", test_role_ros2_messages()))
    all_results.append(("role_ros2 Services", test_role_ros2_services()))
    all_results.append(("Launch Dependencies", test_launch_file_dependencies()))
    
    # Optional dependencies
    ik_available, transforms_available = test_ik_solver_and_transformations()
    all_results.append(("IK Solver", ik_available))
    all_results.append(("Transformations", transforms_available))
    
    # Configuration and files
    config_valid, config = test_config_file()
    all_results.append(("Config File", config_valid))
    
    if config_valid and config:
        all_results.append(("URDF File", test_urdf_file(config)))
    else:
        all_results.append(("URDF File", False))
    
    # Executables
    all_results.append(("Executables", test_executables()))
    
    # Environment
    all_results.append(("Environment", test_environment_variables()))
    
    # Summary
    print_header("Test Summary")
    
    required_tests = [
        "PyTorch", "Polymetis", "ROS2 Core", "role_ros2 Messages",
        "role_ros2 Services", "Launch Dependencies", "Config File", "URDF File", "Executables"
    ]
    
    optional_tests = ["IK Solver", "Transformations", "Environment"]
    
    required_passed = 0
    required_total = 0
    optional_passed = 0
    optional_total = 0
    
    for name, result in all_results:
        if name in required_tests:
            required_total += 1
            if result:
                required_passed += 1
                print_success(f"{name}: PASSED")
            else:
                print_error(f"{name}: FAILED")
        elif name in optional_tests:
            optional_total += 1
            if result:
                optional_passed += 1
                print_success(f"{name}: PASSED (optional)")
            else:
                print_warning(f"{name}: FAILED (optional)")
    
    print()
    print(f"Required tests: {required_passed}/{required_total} passed")
    print(f"Optional tests: {optional_passed}/{optional_total} passed")
    
    if required_passed == required_total:
        print_success("\n✓ All required dependencies are available!")
        print_success("  polymetis_bridge_node.py and franka_robot.launch.py should work correctly.")
        return 0
    else:
        print_error(f"\n✗ {required_total - required_passed} required dependency(ies) missing!")
        print_error("  Please fix the issues above before running polymetis_bridge_node.")
        return 1


if __name__ == '__main__':
    sys.exit(main())


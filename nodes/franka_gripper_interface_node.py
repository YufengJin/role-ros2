#!/usr/bin/env python3
"""
Franka Gripper Interface Node - ROS 2 Driver for Polymetis Gripper

This node provides a ROS 2 interface for the Franka gripper via Polymetis.
It handles gripper control only; arm is handled by a separate node.

Features:
- Publishers:
  - /{namespace}/gripper_state: Gripper state information
  - /{namespace}/joint_states: Gripper joint states for tf tree
- Subscribers:
  - /{namespace}/gripper/command: Gripper control commands
- Services:
  - /{namespace}/gripper/goto: Move gripper to specified width
  - /{namespace}/gripper/grasp: Close gripper to grasp
  - /{namespace}/gripper/open: Open gripper fully
  - /{namespace}/gripper/close: Close gripper fully

Author: Role-ROS2 Team
"""

import time
import threading
import os
import sys
import yaml
import traceback
from pathlib import Path
from typing import Optional, List, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

# Try to import Polymetis (graceful fallback if not available)
try:
    from polymetis import GripperInterface
    POLYMETIS_AVAILABLE = True
except ImportError:
    POLYMETIS_AVAILABLE = False
    print("Warning: Polymetis not available. Using MockGripperInterface.")

# Import custom messages (V2)
from role_ros2.msg import GripperState, GripperCommand

# Import services
from role_ros2.srv import GripperGoto, GripperGrasp


# ============================================================================
# HYPERPARAMETERS
# ============================================================================

DEFAULT_GRIPPER_SPEED = 0.05  # Default gripper movement speed (m/s)
DEFAULT_GRIPPER_FORCE = 0.1  # Default gripper force (normalized 0-1)

# ============================================================================


def load_gripper_joint_names_from_config(config_file: Optional[str] = None) -> Tuple[List[str], str]:
    """
    Load gripper joint names from config/franka_robot_config.yaml.
    
    Args:
        config_file: Path to config file. If None, use ROS2 package share directory.
    
    Returns:
        tuple: (gripper_joint_names, arm_id)
    """
    if config_file is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('role_ros2')
            config_file = Path(package_share_dir) / 'config' / 'franka_robot_config.yaml'
        except Exception as e:
            raise FileNotFoundError(f"Failed to get package share directory: {e}")
    else:
        config_file = Path(config_file)
    
    if not config_file.exists():
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        raise ValueError(f"Failed to parse config file {config_file}: {e}")
    
    if config is None:
        raise ValueError(f"Config file {config_file} is empty or invalid YAML")
    
    gripper_joints = config.get('gripper_joints', [])
    arm_id = config.get('arm_id', '')
    
    if not isinstance(gripper_joints, list):
        raise ValueError(f"gripper_joints must be a list, got {type(gripper_joints).__name__}")
    if not isinstance(arm_id, str) or not arm_id:
        raise ValueError("arm_id is missing or empty in franka_robot_config.yaml")
    if not gripper_joints:
        raise ValueError("gripper_joints is empty or missing in franka_robot_config.yaml")
    
    for joint in gripper_joints:
        if not isinstance(joint, str):
            raise ValueError(f"Joint name must be a string, got {type(joint).__name__}: {joint}")
        if not joint.startswith(f'{arm_id}_'):
            raise ValueError(
                f"Gripper joint name '{joint}' does not match arm_id '{arm_id}'. "
                f"Expected format: '{arm_id}_panda_finger_jointX'"
            )
    
    return gripper_joints, arm_id


class MockGripperInterface:
    """
    Mock implementation of Polymetis GripperInterface for testing without hardware.
    
    State updates based on commands, simulating realistic motion.
    """
    
    def __init__(self, update_rate: float = 50.0):
        """
        Initialize mock gripper interface.
        
        Args:
            update_rate: Update rate in Hz for state simulation
        """
        self._current_width = 0.08  # Current width in meters
        self._target_width = 0.08  # Target width from commands
        self._is_moving = False
        self._last_update_time = time.time()
        self._update_rate = update_rate
        self._max_speed = 0.1  # m/s
        self.metadata = type('obj', (object,), {'max_width': 0.08})()
        
        # Thread safety
        self._state_lock = threading.Lock()
    
    def update_state(self, dt: Optional[float] = None):
        """Update gripper state based on target width."""
        with self._state_lock:
            if dt is None:
                current_time = time.time()
                dt = min(current_time - self._last_update_time, 0.1)
                self._last_update_time = current_time
            
            width_error = self._target_width - self._current_width
            
            if abs(width_error) > 0.001:
                self._is_moving = True
                max_step = self._max_speed * dt
                step = np.clip(width_error, -max_step, max_step)
                self._current_width += step
                self._current_width = np.clip(self._current_width, 0.0, self.metadata.max_width)
            else:
                self._is_moving = False
                self._current_width = self._target_width
    
    def goto(self, width: float, speed: float = 0.05, force: float = 0.1, blocking: bool = False):
        """Move gripper to specified width."""
        with self._state_lock:
            self._target_width = max(0.0, min(width, self.metadata.max_width))
            self._max_speed = 0.1 * speed if speed > 0 else 0.1
        
        if blocking:
            start_time = time.time()
            timeout = 5.0
            dt = 1.0 / self._update_rate
            while True:
                with self._state_lock:
                    error = abs(self._target_width - self._current_width)
                if error <= 0.001 or (time.time() - start_time) >= timeout:
                    break
                self.update_state(dt=dt)
                time.sleep(0.01)
            with self._state_lock:
                self._is_moving = False
    
    def grasp(self, speed: float = 0.05, force: float = 0.1, blocking: bool = True) -> bool:
        """Close gripper to grasp an object."""
        self.goto(width=0.0, speed=speed, force=force, blocking=blocking)
        # In mock mode, assume grasp is successful if width is near zero
        with self._state_lock:
            return self._current_width < 0.01
    
    def get_state(self):
        """Get gripper state."""
        with self._state_lock:
            width = float(self._current_width)
            is_moving = bool(self._is_moving)
        
        class MockGripperState:
            def __init__(self, width, is_moving):
                self.width = float(width)
                self.is_moving = bool(is_moving)
                self.is_grasped = bool(width < 0.01)
                self.prev_command_successful = True
                self.error_code = 0
        
        return MockGripperState(width, is_moving)


class FrankaGripperInterfaceNode(Node):
    """
    ROS 2 Node that provides interface for Franka gripper via Polymetis.
    
    This node handles gripper control only; arm is handled by a separate node.
    """
    
    def __init__(self, use_mock: bool = False, ip_address: str = "localhost", namespace: str = ""):
        """
        Initialize the Franka Gripper Interface Node.
        
        Args:
            use_mock: If True, use MockGripperInterface instead of real hardware
            ip_address: IP address of Polymetis server
            namespace: ROS namespace for topics/services (e.g., "fr3_gripper")
        """
        node_name = 'franka_gripper_interface_node'
        super().__init__(node_name, namespace=namespace)
        
        # Declare parameters
        self.declare_parameter('use_mock', use_mock)
        self.declare_parameter('ip_address', ip_address)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('namespace', namespace)
        
        # Load gripper joint names from config
        try:
            gripper_joints, arm_id = load_gripper_joint_names_from_config()
            self.get_logger().info(f"Loaded {len(gripper_joints)} gripper joint names for {arm_id}")
        except (FileNotFoundError, ValueError) as e:
            error_msg = f"Failed to load gripper joint names: {e}"
            self.get_logger().error(error_msg)
            raise
        
        self.declare_parameter('gripper_joint_names', gripper_joints)
        
        # Get parameters
        use_mock = self.get_parameter('use_mock').get_parameter_value().bool_value
        ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.gripper_joint_names = list(self.get_parameter('gripper_joint_names').get_parameter_value().string_array_value)
        self._namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        self.get_logger().info(f"Using gripper joint names: {self.gripper_joint_names}")
        
        # Initialize gripper interface
        self._gripper: Optional[GripperInterface] = None
        
        try:
            if use_mock or not POLYMETIS_AVAILABLE:
                self.get_logger().info("Using MockGripperInterface (no hardware)")
                self._gripper = MockGripperInterface(update_rate=self.publish_rate)
            else:
                self.get_logger().info(f"Connecting to Polymetis gripper server at {ip_address}")
                self._gripper = self._init_gripper_interface_with_retry(ip_address)
                if self._gripper is None:
                    self.get_logger().warn("Failed to initialize real gripper. Using mock.")
                    self._gripper = MockGripperInterface(update_rate=self.publish_rate)
                else:
                    self.get_logger().info("Successfully connected to Polymetis gripper server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Polymetis gripper: {e}. Using mock interface.")
            self._gripper = MockGripperInterface(update_rate=self.publish_rate)
        
        # Get max gripper width from metadata
        try:
            if hasattr(self._gripper, 'metadata') and self._gripper.metadata is not None:
                self._max_gripper_width = self._gripper.metadata.max_width
                self.get_logger().info(f"Using gripper max_width={self._max_gripper_width} from metadata")
            else:
                self._max_gripper_width = 0.08
                self.get_logger().info("Using default max_gripper_width=0.08")
        except (AttributeError, TypeError) as e:
            self._max_gripper_width = 0.08
            self.get_logger().info(f"Using default max_gripper_width=0.08 (error: {type(e).__name__})")
        
        # Callback groups
        self._timer_callback_group = ReentrantCallbackGroup()
        self._service_callback_group = MutuallyExclusiveCallbackGroup()
        
        # ===== Publishers =====
        self._gripper_state_publisher = self.create_publisher(
            GripperState, 'gripper_state', 10
        )
        
        self._joint_state_publisher = self.create_publisher(
            JointState, 'joint_states', 10
        )
        
        # ===== Subscribers =====
        self._gripper_cmd_subscriber = self.create_subscription(
            GripperCommand,
            'gripper/command',
            self._gripper_command_callback,
            10
        )
        
        # ===== Services =====
        self._gripper_goto_service = self.create_service(
            GripperGoto, 'gripper/goto',
            self._gripper_goto_callback,
            callback_group=self._service_callback_group
        )
        
        self._gripper_grasp_service = self.create_service(
            GripperGrasp, 'gripper/grasp',
            self._gripper_grasp_callback,
            callback_group=self._service_callback_group
        )
        
        self._gripper_open_service = self.create_service(
            Trigger, 'gripper/open',
            self._gripper_open_callback,
            callback_group=self._service_callback_group
        )
        
        self._gripper_close_service = self.create_service(
            Trigger, 'gripper/close',
            self._gripper_close_callback,
            callback_group=self._service_callback_group
        )
        
        # Timer for publishing states
        timer_period = 1.0 / self.publish_rate
        self._state_timer = self.create_timer(
            timer_period, self._publish_states,
            callback_group=self._timer_callback_group
        )
        
        self.get_logger().info(
            f'FrankaGripperInterfaceNode initialized. '
            f'Namespace: {self._namespace or "(default)"}, '
            f'Publishing at {self.publish_rate} Hz'
        )
        
        # Open gripper on startup
        self._open_gripper_on_startup()
    
    def _open_gripper_on_startup(self):
        """Open gripper to known state on startup."""
        # Wait for gripper server to be ready if using real gripper
        if not isinstance(self._gripper, MockGripperInterface):
            max_wait = 10.0  # Maximum wait time in seconds
            wait_interval = 0.5  # Check interval
            waited = 0.0
            
            self.get_logger().info("Waiting for gripper server to be ready...")
            while waited < max_wait:
                try:
                    # Try to get state to verify server is ready
                    self._gripper.get_state()
                    self.get_logger().info("Gripper server is ready")
                    break  # Server is ready
                except Exception as e:
                    if waited < max_wait - wait_interval:
                        time.sleep(wait_interval)
                        waited += wait_interval
                        self.get_logger().debug(
                            f"Gripper server not ready yet (waited {waited:.1f}s): {e}. Retrying..."
                        )
                    else:
                        self.get_logger().warn(
                            f"Gripper server not ready after {max_wait}s: {e}. "
                            "Skipping startup gripper open."
                        )
                        return
        
        try:
            self.get_logger().info("Opening gripper on startup...")
            self._gripper.goto(
                width=self._max_gripper_width,
                speed=DEFAULT_GRIPPER_SPEED,
                force=DEFAULT_GRIPPER_FORCE,
                blocking=True
            )
            self.get_logger().info(f"✅ Gripper opened to {self._max_gripper_width:.4f}m on startup")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Failed to open gripper on startup: {e}")
    
    def _init_gripper_interface_with_retry(self, ip_address: str, max_retries: int = 10, retry_delay: float = 0.5):
        """
        Initialize GripperInterface with retry mechanism.
        
        Args:
            ip_address: IP address of the gripper server
            max_retries: Maximum number of retry attempts
            retry_delay: Initial delay between retries in seconds
        
        Returns:
            GripperInterface or None if all retries failed
        """
        import grpc
        
        for attempt in range(max_retries):
            try:
                if attempt > 0:
                    self.get_logger().info(
                        f"Initializing GripperInterface (attempt {attempt + 1}/{max_retries})..."
                    )
                gripper = GripperInterface(ip_address=ip_address)
                
                if hasattr(gripper, 'metadata') and gripper.metadata is not None:
                    self.get_logger().info(
                        f"GripperInterface initialized with metadata (max_width={gripper.metadata.max_width})"
                    )
                    return gripper
                else:
                    self.get_logger().info(
                        f"GripperInterface created but metadata not available (attempt {attempt + 1})"
                    )
                    try:
                        import polymetis_pb2
                        default_metadata = polymetis_pb2.GripperMetadata()
                        default_metadata.polymetis_version = "0.2"
                        default_metadata.hz = 50
                        default_metadata.max_width = 0.08
                        gripper.metadata = default_metadata
                        return gripper
                    except ImportError as e:
                        self.get_logger().warn(f"Could not import polymetis_pb2: {e}")
                        if attempt < max_retries - 1:
                            time.sleep(retry_delay * (attempt + 1))
                            continue
                        return None
                
            except (grpc.RpcError, TypeError) as e:
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (attempt + 1)
                    self.get_logger().info(
                        f"GripperInterface initialization failed ({type(e).__name__}): {str(e)[:100]}. "
                        f"Retrying in {wait_time:.1f}s..."
                    )
                    time.sleep(wait_time)
                else:
                    self.get_logger().warn(
                        f"GripperInterface initialization failed after {max_retries} attempts: {e}"
                    )
            except Exception as e:
                self.get_logger().error(f"Unexpected error initializing GripperInterface: {e}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay * (attempt + 1))
                else:
                    return None
        
        return None
    
    def _publish_states(self):
        """Publish gripper state and joint states."""
        # Update mock state if using mock interface
        if isinstance(self._gripper, MockGripperInterface):
            dt = 1.0 / self.publish_rate
            self._gripper.update_state(dt=dt)
        
        self._publish_gripper_state()
        self._publish_joint_states()
    
    def _publish_gripper_state(self):
        """Publish gripper state."""
        try:
            data_ros_time = self.get_clock().now()
            data_timestamp_ns = data_ros_time.nanoseconds
            
            gripper_state = self._gripper.get_state()
            
            msg = GripperState()
            msg.header.stamp = data_ros_time.to_msg()
            msg.header.frame_id = 'gripper_link'
            
            msg.width = float(gripper_state.width)
            # position: 0=open (width=max), 1=closed (width=0) - matches droid convention
            msg.position = float(1.0 - gripper_state.width / self._max_gripper_width)
            msg.is_grasped = bool(getattr(gripper_state, 'is_grasped', False))
            msg.is_moving = bool(getattr(gripper_state, 'is_moving', False))
            msg.prev_command_successful = bool(getattr(gripper_state, 'prev_command_successful', True))
            msg.error_code = int(getattr(gripper_state, 'error_code', 0))
            msg.max_width = self._max_gripper_width
            msg.timestamp_ns = int(data_timestamp_ns)
            
            self._gripper_state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing gripper state: {e}")
    
    def _publish_joint_states(self):
        """Publish gripper joint states."""
        try:
            gripper_state = self._gripper.get_state()
            gripper_width = gripper_state.width
            
            # Each finger moves half the width
            finger1_position = gripper_width / 2.0
            finger2_position = gripper_width / 2.0
            
            msg = JointState()
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'base_link'
            msg.name = list(self.gripper_joint_names)
            msg.position = [finger1_position, finger2_position]
            msg.velocity = [0.0, 0.0]
            msg.effort = [0.0, 0.0]
            
            self._joint_state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing gripper joint states: {e}")
    
    def _gripper_command_callback(self, msg: GripperCommand):
        """Callback for gripper commands."""
        try:
            width = msg.width
            speed = msg.speed if msg.speed > 0 else DEFAULT_GRIPPER_SPEED
            force = msg.force if msg.force > 0 else DEFAULT_GRIPPER_FORCE
            blocking = msg.blocking
            
            if blocking:
                def gripper_thread():
                    self._gripper.goto(width=width, speed=speed, force=force, blocking=True)
                threading.Thread(target=gripper_thread, daemon=True).start()
            else:
                self._gripper.goto(width=width, speed=speed, force=force, blocking=False)
        except Exception as e:
            self.get_logger().error(f"Error processing gripper command: {e}")
    
    def _gripper_goto_callback(self, request, response):
        """Service callback for gripper goto."""
        try:
            width = request.width
            speed = request.speed if request.speed > 0 else DEFAULT_GRIPPER_SPEED
            force = request.force if request.force > 0 else DEFAULT_GRIPPER_FORCE
            blocking = request.blocking
            
            if blocking:
                self._gripper.goto(width=width, speed=speed, force=force, blocking=True)
                response.success = True
                response.message = f"Moved to width {width:.4f}m"
            else:
                self._gripper.goto(width=width, speed=speed, force=force, blocking=False)
                response.success = True
                response.message = f"Command sent to move to width {width:.4f}m"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _gripper_grasp_callback(self, request, response):
        """Service callback for gripper grasp."""
        try:
            speed = request.speed if request.speed > 0 else DEFAULT_GRIPPER_SPEED
            force = request.force if request.force > 0 else DEFAULT_GRIPPER_FORCE
            blocking = request.blocking
            
            if isinstance(self._gripper, MockGripperInterface):
                is_grasped = self._gripper.grasp(speed=speed, force=force, blocking=blocking)
                response.success = True
                response.message = "Grasp completed" if is_grasped else "Grasp completed (no object detected)"
                response.is_grasped = is_grasped
            else:
                # For real gripper, use goto with width=0
                self._gripper.goto(width=0.0, speed=speed, force=force, blocking=blocking)
                gripper_state = self._gripper.get_state()
                response.success = True
                response.message = "Grasp completed"
                response.is_grasped = bool(getattr(gripper_state, 'is_grasped', gripper_state.width < 0.01))
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
            response.is_grasped = False
        
        return response
    
    def _gripper_open_callback(self, request, response):
        """Service callback for gripper open."""
        try:
            self._gripper.goto(
                width=self._max_gripper_width,
                speed=DEFAULT_GRIPPER_SPEED,
                force=DEFAULT_GRIPPER_FORCE,
                blocking=True
            )
            response.success = True
            response.message = f"Opened to {self._max_gripper_width:.4f}m"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def _gripper_close_callback(self, request, response):
        """Service callback for gripper close."""
        try:
            self._gripper.goto(
                width=0.0,
                speed=DEFAULT_GRIPPER_SPEED,
                force=DEFAULT_GRIPPER_FORCE,
                blocking=True
            )
            response.success = True
            response.message = "Closed"
        except Exception as e:
            response.success = False
            response.message = f"Failed: {str(e)}"
        
        return response
    
    def destroy_node(self):
        """Clean up resources on shutdown."""
        super().destroy_node()


def main(args=None):
    """Main function to run the Franka Gripper Interface Node."""
    from rclpy.executors import MultiThreadedExecutor
    
    rclpy.init(args=args)
    
    # Get namespace from command line or environment
    namespace = os.environ.get('GRIPPER_NAMESPACE', 'fr3_gripper')
    
    node = FrankaGripperInterfaceNode(
        use_mock=not POLYMETIS_AVAILABLE,
        namespace=namespace
    )
    
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()


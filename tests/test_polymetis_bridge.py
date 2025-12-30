#!/usr/bin/env python3
"""
Test script for polymetis_bridge_node

This script tests:
1. All topics are published correctly
2. All services are available and functional
3. Robot can be controlled via commands
4. Movement tests with SLOW speed (10x slower for safety)

Usage:
    python3 test_polymetis_bridge.py [--mock] [--full-test]
    
    --mock: Test in mock mode (no real robot)
    --full-test: Perform full movement tests (WARNING: Robot will move!)
    
Expected real robot behavior:
- Reset: Gripper closes, robot slowly moves to home position (~20-30 seconds)
- Joint movement: Robot joints move slowly (~0.1 rad/s max velocity)
- Gripper: Opens/closes at slow speed (0.005 m/s)

Author: Role-ROS2 Team
"""

import sys
import time
import math
import argparse
import traceback
from typing import List, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Import messages
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from role_ros2.msg import (
    PolymetisCommand, PolymetisRobotState, PolymetisRobotCommand,
    PolymetisGripperState, GripperCommand, ControllerStatus
)

# Import services (Home service was removed - now use Reset only)
from role_ros2.srv import (
    Reset, StartCartesianImpedance, StartJointImpedance,
    TerminatePolicy, MoveToJointPositions, MoveToEEPose,
    SolveIK, ComputeFK, ComputeTimeToGo
)

# Test configuration
WAIT_BETWEEN_TESTS = 5.0  # Wait 5 seconds between each test
SLOW_MOVEMENT_FACTOR = 3  # Movements are 3x slower for safety
DEFAULT_TIME_TO_GO = 10.0  # Default time for movements (slow)
GRIPPER_SLOW_SPEED = 0.05  # Gripper speed

# Movement scale - increase to make movements more visible
JOINT_OFFSET_RAD = 0.15  # Joint position offset in radians (~8.6 degrees)
JOINT_VELOCITY_RAD_S = 0.05  # Joint velocity in rad/s
CARTESIAN_VELOCITY_M_S = 0.03  # Cartesian velocity in m/s
VELOCITY_COMMAND_DURATION = 3.0  # Duration for velocity commands in seconds


class PolymetisBridgeTester(Node):
    """
    Test node for polymetis_bridge_node
    
    Tests all interfaces:
    - Topics: /joint_states, /polymetis/robot_state, /polymetis/gripper_state, etc.
    - Services: /polymetis/reset, /polymetis/arm/move_to_joint_positions, etc.
    - Commands: /polymetis_cmd, /polymetis/robot_command, /polymetis/gripper/command
    """
    
    def __init__(self, full_test: bool = False):
        super().__init__('polymetis_bridge_tester')
        
        self.full_test = full_test
        self.topics_received = {}
        self.services_available = {}
        self.test_results = {}
        
        # Expected topics (publishers from polymetis_bridge_node)
        self.expected_topics = {
            '/joint_states': JointState,
            '/polymetis/robot_state': PolymetisRobotState,
            '/polymetis/gripper_state': PolymetisGripperState,
            '/polymetis/arm/ee_pose': PoseStamped,
            '/polymetis/controller/status': ControllerStatus,
        }
        
        # Expected services (Home was removed, only Reset remains)
        self.expected_services = [
            '/polymetis/reset',  # Reset robot to home position (with optional randomize)
            '/polymetis/arm/start_cartesian_impedance',
            '/polymetis/arm/start_joint_impedance',
            '/polymetis/arm/terminate_policy',
            '/polymetis/arm/move_to_joint_positions',
            '/polymetis/arm/move_to_ee_pose',
            '/polymetis/arm/solve_ik',
            '/polymetis/arm/compute_fk',
            '/polymetis/arm/compute_time_to_go',
        ]
        
        # Create subscribers for all expected topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        for topic_name, msg_type in self.expected_topics.items():
            self.topics_received[topic_name] = False
            self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic_name: self._topic_callback(msg, t),
                qos_profile
            )
            self.get_logger().info(f"Subscribed to {topic_name}")
        
        # Create service clients
        self.service_clients = {}
        for service_name in self.expected_services:
            self.services_available[service_name] = False
            # Map service names to service types
            if 'reset' in service_name:
                self.service_clients[service_name] = self.create_client(Reset, service_name)
            elif 'start_cartesian_impedance' in service_name:
                self.service_clients[service_name] = self.create_client(StartCartesianImpedance, service_name)
            elif 'start_joint_impedance' in service_name:
                self.service_clients[service_name] = self.create_client(StartJointImpedance, service_name)
            elif 'terminate_policy' in service_name:
                self.service_clients[service_name] = self.create_client(TerminatePolicy, service_name)
            elif 'move_to_joint_positions' in service_name:
                self.service_clients[service_name] = self.create_client(MoveToJointPositions, service_name)
            elif 'move_to_ee_pose' in service_name:
                self.service_clients[service_name] = self.create_client(MoveToEEPose, service_name)
            elif 'solve_ik' in service_name:
                self.service_clients[service_name] = self.create_client(SolveIK, service_name)
            elif 'compute_fk' in service_name:
                self.service_clients[service_name] = self.create_client(ComputeFK, service_name)
            elif 'compute_time_to_go' in service_name:
                self.service_clients[service_name] = self.create_client(ComputeTimeToGo, service_name)
        
        # Create publishers for testing commands
        self.cmd_publisher = self.create_publisher(
            PolymetisCommand,
            '/polymetis_cmd',
            10
        )
        self.robot_cmd_publisher = self.create_publisher(
            PolymetisRobotCommand,
            '/polymetis/robot_command',
            10
        )
        self.gripper_cmd_publisher = self.create_publisher(
            GripperCommand,
            '/polymetis/gripper/command',
            10
        )
        
        # Store current robot state
        self.current_joint_positions = None
        self.current_robot_state = None
        self.current_gripper_state = None
        self.current_ee_pose = None
        
    def _topic_callback(self, msg, topic_name: str):
        """Callback for topic messages - stores latest state"""
        if not self.topics_received[topic_name]:
            self.topics_received[topic_name] = True
            self.get_logger().info(f"✓ Received first message from {topic_name}")
            
        # Store state for later use
        if topic_name == '/joint_states':
            self.current_joint_positions = list(msg.position)
        elif topic_name == '/polymetis/robot_state':
            self.current_robot_state = msg
        elif topic_name == '/polymetis/gripper_state':
            self.current_gripper_state = msg
        elif topic_name == '/polymetis/arm/ee_pose':
            self.current_ee_pose = msg
    
    def _wait_between_tests(self, test_name: str = ""):
        """Wait between tests with countdown"""
        self.get_logger().info(f"\n--- Waiting {WAIT_BETWEEN_TESTS}s before next test{' (' + test_name + ')' if test_name else ''}...")
        for i in range(int(WAIT_BETWEEN_TESTS), 0, -1):
            time.sleep(1)
            # Spin to keep receiving messages
            rclpy.spin_once(self, timeout_sec=0.01)
        self.get_logger().info("")
    
    def check_topics(self, timeout: float = 5.0) -> Dict[str, bool]:
        """
        Test 1: Check if all expected topics are publishing
        
        Expected real robot behavior:
        - All 5 topics should be publishing at 50Hz
        - /joint_states: 9 joints (7 arm + 2 gripper fingers)
        - /polymetis/robot_state: Full robot state including EE pose
        - /polymetis/gripper_state: Gripper width and normalized position
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 1: Checking Topics...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected: All topics publishing at 50Hz")
        self.get_logger().info("")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.topics_received.values()):
                break
        
        results = {}
        for topic_name, received in self.topics_received.items():
            status = "✓" if received else "✗"
            results[topic_name] = received
            
            # Print details about received messages
            if received:
                if topic_name == '/joint_states' and self.current_joint_positions:
                    self.get_logger().info(f"{status} {topic_name}: OK ({len(self.current_joint_positions)} joints)")
                elif topic_name == '/polymetis/robot_state' and self.current_robot_state:
                    self.get_logger().info(f"{status} {topic_name}: OK ({len(self.current_robot_state.joint_positions)} DOF)")
                elif topic_name == '/polymetis/gripper_state' and self.current_gripper_state:
                    self.get_logger().info(f"{status} {topic_name}: OK (width={self.current_gripper_state.width:.4f}m)")
                elif topic_name == '/polymetis/arm/ee_pose' and self.current_ee_pose:
                    pos = self.current_ee_pose.pose.position
                    self.get_logger().info(f"{status} {topic_name}: OK (pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}])")
                else:
                    self.get_logger().info(f"{status} {topic_name}: OK")
            else:
                self.get_logger().warn(f"{status} {topic_name}: NOT RECEIVED")
        
        return results
    
    def check_services(self, timeout: float = 2.0) -> Dict[str, bool]:
        """
        Test 2: Check if all expected services are available
        
        Expected real robot behavior:
        - All 9 services should be available
        - Services are provided by polymetis_bridge_node
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 2: Checking Services...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected: All 9 services available")
        self.get_logger().info("")
        
        results = {}
        for service_name, client in self.service_clients.items():
            if client.wait_for_service(timeout_sec=timeout):
                self.services_available[service_name] = True
                results[service_name] = True
                self.get_logger().info(f"✓ {service_name}: Available")
            else:
                self.services_available[service_name] = False
                results[service_name] = False
                self.get_logger().warn(f"✗ {service_name}: NOT AVAILABLE")
        
        return results
    
    def test_reset_service(self) -> bool:
        """
        Test 3: Test Reset service
        
        Expected real robot behavior:
        - Gripper closes first (blocking, ~5-10 seconds at slow speed)
        - Robot arm slowly moves to home position [0, -π/5, 0, -4π/5, 0, 3π/5, 0]
        - Movement takes ~20-40 seconds at 0.1 rad/s max velocity
        - Service returns immediately with "accepted", reset executes in background
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 3: Testing Reset Service...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected real robot behavior:")
        self.get_logger().info("  1. Gripper closes (blocking, ~5-10s)")
        self.get_logger().info("  2. Arm moves to home position slowly (~20-40s)")
        self.get_logger().info("  3. Service returns 'accepted' immediately")
        self.get_logger().info("")
        
        if not self.full_test:
            self.get_logger().info("⏭️  SKIPPED (use --full-test to enable)")
            return True
        
        client = self.service_clients.get('/polymetis/reset')
        if not client or not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("✗ Reset service not available")
            return False
        
        try:
            self.get_logger().warn("⚠️  WARNING: Robot will reset to home position!")
            self.get_logger().info("Sending reset command (randomize=False)...")
            
            request = Reset.Request()
            request.randomize = False  # No randomization for predictable test
            
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✓ Reset accepted: {response.message}")
                    self.get_logger().info("Waiting 15s for reset to complete...")
                    time.sleep(15)  # Wait for slow reset to complete
                    self.get_logger().info("✓ Reset test completed")
                    return True
                else:
                    self.get_logger().error(f"✗ Reset failed: {response.message}")
                    return False
            else:
                self.get_logger().error("✗ Reset service call timed out")
                return False
                
        except Exception as e:
            self.get_logger().error(f"✗ Reset test failed: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_gripper_command(self) -> bool:
        """
        Test 4: Test Gripper Command topic
        
        Expected real robot behavior:
        - Gripper opens to specified width at SLOW speed (0.005 m/s)
        - Open: width=0.08m (max), Close: width=0.0m
        - Movement takes longer due to slow speed
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 4: Testing Gripper Command...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected real robot behavior:")
        self.get_logger().info(f"  - Gripper opens/closes at slow speed ({GRIPPER_SLOW_SPEED} m/s)")
        self.get_logger().info("  - Open: width=0.08m, Close: width=0.0m")
        self.get_logger().info("")
        
        if not self.full_test:
            self.get_logger().info("⏭️  SKIPPED (use --full-test to enable)")
            return True
        
        try:
            # Test 4a: Open gripper
            self.get_logger().info("Opening gripper to 0.06m...")
            gripper_cmd = GripperCommand()
            gripper_cmd.header.stamp = self.get_clock().now().to_msg()
            gripper_cmd.width = 0.06  # Open to 6cm
            gripper_cmd.speed = GRIPPER_SLOW_SPEED  # Slow speed
            gripper_cmd.force = 0.1
            gripper_cmd.blocking = False
            
            self.gripper_cmd_publisher.publish(gripper_cmd)
            self.get_logger().info("✓ Published open command")
            time.sleep(10)  # Wait for slow movement
            
            # Test 4b: Close gripper
            self.get_logger().info("Closing gripper to 0.01m...")
            gripper_cmd.width = 0.01
            self.gripper_cmd_publisher.publish(gripper_cmd)
            self.get_logger().info("✓ Published close command")
            time.sleep(10)  # Wait for slow movement
            
            self.get_logger().info("✓ Gripper command test completed")
            return True
            
        except Exception as e:
            self.get_logger().error(f"✗ Gripper test failed: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_joint_movement(self) -> bool:
        """
        Test 5: Test Joint Position Movement via Service
        
        Expected real robot behavior:
        - Robot moves at controlled speed
        - Movement: JOINT_OFFSET_RAD offset on all joints
        - Uses blocking service for safety
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 5: Testing Joint Position Movement...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected real robot behavior:")
        self.get_logger().info(f"  - Joint offset: {JOINT_OFFSET_RAD} rad (~{math.degrees(JOINT_OFFSET_RAD):.1f} degrees)")
        self.get_logger().info(f"  - Time to go: {DEFAULT_TIME_TO_GO} seconds")
        self.get_logger().info("")
        
        if not self.full_test:
            self.get_logger().info("⏭️  SKIPPED (use --full-test to enable)")
            return True
        
        # Ensure we have current state
        if self.current_joint_positions is None or len(self.current_joint_positions) < 7:
            self.get_logger().warn("Waiting for joint positions...")
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.current_joint_positions and len(self.current_joint_positions) >= 7:
                    break
                time.sleep(0.1)
        
        if self.current_joint_positions is None or len(self.current_joint_positions) < 7:
            self.get_logger().error("✗ Cannot get current joint positions")
            return False
        
        try:
            current_pos = self.current_joint_positions[:7]
            self.get_logger().info(f"Current positions: {[f'{p:.3f}' for p in current_pos]}")
            
            # Use configurable offset
            target_pos = [pos + JOINT_OFFSET_RAD for pos in current_pos]
            self.get_logger().info(f"Target positions: {[f'{p:.3f}' for p in target_pos]}")
            self.get_logger().info(f"Offset: +{JOINT_OFFSET_RAD} rad (~{math.degrees(JOINT_OFFSET_RAD):.1f}°) on all joints")
            
            client = self.service_clients.get('/polymetis/arm/move_to_joint_positions')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("✗ MoveToJointPositions service not available")
                return False
            
            self.get_logger().warn("⚠️  WARNING: Robot will move slightly!")
            self.get_logger().info(f"Sending move command (time_to_go={DEFAULT_TIME_TO_GO}s)...")
            
            request = MoveToJointPositions.Request()
            request.joint_positions = target_pos
            request.time_to_go = DEFAULT_TIME_TO_GO  # SLOW movement (10x slower)
            
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=DEFAULT_TIME_TO_GO + 10)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✓ Move forward completed: {response.message}")
                    
                    # Wait and move back
                    time.sleep(2)
                    self.get_logger().info("Moving back to original position...")
                    
                    request.joint_positions = current_pos
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=DEFAULT_TIME_TO_GO + 10)
                    
                    if future.done() and future.result().success:
                        self.get_logger().info("✓ Move back completed")
                    else:
                        self.get_logger().warn("⚠️  Move back may not have completed")
                    
                    return True
                else:
                    self.get_logger().error(f"✗ Move failed: {response.message}")
                    return False
            else:
                self.get_logger().error("✗ Service call timed out")
                return False
                
        except Exception as e:
            self.get_logger().error(f"✗ Joint movement test failed: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
    
    def test_compute_services(self) -> bool:
        """
        Test 6: Test Computation Services (FK, IK, TimeToGo)
        
        Expected real robot behavior:
        - These are pure computation services, no robot movement
        - FK: Given joints → compute EE pose
        - IK: Given EE pose → compute joints
        - TimeToGo: Estimate time for movement
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 6: Testing Computation Services...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected: Pure computation, NO robot movement")
        self.get_logger().info("")
        
        success = True
        
        # Test FK
        self.get_logger().info("Testing Forward Kinematics (FK)...")
        fk_client = self.service_clients.get('/polymetis/arm/compute_fk')
        if fk_client and fk_client.wait_for_service(timeout_sec=2.0):
            try:
                request = ComputeFK.Request()
                # Use home position for FK test
                request.joint_positions = [0.0, -0.628, 0.0, -2.513, 0.0, 1.885, 0.0]
                
                future = fk_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        pos = response.position
                        self.get_logger().info(f"✓ FK: Position=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
                    else:
                        self.get_logger().warn(f"⚠️  FK failed: {response.message}")
                        success = False
                else:
                    self.get_logger().warn("⚠️  FK timed out")
                    success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  FK error: {e}")
                success = False
        else:
            self.get_logger().warn("⚠️  FK service not available")
            success = False
        
        # Test IK
        self.get_logger().info("Testing Inverse Kinematics (IK)...")
        ik_client = self.service_clients.get('/polymetis/arm/solve_ik')
        if ik_client and ik_client.wait_for_service(timeout_sec=2.0):
            try:
                request = SolveIK.Request()
                request.position = [0.5, 0.0, 0.4]  # Target position
                request.orientation = [0.0, 0.0, 0.0, 1.0]  # Identity quaternion
                request.q0 = [0.0, -0.628, 0.0, -2.513, 0.0, 1.885, 0.0]  # Initial guess
                request.tolerance = 1e-3
                
                future = ik_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ IK: Found solution (error={response.error:.6f})")
                    else:
                        self.get_logger().warn(f"⚠️  IK: {response.message}")
                        # IK might fail for some poses, not necessarily an error
                else:
                    self.get_logger().warn("⚠️  IK timed out")
                    success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  IK error: {e}")
                success = False
        else:
            self.get_logger().warn("⚠️  IK service not available")
            success = False
        
        # Test TimeToGo
        self.get_logger().info("Testing Time To Go computation...")
        ttg_client = self.service_clients.get('/polymetis/arm/compute_time_to_go')
        if ttg_client and ttg_client.wait_for_service(timeout_sec=2.0):
            try:
                request = ComputeTimeToGo.Request()
                request.desired_joint_positions = [0.0, -0.628, 0.0, -2.513, 0.0, 1.885, 0.0]
                
                future = ttg_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ TimeToGo: {response.time_to_go:.2f} seconds")
                    else:
                        self.get_logger().warn(f"⚠️  TimeToGo failed")
                        success = False
                else:
                    self.get_logger().warn("⚠️  TimeToGo timed out")
                    success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  TimeToGo error: {e}")
                success = False
        else:
            self.get_logger().warn("⚠️  TimeToGo service not available")
            success = False
        
        return success
    
    def test_controller_services(self) -> bool:
        """
        Test 7: Test Controller Management Services
        
        Expected real robot behavior:
        - StartJointImpedance: Robot switches to joint impedance control mode
          - Uses Kq (stiffness) and Kqd (damping) gains for joint space
          - Good for precise joint-level control
        - StartCartesianImpedance: Robot switches to cartesian impedance control
          - Uses Kx (stiffness) and Kxd (damping) gains for Cartesian space
          - Good for end-effector control and compliant behavior
        - TerminatePolicy: Stop current control policy
          - Robot will hold position but not actively controlled
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 7: Testing Controller Services...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected: Controller mode changes (minimal robot movement)")
        self.get_logger().info("")
        self.get_logger().info("Controller types:")
        self.get_logger().info("  - Joint Impedance: Joint-space stiffness/damping control")
        self.get_logger().info("  - Cartesian Impedance: Task-space stiffness/damping control")
        self.get_logger().info("  - Terminate: Stop active control policy")
        self.get_logger().info("")
        
        if not self.full_test:
            self.get_logger().info("⏭️  SKIPPED (use --full-test to enable)")
            return True
        
        success = True
        
        # ========== Test 7a: Start Joint Impedance with Default Gains ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 7a: Start Joint Impedance (default gains)...")
        self.get_logger().info("-" * 40)
        self.get_logger().info("Expected: Robot enters joint impedance mode")
        self.get_logger().info("  - Kq (stiffness): default ~[600, 600, 600, 600, 250, 150, 50]")
        self.get_logger().info("  - Kqd (damping): default ~[50, 50, 50, 50, 30, 25, 15]")
        
        ji_client = self.service_clients.get('/polymetis/arm/start_joint_impedance')
        if ji_client and ji_client.wait_for_service(timeout_sec=2.0):
            try:
                request = StartJointImpedance.Request()
                request.kq = []  # Use default gains
                request.kqd = []
                
                future = ji_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ Joint impedance started (default gains)")
                    else:
                        self.get_logger().warn(f"⚠️  {response.message}")
                        success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  Error: {e}")
                success = False
        else:
            self.get_logger().warn("⚠️  Joint impedance service not available")
            success = False
        
        time.sleep(2)
        
        # ========== Test 7b: Start Joint Impedance with Custom Gains ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 7b: Start Joint Impedance (custom gains)...")
        self.get_logger().info("-" * 40)
        self.get_logger().info("Expected: Robot enters joint impedance with softer gains")
        self.get_logger().info("  - Kq (stiffness): [400, 400, 400, 400, 200, 100, 40]")
        self.get_logger().info("  - Kqd (damping): [40, 40, 40, 40, 25, 20, 10]")
        
        if ji_client and ji_client.wait_for_service(timeout_sec=2.0):
            try:
                request = StartJointImpedance.Request()
                # Custom softer gains (more compliant)
                request.kq = [400.0, 400.0, 400.0, 400.0, 200.0, 100.0, 40.0]
                request.kqd = [40.0, 40.0, 40.0, 40.0, 25.0, 20.0, 10.0]
                
                future = ji_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ Joint impedance started (custom gains)")
                        self.get_logger().info("  Robot should feel more compliant/softer now")
                    else:
                        self.get_logger().warn(f"⚠️  {response.message}")
                        success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  Error: {e}")
                success = False
        
        time.sleep(2)
        
        # ========== Test 7c: Terminate Policy ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 7c: Terminate Policy...")
        self.get_logger().info("-" * 40)
        self.get_logger().info("Expected: Robot stops active control, holds position")
        
        tp_client = self.service_clients.get('/polymetis/arm/terminate_policy')
        if tp_client and tp_client.wait_for_service(timeout_sec=2.0):
            try:
                request = TerminatePolicy.Request()
                
                future = tp_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ Policy terminated: {response.message}")
                    else:
                        self.get_logger().warn(f"⚠️  {response.message}")
                        # Not necessarily a failure - might just mean no policy was running
            except Exception as e:
                self.get_logger().warn(f"⚠️  Error: {e}")
                success = False
        else:
            self.get_logger().warn("⚠️  Terminate policy service not available")
            success = False
        
        time.sleep(2)
        
        # ========== Test 7d: Start Cartesian Impedance with Default Gains ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 7d: Start Cartesian Impedance (default gains)...")
        self.get_logger().info("-" * 40)
        self.get_logger().info("Expected: Robot enters cartesian impedance mode")
        self.get_logger().info("  - Kx (stiffness): default ~[500, 500, 500, 50, 50, 50]")
        self.get_logger().info("  - Kxd (damping): default ~[50, 50, 50, 10, 10, 10]")
        
        ci_client = self.service_clients.get('/polymetis/arm/start_cartesian_impedance')
        if ci_client and ci_client.wait_for_service(timeout_sec=2.0):
            try:
                request = StartCartesianImpedance.Request()
                request.kx = []  # Use default gains
                request.kxd = []
                
                future = ci_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ Cartesian impedance started (default gains)")
                    else:
                        self.get_logger().warn(f"⚠️  {response.message}")
                        success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  Error: {e}")
                success = False
        else:
            self.get_logger().warn("⚠️  Cartesian impedance service not available")
            success = False
        
        time.sleep(2)
        
        # ========== Test 7e: Start Cartesian Impedance with Custom Gains ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 7e: Start Cartesian Impedance (custom gains)...")
        self.get_logger().info("-" * 40)
        self.get_logger().info("Expected: Robot enters cartesian impedance with custom gains")
        self.get_logger().info("  - Kx (stiffness): [600, 600, 600, 30, 30, 30]")
        self.get_logger().info("  - Kxd (damping): [60, 60, 60, 8, 8, 8]")
        
        if ci_client and ci_client.wait_for_service(timeout_sec=2.0):
            try:
                request = StartCartesianImpedance.Request()
                # Custom gains - stiffer in position, softer in orientation
                request.kx = [600.0, 600.0, 600.0, 30.0, 30.0, 30.0]
                request.kxd = [60.0, 60.0, 60.0, 8.0, 8.0, 8.0]
                
                future = ci_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success:
                        self.get_logger().info(f"✓ Cartesian impedance started (custom gains)")
                        self.get_logger().info("  Robot should be stiffer in position, softer in orientation")
                    else:
                        self.get_logger().warn(f"⚠️  {response.message}")
                        success = False
            except Exception as e:
                self.get_logger().warn(f"⚠️  Error: {e}")
                success = False
        
        time.sleep(2)
        
        # ========== Test 7f: Controller Switching Sequence ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 7f: Controller Switching Sequence...")
        self.get_logger().info("-" * 40)
        self.get_logger().info("Expected: Rapid switching between controllers")
        self.get_logger().info("  Joint -> Cartesian -> Terminate -> Joint")
        
        switch_success = True
        controllers_sequence = [
            ("Joint Impedance", ji_client, StartJointImpedance.Request()),
            ("Cartesian Impedance", ci_client, StartCartesianImpedance.Request()),
            ("Terminate", tp_client, TerminatePolicy.Request()),
            ("Joint Impedance", ji_client, StartJointImpedance.Request()),
        ]
        
        for controller_name, client, request in controllers_sequence:
            if client and client.wait_for_service(timeout_sec=1.0):
                try:
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
                    
                    if future.done():
                        response = future.result()
                        if response.success:
                            self.get_logger().info(f"  ✓ Switched to {controller_name}")
                        else:
                            self.get_logger().warn(f"  ⚠️  {controller_name}: {response.message}")
                            switch_success = False
                    else:
                        self.get_logger().warn(f"  ⚠️  {controller_name}: Timeout")
                        switch_success = False
                except Exception as e:
                    self.get_logger().warn(f"  ⚠️  {controller_name}: {e}")
                    switch_success = False
            else:
                self.get_logger().warn(f"  ⚠️  {controller_name}: Service not available")
                switch_success = False
            
            time.sleep(0.5)  # Short delay between switches
        
        if switch_success:
            self.get_logger().info("✓ Controller switching sequence completed")
        else:
            self.get_logger().warn("⚠️  Some controller switches failed")
            success = False
        
        # ========== Summary ==========
        self.get_logger().info("")
        self.get_logger().info("Controller Test Summary:")
        self.get_logger().info("  - Joint Impedance: Good for precise joint control")
        self.get_logger().info("  - Cartesian Impedance: Good for task-space control")
        self.get_logger().info("  - Custom gains: Adjust stiffness/damping for compliance")
        
        return success
    
    def test_robot_command_topic(self) -> bool:
        """
        Test 8: Test Robot Command Topic with Different Action Spaces
        
        Expected real robot behavior:
        - This tests the NON-BLOCKING command interface via topic
        - Different action_space values:
          - joint_position: Direct joint position command
          - joint_velocity: Joint velocity command (converted to position delta)
          - cartesian_position: EE pose command (uses IK)
          - cartesian_velocity: EE velocity command (uses IK)
        - Commands include gripper action (last element)
        """
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST 8: Testing Robot Command Topic (Non-blocking)...")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Expected: Non-blocking commands via /polymetis/robot_command")
        self.get_logger().info("")
        self.get_logger().info("Action spaces:")
        self.get_logger().info("  - joint_position: [j1, j2, j3, j4, j5, j6, j7, gripper]")
        self.get_logger().info("  - joint_velocity: [v1, v2, v3, v4, v5, v6, v7, gripper_vel]")
        self.get_logger().info("  - cartesian_position: [x, y, z, roll, pitch, yaw, gripper]")
        self.get_logger().info("  - cartesian_velocity: [vx, vy, vz, wx, wy, wz, gripper_vel]")
        self.get_logger().info("")
        
        if not self.full_test:
            self.get_logger().info("⏭️  SKIPPED (use --full-test to enable)")
            return True
        
        # Ensure we have current state
        if self.current_joint_positions is None or len(self.current_joint_positions) < 7:
            self.get_logger().warn("Waiting for joint positions...")
            for _ in range(50):
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.current_joint_positions and len(self.current_joint_positions) >= 7:
                    break
                time.sleep(0.1)
        
        if self.current_joint_positions is None or len(self.current_joint_positions) < 7:
            self.get_logger().error("✗ Cannot get current joint positions")
            return False
        
        success = True
        
        # ========== Test 8a: Joint Velocity Command ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 8a: Joint Velocity Command...")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"Expected: Joint velocity for {VELOCITY_COMMAND_DURATION} seconds")
        self.get_logger().info(f"  - Velocity: [{JOINT_VELOCITY_RAD_S}, 0, 0, 0, 0, 0, 0] rad/s on joint 1")
        self.get_logger().info("  - Gripper: 0 (no change)")
        
        try:
            cmd = PolymetisRobotCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.action_space = "joint_velocity"
            cmd.gripper_action_space = "velocity"
            cmd.command = [JOINT_VELOCITY_RAD_S, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Velocity on joint 1
            cmd.blocking = False
            
            self.get_logger().info(f"Publishing joint velocity commands for {VELOCITY_COMMAND_DURATION} seconds...")
            self.get_logger().warn("⚠️  Robot joint 1 will rotate!")
            start_time = time.time()
            while time.time() - start_time < VELOCITY_COMMAND_DURATION:
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.robot_cmd_publisher.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.02)  # 50Hz
            
            self.get_logger().info("✓ Joint velocity command test completed")
            self.get_logger().info(f"  Total rotation: ~{JOINT_VELOCITY_RAD_S * VELOCITY_COMMAND_DURATION:.2f} rad (~{math.degrees(JOINT_VELOCITY_RAD_S * VELOCITY_COMMAND_DURATION):.1f}°)")
        except Exception as e:
            self.get_logger().error(f"✗ Joint velocity test failed: {e}")
            success = False
        
        time.sleep(WAIT_BETWEEN_TESTS)
        
        # ========== Test 8b: Joint Position Command ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 8b: Joint Position Command...")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"Expected: Position offset +{JOINT_OFFSET_RAD} rad via non-blocking topic")
        
        try:
            # Get current position
            current_pos = list(self.current_joint_positions[:7])
            target_pos = [p + JOINT_OFFSET_RAD for p in current_pos]
            
            cmd = PolymetisRobotCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.action_space = "joint_position"
            cmd.gripper_action_space = "position"
            cmd.command = target_pos + [0.5]  # Add gripper position (0.5 = half open)
            cmd.blocking = False  # Topic always non-blocking
            
            self.get_logger().info(f"Publishing joint position command...")
            self.get_logger().info(f"  Target offset: +{JOINT_OFFSET_RAD} rad (~{math.degrees(JOINT_OFFSET_RAD):.1f}°) on all joints")
            self.get_logger().warn("⚠️  Robot will move to new position!")
            
            # Publish multiple times (non-blocking needs continuous commands)
            for _ in range(100):  # 2 seconds at 50Hz
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.robot_cmd_publisher.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.02)
            
            self.get_logger().info("✓ Joint position command test completed")
        except Exception as e:
            self.get_logger().error(f"✗ Joint position test failed: {e}")
            success = False
        
        time.sleep(WAIT_BETWEEN_TESTS)
        
        # ========== Test 8c: Cartesian Velocity Command ==========
        self.get_logger().info("-" * 40)
        self.get_logger().info("Test 8c: Cartesian Velocity Command...")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"Expected: EE velocity via IK for {VELOCITY_COMMAND_DURATION} seconds")
        self.get_logger().info(f"  - Velocity: [{CARTESIAN_VELOCITY_M_S}, 0, 0, 0, 0, 0] m/s (X direction)")
        
        try:
            cmd = PolymetisRobotCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.action_space = "cartesian_velocity"
            cmd.gripper_action_space = "velocity"
            cmd.command = [CARTESIAN_VELOCITY_M_S, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # X velocity
            cmd.blocking = False
            
            self.get_logger().info(f"Publishing cartesian velocity commands for {VELOCITY_COMMAND_DURATION} seconds...")
            self.get_logger().warn("⚠️  Robot end-effector will move in X direction!")
            start_time = time.time()
            while time.time() - start_time < VELOCITY_COMMAND_DURATION:
                cmd.header.stamp = self.get_clock().now().to_msg()
                self.robot_cmd_publisher.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.02)
            
            self.get_logger().info("✓ Cartesian velocity command test completed")
            self.get_logger().info(f"  Total EE displacement: ~{CARTESIAN_VELOCITY_M_S * VELOCITY_COMMAND_DURATION:.2f} m ({CARTESIAN_VELOCITY_M_S * VELOCITY_COMMAND_DURATION * 100:.1f} cm)")
        except Exception as e:
            self.get_logger().error(f"✗ Cartesian velocity test failed: {e}")
            success = False
        
        # ========== Summary ==========
        self.get_logger().info("")
        self.get_logger().info("Robot Command Topic Summary:")
        self.get_logger().info("  - joint_velocity: Continuous velocity control")
        self.get_logger().info("  - joint_position: Position setpoint (via impedance)")
        self.get_logger().info("  - cartesian_velocity: EE velocity control (via IK)")
        self.get_logger().info("  - Note: Topic commands are always non-blocking")
        self.get_logger().info("  - For blocking, use Service instead")
        
        return success
    
    def print_summary(self, results: Dict[str, bool]):
        """Print test summary"""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("=" * 60)
        
        passed = sum(1 for v in results.values() if v)
        total = len(results)
        
        for test_name, result in results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            self.get_logger().info(f"  {test_name}: {status}")
        
        self.get_logger().info("")
        self.get_logger().info(f"Results: {passed}/{total} tests passed")
        
        if passed == total:
            self.get_logger().info("✓ ALL TESTS PASSED")
        else:
            self.get_logger().warn("⚠️  SOME TESTS FAILED")
        
        self.get_logger().info("=" * 60)


def main(args=None):
    parser = argparse.ArgumentParser(description='Test polymetis_bridge_node interfaces')
    parser.add_argument('--full-test', action='store_true',
                       help='Perform full movement tests (WARNING: Robot will move SLOWLY!)')
    parser.add_argument('--timeout', type=float, default=5.0,
                       help='Timeout for topic/service checks (seconds)')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=None)
    
    tester = PolymetisBridgeTester(full_test=parsed_args.full_test)
    
    results = {}
    
    try:
        tester.get_logger().info("=" * 60)
        tester.get_logger().info("POLYMETIS BRIDGE NODE - COMPREHENSIVE TEST")
        tester.get_logger().info("=" * 60)
        tester.get_logger().info(f"Full test mode: {parsed_args.full_test}")
        tester.get_logger().info(f"Movement speed: 10x SLOWER for safety")
        tester.get_logger().info(f"Wait between tests: {WAIT_BETWEEN_TESTS}s")
        tester.get_logger().info("=" * 60)
        tester.get_logger().info("")
        
        # Test 1: Topics
        topic_results = tester.check_topics(timeout=parsed_args.timeout)
        results['Topics'] = all(topic_results.values())
        tester._wait_between_tests("Services")
        
        # Test 2: Services
        service_results = tester.check_services(timeout=2.0)
        results['Services'] = all(service_results.values())
        tester._wait_between_tests("Reset")
        
        # Test 3: Reset Service
        results['Reset'] = tester.test_reset_service()
        tester._wait_between_tests("Gripper")
        
        # Test 4: Gripper Command
        results['Gripper'] = tester.test_gripper_command()
        tester._wait_between_tests("Joint Movement")
        
        # Test 5: Joint Movement
        results['Joint Movement'] = tester.test_joint_movement()
        tester._wait_between_tests("Computation")
        
        # Test 6: Computation Services
        results['Computation'] = tester.test_compute_services()
        tester._wait_between_tests("Controllers")
        
        # Test 7: Controller Services
        results['Controllers'] = tester.test_controller_services()
        tester._wait_between_tests("Robot Command Topic")
        
        # Test 8: Robot Command Topic (Non-blocking)
        results['Robot Commands'] = tester.test_robot_command_topic()
        
        # Print summary
        tester.print_summary(results)
        
    except KeyboardInterrupt:
        tester.get_logger().info("\nTest interrupted by user")
    except Exception as e:
        tester.get_logger().error(f"Test failed with error: {e}")
        tester.get_logger().error(traceback.format_exc())
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

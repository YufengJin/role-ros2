#!/usr/bin/env python3
"""
Test script to verify Franka robot arm and gripper controllers are correctly started.

This script checks:
1. Controller manager is running
2. Required controllers are loaded and active
3. Joint states are being published
4. Robot state is being published
5. Gripper controller is available (if enabled)

Usage:
    python3 test_franka_controllers.py [--arm-id fr3] [--timeout 30]
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from controller_manager_msgs.srv import ListControllers
    from sensor_msgs.msg import JointState
    from franka_msgs.msg import FrankaRobotState
    from std_msgs.msg import String
except ImportError as e:
    print(f"ERROR: Failed to import required ROS2 packages: {e}")
    print("Please ensure ROS2 environment is sourced:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source ros2_ws/install/setup.bash")
    sys.exit(1)


class ControllerTester(Node):
    def __init__(self, arm_id='fr3', timeout=30):
        super().__init__('franka_controller_tester')
        self.arm_id = arm_id
        self.timeout = timeout
        self.test_results = {
            'controller_manager': False,
            'joint_state_broadcaster': False,
            'franka_robot_state_broadcaster': False,
            'arm_controller': False,
            'gripper_controller': False,
            'joint_states': False,
            'robot_state': False,
        }
        
        # Service client for controller manager
        self.controller_list_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        
        # Subscribers for state checking
        self.joint_state_received = False
        self.robot_state_received = False
        
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )
        
        self.robot_state_sub = self.create_subscription(
            FrankaRobotState, f'{arm_id}/robot_state', 
            self.robot_state_callback, 10
        )
    
    def joint_state_callback(self, msg):
        self.joint_state_received = True
        self.test_results['joint_states'] = True
    
    def robot_state_callback(self, msg):
        self.robot_state_received = True
        self.test_results['robot_state'] = True
    
    def check_controllers(self):
        """Check if controllers are loaded and active"""
        if not self.controller_list_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Controller manager service not available!')
            return False
        
        request = ListControllers.Request()
        future = self.controller_list_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.get_logger().error('Failed to get controller list')
            return False
        
        controllers = future.result().controller
        active_controllers = [c.name for c in controllers if c.state == 'active']
        loaded_controllers = [c.name for c in controllers]
        
        self.get_logger().info(f'Loaded controllers: {loaded_controllers}')
        self.get_logger().info(f'Active controllers: {active_controllers}')
        
        # Check required controllers
        required_controllers = [
            'joint_state_broadcaster',
            'franka_robot_state_broadcaster',
        ]
        
        # Check arm controller (name depends on configuration)
        arm_controllers = [c for c in active_controllers if 'arm_controller' in c or 
                          'joint_trajectory_controller' in c or 
                          'example_controller' in c]
        if arm_controllers:
            self.test_results['arm_controller'] = True
            self.get_logger().info(f'✓ Arm controller found: {arm_controllers[0]}')
        else:
            self.get_logger().warn('✗ No arm controller found in active controllers')
        
        # Check gripper controller
        gripper_controllers = [c for c in active_controllers if 'gripper' in c.lower()]
        if gripper_controllers:
            self.test_results['gripper_controller'] = True
            self.get_logger().info(f'✓ Gripper controller found: {gripper_controllers[0]}')
        else:
            self.get_logger().warn('✗ No gripper controller found (may be disabled)')
        
        # Check required controllers
        for req_ctrl in required_controllers:
            if req_ctrl in active_controllers:
                self.test_results[req_ctrl] = True
                self.get_logger().info(f'✓ {req_ctrl} is active')
            else:
                self.get_logger().error(f'✗ {req_ctrl} is not active!')
        
        self.test_results['controller_manager'] = True
        return True
    
    def wait_for_states(self):
        """Wait for joint states and robot state messages"""
        self.get_logger().info('Waiting for joint states and robot state messages...')
        start_time = time.time()
        
        while (time.time() - start_time) < self.timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_state_received and self.robot_state_received:
                return True
        
        return False
    
    def run_tests(self):
        """Run all tests"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Franka Robot Controller Test')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Arm ID: {self.arm_id}')
        self.get_logger().info(f'Timeout: {self.timeout}s')
        self.get_logger().info('')
        
        # Test 1: Check controller manager and controllers
        self.get_logger().info('Test 1: Checking controllers...')
        if not self.check_controllers():
            self.get_logger().error('Failed to check controllers!')
            return False
        
        # Test 2: Wait for state messages
        self.get_logger().info('')
        self.get_logger().info('Test 2: Checking state messages...')
        if self.wait_for_states():
            self.get_logger().info('✓ Joint states received')
            self.get_logger().info('✓ Robot state received')
        else:
            self.get_logger().warn('⚠ Timeout waiting for state messages')
            if not self.joint_state_received:
                self.get_logger().error('✗ Joint states not received')
            if not self.robot_state_received:
                self.get_logger().warn('⚠ Robot state not received (may be using fake hardware)')
        
        # Print summary
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Test Summary')
        self.get_logger().info('=' * 60)
        
        all_passed = True
        for test_name, result in self.test_results.items():
            status = '✓ PASS' if result else '✗ FAIL'
            self.get_logger().info(f'{status}: {test_name}')
            if not result and test_name in ['controller_manager', 'joint_state_broadcaster', 
                                            'arm_controller', 'joint_states']:
                all_passed = False
        
        self.get_logger().info('')
        if all_passed:
            self.get_logger().info('✓ All critical tests passed!')
            return True
        else:
            self.get_logger().error('✗ Some critical tests failed!')
            return False


def main():
    parser = argparse.ArgumentParser(
        description='Test Franka robot controllers'
    )
    parser.add_argument(
        '--arm-id',
        default='fr3',
        help='Arm ID (default: fr3)'
    )
    parser.add_argument(
        '--timeout',
        type=int,
        default=30,
        help='Timeout in seconds (default: 30)'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        tester = ControllerTester(arm_id=args.arm_id, timeout=args.timeout)
        success = tester.run_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
        sys.exit(1)
    except Exception as e:
        print(f'ERROR: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


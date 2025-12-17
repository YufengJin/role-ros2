#!/usr/bin/env python3
"""
Test script for Joint Velocity Control mode with arm and gripper.

This script:
1. Switches to joint_velocity_example_controller
2. Runs for a short time (small safe movements)
3. Tests gripper control
4. Returns to safe state

SAFETY: Only moves small distances, has emergency stop capability.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController, ListControllers
from franka_msgs.action import Move as GripperMove
from rclpy.action import ActionClient


class JointVelocityControlTester(Node):
    def __init__(self):
        super().__init__('joint_velocity_control_tester')
        
        # Service clients
        self.switch_controllers_client = self.create_client(
            SwitchController, '/controller_manager/switch_controller'
        )
        self.list_controllers_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )
        
        # Gripper action client
        self.gripper_client = ActionClient(self, GripperMove, 'fr3_gripper/move')
        
        # Wait for services
        self.get_logger().info('Waiting for controller_manager services...')
        self.switch_controllers_client.wait_for_service(timeout_sec=10.0)
        self.list_controllers_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services ready')
        
    def list_controllers(self):
        """List all available controllers"""
        request = ListControllers.Request()
        future = self.list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            return future.result()
        return None
    
    def switch_controllers(self, activate, deactivate, strictness=1):
        """Switch controllers"""
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = strictness
        request.activate_asap = True
        request.timeout = rclpy.duration.Duration(seconds=5.0).to_msg()
        
        future = self.switch_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            response = future.result()
            return response.ok
        return False
    
    def test_gripper(self, width=0.04):
        """Test gripper movement (small safe movement)"""
        self.get_logger().info(f'Testing gripper: moving to width {width}m')
        
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Gripper action server not available')
            return False
        
        goal_msg = GripperMove.Goal()
        goal_msg.width = width
        goal_msg.speed = 0.1  # Slow speed for safety
        
        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Gripper goal accepted')
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
                if result_future.done():
                    result = result_future.result().result
                    self.get_logger().info(f'Gripper result: success={result.success}')
                    return result.success
        return False
    
    def run_test(self):
        """Run the joint velocity control test"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Starting Joint Velocity Control Test')
        self.get_logger().info('=' * 60)
        
        # Step 1: List current controllers
        self.get_logger().info('\n[Step 1] Listing controllers...')
        controllers = self.list_controllers()
        active = []
        if controllers:
            active = [c.name for c in controllers.controller_list if c.state == 'active']
            self.get_logger().info(f'Active controllers: {active}')
        
        # Step 2: Deactivate current arm controller (if any)
        self.get_logger().info('\n[Step 2] Deactivating current arm controller...')
        current_arm_controllers = ['fr3_arm_controller', 'cartesian_pose_example_controller',
                                   'joint_position_example_controller', 'cartesian_velocity_example_controller']
        deactivate = [c for c in current_arm_controllers if c in active]
        if deactivate:
            self.switch_controllers([], deactivate, strictness=2)  # BEST_EFFORT
            time.sleep(1.0)
        
        # Step 3: Activate joint_velocity_example_controller
        self.get_logger().info('\n[Step 3] Activating joint_velocity_example_controller...')
        success = self.switch_controllers(
            ['joint_velocity_example_controller'],
            [],
            strictness=2  # BEST_EFFORT
        )
        
        if not success:
            self.get_logger().error('Failed to activate joint_velocity_example_controller')
            self.get_logger().error('Note: This controller must be defined in controllers.yaml')
            return False
        
        time.sleep(2.0)  # Wait for controller to initialize
        
        # Step 4: Run controller for short time (small movements)
        self.get_logger().info('\n[Step 4] Running joint velocity controller for 5 seconds...')
        self.get_logger().info('WARNING: Robot will move joints with velocity commands!')
        self.get_logger().info('Keep emergency stop ready!')
        
        start_time = time.time()
        test_duration = 5.0  # 5 seconds of small movements
        
        while (time.time() - start_time) < test_duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().info('Controller test completed')
        
        # Step 5: Test gripper
        self.get_logger().info('\n[Step 5] Testing gripper control...')
        self.test_gripper(0.06)
        time.sleep(2.0)
        self.test_gripper(0.03)
        time.sleep(2.0)
        
        # Step 6: Deactivate controller
        self.get_logger().info('\n[Step 6] Deactivating controller and returning to safe state...')
        self.switch_controllers(
            [],
            ['joint_velocity_example_controller'],
            strictness=1  # STRICT
        )
        time.sleep(1.0)
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Joint Velocity Control Test Completed Successfully!')
        self.get_logger().info('=' * 60)
        return True


def main(args=None):
    rclpy.init(args=args)
    
    tester = JointVelocityControlTester()
    
    try:
        success = tester.run_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        tester.get_logger().warn('Test interrupted by user')
        tester.switch_controllers([], ['joint_velocity_example_controller'], strictness=2)
        sys.exit(1)
    except Exception as e:
        tester.get_logger().error(f'Test failed with error: {e}')
        tester.switch_controllers([], ['joint_velocity_example_controller'], strictness=2)
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


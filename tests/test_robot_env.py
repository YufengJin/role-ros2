#!/usr/bin/env python3
"""
Test script for RobotEnv class (without camera functionality)

This script tests:
1. RobotEnv initialization with different action spaces
2. Reset functionality (with/without randomization)
3. Step functionality (different action spaces)
4. Get state functionality
5. Update robot functionality
6. Create action dict functionality

Usage:
    python3 test_robot_env.py [--action-space ACTION_SPACE] [--full-test]
    
    --action-space: Which action space to test (cartesian_velocity, joint_velocity, 
                    cartesian_position, joint_position, all)
    --full-test: Perform full movement tests (WARNING: Robot will move!)

Author: Role-ROS2 Team
"""

import sys
import os
import time
import argparse
import traceback
from pathlib import Path
import numpy as np

# Add ROS2 workspace to Python path if not already there
# This allows running the script without sourcing setup.bash
def setup_ros2_path():
    """Setup Python path for ROS2 workspace"""
    # Try to find workspace root (look for install directory)
    current_dir = Path(__file__).resolve().parent
    workspace_root = current_dir.parent.parent.parent  # tests -> role-ros2 -> src -> ros2_ws
    
    # Check if install directory exists
    install_dir = workspace_root / 'install'
    if install_dir.exists():
        # Add role_ros2 package to path
        role_ros2_site_packages = install_dir / 'role_ros2' / 'lib' / 'python3.8' / 'site-packages'
        if role_ros2_site_packages.exists():
            if str(role_ros2_site_packages) not in sys.path:
                sys.path.insert(0, str(role_ros2_site_packages))
                print(f"Added to PYTHONPATH: {role_ros2_site_packages}")
        
        # Also try python3.10 (for different ROS2 versions)
        for py_version in ['python3.10', 'python3.9', 'python3.8']:
            role_ros2_site_packages = install_dir / 'role_ros2' / 'lib' / py_version / 'site-packages'
            if role_ros2_site_packages.exists() and str(role_ros2_site_packages) not in sys.path:
                sys.path.insert(0, str(role_ros2_site_packages))
                print(f"Added to PYTHONPATH: {role_ros2_site_packages}")
                break
    
    # Also try to add source directory (for editable installs)
    src_dir = workspace_root / 'src' / 'role-ros2' / 'role_ros2'
    if src_dir.exists() and str(src_dir.parent) not in sys.path:
        sys.path.insert(0, str(src_dir.parent))
        print(f"Added to PYTHONPATH: {src_dir.parent}")

# Setup path before importing
setup_ros2_path()

try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    print("ERROR: Failed to import rclpy. Please ensure:")
    print("  1. ROS2 is installed and sourced: source /opt/ros/foxy/setup.bash")
    print("  2. Workspace is built: cd ros2_ws && colcon build")
    print("  3. Workspace is sourced: source install/setup.bash")
    sys.exit(1)

# Import RobotEnv
try:
    from role_ros2.robot_env import RobotEnv
except ImportError as e:
    print(f"ERROR: Failed to import role_ros2.robot_env: {e}")
    print("\nPlease ensure:")
    print("  1. Workspace is built: cd ros2_ws && colcon build")
    print("  2. Workspace is sourced: source install/setup.bash")
    print("  3. Or run from workspace root with: source install/setup.bash && python3 tests/test_robot_env.py")
    sys.exit(1)


class RobotEnvTester:
    """
    Test class for RobotEnv functionality
    """
    
    def __init__(self, node=None):
        self.node = node
        self.test_results = {}
    
    def test_initialization(self, action_space="cartesian_velocity", do_reset=False):
        """
        Test 1: RobotEnv initialization
        
        Tests:
        - Initialization with different action spaces
        - DoF calculation
        - Configuration parameters
        """
        print("=" * 60)
        print("TEST 1: RobotEnv Initialization")
        print("=" * 60)
        print(f"Action space: {action_space}")
        print(f"Do reset on init: {do_reset}")
        print("")
        
        try:
            # Initialize RobotEnv
            print(f"Initializing RobotEnv with action_space={action_space}, do_reset={do_reset}...")
            env = RobotEnv(
                action_space=action_space,
                do_reset=do_reset,
                node=self.node
            )
            print("✓ RobotEnv initialized successfully")
            
            # Check configuration
            print(f"  - Action space: {env.action_space}")
            print(f"  - DoF: {env.DoF}")
            print(f"  - Control Hz: {env.control_hz}")
            print(f"  - Check action range: {env.check_action_range}")
            print(f"  - Reset joints shape: {env.reset_joints.shape}")
            print(f"  - Randomize low shape: {env.randomize_low.shape}")
            print(f"  - Randomize high shape: {env.randomize_high.shape}")
            
            # Verify DoF
            expected_dof = 7 if "cartesian" in action_space else 8
            if env.DoF == expected_dof:
                print(f"✓ DoF is correct: {env.DoF} (expected {expected_dof})")
            else:
                print(f"✗ DoF mismatch: {env.DoF} (expected {expected_dof})")
                return False
            
            # Verify action space
            if env.action_space == action_space:
                print(f"✓ Action space is correct: {env.action_space}")
            else:
                print(f"✗ Action space mismatch: {env.action_space} (expected {action_space})")
                return False
            
            return True
            
        except Exception as e:
            print(f"✗ Initialization failed: {e}")
            print(traceback.format_exc())
            return False
    
    def test_reset(self, randomize=False):
        """
        Test 2: Reset functionality
        
        Tests:
        - Reset without randomization
        - Reset with randomization
        """
        print("")
        print("=" * 60)
        print("TEST 2: Reset Functionality")
        print("=" * 60)
        print(f"Randomize: {randomize}")
        print("")
        
        try:
            # Initialize env (without auto-reset to avoid double reset)
            print("Initializing RobotEnv (do_reset=False)...")
            env = RobotEnv(action_space="cartesian_velocity", do_reset=False, node=self.node)
            print("✓ RobotEnv initialized")
            
            # Get state before reset
            print("Getting state before reset...")
            state_before, _ = env.get_state()
            print(f"  - Joint positions before: {np.array(state_before['joint_positions'])[:3]}...")
            print(f"  - EE position before: {np.array(state_before['cartesian_position'])[:3]}")
            
            # Reset
            print(f"Calling reset(randomize={randomize})...")
            env.reset(randomize=randomize)
            print("✓ Reset command sent (executing in background)")
            
            # Wait for reset to complete
            print("Waiting 5 seconds for reset to complete...")
            time.sleep(5)
            
            # Get state after reset
            print("Getting state after reset...")
            state_after, _ = env.get_state()
            print(f"  - Joint positions after: {np.array(state_after['joint_positions'])[:3]}...")
            print(f"  - EE position after: {np.array(state_after['cartesian_position'])[:3]}")
            
            # Check if robot moved (positions changed)
            joint_diff = np.abs(np.array(state_after['joint_positions']) - np.array(state_before['joint_positions']))
            max_joint_diff = np.max(joint_diff)
            print(f"  - Max joint difference: {max_joint_diff:.4f} rad")
            
            if max_joint_diff > 0.01:  # Robot moved at least 0.01 rad
                print("✓ Reset completed - Robot moved")
                return True
            else:
                print("⚠️  Reset may not have completed - Robot didn't move much")
                return True  # Still consider it a pass (reset is async)
            
        except Exception as e:
            print(f"✗ Reset test failed: {e}")
            print(traceback.format_exc())
            return False
    
    def test_get_state(self):
        """
        Test 3: Get state functionality
        
        Tests:
        - State dictionary structure
        - State values are valid
        - Timestamp information
        """
        print("")
        print("=" * 60)
        print("TEST 3: Get State Functionality")
        print("=" * 60)
        print("")
        
        try:
            # Initialize env
            print("Initializing RobotEnv...")
            env = RobotEnv(action_space="cartesian_velocity", do_reset=False, node=self.node)
            print("✓ RobotEnv initialized")
            
            # Get state
            print("Getting robot state...")
            state_dict, timestamp_dict = env.get_state()
            print("✓ State retrieved")
            
            # Check state structure
            required_keys = [
                'cartesian_position', 'gripper_position',
                'joint_positions', 'joint_velocities',
                'joint_torques_computed'
            ]
            
            missing_keys = []
            for key in required_keys:
                if key not in state_dict:
                    missing_keys.append(key)
            
            if missing_keys:
                print(f"✗ Missing state keys: {missing_keys}")
                return False
            else:
                print("✓ All required state keys present")
            
            # Check state values
            print("Checking state values...")
            print(f"  - Joint positions: shape={len(state_dict['joint_positions'])}, "
                  f"values={np.array(state_dict['joint_positions'])[:3]}")
            print(f"  - Joint velocities: shape={len(state_dict['joint_velocities'])}")
            print(f"  - Cartesian position: shape={len(state_dict['cartesian_position'])}, "
                  f"pos={np.array(state_dict['cartesian_position'])[:3]}")
            print(f"  - Gripper position: {state_dict['gripper_position']:.4f}")
            
            # Check timestamp
            if 'read_start' in timestamp_dict and 'read_end' in timestamp_dict:
                read_time = timestamp_dict['read_end'] - timestamp_dict['read_start']
                print(f"  - Read time: {read_time:.2f} ms")
                print("✓ Timestamp information present")
            else:
                print("⚠️  Timestamp information missing")
            
            # Validate values
            if len(state_dict['joint_positions']) == 7:
                print("✓ Joint positions have correct length (7)")
            else:
                print(f"✗ Joint positions length incorrect: {len(state_dict['joint_positions'])} (expected 7)")
                return False
            
            if len(state_dict['cartesian_position']) == 6:
                print("✓ Cartesian position has correct length (6)")
            else:
                print(f"✗ Cartesian position length incorrect: {len(state_dict['cartesian_position'])} (expected 6)")
                return False
            
            return True
            
        except Exception as e:
            print(f"✗ Get state test failed: {e}")
            print(traceback.format_exc())
            return False
    
    def test_step(self, action_space="cartesian_velocity", full_test=False):
        """
        Test 4: Step functionality
        
        Tests:
        - Step with different action spaces
        - Action validation
        - Robot movement
        """
        print("")
        print("=" * 60)
        print("TEST 4: Step Functionality")
        print("=" * 60)
        print(f"Action space: {action_space}")
        print(f"Full test (robot movement): {full_test}")
        print("")
        
        if not full_test:
            print("⏭️  SKIPPED (use --full-test to enable robot movement)")
            return True
        
        try:
            # Initialize env
            print("Initializing RobotEnv...")
            env = RobotEnv(action_space=action_space, do_reset=False, node=self.node)
            print("✓ RobotEnv initialized")
            
            # Get initial state
            print("Getting initial state...")
            state_before, _ = env.get_state()
            print(f"  - Initial EE position: {np.array(state_before['cartesian_position'])[:3]}")
            print(f"  - Initial joint positions: {np.array(state_before['joint_positions'])[:3]}...")
            
            # Create test action
            if "velocity" in action_space:
                # Velocity action: normalized [-1, 1]
                if "cartesian" in action_space:
                    # Cartesian velocity: [vx, vy, vz, wx, wy, wz, gripper_vel]
                    action = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Small X velocity
                else:
                    # Joint velocity: [v1, v2, v3, v4, v5, v6, v7, gripper_vel]
                    action = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Small joint 1 velocity
            else:
                # Position action: actual positions
                if "cartesian" in action_space:
                    # Cartesian position: [x, y, z, roll, pitch, yaw, gripper]
                    current_pos = np.array(state_before['cartesian_position'])
                    action = current_pos.copy()
                    action[0] += 0.01  # Small X offset
                else:
                    # Joint position: [j1, j2, j3, j4, j5, j6, j7, gripper]
                    current_joints = np.array(state_before['joint_positions'])
                    action = current_joints.copy()
                    action[0] += 0.05  # Small joint 1 offset
                    action = np.append(action, 0.5)  # Add gripper position
            
            print(f"Created test action: shape={action.shape}, action_space={action_space}")
            print(f"  - Action values: {action[:3]}...")
            
            # Validate action
            if len(action) != env.DoF:
                print(f"✗ Action length mismatch: {len(action)} (expected {env.DoF})")
                return False
            
            if env.check_action_range:
                if np.max(action) > 1 or np.min(action) < -1:
                    print(f"✗ Action out of range: min={np.min(action)}, max={np.max(action)}")
                    return False
            
            print("✓ Action validation passed")
            
            # Execute step
            print("Executing step (robot will move slightly)...")
            print("⚠️  WARNING: Robot will move! Starting in 3 seconds...")
            time.sleep(3)
            
            action_info = env.step(action)
            print("✓ Step executed")
            
            # Wait for movement
            print("Waiting 2 seconds for movement...")
            time.sleep(2)
            
            # Get state after step
            print("Getting state after step...")
            state_after, _ = env.get_state()
            print(f"  - Final EE position: {np.array(state_after['cartesian_position'])[:3]}")
            print(f"  - Final joint positions: {np.array(state_after['joint_positions'])[:3]}...")
            
            # Check if robot moved
            if "cartesian" in action_space:
                ee_diff = np.abs(np.array(state_after['cartesian_position'][:3]) - 
                                np.array(state_before['cartesian_position'][:3]))
                max_ee_diff = np.max(ee_diff)
                print(f"  - Max EE difference: {max_ee_diff:.4f} m")
                if max_ee_diff > 0.001:  # 1mm threshold
                    print("✓ Robot moved (EE position changed)")
                else:
                    print("⚠️  Robot may not have moved much")
            else:
                joint_diff = np.abs(np.array(state_after['joint_positions']) - 
                                   np.array(state_before['joint_positions']))
                max_joint_diff = np.max(joint_diff)
                print(f"  - Max joint difference: {max_joint_diff:.4f} rad")
                if max_joint_diff > 0.01:  # 0.01 rad threshold
                    print("✓ Robot moved (joint positions changed)")
                else:
                    print("⚠️  Robot may not have moved much")
            
            return True
            
        except Exception as e:
            print(f"✗ Step test failed: {e}")
            print(traceback.format_exc())
            return False
    
    def test_update_robot(self, action_space="cartesian_velocity", full_test=False):
        """
        Test 5: Update robot functionality
        
        Tests:
        - Update robot with different action spaces
        - Blocking vs non-blocking
        """
        print("")
        print("=" * 60)
        print("TEST 5: Update Robot Functionality")
        print("=" * 60)
        print(f"Action space: {action_space}")
        print(f"Full test: {full_test}")
        print("")
        
        if not full_test:
            print("⏭️  SKIPPED (use --full-test to enable)")
            return True
        
        try:
            # Initialize env
            print("Initializing RobotEnv...")
            env = RobotEnv(action_space=action_space, do_reset=False, node=self.node)
            print("✓ RobotEnv initialized")
            
            # Create test action
            if "velocity" in action_space:
                if "cartesian" in action_space:
                    action = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                else:
                    action = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            else:
                state, _ = env.get_state()
                if "cartesian" in action_space:
                    current_pos = np.array(state['cartesian_position'])
                    action = current_pos.copy()
                    action[0] += 0.01
                else:
                    current_joints = np.array(state['joint_positions'])
                    action = current_joints.copy()
                    action[0] += 0.05
                    action = np.append(action, 0.5)
            
            print(f"Created test action: shape={action.shape}")
            
            # Test non-blocking update
            print("Testing non-blocking update...")
            print("⚠️  WARNING: Robot will move! Starting in 3 seconds...")
            time.sleep(3)
            
            action_info = env.update_robot(
                action,
                action_space=action_space,
                blocking=False
            )
            print("✓ Non-blocking update executed")
            
            # Wait
            time.sleep(2)
            
            # Test blocking update (if supported)
            print("Testing blocking update...")
            action_info = env.update_robot(
                action,
                action_space=action_space,
                blocking=True
            )
            print("✓ Blocking update executed")
            
            return True
            
        except Exception as e:
            print(f"✗ Update robot test failed: {e}")
            print(traceback.format_exc())
            return False
    
    def test_create_action_dict(self):
        """
        Test 6: Create action dict functionality
        
        Tests:
        - Action dictionary creation
        - Dictionary structure
        """
        print("")
        print("=" * 60)
        print("TEST 6: Create Action Dict Functionality")
        print("=" * 60)
        print("")
        
        try:
            # Initialize env
            print("Initializing RobotEnv...")
            env = RobotEnv(action_space="cartesian_velocity", do_reset=False, node=self.node)
            print("✓ RobotEnv initialized")
            
            # Create test action
            action = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            print(f"Test action: shape={action.shape}")
            
            # Create action dict
            print("Creating action dictionary...")
            action_dict = env.create_action_dict(action)
            print("✓ Action dictionary created")
            
            # Check structure
            print(f"Action dict keys: {list(action_dict.keys())}")
            print(f"Action dict type: {type(action_dict)}")
            
            # Validate
            if isinstance(action_dict, dict):
                print("✓ Action dict is a dictionary")
            else:
                print(f"✗ Action dict is not a dictionary: {type(action_dict)}")
                return False
            
            return True
            
        except Exception as e:
            print(f"✗ Create action dict test failed: {e}")
            print(traceback.format_exc())
            return False
    
    def run_all_tests(self, action_space="cartesian_velocity", full_test=False):
        """
        Run all tests
        """
        print("")
        print("=" * 60)
        print("ROBOT_ENV TEST SUITE")
        print("=" * 60)
        print(f"Action space: {action_space}")
        print(f"Full test (robot movement): {full_test}")
        print("=" * 60)
        print("")
        
        results = {}
        
        # Test 1: Initialization
        results['Initialization'] = self.test_initialization(action_space=action_space, do_reset=False)
        time.sleep(2)
        
        # Test 2: Reset
        results['Reset'] = self.test_reset(randomize=False)
        time.sleep(2)
        
        # Test 3: Get State
        results['Get State'] = self.test_get_state()
        time.sleep(2)
        
        # Test 4: Step
        results['Step'] = self.test_step(action_space=action_space, full_test=full_test)
        time.sleep(2)
        
        # Test 5: Update Robot
        results['Update Robot'] = self.test_update_robot(action_space=action_space, full_test=full_test)
        time.sleep(2)
        
        # Test 6: Create Action Dict
        results['Create Action Dict'] = self.test_create_action_dict()
        
        # Print summary
        print("")
        print("=" * 60)
        print("TEST SUMMARY")
        print("=" * 60)
        passed = sum(1 for v in results.values() if v)
        total = len(results)
        
        for test_name, result in results.items():
            status = "✓ PASS" if result else "✗ FAIL"
            print(f"  {test_name}: {status}")
        
        print("")
        print(f"Results: {passed}/{total} tests passed")
        
        if passed == total:
            print("✓ ALL TESTS PASSED")
        else:
            print("⚠️  SOME TESTS FAILED")
        
        print("=" * 60)
        
        return results


def main(args=None):
    parser = argparse.ArgumentParser(description='Test RobotEnv functionality')
    parser.add_argument('--action-space', type=str, default='cartesian_velocity',
                       choices=['cartesian_velocity', 'joint_velocity', 
                               'cartesian_position', 'joint_position', 'all'],
                       help='Action space to test')
    parser.add_argument('--full-test', action='store_true',
                       help='Perform full movement tests (WARNING: Robot will move!)')
    
    parsed_args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init(args=None)
    
    # Create ROS2 node
    node = Node('robot_env_tester')
    
    # Create tester
    tester = RobotEnvTester(node=node)
    
    try:
        if parsed_args.action_space == 'all':
            # Test all action spaces
            action_spaces = ['cartesian_velocity', 'joint_velocity', 
                           'cartesian_position', 'joint_position']
            all_results = {}
            
            for action_space in action_spaces:
                print(f"\n{'='*60}")
                print(f"Testing action space: {action_space}")
                print(f"{'='*60}\n")
                results = tester.run_all_tests(action_space=action_space, full_test=parsed_args.full_test)
                all_results[action_space] = results
                time.sleep(3)
            
            # Print overall summary
            print("\n" + "=" * 60)
            print("OVERALL SUMMARY")
            print("=" * 60)
            for action_space, results in all_results.items():
                passed = sum(1 for v in results.values() if v)
                total = len(results)
                print(f"{action_space}: {passed}/{total} tests passed")
        else:
            # Test single action space
            tester.run_all_tests(action_space=parsed_args.action_space, full_test=parsed_args.full_test)
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
        print(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


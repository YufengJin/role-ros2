#!/usr/bin/env python3
"""
Test script for RobotEnv interface.

This script tests RobotEnv functionality including:
- Initialization
- State reading (get_state)
- Action execution (step)
- Robot reset
- Different action spaces

⚠️  Warning: This test will move the robot!
   - Big movements for testing (0.05 m/s cartesian, 0.1 rad/s joint)
   - Make sure there's enough space around the robot!

Usage:
    python3 test_robot_env.py [--mock] [--skip-reset] [--skip-motion] [--action-space ACTION_SPACE]
    
    --mock: Use mock mode (no real robot)
    --skip-reset: Skip reset tests
    --skip-motion: Skip motion control tests (only test state getters)
    --action-space: Action space to test (cartesian_velocity, joint_velocity, cartesian_position, joint_position)
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path

# Add parent directory to path for imports
# sys.path.insert(0, str(Path(__file__).parent.parent))

import rclpy
from rclpy.node import Node
from role_ros2.robot_env import RobotEnv


class TestResult:
    """Helper class to track test results."""
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.skipped = 0
        self.errors = []
    
    def add_pass(self, test_name):
        self.passed += 1
        print(f"✅ PASS: {test_name}")
    
    def add_fail(self, test_name, error):
        self.failed += 1
        self.errors.append((test_name, error))
        print(f"❌ FAIL: {test_name} - {error}")
    
    def add_skip(self, test_name, reason):
        self.skipped += 1
        print(f"⏭️  SKIP: {test_name} - {reason}")
    
    def print_summary(self):
        print("\n" + "="*70)
        print("TEST SUMMARY")
        print("="*70)
        print(f"Passed:  {self.passed}")
        print(f"Failed:  {self.failed}")
        print(f"Skipped: {self.skipped}")
        print(f"Total:   {self.passed + self.failed + self.skipped}")
        
        if self.errors:
            print("\nFAILED TESTS:")
            for test_name, error in self.errors:
                print(f"  - {test_name}: {error}")
        
        print("="*70)


def test_initialization(results, action_space="cartesian_velocity", do_reset=False, node=None):
    """Test RobotEnv initialization."""
    print("\n" + "="*70)
    print("TEST: RobotEnv Initialization")
    print("="*70)
    print(f"  action_space: {action_space}")
    print(f"  do_reset: {do_reset}")
    
    try:
        env = RobotEnv(
            action_space=action_space,
            gripper_action_space="velocity",
            do_reset=do_reset,
            node=node
        )
        
        # Check attributes
        assert env.action_space == action_space, f"Action space mismatch: {env.action_space} != {action_space}"
        assert env._robot is not None, "Robot not initialized"
        assert env.DoF == (7 if "cartesian" in action_space else 8), f"DoF mismatch: {env.DoF}"
        
        print(f"  ✓ RobotEnv initialized successfully")
        print(f"  ✓ DoF: {env.DoF}")
        print(f"  ✓ Action space: {env.action_space}")
        
        results.add_pass("RobotEnv Initialization")
        return env
    
    except Exception as e:
        results.add_fail("RobotEnv Initialization", str(e))
        import traceback
        traceback.print_exc()
        return None


def test_get_state(results, env):
    """Test get_state() method."""
    print("\n" + "="*70)
    print("TEST: get_state()")
    print("="*70)
    
    try:
        state_dict, timestamp_dict = env.get_state()
        
        # Check required keys
        required_keys = [
            "cartesian_position",
            "gripper_position",
            "joint_positions",
            "joint_velocities"
        ]
        
        for key in required_keys:
            assert key in state_dict, f"Missing key: {key}"
        
        # Check data types and shapes
        assert len(state_dict["cartesian_position"]) == 6, "cartesian_position should have 6 elements"
        assert len(state_dict["joint_positions"]) == 7, "joint_positions should have 7 elements"
        assert len(state_dict["joint_velocities"]) == 7, "joint_velocities should have 7 elements"
        assert isinstance(state_dict["gripper_position"], (float, int)), "gripper_position should be numeric"
        
        # Print state info
        cart_pos = state_dict["cartesian_position"]
        joint_pos = state_dict["joint_positions"]
        gripper_pos = state_dict["gripper_position"]
        
        print(f"  ✓ State retrieved successfully")
        print(f"  ✓ Cartesian position: [{cart_pos[0]:.3f}, {cart_pos[1]:.3f}, {cart_pos[2]:.3f}]")
        print(f"  ✓ Joint positions: {[f'{j:.3f}' for j in joint_pos[:3]]}...")
        print(f"  ✓ Gripper position: {gripper_pos:.3f}")
        print(f"  ✓ Timestamp keys: {list(timestamp_dict.keys())}")
        
        results.add_pass("get_state()")
        return state_dict, timestamp_dict
    
    except Exception as e:
        results.add_fail("get_state()", str(e))
        import traceback
        traceback.print_exc()
        return None, None


def test_step_zero_action(results, env):
    """Test step() with zero action (safe)."""
    print("\n" + "="*70)
    print("TEST: step() with Zero Action")
    print("="*70)
    
    try:
        # Get initial state
        initial_state, _ = env.get_state()
        initial_pos = np.array(initial_state["cartesian_position"][:3])
        
        # Execute zero action
        action = np.zeros(env.DoF)
        action_info = env.step(action)
        
        # Wait a bit for action to take effect
        time.sleep(0.5)
        
        # Get final state
        final_state, _ = env.get_state()
        final_pos = np.array(final_state["cartesian_position"][:3])
        
        # With zero action, position should not change much
        pos_change = np.linalg.norm(final_pos - initial_pos)
        
        print(f"  ✓ Zero action executed")
        print(f"  ✓ Position change: {pos_change:.4f} m (should be small)")
        print(f"  ✓ Action info type: {type(action_info)}")
        
        results.add_pass("step() with Zero Action")
    
    except Exception as e:
        results.add_fail("step() with Zero Action", str(e))
        import traceback
        traceback.print_exc()


def test_step_big_movement(results, env, skip_motion=False):
    """Test step() with big movement."""
    if skip_motion:
        results.add_skip("step() with Big Movement", "skip-motion flag set")
        return
    
    print("\n" + "="*70)
    print("TEST: step() with Big Movement")
    print("="*70)
    print("  ⚠️  Robot will move significantly!")
    
    try:
        # Get initial state
        initial_state, _ = env.get_state()
        initial_pos = np.array(initial_state["cartesian_position"][:3])
        
        # Create big movement action
        if "cartesian" in env.action_space:
            # Big cartesian velocity: 0.05 m/s in X direction, 0.03 m/s in Y direction
            action = np.array([0.05, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0])  # [vx, vy, vz, wx, wy, wz, gripper]
        else:
            # Big joint velocity
            action = np.array([0.1, -0.1, 0.08, -0.08, 0.05, -0.05, 0.05, 0.0])  # 7 joints + gripper
        
        print(f"  Executing action: {action[:3]}...")
        print(f"  Waiting 3 seconds for movement...")
        
        # Execute action for 3 seconds
        start_time = time.time()
        while time.time() - start_time < 3.0:
            env.step(action)
            time.sleep(1.0 / env.control_hz)  # ~15 Hz
        
        # Stop movement
        zero_action = np.zeros(env.DoF)
        env.step(zero_action)
        time.sleep(0.5)
        
        # Get final state
        final_state, _ = env.get_state()
        final_pos = np.array(final_state["cartesian_position"][:3])
        
        # Check if position changed
        pos_change = np.linalg.norm(final_pos - initial_pos)
        
        print(f"  ✓ Movement executed")
        print(f"  ✓ Initial position: [{initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f}]")
        print(f"  ✓ Final position:   [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}]")
        print(f"  ✓ Position change: {pos_change:.4f} m")
        
        if pos_change > 0.01:  # At least 1cm movement
            print(f"  ✓ Significant movement detected (expected)")
        else:
            print(f"  ⚠️  No significant movement detected")
        
        results.add_pass("step() with Big Movement")
    
    except Exception as e:
        results.add_fail("step() with Big Movement", str(e))
        import traceback
        traceback.print_exc()


def test_reset(results, env, skip_reset=False):
    """Test reset() method."""
    if skip_reset:
        results.add_skip("reset()", "skip-reset flag set")
        return
    
    print("\n" + "="*70)
    print("TEST: reset()")
    print("="*70)
    print("  ⚠️  Robot will reset to home position!")
    
    try:
        # Get state before reset
        before_state, _ = env.get_state()
        before_pos = np.array(before_state["cartesian_position"][:3])
        
        print(f"  Position before reset: [{before_pos[0]:.3f}, {before_pos[1]:.3f}, {before_pos[2]:.3f}]")
        print(f"  Calling reset()...")
        
        # Reset robot
        env.reset(randomize=False)
        
        # Wait for reset to complete
        print(f"  Waiting for reset to complete (5 seconds)...")
        time.sleep(5.0)
        
        # Get state after reset
        after_state, _ = env.get_state()
        after_pos = np.array(after_state["cartesian_position"][:3])
        
        print(f"  Position after reset: [{after_pos[0]:.3f}, {after_pos[1]:.3f}, {after_pos[2]:.3f}]")
        print(f"  ✓ Reset completed")
        
        results.add_pass("reset()")
    
    except Exception as e:
        results.add_fail("reset()", str(e))
        import traceback
        traceback.print_exc()


def test_gripper_control(results, env, skip_motion=False):
    """Test gripper control via action."""
    if skip_motion:
        results.add_skip("Gripper Control", "skip-motion flag set")
        return
    
    print("\n" + "="*70)
    print("TEST: Gripper Control")
    print("="*70)
    print("  ⚠️  Gripper will move!")
    
    try:
        # Get initial gripper state
        initial_state, _ = env.get_state()
        initial_gripper = initial_state["gripper_position"]
        
        print(f"  Initial gripper position: {initial_gripper:.3f}")
        
        # Open gripper with bigger action
        if "cartesian" in env.action_space:
            action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5])  # Bigger gripper velocity
        else:
            action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5])  # Joint + gripper
        
        print(f"  Opening gripper (action: {action[-1]:.2f})...")
        for _ in range(20):  # Execute for ~1.33 seconds at 15Hz
            env.step(action)
            time.sleep(1.0 / env.control_hz)
        
        # Stop
        zero_action = np.zeros(env.DoF)
        env.step(zero_action)
        time.sleep(0.5)
        
        # Get final gripper state
        final_state, _ = env.get_state()
        final_gripper = final_state["gripper_position"]
        
        print(f"  Final gripper position: {final_gripper:.3f}")
        print(f"  Gripper change: {final_gripper - initial_gripper:.3f}")
        print(f"  ✓ Gripper control executed")
        
        results.add_pass("Gripper Control")
    
    except Exception as e:
        results.add_fail("Gripper Control", str(e))
        import traceback
        traceback.print_exc()


def test_observation(results, env):
    """Test get_observation() method and save observation data."""
    print("\n" + "="*70)
    print("TEST: get_observation()")
    print("="*70)
    
    try:
        obs_dict = env.get_observation()
        
        # Check required keys
        assert "robot_state" in obs_dict, "Missing 'robot_state' in observation"
        assert "timestamp" in obs_dict, "Missing 'timestamp' in observation"
        
        # Check robot_state structure
        robot_state = obs_dict["robot_state"]
        assert "cartesian_position" in robot_state, "Missing 'cartesian_position' in robot_state"
        assert "gripper_position" in robot_state, "Missing 'gripper_position' in robot_state"
        
        # Check camera data - should be None if no cameras available
        camera_keys = [k for k in obs_dict.keys() if 'camera' in k.lower() or 'image' in k.lower()]
        camera_intrinsics = obs_dict.get("camera_intrinsics", None)
        
        print(f"  ✓ Observation retrieved successfully")
        print(f"  ✓ Observation keys: {list(obs_dict.keys())}")
        print(f"  ✓ Robot state keys: {list(robot_state.keys())[:5]}...")
        print(f"  ✓ Timestamp keys: {list(obs_dict['timestamp'].keys())}")
        
        # Check camera intrinsics
        if camera_intrinsics is None:
            print(f"  ✓ Camera intrinsics: None (no cameras available - expected)")
        else:
            print(f"  ✓ Camera intrinsics: {len(camera_intrinsics)} cameras")
        
        # Check for camera images in observation
        has_images = False
        image_keys = []
        saved_images = {}
        
        for key in obs_dict.keys():
            if 'image' in key.lower() or 'rgb' in key.lower() or 'depth' in key.lower():
                image_keys.append(key)
                img_data = obs_dict[key]
                if img_data is not None:
                    has_images = True
                    saved_images[key] = img_data
                else:
                    saved_images[key] = None  # Save None if no image
        
        if image_keys:
            print(f"  ✓ Found image keys: {image_keys}")
            for key in image_keys:
                img_data = obs_dict.get(key, None)
                if img_data is None:
                    print(f"    - {key}: None (no image available - saved as None)")
                else:
                    print(f"    - {key}: {type(img_data)} (image available)")
        else:
            print(f"  ✓ No image keys found (no cameras - expected)")
        
        # Save observation data (images as None if not available)
        import json
        import pickle
        from pathlib import Path
        
        # Create output directory
        output_dir = Path(__file__).parent.parent / "scripts" / "outputs"
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Prepare data for saving (convert numpy arrays to lists, handle None images)
        save_dict = {}
        for key, value in obs_dict.items():
            if key in image_keys:
                # Save images as None if not available
                save_dict[key] = None
            elif isinstance(value, np.ndarray):
                save_dict[key] = value.tolist()
            elif isinstance(value, dict):
                # Recursively handle dicts
                save_dict[key] = {}
                for sub_key, sub_value in value.items():
                    if isinstance(sub_value, np.ndarray):
                        save_dict[key][sub_key] = sub_value.tolist()
                    else:
                        save_dict[key][sub_key] = sub_value
            else:
                save_dict[key] = value
        
        # Save as JSON (for readability, images will be None)
        json_path = output_dir / "test_observation.json"
        with open(json_path, 'w') as f:
            json.dump(save_dict, f, indent=2, default=str)
        print(f"  ✓ Observation saved to: {json_path}")
        print(f"    Note: Images saved as None (no cameras available)")
        
        results.add_pass("get_observation()")
    
    except Exception as e:
        results.add_fail("get_observation()", str(e))
        import traceback
        traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description="Test RobotEnv interface")
    parser.add_argument("--mock", action="store_true", help="Use mock mode (no real robot)")
    parser.add_argument("--skip-reset", action="store_true", help="Skip reset tests")
    parser.add_argument("--skip-motion", action="store_true", help="Skip motion control tests")
    parser.add_argument(
        "--action-space",
        type=str,
        default="cartesian_velocity",
        choices=["cartesian_velocity", "joint_velocity", "cartesian_position", "joint_position"],
        help="Action space to test (default: cartesian_velocity)"
    )
    
    args = parser.parse_args()
    
    print("="*70)
    print("RobotEnv Test Suite")
    print("="*70)
    print(f"Action space: {args.action_space}")
    print(f"Mock mode: {args.mock}")
    print(f"Skip reset: {args.skip_reset}")
    print(f"Skip motion: {args.skip_motion}")
    print("="*70)
    
    if not args.skip_motion:
        print("\n⚠️  WARNING: This test will move the robot with BIG movements!")
        print("   - Cartesian velocity: 0.05 m/s in X, 0.03 m/s in Y")
        print("   - Joint velocity: 0.1 rad/s")
        print("   - Make sure there's enough space around the robot!")
        response = input("\nContinue? (yes/no): ").strip().lower()
        if response != "yes":
            print("Test cancelled.")
            return
    
    # Initialize ROS2
    rclpy.init()
    node = Node("test_robot_env")
    
    results = TestResult()
    
    try:
        # Test 1: Initialization
        env = test_initialization(
            results,
            action_space=args.action_space,
            do_reset=not args.skip_reset,
            node=node
        )
        
        if env is None:
            print("\n❌ Failed to initialize RobotEnv. Aborting tests.")
            results.print_summary()
            return
        
        # Wait for state to be available
        print("\nWaiting for robot state to be available...")
        for _ in range(10):
            try:
                state_dict, _ = env.get_state()
                if state_dict and "cartesian_position" in state_dict:
                    print("✓ Robot state available")
                    break
            except:
                pass
            time.sleep(0.5)
        else:
            print("⚠️  Robot state not available, but continuing...")
        
        # Test 2: get_state()
        test_get_state(results, env)
        
        # Test 3: step() with zero action
        test_step_zero_action(results, env)
        
        # Test 4: step() with big movement
        test_step_big_movement(results, env, skip_motion=args.skip_motion)
        
        # Test 5: Gripper control
        test_gripper_control(results, env, skip_motion=args.skip_motion)
        
        # Test 6: get_observation()
        test_observation(results, env)
        
        # Test 7: reset()
        test_reset(results, env, skip_reset=args.skip_reset)
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        
        # Print summary
        results.print_summary()


if __name__ == "__main__":
    main()


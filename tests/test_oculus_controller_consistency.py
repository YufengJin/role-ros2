#!/usr/bin/env python3
"""
Test script to verify consistency between droid and role-ros2 VRPolicy implementations.

Two modes:
- test: Use fake input/output to check for differences
- live: Connect to real Quest device and compare outputs
"""

import sys
import time
import argparse
import numpy as np
from pathlib import Path
from typing import Dict, Any, Tuple
from unittest.mock import Mock, MagicMock, patch


# Try to find droid - check common locations
droid_path = None
possible_droid_paths = [
    Path("/app/droid"),                # Container path: /app/droid
]

for path in possible_droid_paths:
    if path.exists():
        # Check if droid package exists (either path/droid or path itself is droid)
        if (path / "droid").exists():
            droid_path = path
            break
        elif (path / "controllers" / "oculus_controller.py").exists():
            # Path is already droid/droid
            droid_path = path.parent if path.name == "droid" else path
            break

if droid_path is None:
    print("⚠️  Warning: Could not find droid repository. Some tests may fail.")
    print("   Please ensure droid is in one of these locations:")
    for path in possible_droid_paths:
        print(f"     - {path}")
    print("   Or ensure /app/droid/controllers/oculus_controller.py exists (container mode)")
else:
    print(f"✅ Found droid at: {droid_path}")

if droid_path:
    # Add droid path to sys.path
    # Try both droid_path and droid_path/droid
    if str(droid_path) not in sys.path:
        sys.path.insert(0, str(droid_path))
    
    droid_package_path = droid_path / "droid"
    if droid_package_path.exists() and str(droid_package_path) not in sys.path:
        sys.path.insert(0, str(droid_package_path))

class MockOculusReader:
    """Mock OculusReader for testing mode."""
    
    def __init__(self, test_data: Dict[str, Any] = None, run: bool = True):
        """
        Initialize mock reader.
        
        Args:
            test_data: Dictionary with test poses and buttons
            run: If True, simulate running state (default: True)
        """
        if test_data is None:
            # Default test data - create a realistic pose matrix
            # Identity matrix with small translation
            pose_r = np.eye(4)
            pose_r[:3, 3] = [0.1, 0.2, 0.3]  # Position
            pose_l = np.eye(4)
            pose_l[:3, 3] = [-0.1, -0.2, -0.3]
            
            self.test_data = {
                "poses": {
                    "r": pose_r,
                    "l": pose_l,
                },
                "buttons": {
                    "RG": False,
                    "LG": False,
                    "RJ": False,
                    "LJ": False,
                    "A": False,
                    "B": False,
                    "X": False,
                    "Y": False,
                    "rightTrig": [0.0],
                    "leftTrig": [0.0],
                }
            }
        else:
            self.test_data = test_data
        self._call_count = 0
        self.running = run
    
    def get_transformations_and_buttons(self):
        """Return mock poses and buttons."""
        self._call_count += 1
        # Return copies to avoid mutation issues
        poses_copy = {}
        for key, value in self.test_data["poses"].items():
            if isinstance(value, np.ndarray):
                poses_copy[key] = value.copy()
            else:
                poses_copy[key] = value
        buttons_copy = self.test_data["buttons"].copy()
        return poses_copy, buttons_copy
    
    def stop(self):
        """Stop the mock reader."""
        self.running = False
    
    def run(self):
        """Start the mock reader."""
        self.running = True


def create_test_robot_state() -> Dict[str, Any]:
    """Create a test robot state dictionary."""
    return {
        "cartesian_position": [0.5, 0.0, 0.3, 0.0, 0.0, 0.0],  # [x, y, z, roll, pitch, yaw]
        "gripper_position": 0.5,
    }


def compare_arrays(arr1: np.ndarray, arr2: np.ndarray, name: str, rtol: float = 1e-5, atol: float = 1e-8) -> bool:
    """
    Compare two numpy arrays and print differences.
    
    Args:
        arr1: First array
        arr2: Second array
        name: Name for logging
        rtol: Relative tolerance
        atol: Absolute tolerance
    
    Returns:
        True if arrays are equal within tolerance
    """
    if arr1.shape != arr2.shape:
        print(f"❌ {name}: Shape mismatch - {arr1.shape} vs {arr2.shape}")
        return False
    
    if not np.allclose(arr1, arr2, rtol=rtol, atol=atol):
        diff = np.abs(arr1 - arr2)
        max_diff = np.max(diff)
        print(f"❌ {name}: Values differ - max diff: {max_diff:.6f}")
        print(f"   droid:    {arr1}")
        print(f"   role_ros2: {arr2}")
        return False
    
    print(f"✅ {name}: Arrays match")
    return True


def compare_dicts(dict1: Dict, dict2: Dict, name: str, rtol: float = 1e-5, atol: float = 1e-8) -> bool:
    """
    Compare two dictionaries and print differences.
    
    Args:
        dict1: First dictionary
        dict2: Second dictionary
        name: Name for logging
        rtol: Relative tolerance for numeric values
        atol: Absolute tolerance for numeric values
    
    Returns:
        True if dictionaries are equal
    """
    if set(dict1.keys()) != set(dict2.keys()):
        print(f"❌ {name}: Key mismatch - {set(dict1.keys())} vs {set(dict2.keys())}")
        return False
    
    all_match = True
    for key in dict1.keys():
        val1 = dict1[key]
        val2 = dict2[key]
        
        if isinstance(val1, np.ndarray) and isinstance(val2, np.ndarray):
            if not compare_arrays(val1, val2, f"{name}['{key}']", rtol, atol):
                all_match = False
        elif isinstance(val1, (int, float)) and isinstance(val2, (int, float)):
            if not np.isclose(val1, val2, rtol=rtol, atol=atol):
                print(f"❌ {name}['{key}']: Values differ - {val1} vs {val2}")
                all_match = False
            else:
                print(f"✅ {name}['{key}']: Values match ({val1})")
        elif val1 != val2:
            print(f"❌ {name}['{key}']: Values differ - {val1} vs {val2}")
            all_match = False
        else:
            print(f"✅ {name}['{key}']: Values match ({val1})")
    
    return all_match


def test_reset_state(droid_policy, role_policy):
    """Test reset_state method."""
    print("\n" + "="*80)
    print("Testing reset_state()")
    print("="*80)
    
    # Reset both
    droid_policy.reset_state()
    role_policy.reset_state()
    
    # Compare state dictionaries
    droid_state = droid_policy._state
    role_state = role_policy._state
    
    state_match = compare_dicts(droid_state, role_state, "reset_state._state")
    
    # Compare other attributes
    attrs_to_check = [
        "update_sensor",
        "reset_origin",
        "robot_origin",
        "vr_origin",
        "vr_state",
    ]
    
    attrs_match = True
    for attr in attrs_to_check:
        val1 = getattr(droid_policy, attr, None)
        val2 = getattr(role_policy, attr, None)
        if val1 != val2:
            print(f"❌ reset_state.{attr}: Values differ - {val1} vs {val2}")
            attrs_match = False
        else:
            print(f"✅ reset_state.{attr}: Values match ({val1})")
    
    return state_match and attrs_match


def test_get_info(droid_policy, role_policy):
    """Test get_info method."""
    print("\n" + "="*80)
    print("Testing get_info()")
    print("="*80)
    
    # Set up test state
    droid_policy._state["buttons"] = {
        "A": True,
        "B": False,
        "X": False,
        "Y": True,
    }
    droid_policy._state["movement_enabled"] = True
    droid_policy._state["controller_on"] = True
    
    role_policy._state["buttons"] = {
        "A": True,
        "B": False,
        "X": False,
        "Y": True,
    }
    role_policy._state["movement_enabled"] = True
    role_policy._state["controller_on"] = True
    
    # Test right controller
    droid_policy.controller_id = "r"
    role_policy.controller_id = "r"
    
    droid_info = droid_policy.get_info()
    role_info = role_policy.get_info()
    
    right_match = compare_dicts(droid_info, role_info, "get_info (right controller)")
    
    # Test left controller
    droid_policy.controller_id = "l"
    role_policy.controller_id = "l"
    
    droid_info = droid_policy.get_info()
    role_info = role_policy.get_info()
    
    left_match = compare_dicts(droid_info, role_info, "get_info (left controller)")
    
    return right_match and left_match


def test_forward(droid_policy, role_policy, robot_state: Dict[str, Any]):
    """Test forward method."""
    print("\n" + "="*80)
    print("Testing forward()")
    print("="*80)
    
    # Set up identical VR state for both policies
    vr_pos = np.array([0.1, 0.2, 0.3])
    vr_quat = np.array([0.0, 0.0, 0.0, 1.0])
    vr_gripper = 0.7
    
    robot_pos = np.array(robot_state["cartesian_position"][:3])
    robot_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Will be computed from euler
    
    droid_policy.vr_state = {
        "pos": vr_pos.copy(),
        "quat": vr_quat.copy(),
        "gripper": vr_gripper,
    }
    droid_policy.robot_origin = {
        "pos": robot_pos.copy(),
        "quat": robot_quat.copy(),
    }
    droid_policy.vr_origin = {
        "pos": vr_pos.copy(),
        "quat": vr_quat.copy(),
    }
    droid_policy.update_sensor = False
    droid_policy.reset_origin = False
    droid_policy._state["poses"] = {"r": np.eye(4)}  # Non-empty to avoid early return
    
    role_policy.vr_state = {
        "pos": vr_pos.copy(),
        "quat": vr_quat.copy(),
        "gripper": vr_gripper,
    }
    role_policy.robot_origin = {
        "pos": robot_pos.copy(),
        "quat": robot_quat.copy(),
    }
    role_policy.vr_origin = {
        "pos": vr_pos.copy(),
        "quat": vr_quat.copy(),
    }
    role_policy.update_sensor = False
    role_policy.reset_origin = False
    role_policy._state["poses"] = {"r": np.eye(4)}  # Non-empty to avoid early return
    
    obs_dict = {"robot_state": robot_state}
    
    # Test without info
    print("\n--- Testing forward(obs_dict, include_info=False) ---")
    droid_action = droid_policy.forward(obs_dict, include_info=False)
    role_action = role_policy.forward(obs_dict, include_info=False)
    
    action_match = compare_arrays(
        np.array(droid_action),
        np.array(role_action),
        "forward() action (no info)",
        rtol=1e-4,
        atol=1e-6
    )
    
    # Reset state for next test
    droid_policy.update_sensor = False
    droid_policy.reset_origin = False
    role_policy.update_sensor = False
    role_policy.reset_origin = False
    
    # Test with info
    print("\n--- Testing forward(obs_dict, include_info=True) ---")
    droid_result = droid_policy.forward(obs_dict, include_info=True)
    role_result = role_policy.forward(obs_dict, include_info=True)
    
    droid_action_info, droid_info = droid_result
    role_action_info, role_info = role_result
    
    action_info_match = compare_arrays(
        np.array(droid_action_info),
        np.array(role_action_info),
        "forward() action (with info)",
        rtol=1e-4,
        atol=1e-6
    )
    
    info_match = compare_dicts(droid_info, role_info, "forward() info dict", rtol=1e-4, atol=1e-6)
    
    # Test empty poses case
    print("\n--- Testing forward() with empty poses ---")
    droid_policy._state["poses"] = {}
    role_policy._state["poses"] = {}
    
    droid_action_empty = droid_policy.forward(obs_dict, include_info=False)
    role_action_empty = role_policy.forward(obs_dict, include_info=False)
    
    empty_match = compare_arrays(
        np.array(droid_action_empty),
        np.array(role_action_empty),
        "forward() action (empty poses)"
    )
    
    return action_match and action_info_match and info_match and empty_match


def run_test_mode():
    """Run tests with mock data."""
    print("\n" + "="*80)
    print("TEST MODE: Using Mock Data")
    print("="*80)
    
    # Create shared mock OculusReader (both implementations will use same data)
    mock_reader = MockOculusReader(run=False)  # Don't auto-run to avoid threading issues
    
    # Import both implementations
    try:
        from droid.controllers.oculus_controller import VRPolicy as DroidVRPolicy
        print("✅ Imported droid.controllers.oculus_controller.VRPolicy")
    except ImportError as e:
        print(f"❌ Failed to import droid VRPolicy: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    try:
        from role_ros2.controllers.oculus_controller import VRPolicy as RoleVRPolicy
        print("✅ Imported role_ros2.controllers.oculus_controller.VRPolicy")
    except ImportError as e:
        print(f"❌ Failed to import role_ros2 VRPolicy: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # Patch OculusReader for both implementations
    # Use a factory function to return the same mock instance
    def mock_reader_factory(*args, **kwargs):
        return mock_reader
    
    with patch('droid.controllers.oculus_controller.OculusReader', side_effect=mock_reader_factory), \
         patch('role_ros2.controllers.oculus_controller.OculusReader', side_effect=mock_reader_factory):
        
        # Create instances with identical parameters
        print("\nCreating VRPolicy instances with identical parameters...")
        init_params = {
            "right_controller": True,
            "max_lin_vel": 1.0,
            "max_rot_vel": 1.0,
            "max_gripper_vel": 1.0,
            "spatial_coeff": 1.0,
            "pos_action_gain": 5.0,
            "rot_action_gain": 2.0,
            "gripper_action_gain": 3.0,
            "rmat_reorder": [-2, -1, -3, 4],
        }
        
        try:
            droid_policy = DroidVRPolicy(**init_params)
            role_policy = RoleVRPolicy(**init_params)
            print("✅ Both VRPolicy instances created successfully")
        except Exception as e:
            print(f"❌ Failed to create VRPolicy instances: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        # Wait for background threads to initialize and read some data
        print("\nWaiting for background threads to initialize (1 second)...")
        time.sleep(1.0)
        
        # Manually inject test data into both policies' internal state
        # This ensures both have the same input data
        test_poses = {"r": mock_reader.test_data["poses"]["r"].copy()}
        test_buttons = mock_reader.test_data["buttons"].copy()
        
        droid_policy._state["poses"] = test_poses.copy()
        droid_policy._state["buttons"] = test_buttons.copy()
        role_policy._state["poses"] = test_poses.copy()
        role_policy._state["buttons"] = test_buttons.copy()
        
        # Create test robot state
        robot_state = create_test_robot_state()
        
        # Run tests
        print("\n" + "="*80)
        print("Running Consistency Tests")
        print("="*80)
        
        results = {}
        results["reset_state"] = test_reset_state(droid_policy, role_policy)
        results["get_info"] = test_get_info(droid_policy, role_policy)
        results["forward"] = test_forward(droid_policy, role_policy, robot_state)
        
        # Summary
        print("\n" + "="*80)
        print("TEST SUMMARY")
        print("="*80)
        all_passed = True
        for test_name, passed in results.items():
            status = "✅ PASS" if passed else "❌ FAIL"
            print(f"{status}: {test_name}")
            if not passed:
                all_passed = False
        
        if all_passed:
            print("\n🎉 All tests passed! Implementations are consistent.")
        else:
            print("\n⚠️  Some tests failed. Please review the differences above.")
        
        return all_passed


def run_live_mode():
    """Run tests with real Quest device."""
    print("\n" + "="*80)
    print("LIVE MODE: Connecting to Real Quest Device")
    print("="*80)
    
    # Import both implementations
    try:
        from droid.controllers.oculus_controller import VRPolicy as DroidVRPolicy
        print("✅ Imported droid.controllers.oculus_controller.VRPolicy")
    except ImportError as e:
        print(f"❌ Failed to import droid VRPolicy: {e}")
        return False
    
    try:
        from role_ros2.controllers.oculus_controller import VRPolicy as RoleVRPolicy
        print("✅ Imported role_ros2.controllers.oculus_controller.VRPolicy")
    except ImportError as e:
        print(f"❌ Failed to import role_ros2 VRPolicy: {e}")
        return False
    
    # Create instances with real OculusReader
    print("\nCreating VRPolicy instances with real Quest connection...")
    print("⚠️  Make sure Quest is connected and controllers are on!")
    print("⚠️  This will create TWO OculusReader instances (one for each implementation)")
    
    init_params = {
        "right_controller": True,
        "max_lin_vel": 1.0,
        "max_rot_vel": 1.0,
        "max_gripper_vel": 1.0,
        "spatial_coeff": 1.0,
        "pos_action_gain": 5.0,
        "rot_action_gain": 2.0,
        "gripper_action_gain": 3.0,
        "rmat_reorder": [-2, -1, -3, 4],
    }
    
    try:
        droid_policy = DroidVRPolicy(**init_params)
        print("✅ Droid VRPolicy created")
        
        role_policy = RoleVRPolicy(**init_params)
        print("✅ Role-ROS2 VRPolicy created")
    except Exception as e:
        print(f"❌ Failed to create VRPolicy instances: {e}")
        print("   Make sure Quest is connected and Oculus software is running")
        import traceback
        traceback.print_exc()
        return False
    
    # Wait for controllers to connect and read initial data
    print("\nWaiting for controllers to connect and read initial data (5 seconds)...")
    time.sleep(5)
    
    # Check if controllers are connected
    if not droid_policy._state.get("poses", {}) or not role_policy._state.get("poses", {}):
        print("⚠️  Warning: Controllers may not be connected. Poses are empty.")
        print("   Continuing anyway...")
    
    # Create test robot state
    robot_state = create_test_robot_state()
    obs_dict = {"robot_state": robot_state}
    
    # Run comparison for multiple iterations
    print("\nRunning live comparison (10 iterations)...")
    print("="*80)
    
    all_match = True
    match_count = 0
    total_checks = 0
    
    for i in range(100):
        print(f"\n--- Iteration {i+1}/10 ---")
        
        # Wait a bit for state updates
        time.sleep(0.2)
        
        # Test get_info
        total_checks += 1
        droid_info = droid_policy.get_info()
        role_info = role_policy.get_info()
        
        info_match = compare_dicts(droid_info, role_info, f"get_info (iter {i+1})")
        if info_match:
            match_count += 1
        else:
            all_match = False
        
        # Test forward (if poses available)
        droid_poses = droid_policy._state.get("poses", {})
        role_poses = role_policy._state.get("poses", {})
        
        if droid_poses and role_poses:
            total_checks += 1
            try:
                droid_action = droid_policy.forward(obs_dict, include_info=False)
                role_action = role_policy.forward(obs_dict, include_info=False)
                
                action_match = compare_arrays(
                    np.array(droid_action),
                    np.array(role_action),
                    f"forward() action (iter {i+1})",
                    rtol=1e-4,
                    atol=1e-6
                )
                if action_match:
                    match_count += 1
                else:
                    all_match = False
            except Exception as e:
                print(f"⚠️  Error in forward() at iteration {i+1}: {e}")
                all_match = False
        else:
            print(f"⚠️  Skipping forward() test - poses not available (droid: {bool(droid_poses)}, role: {bool(role_poses)})")
    
    print("\n" + "="*80)
    print("LIVE TEST SUMMARY")
    print("="*80)
    print(f"Total checks: {total_checks}")
    print(f"Matches: {match_count}")
    print(f"Match rate: {match_count/total_checks*100:.1f}%" if total_checks > 0 else "N/A")
    
    if all_match and match_count == total_checks:
        print("✅ All comparisons passed!")
    elif match_count > 0:
        print(f"⚠️  {match_count}/{total_checks} comparisons passed")
    else:
        print("❌ All comparisons failed - check output above")
    
    return all_match


def compare_code_logic():
    """
    Compare the code logic between droid and role_ros2 implementations.
    
    This function reads both files and compares key method implementations.
    """
    print("\n" + "="*80)
    print("CODE LOGIC COMPARISON")
    print("="*80)
    
    # Read both files
    script_dir = Path(__file__).parent.absolute()
    role_ros2_path = script_dir.parent
    
    # Find droid path
    droid_path = None
    possible_droid_paths = [
        Path("/app/droid"),                # Container path: /app/droid
        role_ros2_path.parent / "droid",   # ../droid
        Path("/home/yjin/repos/droid"),    # Absolute path
        Path.home() / "repos" / "droid",   # ~/repos/droid
    ]
    
    for path in possible_droid_paths:
        if path.exists():
            # Check if droid package exists (either path/droid or path itself is droid)
            if (path / "droid").exists():
                droid_path = path
                break
            elif (path / "controllers" / "oculus_controller.py").exists():
                # Path is already droid/droid
                droid_path = path.parent if path.name == "droid" else path
                break
    
    if droid_path is None:
        print("⚠️  Could not find droid repository. Skipping code comparison.")
        return True
    
    # Try different possible paths for droid file
    possible_droid_files = [
        droid_path / "droid" / "controllers" / "oculus_controller.py",  # Standard: /app/droid/droid/controllers/...
        droid_path / "controllers" / "oculus_controller.py",            # Direct: /app/droid/controllers/...
    ]
    
    droid_file = None
    for df in possible_droid_files:
        if df.exists():
            droid_file = df
            break
    
    role_file = role_ros2_path / "role_ros2" / "controllers" / "oculus_controller.py"
    
    if droid_file is None:
        print(f"⚠️  Droid file not found. Checked:")
        for df in possible_droid_files:
            print(f"     - {df}")
        return True
    
    if not role_file.exists():
        print(f"⚠️  Role-ROS2 file not found: {role_file}")
        return True
    
    print(f"Comparing:")
    print(f"  Droid:    {droid_file}")
    print(f"  Role-ROS2: {role_file}")
    
    # Read files
    with open(droid_file, 'r') as f:
        droid_code = f.read()
    
    with open(role_file, 'r') as f:
        role_code = f.read()
    
    # Extract key methods for comparison
    import re
    
    methods_to_check = [
        'reset_state',
        'get_info',
        'forward',
        '_calculate_action',
        '_limit_velocity',
        '_process_reading',
    ]
    
    all_match = True
    for method_name in methods_to_check:
        # Extract method from droid
        droid_pattern = rf'def {method_name}\(.*?\):(.*?)(?=\n    def |\nclass |\Z)'
        droid_match = re.search(droid_pattern, droid_code, re.DOTALL)
        
        # Extract method from role_ros2
        role_match = re.search(droid_pattern, role_code, re.DOTALL)
        
        if droid_match and role_match:
            droid_method = droid_match.group(0).strip()
            role_method = role_match.group(0).strip()
            
            # Normalize whitespace and comments for comparison
            droid_normalized = re.sub(r'#.*', '', droid_method)  # Remove comments
            droid_normalized = re.sub(r'\s+', ' ', droid_normalized)  # Normalize whitespace
            role_normalized = re.sub(r'#.*', '', role_method)
            role_normalized = re.sub(r'\s+', ' ', role_normalized)
            
            if droid_normalized == role_normalized:
                print(f"✅ {method_name}(): Logic matches")
            else:
                print(f"❌ {method_name}(): Logic differs")
                # Show first difference
                droid_lines = droid_method.split('\n')
                role_lines = role_method.split('\n')
                min_len = min(len(droid_lines), len(role_lines))
                for i in range(min_len):
                    if droid_lines[i].strip() != role_lines[i].strip():
                        print(f"   First difference at line {i+1}:")
                        print(f"     Droid:    {droid_lines[i][:80]}")
                        print(f"     Role-ROS2: {role_lines[i][:80]}")
                        break
                all_match = False
        elif droid_match or role_match:
            print(f"⚠️  {method_name}(): Found in one but not both")
            all_match = False
        else:
            print(f"⚠️  {method_name}(): Not found in either file")
    
    return all_match


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test consistency between droid and role-ros2 VRPolicy implementations'
    )
    parser.add_argument(
        '--mode',
        type=str,
        choices=['test', 'live', 'code'],
        default='test',
        help='Test mode: "test" for mock data, "live" for real Quest device, "code" for code comparison'
    )
    
    args = parser.parse_args()
    
    if args.mode == 'code':
        success = compare_code_logic()
    elif args.mode == 'test':
        success = run_test_mode()
    else:
        success = run_live_mode()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

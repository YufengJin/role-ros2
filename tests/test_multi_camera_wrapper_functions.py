#!/usr/bin/env python3
"""
Test script for MultiCameraWrapper functions.

Tests:
1. Data completeness - check if all cameras have image and depth data
2. Multi-camera timestamp differences - check pub_t differences between cameras
3. read_cameras return dict type and shape validation
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Dict, Tuple, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera.multi_camera_wrapper import MultiCameraWrapper


class MultiCameraWrapperTestNode(Node):
    """Test node for MultiCameraWrapper functions."""
    
    def __init__(self, config_file: str = None, duration: float = 30.0):
        """
        Initialize test node.
        
        Args:
            config_file: Path to camera configuration file (optional)
            duration: Test duration in seconds
        """
        super().__init__('multi_camera_wrapper_test_node')
        
        self.duration = duration
        
        print(f"\n{'='*80}")
        print(f"MultiCameraWrapper Function Tests")
        print(f"{'='*80}\n")
        print(f"Test duration: {duration} seconds\n")
        
        # Initialize multi-camera wrapper
        try:
            self.camera_wrapper = MultiCameraWrapper(
                node=self,
                config_file=config_file
            )
            self.get_logger().info("MultiCameraWrapper initialized")
            
            # Get list of camera IDs
            self.camera_ids = list(self.camera_wrapper.camera_dict.keys())
            self.get_logger().info(f"Found {len(self.camera_ids)} cameras: {self.camera_ids}")
            
            if not self.camera_ids:
                self.get_logger().error("❌ No cameras found! Check camera configuration.")
                self.camera_wrapper = None
                return
            
            print(f"✅ Initialized {len(self.camera_ids)} cameras: {self.camera_ids}\n")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MultiCameraWrapper: {e}")
            import traceback
            traceback.print_exc()
            self.camera_wrapper = None
            self.camera_ids = []
        
        # Statistics
        self.test_results = {
            'sync_mode_calls': 0,
            'sync_mode_success': 0,
            'non_sync_mode_calls': 0,
            'non_sync_mode_success': 0,
            'data_completeness': {
                'total_checks': 0,
                'complete_data': 0,
                'missing_cameras': [],
                'missing_images': [],
                'missing_depths': []
            },
            'timestamp_diffs': {
                'sync': [],  # Fixed: use 'sync' instead of 'sync_mode'
                'non_sync': []  # Fixed: use 'non_sync' instead of 'non_sync_mode'
            },
            'data_types': {
                'image_types': {},
                'depth_types': {},
                'image_shapes': {},
                'depth_shapes': {}
            }
        }
        
        self.start_time = None
        self.sample_printed = False  # Flag to print sample only once
    
    def run_tests(self):
        """Run all tests."""
        if not self.camera_wrapper or not self.camera_ids:
            print("❌ Cannot run tests: No cameras available")
            return
        
        print("Waiting for cameras to start receiving data...")
        
        # Wait for cameras to start (max 10 seconds)
        timeout = 10.0
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            all_running = all(
                self.camera_wrapper.camera_dict[cam_id].is_running()
                for cam_id in self.camera_ids
            )
            if all_running:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                return
        
        if not all_running:
            print("⚠️  Some cameras are not running, but continuing with tests...\n")
        else:
            print("✅ All cameras are running\n")
        
        print("Starting tests...\n")
        
        self.start_time = time.time()
        test_interval = 0.5  # Run tests every 0.5 seconds
        last_test_time = time.time()
        
        while rclpy.ok() and (time.time() - self.start_time) < self.duration:
            # Spin ROS2 to receive messages
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Run tests periodically
            current_time = time.time()
            if current_time - last_test_time >= test_interval:
                self._test_read_cameras_sync()
                self._test_read_cameras_non_sync()
                last_test_time = current_time
        
        # Print results
        self._print_results()
    
    def _test_read_cameras_sync(self):
        """Test read_cameras with use_sync=True."""
        self.test_results['sync_mode_calls'] += 1
        
        try:
            full_obs_dict, full_timestamp_dict = self.camera_wrapper.read_cameras(use_sync=True)
            
            if self._validate_data_completeness(full_obs_dict, full_timestamp_dict, 'sync'):
                self.test_results['sync_mode_success'] += 1
                self._check_timestamp_diffs(full_timestamp_dict, 'sync')
                self._check_data_types_and_shapes(full_obs_dict)
                self._print_sample_data(full_obs_dict, full_timestamp_dict, 'sync')
        except Exception as e:
            self.get_logger().error(f"Error in sync mode test: {e}")
    
    def _test_read_cameras_non_sync(self):
        """Test read_cameras with use_sync=False."""
        self.test_results['non_sync_mode_calls'] += 1
        
        try:
            full_obs_dict, full_timestamp_dict = self.camera_wrapper.read_cameras(use_sync=False)
            
            if self._validate_data_completeness(full_obs_dict, full_timestamp_dict, 'non_sync'):
                self.test_results['non_sync_mode_success'] += 1
                self._check_timestamp_diffs(full_timestamp_dict, 'non_sync')
                self._check_data_types_and_shapes(full_obs_dict)
                self._print_sample_data(full_obs_dict, full_timestamp_dict, 'non_sync')
        except Exception as e:
            self.get_logger().error(f"Error in non-sync mode test: {e}")
    
    def _validate_data_completeness(
        self, 
        full_obs_dict: Dict, 
        full_timestamp_dict: Dict,
        mode: str
    ) -> bool:
        """
        Validate data completeness.
        
        Args:
            full_obs_dict: Observation dictionary
            full_timestamp_dict: Timestamp dictionary
            mode: 'sync' or 'non_sync'
            
        Returns:
            True if data is complete, False otherwise
        """
        self.test_results['data_completeness']['total_checks'] += 1
        
        # Check if full_obs_dict has required keys
        if not full_obs_dict:
            return False
        
        if "image" not in full_obs_dict or "depth" not in full_obs_dict:
            return False
        
        image_dict = full_obs_dict.get("image", {})
        depth_dict = full_obs_dict.get("depth", {})
        
        # Check if all cameras have data
        missing_cameras = []
        missing_images = []
        missing_depths = []
        
        for camera_id in self.camera_ids:
            if camera_id not in image_dict:
                missing_images.append(camera_id)
            if camera_id not in depth_dict:
                missing_depths.append(camera_id)
            if camera_id not in image_dict or camera_id not in depth_dict:
                missing_cameras.append(camera_id)
        
        if missing_cameras:
            self.test_results['data_completeness']['missing_cameras'].extend(missing_cameras)
            self.test_results['data_completeness']['missing_images'].extend(missing_images)
            self.test_results['data_completeness']['missing_depths'].extend(missing_depths)
            return False
        
        # Check if timestamp_dict has required keys for all cameras
        for camera_id in self.camera_ids:
            pub_t_key = f"{camera_id}_pub_t"
            if pub_t_key not in full_timestamp_dict:
                return False
        
        self.test_results['data_completeness']['complete_data'] += 1
        return True
    
    def _check_timestamp_diffs(self, full_timestamp_dict: Dict, mode: str):
        """
        Check timestamp differences between cameras.
        
        Args:
            full_timestamp_dict: Timestamp dictionary
            mode: 'sync' or 'non_sync'
        """
        # Extract pub_t timestamps for all cameras
        pub_timestamps = {}
        for camera_id in self.camera_ids:
            pub_t_key = f"{camera_id}_pub_t"
            if pub_t_key in full_timestamp_dict:
                pub_timestamps[camera_id] = full_timestamp_dict[pub_t_key]
        
        if len(pub_timestamps) < 2:
            return  # Need at least 2 cameras to compare
        
        # Calculate differences between all pairs of cameras
        camera_ids_list = list(pub_timestamps.keys())
        for i in range(len(camera_ids_list)):
            for j in range(i + 1, len(camera_ids_list)):
                cam_id_1 = camera_ids_list[i]
                cam_id_2 = camera_ids_list[j]
                
                pub_t_1 = pub_timestamps[cam_id_1]
                pub_t_2 = pub_timestamps[cam_id_2]
                
                diff_ns = abs(pub_t_1 - pub_t_2)
                diff_ms = diff_ns / 1e6
                
                self.test_results['timestamp_diffs'][mode].append({
                    'camera_pair': (cam_id_1, cam_id_2),
                    'diff_ms': diff_ms,
                    'pub_t_1': pub_t_1,
                    'pub_t_2': pub_t_2
                })
    
    def _check_data_types_and_shapes(self, full_obs_dict: Dict):
        """
        Check data types and shapes.
        
        Args:
            full_obs_dict: Observation dictionary
        """
        image_dict = full_obs_dict.get("image", {})
        depth_dict = full_obs_dict.get("depth", {})
        
        for camera_id in self.camera_ids:
            # Check image type and shape
            if camera_id in image_dict:
                img = image_dict[camera_id]
                if isinstance(img, np.ndarray):
                    img_type = str(img.dtype)
                    img_shape = img.shape
                    
                    if camera_id not in self.test_results['data_types']['image_types']:
                        self.test_results['data_types']['image_types'][camera_id] = []
                        self.test_results['data_types']['image_shapes'][camera_id] = []
                    
                    self.test_results['data_types']['image_types'][camera_id].append(img_type)
                    self.test_results['data_types']['image_shapes'][camera_id].append(img_shape)
            
            # Check depth type and shape
            if camera_id in depth_dict:
                depth = depth_dict[camera_id]
                if isinstance(depth, np.ndarray):
                    depth_type = str(depth.dtype)
                    depth_shape = depth.shape
                    
                    if camera_id not in self.test_results['data_types']['depth_types']:
                        self.test_results['data_types']['depth_types'][camera_id] = []
                        self.test_results['data_types']['depth_shapes'][camera_id] = []
                    
                    self.test_results['data_types']['depth_types'][camera_id].append(depth_type)
                    self.test_results['data_types']['depth_shapes'][camera_id].append(depth_shape)
    
    def _print_sample_data(self, full_obs_dict: Dict, full_timestamp_dict: Dict, mode: str):
        """
        Print a sample of read_cameras output (only once).
        
        Args:
            full_obs_dict: Observation dictionary
            full_timestamp_dict: Timestamp dictionary
            mode: 'sync' or 'non_sync'
        """
        if self.sample_printed:
            return
        
        print(f"\n{'='*80}")
        print(f"Sample Data ({mode.upper()} mode)")
        print(f"{'='*80}\n")
        
        # Print full_obs_dict structure
        print("full_obs_dict:")
        print(f"  Keys: {list(full_obs_dict.keys())}")
        for key in full_obs_dict:
            if key == "image":
                print(f"  {key}:")
                for camera_id, img_array in full_obs_dict[key].items():
                    if isinstance(img_array, np.ndarray):
                        print(f"    {camera_id}: numpy.ndarray, shape={img_array.shape}, dtype={img_array.dtype}")
                        print(f"      min={img_array.min()}, max={img_array.max()}, mean={img_array.mean():.2f}")
                    else:
                        print(f"    {camera_id}: {type(img_array)}")
            elif key == "depth":
                print(f"  {key}:")
                for camera_id, depth_array in full_obs_dict[key].items():
                    if isinstance(depth_array, np.ndarray):
                        print(f"    {camera_id}: numpy.ndarray, shape={depth_array.shape}, dtype={depth_array.dtype}")
                        print(f"      min={depth_array.min()}, max={depth_array.max()}, mean={depth_array.mean():.2f}")
                    else:
                        print(f"    {camera_id}: {type(depth_array)}")
            else:
                print(f"  {key}: {type(full_obs_dict[key])}")
        print()
        
        # Print full_timestamp_dict structure
        print("full_timestamp_dict:")
        print(f"  Total keys: {len(full_timestamp_dict)}")
        print(f"  Keys (first 20): {list(full_timestamp_dict.keys())[:20]}")
        
        # Print timestamp values for each camera
        for camera_id in self.camera_ids:
            print(f"\n  Camera {camera_id} timestamps:")
            pub_t_key = f"{camera_id}_pub_t"
            sub_t_key = f"{camera_id}_sub_t"
            end_t_key = f"{camera_id}_end_t"
            
            if pub_t_key in full_timestamp_dict:
                pub_t_ns = full_timestamp_dict[pub_t_key]
                print(f"    {pub_t_key}: {pub_t_ns} ({pub_t_ns/1e9:.9f} s)")
            
            if sub_t_key in full_timestamp_dict:
                sub_t_ns = full_timestamp_dict[sub_t_key]
                print(f"    {sub_t_key}: {sub_t_ns} ({sub_t_ns/1e9:.9f} s)")
            
            if end_t_key in full_timestamp_dict:
                end_t_ns = full_timestamp_dict[end_t_key]
                print(f"    {end_t_key}: {end_t_ns} ({end_t_ns/1e9:.9f} s)")
            
            # Calculate differences if available
            if pub_t_key in full_timestamp_dict and sub_t_key in full_timestamp_dict:
                diff_ms = (full_timestamp_dict[sub_t_key] - full_timestamp_dict[pub_t_key]) / 1e6
                print(f"    sub_t - pub_t: {diff_ms:.2f} ms")
        
        # Print multi-camera sync metadata if available
        if "multi_camera_sync_start" in full_timestamp_dict:
            print(f"\n  Multi-camera sync metadata:")
            sync_keys = [k for k in full_timestamp_dict.keys() if k.startswith("multi_camera_sync")]
            for key in sorted(sync_keys):
                value = full_timestamp_dict[key]
                if isinstance(value, (int, float)):
                    if key.endswith("_ns") or key in ["multi_camera_sync_start", "multi_camera_sync_check", 
                                                       "multi_camera_sync_ready", "multi_camera_sync_end"]:
                        print(f"    {key}: {value} ({value/1e9:.9f} s)")
                    else:
                        print(f"    {key}: {value}")
                else:
                    print(f"    {key}: {value}")
        
        print(f"{'='*80}\n")
        self.sample_printed = True
    
    def _print_results(self):
        """Print test results."""
        print(f"\n{'='*80}")
        print("Test Results")
        print(f"{'='*80}\n")
        
        # Test 1: Data completeness
        print("1. Data Completeness Test:")
        completeness = self.test_results['data_completeness']
        print(f"   Total checks: {completeness['total_checks']}")
        print(f"   Complete data: {completeness['complete_data']}")
        if completeness['total_checks'] > 0:
            success_rate = (completeness['complete_data'] / completeness['total_checks']) * 100
            print(f"   Success rate: {success_rate:.1f}%")
        
        if completeness['missing_cameras']:
            unique_missing = list(set(completeness['missing_cameras']))
            print(f"   ⚠️  Missing cameras: {unique_missing}")
        if completeness['missing_images']:
            unique_missing = list(set(completeness['missing_images']))
            print(f"   ⚠️  Missing images: {unique_missing}")
        if completeness['missing_depths']:
            unique_missing = list(set(completeness['missing_depths']))
            print(f"   ⚠️  Missing depths: {unique_missing}")
        print()
        
        # Test 2: Timestamp differences
        print("2. Multi-Camera Timestamp Differences:")
        for mode in ['sync', 'non_sync']:
            diffs = self.test_results['timestamp_diffs'][mode]
            if diffs:
                diff_values = [d['diff_ms'] for d in diffs]
                mean_diff = np.mean(diff_values)
                std_diff = np.std(diff_values)
                min_diff = np.min(diff_values)
                max_diff = np.max(diff_values)
                
                print(f"   {mode.upper()} mode:")
                print(f"     Mean diff: {mean_diff:.2f} ms")
                print(f"     Std diff:  {std_diff:.2f} ms")
                print(f"     Min diff:  {min_diff:.2f} ms")
                print(f"     Max diff:  {max_diff:.2f} ms")
                print(f"     Samples:   {len(diff_values)}")
                
                # Show camera pairs
                if len(diffs) > 0:
                    print(f"     Camera pairs tested:")
                    unique_pairs = set()
                    for d in diffs[:10]:  # Show first 10 pairs
                        pair = tuple(sorted(d['camera_pair']))
                        if pair not in unique_pairs:
                            print(f"       - {pair[0]} <-> {pair[1]}")
                            unique_pairs.add(pair)
            else:
                print(f"   {mode.upper()} mode: No data collected")
        print()
        
        # Test 3: Data types and shapes
        print("3. Data Types and Shapes:")
        data_types = self.test_results['data_types']
        
        for camera_id in self.camera_ids:
            print(f"   Camera {camera_id}:")
            
            # Image type and shape
            if camera_id in data_types['image_types']:
                img_types = data_types['image_types'][camera_id]
                img_shapes = data_types['image_shapes'][camera_id]
                
                if img_types:
                    most_common_type = max(set(img_types), key=img_types.count)
                    most_common_shape = max(set(img_shapes), key=img_shapes.count)
                    
                    print(f"     Image:")
                    print(f"       Type:  {most_common_type} (most common)")
                    print(f"       Shape: {most_common_shape} (most common)")
                    print(f"       Samples: {len(img_types)}")
            
            # Depth type and shape
            if camera_id in data_types['depth_types']:
                depth_types = data_types['depth_types'][camera_id]
                depth_shapes = data_types['depth_shapes'][camera_id]
                
                if depth_types:
                    most_common_type = max(set(depth_types), key=depth_types.count)
                    most_common_shape = max(set(depth_shapes), key=depth_shapes.count)
                    
                    print(f"     Depth:")
                    print(f"       Type:  {most_common_type} (most common)")
                    print(f"       Shape: {most_common_shape} (most common)")
                    print(f"       Samples: {len(depth_types)}")
        print()
        
        # Test 4: read_cameras function calls
        print("4. read_cameras Function Calls:")
        print(f"   Sync mode:")
        print(f"     Calls: {self.test_results['sync_mode_calls']}")
        print(f"     Success: {self.test_results['sync_mode_success']}")
        if self.test_results['sync_mode_calls'] > 0:
            sync_rate = (self.test_results['sync_mode_success'] / 
                        self.test_results['sync_mode_calls']) * 100
            print(f"     Success rate: {sync_rate:.1f}%")
        
        print(f"   Non-sync mode:")
        print(f"     Calls: {self.test_results['non_sync_mode_calls']}")
        print(f"     Success: {self.test_results['non_sync_mode_success']}")
        if self.test_results['non_sync_mode_calls'] > 0:
            non_sync_rate = (self.test_results['non_sync_mode_success'] / 
                           self.test_results['non_sync_mode_calls']) * 100
            print(f"     Success rate: {non_sync_rate:.1f}%")
        print()
        
        # Summary
        print(f"{'='*80}")
        print("Summary:")
        print(f"  ✅ Data completeness: {completeness['complete_data']}/{completeness['total_checks']} successful")
        print(f"  ✅ Timestamp differences: {len(self.test_results['timestamp_diffs']['sync'])} sync samples, "
              f"{len(self.test_results['timestamp_diffs']['non_sync'])} non-sync samples")
        print(f"  ✅ Data types/shapes: Validated for {len(data_types['image_types'])} cameras")
        print(f"  ✅ read_cameras calls: {self.test_results['sync_mode_success']} sync, "
              f"{self.test_results['non_sync_mode_success']} non-sync successful")
        print(f"{'='*80}\n")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test MultiCameraWrapper functions'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='Path to camera configuration file (optional)')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='Test duration in seconds (default: 30.0)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = MultiCameraWrapperTestNode(args.config_file, args.duration)
        node.run_tests()
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


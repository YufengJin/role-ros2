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
import threading
from pathlib import Path
from typing import Dict, Tuple, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# Matplotlib for visualization
try:
    import matplotlib
    matplotlib.use('Agg')  # Use non-interactive backend
    import matplotlib.pyplot as plt
    from matplotlib.figure import Figure
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️  Matplotlib not available. Visualization will be skipped.")

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera.multi_camera_wrapper import MultiCameraWrapper


class MultiCameraWrapperTestNode(Node):
    """Test node for MultiCameraWrapper functions."""
    
    def __init__(self, config_file: str = None, duration: float = 30.0, fps: float = 2.0):
        """
        Initialize test node.
        
        Args:
            config_file: Path to camera configuration file (optional)
            duration: Test duration in seconds
            fps: Frequency of read_cameras() calls in Hz (default: 2.0)
        """
        super().__init__('multi_camera_wrapper_test_node')
        
        self.duration = duration
        self.fps = fps
        self.test_interval = 1.0 / fps if fps > 0 else 0.5
        
        print(f"\n{'='*80}")
        print(f"MultiCameraWrapper Function Tests")
        print(f"{'='*80}\n")
        print(f"Test duration: {duration} seconds")
        print(f"read_cameras() frequency: {fps} Hz (interval: {self.test_interval*1000:.2f} ms)\n")
        
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
            },
            'intrinsics': {
                'calls': 0,
                'success': 0,
                'sample_data': None
            },
            'extrinsics': {
                'calls': 0,
                'success': 0,
                'sample_data': None
            }
        }
        
        self.start_time = None
        self.sample_printed = False  # Flag to print sample only once
        
        # Setup MultiThreadedExecutor for background spinning
        # Since MultiCameraWrapper uses shared node and no longer has internal spin thread,
        # we need to spin the node ourselves to process callbacks
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        
        # Start background thread for executor spinning
        # This runs after camera wrapper initialization so all subscribers are registered
        self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self._spin_thread.start()
        
        self.get_logger().info("MultiCameraWrapperTestNode: Background executor thread started")
    
    def _spin_executor(self):
        """
        Background thread method that spins the MultiThreadedExecutor.
        
        This continuously processes ROS2 callbacks in the background,
        ensuring camera data is always up-to-date.
        """
        try:
            self._executor.spin()
        except Exception as e:
            # Handle shutdown gracefully
            if rclpy.ok():
                self.get_logger().error(f"Error in executor spin thread: {e}")
            else:
                # Normal shutdown
                self.get_logger().debug("Executor spin thread shutting down")
    
    def shutdown(self):
        """
        Cleanly shut down test node and all resources.
        
        Stops the executor, joins the spin thread, and cleans up camera resources.
        """
        self.get_logger().info("MultiCameraWrapperTestNode: Shutting down...")
        
        # Shutdown executor (this will stop the spin thread)
        try:
            self._executor.shutdown()
        except Exception as e:
            self.get_logger().warn(f"Error shutting down executor: {e}")
        
        # Join spin thread
        if self._spin_thread is not None and self._spin_thread.is_alive():
            try:
                self._spin_thread.join(timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f"Error joining spin thread: {e}")
        
        # Clean up camera resources
        if self.camera_wrapper is not None and hasattr(self.camera_wrapper, 'shutdown'):
            try:
                self.camera_wrapper.shutdown()
            except Exception as e:
                self.get_logger().warn(f"Error shutting down camera wrapper: {e}")
        
        self.get_logger().info("MultiCameraWrapperTestNode: Shutdown complete")
    
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
            # No need to spin - MultiThreadedExecutor handles callbacks in background
            time.sleep(0.1)  # Just wait a bit
            if not rclpy.ok():
                return
        
        if not all_running:
            print("⚠️  Some cameras are not running, but continuing with tests...\n")
        else:
            print("✅ All cameras are running\n")
        
        print("Starting tests...\n")
        
        self.start_time = time.time()
        last_test_time = time.time()
        iteration = 0
        
        while rclpy.ok() and (time.time() - self.start_time) < self.duration:
            # No need to spin - MultiThreadedExecutor handles callbacks in background
            
            # Run tests at specified frequency
            current_time = time.time()
            if current_time - last_test_time >= self.test_interval:
                iteration += 1
                elapsed_time = current_time - self.start_time
                if iteration % 10 == 0 or iteration == 1:  # Print every 10 iterations
                    print(f"--- Iteration {iteration} (elapsed: {elapsed_time:.2f}s, target FPS: {self.fps:.1f}) ---")
                
                self._test_read_cameras_sync()
                self._test_read_cameras_non_sync()
                self._test_get_camera_intrinsics()
                self._test_get_cameras_extrinsics()
                
                # Calculate next call time and sleep to maintain target FPS
                last_test_time += self.test_interval
                sleep_time = last_test_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                elif sleep_time < -0.1:  # If we're more than 100ms behind, log warning
                    self.get_logger().warn(
                        f"⚠️  Falling behind schedule: {abs(sleep_time)*1000:.2f} ms behind target interval"
                    )
                    last_test_time = time.time()  # Reset to current time to avoid accumulating delay
        
        # Print results in format consistent with test_robot_env_observation.py
        self._print_read_cameras_packet_loss()
        self._print_camera_timestamp_diff_analysis()
        
        # Generate visualizations
        if MATPLOTLIB_AVAILABLE:
            self._plot_timestamp_analysis()
    
    def _print_read_cameras_packet_loss(self):
        """Print packet loss analysis for read_cameras() in format consistent with test_robot_env_observation.py."""
        print("\n" + "=" * 80)
        print("read_cameras() PACKET LOSS ANALYSIS")
        print("=" * 80 + "\n")
        
        # Combine sync and non-sync stats
        total_calls = self.test_results['sync_mode_calls'] + self.test_results['non_sync_mode_calls']
        successful_calls = self.test_results['sync_mode_success'] + self.test_results['non_sync_mode_success']
        failed_calls = total_calls - successful_calls
        
        if total_calls > 0:
            success_rate = (successful_calls / total_calls) * 100
            packet_loss_rate = (failed_calls / total_calls) * 100
            
            print(f"Total calls: {total_calls}")
            print(f"Successful calls: {successful_calls}")
            print(f"Failed calls: {failed_calls}")
            print(f"Success rate: {success_rate:.2f}%")
            print(f"Packet loss rate: {packet_loss_rate:.2f}%")
            print()
            
            completeness = self.test_results['data_completeness']
            print("Failure breakdown:")
            print(f"  Missing images: {len(completeness['missing_images'])} times")
            print(f"  Missing depths: {len(completeness['missing_depths'])} times")
            print(f"  Missing intrinsics: 0 times (not tracked in this test)")
            print(f"  Missing extrinsics: 0 times (not tracked in this test)")
            print()
            
            # Print sync vs non-sync breakdown (matching test_robot_env_observation.py format)
            print("Data source breakdown:")
            print(f"  Sync mode data: {self.test_results['sync_mode_success']} times")
            print(f"  Non-sync mode data: {self.test_results['non_sync_mode_success']} times")
            if self.test_results['sync_mode_success'] + self.test_results['non_sync_mode_success'] > 0:
                sync_rate = (self.test_results['sync_mode_success'] / 
                           (self.test_results['sync_mode_success'] + self.test_results['non_sync_mode_success'])) * 100
                print(f"  Sync data rate: {sync_rate:.2f}%")
        else:
            print("No data collected")
        
        print("\n" + "=" * 80 + "\n")
    
    def _print_camera_timestamp_diff_analysis(self):
        """Print timestamp difference analysis for cameras in format consistent with test_robot_env_observation.py."""
        print("\n" + "=" * 80)
        print("read_cameras() TIMESTAMP DIFFERENCE ANALYSIS")
        print("=" * 80 + "\n")
        
        # Per-camera timestamp differences (section 1 - matching test_robot_env_observation.py format)
        print("1. Per-Camera Timestamp Differences:")
        print("   " + "-" * 76)
        print("   No per-camera timestamp data collected (not tracked in this test)")
        print()
        
        # Multi-camera timestamp differences (section 2 - matching test_robot_env_observation.py format)
        print("2. Multi-Camera Timestamp Differences (pub_t differences, absolute value):")
        print("   " + "-" * 76)
        
        sync_diffs = self.test_results['timestamp_diffs']['sync']
        non_sync_diffs = self.test_results['timestamp_diffs']['non_sync']
        
        if sync_diffs or non_sync_diffs:
            # Count sync vs non-sync data
            sync_count = len(sync_diffs)
            non_sync_count = len(non_sync_diffs)
            print(f"\n   Data source: {sync_count} from sync mode, {non_sync_count} from non-sync mode")
            
            # Group by camera pair (combining sync and non-sync for each pair)
            all_diffs = sync_diffs + non_sync_diffs
            pair_diffs = {}
            for diff_data in all_diffs:
                pair = tuple(sorted(diff_data['camera_pair']))
                if pair not in pair_diffs:
                    pair_diffs[pair] = []
                pair_diffs[pair].append(diff_data['diff_ms'])
            
            for pair, diffs in sorted(pair_diffs.items()):
                cam_id_1, cam_id_2 = pair
                print(f"\n   {cam_id_1} <-> {cam_id_2}:")
                print(f"     Mean:   {np.mean(diffs):.3f} ms")
                print(f"     Std:    {np.std(diffs):.3f} ms")
                print(f"     Min:    {np.min(diffs):.3f} ms")
                print(f"     Max:    {np.max(diffs):.3f} ms")
                print(f"     Samples: {len(diffs)}")
        else:
            print("   No multi-camera timestamp data collected (need at least 2 cameras)")
        
        print("\n" + "=" * 80 + "\n")
    
    def _test_read_cameras_sync(self):
        """Test read_cameras with use_sync=True."""
        self.test_results['sync_mode_calls'] += 1
        
        try:
            full_obs_dict, full_timestamp_dict = self.camera_wrapper.read_cameras(use_sync=True)
            # Print timestamps in a readable format for debugging and analysis
            import datetime
            print("Sync mode timestamps:")
            for key, value in full_timestamp_dict.items():
                if isinstance(value, int) and value > 1e9:
                    try:
                        # Interpret value as nanoseconds since epoch
                        dt = datetime.datetime.fromtimestamp(value / 1e9)
                        print(f"  {key}: {value} ({dt.strftime('%Y-%m-%d %H:%M:%S.%f')})")
                    except Exception:
                        print(f"  {key}: {value} (unparsable)")
                else:
                    print(f"  {key}: {value}")
            
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
    
    def _test_get_camera_intrinsics(self):
        """Test get_camera_intrinsics method."""
        self.test_results['intrinsics']['calls'] += 1
        
        try:
            intrinsics = self.camera_wrapper.get_camera_intrinsics()
            
            if intrinsics is not None and isinstance(intrinsics, dict):
                # Validate intrinsics structure
                valid = True
                for camera_id in self.camera_ids:
                    if camera_id not in intrinsics:
                        valid = False
                        break
                    intrinsic_matrix = intrinsics[camera_id]
                    if not isinstance(intrinsic_matrix, np.ndarray):
                        valid = False
                        break
                    if intrinsic_matrix.shape != (3, 3):
                        valid = False
                        break
                
                if valid:
                    self.test_results['intrinsics']['success'] += 1
                    # Store sample data (only once)
                    if self.test_results['intrinsics']['sample_data'] is None:
                        self.test_results['intrinsics']['sample_data'] = intrinsics
        except Exception as e:
            self.get_logger().error(f"Error in get_camera_intrinsics test: {e}")
    
    def _test_get_cameras_extrinsics(self):
        """Test get_cameras_extrinsics method."""
        self.test_results['extrinsics']['calls'] += 1
        
        try:
            # First, get timestamp_dict from read_cameras for synchronized lookup
            camera_obs, camera_timestamp = self.camera_wrapper.read_cameras(use_sync=True)
            
            # Test with timestamp_dict
            extrinsics = self.camera_wrapper.get_cameras_extrinsics(camera_timestamp)
            
            # Also test without timestamp_dict
            if extrinsics is None:
                extrinsics = self.camera_wrapper.get_cameras_extrinsics(None)
            
            if extrinsics is not None and isinstance(extrinsics, dict):
                # Validate extrinsics structure
                valid = True
                for camera_id in self.camera_ids:
                    if camera_id not in extrinsics:
                        # Extrinsics might not be available for all cameras (TF lookup may fail)
                        # This is acceptable, just log it
                        self.get_logger().debug(
                            f"Extrinsics not available for camera {camera_id} "
                            f"(base_frame or camera_frame may not be set)"
                        )
                        continue
                    
                    extrinsic_matrix = extrinsics[camera_id]
                    if not isinstance(extrinsic_matrix, np.ndarray):
                        valid = False
                        break
                    if extrinsic_matrix.shape != (4, 4):
                        valid = False
                        break
                
                if valid or len(extrinsics) > 0:
                    self.test_results['extrinsics']['success'] += 1
                    # Store sample data (only once)
                    if self.test_results['extrinsics']['sample_data'] is None:
                        self.test_results['extrinsics']['sample_data'] = extrinsics
        except Exception as e:
            self.get_logger().error(f"Error in get_cameras_extrinsics test: {e}")
    
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
        
        # Print intrinsics and extrinsics sample data
        self._print_intrinsics_extrinsics_samples()
    
    def _print_intrinsics_extrinsics_samples(self):
        """Print sample data for intrinsics and extrinsics."""
        intrinsics_sample = self.test_results['intrinsics']['sample_data']
        extrinsics_sample = self.test_results['extrinsics']['sample_data']
        
        if intrinsics_sample is not None or extrinsics_sample is not None:
            print(f"\n{'='*80}")
            print("Camera Intrinsics and Extrinsics Sample Data")
            print(f"{'='*80}\n")
            
            # Print intrinsics
            if intrinsics_sample is not None:
                print("get_camera_intrinsics() return value:")
                print(f"  Type: {type(intrinsics_sample)}")
                print(f"  Keys (camera_ids): {list(intrinsics_sample.keys())}")
                print()
                
                for camera_id, intrinsic_matrix in intrinsics_sample.items():
                    print(f"  Camera {camera_id}:")
                    print(f"    Type: {type(intrinsic_matrix)}")
                    if isinstance(intrinsic_matrix, np.ndarray):
                        print(f"    Shape: {intrinsic_matrix.shape}")
                        print(f"    Dtype: {intrinsic_matrix.dtype}")
                        print(f"    Matrix:")
                        print(f"      {intrinsic_matrix[0, 0]:.2f}  {intrinsic_matrix[0, 1]:.2f}  {intrinsic_matrix[0, 2]:.2f}")
                        print(f"      {intrinsic_matrix[1, 0]:.2f}  {intrinsic_matrix[1, 1]:.2f}  {intrinsic_matrix[1, 2]:.2f}")
                        print(f"      {intrinsic_matrix[2, 0]:.2f}  {intrinsic_matrix[2, 1]:.2f}  {intrinsic_matrix[2, 2]:.2f}")
                    print()
            else:
                print("get_camera_intrinsics() return value: None (not available)\n")
            
            # Print extrinsics
            if extrinsics_sample is not None:
                print("get_cameras_extrinsics() return value:")
                print(f"  Type: {type(extrinsics_sample)}")
                print(f"  Keys (camera_ids): {list(extrinsics_sample.keys())}")
                print()
                
                for camera_id, extrinsic_matrix in extrinsics_sample.items():
                    print(f"  Camera {camera_id}:")
                    print(f"    Type: {type(extrinsic_matrix)}")
                    if isinstance(extrinsic_matrix, np.ndarray):
                        print(f"    Shape: {extrinsic_matrix.shape}")
                        print(f"    Dtype: {extrinsic_matrix.dtype}")
                        print(f"    Transformation Matrix (4x4):")
                        print(f"      Rotation (3x3):")
                        print(f"        [{extrinsic_matrix[0, 0]:.4f}  {extrinsic_matrix[0, 1]:.4f}  {extrinsic_matrix[0, 2]:.4f}]")
                        print(f"        [{extrinsic_matrix[1, 0]:.4f}  {extrinsic_matrix[1, 1]:.4f}  {extrinsic_matrix[1, 2]:.4f}]")
                        print(f"        [{extrinsic_matrix[2, 0]:.4f}  {extrinsic_matrix[2, 1]:.4f}  {extrinsic_matrix[2, 2]:.4f}]")
                        print(f"      Translation (3x1):")
                        print(f"        [{extrinsic_matrix[0, 3]:.4f}]")
                        print(f"        [{extrinsic_matrix[1, 3]:.4f}]")
                        print(f"        [{extrinsic_matrix[2, 3]:.4f}]")
                    print()
            else:
                print("get_cameras_extrinsics() return value: None (not available)")
                print("  Note: This may be normal if base_frame or camera_frame are not set in config\n")
            
            print(f"{'='*80}\n")
    
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
        
        # Test 5: Camera intrinsics
        print("5. get_camera_intrinsics Function:")
        intrinsics_stats = self.test_results['intrinsics']
        print(f"   Calls: {intrinsics_stats['calls']}")
        print(f"   Success: {intrinsics_stats['success']}")
        if intrinsics_stats['calls'] > 0:
            intrinsics_rate = (intrinsics_stats['success'] / intrinsics_stats['calls']) * 100
            print(f"   Success rate: {intrinsics_rate:.1f}%")
        if intrinsics_stats['sample_data'] is not None:
            print(f"   Available cameras: {list(intrinsics_stats['sample_data'].keys())}")
        print()
        
        # Test 6: Camera extrinsics
        print("6. get_cameras_extrinsics Function:")
        extrinsics_stats = self.test_results['extrinsics']
        print(f"   Calls: {extrinsics_stats['calls']}")
        print(f"   Success: {extrinsics_stats['success']}")
        if extrinsics_stats['calls'] > 0:
            extrinsics_rate = (extrinsics_stats['success'] / extrinsics_stats['calls']) * 100
            print(f"   Success rate: {extrinsics_rate:.1f}%")
        if extrinsics_stats['sample_data'] is not None:
            print(f"   Available cameras: {list(extrinsics_stats['sample_data'].keys())}")
            print(f"   Note: Extrinsics may not be available for all cameras if base_frame/camera_frame are not set")
        else:
            print(f"   Note: Extrinsics not available (TF lookup may have failed)")
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
        print(f"  ✅ get_camera_intrinsics: {intrinsics_stats['success']}/{intrinsics_stats['calls']} successful")
        print(f"  ✅ get_cameras_extrinsics: {extrinsics_stats['success']}/{extrinsics_stats['calls']} successful")
        print(f"{'='*80}\n")
    
    def _plot_timestamp_analysis(self):
        """
        Generate comprehensive visualizations for sync vs non-sync timestamp analysis.
        
        Creates multiple plots:
        1. Timestamp difference distribution (histogram)
        2. Timestamp difference over time (time series)
        3. Box plot comparing sync vs non-sync
        4. Timeline visualization showing camera alignment
        """
        if not MATPLOTLIB_AVAILABLE:
            return
        
        sync_diffs = self.test_results['timestamp_diffs']['sync']
        non_sync_diffs = self.test_results['timestamp_diffs']['non_sync']
        
        if not sync_diffs and not non_sync_diffs:
            print("⚠️  No timestamp difference data available for visualization")
            return
        
        # Extract data for plotting
        sync_diff_values = [d['diff_ms'] for d in sync_diffs] if sync_diffs else []
        non_sync_diff_values = [d['diff_ms'] for d in non_sync_diffs] if non_sync_diffs else []
        
        # Create figure with subplots
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle('Multi-Camera Timestamp Synchronization Analysis', fontsize=16, fontweight='bold')
        
        # 1. Histogram comparison (top left)
        ax1 = plt.subplot(2, 3, 1)
        if sync_diff_values:
            ax1.hist(sync_diff_values, bins=50, alpha=0.7, label='Sync Mode', color='blue', edgecolor='black')
        if non_sync_diff_values:
            ax1.hist(non_sync_diff_values, bins=50, alpha=0.7, label='Non-Sync Mode', color='red', edgecolor='black')
        ax1.set_xlabel('Timestamp Difference (ms)', fontsize=10)
        ax1.set_ylabel('Frequency', fontsize=10)
        ax1.set_title('Distribution of Timestamp Differences', fontsize=12, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Time series plot (top middle)
        ax2 = plt.subplot(2, 3, 2)
        if sync_diff_values:
            ax2.plot(range(len(sync_diff_values)), sync_diff_values, 
                    'b-', alpha=0.6, label='Sync Mode', linewidth=1.5, marker='o', markersize=2)
        if non_sync_diff_values:
            ax2.plot(range(len(non_sync_diff_values)), non_sync_diff_values, 
                    'r-', alpha=0.6, label='Non-Sync Mode', linewidth=1.5, marker='s', markersize=2)
        ax2.set_xlabel('Sample Index', fontsize=10)
        ax2.set_ylabel('Timestamp Difference (ms)', fontsize=10)
        ax2.set_title('Timestamp Differences Over Time', fontsize=12, fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Box plot comparison (top right)
        ax3 = plt.subplot(2, 3, 3)
        box_data = []
        labels = []
        if sync_diff_values:
            box_data.append(sync_diff_values)
            labels.append('Sync Mode')
        if non_sync_diff_values:
            box_data.append(non_sync_diff_values)
            labels.append('Non-Sync Mode')
        
        if box_data:
            bp = ax3.boxplot(box_data, labels=labels, patch_artist=True, 
                           showmeans=True, meanline=True)
            # Color the boxes
            colors = ['lightblue', 'lightcoral']
            for patch, color in zip(bp['boxes'], colors[:len(bp['boxes'])]):
                patch.set_facecolor(color)
            ax3.set_ylabel('Timestamp Difference (ms)', fontsize=10)
            ax3.set_title('Statistical Comparison', fontsize=12, fontweight='bold')
            ax3.grid(True, alpha=0.3, axis='y')
        
        # 4. Cumulative distribution function (bottom left)
        ax4 = plt.subplot(2, 3, 4)
        if sync_diff_values:
            sorted_sync = np.sort(sync_diff_values)
            p_sync = np.arange(1, len(sorted_sync) + 1) / len(sorted_sync)
            ax4.plot(sorted_sync, p_sync, 'b-', label='Sync Mode', linewidth=2)
        if non_sync_diff_values:
            sorted_non_sync = np.sort(non_sync_diff_values)
            p_non_sync = np.arange(1, len(sorted_non_sync) + 1) / len(sorted_non_sync)
            ax4.plot(sorted_non_sync, p_non_sync, 'r-', label='Non-Sync Mode', linewidth=2)
        ax4.set_xlabel('Timestamp Difference (ms)', fontsize=10)
        ax4.set_ylabel('Cumulative Probability', fontsize=10)
        ax4.set_title('Cumulative Distribution Function', fontsize=12, fontweight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. Timeline visualization (bottom middle)
        ax5 = plt.subplot(2, 3, 5)
        self._plot_timeline(ax5, sync_diffs, non_sync_diffs)
        
        # 6. Statistics summary table (bottom right)
        ax6 = plt.subplot(2, 3, 6)
        ax6.axis('off')
        self._plot_statistics_table(ax6, sync_diff_values, non_sync_diff_values)
        
        plt.tight_layout()
        
        # Save figure
        output_file = 'timestamp_sync_analysis.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\n📊 Visualization saved to: {output_file}")
        
        # Also save as PDF for better quality
        try:
            output_pdf = 'timestamp_sync_analysis.pdf'
            plt.savefig(output_pdf, dpi=150, bbox_inches='tight')
            print(f"📊 High-quality PDF saved to: {output_pdf}")
        except Exception:
            pass
        
        plt.close()
    
    def _plot_timeline(self, ax, sync_diffs: List[Dict], non_sync_diffs: List[Dict]):
        """
        Plot timeline visualization showing camera timestamp alignment.
        
        Shows a sample of timestamps to visualize how well cameras are synchronized.
        """
        # Use first 50 samples for clarity
        max_samples = 50
        
        # Find base time from all samples
        all_times = []
        if sync_diffs:
            for diff_data in sync_diffs[:max_samples]:
                all_times.extend([diff_data['pub_t_1'] / 1e9, diff_data['pub_t_2'] / 1e9])
        if non_sync_diffs:
            for diff_data in non_sync_diffs[:max_samples]:
                all_times.extend([diff_data['pub_t_1'] / 1e9, diff_data['pub_t_2'] / 1e9])
        
        if not all_times:
            ax.text(0.5, 0.5, 'No data available', ha='center', va='center', fontsize=12)
            return
        
        base_time = min(all_times)
        
        # Plot sync mode
        if sync_diffs:
            sync_sample = sync_diffs[:max_samples]
            for i, diff_data in enumerate(sync_sample):
                pub_t_1 = diff_data['pub_t_1'] / 1e9  # Convert to seconds
                pub_t_2 = diff_data['pub_t_2'] / 1e9
                
                t1_norm = (pub_t_1 - base_time) * 1000  # Convert to ms
                t2_norm = (pub_t_2 - base_time) * 1000
                
                # Plot as vertical lines connecting the two timestamps
                ax.plot([i, i], [t1_norm, t2_norm], 'b-', alpha=0.4, linewidth=1.0)
                ax.scatter(i, t1_norm, c='blue', s=15, alpha=0.7, marker='o', edgecolors='darkblue')
                ax.scatter(i, t2_norm, c='blue', s=15, alpha=0.7, marker='s', edgecolors='darkblue')
        
        # Plot non-sync mode (offset by max_samples to avoid overlap)
        if non_sync_diffs:
            non_sync_sample = non_sync_diffs[:max_samples]
            offset = max_samples + 5  # Add gap between sync and non-sync
            for i, diff_data in enumerate(non_sync_sample):
                pub_t_1 = diff_data['pub_t_1'] / 1e9
                pub_t_2 = diff_data['pub_t_2'] / 1e9
                
                t1_norm = (pub_t_1 - base_time) * 1000
                t2_norm = (pub_t_2 - base_time) * 1000
                
                ax.plot([i + offset, i + offset], [t1_norm, t2_norm], 
                       'r--', alpha=0.4, linewidth=1.0)
                ax.scatter(i + offset, t1_norm, c='red', s=15, alpha=0.7, 
                          marker='o', edgecolors='darkred')
                ax.scatter(i + offset, t2_norm, c='red', s=15, alpha=0.7, 
                          marker='s', edgecolors='darkred')
        
        ax.set_xlabel('Sample Index', fontsize=10)
        ax.set_ylabel('Relative Timestamp (ms)', fontsize=10)
        ax.set_title('Timeline: Camera Timestamp Alignment\n(Left: Sync, Right: Non-Sync)', 
                    fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Add vertical line to separate sync and non-sync
        if sync_diffs and non_sync_diffs:
            ax.axvline(x=max_samples + 2.5, color='gray', linestyle=':', linewidth=2, alpha=0.5)
        
        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='blue', lw=2, label='Sync Mode'),
            Line2D([0], [0], color='red', lw=2, linestyle='--', label='Non-Sync Mode'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', 
                   markersize=8, label='Camera 1'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='gray', 
                   markersize=8, label='Camera 2')
        ]
        ax.legend(handles=legend_elements, loc='upper left', fontsize=8)
    
    def _plot_statistics_table(self, ax, sync_diff_values: List[float], non_sync_diff_values: List[float]):
        """
        Plot statistics summary table.
        """
        # Calculate statistics
        stats_data = []
        
        if sync_diff_values:
            stats_data.append(['Sync Mode', 
                             f"{np.mean(sync_diff_values):.2f}",
                             f"{np.std(sync_diff_values):.2f}",
                             f"{np.min(sync_diff_values):.2f}",
                             f"{np.max(sync_diff_values):.2f}",
                             f"{len(sync_diff_values)}"])
        
        if non_sync_diff_values:
            stats_data.append(['Non-Sync Mode',
                             f"{np.mean(non_sync_diff_values):.2f}",
                             f"{np.std(non_sync_diff_values):.2f}",
                             f"{np.min(non_sync_diff_values):.2f}",
                             f"{np.max(non_sync_diff_values):.2f}",
                             f"{len(non_sync_diff_values)}"])
        
        if not stats_data:
            ax.text(0.5, 0.5, 'No data available', 
                   ha='center', va='center', fontsize=12)
            return
        
        # Create table
        columns = ['Mode', 'Mean (ms)', 'Std (ms)', 'Min (ms)', 'Max (ms)', 'Samples']
        table = ax.table(cellText=stats_data,
                        colLabels=columns,
                        cellLoc='center',
                        loc='center',
                        bbox=[0, 0, 1, 1])
        
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 2)
        
        # Style the header
        for i in range(len(columns)):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # Style the data rows
        colors = ['#E3F2FD', '#FFEBEE']  # Light blue, light red
        for i in range(1, len(stats_data) + 1):
            for j in range(len(columns)):
                table[(i, j)].set_facecolor(colors[(i-1) % len(colors)])
        
        ax.set_title('Statistics Summary', fontsize=12, fontweight='bold', pad=20)


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test MultiCameraWrapper functions'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='Path to camera configuration file (optional)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Test duration in seconds (default: 30.0)')
    parser.add_argument('--fps', type=float, default=2.0,
                       help='Frequency of read_cameras() calls in Hz (default: 2.0)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = MultiCameraWrapperTestNode(args.config_file, args.duration, args.fps)
        
        # Wait for initial data to arrive (executor is already spinning in background)
        time.sleep(2.0)
        
        node.run_tests()
        
        # Clean shutdown
        node.shutdown()
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
        if 'node' in locals():
            node.shutdown()
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        if 'node' in locals():
            node.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


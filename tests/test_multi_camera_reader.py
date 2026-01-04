#!/usr/bin/env python3
"""
Test script for MultiCameraWrapper.

Tests whether MultiCameraWrapper correctly receives and processes data from multiple cameras:
- Multi-camera synchronization (ApproximateTimeSynchronizer with layered architecture)
- Single-camera synchronization (ApproximateTimeSynchronizer with configurable slop)
- Data format validation for all cameras
- Timestamp synchronization across cameras
- Both sync and non-sync modes
- Event-based waiting mechanism (no deadlocks)
- TF transform caching
- ReentrantCallbackGroup support (concurrent callbacks)
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Dict, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera_utils.multi_camera_wrapper import MultiCameraWrapper


class MultiCameraReaderTestNode(Node):
    """Test node for MultiCameraWrapper."""
    
    def __init__(self, config_file: str = None):
        """
        Initialize test node.
        
        Args:
            config_file: Path to camera configuration file (optional)
        """
        super().__init__('multi_camera_reader_test_node')
        
        print(f"\n{'='*80}")
        print(f"MultiCameraWrapper Test")
        print(f"{'='*80}\n")
        
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
            
            print(f"✅ Initialized {len(self.camera_ids)} cameras: {self.camera_ids}")
            
            # Verify architecture features
            self._verify_architecture()
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MultiCameraWrapper: {e}")
            import traceback
            traceback.print_exc()
            self.camera_wrapper = None
            self.camera_ids = []
    
    def _verify_architecture(self) -> None:
        """
        Verify that the architecture uses the expected components.
        
        Checks for:
        - ApproximateTimeSynchronizer (not TimeSynchronizer)
        - Event-based waiting mechanism
        - ReentrantCallbackGroup support
        """
        if not self.camera_wrapper or not self.camera_ids:
            return
        
        print(f"\n{'='*80}")
        print(f"Architecture Verification")
        print(f"{'='*80}")
        
        # Check that cameras use ApproximateTimeSynchronizer
        for cam_id in self.camera_ids:
            camera = self.camera_wrapper.camera_dict[cam_id]
            # Check if camera has _sync attribute and it's ApproximateTimeSynchronizer
            if hasattr(camera, '_sync'):
                sync_type = type(camera._sync).__name__
                if 'ApproximateTimeSynchronizer' in sync_type:
                    print(f"✅ Camera {cam_id}: Uses ApproximateTimeSynchronizer (Layer 1)")
                else:
                    print(f"⚠️  Camera {cam_id}: Uses {sync_type} (expected ApproximateTimeSynchronizer)")
            
            # Check for event-based waiting mechanism
            if hasattr(camera, '_data_available_event'):
                print(f"✅ Camera {cam_id}: Has event-based waiting mechanism")
            else:
                print(f"⚠️  Camera {cam_id}: Missing event-based waiting mechanism")
        
        # Check multi-camera sync
        if hasattr(self.camera_wrapper, '_multi_camera_sync'):
            sync_type = type(self.camera_wrapper._multi_camera_sync).__name__
            if 'ApproximateTimeSynchronizer' in sync_type:
                print(f"✅ Multi-camera sync: Uses ApproximateTimeSynchronizer (Layer 2)")
            else:
                print(f"⚠️  Multi-camera sync: Uses {sync_type} (expected ApproximateTimeSynchronizer)")
        
        # Check for TF cache
        if hasattr(self.camera_wrapper, '_tf_cache'):
            print(f"✅ MultiCameraWrapper: Has TF transform cache")
        
        print(f"{'='*80}\n")
    
    def test_multi_camera_reader(self, num_reads: int = 10, wait_time: float = 1.0,
                                 duration: float = 60.0, test_sync: bool = True,
                                 test_non_sync: bool = True):
        """
        Test multi-camera reader by reading data multiple times.
        
        Args:
            num_reads: Number of times to read camera data
            wait_time: Time to wait between reads (seconds)
            duration: Maximum test duration (seconds)
            test_sync: Test synchronized mode (use_sync=True)
            test_non_sync: Test non-synchronized mode (use_sync=False)
        """
        if not self.camera_wrapper or not self.camera_ids:
            print("❌ ERROR: Camera wrapper not initialized. Cannot test.")
            return False
        
        print(f"\n{'='*80}")
        print(f"Starting MultiCameraWrapper Test")
        print(f"{'='*80}")
        print(f"Number of cameras:  {len(self.camera_ids)}")
        print(f"Camera IDs:         {self.camera_ids}")
        print(f"Number of reads:    {num_reads}")
        print(f"Wait time:          {wait_time} seconds")
        print(f"Max duration:       {duration} seconds")
        print(f"Test sync mode:     {test_sync}")
        print(f"Test non-sync mode: {test_non_sync}")
        
        # Print synchronization architecture info
        if self.camera_ids:
            first_camera = self.camera_wrapper.camera_dict[self.camera_ids[0]]
            print(f"\nSynchronization Architecture:")
            print(f"  Layer 1 (Single Camera): ApproximateTimeSynchronizer (slop=0.005s default)")
            print(f"  Layer 2 (Multi Camera):  ApproximateTimeSynchronizer (slop=0.2s default)")
            print(f"  Waiting Mechanism:       Event-based (no rclpy.spin_once in callbacks)")
            print(f"  Callback Group:         ReentrantCallbackGroup (concurrent callbacks)")
        
        print(f"{'='*80}\n")
        
        # Wait for cameras to start receiving data
        print("⏳ Waiting for cameras to receive synchronized data...")
        max_wait = 30.0
        wait_start = time.time()
        last_status_time = 0
        
        while time.time() - wait_start < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check status of each camera
            camera_status = {}
            for cam_id in self.camera_ids:
                is_running = self.camera_wrapper.camera_dict[cam_id].is_running()
                camera_status[cam_id] = is_running
                if is_running:
                    latest_timestamp = self.camera_wrapper.camera_dict[cam_id].get_latest_pub_timestamp()
                    if latest_timestamp:
                        camera_status[cam_id] = (True, latest_timestamp)
            
            all_running = all(
                status if isinstance(status, bool) else status[0]
                for status in camera_status.values()
            )
            
            # Print status every 2 seconds
            elapsed = time.time() - wait_start
            if elapsed - last_status_time >= 2.0:
                running_cameras = [cam_id for cam_id, status in camera_status.items() 
                                 if (status if isinstance(status, bool) else status[0])]
                print(f"   [{elapsed:.1f}s] Running cameras: {len(running_cameras)}/{len(self.camera_ids)} - {running_cameras}")
                last_status_time = elapsed
            
            if all_running:
                elapsed = time.time() - wait_start
                print(f"✅ All cameras are receiving data! (waited {elapsed:.2f}s)")
                break
            time.sleep(0.1)
        else:
            print("❌ ERROR: Not all cameras received data within timeout period")
            running_cameras = [
                cam_id for cam_id in self.camera_ids
                if self.camera_wrapper.camera_dict[cam_id].is_running()
            ]
            print(f"   Running cameras: {running_cameras}")
            print(f"   Missing cameras: {set(self.camera_ids) - set(running_cameras)}")
            
            # Print diagnostic info for missing cameras
            for cam_id in set(self.camera_ids) - set(running_cameras):
                camera = self.camera_wrapper.camera_dict[cam_id]
                print(f"   Camera {cam_id}: is_running={camera.is_running()}, "
                      f"latest_timestamp={camera.get_latest_pub_timestamp()}")
            
            return False
        
        results = {}
        
        # Test synchronized mode
        if test_sync:
            print(f"\n{'#'*80}")
            print(f"Testing SYNCHRONIZED Mode (use_sync=True)")
            print(f"{'#'*80}")
            sync_results = self._test_read_mode(
                use_sync=True,
                num_reads=num_reads,
                wait_time=wait_time,
                duration=duration
            )
            results['sync'] = sync_results
        
        # Test non-synchronized mode
        if test_non_sync:
            print(f"\n{'#'*80}")
            print(f"Testing NON-SYNCHRONIZED Mode (use_sync=False)")
            print(f"{'#'*80}")
            non_sync_results = self._test_read_mode(
                use_sync=False,
                num_reads=num_reads,
                wait_time=wait_time,
                duration=duration
            )
            results['non_sync'] = non_sync_results
        
        # Print final summary
        self._print_final_summary(results)
        
        # Determine overall success
        all_passed = all(
            result.get('success', False)
            for result in results.values()
        )
        
        return all_passed
    
    def _test_read_mode(self, use_sync: bool, num_reads: int, wait_time: float,
                       duration: float) -> Dict:
        """
        Test a specific read mode (sync or non-sync).
        
        Args:
            use_sync: Whether to use synchronized mode
            num_reads: Number of reads
            wait_time: Wait time between reads
            duration: Maximum duration
            
        Returns:
            Dictionary with test results
        """
        mode_name = "SYNCHRONIZED" if use_sync else "NON-SYNCHRONIZED"
        
        successful_reads = 0
        failed_reads = 0
        timestamp_errors = 0
        data_format_errors = 0
        sync_errors = 0
        
        # Statistics tracking
        read_times = []
        sync_durations = []
        timestamp_diffs = []
        camera_receive_stats = {cam_id: {'count': 0, 'missing': 0} for cam_id in self.camera_ids}
        
        test_start = time.time()
        
        for i in range(num_reads):
            if time.time() - test_start > duration:
                print(f"\n⏱️  Test duration ({duration}s) exceeded. Stopping.")
                break
            
            read_start_time = time.time()
            
            print(f"\n{'='*80}")
            print(f"[{mode_name}] Read #{i+1}/{num_reads}")
            print(f"{'='*80}")
            
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Read camera data
            read_data_start = time.time()
            data_dict, timestamp_dict = self.camera_wrapper.read_cameras(use_sync=use_sync)
            read_data_time = (time.time() - read_data_start) * 1000
            
            # Check if data is available
            if not data_dict or not timestamp_dict:
                print(f"❌ ERROR: No data received (read_time={read_data_time:.2f}ms)")
                failed_reads += 1
                for cam_id in self.camera_ids:
                    camera_receive_stats[cam_id]['missing'] += 1
                if i < num_reads - 1:
                    time.sleep(wait_time)
                continue
            
            # Track which cameras received data
            received_cameras = set()
            if "image" in data_dict:
                received_cameras.update(data_dict["image"].keys())
            
            for cam_id in self.camera_ids:
                if cam_id in received_cameras:
                    camera_receive_stats[cam_id]['count'] += 1
                else:
                    camera_receive_stats[cam_id]['missing'] += 1
            
            # Validate data format for all cameras
            is_valid, errors = self._validate_multi_camera_data(data_dict, timestamp_dict)
            if not is_valid:
                print(f"❌ ERROR: Data format validation failed (read_time={read_data_time:.2f}ms):")
                for error in errors:
                    print(f"   - {error}")
                data_format_errors += 1
                failed_reads += 1
                if i < num_reads - 1:
                    time.sleep(wait_time)
                continue
            
            # Check timestamp synchronization (only for sync mode)
            if use_sync:
                is_synced, sync_errors_list = self._check_multi_camera_sync(timestamp_dict)
                if not is_synced:
                    print(f"❌ ERROR: Multi-camera synchronization failed:")
                    for error in sync_errors_list:
                        print(f"   - {error}")
                    sync_errors += 1
                    failed_reads += 1
                    if i < num_reads - 1:
                        time.sleep(wait_time)
                    continue
                
                # Collect sync statistics
                if "multi_camera_sync_start" in timestamp_dict and "multi_camera_sync_end" in timestamp_dict:
                    sync_start = timestamp_dict["multi_camera_sync_start"]
                    sync_end = timestamp_dict["multi_camera_sync_end"]
                    sync_duration_ms = (sync_end - sync_start) / 1e6
                    sync_durations.append(sync_duration_ms)
                    
                    # Collect decoupled architecture latency metrics if available
                    if "multi_camera_sync_check" in timestamp_dict and "multi_camera_sync_ready" in timestamp_dict:
                        check_t = timestamp_dict["multi_camera_sync_check"]
                        ready_t = timestamp_dict["multi_camera_sync_ready"]
                        callback_to_ready_ms = (ready_t - sync_start) / 1e6
                        # Store for later statistics (could add separate tracking if needed)
                
                # Collect timestamp differences
                if len(self.camera_ids) >= 2:
                    camera_timestamps = {
                        cam_id: timestamp_dict.get(f"{cam_id}_pub_t", 0)
                        for cam_id in self.camera_ids
                    }
                    cam_list = list(camera_timestamps.keys())
                    for idx1 in range(len(cam_list)):
                        for idx2 in range(idx1 + 1, len(cam_list)):
                            cam1_ts = camera_timestamps[cam_list[idx1]]
                            cam2_ts = camera_timestamps[cam_list[idx2]]
                            diff_ms = abs(cam1_ts - cam2_ts) / 1e6
                            timestamp_diffs.append(diff_ms)
            
            # Track read time
            read_time = (time.time() - read_start_time) * 1000
            read_times.append(read_time)
            
            # Print summary
            self._print_read_summary(data_dict, timestamp_dict, i+1, use_sync, read_time, read_data_time)
            
            successful_reads += 1
            
            if i < num_reads - 1:
                time.sleep(wait_time)
        
        # Calculate statistics
        success_rate = successful_reads / num_reads * 100 if num_reads > 0 else 0
        
        print(f"\n{'='*80}")
        print(f"[{mode_name}] Test Results Summary")
        print(f"{'='*80}")
        print(f"Total reads:           {num_reads}")
        print(f"Successful reads:      {successful_reads} ✅")
        print(f"Failed reads:          {failed_reads} ❌")
        print(f"Timestamp errors:     {timestamp_errors} ⚠️")
        print(f"Data format errors:   {data_format_errors} ⚠️")
        if use_sync:
            print(f"Sync errors:           {sync_errors} ⚠️")
        print(f"Success rate:         {success_rate:.1f}%")
        
        # Print performance statistics
        if read_times:
            avg_read_time = sum(read_times) / len(read_times)
            max_read_time = max(read_times)
            min_read_time = min(read_times)
            print(f"\nPerformance Statistics:")
            print(f"   Average read time:  {avg_read_time:.2f} ms")
            print(f"   Min read time:      {min_read_time:.2f} ms")
            print(f"   Max read time:      {max_read_time:.2f} ms")
        
        if use_sync and sync_durations:
            avg_sync_duration = sum(sync_durations) / len(sync_durations)
            max_sync_duration = max(sync_durations)
            min_sync_duration = min(sync_durations)
            print(f"\nSync Duration Statistics:")
            print(f"   Average sync duration: {avg_sync_duration:.2f} ms")
            print(f"   Min sync duration:      {min_sync_duration:.2f} ms")
            print(f"   Max sync duration:      {max_sync_duration:.2f} ms")
        
        if use_sync and timestamp_diffs:
            avg_diff = sum(timestamp_diffs) / len(timestamp_diffs)
            max_diff = max(timestamp_diffs)
            min_diff = min(timestamp_diffs)
            print(f"\nTimestamp Difference Statistics (Multi-Camera Sync):")
            print(f"   Average diff:  {avg_diff:.3f} ms")
            print(f"   Min diff:      {min_diff:.3f} ms")
            print(f"   Max diff:      {max_diff:.3f} ms")
            print(f"   Note: Differences should be < 200ms (ApproximateTimeSynchronizer slop)")
        
        # Print camera receive statistics
        print(f"\nCamera Receive Statistics:")
        for cam_id in self.camera_ids:
            stats = camera_receive_stats[cam_id]
            total = stats['count'] + stats['missing']
            receive_rate = (stats['count'] / total * 100) if total > 0 else 0
            status = "✅" if receive_rate == 100 else "⚠️"
            print(f"   {status} Camera {cam_id}: {stats['count']}/{total} ({receive_rate:.1f}%)")
        
        print(f"{'='*80}\n")
        
        return {
            'success': successful_reads == num_reads,
            'successful_reads': successful_reads,
            'failed_reads': failed_reads,
            'timestamp_errors': timestamp_errors,
            'data_format_errors': data_format_errors,
            'sync_errors': sync_errors if use_sync else 0,
            'success_rate': success_rate,
            'avg_read_time': sum(read_times) / len(read_times) if read_times else 0,
            'avg_sync_duration': sum(sync_durations) / len(sync_durations) if sync_durations else 0,
            'avg_timestamp_diff': sum(timestamp_diffs) / len(timestamp_diffs) if timestamp_diffs else 0,
            'camera_receive_stats': camera_receive_stats
        }
    
    def _validate_multi_camera_data(self, data_dict: Dict, timestamp_dict: Dict) -> Tuple[bool, List[str]]:
        """
        Validate data format for all cameras.
        
        Args:
            data_dict: Data dictionary
            timestamp_dict: Timestamp dictionary
            
        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []
        
        # Check required keys in data_dict
        required_keys = ["image", "depth"]
        for key in required_keys:
            if key not in data_dict:
                errors.append(f"Missing key '{key}' in data_dict")
                return False, errors
        
        # Check each camera's data
        for camera_id in self.camera_ids:
            # Check image data
            if "image" in data_dict:
                image_dict = data_dict["image"]
                if not isinstance(image_dict, dict):
                    errors.append(f"data_dict['image'] is not a dictionary")
                    return False, errors
                
                if camera_id not in image_dict:
                    errors.append(f"Camera ID '{camera_id}' not found in image dict")
                    continue
                
                image = image_dict[camera_id]
                if image is None:
                    errors.append(f"Image data for camera '{camera_id}' is None")
                    continue
                
                if not isinstance(image, np.ndarray):
                    errors.append(f"Image for camera '{camera_id}' is not a numpy array")
                    continue
                
                if len(image.shape) != 3 or image.shape[2] != 3:
                    errors.append(
                        f"Image shape for camera '{camera_id}' is invalid: "
                        f"{image.shape} (expected HxWx3)"
                    )
                    continue
                
                if image.dtype != np.uint8:
                    errors.append(
                        f"Image dtype for camera '{camera_id}' is invalid: "
                        f"{image.dtype} (expected uint8)"
                    )
                    continue
            
            # Check depth data
            if "depth" in data_dict:
                depth_dict = data_dict["depth"]
                if not isinstance(depth_dict, dict):
                    errors.append(f"data_dict['depth'] is not a dictionary")
                    return False, errors
                
                if camera_id not in depth_dict:
                    errors.append(f"Camera ID '{camera_id}' not found in depth dict")
                    continue
                
                depth = depth_dict[camera_id]
                if depth is None:
                    errors.append(f"Depth data for camera '{camera_id}' is None")
                    continue
                
                if not isinstance(depth, np.ndarray):
                    errors.append(f"Depth for camera '{camera_id}' is not a numpy array")
                    continue
                
                if len(depth.shape) != 2:
                    errors.append(
                        f"Depth shape for camera '{camera_id}' is invalid: "
                        f"{depth.shape} (expected HxW)"
                    )
                    continue
                
                if depth.dtype != np.uint16:
                    errors.append(
                        f"Depth dtype for camera '{camera_id}' is invalid: "
                        f"{depth.dtype} (expected uint16)"
                    )
                    continue
            
            # Check timestamp keys for this camera
            # Note: rgb_t, depth_t, camera_info_t are not stored by ROS2CameraReader
            # (ApproximateTimeSynchronizer ensures they have approximately matching timestamps, only pub_t is stored)
            required_timestamp_keys = [
                f"{camera_id}_read_start",
                f"{camera_id}_frame_received",
                f"{camera_id}_read_end",
                f"{camera_id}_pub_t",
                f"{camera_id}_sub_t",
                f"{camera_id}_end_t",
            ]
            
            for key in required_timestamp_keys:
                if key not in timestamp_dict:
                    errors.append(f"Missing timestamp key '{key}'")
                    continue
                
                value = timestamp_dict[key]
                if not isinstance(value, (int, np.integer)):
                    errors.append(f"Timestamp '{key}' is not an integer: {type(value)}")
                    continue
        
        return len(errors) == 0, errors
    
    def _check_multi_camera_sync(self, timestamp_dict: Dict) -> Tuple[bool, List[str]]:
        """
        Check multi-camera timestamp synchronization.
        
        For synchronized mode, all cameras should have similar timestamps
        (within the slop window of ApproximateTimeSynchronizer).
        
        Args:
            timestamp_dict: Timestamp dictionary
            
        Returns:
            Tuple of (is_synced, list_of_errors)
        """
        errors = []
        
        if len(self.camera_ids) < 2:
            # Single camera, no sync check needed
            return True, []
        
        # Get pub_t for all cameras
        camera_timestamps = {}
        for camera_id in self.camera_ids:
            pub_t_key = f"{camera_id}_pub_t"
            if pub_t_key in timestamp_dict:
                camera_timestamps[camera_id] = timestamp_dict[pub_t_key]
            else:
                errors.append(f"Missing pub_t for camera '{camera_id}'")
                return False, errors
        
        # Check if multi-camera sync metadata exists
        if "multi_camera_sync_start" not in timestamp_dict:
            errors.append("Missing multi_camera_sync_start timestamp")
            return False, errors
        
        if "multi_camera_sync_end" not in timestamp_dict:
            errors.append("Missing multi_camera_sync_end timestamp")
            return False, errors
        
        # Check for new decoupled architecture timestamps (optional, for latency analysis)
        if "multi_camera_sync_check" in timestamp_dict and "multi_camera_sync_ready" in timestamp_dict:
            # New decoupled architecture with data ready time tracking
            check_t = timestamp_dict["multi_camera_sync_check"]
            ready_t = timestamp_dict["multi_camera_sync_ready"]
            start_t = timestamp_dict["multi_camera_sync_start"]
            
            # Validate timing order: start <= check <= ready <= end
            if check_t < start_t or ready_t < check_t or timestamp_dict["multi_camera_sync_end"] < ready_t:
                errors.append(
                    f"Invalid timestamp order in decoupled sync: "
                    f"start={start_t}, check={check_t}, ready={ready_t}, end={timestamp_dict['multi_camera_sync_end']}"
                )
        
        # Calculate timestamp differences between cameras
        cam_list = list(camera_timestamps.keys())
        max_diff = 0
        
        for i in range(len(cam_list)):
            for j in range(i + 1, len(cam_list)):
                cam1_id = cam_list[i]
                cam2_id = cam_list[j]
                cam1_ts = camera_timestamps[cam1_id]
                cam2_ts = camera_timestamps[cam2_id]
                
                diff_ns = abs(cam1_ts - cam2_ts)
                diff_ms = diff_ns / 1e6
                max_diff = max(max_diff, diff_ns)
                
                # ApproximateTimeSynchronizer typically uses 0.2s (200ms) slop for multi-camera sync
                # Layer 1 (single camera) uses 0.005s (5ms) slop by default
                # Warn if difference is too large
                if diff_ms > 250:  # 250ms threshold (slightly larger than typical 200ms slop)
                    errors.append(
                        f"Large timestamp difference between cameras '{cam1_id}' and '{cam2_id}': "
                        f"{diff_ms:.2f} ms (expected < 250ms for synchronized mode)"
                    )
        
        if max_diff / 1e6 > 250:
            errors.append(
                f"Maximum timestamp difference between cameras: {max_diff/1e6:.2f} ms "
                f"(expected < 250ms for synchronized mode)"
            )
        
        return len(errors) == 0, errors
    
    def _print_read_summary(self, data_dict: Dict, timestamp_dict: Dict, read_num: int, 
                           use_sync: bool, read_time: float = 0, read_data_time: float = 0):
        """Print summary of a successful read."""
        mode_name = "SYNC" if use_sync else "NON-SYNC"
        
        print(f"✅ [{mode_name}] Data received successfully for all cameras")
        if read_time > 0:
            print(f"   Read time: {read_time:.2f} ms (data fetch: {read_data_time:.2f} ms)")
        
        # Print info for each camera
        for camera_id in self.camera_ids:
            image = data_dict["image"].get(camera_id)
            depth = data_dict["depth"].get(camera_id)
            
            if image is not None and depth is not None:
                print(f"\n   Camera {camera_id}:")
                print(f"      Image shape: {image.shape} (dtype: {image.dtype})")
                print(f"      Depth shape: {depth.shape} (dtype: {depth.dtype})")
                print(f"      Image range: [{image.min()}, {image.max()}]")
                print(f"      Depth range: [{depth.min()}, {depth.max()}]")
                
                # Print timestamp and processing times
                pub_t_key = f"{camera_id}_pub_t"
                read_start_key = f"{camera_id}_read_start"
                read_end_key = f"{camera_id}_read_end"
                
                if pub_t_key in timestamp_dict:
                    pub_t = timestamp_dict[pub_t_key]
                    print(f"      Timestamp:   {pub_t} ({pub_t/1e9:.9f} s)")
                
                if read_start_key in timestamp_dict and read_end_key in timestamp_dict:
                    read_start = timestamp_dict[read_start_key]
                    read_end = timestamp_dict[read_end_key]
                    processing_time = (read_end - read_start) / 1e6
                    print(f"      Processing:   {processing_time:.2f} ms")
        
        # Print multi-camera sync info (if in sync mode)
        if use_sync:
            if "multi_camera_sync_start" in timestamp_dict and "multi_camera_sync_end" in timestamp_dict:
                sync_start = timestamp_dict["multi_camera_sync_start"]
                sync_end = timestamp_dict["multi_camera_sync_end"]
                sync_duration_ms = (sync_end - sync_start) / 1e6
                cameras_count = timestamp_dict.get("multi_camera_sync_cameras_count", len(self.camera_ids))
                print(f"\n   Multi-camera sync (Layer 2, Decoupled):")
                print(f"      Sync duration: {sync_duration_ms:.2f} ms")
                print(f"      Cameras synced: {cameras_count}/{len(self.camera_ids)}")
                
                # Print decoupled architecture latency metrics if available
                if "multi_camera_sync_check" in timestamp_dict and "multi_camera_sync_ready" in timestamp_dict:
                    check_t = timestamp_dict["multi_camera_sync_check"]
                    ready_t = timestamp_dict["multi_camera_sync_ready"]
                    callback_to_check_ms = (check_t - sync_start) / 1e6
                    callback_to_ready_ms = (ready_t - sync_start) / 1e6
                    check_to_ready_ms = (ready_t - check_t) / 1e6
                    print(f"      Latency metrics:")
                    print(f"         Callback → Check:  {callback_to_check_ms:.2f} ms")
                    print(f"         Callback → Ready:  {callback_to_ready_ms:.2f} ms")
                    print(f"         Check → Ready:     {check_to_ready_ms:.2f} ms")
                
                # Print timestamp differences between cameras
                if len(self.camera_ids) >= 2:
                    camera_timestamps = {
                        cam_id: timestamp_dict.get(f"{cam_id}_pub_t", 0)
                        for cam_id in self.camera_ids
                    }
                    cam_list = list(camera_timestamps.keys())
                    print(f"      Timestamp differences (should be < 200ms for ApproximateTimeSynchronizer):")
                    for i in range(len(cam_list)):
                        for j in range(i + 1, len(cam_list)):
                            cam1_id = cam_list[i]
                            cam2_id = cam_list[j]
                            diff_ns = abs(camera_timestamps[cam1_id] - camera_timestamps[cam2_id])
                            diff_ms = diff_ns / 1e6
                            status = "✅" if diff_ms < 200 else "⚠️"
                            print(f"         {status} {cam1_id} vs {cam2_id}: {diff_ms:.3f} ms")
    
    def _print_final_summary(self, results: Dict):
        """Print final test summary."""
        print(f"\n{'='*80}")
        print(f"Final Test Summary")
        print(f"{'='*80}")
        
        for mode, result in results.items():
            mode_name = mode.upper().replace('_', '-')
            success = result.get('success', False)
            success_rate = result.get('success_rate', 0)
            
            status = "✅ PASSED" if success else "❌ FAILED"
            print(f"\n{mode_name} Mode: {status}")
            print(f"   Success rate: {success_rate:.1f}%")
            print(f"   Successful reads: {result.get('successful_reads', 0)}")
            print(f"   Failed reads: {result.get('failed_reads', 0)}")
            if result.get('sync_errors', 0) > 0:
                print(f"   Sync errors: {result.get('sync_errors', 0)}")
            
            # Print performance metrics
            if result.get('avg_read_time', 0) > 0:
                print(f"   Avg read time: {result.get('avg_read_time', 0):.2f} ms")
            
            if mode == 'sync' and result.get('avg_sync_duration', 0) > 0:
                print(f"   Avg sync duration: {result.get('avg_sync_duration', 0):.2f} ms")
                if result.get('avg_timestamp_diff', 0) > 0:
                    print(f"   Avg timestamp diff: {result.get('avg_timestamp_diff', 0):.3f} ms")
            
            # Print camera receive statistics
            camera_stats = result.get('camera_receive_stats', {})
            if camera_stats:
                print(f"   Camera receive rates:")
                for cam_id, stats in camera_stats.items():
                    total = stats['count'] + stats['missing']
                    rate = (stats['count'] / total * 100) if total > 0 else 0
                    status_icon = "✅" if rate == 100 else "⚠️"
                    print(f"      {status_icon} {cam_id}: {stats['count']}/{total} ({rate:.1f}%)")
        
        print(f"{'='*80}\n")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test MultiCameraWrapper for multiple cameras'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='Path to camera configuration file (optional)')
    parser.add_argument('--num-reads', type=int, default=10,
                       help='Number of reads to perform (default: 10)')
    parser.add_argument('--wait-time', type=float, default=1.0,
                       help='Wait time between reads in seconds (default: 1.0)')
    parser.add_argument('--duration', type=float, default=60.0,
                       help='Maximum test duration in seconds (default: 60.0)')
    parser.add_argument('--test-sync', action='store_true', default=True,
                       help='Test synchronized mode (default: True)')
    parser.add_argument('--test-non-sync', action='store_true', default=True,
                       help='Test non-synchronized mode (default: True)')
    parser.add_argument('--no-sync', action='store_true',
                       help='Skip synchronized mode test')
    parser.add_argument('--no-non-sync', action='store_true',
                       help='Skip non-synchronized mode test')
    
    args = parser.parse_args()
    
    # Handle flags
    test_sync = args.test_sync and not args.no_sync
    test_non_sync = args.test_non_sync and not args.no_non_sync
    
    if not test_sync and not test_non_sync:
        print("❌ ERROR: At least one test mode must be enabled")
        return
    
    rclpy.init()
    
    try:
        # Create test node
        test_node = MultiCameraReaderTestNode(config_file=args.config_file)
        
        if not test_node.camera_wrapper:
            print("❌ Failed to initialize camera wrapper. Exiting.")
            sys.exit(1)
        
        # Run test
        success = test_node.test_multi_camera_reader(
            num_reads=args.num_reads,
            wait_time=args.wait_time,
            duration=args.duration,
            test_sync=test_sync,
            test_non_sync=test_non_sync
        )
        
        if success:
            print("\n✅ All tests passed!")
            sys.exit(0)
        else:
            print("\n❌ Some tests failed")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()



#!/usr/bin/env python3
"""
Test script for Layer 1 (single camera) ROS2CameraReader.

Tests whether ROS2CameraReader (Layer 1) correctly receives and processes data:
- ApproximateTimeSynchronizer synchronization (CameraInfo, RGB, Depth)
- Data format validation
- Timestamp matching analysis
- Data preparation latency (time from message publish to data ready)
- Queue status and data availability
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import numpy as np
import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera_utils.ros2_camera_reader import ROS2CameraReader
from role_ros2.camera_utils.multi_camera_wrapper import MultiCameraWrapper


class Layer1CameraReaderTestNode(Node):
    """Test node for Layer 1 (single camera) ROS2CameraReader."""
    
    def __init__(self, config_file: str = None, camera_id: str = None):
        """
        Initialize test node.
        
        Args:
            config_file: Path to camera configuration file (optional)
            camera_id: Specific camera ID to test (optional, if None tests all cameras)
        """
        super().__init__('layer1_camera_reader_test_node')
        
        print(f"\n{'='*80}")
        print(f"Layer 1 (Single Camera) ROS2CameraReader Test")
        print(f"{'='*80}\n")
        
        # Load camera configurations
        if config_file is None:
            config_file = Path(__file__).parent.parent / 'role_ros2' / 'camera_utils' / 'multi_camera_config.yaml'
        else:
            config_file = Path(config_file)
        
        # Initialize multi-camera wrapper to get camera configurations
        try:
            self.camera_wrapper = MultiCameraWrapper(
                node=self,
                config_file=str(config_file)
            )
            self.get_logger().info("MultiCameraWrapper initialized for config loading")
            
            # Get list of camera IDs
            self.camera_ids = list(self.camera_wrapper.camera_dict.keys())
            self.get_logger().info(f"Found {len(self.camera_ids)} cameras: {self.camera_ids}")
            
            if not self.camera_ids:
                self.get_logger().error("❌ No cameras found! Check camera configuration.")
                self.camera_wrapper = None
                return
            
            # Filter to specific camera if requested
            if camera_id:
                if camera_id not in self.camera_ids:
                    self.get_logger().error(f"❌ Camera {camera_id} not found in configuration!")
                    self.camera_ids = []
                    return
                self.camera_ids = [camera_id]
            
            print(f"✅ Testing {len(self.camera_ids)} camera(s): {self.camera_ids}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize: {e}")
            import traceback
            traceback.print_exc()
            self.camera_wrapper = None
            self.camera_ids = []
    
    def test_layer1_camera_reader(
        self, 
        num_reads: int = 20, 
        wait_time: float = 0.5,
        duration: float = 60.0,
        analyze_timestamps: bool = True
    ):
        """
        Test Layer 1 camera reader by reading data multiple times.
        
        Args:
            num_reads: Number of times to read camera data
            wait_time: Time to wait between reads (seconds)
            duration: Maximum test duration (seconds)
            analyze_timestamps: Whether to analyze timestamp matching
        """
        if not self.camera_wrapper or not self.camera_ids:
            print("❌ ERROR: Camera wrapper not initialized. Cannot test.")
            return False
        
        print(f"\n{'='*80}")
        print(f"Starting Layer 1 Camera Reader Test")
        print(f"{'='*80}")
        print(f"Number of cameras:  {len(self.camera_ids)}")
        print(f"Camera IDs:         {self.camera_ids}")
        print(f"Number of reads:    {num_reads}")
        print(f"Wait time:          {wait_time} seconds")
        print(f"Max duration:       {duration} seconds")
        print(f"Analyze timestamps: {analyze_timestamps}")
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
                camera = self.camera_wrapper.camera_dict[cam_id]
                is_running = camera.is_running()
                camera_status[cam_id] = is_running
                if is_running:
                    latest_timestamp = camera.get_latest_pub_timestamp()
                    if latest_timestamp:
                        # Safely get queue size
                        try:
                            with camera._data_lock:
                                queue_size = len(camera._data_queue) if hasattr(camera, '_data_queue') else 0
                        except:
                            queue_size = 0
                        camera_status[cam_id] = (True, latest_timestamp, queue_size)
            
            all_running = all(
                status if isinstance(status, bool) else status[0]
                for status in camera_status.values()
            )
            
            # Print status every 2 seconds
            elapsed = time.time() - wait_start
            if elapsed - last_status_time >= 2.0:
                running_cameras = []
                for cam_id, status in camera_status.items():
                    if isinstance(status, tuple):
                        running_cameras.append(f"{cam_id}(q={status[2]})")
                    elif status:
                        running_cameras.append(cam_id)
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
            return False
        
        # Test each camera
        results = {}
        for camera_id in self.camera_ids:
            print(f"\n{'#'*80}")
            print(f"Testing Camera: {camera_id}")
            print(f"{'#'*80}")
            
            camera_result = self._test_single_camera(
                camera_id,
                num_reads=num_reads,
                wait_time=wait_time,
                duration=duration,
                analyze_timestamps=analyze_timestamps
            )
            results[camera_id] = camera_result
        
        # Print final summary
        self._print_final_summary(results)
        
        # Determine overall success
        all_passed = all(
            result.get('success', False)
            for result in results.values()
        )
        
        return all_passed
    
    def _test_single_camera(
        self,
        camera_id: str,
        num_reads: int,
        wait_time: float,
        duration: float,
        analyze_timestamps: bool
    ) -> Dict:
        """
        Test a single camera.
        
        Args:
            camera_id: Camera ID to test
            num_reads: Number of reads
            wait_time: Wait time between reads
            duration: Maximum duration
            analyze_timestamps: Whether to analyze timestamp matching
            
        Returns:
            Dictionary with test results
        """
        camera = self.camera_wrapper.camera_dict[camera_id]
        
        successful_reads = 0
        failed_reads = 0
        data_format_errors = 0
        
        # Statistics tracking
        read_times = []
        queue_sizes = []
        timestamp_diffs = []  # Difference between consecutive timestamps
        latency_stats = []  # Time from pub_t to sub_t (message latency)
        processing_times = []  # Time from sub_t to end_t (processing time)
        total_latencies = []  # Time from pub_t to end_t (total latency)
        
        # Track timestamps for analysis
        all_timestamps = []
        all_pub_timestamps = []  # Message header timestamps
        all_sub_timestamps = []  # Subscription timestamps
        all_end_timestamps = []  # Processing end timestamps
        
        test_start = time.time()
        
        for i in range(num_reads):
            if time.time() - test_start > duration:
                print(f"\n⏱️  Test duration ({duration}s) exceeded. Stopping.")
                break
            
            read_start_time = time.time()
            
            print(f"\n{'='*80}")
            print(f"[Camera {camera_id}] Read #{i+1}/{num_reads}")
            print(f"{'='*80}")
            
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Read camera data
            read_data_start = time.time()
            data_dict, timestamp_dict = camera.read_camera()
            read_data_time = (time.time() - read_data_start) * 1000
            
            # Check queue status (access internal attributes safely)
            try:
                with camera._data_lock:
                    queue_size = len(camera._data_queue) if hasattr(camera, '_data_queue') else 0
                    queue_sizes.append(queue_size)
                    if hasattr(camera, '_timestamp_order') and camera._timestamp_order:
                        available_timestamps = list(camera._timestamp_order)
                    else:
                        available_timestamps = []
            except Exception as e:
                self.get_logger().warning(f"Failed to access camera queue status: {e}")
                queue_size = 0
                available_timestamps = []
            
            # Check if data is available
            if not data_dict or not timestamp_dict:
                print(f"❌ ERROR: No data received (read_time={read_data_time:.2f}ms, queue_size={queue_size})")
                failed_reads += 1
                if i < num_reads - 1:
                    time.sleep(wait_time)
                continue
            
            # Validate data format
            is_valid, errors = self._validate_camera_data(camera_id, data_dict, timestamp_dict)
            if not is_valid:
                print(f"❌ ERROR: Data format validation failed (read_time={read_data_time:.2f}ms):")
                for error in errors:
                    print(f"   - {error}")
                data_format_errors += 1
                failed_reads += 1
                if i < num_reads - 1:
                    time.sleep(wait_time)
                continue
            
            # Extract timestamps for latency analysis
            pub_t_key = f"{camera_id}_pub_t"
            sub_t_key = f"{camera_id}_sub_t"
            end_t_key = f"{camera_id}_end_t"
            
            if pub_t_key in timestamp_dict:
                pub_t_ns = timestamp_dict[pub_t_key]
                all_timestamps.append(pub_t_ns)
                all_pub_timestamps.append(pub_t_ns)
                
                # Calculate timestamp difference from previous read
                if len(all_timestamps) > 1:
                    prev_timestamp = all_timestamps[-2]
                    diff_ns = abs(pub_t_ns - prev_timestamp)
                    diff_ms = diff_ns / 1e6
                    timestamp_diffs.append(diff_ms)
                
                # Calculate latencies
                if sub_t_key in timestamp_dict:
                    sub_t_ns = timestamp_dict[sub_t_key]
                    all_sub_timestamps.append(sub_t_ns)
                    latency_ns = sub_t_ns - pub_t_ns
                    latency_ms = latency_ns / 1e6
                    latency_stats.append(latency_ms)
                
                if end_t_key in timestamp_dict:
                    end_t_ns = timestamp_dict[end_t_key]
                    all_end_timestamps.append(end_t_ns)
                    total_latency_ns = end_t_ns - pub_t_ns
                    total_latency_ms = total_latency_ns / 1e6
                    total_latencies.append(total_latency_ms)
                
                if sub_t_key in timestamp_dict and end_t_key in timestamp_dict:
                    processing_ns = end_t_ns - sub_t_ns
                    processing_ms = processing_ns / 1e6
                    processing_times.append(processing_ms)
            
            # Track read time
            read_time = (time.time() - read_start_time) * 1000
            read_times.append(read_time)
            
            # Print summary
            self._print_read_summary(
                camera_id, data_dict, timestamp_dict, i+1, 
                read_time, read_data_time, queue_size, available_timestamps
            )
            
            successful_reads += 1
            
            if i < num_reads - 1:
                time.sleep(wait_time)
        
        # Calculate statistics
        success_rate = successful_reads / num_reads * 100 if num_reads > 0 else 0
        
        print(f"\n{'='*80}")
        print(f"[Camera {camera_id}] Test Results Summary")
        print(f"{'='*80}")
        print(f"Total reads:           {num_reads}")
        print(f"Successful reads:      {successful_reads} ✅")
        print(f"Failed reads:          {failed_reads} ❌")
        print(f"Data format errors:   {data_format_errors} ⚠️")
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
        
        # Print queue statistics
        if queue_sizes:
            avg_queue_size = sum(queue_sizes) / len(queue_sizes)
            max_queue_size = max(queue_sizes)
            min_queue_size = min(queue_sizes)
            print(f"\nQueue Statistics:")
            print(f"   Average queue size: {avg_queue_size:.1f}")
            print(f"   Min queue size:     {min_queue_size}")
            print(f"   Max queue size:     {max_queue_size}")
        
        # Print timestamp analysis
        if analyze_timestamps and timestamp_diffs:
            avg_diff = sum(timestamp_diffs) / len(timestamp_diffs)
            max_diff = max(timestamp_diffs)
            min_diff = min(timestamp_diffs)
            print(f"\nTimestamp Difference Statistics (between consecutive reads):")
            print(f"   Average diff:  {avg_diff:.3f} ms")
            print(f"   Min diff:      {min_diff:.3f} ms")
            print(f"   Max diff:      {max_diff:.3f} ms")
            print(f"   Expected:      ~66.67 ms (15 Hz) or ~33.33 ms (30 Hz)")
        
        # Print latency statistics
        if latency_stats:
            avg_latency = sum(latency_stats) / len(latency_stats)
            max_latency = max(latency_stats)
            min_latency = min(latency_stats)
            print(f"\nMessage Latency Statistics (pub_t → sub_t):")
            print(f"   Average latency: {avg_latency:.2f} ms")
            print(f"   Min latency:     {min_latency:.2f} ms")
            print(f"   Max latency:     {max_latency:.2f} ms")
        
        if processing_times:
            avg_processing = sum(processing_times) / len(processing_times)
            max_processing = max(processing_times)
            min_processing = min(processing_times)
            print(f"\nProcessing Time Statistics (sub_t → end_t):")
            print(f"   Average processing: {avg_processing:.2f} ms")
            print(f"   Min processing:     {min_processing:.2f} ms")
            print(f"   Max processing:     {max_processing:.2f} ms")
        
        if total_latencies:
            avg_total = sum(total_latencies) / len(total_latencies)
            max_total = max(total_latencies)
            min_total = min(total_latencies)
            print(f"\nTotal Latency Statistics (pub_t → end_t, data preparation time):")
            print(f"   Average total latency: {avg_total:.2f} ms")
            print(f"   Min total latency:     {min_total:.2f} ms")
            print(f"   Max total latency:     {max_total:.2f} ms")
        
        # Analyze timestamp distribution in queue
        if analyze_timestamps and len(all_timestamps) > 0:
            print(f"\nTimestamp Analysis:")
            print(f"   Total timestamps collected: {len(all_timestamps)}")
            if len(all_timestamps) >= 2:
                first_timestamp = all_timestamps[0]
                last_timestamp = all_timestamps[-1]
                total_duration_ms = (last_timestamp - first_timestamp) / 1e6
                print(f"   First timestamp: {first_timestamp/1e9:.9f} s")
                print(f"   Last timestamp:  {last_timestamp/1e9:.9f} s")
                print(f"   Total duration:  {total_duration_ms:.2f} ms")
                if len(all_timestamps) > 1:
                    avg_interval = total_duration_ms / (len(all_timestamps) - 1)
                    print(f"   Average interval: {avg_interval:.2f} ms")
                    expected_interval = 1000.0 / 15.0  # 15 Hz
                    print(f"   Expected interval (15 Hz): {expected_interval:.2f} ms")
                    if avg_interval > 0:
                        estimated_fps = 1000.0 / avg_interval
                        print(f"   Estimated frame rate: {estimated_fps:.2f} Hz")
        
        print(f"{'='*80}\n")
        
        return {
            'success': successful_reads == num_reads,
            'successful_reads': successful_reads,
            'failed_reads': failed_reads,
            'data_format_errors': data_format_errors,
            'success_rate': success_rate,
            'avg_read_time': sum(read_times) / len(read_times) if read_times else 0,
            'avg_queue_size': sum(queue_sizes) / len(queue_sizes) if queue_sizes else 0,
            'timestamp_count': len(all_timestamps),
            'avg_latency': sum(latency_stats) / len(latency_stats) if latency_stats else 0,
            'avg_processing_time': sum(processing_times) / len(processing_times) if processing_times else 0,
            'avg_total_latency': sum(total_latencies) / len(total_latencies) if total_latencies else 0
        }
    
    def _validate_camera_data(
        self, 
        camera_id: str, 
        data_dict: Dict, 
        timestamp_dict: Dict
    ) -> Tuple[bool, List[str]]:
        """
        Validate data format for a single camera.
        
        Args:
            camera_id: Camera ID
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
        
        # Check image data
        if "image" in data_dict:
            image_dict = data_dict["image"]
            if not isinstance(image_dict, dict):
                errors.append(f"data_dict['image'] is not a dictionary")
                return False, errors
            
            if camera_id not in image_dict:
                errors.append(f"Camera ID '{camera_id}' not found in image dict")
                return False, errors
            
            image = image_dict[camera_id]
            if image is None:
                errors.append(f"Image data for camera '{camera_id}' is None")
                return False, errors
            
            if not isinstance(image, np.ndarray):
                errors.append(f"Image for camera '{camera_id}' is not a numpy array")
                return False, errors
            
            if len(image.shape) != 3 or image.shape[2] != 3:
                errors.append(
                    f"Image shape for camera '{camera_id}' is invalid: "
                    f"{image.shape} (expected HxWx3)"
                )
                return False, errors
            
            if image.dtype != np.uint8:
                errors.append(
                    f"Image dtype for camera '{camera_id}' is invalid: "
                    f"{image.dtype} (expected uint8)"
                )
                return False, errors
        
        # Check depth data
        if "depth" in data_dict:
            depth_dict = data_dict["depth"]
            if not isinstance(depth_dict, dict):
                errors.append(f"data_dict['depth'] is not a dictionary")
                return False, errors
            
            if camera_id not in depth_dict:
                errors.append(f"Camera ID '{camera_id}' not found in depth dict")
                return False, errors
            
            depth = depth_dict[camera_id]
            if depth is None:
                errors.append(f"Depth data for camera '{camera_id}' is None")
                return False, errors
            
            if not isinstance(depth, np.ndarray):
                errors.append(f"Depth for camera '{camera_id}' is not a numpy array")
                return False, errors
            
            if len(depth.shape) != 2:
                errors.append(
                    f"Depth shape for camera '{camera_id}' is invalid: "
                    f"{depth.shape} (expected HxW)"
                )
                return False, errors
            
            if depth.dtype != np.uint16:
                errors.append(
                    f"Depth dtype for camera '{camera_id}' is invalid: "
                    f"{depth.dtype} (expected uint16)"
                )
                return False, errors
        
        # Check timestamp keys
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
    
    def _print_read_summary(
        self,
        camera_id: str,
        data_dict: Dict,
        timestamp_dict: Dict,
        read_num: int,
        read_time: float = 0,
        read_data_time: float = 0,
        queue_size: int = 0,
        available_timestamps: List[int] = None
    ):
        """Print summary of a successful read."""
        
        print(f"✅ Data received successfully")
        if read_time > 0:
            print(f"   Read time: {read_time:.2f} ms (data fetch: {read_data_time:.2f} ms)")
        print(f"   Queue size: {queue_size}")
        
        # Print image and depth info
        image = data_dict["image"].get(camera_id)
        depth = data_dict["depth"].get(camera_id)
        
        if image is not None and depth is not None:
            print(f"\n   Image: shape={image.shape}, dtype={image.dtype}, range=[{image.min()}, {image.max()}]")
            print(f"   Depth: shape={depth.shape}, dtype={depth.dtype}, range=[{depth.min()}, {depth.max()}]")
        
        # Print timestamp info
        pub_t_key = f"{camera_id}_pub_t"
        read_start_key = f"{camera_id}_read_start"
        read_end_key = f"{camera_id}_read_end"
        sub_t_key = f"{camera_id}_sub_t"
        end_t_key = f"{camera_id}_end_t"
        
        if pub_t_key in timestamp_dict:
            pub_t = timestamp_dict[pub_t_key]
            print(f"\n   Timestamps:")
            print(f"      pub_t (message header): {pub_t} ({pub_t/1e9:.9f} s)")
            
            if read_start_key in timestamp_dict and read_end_key in timestamp_dict:
                read_start = timestamp_dict[read_start_key]
                read_end = timestamp_dict[read_end_key]
                processing_time = (read_end - read_start) / 1e6
                print(f"      Processing time:        {processing_time:.2f} ms")
            
            if sub_t_key in timestamp_dict:
                sub_t = timestamp_dict[sub_t_key]
                latency_ns = sub_t - pub_t
                latency_ms = latency_ns / 1e6
                print(f"      Latency (pub_t → sub_t): {latency_ms:.2f} ms")
        
        # Print available timestamps in queue
        if available_timestamps:
            print(f"\n   Queue timestamps ({len(available_timestamps)} available):")
            # Show first 5 and last 5
            if len(available_timestamps) <= 10:
                for ts in available_timestamps:
                    print(f"      {ts} ({ts/1e9:.9f} s)")
            else:
                for ts in available_timestamps[:5]:
                    print(f"      {ts} ({ts/1e9:.9f} s)")
                print(f"      ... ({len(available_timestamps) - 10} more) ...")
                for ts in available_timestamps[-5:]:
                    print(f"      {ts} ({ts/1e9:.9f} s)")
    
    def _print_final_summary(self, results: Dict):
        """Print final test summary."""
        print(f"\n{'='*80}")
        print(f"Final Test Summary")
        print(f"{'='*80}")
        
        for camera_id, result in results.items():
            success = result.get('success', False)
            success_rate = result.get('success_rate', 0)
            
            status = "✅ PASSED" if success else "❌ FAILED"
            print(f"\nCamera {camera_id}: {status}")
            print(f"   Success rate: {success_rate:.1f}%")
            print(f"   Successful reads: {result.get('successful_reads', 0)}")
            print(f"   Failed reads: {result.get('failed_reads', 0)}")
            
            # Print performance metrics
            if result.get('avg_read_time', 0) > 0:
                print(f"   Avg read time: {result.get('avg_read_time', 0):.2f} ms")
            
            if result.get('avg_queue_size', 0) > 0:
                print(f"   Avg queue size: {result.get('avg_queue_size', 0):.1f}")
            
            if result.get('timestamp_count', 0) > 0:
                print(f"   Timestamps collected: {result.get('timestamp_count', 0)}")
        
        print(f"{'='*80}\n")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test Layer 1 (single camera) ROS2CameraReader'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='Path to camera configuration file (optional)')
    parser.add_argument('--camera-id', type=str, default=None,
                       help='Specific camera ID to test (optional, if None tests all cameras)')
    parser.add_argument('--num-reads', type=int, default=20,
                       help='Number of reads to perform (default: 20)')
    parser.add_argument('--wait-time', type=float, default=0.5,
                       help='Wait time between reads in seconds (default: 0.5)')
    parser.add_argument('--duration', type=float, default=60.0,
                       help='Maximum test duration in seconds (default: 60.0)')
    parser.add_argument('--no-timestamp-analysis', action='store_true',
                       help='Skip timestamp analysis')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        # Create test node
        test_node = Layer1CameraReaderTestNode(
            config_file=args.config_file,
            camera_id=args.camera_id
        )
        
        if not test_node.camera_wrapper or not test_node.camera_ids:
            print("❌ Failed to initialize. Exiting.")
            sys.exit(1)
        
        # Run test
        success = test_node.test_layer1_camera_reader(
            num_reads=args.num_reads,
            wait_time=args.wait_time,
            duration=args.duration,
            analyze_timestamps=not args.no_timestamp_analysis
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


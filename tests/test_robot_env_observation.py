#!/usr/bin/env python3
"""
Test script for RobotEnv.get_observation().

Tests whether image data and robot state are received correctly,
and calculates publish latency.
"""

import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node

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

from role_ros2.robot_env import RobotEnv
from role_ros2.camera.multi_camera_wrapper import MultiCameraWrapper


class ObservationTestNode(Node):
    """Test node for RobotEnv observation testing."""
    
    def __init__(self):
        super().__init__('observation_test_node')
        self.get_logger().info("Initializing observation test node...")
        
        # Initialize camera reader
        try:
            self.camera_reader = MultiCameraWrapper(node=self)
            self.get_logger().info("✅ Camera reader initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize camera reader: {e}")
            self.camera_reader = None
        
        # Initialize robot environment
        try:
            self.robot_env = RobotEnv(
                action_space="cartesian_velocity",
                node=self,
                camera_reader=self.camera_reader,
                do_reset=False  # Skip reset for testing
            )
            self.get_logger().info("✅ RobotEnv initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize RobotEnv: {e}")
            self.robot_env = None
    
        # Flag to print observation dict only once
        self.obs_dict_printed = False
        
        # Statistics for latency analysis
        self.robot_latencies = {
            "pub_to_sub": [],  # robot_sub_t - robot_pub_t
            "sub_to_end": [],  # robot_end_t - robot_sub_t
            "pub_to_end": []  # robot_end_t - robot_pub_t
        }
        self.camera_robot_latencies = {}  # {camera_id: [camera_pub_t - robot_pub_t]}
        
        # Statistics for packet loss analysis
        self.read_cameras_stats = {
            "total_calls": 0,
            "successful_calls": 0,
            "failed_calls": 0,
            "missing_images": 0,
            "missing_depths": 0,
            "missing_intrinsics": 0,
            "missing_extrinsics": 0
        }
        self.get_state_stats = {
            "total_calls": 0,
            "successful_calls": 0,
            "failed_calls": 0,
            "missing_robot_state": 0
        }
        
        # Statistics for timestamp difference analysis
        self.camera_timestamp_diffs = {
            # Per-camera timestamp differences
            "per_camera": {},  # {camera_id: {"pub_to_sub": [], "sub_to_end": [], "pub_to_end": []}}
            # Multi-camera timestamp differences
            "multi_camera": []  # [{"camera_pair": (id1, id2), "diff_ms": float, ...}]
        }
        self.robot_timestamp_diffs = {
            "pub_to_sub": [],  # robot_sub_t - robot_pub_t
            "sub_to_end": [],  # robot_end_t - robot_sub_t
            "pub_to_end": []  # robot_end_t - robot_pub_t
        }
        
        # Raw timestamp data for visualization (recorded per iteration)
        self.robot_timestamps_raw: List[Dict] = []  # List of timestamp dicts per iteration
        self.camera_timestamps_raw: List[Dict] = []  # List of timestamp dicts per iteration
        self.test_type: Optional[str] = None  # Track which test is running
    
    def _print_obs_dict(self, obs: Dict[str, Any], indent: int = 0):
        """
        Recursively print observation dictionary.
        If value is numpy array, print its shape instead of content.
        
        Args:
            obs: Observation dictionary
            indent: Current indentation level
        """
        prefix = "  " * indent
        for key, value in obs.items():
            if isinstance(value, np.ndarray):
                print(f"{prefix}{key}: numpy.ndarray, shape={value.shape}, dtype={value.dtype}")
            elif isinstance(value, dict):
                print(f"{prefix}{key}:")
                self._print_obs_dict(value, indent + 1)
            elif isinstance(value, (list, tuple)) and len(value) > 0 and isinstance(value[0], (int, float)):
                # Print list/tuple summary if it's numeric
                if len(value) > 5:
                    print(f"{prefix}{key}: {type(value).__name__}[{len(value)}] = [{value[0]:.3f}, {value[1]:.3f}, ..., {value[-1]:.3f}]")
                else:
                    print(f"{prefix}{key}: {value}")
            else:
                print(f"{prefix}{key}: {value}")
    
    def test_get_observation(self, duration_sec=10.0, fps=20.0):
        """
        Test get_observation() and calculate latencies at specified frequency.
        
        Args:
            duration_sec: Test duration in seconds (default: 10.0)
            fps: Frequency of get_observation() calls in Hz (default: 20.0)
        """
        if self.robot_env is None:
            self.get_logger().error("RobotEnv not initialized. Cannot run test.")
            return
        
        # Clear previous timestamp data
        self.robot_timestamps_raw = []
        self.camera_timestamps_raw = []
        
        print("\n" + "=" * 80)
        print(f"Starting observation test")
        print(f"  Duration: {duration_sec} seconds")
        print(f"  Frequency: {fps} Hz (interval: {1.0/fps*1000:.2f} ms)")
        print("=" * 80 + "\n")
        
        # Calculate interval between calls (in seconds)
        interval_sec = 1.0 / fps
        
        # Start timing
        start_time = time.time()
        iteration = 0
        next_call_time = start_time
        last_call_time = start_time
        
        while time.time() - start_time < duration_sec:
            iteration += 1
            call_start_time = time.time()
            
            # No need to spin - RobotEnv's MultiThreadedExecutor handles callbacks in background
            
            # Calculate actual FPS based on time since last call
            if iteration > 1:
                actual_interval = call_start_time - last_call_time
                actual_fps = 1.0 / actual_interval if actual_interval > 0 else 0
            else:
                actual_fps = fps
            
            elapsed_time = call_start_time - start_time
            print(f"--- Iteration {iteration} (elapsed: {elapsed_time:.2f}s, target FPS: {fps:.1f}, actual FPS: {actual_fps:.1f}) ---")
            
            try:
                obs = self.robot_env.get_observation()
                
                # Print observation dict structure once
                if not self.obs_dict_printed:
                    print("\n" + "=" * 80)
                    print("Observation Dictionary Structure (printed once)")
                    print("=" * 80)
                    self._print_obs_dict(obs)
                    print("=" * 80 + "\n")
                    self.obs_dict_printed = True
                
                # Analyze robot timestamp latencies
                    robot_timestamp = obs.get("timestamp", {}).get("robot_state", {})
                if robot_timestamp:
                    # Store raw timestamp data for visualization
                    self.robot_timestamps_raw.append(robot_timestamp.copy())
                    
                    if "robot_pub_t" in robot_timestamp and "robot_sub_t" in robot_timestamp and "robot_end_t" in robot_timestamp:
                        robot_pub_t = robot_timestamp["robot_pub_t"]
                        robot_sub_t = robot_timestamp["robot_sub_t"]
                        robot_end_t = robot_timestamp["robot_end_t"]
                        
                        # Calculate latencies (all in nanoseconds, convert to ms)
                        pub_to_sub_ms = (robot_sub_t - robot_pub_t) / 1e6
                        sub_to_end_ms = (robot_end_t - robot_sub_t) / 1e6
                        pub_to_end_ms = (robot_end_t - robot_pub_t) / 1e6
                        
                        self.robot_latencies["pub_to_sub"].append(pub_to_sub_ms)
                        self.robot_latencies["sub_to_end"].append(sub_to_end_ms)
                        self.robot_latencies["pub_to_end"].append(pub_to_end_ms)
                
                # Analyze camera-robot timestamp latencies
                camera_timestamp = obs.get("timestamp", {}).get("cameras", {})
                if camera_timestamp:
                    # Store raw timestamp data for visualization
                    self.camera_timestamps_raw.append(camera_timestamp.copy())
                
                robot_timestamp = obs.get("timestamp", {}).get("robot_state", {})
                
                if "robot_pub_t" in robot_timestamp:
                    robot_pub_t = robot_timestamp["robot_pub_t"]
                    
                    # Find all camera pub_t timestamps
                    for key, value in camera_timestamp.items():
                        if key.endswith("_pub_t") and isinstance(value, (int, float)):
                            camera_id = key.replace("_pub_t", "")
                            camera_pub_t = value
                            
                            # Calculate latency: camera_pub_t - robot_pub_t
                            latency_ns = camera_pub_t - robot_pub_t
                            latency_ms = latency_ns / 1e6
                            
                            if camera_id not in self.camera_robot_latencies:
                                self.camera_robot_latencies[camera_id] = []
                            self.camera_robot_latencies[camera_id].append(latency_ms)
                
            except Exception as e:
                self.get_logger().error(f"❌ Error getting observation: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
            
            # Record call end time
            call_end_time = time.time()
            last_call_time = call_start_time
            
            # Calculate next call time and sleep to maintain target FPS
            next_call_time += interval_sec
            sleep_time = next_call_time - call_end_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.1:  # If we're more than 100ms behind, log warning
                self.get_logger().warn(
                    f"⚠️  Falling behind schedule: {abs(sleep_time)*1000:.2f} ms behind target interval"
                )
        
        # Print latency analysis
        print(f"\n✅ Test completed: {iteration} iterations in {time.time() - start_time:.2f} seconds")
        print(f"   Average FPS: {iteration / (time.time() - start_time):.2f} Hz")
        self._print_latency_analysis()
        
        # Generate visualizations
        if MATPLOTLIB_AVAILABLE:
            self.test_type = 'observation'
            self._plot_timestamp_visualization()
    
    def _print_latency_analysis(self):
        """Print latency analysis results."""
        print("\n" + "=" * 80)
        print("LATENCY ANALYSIS")
        print("=" * 80 + "\n")
        
        # Robot timestamp latencies
        print("1. Robot Timestamp Latencies (robot_pub_t, robot_sub_t, robot_end_t):")
        print("   " + "-" * 76)
        
        if self.robot_latencies["pub_to_sub"]:
            pub_to_sub = self.robot_latencies["pub_to_sub"]
            print(f"   robot_sub_t - robot_pub_t:")
            print(f"     Mean:   {np.mean(pub_to_sub):.3f} ms")
            print(f"     Std:    {np.std(pub_to_sub):.3f} ms")
            print(f"     Min:    {np.min(pub_to_sub):.3f} ms")
            print(f"     Max:    {np.max(pub_to_sub):.3f} ms")
            print(f"     Samples: {len(pub_to_sub)}")
        
        if self.robot_latencies["sub_to_end"]:
            sub_to_end = self.robot_latencies["sub_to_end"]
            print(f"\n   robot_end_t - robot_sub_t:")
            print(f"     Mean:   {np.mean(sub_to_end):.3f} ms")
            print(f"     Std:    {np.std(sub_to_end):.3f} ms")
            print(f"     Min:    {np.min(sub_to_end):.3f} ms")
            print(f"     Max:    {np.max(sub_to_end):.3f} ms")
            print(f"     Samples: {len(sub_to_end)}")
        
        if self.robot_latencies["pub_to_end"]:
            pub_to_end = self.robot_latencies["pub_to_end"]
            print(f"\n   robot_end_t - robot_pub_t (total):")
            print(f"     Mean:   {np.mean(pub_to_end):.3f} ms")
            print(f"     Std:    {np.std(pub_to_end):.3f} ms")
            print(f"     Min:    {np.min(pub_to_end):.3f} ms")
            print(f"     Max:    {np.max(pub_to_end):.3f} ms")
            print(f"     Samples: {len(pub_to_end)}")
        
        print()
        
        # Camera-robot timestamp latencies
        print("2. Camera-Robot Timestamp Latencies ({camera_id}_pub_t - robot_pub_t):")
        print("   " + "-" * 76)
        
        if self.camera_robot_latencies:
            for camera_id, latencies in sorted(self.camera_robot_latencies.items()):
                print(f"\n   Camera {camera_id}:")
                print(f"     Mean:   {np.mean(latencies):.3f} ms")
                print(f"     Std:    {np.std(latencies):.3f} ms")
                print(f"     Min:    {np.min(latencies):.3f} ms")
                print(f"     Max:    {np.max(latencies):.3f} ms")
                print(f"     Samples: {len(latencies)}")
                print(f"     Note: Positive values mean camera timestamp is later than robot timestamp")
                else:
            print("   No camera-robot latency data collected")
        
        print("\n" + "=" * 80 + "\n")
    
    def test_read_cameras(self, duration_sec=10.0, fps=20.0):
        """
        Test read_cameras() separately and calculate packet loss rate.
        
        Args:
            duration_sec: Test duration in seconds (default: 10.0)
            fps: Frequency of read_cameras() calls in Hz (default: 20.0)
        """
        if self.robot_env is None or self.robot_env.camera_reader is None:
            self.get_logger().error("RobotEnv or camera_reader not initialized. Cannot run test.")
            return
        
        # Clear previous timestamp data
        self.camera_timestamps_raw = []
        
        print("\n" + "=" * 80)
        print("Testing read_cameras() separately")
        print(f"  Duration: {duration_sec} seconds")
        print(f"  Frequency: {fps} Hz (interval: {1.0/fps*1000:.2f} ms)")
        print("=" * 80 + "\n")
        
        # Calculate interval between calls (in seconds)
        interval_sec = 1.0 / fps
        
        # Get camera IDs for validation
        camera_ids = list(self.robot_env.camera_reader.camera_dict.keys()) if self.robot_env.camera_reader else []
        print(f"Testing with {len(camera_ids)} cameras: {camera_ids}\n")
        
        # Wait for cameras to start receiving data
        print("Waiting for cameras to start receiving data...")
        timeout = 10.0
        start_wait = time.time()
        while time.time() - start_wait < timeout:
            all_running = all(
                self.robot_env.camera_reader.camera_dict[cam_id].is_running()
                for cam_id in camera_ids
            ) if camera_ids else False
            if all_running:
                break
            # No need to spin - RobotEnv's MultiThreadedExecutor handles callbacks in background
            time.sleep(0.1)  # Just wait a bit
            if not rclpy.ok():
                return
        
        if not all_running and camera_ids:
            print("⚠️  Some cameras are not running, but continuing with tests...\n")
        elif camera_ids:
            print("✅ All cameras are running\n")
        
        # Start timing
        start_time = time.time()
        iteration = 0
        next_call_time = start_time
        last_call_time = start_time
        
        while time.time() - start_time < duration_sec:
            iteration += 1
            call_start_time = time.time()
            
            # No need to spin - RobotEnv's MultiThreadedExecutor handles callbacks in background
            
            # Calculate actual FPS
            if iteration > 1:
                actual_interval = call_start_time - last_call_time
                actual_fps = 1.0 / actual_interval if actual_interval > 0 else 0
            else:
                actual_fps = fps
            
            elapsed_time = call_start_time - start_time
            if iteration % 20 == 0 or iteration == 1:  # Print every 20 iterations
                print(f"--- Iteration {iteration} (elapsed: {elapsed_time:.2f}s, target FPS: {fps:.1f}, actual FPS: {actual_fps:.1f}) ---")
            
            try:
                camera_obs, camera_timestamp = self.robot_env.read_cameras(use_sync=False)
                self.read_cameras_stats["total_calls"] += 1
                
                # Validate data completeness
                success = True
                image_dict = camera_obs.get("image", {})
                depth_dict = camera_obs.get("depth", {})
                intrinsics = camera_obs.get("camera_intrinsics")
                extrinsics = camera_obs.get("camera_extrinsics")
                
                # Check if data is from sync mode
                is_sync_data = "multi_camera_sync_start" in camera_timestamp
                if is_sync_data:
                    self.read_cameras_stats["sync_data_count"] = self.read_cameras_stats.get("sync_data_count", 0) + 1
                else:
                    self.read_cameras_stats["non_sync_data_count"] = self.read_cameras_stats.get("non_sync_data_count", 0) + 1
                
                # Debug: Print data availability on first few iterations
                if iteration <= 3:
                    print(f"    Debug - camera_obs keys: {list(camera_obs.keys())}")
                    print(f"    Debug - image_dict keys: {list(image_dict.keys())}")
                    print(f"    Debug - depth_dict keys: {list(depth_dict.keys())}")
                    print(f"    Debug - intrinsics: {intrinsics is not None and len(intrinsics) > 0 if intrinsics else False}")
                    print(f"    Debug - extrinsics: {extrinsics is not None and len(extrinsics) > 0 if extrinsics else False}")
                    print(f"    Debug - is_sync_data: {is_sync_data}")
                
                # Check images
                for camera_id in camera_ids:
                    if camera_id not in image_dict:
                        self.read_cameras_stats["missing_images"] += 1
                        success = False
                
                # Check depths
                for camera_id in camera_ids:
                    if camera_id not in depth_dict:
                        self.read_cameras_stats["missing_depths"] += 1
                        success = False
                
                # Check intrinsics
                if intrinsics is None or len(intrinsics) == 0:
                    self.read_cameras_stats["missing_intrinsics"] += 1
                    success = False
                
                # Check extrinsics (may be None, which is acceptable)
                if extrinsics is None:
                    self.read_cameras_stats["missing_extrinsics"] += 1
                    # Don't count as failure for extrinsics (TF lookup may fail)
                
                if success:
                    self.read_cameras_stats["successful_calls"] += 1
                else:
                    self.read_cameras_stats["failed_calls"] += 1
                
                # Store raw timestamp data for visualization
                self.camera_timestamps_raw.append(camera_timestamp.copy())
                
                # Analyze timestamp differences for cameras
                self._analyze_camera_timestamp_diffs(camera_timestamp, camera_ids)
                
            except Exception as e:
                self.read_cameras_stats["total_calls"] += 1
                self.read_cameras_stats["failed_calls"] += 1
                self.get_logger().error(f"❌ Error in read_cameras: {e}")
                if iteration <= 5:  # Only print first few errors
                import traceback
                self.get_logger().error(traceback.format_exc())
            
            # Record call end time
            call_end_time = time.time()
            last_call_time = call_start_time
            
            # Calculate next call time and sleep to maintain target FPS
            next_call_time += interval_sec
            sleep_time = next_call_time - call_end_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.1:
                self.get_logger().warn(
                    f"⚠️  Falling behind schedule: {abs(sleep_time)*1000:.2f} ms behind target interval"
                )
        
        # Print packet loss analysis
        print(f"\n✅ Test completed: {iteration} iterations in {time.time() - start_time:.2f} seconds")
        print(f"   Average FPS: {iteration / (time.time() - start_time):.2f} Hz")
        self._print_read_cameras_packet_loss()
        self._print_camera_timestamp_diff_analysis()
        
        # Generate visualizations
        if MATPLOTLIB_AVAILABLE:
            self.test_type = 'read_cameras'
            self._plot_timestamp_visualization()
    
    def test_get_state(self, duration_sec=10.0, fps=20.0):
        """
        Test get_state() separately and calculate packet loss rate.
        
        Args:
            duration_sec: Test duration in seconds (default: 10.0)
            fps: Frequency of get_state() calls in Hz (default: 20.0)
        """
        if self.robot_env is None:
            self.get_logger().error("RobotEnv not initialized. Cannot run test.")
            return
        
        print("\n" + "=" * 80)
        print("Testing get_state() separately")
        print(f"  Duration: {duration_sec} seconds")
        print(f"  Frequency: {fps} Hz (interval: {1.0/fps*1000:.2f} ms)")
        print("=" * 80 + "\n")
        
        # Clear previous timestamp data
        self.robot_timestamps_raw = []
        
        # Calculate interval between calls (in seconds)
        interval_sec = 1.0 / fps
        
        # Start timing
        start_time = time.time()
        iteration = 0
        next_call_time = start_time
        last_call_time = start_time
        
        while time.time() - start_time < duration_sec:
            iteration += 1
            call_start_time = time.time()
            
            # No need to spin - RobotEnv's MultiThreadedExecutor handles callbacks in background
            # Messages arrive at 50Hz and are processed continuously by the executor
            
            # Calculate actual FPS
            if iteration > 1:
                actual_interval = call_start_time - last_call_time
                actual_fps = 1.0 / actual_interval if actual_interval > 0 else 0
            else:
                actual_fps = fps
            
            elapsed_time = call_start_time - start_time
            if iteration % 20 == 0 or iteration == 1:  # Print every 20 iterations
                print(f"--- Iteration {iteration} (elapsed: {elapsed_time:.2f}s, target FPS: {fps:.1f}, actual FPS: {actual_fps:.1f}) ---")
            
            try:
                state_dict, timestamp_dict = self.robot_env.get_state()
                self.get_state_stats["total_calls"] += 1
                
                # Validate data completeness
                success = True
                if not state_dict or len(state_dict) == 0:
                    self.get_state_stats["missing_robot_state"] += 1
                    success = False
                else:
                    # Check required fields
                    required_fields = ["cartesian_position", "joint_positions", "gripper_position"]
                    for field in required_fields:
                        if field not in state_dict:
                            self.get_state_stats["missing_robot_state"] += 1
                            success = False
                            break
                
                if success:
                    self.get_state_stats["successful_calls"] += 1
                else:
                    self.get_state_stats["failed_calls"] += 1
                
                # Store raw timestamp data for visualization
                self.robot_timestamps_raw.append(timestamp_dict.copy())
                
                # Analyze timestamp differences for robot state
                self._analyze_robot_timestamp_diffs(timestamp_dict)
                
            except Exception as e:
                self.get_state_stats["total_calls"] += 1
                self.get_state_stats["failed_calls"] += 1
                self.get_logger().error(f"❌ Error in get_state: {e}")
                if iteration <= 5:  # Only print first few errors
                    import traceback
                    self.get_logger().error(traceback.format_exc())
            
            # Record call end time
            call_end_time = time.time()
            last_call_time = call_start_time
            
            # Calculate next call time and sleep to maintain target FPS
            next_call_time += interval_sec
            sleep_time = next_call_time - call_end_time
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -0.1:
                self.get_logger().warn(
                    f"⚠️  Falling behind schedule: {abs(sleep_time)*1000:.2f} ms behind target interval"
                )
        
        # Print packet loss analysis
        print(f"\n✅ Test completed: {iteration} iterations in {time.time() - start_time:.2f} seconds")
        print(f"   Average FPS: {iteration / (time.time() - start_time):.2f} Hz")
        self._print_get_state_packet_loss()
        self._print_robot_timestamp_diff_analysis()
        
        # Generate visualizations
        if MATPLOTLIB_AVAILABLE:
            self.test_type = 'get_state'
            self._plot_timestamp_visualization()
    
    def _print_read_cameras_packet_loss(self):
        """Print packet loss analysis for read_cameras()."""
        print("\n" + "=" * 80)
        print("read_cameras() PACKET LOSS ANALYSIS")
        print("=" * 80 + "\n")
        
        stats = self.read_cameras_stats
        total = stats["total_calls"]
        successful = stats["successful_calls"]
        failed = stats["failed_calls"]
        
        if total > 0:
            success_rate = (successful / total) * 100
            packet_loss_rate = (failed / total) * 100
            
            print(f"Total calls: {total}")
            print(f"Successful calls: {successful}")
            print(f"Failed calls: {failed}")
            print(f"Success rate: {success_rate:.2f}%")
            print(f"Packet loss rate: {packet_loss_rate:.2f}%")
            print()
            
            print("Failure breakdown:")
            print(f"  Missing images: {stats['missing_images']} times")
            print(f"  Missing depths: {stats['missing_depths']} times")
            print(f"  Missing intrinsics: {stats['missing_intrinsics']} times")
            print(f"  Missing extrinsics: {stats['missing_extrinsics']} times (may be normal if TF lookup fails)")
            print()
            
            # Print sync data statistics
            sync_count = stats.get("sync_data_count", 0)
            non_sync_count = stats.get("non_sync_data_count", 0)
            print("Data source breakdown:")
            print(f"  Sync mode data: {sync_count} times")
            print(f"  Non-sync mode data: {non_sync_count} times")
            if sync_count + non_sync_count > 0:
                sync_rate = (sync_count / (sync_count + non_sync_count)) * 100
                print(f"  Sync data rate: {sync_rate:.2f}%")
        else:
            print("No data collected")
        
        print("\n" + "=" * 80 + "\n")
    
    def _print_get_state_packet_loss(self):
        """Print packet loss analysis for get_state()."""
        print("\n" + "=" * 80)
        print("get_state() PACKET LOSS ANALYSIS")
        print("=" * 80 + "\n")
        
        stats = self.get_state_stats
        total = stats["total_calls"]
        successful = stats["successful_calls"]
        failed = stats["failed_calls"]
        
        if total > 0:
            success_rate = (successful / total) * 100
            packet_loss_rate = (failed / total) * 100
            
            print(f"Total calls: {total}")
            print(f"Successful calls: {successful}")
            print(f"Failed calls: {failed}")
            print(f"Success rate: {success_rate:.2f}%")
            print(f"Packet loss rate: {packet_loss_rate:.2f}%")
            print()
            
            print("Failure breakdown:")
            print(f"  Missing robot state: {stats['missing_robot_state']} times")
        else:
            print("No data collected")
        
        print("\n" + "=" * 80 + "\n")
    
    def _analyze_camera_timestamp_diffs(self, camera_timestamp: Dict, camera_ids: list):
        """
        Analyze timestamp differences for camera data.
        
        Args:
            camera_timestamp: Camera timestamp dictionary
            camera_ids: List of camera IDs
        """
        # Analyze per-camera timestamp differences
        for camera_id in camera_ids:
            if camera_id not in self.camera_timestamp_diffs["per_camera"]:
                self.camera_timestamp_diffs["per_camera"][camera_id] = {
                    "pub_to_sub": [],
                    "sub_to_end": [],
                    "pub_to_end": []
                }
            
            pub_t_key = f"{camera_id}_pub_t"
            sub_t_key = f"{camera_id}_sub_t"
            end_t_key = f"{camera_id}_end_t"
            
            if pub_t_key in camera_timestamp and sub_t_key in camera_timestamp:
                pub_t = camera_timestamp[pub_t_key]
                sub_t = camera_timestamp[sub_t_key]
                pub_to_sub_ms = (sub_t - pub_t) / 1e6
                self.camera_timestamp_diffs["per_camera"][camera_id]["pub_to_sub"].append(pub_to_sub_ms)
            
            if sub_t_key in camera_timestamp and end_t_key in camera_timestamp:
                sub_t = camera_timestamp[sub_t_key]
                end_t = camera_timestamp[end_t_key]
                sub_to_end_ms = (end_t - sub_t) / 1e6
                self.camera_timestamp_diffs["per_camera"][camera_id]["sub_to_end"].append(sub_to_end_ms)
            
            if pub_t_key in camera_timestamp and end_t_key in camera_timestamp:
                pub_t = camera_timestamp[pub_t_key]
                end_t = camera_timestamp[end_t_key]
                pub_to_end_ms = (end_t - pub_t) / 1e6
                self.camera_timestamp_diffs["per_camera"][camera_id]["pub_to_end"].append(pub_to_end_ms)
        
        # Analyze multi-camera timestamp differences (pub_t differences between cameras)
        if len(camera_ids) >= 2:
            pub_timestamps = {}
            for camera_id in camera_ids:
                pub_t_key = f"{camera_id}_pub_t"
                if pub_t_key in camera_timestamp:
                    pub_timestamps[camera_id] = camera_timestamp[pub_t_key]
            
            # Check if using synchronized data (contains multi_camera_sync_start)
            is_sync_data = "multi_camera_sync_start" in camera_timestamp
            
            # Calculate differences between all pairs of cameras
            camera_ids_list = list(pub_timestamps.keys())
            for i in range(len(camera_ids_list)):
                for j in range(i + 1, len(camera_ids_list)):
                    cam_id_1 = camera_ids_list[i]
                    cam_id_2 = camera_ids_list[j]
                    
                    pub_t_1 = pub_timestamps[cam_id_1]
                    pub_t_2 = pub_timestamps[cam_id_2]
                    
                    # Use absolute value for consistency with test_multi_camera_wrapper_functions.py
                    diff_ns = abs(pub_t_1 - pub_t_2)
                    diff_ms = diff_ns / 1e6
                    
                    self.camera_timestamp_diffs["multi_camera"].append({
                        "camera_pair": (cam_id_1, cam_id_2),
                        "diff_ms": diff_ms,
                        "pub_t_1": pub_t_1,
                        "pub_t_2": pub_t_2,
                        "is_sync_data": is_sync_data
                    })
    
    def _analyze_robot_timestamp_diffs(self, timestamp_dict: Dict):
        """
        Analyze timestamp differences for robot state.
        
        Args:
            timestamp_dict: Robot state timestamp dictionary
        """
        if "robot_pub_t" in timestamp_dict and "robot_sub_t" in timestamp_dict:
            robot_pub_t = timestamp_dict["robot_pub_t"]
            robot_sub_t = timestamp_dict["robot_sub_t"]
            pub_to_sub_ms = (robot_sub_t - robot_pub_t) / 1e6
            self.robot_timestamp_diffs["pub_to_sub"].append(pub_to_sub_ms)
        
        if "robot_sub_t" in timestamp_dict and "robot_end_t" in timestamp_dict:
            robot_sub_t = timestamp_dict["robot_sub_t"]
            robot_end_t = timestamp_dict["robot_end_t"]
            sub_to_end_ms = (robot_end_t - robot_sub_t) / 1e6
            self.robot_timestamp_diffs["sub_to_end"].append(sub_to_end_ms)
        
        if "robot_pub_t" in timestamp_dict and "robot_end_t" in timestamp_dict:
            robot_pub_t = timestamp_dict["robot_pub_t"]
            robot_end_t = timestamp_dict["robot_end_t"]
            pub_to_end_ms = (robot_end_t - robot_pub_t) / 1e6
            self.robot_timestamp_diffs["pub_to_end"].append(pub_to_end_ms)
    
    def _print_camera_timestamp_diff_analysis(self):
        """Print timestamp difference analysis for cameras."""
        print("\n" + "=" * 80)
        print("read_cameras() TIMESTAMP DIFFERENCE ANALYSIS")
        print("=" * 80 + "\n")
        
        # Per-camera timestamp differences
        print("1. Per-Camera Timestamp Differences:")
        print("   " + "-" * 76)
        
        per_camera = self.camera_timestamp_diffs["per_camera"]
        if per_camera:
            for camera_id in sorted(per_camera.keys()):
                diffs = per_camera[camera_id]
                print(f"\n   Camera {camera_id}:")
                
                if diffs["pub_to_sub"]:
                    pub_to_sub = diffs["pub_to_sub"]
                    print(f"     {camera_id}_sub_t - {camera_id}_pub_t:")
                    print(f"       Mean:   {np.mean(pub_to_sub):.3f} ms")
                    print(f"       Std:    {np.std(pub_to_sub):.3f} ms")
                    print(f"       Min:    {np.min(pub_to_sub):.3f} ms")
                    print(f"       Max:    {np.max(pub_to_sub):.3f} ms")
                    print(f"       Samples: {len(pub_to_sub)}")
                
                if diffs["sub_to_end"]:
                    sub_to_end = diffs["sub_to_end"]
                    print(f"\n     {camera_id}_end_t - {camera_id}_sub_t:")
                    print(f"       Mean:   {np.mean(sub_to_end):.3f} ms")
                    print(f"       Std:    {np.std(sub_to_end):.3f} ms")
                    print(f"       Min:    {np.min(sub_to_end):.3f} ms")
                    print(f"       Max:    {np.max(sub_to_end):.3f} ms")
                    print(f"       Samples: {len(sub_to_end)}")
                
                if diffs["pub_to_end"]:
                    pub_to_end = diffs["pub_to_end"]
                    print(f"\n     {camera_id}_end_t - {camera_id}_pub_t (total):")
                    print(f"       Mean:   {np.mean(pub_to_end):.3f} ms")
                    print(f"       Std:    {np.std(pub_to_end):.3f} ms")
                    print(f"       Min:    {np.min(pub_to_end):.3f} ms")
                    print(f"       Max:    {np.max(pub_to_end):.3f} ms")
                    print(f"       Samples: {len(pub_to_end)}")
        else:
            print("   No per-camera timestamp data collected")
        
        print()
        
        # Multi-camera timestamp differences
        print("2. Multi-Camera Timestamp Differences (pub_t differences, absolute value):")
        print("   " + "-" * 76)
        
        multi_camera = self.camera_timestamp_diffs["multi_camera"]
        if multi_camera:
            # Count sync vs non-sync data
            sync_count = sum(1 for d in multi_camera if d.get("is_sync_data", False))
            non_sync_count = len(multi_camera) - sync_count
            print(f"\n   Data source: {sync_count} from sync mode, {non_sync_count} from non-sync mode")
            
            # Group by camera pair
            pair_diffs = {}
            for diff_data in multi_camera:
                pair = tuple(sorted(diff_data["camera_pair"]))
                if pair not in pair_diffs:
                    pair_diffs[pair] = []
                pair_diffs[pair].append(diff_data["diff_ms"])
            
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
    
    def _print_robot_timestamp_diff_analysis(self):
        """Print timestamp difference analysis for robot state."""
        print("\n" + "=" * 80)
        print("get_state() TIMESTAMP DIFFERENCE ANALYSIS")
        print("=" * 80 + "\n")
        
        diffs = self.robot_timestamp_diffs
        
        if diffs["pub_to_sub"]:
            pub_to_sub = diffs["pub_to_sub"]
            print("robot_sub_t - robot_pub_t:")
            print(f"  Mean:   {np.mean(pub_to_sub):.3f} ms")
            print(f"  Std:    {np.std(pub_to_sub):.3f} ms")
            print(f"  Min:    {np.min(pub_to_sub):.3f} ms")
            print(f"  Max:    {np.max(pub_to_sub):.3f} ms")
            print(f"  Samples: {len(pub_to_sub)}")
        
        if diffs["sub_to_end"]:
            sub_to_end = diffs["sub_to_end"]
            print(f"\nrobot_end_t - robot_sub_t:")
            print(f"  Mean:   {np.mean(sub_to_end):.3f} ms")
            print(f"  Std:    {np.std(sub_to_end):.3f} ms")
            print(f"  Min:    {np.min(sub_to_end):.3f} ms")
            print(f"  Max:    {np.max(sub_to_end):.3f} ms")
            print(f"  Samples: {len(sub_to_end)}")
        
        if diffs["pub_to_end"]:
            pub_to_end = diffs["pub_to_end"]
            print(f"\nrobot_end_t - robot_pub_t (total):")
            print(f"  Mean:   {np.mean(pub_to_end):.3f} ms")
            print(f"  Std:    {np.std(pub_to_end):.3f} ms")
            print(f"  Min:    {np.min(pub_to_end):.3f} ms")
            print(f"  Max:    {np.max(pub_to_end):.3f} ms")
            print(f"  Samples: {len(pub_to_end)}")
        
        if not any(diffs.values()):
            print("No timestamp data collected")
        
        print("\n" + "=" * 80 + "\n")
    
    def _plot_timestamp_visualization(self):
        """
        Generate comprehensive timestamp visualization.
        
        Creates timeline plots showing pub_t, sub_t, end_t for robot and cameras.
        Uses different markers for different timestamp types.
        """
        if not MATPLOTLIB_AVAILABLE:
            return
        
        if self.test_type is None:
            return
        
        # Determine what data to plot based on test type
        plot_robot = self.test_type in ['observation', 'get_state'] and len(self.robot_timestamps_raw) > 0
        plot_camera = self.test_type in ['observation', 'read_cameras'] and len(self.camera_timestamps_raw) > 0
        
        if not plot_robot and not plot_camera:
            print("⚠️  No timestamp data available for visualization")
            return
        
        # Create figure with subplots
        num_plots = sum([plot_robot, plot_camera])
        if num_plots == 0:
            return
        
        # Create figure: main timeline + time diff subplot for each + pub_t comparison
        # Add one more subplot for pub_t comparison if both robot and camera data available
        has_both = plot_robot and plot_camera
        num_subplots = num_plots * 2 + (1 if has_both else 0)
        
        fig = plt.figure(figsize=(18, 6 * num_subplots))
        fig.suptitle(f'Timestamp Visualization - {self.test_type.upper()} Test', 
                     fontsize=16, fontweight='bold')
        
        plot_idx = 1
        
        # Plot robot timestamps (timeline + time diff)
        if plot_robot:
            # Timeline plot
            ax_robot = plt.subplot(num_subplots, 1, plot_idx)
            self._plot_robot_timestamps(ax_robot)
            plot_idx += 1
            
            # Time difference plot
            ax_robot_diff = plt.subplot(num_subplots, 1, plot_idx)
            self._plot_robot_time_diffs(ax_robot_diff)
            plot_idx += 1
        
        # Plot camera timestamps (timeline + time diff)
        if plot_camera:
            # Timeline plot
            ax_camera = plt.subplot(num_subplots, 1, plot_idx)
            self._plot_camera_timestamps(ax_camera)
            plot_idx += 1
            
            # Time difference plot
            ax_camera_diff = plt.subplot(num_subplots, 1, plot_idx)
            self._plot_camera_time_diffs(ax_camera_diff)
            plot_idx += 1
        
        # Plot pub_t comparison (robot vs all cameras)
        if has_both:
            ax_pub_comparison = plt.subplot(num_subplots, 1, plot_idx)
            self._plot_pub_t_comparison(ax_pub_comparison)
        
        plt.tight_layout()
        
        # Save figure
        output_file = f'timestamp_visualization_{self.test_type}.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\n📊 Timestamp visualization saved to: {output_file}")
        
        # Also save as PDF
        try:
            output_pdf = f'timestamp_visualization_{self.test_type}.pdf'
            plt.savefig(output_pdf, dpi=150, bbox_inches='tight')
            print(f"📊 High-quality PDF saved to: {output_pdf}")
        except Exception:
            pass
        
        plt.close()
    
    def _plot_robot_timestamps(self, ax):
        """
        Plot robot timestamp timeline.
        
        Shows: robot_polymetis_t, robot_pub_t, robot_sub_t, robot_end_t
        with different markers and colors.
        """
        if not self.robot_timestamps_raw:
            ax.text(0.5, 0.5, 'No robot timestamp data', ha='center', va='center', fontsize=12)
            return
        
        # Extract timestamps
        iterations = list(range(len(self.robot_timestamps_raw)))
        polymetis_ts = []
        pub_ts = []
        sub_ts = []
        end_ts = []
        
        # Base time (first timestamp) for relative time display
        base_time_ns = None
        
        for ts_dict in self.robot_timestamps_raw:
            # Get timestamps (in nanoseconds)
            polymetis_t = ts_dict.get("robot_polymetis_t", 0)
            pub_t = ts_dict.get("robot_pub_t", 0)
            sub_t = ts_dict.get("robot_sub_t", 0)
            end_t = ts_dict.get("robot_end_t", 0)
            
            # Set base time from first valid timestamp
            if base_time_ns is None:
                for t in [polymetis_t, pub_t, sub_t, end_t]:
                    if t > 0:
                        base_time_ns = t
                        break
            
            polymetis_ts.append(polymetis_t)
            pub_ts.append(pub_t)
            sub_ts.append(sub_t)
            end_ts.append(end_t)
        
        # Convert to relative time (ms) from base
        if base_time_ns is None:
            base_time_ns = 0
        
        def to_ms(ts_ns):
            return (ts_ns - base_time_ns) / 1e6 if ts_ns > 0 else None
        
        polymetis_ms = [to_ms(t) for t in polymetis_ts]
        pub_ms = [to_ms(t) for t in pub_ts]
        sub_ms = [to_ms(t) for t in sub_ts]
        end_ms = [to_ms(t) for t in end_ts]
        
        # Plot with different markers
        # robot_polymetis_t: star marker, cyan
        valid_polymetis = [(i, t) for i, t in enumerate(polymetis_ms) if t is not None]
        if valid_polymetis:
            idxs, vals = zip(*valid_polymetis)
            ax.scatter(idxs, vals, marker='*', s=100, c='cyan', 
                      label='robot_polymetis_t', alpha=0.7, edgecolors='darkcyan', linewidths=1)
        
        # robot_pub_t: circle marker, blue
        valid_pub = [(i, t) for i, t in enumerate(pub_ms) if t is not None]
        if valid_pub:
            idxs, vals = zip(*valid_pub)
            ax.scatter(idxs, vals, marker='o', s=80, c='blue', 
                      label='robot_pub_t', alpha=0.7, edgecolors='darkblue', linewidths=1)
        
        # robot_sub_t: square marker, green
        valid_sub = [(i, t) for i, t in enumerate(sub_ms) if t is not None]
        if valid_sub:
            idxs, vals = zip(*valid_sub)
            ax.scatter(idxs, vals, marker='s', s=80, c='green', 
                      label='robot_sub_t', alpha=0.7, edgecolors='darkgreen', linewidths=1)
        
        # robot_end_t: diamond marker, red
        valid_end = [(i, t) for i, t in enumerate(end_ms) if t is not None]
        if valid_end:
            idxs, vals = zip(*valid_end)
            ax.scatter(idxs, vals, marker='D', s=80, c='red', 
                      label='robot_end_t', alpha=0.7, edgecolors='darkred', linewidths=1)
        
        # Draw lines connecting pub_t -> sub_t -> end_t for each iteration
        for i in range(len(iterations)):
            if pub_ms[i] is not None and sub_ms[i] is not None:
                ax.plot([i, i], [pub_ms[i], sub_ms[i]], 'b-', alpha=0.3, linewidth=0.5)
            if sub_ms[i] is not None and end_ms[i] is not None:
                ax.plot([i, i], [sub_ms[i], end_ms[i]], 'g-', alpha=0.3, linewidth=0.5)
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Relative Timestamp (ms)', fontsize=12)
        ax.set_title('Robot Timestamp Timeline\n(polymetis_t: *, pub_t: o, sub_t: □, end_t: ◇)', 
                    fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # Add time difference annotations for first and last few iterations
        self._annotate_time_diffs(ax, iterations[:5] + iterations[-5:], 
                                  pub_ms, sub_ms, end_ms, 'robot')
    
    def _plot_camera_timestamps(self, ax):
        """
        Plot camera timestamp timeline.
        
        Shows: {camera_id}_pub_t, {camera_id}_sub_t, {camera_id}_end_t
        for each camera with different markers and colors.
        """
        if not self.camera_timestamps_raw:
            ax.text(0.5, 0.5, 'No camera timestamp data', ha='center', va='center', fontsize=12)
            return
        
        # Extract camera IDs from first timestamp dict
        first_ts = self.camera_timestamps_raw[0]
        camera_ids = set()
        for key in first_ts.keys():
            if key.endswith('_pub_t'):
                camera_ids.add(key.replace('_pub_t', ''))
        
        if not camera_ids:
            ax.text(0.5, 0.5, 'No camera IDs found in timestamp data', 
                   ha='center', va='center', fontsize=12)
            return
        
        camera_ids = sorted(camera_ids)
        iterations = list(range(len(self.camera_timestamps_raw)))
        
        # Base time (first timestamp) for relative time display
        base_time_ns = None
        for ts_dict in self.camera_timestamps_raw:
            for cam_id in camera_ids:
                pub_key = f"{cam_id}_pub_t"
                if pub_key in ts_dict and ts_dict[pub_key] > 0:
                    base_time_ns = ts_dict[pub_key]
                    break
            if base_time_ns is not None:
                break
        
        if base_time_ns is None:
            base_time_ns = 0
        
        # Color palette for different cameras
        colors = plt.cm.tab10(np.linspace(0, 1, len(camera_ids)))
        
        # Plot timestamps for each camera
        for cam_idx, cam_id in enumerate(camera_ids):
            pub_ts = []
            sub_ts = []
            end_ts = []
            
            for ts_dict in self.camera_timestamps_raw:
                pub_key = f"{cam_id}_pub_t"
                sub_key = f"{cam_id}_sub_t"
                end_key = f"{cam_id}_end_t"
                
                pub_ts.append(ts_dict.get(pub_key, 0))
                sub_ts.append(ts_dict.get(sub_key, 0))
                end_ts.append(ts_dict.get(end_key, 0))
            
            # Convert to relative time (ms)
            def to_ms(ts_ns):
                return (ts_ns - base_time_ns) / 1e6 if ts_ns > 0 else None
            
            pub_ms = [to_ms(t) for t in pub_ts]
            sub_ms = [to_ms(t) for t in sub_ts]
            end_ms = [to_ms(t) for t in end_ts]
            
            color = colors[cam_idx]
            
            # pub_t: circle marker
            valid_pub = [(i, t) for i, t in enumerate(pub_ms) if t is not None]
            if valid_pub:
                idxs, vals = zip(*valid_pub)
                ax.scatter(idxs, vals, marker='o', s=60, c=[color], 
                          label=f'{cam_id}_pub_t', alpha=0.7, edgecolors=color*0.7, linewidths=1)
            
            # sub_t: square marker
            valid_sub = [(i, t) for i, t in enumerate(sub_ms) if t is not None]
            if valid_sub:
                idxs, vals = zip(*valid_sub)
                ax.scatter(idxs, vals, marker='s', s=60, c=[color], 
                          label=f'{cam_id}_sub_t', alpha=0.5, edgecolors=color*0.7, linewidths=1)
            
            # end_t: diamond marker
            valid_end = [(i, t) for i, t in enumerate(end_ms) if t is not None]
            if valid_end:
                idxs, vals = zip(*valid_end)
                ax.scatter(idxs, vals, marker='D', s=60, c=[color], 
                          label=f'{cam_id}_end_t', alpha=0.5, edgecolors=color*0.7, linewidths=1)
            
            # Draw lines connecting pub_t -> sub_t -> end_t
            for i in range(len(iterations)):
                if pub_ms[i] is not None and sub_ms[i] is not None:
                    ax.plot([i, i], [pub_ms[i], sub_ms[i]], 
                           color=color, linestyle='-', alpha=0.2, linewidth=0.5)
                if sub_ms[i] is not None and end_ms[i] is not None:
                    ax.plot([i, i], [sub_ms[i], end_ms[i]], 
                           color=color, linestyle='--', alpha=0.2, linewidth=0.5)
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Relative Timestamp (ms)', fontsize=12)
        ax.set_title('Camera Timestamp Timeline\n(pub_t: o, sub_t: □, end_t: ◇)', 
                    fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=9, ncol=2)
        ax.grid(True, alpha=0.3)
        
        # Add time difference annotations
        # For cameras, we'll annotate the first camera's timestamps
        if camera_ids:
            first_cam_id = camera_ids[0]
            pub_ts = [ts_dict.get(f"{first_cam_id}_pub_t", 0) for ts_dict in self.camera_timestamps_raw]
            sub_ts = [ts_dict.get(f"{first_cam_id}_sub_t", 0) for ts_dict in self.camera_timestamps_raw]
            end_ts = [ts_dict.get(f"{first_cam_id}_end_t", 0) for ts_dict in self.camera_timestamps_raw]
            
            pub_ms = [(t - base_time_ns) / 1e6 if t > 0 else None for t in pub_ts]
            sub_ms = [(t - base_time_ns) / 1e6 if t > 0 else None for t in sub_ts]
            end_ms = [(t - base_time_ns) / 1e6 if t > 0 else None for t in end_ts]
            
            self._annotate_time_diffs(ax, iterations[:5] + iterations[-5:], 
                                      pub_ms, sub_ms, end_ms, f'camera_{first_cam_id}')
    
    def _annotate_time_diffs(self, ax, iterations, pub_ms, sub_ms, end_ms, prefix):
        """
        Annotate time differences on the plot.
        
        Args:
            ax: Matplotlib axis
            iterations: List of iteration indices to annotate
            pub_ms: List of pub_t values (ms, relative)
            sub_ms: List of sub_t values (ms, relative)
            end_ms: List of end_t values (ms, relative)
            prefix: Prefix for annotation text (e.g., 'robot', 'camera_11022812')
        """
        for i in iterations:
            if i >= len(pub_ms):
                continue
            
            # Calculate time differences
            annotations = []
            y_pos = None
            
            if pub_ms[i] is not None and sub_ms[i] is not None:
                diff = sub_ms[i] - pub_ms[i]
                annotations.append(f"pub→sub: {diff:.2f}ms")
                y_pos = (pub_ms[i] + sub_ms[i]) / 2
            
            if sub_ms[i] is not None and end_ms[i] is not None:
                diff = end_ms[i] - sub_ms[i]
                annotations.append(f"sub→end: {diff:.2f}ms")
                if y_pos is None:
                    y_pos = (sub_ms[i] + end_ms[i]) / 2
            
            if pub_ms[i] is not None and end_ms[i] is not None:
                diff = end_ms[i] - pub_ms[i]
                annotations.append(f"pub→end: {diff:.2f}ms")
            
            # Annotate (only for first few iterations to avoid clutter)
            if annotations and i < 3 and y_pos is not None:
                ax.annotate('\n'.join(annotations), 
                           xy=(i, y_pos), 
                           xytext=(10, 10), 
                           textcoords='offset points',
                           fontsize=7,
                           bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.5),
                           arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0', alpha=0.5))
    
    def _plot_robot_time_diffs(self, ax):
        """
        Plot robot timestamp differences over iterations.
        
        Shows: pub_to_sub, sub_to_end, pub_to_end differences.
        """
        if not self.robot_timestamps_raw:
            ax.text(0.5, 0.5, 'No robot timestamp data', ha='center', va='center', fontsize=12)
            return
        
        iterations = list(range(len(self.robot_timestamps_raw)))
        pub_to_sub_diffs = []
        sub_to_end_diffs = []
        pub_to_end_diffs = []
        
        for ts_dict in self.robot_timestamps_raw:
            pub_t = ts_dict.get("robot_pub_t", 0)
            sub_t = ts_dict.get("robot_sub_t", 0)
            end_t = ts_dict.get("robot_end_t", 0)
            
            if pub_t > 0 and sub_t > 0:
                pub_to_sub_diffs.append((sub_t - pub_t) / 1e6)  # Convert to ms
            else:
                pub_to_sub_diffs.append(None)
            
            if sub_t > 0 and end_t > 0:
                sub_to_end_diffs.append((end_t - sub_t) / 1e6)
            else:
                sub_to_end_diffs.append(None)
            
            if pub_t > 0 and end_t > 0:
                pub_to_end_diffs.append((end_t - pub_t) / 1e6)
            else:
                pub_to_end_diffs.append(None)
        
        # Plot time differences
        valid_pub_sub = [(i, d) for i, d in enumerate(pub_to_sub_diffs) if d is not None]
        if valid_pub_sub:
            idxs, diffs = zip(*valid_pub_sub)
            ax.plot(idxs, diffs, 'b-o', label='pub_t → sub_t', linewidth=1.5, markersize=4, alpha=0.7)
        
        valid_sub_end = [(i, d) for i, d in enumerate(sub_to_end_diffs) if d is not None]
        if valid_sub_end:
            idxs, diffs = zip(*valid_sub_end)
            ax.plot(idxs, diffs, 'g-s', label='sub_t → end_t', linewidth=1.5, markersize=4, alpha=0.7)
        
        valid_pub_end = [(i, d) for i, d in enumerate(pub_to_end_diffs) if d is not None]
        if valid_pub_end:
            idxs, diffs = zip(*valid_pub_end)
            ax.plot(idxs, diffs, 'r-D', label='pub_t → end_t (total)', linewidth=1.5, markersize=4, alpha=0.7)
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Time Difference (ms)', fontsize=12)
        ax.set_title('Robot Timestamp Differences Over Time', fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, alpha=0.3)
        
        # Add statistics text box
        if pub_to_sub_diffs and any(d is not None for d in pub_to_sub_diffs):
            valid_diffs = [d for d in pub_to_sub_diffs if d is not None]
            mean_val = np.mean(valid_diffs)
            std_val = np.std(valid_diffs)
            ax.text(0.02, 0.98, f'pub→sub: Mean={mean_val:.2f}ms, Std={std_val:.2f}ms',
                   transform=ax.transAxes, fontsize=9, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    def _plot_camera_time_diffs(self, ax):
        """
        Plot camera timestamp differences over iterations.
        
        Shows: pub_to_sub, sub_to_end, pub_to_end differences for each camera.
        """
        if not self.camera_timestamps_raw:
            ax.text(0.5, 0.5, 'No camera timestamp data', ha='center', va='center', fontsize=12)
            return
        
        # Extract camera IDs
        first_ts = self.camera_timestamps_raw[0]
        camera_ids = set()
        for key in first_ts.keys():
            if key.endswith('_pub_t'):
                camera_ids.add(key.replace('_pub_t', ''))
        
        if not camera_ids:
            return
        
        camera_ids = sorted(camera_ids)
        iterations = list(range(len(self.camera_timestamps_raw)))
        
        # Color palette for different cameras
        colors = plt.cm.tab10(np.linspace(0, 1, len(camera_ids)))
        
        # Plot time differences for each camera
        for cam_idx, cam_id in enumerate(camera_ids):
            pub_to_sub_diffs = []
            sub_to_end_diffs = []
            pub_to_end_diffs = []
            
            for ts_dict in self.camera_timestamps_raw:
                pub_key = f"{cam_id}_pub_t"
                sub_key = f"{cam_id}_sub_t"
                end_key = f"{cam_id}_end_t"
                
                pub_t = ts_dict.get(pub_key, 0)
                sub_t = ts_dict.get(sub_key, 0)
                end_t = ts_dict.get(end_key, 0)
                
                if pub_t > 0 and sub_t > 0:
                    pub_to_sub_diffs.append((sub_t - pub_t) / 1e6)
                else:
                    pub_to_sub_diffs.append(None)
                
                if sub_t > 0 and end_t > 0:
                    sub_to_end_diffs.append((end_t - sub_t) / 1e6)
                else:
                    sub_to_end_diffs.append(None)
                
                if pub_t > 0 and end_t > 0:
                    pub_to_end_diffs.append((end_t - pub_t) / 1e6)
                else:
                    pub_to_end_diffs.append(None)
            
            color = colors[cam_idx]
            
            # Plot pub_to_sub
            valid_pub_sub = [(i, d) for i, d in enumerate(pub_to_sub_diffs) if d is not None]
            if valid_pub_sub:
                idxs, diffs = zip(*valid_pub_sub)
                ax.plot(idxs, diffs, 'o-', color=color, 
                       label=f'{cam_id}: pub→sub', linewidth=1.5, markersize=4, alpha=0.7)
            
            # Plot sub_to_end
            valid_sub_end = [(i, d) for i, d in enumerate(sub_to_end_diffs) if d is not None]
            if valid_sub_end:
                idxs, diffs = zip(*valid_sub_end)
                ax.plot(idxs, diffs, 's--', color=color, 
                       label=f'{cam_id}: sub→end', linewidth=1.5, markersize=4, alpha=0.5)
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Time Difference (ms)', fontsize=12)
        ax.set_title('Camera Timestamp Differences Over Time', fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=9, ncol=2)
        ax.grid(True, alpha=0.3)
    
    def _plot_pub_t_comparison(self, ax):
        """
        Plot comparison of robot_pub_t and all camera pub_t timestamps.
        
        Shows all camera pub_t and robot_pub_t on the same timeline for easy comparison.
        """
        if not self.robot_timestamps_raw or not self.camera_timestamps_raw:
            ax.text(0.5, 0.5, 'No robot or camera timestamp data', 
                   ha='center', va='center', fontsize=12)
            return
        
        # Extract camera IDs from first camera timestamp dict
        first_camera_ts = self.camera_timestamps_raw[0]
        camera_ids = set()
        for key in first_camera_ts.keys():
            if key.endswith('_pub_t'):
                camera_ids.add(key.replace('_pub_t', ''))
        
        if not camera_ids:
            ax.text(0.5, 0.5, 'No camera IDs found', 
                   ha='center', va='center', fontsize=12)
            return
        
        camera_ids = sorted(camera_ids)
        iterations = list(range(min(len(self.robot_timestamps_raw), 
                                   len(self.camera_timestamps_raw))))
        
        # Find base time (first valid timestamp from robot or any camera)
        base_time_ns = None
        for ts_dict in self.robot_timestamps_raw:
            robot_pub_t = ts_dict.get("robot_pub_t", 0)
            if robot_pub_t > 0:
                base_time_ns = robot_pub_t
                break
        
        if base_time_ns is None:
            for ts_dict in self.camera_timestamps_raw:
                for cam_id in camera_ids:
                    pub_key = f"{cam_id}_pub_t"
                    if pub_key in ts_dict and ts_dict[pub_key] > 0:
                        base_time_ns = ts_dict[pub_key]
                        break
                if base_time_ns is not None:
                    break
        
        if base_time_ns is None:
            base_time_ns = 0
        
        # Convert to relative time (ms)
        def to_ms(ts_ns):
            return (ts_ns - base_time_ns) / 1e6 if ts_ns > 0 else None
        
        # Extract robot_pub_t
        robot_pub_ts = []
        for ts_dict in self.robot_timestamps_raw[:len(iterations)]:
            robot_pub_t = ts_dict.get("robot_pub_t", 0)
            robot_pub_ts.append(robot_pub_t)
        
        robot_pub_ms = [to_ms(t) for t in robot_pub_ts]
        
        # Plot robot_pub_t (thick line, red)
        valid_robot = [(i, t) for i, t in enumerate(robot_pub_ms) if t is not None]
        if valid_robot:
            idxs, vals = zip(*valid_robot)
            ax.plot(idxs, vals, 'r-', linewidth=2.5, label='robot_pub_t', 
                   alpha=0.8, marker='o', markersize=5)
        
        # Color palette for different cameras
        colors = plt.cm.tab10(np.linspace(0, 1, len(camera_ids)))
        
        # Extract and plot each camera's pub_t
        for cam_idx, cam_id in enumerate(camera_ids):
            camera_pub_ts = []
            for ts_dict in self.camera_timestamps_raw[:len(iterations)]:
                pub_key = f"{cam_id}_pub_t"
                camera_pub_t = ts_dict.get(pub_key, 0)
                camera_pub_ts.append(camera_pub_t)
            
            camera_pub_ms = [to_ms(t) for t in camera_pub_ts]
            
            color = colors[cam_idx]
            valid_camera = [(i, t) for i, t in enumerate(camera_pub_ms) if t is not None]
            
            if valid_camera:
                idxs, vals = zip(*valid_camera)
                ax.plot(idxs, vals, '-', color=color, linewidth=1.5, 
                       label=f'{cam_id}_pub_t', alpha=0.7, marker='s', markersize=4)
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Relative Timestamp (ms)', fontsize=12)
        ax.set_title('Publication Timestamp Comparison\n(robot_pub_t vs all camera pub_t)', 
                    fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=10, ncol=2)
        ax.grid(True, alpha=0.3)
        
        # Add statistics text box
        if valid_robot and len(valid_robot) > 0:
            robot_vals = [v for _, v in valid_robot]
            # Calculate time differences between robot and each camera
            stats_text = []
            for cam_idx, cam_id in enumerate(camera_ids):
                camera_pub_ts = []
                for ts_dict in self.camera_timestamps_raw[:len(iterations)]:
                    pub_key = f"{cam_id}_pub_t"
                    camera_pub_t = ts_dict.get(pub_key, 0)
                    camera_pub_ts.append(camera_pub_t)
                
                camera_pub_ms = [to_ms(t) for t in camera_pub_ts]
                valid_camera = [(i, t) for i, t in enumerate(camera_pub_ms) if t is not None]
                
                if valid_camera and len(valid_camera) > 0:
                    camera_vals = [v for _, v in valid_camera]
                    # Calculate differences (camera - robot) for matching iterations
                    diffs = []
                    for i in range(min(len(robot_vals), len(camera_vals))):
                        if robot_vals[i] is not None and camera_vals[i] is not None:
                            diffs.append(camera_vals[i] - robot_vals[i])
                    
                    if diffs:
                        mean_diff = np.mean(diffs)
                        std_diff = np.std(diffs)
                        stats_text.append(f'{cam_id}: {mean_diff:+.2f}±{std_diff:.2f}ms')
            
            if stats_text:
                stats_str = 'Camera-robot diff:\n' + '\n'.join(stats_text[:5])  # Limit to 5 cameras
                ax.text(0.02, 0.98, stats_str,
                       transform=ax.transAxes, fontsize=9, verticalalignment='top',
                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))


def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Test RobotEnv observation, read_cameras, and get_state functions'
    )
    parser.add_argument('--test', type=str, default='all',
                       choices=['all', 'observation', 'read_cameras', 'get_state'],
                       help='Which test to run (default: all)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Test duration in seconds (default: 10.0)')
    parser.add_argument('--fps', type=float, default=20.0,
                       help='Test frequency in Hz (default: 20.0)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = ObservationTestNode()
        
        # No need to spin - RobotEnv's MultiThreadedExecutor handles callbacks in background
        time.sleep(2.0)  # Wait for initial data to arrive
        
        # Run selected test(s)
        if args.test == 'all' or args.test == 'observation':
            print("\n" + "=" * 80)
            print("TEST 1: get_observation()")
            print("=" * 80)
            node.test_get_observation(duration_sec=args.duration, fps=args.fps)
            time.sleep(1.0)  # Brief pause between tests
        
        if args.test == 'all' or args.test == 'read_cameras':
            print("\n" + "=" * 80)
            print("TEST 2: read_cameras()")
            print("=" * 80)
            node.test_read_cameras(duration_sec=args.duration, fps=args.fps)
            time.sleep(1.0)  # Brief pause between tests
        
        if args.test == 'all' or args.test == 'get_state':
            print("\n" + "=" * 80)
            print("TEST 3: get_state()")
            print("=" * 80)
            node.test_get_state(duration_sec=args.duration, fps=args.fps)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


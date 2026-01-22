#!/usr/bin/env python3
"""
Test script to detect multi-camera and robot state synchronization.

This script:
1. Continuously calls get_observation() at a fixed control rate (default 15 Hz)
2. Analyzes timestamps to detect synchronization issues between:
   - Multiple cameras (camera-to-camera sync)
   - Cameras and robot state (camera-to-robot sync)
3. Displays real-time statistics and warnings for sync issues

Usage:
    python3 test_camera_robot_sync.py
    python3 test_camera_robot_sync.py --control-hz 20.0 --duration 30.0
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Any
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node


from role_ros2.robot_env import RobotEnv
from role_ros2.misc.ros2_utils import get_ros_time_ns


class CameraRobotSyncTester(Node):
    """
    Test node for detecting multi-camera and robot state synchronization issues.
    """
    
    def __init__(self, control_hz: float = 15.0, duration: float = 60.0):
        """
        Initialize sync tester.
        
        Args:
            control_hz: Control loop frequency in Hz (default: 15.0)
            duration: Test duration in seconds (default: 60.0)
        """
        super().__init__('camera_robot_sync_tester')
        
        self.control_hz = control_hz
        self.duration = duration
        self.control_period = 1.0 / control_hz if control_hz > 0 else 1.0
        
        # Statistics storage
        self.stats = {
            'total_steps': 0,
            'camera_sync_latencies': [],  # Max - min camera pub_t per step
            'robot_cam_latencies': [],   # Robot pub_t - max(camera pub_t) per step
            'camera_pub_times': defaultdict(list),  # Per-camera pub_t values
            'robot_pub_times': [],  # Robot pub_t values
            'step_times': [],  # Step start times
            'sync_warnings': [],  # List of (step, warning_message)
        }
        
        # Thresholds for warnings (in milliseconds)
        self.camera_sync_threshold_ms = 50.0  # Max allowed camera-to-camera delay
        self.robot_cam_threshold_ms = 100.0   # Max allowed robot-to-camera delay
        
        # Initialization wait period (skip warnings during startup)
        self.init_wait_steps = 10  # Wait 10 steps for cameras to initialize
        self.init_wait_complete = False
        
        # Initialize RobotEnv
        self.get_logger().info("Initializing RobotEnv...")
        self.robot_env = RobotEnv(node=self)
        self.get_logger().info("RobotEnv initialized successfully")
        
    def analyze_timestamps(self, obs: Dict[str, Any], step: int) -> Dict[str, Any]:
        """
        Analyze timestamps from observation to detect sync issues.
        
        Args:
            obs: Observation dictionary from get_observation()
            step: Current step number
            
        Returns:
            dict: Analysis results with warnings and statistics
        """
        analysis = {
            'step': step,
            'warnings': [],
            'camera_sync_latency_ms': None,
            'robot_cam_latency_ms': None,
            'camera_pub_times': {},
            'robot_pub_t': None,
        }
        
        ts_data = obs.get("timestamp", {})
        if not ts_data:
            analysis['warnings'].append("No timestamp data in observation")
            return analysis
        
        # Extract camera timestamps
        camera_ts = ts_data.get("cameras", {})
        robot_ts = ts_data.get("robot_state", {})
        
        # Debug: Print available timestamp keys on first few steps
        if step < 3:
            self.get_logger().debug(
                f"Step {step}: Camera timestamp keys: {list(camera_ts.keys())}"
            )
        
        # Get camera pub_t times
        camera_pub_times = {}
        for key, value in camera_ts.items():
            if key.endswith("_pub_t") and value is not None:
                camera_id = key.replace("_pub_t", "")
                camera_pub_times[camera_id] = value
                analysis['camera_pub_times'][camera_id] = value
        
        # Get robot pub_t
        robot_pub_t = robot_ts.get("robot_pub_t")
        if robot_pub_t is not None:
            analysis['robot_pub_t'] = robot_pub_t
        
        # Analyze camera-to-camera synchronization
        # Skip warnings during initialization period
        skip_warning = step < self.init_wait_steps
        
        if len(camera_pub_times) >= 2:
            pub_times_ns = list(camera_pub_times.values())
            max_pub_t = max(pub_times_ns)
            min_pub_t = min(pub_times_ns)
            camera_sync_latency_ns = max_pub_t - min_pub_t
            camera_sync_latency_ms = camera_sync_latency_ns / 1e6
            
            analysis['camera_sync_latency_ms'] = camera_sync_latency_ms
            
            if camera_sync_latency_ms > self.camera_sync_threshold_ms and not skip_warning:
                warning = (
                    f"Step {step}: Camera sync latency {camera_sync_latency_ms:.2f} ms "
                    f"(threshold: {self.camera_sync_threshold_ms} ms)"
                )
                analysis['warnings'].append(warning)
                self.stats['sync_warnings'].append((step, warning))
        elif len(camera_pub_times) == 1:
            if not skip_warning:
                analysis['warnings'].append(
                    f"Step {step}: Only 1 camera timestamp found (need ≥2 for sync analysis). "
                    f"Found camera: {list(camera_pub_times.keys())}"
                )
        else:
            # No camera timestamps found
            if not skip_warning:
                # Only warn if we're past initialization and still no data
                available_keys = list(camera_ts.keys())
                analysis['warnings'].append(
                    f"Step {step}: No camera timestamps found. "
                    f"Available camera timestamp keys: {available_keys}"
                )
            elif step == self.init_wait_steps - 1:
                # Log debug info on last init step
                available_keys = list(camera_ts.keys())
                self.get_logger().debug(
                    f"Step {step} (init): No camera timestamps yet. "
                    f"Available keys: {available_keys}"
                )
        
        # Analyze robot-to-camera synchronization
        if robot_pub_t is not None and camera_pub_times:
            max_camera_pub_t = max(camera_pub_times.values())
            robot_cam_latency_ns = abs(robot_pub_t - max_camera_pub_t)
            robot_cam_latency_ms = robot_cam_latency_ns / 1e6
            
            analysis['robot_cam_latency_ms'] = robot_cam_latency_ms
            
            if robot_cam_latency_ms > self.robot_cam_threshold_ms:
                warning = (
                    f"Step {step}: Robot-Camera latency {robot_cam_latency_ms:.2f} ms "
                    f"(threshold: {self.robot_cam_threshold_ms} ms)"
                )
                analysis['warnings'].append(warning)
                self.stats['sync_warnings'].append((step, warning))
        
        return analysis
    
    def update_statistics(self, analysis: Dict[str, Any]):
        """
        Update global statistics from analysis results.
        
        Args:
            analysis: Analysis results from analyze_timestamps()
        """
        self.stats['total_steps'] += 1
        
        # Camera sync latency
        if analysis['camera_sync_latency_ms'] is not None:
            self.stats['camera_sync_latencies'].append(analysis['camera_sync_latency_ms'])
        
        # Robot-camera latency
        if analysis['robot_cam_latency_ms'] is not None:
            self.stats['robot_cam_latencies'].append(analysis['robot_cam_latency_ms'])
        
        # Store camera pub times per camera
        for cam_id, pub_t in analysis['camera_pub_times'].items():
            self.stats['camera_pub_times'][cam_id].append(pub_t)
        
        # Store robot pub time
        if analysis['robot_pub_t'] is not None:
            self.stats['robot_pub_times'].append(analysis['robot_pub_t'])
    
    def print_statistics(self):
        """Print current statistics summary."""
        print("\n" + "=" * 80)
        print("SYNCHRONIZATION STATISTICS")
        print("=" * 80)
        print(f"Total steps: {self.stats['total_steps']}")
        print(f"Control frequency: {self.control_hz} Hz")
        print()
        
        # Camera sync statistics
        if self.stats['camera_sync_latencies']:
            arr = np.array(self.stats['camera_sync_latencies'])
            print("Camera-to-Camera Sync Latency:")
            print(f"  Mean: {arr.mean():.2f} ms")
            print(f"  Std:  {arr.std():.2f} ms")
            print(f"  Min:  {arr.min():.2f} ms")
            print(f"  Max:  {arr.max():.2f} ms")
            print(f"  Threshold: {self.camera_sync_threshold_ms} ms")
            if arr.max() > self.camera_sync_threshold_ms:
                print(f"  ⚠️  WARNING: Max latency exceeds threshold!")
        else:
            print("Camera-to-Camera Sync: No data (need ≥2 cameras)")
        
        print()
        
        # Robot-camera sync statistics
        if self.stats['robot_cam_latencies']:
            arr = np.array(self.stats['robot_cam_latencies'])
            print("Robot-to-Camera Sync Latency:")
            print(f"  Mean: {arr.mean():.2f} ms")
            print(f"  Std:  {arr.std():.2f} ms")
            print(f"  Min:  {arr.min():.2f} ms")
            print(f"  Max:  {arr.max():.2f} ms")
            print(f"  Threshold: {self.robot_cam_threshold_ms} ms")
            if arr.max() > self.robot_cam_threshold_ms:
                print(f"  ⚠️  WARNING: Max latency exceeds threshold!")
        else:
            print("Robot-to-Camera Sync: No data")
        
        print()
        
        # Sync warnings summary
        if self.stats['sync_warnings']:
            print(f"⚠️  Total sync warnings: {len(self.stats['sync_warnings'])}")
            print("Recent warnings (last 5):")
            for step, warning in self.stats['sync_warnings'][-5:]:
                print(f"  {warning}")
        else:
            print("✅ No sync warnings detected")
        
        print("=" * 80 + "\n")
    
    def run_test(self):
        """Run the synchronization test loop."""
        print("\n" + "=" * 80)
        print("CAMERA-ROBOT SYNCHRONIZATION TEST")
        print("=" * 80)
        print(f"Control frequency: {self.control_hz} Hz")
        print(f"Test duration: {self.duration} seconds")
        print(f"Camera sync threshold: {self.camera_sync_threshold_ms} ms")
        print(f"Robot-camera sync threshold: {self.robot_cam_threshold_ms} ms")
        print("=" * 80)
        print("\nStarting test... (Press Ctrl+C to stop early)\n")
        
        start_time = time.time()
        step = 0
        next_print_time = start_time + 5.0  # Print stats every 5 seconds
        
        try:
            while time.time() - start_time < self.duration:
                step_start_time = time.time()
                step_start_ns = get_ros_time_ns(self)
                self.stats['step_times'].append(step_start_ns)
                
                # Get observation
                try:
                    obs = self.robot_env.get_observation(use_sync=True)
                except Exception as e:
                    self.get_logger().error(f"Error getting observation at step {step}: {e}")
                    time.sleep(self.control_period)
                    continue
                
                # Analyze timestamps
                analysis = self.analyze_timestamps(obs, step)
                
                # Mark initialization complete if we have camera data
                if not self.init_wait_complete and len(analysis['camera_pub_times']) >= 2:
                    self.init_wait_complete = True
                    self.get_logger().info(
                        f"✅ Camera data ready at step {step}. "
                        f"Found {len(analysis['camera_pub_times'])} cameras: {list(analysis['camera_pub_times'].keys())}"
                    )
                
                # Update statistics (only after initialization)
                if step >= self.init_wait_steps or self.init_wait_complete:
                    self.update_statistics(analysis)
                
                # Print warnings immediately (skip during initialization)
                if analysis['warnings'] and (step >= self.init_wait_steps or self.init_wait_complete):
                    for warning in analysis['warnings']:
                        print(f"⚠️  {warning}")
                
                # Print statistics periodically
                current_time = time.time()
                if current_time >= next_print_time:
                    self.print_statistics()
                    next_print_time = current_time + 5.0
                
                # Regularize control frequency
                comp_time = time.time() - step_start_time
                sleep_time = self.control_period - comp_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                step += 1
        
        except KeyboardInterrupt:
            self.get_logger().info("Test interrupted by user")
        
        # Final statistics
        print("\n" + "=" * 80)
        print("FINAL STATISTICS")
        print("=" * 80)
        self.print_statistics()
        
        # Calculate actual FPS
        if len(self.stats['step_times']) >= 2:
            total_time_ns = self.stats['step_times'][-1] - self.stats['step_times'][0]
            total_time_s = total_time_ns / 1e9
            actual_fps = (len(self.stats['step_times']) - 1) / total_time_s if total_time_s > 0 else 0
            print(f"Actual FPS: {actual_fps:.2f} Hz (target: {self.control_hz} Hz)")
        
        print("=" * 80 + "\n")
    
    def shutdown(self):
        """Clean up resources."""
        if hasattr(self, 'robot_env'):
            self.robot_env.shutdown()
        self.get_logger().info("Sync tester shutdown complete")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test multi-camera and robot state synchronization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Default test (15 Hz, 60 seconds)
    python3 test_camera_robot_sync.py
    
    # Custom control frequency and duration
    python3 test_camera_robot_sync.py --control-hz 20.0 --duration 30.0
    
    # Lower thresholds for stricter sync requirements
    python3 test_camera_robot_sync.py --camera-sync-threshold 20.0 --robot-cam-threshold 50.0
        """
    )
    
    parser.add_argument(
        '--control-hz',
        type=float,
        default=15.0,
        help='Control loop frequency in Hz (default: 15.0)'
    )
    
    parser.add_argument(
        '--duration',
        type=float,
        default=60.0,
        help='Test duration in seconds (default: 60.0)'
    )
    
    parser.add_argument(
        '--camera-sync-threshold',
        type=float,
        default=50.0,
        help='Camera-to-camera sync latency threshold in ms (default: 50.0)'
    )
    
    parser.add_argument(
        '--robot-cam-threshold',
        type=float,
        default=100.0,
        help='Robot-to-camera sync latency threshold in ms (default: 100.0)'
    )
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        tester = CameraRobotSyncTester(
            control_hz=args.control_hz,
            duration=args.duration
        )
        
        # Set thresholds
        tester.camera_sync_threshold_ms = args.camera_sync_threshold
        tester.robot_cam_threshold_ms = args.robot_cam_threshold
        
        tester.run_test()
        tester.shutdown()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

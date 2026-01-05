#!/usr/bin/env python3
"""
Test script for FrankaRobot.get_robot_state().

Tests robot state retrieval directly from FrankaRobot,
measuring latency, packet loss, and timestamp differences.
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node

# Matplotlib for visualization
try:
    import matplotlib
    matplotlib.use('Agg')  # Use non-interactive backend
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("⚠️  Matplotlib not available. Visualization will be skipped.")


from role_ros2.robot.franka.robot import FrankaRobot


class FrankaRobotGetStateTestNode(Node):
    """Test node for FrankaRobot.get_robot_state() testing."""
    
    def __init__(self):
        super().__init__('franka_robot_get_state_test_node')
        self.get_logger().info("Initializing FrankaRobot get_state test node...")
        
        # Initialize robot
        try:
            self.robot = FrankaRobot(node=self)
            self.get_logger().info("✅ FrankaRobot initialized")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize FrankaRobot: {e}")
            self.robot = None
            import traceback
            traceback.print_exc()
        
        # Statistics for packet loss analysis
        self.get_state_stats = {
            "total_calls": 0,
            "successful_calls": 0,
            "failed_calls": 0,
            "missing_robot_state": 0
        }
        
        # Statistics for timestamp difference analysis
        self.robot_timestamp_diffs = {
            "pub_to_sub": [],  # robot_sub_t - robot_pub_t
            "sub_to_end": [],  # robot_end_t - robot_sub_t
            "pub_to_end": []  # robot_end_t - robot_pub_t
        }
        
        # Raw timestamp data for visualization
        self.robot_timestamps_raw: List[Dict] = []
    
    def test_get_state(self, duration_sec=10.0, fps=20.0):
        """
        Test get_robot_state() at fixed frequency and calculate packet loss rate.
        
        Args:
            duration_sec: Test duration in seconds (default: 10.0)
            fps: Frequency of get_robot_state() calls in Hz (default: 20.0)
        """
        if self.robot is None:
            self.get_logger().error("FrankaRobot not initialized. Cannot run test.")
            return
        
        print("\n" + "=" * 80)
        print("Testing FrankaRobot.get_robot_state()")
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
            
            # No need to spin - FrankaRobot's background thread handles callbacks
            # Messages arrive at 50Hz and are processed continuously by the background thread
            
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
                state_dict, timestamp_dict = self.robot.get_robot_state()
                self.get_state_stats["total_calls"] += 1
                
                # Store raw timestamp data for visualization
                self.robot_timestamps_raw.append(timestamp_dict.copy())
                
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
                
                # Analyze timestamp differences for robot state
                self._analyze_robot_timestamp_diffs(timestamp_dict)
                
            except Exception as e:
                self.get_state_stats["total_calls"] += 1
                self.get_state_stats["failed_calls"] += 1
                self.get_logger().error(f"❌ Error in get_robot_state: {e}")
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
            self._plot_timestamp_visualization()
    
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
    
    def _print_get_state_packet_loss(self):
        """Print packet loss analysis for get_robot_state()."""
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
        
        Creates timeline plots showing pub_t, sub_t, end_t for robot.
        Uses different markers for different timestamp types.
        """
        if not MATPLOTLIB_AVAILABLE:
            return
        
        if not self.robot_timestamps_raw:
            print("⚠️  No timestamp data available for visualization")
            return
        
        # Create figure with subplots: timeline + time diff
        fig = plt.figure(figsize=(18, 12))
        fig.suptitle('FrankaRobot.get_robot_state() Timestamp Visualization', 
                     fontsize=16, fontweight='bold')
        
        # Timeline plot
        ax_timeline = plt.subplot(2, 1, 1)
        self._plot_robot_timestamps(ax_timeline)
        
        # Time difference plot
        ax_diff = plt.subplot(2, 1, 2)
        self._plot_robot_time_diffs(ax_diff)
        
        plt.tight_layout()
        
        # Save figure
        output_file = 'franka_robot_get_state_timestamp_visualization.png'
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\n📊 Timestamp visualization saved to: {output_file}")
        
        # Also save as PDF
        try:
            output_pdf = 'franka_robot_get_state_timestamp_visualization.pdf'
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


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Test FrankaRobot.get_robot_state() directly'
    )
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Test duration in seconds (default: 10.0)')
    parser.add_argument('--fps', type=float, default=20.0,
                       help='Test frequency in Hz (default: 20.0)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = FrankaRobotGetStateTestNode()
        
        # No need to spin - FrankaRobot's background thread handles callbacks
        time.sleep(2.0)  # Wait for initial data to arrive
        
        # Run test
        print("\n" + "=" * 80)
        print("TEST: FrankaRobot.get_robot_state()")
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

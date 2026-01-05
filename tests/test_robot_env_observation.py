#!/usr/bin/env python3
"""
Test script for RobotEnv.get_observation().

Tests whether image data and robot state are received correctly,
and calculates publish latency.
"""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

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
    
    def test_get_observation(self, num_iterations=10):
        """Test get_observation() and calculate latencies."""
        if self.robot_env is None:
            self.get_logger().error("RobotEnv not initialized. Cannot run test.")
            return
        
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Starting observation test ({num_iterations} iterations)")
        self.get_logger().info("=" * 80)
        
        latencies = {
            "robot_state": [],
            "camera": [],
            "total": []
        }
        
        for i in range(num_iterations):
            self.get_logger().info(f"\n--- Iteration {i+1}/{num_iterations} ---")
            
            # Get observation
            obs_start_time = self.get_clock().now()
            obs_start_ms = int(obs_start_time.nanoseconds // 1_000_000)
            
            try:
                obs = self.robot_env.get_observation()
                obs_end_time = self.get_clock().now()
                obs_end_ms = int(obs_end_time.nanoseconds // 1_000_000)
                
                total_latency_ms = obs_end_ms - obs_start_ms
                latencies["total"].append(total_latency_ms)
                
                # Check robot state
                if "robot_state" in obs:
                    robot_state = obs["robot_state"]
                    robot_timestamp = obs.get("timestamp", {}).get("robot_state", {})
                    
                    self.get_logger().info(f"✅ Robot state received:")
                    self.get_logger().info(f"   - Cartesian position: {robot_state.get('cartesian_position', 'N/A')[:3]}")
                    self.get_logger().info(f"   - Joint positions: {robot_state.get('joint_positions', 'N/A')[:3]}...")
                    self.get_logger().info(f"   - Gripper position: {robot_state.get('gripper_position', 'N/A')}")
                    
                    # Calculate robot state latency
                    if "ros_time_ms" in robot_timestamp:
                        robot_state_time_ms = robot_timestamp["ros_time_ms"]
                        robot_latency_ms = obs_start_ms - robot_state_time_ms
                        latencies["robot_state"].append(robot_latency_ms)
                        self.get_logger().info(f"   - Robot state latency: {robot_latency_ms} ms")
                    
                    if "read_start" in robot_timestamp and "read_end" in robot_timestamp:
                        read_duration = robot_timestamp["read_end"] - robot_timestamp["read_start"]
                        self.get_logger().info(f"   - Read duration: {read_duration} ms")
                else:
                    self.get_logger().warn("❌ Robot state not found in observation")
                
                # Check camera data
                camera_timestamp = obs.get("timestamp", {}).get("cameras", {})
                image_dict = obs.get("image", {})
                depth_dict = obs.get("depth", {})
                
                if image_dict or depth_dict:
                    self.get_logger().info(f"✅ Camera data received:")
                    self.get_logger().info(f"   - Image cameras: {list(image_dict.keys())}")
                    self.get_logger().info(f"   - Depth cameras: {list(depth_dict.keys())}")
                    
                    # Calculate camera latency for each camera
                    for cam_id in image_dict.keys():
                        frame_received_key = f"{cam_id}_frame_received"
                        pub_t_key = f"{cam_id}_pub_t"
                        sub_t_key = f"{cam_id}_sub_t"
                        end_t_key = f"{cam_id}_end_t"
                        
                        if frame_received_key in camera_timestamp:
                            frame_received_ms = camera_timestamp[frame_received_key]
                            camera_latency_ms = obs_start_ms - frame_received_ms
                            latencies["camera"].append(camera_latency_ms)
                            
                            self.get_logger().info(f"   - Camera {cam_id}:")
                            self.get_logger().info(f"     * Frame received: {frame_received_ms} ms")
                            self.get_logger().info(f"     * Camera latency: {camera_latency_ms} ms")
                            
                            if pub_t_key in camera_timestamp:
                                pub_t_ms = camera_timestamp[pub_t_key]
                                publish_latency_ms = frame_received_ms - pub_t_ms
                                self.get_logger().info(f"     * Publish latency (frame_received - pub_t): {publish_latency_ms} ms")
                            
                            if sub_t_key in camera_timestamp and end_t_key in camera_timestamp:
                                process_duration = camera_timestamp[end_t_key] - camera_timestamp[sub_t_key]
                                self.get_logger().info(f"     * Process duration: {process_duration} ms")
                else:
                    self.get_logger().warn("❌ Camera data not found in observation")
                
                self.get_logger().info(f"✅ Total observation latency: {total_latency_ms} ms")
                
            except Exception as e:
                self.get_logger().error(f"❌ Error getting observation: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
            
            # Sleep between iterations
            if i < num_iterations - 1:
                time.sleep(0.1)
        
        # Print statistics
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("LATENCY STATISTICS")
        self.get_logger().info("=" * 80)
        
        if latencies["total"]:
            total_avg = sum(latencies["total"]) / len(latencies["total"])
            total_min = min(latencies["total"])
            total_max = max(latencies["total"])
            self.get_logger().info(f"Total observation latency:")
            self.get_logger().info(f"  Average: {total_avg:.2f} ms")
            self.get_logger().info(f"  Min: {total_min} ms")
            self.get_logger().info(f"  Max: {total_max} ms")
        
        if latencies["robot_state"]:
            robot_avg = sum(latencies["robot_state"]) / len(latencies["robot_state"])
            robot_min = min(latencies["robot_state"])
            robot_max = max(latencies["robot_state"])
            self.get_logger().info(f"\nRobot state latency:")
            self.get_logger().info(f"  Average: {robot_avg:.2f} ms")
            self.get_logger().info(f"  Min: {robot_min} ms")
            self.get_logger().info(f"  Max: {robot_max} ms")
        
        if latencies["camera"]:
            camera_avg = sum(latencies["camera"]) / len(latencies["camera"])
            camera_min = min(latencies["camera"])
            camera_max = max(latencies["camera"])
            self.get_logger().info(f"\nCamera latency:")
            self.get_logger().info(f"  Average: {camera_avg:.2f} ms")
            self.get_logger().info(f"  Min: {camera_min} ms")
            self.get_logger().info(f"  Max: {camera_max} ms")
        
        self.get_logger().info("=" * 80)


def main():
    """Main function."""
    rclpy.init()
    
    try:
        node = ObservationTestNode()
        
        # Spin a bit to initialize
        rclpy.spin_once(node, timeout_sec=1.0)
        time.sleep(2.0)  # Wait for initial data
        
        # Run test
        node.test_get_observation(num_iterations=10)
        
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


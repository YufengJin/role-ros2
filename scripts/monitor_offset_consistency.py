#!/usr/bin/env python3
"""
VR Policy Offset Consistency Monitor

监控和验证 VR Policy 的 offset 计算逻辑：
理论上，如果机器人完全跟随 VR 控制器，那么：
- robot_pos_offset 应该等于 target_pos_offset (VR offset)
- pos_action 应该接近 0

这个脚本通过订阅 robot_state 和 action 话题来验证这个逻辑。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from role_ros2.msg import VRPolicyAction, PolymetisRobotState


class OffsetConsistencyMonitor(Node):
    def __init__(self):
        super().__init__('offset_consistency_monitor')
        
        # QoS profile
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # State storage
        self.last_robot_state = None
        self.last_action_msg = None
        self.robot_origin = None  # Will be set when movement_enabled changes from False to True
        self.target_origin = None  # Same as robot_origin at reset time
        self.last_movement_enabled = False
        
        # Statistics
        self.stats = {
            'total_samples': 0,
            'offset_diff_sum': 0.0,
            'offset_diff_max': 0.0,
            'action_magnitude_sum': 0.0,
            'action_magnitude_max': 0.0,
        }
        
        # Subscribers
        self.robot_state_sub = self.create_subscription(
            PolymetisRobotState,
            '/polymetis/robot_state',
            self.robot_state_callback,
            qos_profile_reliable
        )
        
        self.action_sub = self.create_subscription(
            VRPolicyAction,
            '/vr_policy/action',
            self.action_callback,
            qos_profile_reliable
        )
        
        # Timer for periodic display
        self.timer = self.create_timer(1.0, self.periodic_display)
        
        self.get_logger().info('Offset Consistency Monitor started')
        self.get_logger().info('Monitoring: /polymetis/robot_state and /vr_policy/action')
        self.get_logger().info('Press Ctrl+C to stop.')
    
    def robot_state_callback(self, msg: PolymetisRobotState):
        """Store latest robot state."""
        self.last_robot_state = msg
    
    def action_callback(self, msg: VRPolicyAction):
        """Process action message and verify offset consistency."""
        self.last_action_msg = msg
        
        # Detect origin reset (when movement_enabled changes from False to True)
        if not self.last_movement_enabled and msg.movement_enabled:
            # Origin reset detected - save current robot position as origin
            # Note: At origin reset, target_pos should equal robot_pos (both offsets are 0)
            if self.last_robot_state is not None:
                robot_pos = np.array([
                    self.last_robot_state.ee_position[0],
                    self.last_robot_state.ee_position[1],
                    self.last_robot_state.ee_position[2]
                ])
                target_pos = np.array(msg.target_cartesian_position[:3])
                
                # At origin reset, both should be the same (or very close)
                # Save robot position as origin
                self.robot_origin = robot_pos.copy()
                # Target origin is the same as robot origin at reset time
                self.target_origin = target_pos.copy()
                
                # Verify they are close (should be within a small threshold)
                origin_diff = np.linalg.norm(robot_pos - target_pos)
                if origin_diff > 0.01:  # 1cm threshold
                    self.get_logger().warn(
                        f"⚠️  Origin reset: robot_pos and target_pos differ by {origin_diff:.4f}m"
                    )
                
                self.get_logger().info(
                    f"🔄 Origin reset detected:\n"
                    f"   Robot origin: {self.robot_origin}\n"
                    f"   Target position at reset: {target_pos}\n"
                    f"   Difference: {origin_diff:.4f}m (should be close to 0)"
                )
                
                # Reset statistics
                self.stats = {
                    'total_samples': 0,
                    'offset_diff_sum': 0.0,
                    'offset_diff_max': 0.0,
                    'action_magnitude_sum': 0.0,
                    'action_magnitude_max': 0.0,
                }
        
        self.last_movement_enabled = msg.movement_enabled
        
        # Verify offset consistency if we have all needed data
        if (self.last_robot_state is not None and 
            msg.movement_enabled and 
            self.robot_origin is not None):
            
            self.verify_offset_consistency(msg)
    
    def verify_offset_consistency(self, action_msg: VRPolicyAction):
        """
        Verify that robot offset matches VR offset.
        
        Theory:
        - robot_pos_offset = robot_pos - robot_origin
        - target_pos_offset = vr_state - vr_origin (this is the VR offset)
        - From code: target_pos = robot_origin + target_pos_offset
        - So: target_pos_offset = target_pos - robot_origin
        - If robot follows VR perfectly: robot_pos_offset should equal target_pos_offset
        """
        # Get current robot position
        robot_pos = np.array([
            self.last_robot_state.ee_position[0],
            self.last_robot_state.ee_position[1],
            self.last_robot_state.ee_position[2]
        ])
        
        # Get target position (this is where robot should be)
        target_pos = np.array(action_msg.target_cartesian_position[:3])
        
        # Calculate offsets
        robot_pos_offset = robot_pos - self.robot_origin
        
        # Calculate target_pos_offset (VR offset)
        # From code logic: target_pos = robot_origin + target_pos_offset
        # So: target_pos_offset = target_pos - robot_origin
        target_pos_offset = target_pos - self.robot_origin
        
        # Calculate position action (what should be sent)
        # From code: target_pos = pos_action + robot_pos
        # So: pos_action = target_pos - robot_pos
        pos_action = target_pos - robot_pos
        
        # Also verify: pos_action should equal target_pos_offset - robot_pos_offset
        # From code: pos_action = target_pos_offset - robot_pos_offset
        expected_pos_action = target_pos_offset - robot_pos_offset
        
        # Also get action from message (this is after scaling and limiting)
        action_lin_vel = np.array(action_msg.action[:3])
        
        # Calculate differences
        offset_diff = target_pos_offset - robot_pos_offset
        offset_diff_norm = np.linalg.norm(offset_diff)
        
        # Verify pos_action calculation matches expected formula
        pos_action_diff = pos_action - expected_pos_action
        pos_action_diff_norm = np.linalg.norm(pos_action_diff)
        
        # Expected: if robot follows VR perfectly, offset_diff should be close to 0
        # Also, pos_action should equal expected_pos_action (within numerical precision)
        
        # Update statistics
        self.stats['total_samples'] += 1
        self.stats['offset_diff_sum'] += offset_diff_norm
        self.stats['offset_diff_max'] = max(self.stats['offset_diff_max'], offset_diff_norm)
        
        action_magnitude = np.linalg.norm(pos_action)
        self.stats['action_magnitude_sum'] += action_magnitude
        self.stats['action_magnitude_max'] = max(self.stats['action_magnitude_max'], action_magnitude)
        
        # Check for inconsistencies
        threshold = 0.01  # 1cm threshold
        if offset_diff_norm > threshold:
            self.get_logger().warn(
                f"⚠️  Offset inconsistency detected:\n"
                f"   Robot offset: {robot_pos_offset}\n"
                f"   Target offset (VR offset): {target_pos_offset}\n"
                f"   Difference: {offset_diff} (norm: {offset_diff_norm:.4f}m)\n"
                f"   Expected: offsets should be equal if robot follows VR\n"
                f"   pos_action: {pos_action}\n"
                f"   expected_pos_action: {expected_pos_action}\n"
                f"   pos_action_diff: {pos_action_diff} (norm: {pos_action_diff_norm:.6f}m)"
            )
    
    def periodic_display(self):
        """Periodically display current status and statistics."""
        print("\033[2J\033[H", end='')  # 清屏
        print("=" * 80)
        print("VR Policy Offset Consistency Monitor")
        print("=" * 80)
        
        if self.last_action_msg is None or self.last_robot_state is None:
            print("\n⏳ Waiting for data...")
            print("   Make sure both /polymetis/robot_state and /vr_policy/action are publishing")
            print("=" * 80)
            return
        
        # Current status
        print(f"\n📊 Current Status:")
        print(f"   Movement Enabled: {'✅ YES' if self.last_action_msg.movement_enabled else '❌ NO'}")
        print(f"   Controller On: {'✅ YES' if self.last_action_msg.controller_on else '❌ NO'}")
        
        if self.robot_origin is None:
            print(f"\n⏳ Waiting for origin reset (press and hold GRIP button)...")
            print("   Origin will be set when movement_enabled changes from False to True")
            print("=" * 80)
            return
        
        # Current positions and offsets
        robot_pos = np.array([
            self.last_robot_state.ee_position[0],
            self.last_robot_state.ee_position[1],
            self.last_robot_state.ee_position[2]
        ])
        target_pos = np.array(self.last_action_msg.target_cartesian_position[:3])
        
        robot_pos_offset = robot_pos - self.robot_origin
        target_pos_offset = target_pos - self.target_origin
        offset_diff = target_pos_offset - robot_pos_offset
        offset_diff_norm = np.linalg.norm(offset_diff)
        
        pos_action = target_pos - robot_pos
        action_magnitude = np.linalg.norm(pos_action)
        
        print(f"\n📍 Positions:")
        print(f"   Robot Origin:    [{self.robot_origin[0]:7.3f}, {self.robot_origin[1]:7.3f}, {self.robot_origin[2]:7.3f}]")
        print(f"   Target Origin:   [{self.target_origin[0]:7.3f}, {self.target_origin[1]:7.3f}, {self.target_origin[2]:7.3f}]")
        print(f"   Robot Current:   [{robot_pos[0]:7.3f}, {robot_pos[1]:7.3f}, {robot_pos[2]:7.3f}]")
        print(f"   Target Current:  [{target_pos[0]:7.3f}, {target_pos[1]:7.3f}, {target_pos[2]:7.3f}]")
        
        # Calculate expected pos_action for display
        expected_pos_action = target_pos_offset - robot_pos_offset
        
        print(f"\n📐 Offsets (relative to robot_origin):")
        print(f"   Robot Offset:      [{robot_pos_offset[0]:7.3f}, {robot_pos_offset[1]:7.3f}, {robot_pos_offset[2]:7.3f}] "
              f"(norm: {np.linalg.norm(robot_pos_offset):.4f}m)")
        print(f"   Target Offset:     [{target_pos_offset[0]:7.3f}, {target_pos_offset[1]:7.3f}, {target_pos_offset[2]:7.3f}] "
              f"(norm: {np.linalg.norm(target_pos_offset):.4f}m)")
        print(f"   Offset Difference: [{offset_diff[0]:7.3f}, {offset_diff[1]:7.3f}, {offset_diff[2]:7.3f}] "
              f"(norm: {offset_diff_norm:.4f}m)")
        print(f"   ⚠️  Expected: Robot offset should equal Target offset (VR offset)")
        
        print(f"\n⚡ Action:")
        print(f"   Position Action (target - robot): [{pos_action[0]:7.3f}, {pos_action[1]:7.3f}, {pos_action[2]:7.3f}] "
              f"(norm: {action_magnitude:.4f}m)")
        print(f"   Expected Action (target_offset - robot_offset): [{expected_pos_action[0]:7.3f}, {expected_pos_action[1]:7.3f}, {expected_pos_action[2]:7.3f}]")
        print(f"   Velocity Action: {self.last_action_msg.action[:3]} "
              f"(norm: {np.linalg.norm(self.last_action_msg.action[:3]):.4f})")
        
        # Verification
        print(f"\n✅ Verification:")
        threshold = 0.01  # 1cm
        if offset_diff_norm < threshold:
            print(f"   ✅ PASS: Offset difference ({offset_diff_norm:.4f}m) < threshold ({threshold}m)")
            print(f"      Robot offset matches target offset (robot is following VR correctly)")
        else:
            print(f"   ⚠️  WARNING: Offset difference ({offset_diff_norm:.4f}m) >= threshold ({threshold}m)")
            print(f"      Robot offset does NOT match target offset")
            print(f"      This may indicate:")
            print(f"         - Robot hasn't caught up with VR movement yet")
            print(f"         - Control gains are too low")
            print(f"         - Origin mismatch")
        
        if action_magnitude < threshold:
            print(f"   ✅ PASS: Position action ({action_magnitude:.4f}m) < threshold ({threshold}m)")
            print(f"      Robot is at target position (no action needed)")
        else:
            print(f"   ℹ️  INFO: Position action ({action_magnitude:.4f}m) >= threshold ({threshold}m)")
            print(f"      Robot needs to move to reach target")
        
        # Statistics
        if self.stats['total_samples'] > 0:
            avg_offset_diff = self.stats['offset_diff_sum'] / self.stats['total_samples']
            avg_action_magnitude = self.stats['action_magnitude_sum'] / self.stats['total_samples']
            
            print(f"\n📈 Statistics (since last origin reset):")
            print(f"   Total Samples: {self.stats['total_samples']}")
            print(f"   Avg Offset Diff: {avg_offset_diff:.4f}m")
            print(f"   Max Offset Diff: {self.stats['offset_diff_max']:.4f}m")
            print(f"   Avg Action Magnitude: {avg_action_magnitude:.4f}m")
            print(f"   Max Action Magnitude: {self.stats['action_magnitude_max']:.4f}m")
        
        print("\n" + "=" * 80)
        print("💡 Expected Behavior:")
        print("   - When VR controller moves, target_offset should change")
        print("   - Robot should follow, so robot_offset should match target_offset")
        print("   - If robot_offset == target_offset, then offset_diff ≈ 0")
        print("   - If offset_diff ≈ 0, then pos_action ≈ 0 (no movement needed)")
        print("=" * 80)


def main(args=None):
    rclpy.init(args=args)
    monitor = OffsetConsistencyMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n👋 Monitor stopped.")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


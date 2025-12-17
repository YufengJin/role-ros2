# ROBOT SPECIFIC IMPORTS
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import torch

# UTILITY IMPORTS
from role_ros2.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver
from role_ros2.robot_ik.arm import FrankaArm
from role_ros2.msg import (
    PolymetisGripperState,
    PolymetisRobotCommand,
    PolymetisRobotState,
)
from dm_control import mjcf
from scipy.spatial.transform import Rotation as R


class FrankaRobot(Node):
    """
    ROS2 Robot API - Provides high-level robot control interface
    Subscribes to /joint_states and publishes control commands to polymetis_manager
    """
    
    def __init__(self, arm_id: str = "fr3"):
        super().__init__('franka_robot_node')
        
        self.arm_id = arm_id
        self._ik_solver = RobotIKSolver()
        
        # Initialize robot model for forward kinematics
        self._arm = FrankaArm()
        self._physics = mjcf.Physics.from_mjcf_model(self._arm.mjcf_model)
        
        # Get joint names from parameter or use defaults
        self.declare_parameter('arm_joint_names', [f'{arm_id}_panda_joint{i+1}' for i in range(7)])
        self.declare_parameter('gripper_joint_names', [f'{arm_id}_finger_joint1', f'{arm_id}_finger_joint2'])
        
        self._arm_joint_names = self.get_parameter('arm_joint_names').get_parameter_value().string_array_value
        self._gripper_joint_names = self.get_parameter('gripper_joint_names').get_parameter_value().string_array_value
        
        # State storage (updated from /polymetis_manager/robot_state)
        self._joint_positions = np.zeros(7)
        self._joint_velocities = np.zeros(7)
        self._joint_efforts = np.zeros(7)
        self._gripper_width = 0.0
        self._gripper_position = 0.0
        self._max_gripper_width = 0.08  # franka hand default
        self._ee_position = np.zeros(3)
        self._ee_euler = np.zeros(3)
        self._ee_quaternion = np.array([0.0, 0.0, 0.0, 1.0])
        self._robot_state_dict = {}
        self._robot_timestamp = {}
        
        # Subscribers - subscribe to comprehensive robot state from polymetis_manager
        self._robot_state_sub = self.create_subscription(
            PolymetisRobotState, 'polymetis_manager/robot_state', self._robot_state_callback, 10
        )
        # Also subscribe to joint_states for compatibility
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        
        # Publishers - publish control commands to polymetis_manager
        self._command_pub = self.create_publisher(
            PolymetisRobotCommand, 'polymetis_manager/robot_command', 10
        )
        
        self.get_logger().info('FrankaRobot API initialized')
    
    def _robot_state_callback(self, msg: PolymetisRobotState):
        """Update robot state from /polymetis_manager/robot_state topic - primary source"""
        # Update joint states
        if len(msg.joint_positions) >= 7:
            self._joint_positions = np.array(msg.joint_positions[:7])
        if len(msg.joint_velocities) >= 7:
            self._joint_velocities = np.array(msg.joint_velocities[:7])
        if len(msg.joint_torques_computed) >= 7:
            self._joint_efforts = np.array(msg.joint_torques_computed[:7])
        
        # Update end-effector pose (pre-computed by polymetis_manager)
        if len(msg.ee_position) >= 3:
            self._ee_position = np.array(msg.ee_position[:3])
        if len(msg.ee_euler) >= 3:
            self._ee_euler = np.array(msg.ee_euler[:3])
        if len(msg.ee_quaternion) >= 4:
            self._ee_quaternion = np.array(msg.ee_quaternion[:4])
        
        # Update gripper state
        self._gripper_width = msg.gripper_width
        self._gripper_position = msg.gripper_position
        
        # Store complete robot state for get_robot_state()
        self._robot_state_dict = {
            "cartesian_position": (self._ee_position.tolist() + self._ee_euler.tolist()),
            "gripper_position": float(msg.gripper_position),
            "joint_positions": msg.joint_positions[:7] if len(msg.joint_positions) >= 7 else list(self._joint_positions),
            "joint_velocities": msg.joint_velocities[:7] if len(msg.joint_velocities) >= 7 else list(self._joint_velocities),
            "joint_torques_computed": msg.joint_torques_computed[:7] if len(msg.joint_torques_computed) >= 7 else list(self._joint_efforts),
            "prev_joint_torques_computed": msg.prev_joint_torques_computed[:7] if len(msg.prev_joint_torques_computed) >= 7 else list(self._joint_efforts),
            "prev_joint_torques_computed_safened": msg.prev_joint_torques_computed_safened[:7] if len(msg.prev_joint_torques_computed_safened) >= 7 else list(self._joint_efforts),
            "motor_torques_measured": msg.motor_torques_measured[:7] if len(msg.motor_torques_measured) >= 7 else list(self._joint_efforts),
            "prev_controller_latency_ms": msg.prev_controller_latency_ms,
            "prev_command_successful": msg.prev_command_successful,
        }
        
        try:
            ros_timestamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        except Exception:
            ros_timestamp_ns = int(self.get_clock().now().nanoseconds)

        self._robot_timestamp = {
            "ros_timestamp_ns": ros_timestamp_ns,
            "polymetis_timestamp_ns": int(msg.polymetis_timestamp_ns),
        }
    
    def _joint_state_callback(self, msg: JointState):
        """Update joint states from /joint_states topic - fallback/compatibility"""
        # Only update if robot_state hasn't been received yet
        if len(self._robot_state_dict) == 0:
            try:
                self._joint_state_stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            except Exception:
                self._joint_state_stamp_ns = int(self.get_clock().now().nanoseconds)
            # Update arm joint states
            for i, name in enumerate(self._arm_joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    if idx < len(msg.position):
                        self._joint_positions[i] = msg.position[idx]
                    if idx < len(msg.velocity):
                        self._joint_velocities[i] = msg.velocity[idx]
                    if idx < len(msg.effort):
                        self._joint_efforts[i] = msg.effort[idx]
            
            # Update gripper width
            for name in self._gripper_joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    if idx < len(msg.position):
                        # Gripper width is sum of both finger positions
                        self._gripper_width = msg.position[idx] * 2.0
                        break
    
    def launch_controller(self):
        """Not needed - controller is launched by polymetis_manager"""
        pass
    
    def launch_robot(self):
        """Not needed - robot is launched by polymetis_manager"""
        pass
    
    def kill_controller(self):
        """Not needed - controller is managed by polymetis_manager"""
        pass
    
    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        """Update robot command - publishes to polymetis_manager"""
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)
        
        # Publish command to polymetis_manager
        cmd_msg = PolymetisRobotCommand()
        now = self.get_clock().now()
        cmd_msg.header.stamp = now.to_msg()
        cmd_msg.header.frame_id = ''
        cmd_msg.action_space = action_space
        cmd_msg.gripper_action_space = gripper_action_space if gripper_action_space else ("velocity" if "velocity" in action_space else "position")
        cmd_msg.command = list(command)
        cmd_msg.blocking = blocking
        cmd_msg.velocity = "velocity" in action_space
        cmd_msg.cartesian_noise = []
        
        self._command_pub.publish(cmd_msg)
        
        return action_dict
    
    def update_pose(self, command, velocity=False, blocking=False):
        """Update end-effector pose"""
        action_space = "cartesian_velocity" if velocity else "cartesian_position"
        self.update_command(command, action_space=action_space, blocking=blocking)
    
    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """Update joint positions or velocities"""
        action_space = "joint_velocity" if velocity else "joint_position"
        
        # Publish command to polymetis_manager
        cmd_msg = PolymetisRobotCommand()
        now = self.get_clock().now()
        cmd_msg.header.stamp = now.to_msg()
        cmd_msg.header.frame_id = ''
        cmd_msg.action_space = action_space
        cmd_msg.gripper_action_space = "velocity" if velocity else "position"
        cmd_msg.command = list(command) + [self.get_gripper_position()]  # Add current gripper position
        cmd_msg.blocking = blocking
        cmd_msg.velocity = velocity
        cmd_msg.cartesian_noise = list(cartesian_noise) if cartesian_noise is not None else []
        
        self._command_pub.publish(cmd_msg)
    
    def update_gripper(self, command, velocity=True, blocking=False):
        """Update gripper position"""
        # Publish gripper command as part of joint command
        # For gripper-only commands, we still need to send arm command (current position)
        current_joints = self.get_joint_positions()
        full_command = list(current_joints) + [command]
        
        action_space = "joint_velocity" if velocity else "joint_position"
        cmd_msg = PolymetisRobotCommand()
        now = self.get_clock().now()
        cmd_msg.header.stamp = now.to_msg()
        cmd_msg.header.frame_id = ''
        cmd_msg.action_space = action_space
        cmd_msg.gripper_action_space = "velocity" if velocity else "position"
        cmd_msg.command = full_command
        cmd_msg.blocking = blocking
        cmd_msg.velocity = velocity
        cmd_msg.cartesian_noise = []
        
        self._command_pub.publish(cmd_msg)
    
    def add_noise_to_joints(self, original_joints, cartesian_noise):
        """Add noise to joints via cartesian space"""
        # Get current pose from forward kinematics (would need robot model)
        # For now, just return original joints
        # This should be implemented using robot model if needed
        return original_joints
    
    def get_joint_positions(self):
        """Get current joint positions"""
        return self._joint_positions.tolist()
    
    def get_joint_velocities(self):
        """Get current joint velocities"""
        return self._joint_velocities.tolist()
    
    def get_gripper_position(self):
        """Get current gripper position (normalized 0-1)"""
        # Use pre-computed normalized position from polymetis_manager
        if hasattr(self, '_gripper_position') and self._gripper_position is not None:
            return float(self._gripper_position)
        else:
            # Fallback calculation
            return 1 - (self._gripper_width / self._max_gripper_width)
    
    def get_gripper_state(self):
        """Alias for get_gripper_position() for compatibility"""
        return self.get_gripper_position()
    
    def get_ee_pose(self):
        """Get current end-effector pose [x, y, z, roll, pitch, yaw]"""
        # Use pre-computed pose from polymetis_manager (no need to compute FK)
        if len(self._ee_position) == 3 and len(self._ee_euler) == 3:
            return (self._ee_position.tolist() + self._ee_euler.tolist())
        else:
            # Fallback: compute from joint positions if not available
            try:
                qpos = np.array(self._joint_positions)
                qvel = np.zeros(7)  # Not needed for FK
                
                self._arm.update_state(self._physics, qpos, qvel)
                
                # Get wrist site pose (end-effector)
                wrist_site = self._arm.wrist_site
                site_xpos = self._physics.bind(wrist_site).xpos
                site_xmat = self._physics.bind(wrist_site).xmat.reshape(3, 3)
                
                # Convert rotation matrix to quaternion then to euler
                rot = R.from_matrix(site_xmat)
                euler = rot.as_euler('xyz')
                
                pos = site_xpos.tolist()
                return pos + euler.tolist()
            except Exception as e:
                self.get_logger().debug(f'Failed to compute forward kinematics: {e}')
                return [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # Default pose
    
    def get_robot_state(self):
        """Get complete robot state - uses pre-computed state from polymetis_manager"""
        # Use cached state from polymetis_manager if available (no computation needed)
        if len(self._robot_state_dict) > 0:
            return self._robot_state_dict.copy(), self._robot_timestamp.copy()
        
        # Fallback: construct state from individual components
        robot_state_dict = {
            "cartesian_position": self.get_ee_pose(),
            "gripper_position": self.get_gripper_position(),
            "joint_positions": self.get_joint_positions(),
            "joint_velocities": self.get_joint_velocities(),
            "joint_torques_computed": self._joint_efforts.tolist(),
            "prev_joint_torques_computed": self._joint_efforts.tolist(),
            "prev_joint_torques_computed_safened": self._joint_efforts.tolist(),
            "motor_torques_measured": self._joint_efforts.tolist(),
            "prev_controller_latency_ms": 0.0,
            "prev_command_successful": True,
        }
        
        # Construct timestamp_dict and handle polymetis timestamp if available
        timestamp_dict = {
            "ros_timestamp_ns": int(self.get_clock().now().nanoseconds),
        }
        # Attempt to add polymetis timestamp if available from _robot_timestamp
        if hasattr(self, '_robot_timestamp') and isinstance(self._robot_timestamp, dict) and "polymetis_timestamp_ns" in self._robot_timestamp:
            timestamp_dict["polymetis_timestamp_ns"] = self._robot_timestamp["polymetis_timestamp_ns"]
        else:
            timestamp_dict["polymetis_timestamp_ns"] = 0  # fallback if not available
        
        return robot_state_dict, timestamp_dict
    
    def adaptive_time_to_go(self, desired_joint_position, t_min=0, t_max=4):
        """Calculate adaptive time to go for joint movement"""
        curr_joint_position = np.array(self.get_joint_positions())
        desired_joint_position = np.array(desired_joint_position)
        displacement = np.abs(desired_joint_position - curr_joint_position)
        
        # Simple heuristic: max joint displacement / max velocity
        max_displacement = np.max(displacement)
        max_velocity = 1.0  # rad/s (approximate)
        time_to_go = max_displacement / max_velocity if max_velocity > 0 else t_min
        clamped_time_to_go = min(t_max, max(time_to_go, t_min))
        return clamped_time_to_go
    
    def create_action_dict(self, action, action_space="cartesian_velocity", gripper_action_space=None, robot_state=None):
        """Create action dictionary from action command"""
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        if robot_state is None:
            robot_state = self.get_robot_state()[0]
        action_dict = {"robot_state": robot_state}
        velocity = "velocity" in action_space
        
        if gripper_action_space is None:
            gripper_action_space = "velocity" if velocity else "position"
        assert gripper_action_space in ["velocity", "position"]
        
        if gripper_action_space == "velocity":
            action_dict["gripper_velocity"] = action[-1]
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(action[-1])
            gripper_position = robot_state["gripper_position"] + gripper_delta
            action_dict["gripper_position"] = float(np.clip(gripper_position, 0, 1))
        else:
            action_dict["gripper_position"] = float(np.clip(action[-1], 0, 1))
            gripper_delta = action_dict["gripper_position"] - robot_state["gripper_position"]
            gripper_velocity = self._ik_solver.gripper_delta_to_velocity(gripper_delta)
            action_dict["gripper_delta"] = gripper_velocity
        
        if "cartesian" in action_space:
            if velocity:
                action_dict["cartesian_velocity"] = action[:-1]
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(action[:-1])
                action_dict["cartesian_position"] = add_poses(
                    cartesian_delta, robot_state["cartesian_position"]
                ).tolist()
            else:
                action_dict["cartesian_position"] = action[:-1]
                cartesian_delta = pose_diff(action[:-1], robot_state["cartesian_position"])
                cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
                action_dict["cartesian_velocity"] = cartesian_velocity.tolist()
            
            action_dict["joint_velocity"] = self._ik_solver.cartesian_velocity_to_joint_velocity(
                action_dict["cartesian_velocity"], robot_state=robot_state
            ).tolist()
            joint_delta = self._ik_solver.joint_velocity_to_delta(action_dict["joint_velocity"])
            action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
        
        if "joint" in action_space:
            if velocity:
                action_dict["joint_velocity"] = action[:-1]
                joint_delta = self._ik_solver.joint_velocity_to_delta(action[:-1])
                action_dict["joint_position"] = (joint_delta + np.array(robot_state["joint_positions"])).tolist()
            else:
                action_dict["joint_position"] = action[:-1]
                joint_delta = np.array(action[:-1]) - np.array(robot_state["joint_positions"])
                joint_velocity = self._ik_solver.joint_delta_to_velocity(joint_delta)
                action_dict["joint_velocity"] = joint_velocity.tolist()
        
        return action_dict

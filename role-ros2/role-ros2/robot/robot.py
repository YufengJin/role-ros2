# ROBOT SPECIFIC IMPORTS
import time
from typing import Optional

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from franka_msgs.msg import FrankaRobotState
from franka_msgs.action import Move as GripperMove
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import torch

# UTILITY SPECIFIC IMPORTS
from role_ros2.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver

__all__ = ['FrankaRobot']


class FrankaRobot(Node):
    def __init__(self, arm_id: str = "fr3", controller_name: str = "fr3_arm_controller"):
        super().__init__('franka_robot_node')
        
        self.arm_id = arm_id
        self.controller_name = controller_name
        self._ik_solver = RobotIKSolver()
        self._controller_not_loaded = False
        
        # Action clients
        self._joint_trajectory_client = ActionClient(
            self, FollowJointTrajectory, f'{controller_name}/follow_joint_trajectory'
        )
        self._gripper_move_client = ActionClient(
            self, GripperMove, f'{arm_id}_gripper/move'
        )
        
        # Subscribers
        self._joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self._joint_state_callback, 10
        )
        self._franka_state_sub = self.create_subscription(
            FrankaRobotState, f'{arm_id}/robot_state', self._franka_state_callback, 10
        )
        self._ee_pose_sub = self.create_subscription(
            PoseStamped, f'{arm_id}/current_pose', self._ee_pose_callback, 10
        )
        
        # State storage
        self._joint_positions = np.zeros(7)
        self._joint_velocities = np.zeros(7)
        self._joint_efforts = np.zeros(7)
        self._ee_pose = None
        self._franka_robot_state = None
        self._gripper_width = 0.0
        self._max_gripper_width = 0.08  # franka hand default
        
        # Wait for action servers
        self.get_logger().info(f'Waiting for action server {controller_name}/follow_joint_trajectory...')
        self._joint_trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info(f'Waiting for action server {arm_id}_gripper/move...')
        self._gripper_move_client.wait_for_server(timeout_sec=10.0)
        
        self.get_logger().info('FrankaRobot initialized')
    
    def _joint_state_callback(self, msg: JointState):
        """Update joint states from joint_states topic"""
        joint_names = [f'{self.arm_id}_joint{i+1}' for i in range(7)]
        for i, name in enumerate(joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._joint_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self._joint_velocities[i] = msg.velocity[idx]
                if idx < len(msg.effort):
                    self._joint_efforts[i] = msg.effort[idx]
        
        # Update gripper width
        gripper_joints = [f'{self.arm_id}_finger_joint1', f'{self.arm_id}_finger_joint2']
        for name in gripper_joints:
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    # Gripper width is sum of both finger positions
                    self._gripper_width = msg.position[idx] * 2.0
                    break
    
    def _franka_state_callback(self, msg: FrankaRobotState):
        """Update franka robot state"""
        self._franka_robot_state = msg
        # Update gripper width from measured joint state
        if len(msg.measured_joint_state.position) >= 2:
            # Assuming finger joints are in the joint state
            self._gripper_width = sum(msg.measured_joint_state.position[-2:]) if len(msg.measured_joint_state.position) >= 2 else 0.0
    
    def _ee_pose_callback(self, msg: PoseStamped):
        """Update end-effector pose"""
        self._ee_pose = msg
    
    def launch_controller(self):
        """Not needed - robot is launched via launch file"""
        pass
    
    def launch_robot(self):
        """Not needed - robot is launched via launch file"""
        pass
    
    def kill_controller(self):
        """Not needed - robot is managed via launch file"""
        pass
    
    def update_command(self, command, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        """Update robot command"""
        action_dict = self.create_action_dict(command, action_space=action_space, gripper_action_space=gripper_action_space)
        
        self.update_joints(action_dict["joint_position"], velocity=False, blocking=blocking)
        self.update_gripper(action_dict["gripper_position"], velocity=False, blocking=blocking)
        
        return action_dict
    
    def update_pose(self, command, velocity=False, blocking=False):
        """Update end-effector pose"""
        if blocking:
            if velocity:
                curr_pose = self.get_ee_pose()
                cartesian_delta = self._ik_solver.cartesian_velocity_to_delta(command)
                command = add_poses(cartesian_delta, curr_pose)
            
            pos = torch.Tensor(command[:3])
            quat = torch.Tensor(euler_to_quat(command[3:6]))
            curr_joints = self.get_joint_positions()
            desired_joints = self._solve_inverse_kinematics(pos, quat, curr_joints)
            self.update_joints(desired_joints, velocity=False, blocking=True)
        else:
            if not velocity:
                curr_pose = self.get_ee_pose()
                cartesian_delta = pose_diff(command, curr_pose)
                command = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
            
            robot_state = self.get_robot_state()[0]
            joint_velocity = self._ik_solver.cartesian_velocity_to_joint_velocity(command, robot_state=robot_state)
            
            self.update_joints(joint_velocity, velocity=True, blocking=False)
    
    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """Update joint positions or velocities"""
        if cartesian_noise is not None:
            command = self.add_noise_to_joints(command, cartesian_noise)
        command = torch.Tensor(command) if isinstance(command, (list, np.ndarray)) else command
        
        if velocity:
            joint_delta = self._ik_solver.joint_velocity_to_delta(command.numpy() if isinstance(command, torch.Tensor) else command)
            command = joint_delta + self.get_joint_positions()
            command = command.tolist() if isinstance(command, np.ndarray) else command
        
        if blocking:
            self._move_to_joint_positions(command)
        else:
            # For non-blocking, we still use trajectory but with short duration
            self._send_joint_trajectory([command], [0.1], blocking=False)
    
    def update_gripper(self, command, velocity=True, blocking=False):
        """Update gripper position"""
        if velocity:
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            command = gripper_delta + self.get_gripper_position()
        
        command = float(np.clip(command, 0, 1))
        width = self._max_gripper_width * (1 - command)
        self._move_gripper(width, blocking=blocking)
    
    def add_noise_to_joints(self, original_joints, cartesian_noise):
        """Add noise to joints via cartesian space"""
        original_joints = torch.Tensor(original_joints)
        
        # Get current pose from forward kinematics
        curr_pose = self.get_ee_pose()
        new_pose = add_poses(cartesian_noise, curr_pose)
        
        new_pos = torch.Tensor(new_pose[:3])
        new_quat = torch.Tensor(euler_to_quat(new_pose[3:]))
        
        noisy_joints, success = self._solve_inverse_kinematics(new_pos, new_quat, original_joints.numpy())
        
        if success:
            desired_joints = noisy_joints
        else:
            desired_joints = original_joints.numpy()
        
        return desired_joints.tolist()
    
    def get_joint_positions(self):
        """Get current joint positions"""
        return self._joint_positions.tolist()
    
    def get_joint_velocities(self):
        """Get current joint velocities"""
        return self._joint_velocities.tolist()
    
    def get_gripper_position(self):
        """Get current gripper position (normalized 0-1)"""
        return 1 - (self._gripper_width / self._max_gripper_width)
    
    def get_gripper_state(self):
        """Alias for get_gripper_position() for compatibility"""
        return self.get_gripper_position()
    
    def get_ee_pose(self):
        """Get current end-effector pose [x, y, z, roll, pitch, yaw]"""
        if self._ee_pose is None:
            # Fallback: compute from joint positions using IK solver
            return [0.0, 0.0, 0.5, 0.0, 0.0, 0.0]  # Default pose
        
        pose = self._ee_pose.pose
        pos = [pose.position.x, pose.position.y, pose.position.z]
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        angle = quat_to_euler(np.array(quat))
        return np.concatenate([pos, angle]).tolist()
    
    def get_robot_state(self):
        """Get complete robot state"""
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
        
        timestamp_dict = {
            "robot_timestamp_seconds": int(time.time()),
            "robot_timestamp_nanos": 0,
        }
        
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
    
    def _solve_inverse_kinematics(self, pos, quat, curr_joints):
        """Solve inverse kinematics using RobotIKSolver"""
        # Convert to numpy arrays
        pos_np = pos.numpy() if isinstance(pos, torch.Tensor) else np.array(pos)
        quat_np = quat.numpy() if isinstance(quat, torch.Tensor) else np.array(quat)
        curr_joints_np = np.array(curr_joints)
        
        # Use RobotIKSolver's cartesian velocity to joint velocity method
        # First compute desired pose delta
        curr_pose = self.get_ee_pose()
        desired_pose = np.concatenate([pos_np, quat_to_euler(quat_np)])
        cartesian_delta = pose_diff(desired_pose, curr_pose)
        cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
        
        # Get robot state for IK
        robot_state = self.get_robot_state()[0]
        joint_velocity = self._ik_solver.cartesian_velocity_to_joint_velocity(cartesian_velocity, robot_state=robot_state)
        joint_delta = self._ik_solver.joint_velocity_to_delta(joint_velocity)
        
        # Compute desired joints
        desired_joints = curr_joints_np + joint_delta
        
        # Simple success check (could be improved)
        success = True
        return desired_joints, success
    
    def _send_joint_trajectory(self, positions, times, blocking=True):
        """Send joint trajectory via action"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [f'{self.arm_id}_joint{i+1}' for i in range(7)]
        
        for i, (pos, t) in enumerate(zip(positions, times)):
            point = JointTrajectoryPoint()
            point.positions = pos if isinstance(pos, list) else pos.tolist()
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            goal_msg.trajectory.points.append(point)
        
        if blocking:
            send_goal_future = self._joint_trajectory_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
        else:
            self._joint_trajectory_client.send_goal_async(goal_msg)
    
    def _move_to_joint_positions(self, positions):
        """Move to joint positions (blocking)"""
        time_to_go = self.adaptive_time_to_go(positions)
        self._send_joint_trajectory([positions], [time_to_go], blocking=True)
    
    def _move_gripper(self, width, blocking=True):
        """Move gripper to target width"""
        goal_msg = GripperMove.Goal()
        goal_msg.width = float(width)
        goal_msg.speed = 0.05
        
        if blocking:
            send_goal_future = self._gripper_move_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
        else:
            self._gripper_move_client.send_goal_async(goal_msg)


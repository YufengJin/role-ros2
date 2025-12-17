#!/usr/bin/env python3
"""
Polymetis Manager - ROS2 node for controlling Franka robot using Polymetis
Acts as communication layer: publishes joint_states and subscribes to control commands
"""

import os
import time
import yaml
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import grpc
import torch
from polymetis import GripperInterface, RobotInterface

from role_ros2.misc.transformations import add_poses, euler_to_quat, pose_diff, quat_to_euler
from role_ros2.misc.urdf_utils import get_all_joint_names
from role_ros2.misc.subprocess_utils import run_terminal_command, run_threaded_command
from role_ros2.misc.parameters import sudo_password
from role_ros2.msg import (
    PolymetisGripperState,
    PolymetisRobotCommand,
    PolymetisRobotState,
)
from role_ros2.robot_ik.robot_ik_solver import RobotIKSolver


class PolymetisManager(Node):
    """ROS2 node that manages Franka robot control via Polymetis - Communication layer only"""
    
    def __init__(self, arm_id: str = "fr3", ip_address: str = "localhost"):
        super().__init__('polymetis_manager_node')
        
        # Declare parameters
        self.declare_parameter('arm_id', arm_id)
        self.declare_parameter('ip_address', ip_address)
        self.declare_parameter('urdf_file', '')
        self.declare_parameter('launch_controller', True)
        self.declare_parameter('launch_robot', True)
        
        # Get parameters
        self.arm_id = self.get_parameter('arm_id').get_parameter_value().string_value
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        urdf_file_param = self.get_parameter('urdf_file').get_parameter_value().string_value
        launch_controller = self.get_parameter('launch_controller').get_parameter_value().bool_value
        launch_robot = self.get_parameter('launch_robot').get_parameter_value().bool_value
        
        # Get URDF file path
        if not urdf_file_param:
            # Try to find URDF file
            from ament_index_python.packages import get_package_share_directory
            package_share_dir = get_package_share_directory('role_ros2')
            src_dir = os.path.join(package_share_dir, '..', '..', 'src', 'role_ros2')
            urdf_file = os.path.join(src_dir, 'role_ros2', 'robot_ik', 'franka', f'{self.arm_id}.urdf')
        else:
            urdf_file = urdf_file_param
        
        # Load joint names from URDF
        try:
            self._arm_joint_names = get_all_joint_names(urdf_file, self.arm_id, include_gripper=False)
            self._gripper_joint_names = [f'{self.arm_id}_finger_joint1', f'{self.arm_id}_finger_joint2']
            self._all_joint_names = self._arm_joint_names + self._gripper_joint_names
            self.get_logger().info(f'Loaded {len(self._arm_joint_names)} arm joints and {len(self._gripper_joint_names)} gripper joints from URDF')
        except Exception as e:
            self.get_logger().warn(f'Failed to load joint names from URDF: {e}, using defaults')
            self._arm_joint_names = [f'{self.arm_id}_panda_joint{i+1}' for i in range(7)]
            self._gripper_joint_names = [f'{self.arm_id}_finger_joint1', f'{self.arm_id}_finger_joint2']
            self._all_joint_names = self._arm_joint_names + self._gripper_joint_names
        
        self._ik_solver = RobotIKSolver()
        self._controller_not_loaded = False
        
        # Launch Polymetis controller and robot if requested
        if launch_controller:
            self.launch_controller()
        
        if launch_robot:
            self.launch_robot()
        
        # State storage
        self._joint_positions = np.zeros(7)
        self._joint_velocities = np.zeros(7)
        self._joint_efforts = np.zeros(7)
        self._gripper_width = 0.0
        
        # Publishers - publish to /polymetis_manager/ namespace
        self._joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self._robot_state_pub = self.create_publisher(
            PolymetisRobotState, '~/robot_state', 10
        )
        self._gripper_state_pub = self.create_publisher(
            PolymetisGripperState, '~/gripper_state', 10
        )
        
        # Subscribers for control commands
        self._command_sub = self.create_subscription(
            PolymetisRobotCommand, '~/robot_command', self._command_callback, 10
        )
        
        # Timer for publishing states at high frequency
        self._publish_rate = 60.0  # Hz - high frequency for real-time control
        self._publish_timer = self.create_timer(
            1.0 / self._publish_rate, self._publish_all_states
        )
        
        # Start cartesian impedance controller for non-blocking control
        self._start_cartesian_impedance()
        
        self.get_logger().info('PolymetisManager initialized')
    
    def launch_controller(self):
        """Launch Polymetis controller (robot and gripper servers)"""
        try:
            self.kill_controller()
        except:
            pass
        
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # Find launch scripts (should be in role_ros2/scripts)
        launch_robot_script = os.path.join(dir_path, 'launch_robot.sh')
        launch_gripper_script = os.path.join(dir_path, 'launch_gripper.sh')
        
        # If scripts don't exist, try to find in droid/franka
        if not os.path.exists(launch_robot_script):
            launch_robot_script = os.path.join(dir_path, '..', '..', '..', 'droid', 'franka', 'launch_robot.sh')
        if not os.path.exists(launch_gripper_script):
            launch_gripper_script = os.path.join(dir_path, '..', '..', '..', 'droid', 'franka', 'launch_gripper.sh')
        
        if os.path.exists(launch_robot_script):
            self._robot_process = run_terminal_command(
                f"echo {sudo_password} | sudo -S bash {launch_robot_script}"
            )
        else:
            self.get_logger().warn(f'Launch robot script not found: {launch_robot_script}')
            self._robot_process = None
        
        if os.path.exists(launch_gripper_script):
            self._gripper_process = run_terminal_command(
                f"echo {sudo_password} | sudo -S bash {launch_gripper_script}"
            )
        else:
            self.get_logger().warn(f'Launch gripper script not found: {launch_gripper_script}')
            self._gripper_process = None
        
        self._server_launched = True
        time.sleep(5)  # Wait for servers to start
        self.get_logger().info('Polymetis controller launched')
    
    def launch_robot(self):
        """Connect to Polymetis robot and gripper interfaces"""
        self.get_logger().info(f'Connecting to Polymetis server at {self.ip_address}...')
        try:
            self._robot = RobotInterface(ip_address=self.ip_address)
            self._gripper = GripperInterface(ip_address=self.ip_address)
            try:
                self._max_gripper_width = self._gripper.metadata.max_width
            except AttributeError:
                self._max_gripper_width = 0.08  # franka hand default
            self.get_logger().info('Successfully connected to Polymetis server')
        except Exception as e:
            self.get_logger().fatal(f'Failed to connect to Polymetis server: {e}')
            raise
    
    def kill_controller(self):
        """Kill Polymetis controller processes"""
        if hasattr(self, '_robot_process') and self._robot_process:
            self._robot_process.kill()
        if hasattr(self, '_gripper_process') and self._gripper_process:
            self._gripper_process.kill()
    
    def _start_cartesian_impedance(self):
        """Start cartesian impedance controller for non-blocking control"""
        try:
            if not self._robot.is_running_policy():
                self._robot.start_cartesian_impedance()
                timeout = time.time() + 5
                while not self._robot.is_running_policy():
                    time.sleep(0.01)
                    if time.time() > timeout:
                        self._robot.start_cartesian_impedance()
                        timeout = time.time() + 5
                self._controller_not_loaded = False
                self.get_logger().info('Cartesian impedance controller started')
        except Exception as e:
            self.get_logger().warn(f'Failed to start cartesian impedance: {e}')
    
    def _publish_all_states(self):
        """Publish all robot and gripper states from Polymetis at high frequency"""
        try:
            # Get robot state from Polymetis
            robot_state = self._robot.get_robot_state()
            
            # Update arm joint states
            joint_positions = list(robot_state.joint_positions)
            joint_velocities = list(robot_state.joint_velocities)
            joint_torques_computed = list(robot_state.joint_torques_computed)
            prev_joint_torques_computed = list(robot_state.prev_joint_torques_computed)
            prev_joint_torques_computed_safened = list(robot_state.prev_joint_torques_computed_safened)
            motor_torques_measured = list(robot_state.motor_torques_measured)
            
            self._joint_positions = np.array(joint_positions)
            self._joint_velocities = np.array(joint_velocities)
            self._joint_efforts = np.array(joint_torques_computed)
            
            # Get gripper state
            try:
                gripper_state_polymetis = self._gripper.get_state()
                self._gripper_width = gripper_state_polymetis.width
                gripper_is_grasped = getattr(gripper_state_polymetis, 'is_grasped', False)
                gripper_is_moving = getattr(gripper_state_polymetis, 'is_moving', False)
                gripper_prev_command_successful = getattr(gripper_state_polymetis, 'prev_command_successful', True)
                gripper_error_code = getattr(gripper_state_polymetis, 'error_code', 0)
                gripper_timestamp = getattr(gripper_state_polymetis, 'timestamp', None)
            except Exception as e:
                self.get_logger().debug(f'Failed to get gripper state: {e}')
                self._gripper_width = 0.0
                gripper_is_grasped = False
                gripper_is_moving = False
                gripper_prev_command_successful = True
                gripper_error_code = 0
                gripper_timestamp = None
            
            # Compute end-effector pose using forward kinematics
            try:
                if hasattr(self._robot, 'robot_model') and self._robot.robot_model is not None:
                    pos, quat = self._robot.robot_model.forward_kinematics(torch.Tensor(joint_positions))
                    ee_position = pos.tolist()
                    ee_quaternion = quat.tolist()  # [x, y, z, w]
                    ee_euler = quat_to_euler(quat.numpy()).tolist()
                else:
                    # Fallback: use get_ee_pose if robot_model not available
                    pos, quat = self._robot.get_ee_pose()
                    ee_position = pos.tolist()
                    ee_quaternion = quat.tolist()
                    ee_euler = quat_to_euler(quat.numpy()).tolist()
            except Exception as e:
                self.get_logger().debug(f'Failed to compute EE pose: {e}')
                ee_position = [0.0, 0.0, 0.0]
                ee_quaternion = [0.0, 0.0, 0.0, 1.0]
                ee_euler = [0.0, 0.0, 0.0]
            
            # Normalized gripper position (0=closed, 1=open)
            gripper_position_normalized = 1 - (self._gripper_width / self._max_gripper_width)
            
            # Check if policy is running
            try:
                is_running_policy = self._robot.is_running_policy()
            except Exception:
                is_running_policy = False
            
            # Get current time (ROS clock)
            now = self.get_clock().now()
            current_time = now.to_msg()
            
            # Publish JointState (for robot_state_publisher and compatibility)
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = current_time
            joint_state_msg.header.frame_id = ''
            joint_state_msg.name = self._arm_joint_names.copy()
            joint_state_msg.position = joint_positions
            joint_state_msg.velocity = joint_velocities
            joint_state_msg.effort = joint_torques_computed
            
            # Add gripper joints (two fingers, each with half the width)
            gripper_pos = self._gripper_width / 2.0
            joint_state_msg.name.extend(self._gripper_joint_names)
            joint_state_msg.position.extend([gripper_pos, gripper_pos])
            joint_state_msg.velocity.extend([0.0, 0.0])
            joint_state_msg.effort.extend([0.0, 0.0])
            
            self._joint_state_pub.publish(joint_state_msg)
            
            # Publish PolymetisRobotState (comprehensive robot information)
            robot_state_msg = PolymetisRobotState()
            robot_state_msg.header.stamp = current_time
            robot_state_msg.header.frame_id = f'{self.arm_id}_panda_link0'
            
            robot_state_msg.joint_positions = joint_positions
            robot_state_msg.joint_velocities = joint_velocities
            robot_state_msg.joint_torques_computed = joint_torques_computed
            robot_state_msg.prev_joint_torques_computed = prev_joint_torques_computed
            robot_state_msg.prev_joint_torques_computed_safened = prev_joint_torques_computed_safened
            robot_state_msg.motor_torques_measured = motor_torques_measured
            
            robot_state_msg.ee_position = ee_position
            robot_state_msg.ee_quaternion = ee_quaternion
            robot_state_msg.ee_euler = ee_euler
            
            robot_state_msg.gripper_width = self._gripper_width
            robot_state_msg.gripper_position = gripper_position_normalized
            robot_state_msg.gripper_is_grasped = gripper_is_grasped
            robot_state_msg.gripper_is_moving = gripper_is_moving
            robot_state_msg.gripper_prev_command_successful = gripper_prev_command_successful
            robot_state_msg.gripper_error_code = gripper_error_code
            
            robot_state_msg.prev_controller_latency_ms = robot_state.prev_controller_latency_ms
            robot_state_msg.prev_command_successful = robot_state.prev_command_successful
            robot_state_msg.is_running_policy = is_running_policy
            
            robot_state_msg.polymetis_timestamp_ns = int(robot_state.timestamp.seconds) * 1_000_000_000 + int(robot_state.timestamp.nanos)
            
            self._robot_state_pub.publish(robot_state_msg)
            
            # Publish PolymetisGripperState (detailed gripper information)
            gripper_state_msg = PolymetisGripperState()
            gripper_state_msg.header.stamp = current_time
            gripper_state_msg.header.frame_id = f'{self.arm_id}_panda_hand'
            
            gripper_state_msg.width = self._gripper_width
            gripper_state_msg.position = gripper_position_normalized
            gripper_state_msg.is_grasped = gripper_is_grasped
            gripper_state_msg.is_moving = gripper_is_moving
            gripper_state_msg.prev_command_successful = gripper_prev_command_successful
            gripper_state_msg.error_code = gripper_error_code
            gripper_state_msg.max_width = self._max_gripper_width
            
            if gripper_timestamp:
                gripper_state_msg.polymetis_timestamp_ns = int(gripper_timestamp.seconds) * 1_000_000_000 + int(gripper_timestamp.nanos)
            else:
                gripper_state_msg.polymetis_timestamp_ns = 0
            
            self._gripper_state_pub.publish(gripper_state_msg)
            
        except grpc.RpcError as e:
            self.get_logger().debug(f'RPC error getting robot state: {e}')
        except Exception as e:
            self.get_logger().warn(f'Error publishing states: {e}')
    
    def _command_callback(self, msg: PolymetisRobotCommand):
        """Handle robot control commands from topics"""
        try:
            action_space = msg.action_space
            gripper_action_space = msg.gripper_action_space if msg.gripper_action_space else None
            command = list(msg.command)
            blocking = msg.blocking
            velocity = msg.velocity if hasattr(msg, 'velocity') else "velocity" in action_space
            cartesian_noise = list(msg.cartesian_noise) if len(msg.cartesian_noise) > 0 else None
            
            # Process command based on action space
            if action_space == "cartesian_position":
                self._handle_cartesian_position_command(command, blocking, cartesian_noise)
            elif action_space == "cartesian_velocity":
                self._handle_cartesian_velocity_command(command, blocking, cartesian_noise)
            elif action_space == "joint_position":
                self._handle_joint_position_command(command, blocking, cartesian_noise)
            elif action_space == "joint_velocity":
                self._handle_joint_velocity_command(command, blocking, cartesian_noise)
            else:
                self.get_logger().warn(f'Unknown action space: {action_space}')
            
            # Handle gripper command
            if len(command) > 6:  # Has gripper command
                gripper_cmd = command[-1]
                gripper_velocity = (gripper_action_space == "velocity") if gripper_action_space else velocity
                self.update_gripper(gripper_cmd, velocity=gripper_velocity, blocking=blocking)
                
        except Exception as e:
            self.get_logger().error(f'Error processing command: {e}')
    
    def _handle_cartesian_position_command(self, command, blocking, cartesian_noise):
        """Handle cartesian position command"""
        if blocking:
            pos = torch.Tensor(command[:3])
            quat = torch.Tensor(euler_to_quat(command[3:6]))
            curr_joints = self._robot.get_joint_positions()
            desired_joints, success = self._robot.solve_inverse_kinematics(pos, quat, curr_joints)
            if success:
                self.update_joints(desired_joints, velocity=False, blocking=True, cartesian_noise=cartesian_noise)
        else:
            # Convert to velocity for non-blocking
            try:
                curr_pose = self.get_ee_pose()
                cartesian_delta = pose_diff(command[:6], curr_pose)
                cartesian_velocity = self._ik_solver.cartesian_delta_to_velocity(cartesian_delta)
                self._handle_cartesian_velocity_command(cartesian_velocity, blocking, cartesian_noise)
            except Exception as e:
                self.get_logger().warn(f'Failed to compute current pose: {e}, using blocking mode')
                # Fallback to blocking mode
                self._handle_cartesian_position_command(command, blocking=True, cartesian_noise=cartesian_noise)
    
    def _handle_cartesian_velocity_command(self, command, blocking, cartesian_noise):
        """Handle cartesian velocity command"""
        robot_state = self.get_robot_state()[0]
        joint_velocity = self._ik_solver.cartesian_velocity_to_joint_velocity(command[:6], robot_state=robot_state)
        self.update_joints(joint_velocity, velocity=True, blocking=blocking, cartesian_noise=cartesian_noise)
    
    def _handle_joint_position_command(self, command, blocking, cartesian_noise):
        """Handle joint position command"""
        self.update_joints(command[:7], velocity=False, blocking=blocking, cartesian_noise=cartesian_noise)
    
    def _handle_joint_velocity_command(self, command, blocking, cartesian_noise):
        """Handle joint velocity command"""
        self.update_joints(command[:7], velocity=True, blocking=blocking, cartesian_noise=cartesian_noise)
    
    def update_joints(self, command, velocity=False, blocking=False, cartesian_noise=None):
        """Update joint positions/velocities"""
        if cartesian_noise is not None:
            command = self.add_noise_to_joints(command, cartesian_noise)
        command = torch.Tensor(command)
        
        if velocity:
            joint_delta = self._ik_solver.joint_velocity_to_delta(command)
            command = joint_delta + self._robot.get_joint_positions()
        
        def helper_non_blocking():
            if not self._robot.is_running_policy():
                self._controller_not_loaded = True
                self._robot.start_cartesian_impedance()
                timeout = time.time() + 5
                while not self._robot.is_running_policy():
                    time.sleep(0.01)
                    if time.time() > timeout:
                        self._robot.start_cartesian_impedance()
                        timeout = time.time() + 5
                self._controller_not_loaded = False
            try:
                self._robot.update_desired_joint_positions(command)
            except grpc.RpcError:
                pass
        
        if blocking:
            if self._robot.is_running_policy():
                self._robot.terminate_current_policy()
            try:
                time_to_go = self.adaptive_time_to_go(command)
                self._robot.move_to_joint_positions(command, time_to_go=time_to_go)
            except grpc.RpcError:
                pass
            self._robot.start_cartesian_impedance()
        else:
            if not self._controller_not_loaded:
                run_threaded_command(helper_non_blocking)
    
    def update_gripper(self, command, velocity=True, blocking=False):
        """Update gripper position"""
        if velocity:
            gripper_delta = self._ik_solver.gripper_velocity_to_delta(command)
            command = gripper_delta + self.get_gripper_position()
        
        command = float(np.clip(command, 0, 1))
        self._gripper.goto(
            width=self._max_gripper_width * (1 - command),
            speed=0.05,
            force=0.1,
            blocking=blocking
        )
    
    def add_noise_to_joints(self, original_joints, cartesian_noise):
        """Add cartesian noise to joints"""
        original_joints = torch.Tensor(original_joints)
        
        pos, quat = self._robot.robot_model.forward_kinematics(original_joints)
        curr_pose = pos.tolist() + quat_to_euler(quat).tolist()
        new_pose = add_poses(cartesian_noise, curr_pose)
        
        new_pos = torch.Tensor(new_pose[:3])
        new_quat = torch.Tensor(euler_to_quat(new_pose[3:]))
        
        noisy_joints, success = self._robot.solve_inverse_kinematics(
            new_pos, new_quat, original_joints
        )
        
        if success:
            desired_joints = noisy_joints
        else:
            desired_joints = original_joints
        
        return desired_joints.tolist()
    
    def get_ee_pose(self):
        """Get current end-effector pose"""
        pos, quat = self._robot.get_ee_pose()
        angle = quat_to_euler(quat.numpy())
        return np.concatenate([pos, angle]).tolist()
    
    def get_gripper_position(self):
        """Get current gripper position (0-1)"""
        return 1 - (self._gripper.get_state().width / self._max_gripper_width)
    
    def get_robot_state(self):
        """Get complete robot state (timestamps use ROS ns int)"""
        robot_state = self._robot.get_robot_state()
        gripper_position = self.get_gripper_position()
        pos, quat = self._robot.robot_model.forward_kinematics(
            torch.Tensor(robot_state.joint_positions)
        )
        cartesian_position = pos.tolist() + quat_to_euler(quat.numpy()).tolist()
        
        state_dict = {
            "cartesian_position": cartesian_position,
            "gripper_position": gripper_position,
            "joint_positions": list(robot_state.joint_positions),
            "joint_velocities": list(robot_state.joint_velocities),
            "joint_torques_computed": list(robot_state.joint_torques_computed),
            "prev_joint_torques_computed": list(robot_state.prev_joint_torques_computed),
            "prev_joint_torques_computed_safened": list(robot_state.prev_joint_torques_computed_safened),
            "motor_torques_measured": list(robot_state.motor_torques_measured),
            "prev_controller_latency_ms": robot_state.prev_controller_latency_ms,
            "prev_command_successful": robot_state.prev_command_successful,
        }

        now_ns = int(self.get_clock().now().nanoseconds)
        timestamp_dict = {
            "ros_timestamp_ns": now_ns,
            "polymetis_timestamp_ns": int(robot_state.timestamp.seconds) * 1_000_000_000 + int(robot_state.timestamp.nanos),
        }
        
        return state_dict, timestamp_dict
    
    def adaptive_time_to_go(self, desired_joint_position, t_min=0, t_max=4):
        """Calculate adaptive time to go for joint movement"""
        curr_joint_position = self._robot.get_joint_positions()
        displacement = desired_joint_position - curr_joint_position
        time_to_go = self._robot._adaptive_time_to_go(displacement)
        clamped_time_to_go = min(t_max, max(time_to_go, t_min))
        return clamped_time_to_go


def main(args=None):
    rclpy.init(args=args)
    
    # Create node (parameters will be declared in __init__)
    node = PolymetisManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.kill_controller()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

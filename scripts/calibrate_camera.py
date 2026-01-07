#!/usr/bin/env python3
"""
Camera Calibration Script for role-ros2.

This script performs hand-eye calibration for cameras using a Charuco board.

Modes:
    - hand: Calibrate hand-mounted camera (camera_optical_frame -> gripper link)
    - third: Calibrate third-person static camera (camera_optical_frame -> base_link)

Usage:
    # Calibrate static camera
    python3 calibrate_camera.py --camera_id 24285872 --mode third
    
    # Calibrate hand camera
    python3 calibrate_camera.py --camera_id 11022812 --mode hand

Workflow:
    1. Reset robot to home position
    2. Move robot with VR controller to position camera facing Charuco board
    3. Press A/X to start calibration (robot will move automatically)
    4. After calibration cycle completes, press:
       - A/X: Accept calibration, publish static TF, save to config
       - B/Y: Reject and recalibrate (reset robot, return to step 2)

Author: Role-ROS2 Team
"""

import argparse
import os
import signal
import subprocess
import time
import traceback
from copy import deepcopy
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.duration import Duration
from scipy.spatial.transform import Rotation as R
import tf2_ros

from role_ros2.calibration.calibration_utils import (
    HandCameraCalibrator,
    ThirdPersonCameraCalibrator,
    calibration_traj,
    save_calibration_results,
)
from role_ros2.controllers.oculus_controller import VRPolicy
from role_ros2.misc.config_loader import get_package_config_path
from role_ros2.misc.transformations import change_pose_frame
from role_ros2.robot_env import RobotEnv


# Long press duration threshold (seconds)
LONG_PRESS_THRESHOLD = 0.5


def load_camera_config(camera_id: str) -> Optional[Dict]:
    """
    Load camera configuration from multi_camera_reader_config.yaml.
    
    Args:
        camera_id: Camera identifier
    
    Returns:
        Camera configuration dictionary or None if not found
    """
    config_path = get_package_config_path('multi_camera_reader_config.yaml')
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        cameras = config.get('cameras', [])
        for camera in cameras:
            if camera.get('camera_id') == camera_id:
                return camera
        return None
    except Exception as e:
        print(f"Error loading camera config: {e}")
        return None


def get_intrinsics_for_calibrator(env: RobotEnv, camera_id: str) -> Optional[Dict]:
    """
    Get camera intrinsics formatted for the calibrator.
    
    Args:
        env: RobotEnv instance
        camera_id: Camera identifier
    
    Returns:
        Dictionary in format {camera_id: {"cameraMatrix": ..., "distCoeffs": ...}}
    """
    if env.camera_reader is None:
        return None
    
    camera_reader = env.camera_reader.camera_dict.get(camera_id)
    if camera_reader is None:
        return None
    
    intrinsics = camera_reader.get_intrinsics()
    if intrinsics is None:
        return None
    
    # Format for calibrator: camera_id -> {"cameraMatrix": ..., "distCoeffs": ...}
    return {
        camera_id: {
            "cameraMatrix": intrinsics["cameraMatrix"],
            "distCoeffs": intrinsics.get("distortionCoefficients", np.zeros(5))
        }
    }


def pose_to_transform_matrix(pose: np.ndarray) -> np.ndarray:
    """
    Convert 6D pose [x, y, z, rx, ry, rz] to 4x4 transformation matrix.
    
    Args:
        pose: 6D pose [x, y, z, rx, ry, rz] (euler XYZ)
    
    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4)
    T[:3, 3] = pose[:3]
    T[:3, :3] = R.from_euler("xyz", pose[3:6]).as_matrix()
    return T


def transform_matrix_to_pose(T: np.ndarray) -> np.ndarray:
    """
    Convert 4x4 transformation matrix to 6D pose.
    
    Args:
        T: 4x4 transformation matrix
    
    Returns:
        6D pose [x, y, z, rx, ry, rz] (euler XYZ)
    """
    pos = T[:3, 3]
    euler = R.from_matrix(T[:3, :3]).as_euler("xyz")
    return np.concatenate([pos, euler])


def lookup_tf_transform(
    tf_buffer: tf2_ros.Buffer,
    target_frame: str,
    source_frame: str,
    timeout_sec: float = 5.0
) -> Optional[np.ndarray]:
    """
    Lookup transform from TF buffer and convert to 4x4 matrix.
    
    Args:
        tf_buffer: TF2 buffer
        target_frame: Target frame name
        source_frame: Source frame name
        timeout_sec: Timeout in seconds
    
    Returns:
        4x4 transformation matrix or None if lookup failed
    """
    try:
        transform = tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=timeout_sec)
        )
        
        # Extract translation and rotation
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Convert to 4x4 matrix
        T = np.eye(4)
        T[:3, 3] = [t.x, t.y, t.z]
        T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        return T
    except Exception as e:
        print(f"TF lookup failed ({source_frame} -> {target_frame}): {e}")
        return None


def publish_static_tf(
    parent_frame: str,
    child_frame: str,
    transformation: np.ndarray,
    node: Node
) -> bool:
    """
    Publish static TF transform using ros2 CLI.
    
    Args:
        parent_frame: Parent frame name
        child_frame: Child frame name  
        transformation: 6D pose [x, y, z, rx, ry, rz]
        node: ROS2 node (for logging)
    
    Returns:
        True if successful
    """
    # Convert euler to quaternion
    quat = R.from_euler("xyz", transformation[3:6]).as_quat()
    
    # Build command
    cmd = [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", str(transformation[0]),
        "--y", str(transformation[1]),
        "--z", str(transformation[2]),
        "--qx", str(quat[0]),
        "--qy", str(quat[1]),
        "--qz", str(quat[2]),
        "--qw", str(quat[3]),
        "--frame-id", parent_frame,
        "--child-frame-id", child_frame
    ]
    
    node.get_logger().info(f"Publishing static TF: {parent_frame} -> {child_frame}")
    node.get_logger().info(f"Translation: [{transformation[0]:.4f}, {transformation[1]:.4f}, {transformation[2]:.4f}]")
    node.get_logger().info(f"Rotation (euler): [{transformation[3]:.4f}, {transformation[4]:.4f}, {transformation[5]:.4f}]")
    node.get_logger().info(f"Rotation (quat): [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")
    
    try:
        # Run in background
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        node.get_logger().error(f"Failed to publish static TF: {e}")
        return False


class CameraCalibrator:
    """
    Camera calibration class for hand-eye calibration.
    
    This class manages the calibration workflow:
    1. Robot reset and positioning
    2. Automatic calibration trajectory execution
    3. Accuracy evaluation
    4. TF publishing and result saving
    """
    
    def __init__(self, args):
        """
        Initialize camera calibrator.
        
        Args:
            args: Parsed command line arguments
        """
        self.camera_id = args.camera_id
        self.mode = args.mode
        self.step_size = args.step_size
        self.pause_time = args.pause_time
        self.image_freq = args.image_freq
        self.right_controller = args.right_controller
        self.output_filepath = args.output
        
        # State variables
        self._shutdown_requested = False
        self._calibration_complete = False
        self._calibration_success = False
        self._calibration_transformation = None
        self._accuracy_info = {}
        
        # Long press detection
        self._success_press_start: Optional[float] = None
        self._failure_press_start: Optional[float] = None
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('camera_calibrator_node')
        
        # Initialize TF2 buffer and listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        
        # Print initialization header
        self._print("=" * 70)
        self._print("📷 Camera Calibration - Starting Initialization")
        self._print("=" * 70)
        self._print(f"   • Camera ID: {self.camera_id}")
        self._print(f"   • Mode: {self.mode}")
        self._print(f"   • Step size: {self.step_size}")
        self._print(f"   • Pause time: {self.pause_time}")
        self._print(f"   • Image frequency: {self.image_freq}")
        self._print(f"   • Output file: {self.output_filepath}")
        self._print("-" * 70)
        
        # Load camera config
        self._print("🔧 [1/5] Loading camera configuration...")
        self.camera_config = load_camera_config(self.camera_id)
        if self.camera_config is None:
            raise ValueError(f"Camera {self.camera_id} not found in config")
        
        self.camera_frame = self.camera_config.get("camera_frame", None)
        self.camera_base_frame = self.camera_config.get("camera_base_frame", None)
        self.camera_base_parent_frame = self.camera_config.get("camera_base_parent_frame", None)
        self.rgb_topic = self.camera_config.get("rgb_topic", "")
        self.base_frame = self.camera_config.get("base_frame", "base_link")
        
        # Validate required frames
        if self.camera_frame is None:
            raise ValueError(f"camera_frame not specified for camera {self.camera_id}")
        if self.camera_base_frame is None:
            raise ValueError(f"camera_base_frame not specified for camera {self.camera_id}")
        if self.camera_base_parent_frame is None:
            raise ValueError(f"camera_base_parent_frame not specified for camera {self.camera_id}")
        
        self._print(f"   ✅ Camera frame (optical): {self.camera_frame}")
        self._print(f"   ✅ Camera base frame: {self.camera_base_frame}")
        self._print(f"   ✅ Camera base parent frame: {self.camera_base_parent_frame}")
        self._print(f"   ✅ RGB topic: {self.rgb_topic}")
        
        # Initialize RobotEnv
        self._print("🤖 [2/5] Initializing RobotEnv...")
        self.env = RobotEnv(
            action_space="cartesian_position",
            do_reset=False,
            node=self._node,
            control_hz=15.0
        )
        self._print("   ✅ RobotEnv initialized")
        
        # Initialize VR controller
        self._print("🎮 [3/5] Initializing VR controller...")
        self.controller = VRPolicy(right_controller=self.right_controller)
        self._print("   ✅ VRPolicy initialized")
        
        # Get intrinsics for calibrator
        self._print("📷 [4/5] Getting camera intrinsics...")
        self.intrinsics_dict = None
        max_retries = 10
        for i in range(max_retries):
            self.intrinsics_dict = get_intrinsics_for_calibrator(self.env, self.camera_id)
            if self.intrinsics_dict is not None:
                break
            self._print(f"   ⏳ Waiting for camera intrinsics... ({i+1}/{max_retries})")
            time.sleep(0.5)
        
        if self.intrinsics_dict is None:
            raise ValueError(f"Could not get intrinsics for camera {self.camera_id}")
        
        self._print("   ✅ Camera intrinsics received")
        
        # Lookup fixed transform: camera_base_frame -> camera_frame (optical)
        self._print("🔗 [5/5] Looking up TF transform (camera_base_frame -> camera_frame)...")
        self._T_base_to_optical = None
        max_tf_retries = 20
        for i in range(max_tf_retries):
            # Wait a bit for TF to be available
            time.sleep(0.5)
            self._T_base_to_optical = lookup_tf_transform(
                self._tf_buffer,
                self.camera_frame,  # target (optical)
                self.camera_base_frame,  # source (base)
                timeout_sec=2.0
            )
            if self._T_base_to_optical is not None:
                break
            self._print(f"   ⏳ Waiting for TF... ({i+1}/{max_tf_retries})")
        
        if self._T_base_to_optical is None:
            raise ValueError(
                f"Could not lookup TF transform from {self.camera_base_frame} "
                f"to {self.camera_frame}. Make sure camera driver is running."
            )
        
        self._print(f"   ✅ TF transform received:")
        tf_pose = transform_matrix_to_pose(self._T_base_to_optical)
        self._print(f"      Translation: [{tf_pose[0]:.4f}, {tf_pose[1]:.4f}, {tf_pose[2]:.4f}]")
        self._print(f"      Rotation: [{tf_pose[3]:.4f}, {tf_pose[4]:.4f}, {tf_pose[5]:.4f}]")
        
        # Initialize calibrator based on mode
        if self.mode == "hand":
            self.calibrator = HandCameraCalibrator(self.intrinsics_dict)
            self._print("   ✅ HandCameraCalibrator initialized")
        else:
            self.calibrator = ThirdPersonCameraCalibrator(self.intrinsics_dict)
            self._print("   ✅ ThirdPersonCameraCalibrator initialized")
        
        self._print("-" * 70)
        self._print("✅ Initialization Complete!")
        self._print("=" * 70)
    
    def _print(self, msg: str):
        """Print message."""
        print(msg)
    
    def _check_long_press(self, controller_info: dict) -> Tuple[bool, bool]:
        """
        Check for long press of A/X (success) or B/Y (failure) buttons.
        
        Returns:
            Tuple[bool, bool]: (is_success_long_press, is_failure_long_press)
        """
        current_time = time.time()
        success_pressed = controller_info.get("success", False)
        failure_pressed = controller_info.get("failure", False)
        
        # Check SUCCESS button (A/X)
        if success_pressed:
            if self._success_press_start is None:
                self._success_press_start = current_time
            is_success = (current_time - self._success_press_start) >= LONG_PRESS_THRESHOLD
        else:
            self._success_press_start = None
            is_success = False
        
        # Check FAILURE button (B/Y)
        if failure_pressed:
            if self._failure_press_start is None:
                self._failure_press_start = current_time
            is_failure = (current_time - self._failure_press_start) >= LONG_PRESS_THRESHOLD
        else:
            self._failure_press_start = None
            is_failure = False
        
        return is_success, is_failure
    
    def wait_for_start_position(self):
        """
        Wait for user to move robot to start position using VR controller.
        Press A/X to start calibration, B/Y to exit.
        """
        self._print("")
        self._print("=" * 70)
        self._print("📍 Move robot to start position facing Charuco board")
        self._print("-" * 70)
        self._print("   • Use VR controller to position the robot")
        self._print("   • Make sure Charuco board is visible in camera")
        self._print(f"   • Press A/X (long press {LONG_PRESS_THRESHOLD}s) to START calibration")
        self._print(f"   • Press B/Y (long press {LONG_PRESS_THRESHOLD}s) to EXIT")
        self._print("=" * 70)
        
        self.controller.reset_state()
        
        while not self._shutdown_requested:
            # Get controller info
            controller_info = self.controller.get_info()
            
            # Check for long press
            is_success, is_failure = self._check_long_press(controller_info)
            
            if is_success:
                self._print("✅ Starting calibration...")
                return True
            elif is_failure:
                self._print("❌ Exiting calibration...")
                return False
            
            # Get state and read cameras
            state, _ = self.env.get_state()
            cam_obs, _ = self.env.read_cameras()
            
            # Augment image with detected markers
            for full_cam_id in cam_obs.get("image", {}):
                if self.camera_id not in full_cam_id:
                    continue
                self.calibrator._curr_cam_id = full_cam_id
                cam_obs["image"][full_cam_id] = self.calibrator.augment_image(
                    full_cam_id, cam_obs["image"][full_cam_id]
                )
            
            # Get action from controller
            action = self.controller.forward({"robot_state": state})
            action[-1] = 0  # Keep gripper open
            
            # Regularize control frequency
            time.sleep(1 / self.env.control_hz)
            
            # Step environment only if movement is enabled
            skip_step = not controller_info.get("movement_enabled", False)
            if not skip_step:
                self.env.step(action)
        
        return False
    
    def run_calibration(self):
        """
        Run automatic calibration trajectory.
        
        The robot moves through a predefined trajectory while collecting
        images of the Charuco board at regular intervals.
        """
        self._print("")
        self._print("=" * 70)
        self._print("🔄 Running calibration trajectory...")
        self._print("-" * 70)
        
        # Get starting pose
        state, _ = self.env.get_state()
        pose_origin = state["cartesian_position"]
        
        hand_camera = (self.mode == "hand")
        i = 0
        
        while not self._shutdown_requested:
            # Check for termination via controller
            controller_info = self.controller.get_info()
            _, is_failure = self._check_long_press(controller_info)
            if is_failure:
                self._print("❌ Calibration cancelled by user")
                return False
            
            start_time = time.time()
            take_picture = (i % self.image_freq) == 0
            
            # Collect observations
            if take_picture:
                time.sleep(self.pause_time)
            
            state, _ = self.env.get_state()
            cam_obs, _ = self.env.read_cameras()
            
            # Add sample and augment images
            for full_cam_id in cam_obs.get("image", {}):
                if self.camera_id not in full_cam_id:
                    continue
                
                if take_picture:
                    img = deepcopy(cam_obs["image"][full_cam_id])
                    pose = state["cartesian_position"].copy()
                    self.calibrator.add_sample(full_cam_id, img, pose)
                
                self.calibrator._curr_cam_id = full_cam_id
                cam_obs["image"][full_cam_id] = self.calibrator.augment_image(
                    full_cam_id, cam_obs["image"][full_cam_id]
                )
            
            # Move to desired next pose
            calib_pose = calibration_traj(i * self.step_size, hand_camera=hand_camera)
            desired_pose = change_pose_frame(calib_pose, pose_origin)
            action = np.concatenate([desired_pose, [0]])  # Gripper open
            
            self.env.update_robot(action, action_space="cartesian_position", blocking=False)
            
            # Regularize control frequency
            comp_time = time.time() - start_time
            sleep_left = (1 / self.env.control_hz) - comp_time
            if sleep_left > 0:
                time.sleep(sleep_left)
            
            # Check if cycle complete
            progress = min(100, (i * self.step_size) / (2 * np.pi) * 100)
            num_samples = self.calibrator.get_num_samples(self.camera_id)
            self._print(f"   Progress: {progress:.1f}% | Samples: {num_samples}")
            
            if (i * self.step_size) >= (2 * np.pi):
                break
            i += 1
        
        self._print("-" * 70)
        self._print("✅ Calibration trajectory complete!")
        return True
    
    def evaluate_calibration(self) -> bool:
        """
        Evaluate calibration accuracy and compute transformation.
        
        Returns:
            True if calibration is accurate
        """
        self._print("")
        self._print("=" * 70)
        self._print("📊 Evaluating calibration accuracy...")
        self._print("-" * 70)
        
        # Find full camera ID
        full_cam_id = None
        for cam_id in self.calibrator._readings_dict.keys():
            if self.camera_id in cam_id:
                full_cam_id = cam_id
                break
        
        if full_cam_id is None:
            self._print("❌ No calibration data found!")
            return False
        
        # Check accuracy
        success, accuracy_info = self.calibrator.is_calibration_accurate(full_cam_id)
        self._accuracy_info = accuracy_info
        
        self._print(f"   • Number of samples: {accuracy_info.get('num_samples', 0)}")
        self._print(f"   • Valid samples: {accuracy_info.get('num_valid_samples', 0)}")
        self._print(f"   • Linear error: {accuracy_info.get('lin_error', 'N/A')}")
        self._print(f"   • Rotation error: {accuracy_info.get('rot_error', 'N/A')}")
        self._print(f"   • Linear success: {accuracy_info.get('lin_success', False)}")
        self._print(f"   • Rotation success: {accuracy_info.get('rot_success', False)}")
        
        if not success:
            self._print("-" * 70)
            self._print("❌ Calibration accuracy check FAILED!")
            self._print("   Consider recalibrating with better lighting or board positioning")
            return False
        
        # Compute transformation
        transformation = self.calibrator.calibrate(full_cam_id)
        if transformation is None:
            self._print("❌ Failed to compute calibration transformation!")
            return False
        
        self._calibration_transformation = transformation
        
        self._print("-" * 70)
        self._print("✅ Calibration accuracy check PASSED!")
        self._print(f"   Transformation (optical frame -> {'gripper' if self.mode == 'hand' else 'base'}):")
        self._print(f"   Translation: [{transformation[0]:.4f}, {transformation[1]:.4f}, {transformation[2]:.4f}]")
        self._print(f"   Rotation: [{transformation[3]:.4f}, {transformation[4]:.4f}, {transformation[5]:.4f}]")
        
        return True
    
    def wait_for_user_decision(self) -> str:
        """
        Wait for user to accept or reject calibration.
        
        Returns:
            "accept", "reject", or "exit"
        """
        self._print("")
        self._print("=" * 70)
        self._print("❓ Calibration complete. Choose action:")
        self._print("-" * 70)
        self._print(f"   • Press A/X (long press {LONG_PRESS_THRESHOLD}s): ACCEPT - Publish TF and save")
        self._print(f"   • Press B/Y (long press {LONG_PRESS_THRESHOLD}s): REJECT - Reset and recalibrate")
        self._print("=" * 70)
        
        # Reset long press state
        self._success_press_start = None
        self._failure_press_start = None
        
        while not self._shutdown_requested:
            controller_info = self.controller.get_info()
            is_success, is_failure = self._check_long_press(controller_info)
            
            if is_success:
                return "accept"
            elif is_failure:
                return "reject"
            
            time.sleep(0.05)
        
        return "exit"
    
    def accept_calibration(self):
        """
        Accept calibration: publish static TF and save results.
        
        Transform computation:
        - calibrator gives: T_optical_to_parent (camera_frame -> camera_base_parent_frame)
        - TF gives: T_base_to_optical (camera_base_frame -> camera_frame)
        - We want: T_base_to_parent (camera_base_frame -> camera_base_parent_frame)
        
        T_base_to_parent = T_optical_to_parent @ T_base_to_optical
        """
        if self._calibration_transformation is None:
            self._print("❌ No calibration transformation available!")
            return
        
        self._print("")
        self._print("=" * 70)
        self._print("💾 Accepting calibration...")
        self._print("-" * 70)
        
        # The calibration gives us T_optical_to_parent:
        # - For third: camera_optical_frame -> base_link
        # - For hand: camera_optical_frame -> fr3_panda_link8
        T_optical_to_parent = pose_to_transform_matrix(self._calibration_transformation)
        
        # TF gives us T_base_to_optical (camera_base_frame -> camera_optical_frame)
        # Already stored in self._T_base_to_optical during init
        
        # Compute T_base_to_parent = T_optical_to_parent @ T_base_to_optical
        T_base_to_parent = T_optical_to_parent @ self._T_base_to_optical
        
        # Convert back to pose
        camera_base_transformation = transform_matrix_to_pose(T_base_to_parent)
        
        # Frame names from config
        parent_frame = self.camera_base_parent_frame
        child_frame = self.camera_base_frame
        
        self._print(f"   Transform chain:")
        self._print(f"   1. Calibrator: {self.camera_frame} -> {parent_frame}")
        self._print(f"   2. TF lookup: {child_frame} -> {self.camera_frame}")
        self._print(f"   3. Result: {child_frame} -> {parent_frame}")
        self._print("")
        self._print(f"   Parent frame: {parent_frame}")
        self._print(f"   Child frame: {child_frame}")
        self._print(f"   Transformation ({child_frame} -> {parent_frame}):")
        self._print(f"   Translation: [{camera_base_transformation[0]:.4f}, {camera_base_transformation[1]:.4f}, {camera_base_transformation[2]:.4f}]")
        self._print(f"   Rotation: [{camera_base_transformation[3]:.4f}, {camera_base_transformation[4]:.4f}, {camera_base_transformation[5]:.4f}]")
        
        # Publish static TF (once)
        self._print("-" * 70)
        self._print("📡 Publishing static TF...")
        publish_static_tf(parent_frame, child_frame, camera_base_transformation, self._node)
        
        # Save calibration results
        self._print("-" * 70)
        self._print(f"💾 Saving calibration results to: {self.output_filepath}")
        
        save_calibration_results(
            camera_id=self.camera_id,
            rgb_topic=self.rgb_topic,
            child_frame=child_frame,
            parent_frame=parent_frame,
            transformation=camera_base_transformation,
            output_filepath=self.output_filepath
        )
        
        self._print("-" * 70)
        self._print("✅ Calibration accepted and saved!")
        self._print("=" * 70)
    
    def reset_for_recalibration(self):
        """Reset robot and calibrator for recalibration."""
        self._print("")
        self._print("=" * 70)
        self._print("🔄 Resetting for recalibration...")
        self._print("-" * 70)
        
        # Reset robot
        self._print("   Resetting robot to home position...")
        self.env.reset(randomize=False)
        
        # Re-initialize calibrator
        if self.mode == "hand":
            self.calibrator = HandCameraCalibrator(self.intrinsics_dict)
        else:
            self.calibrator = ThirdPersonCameraCalibrator(self.intrinsics_dict)
        
        # Reset state
        self._calibration_transformation = None
        self._accuracy_info = {}
        self._success_press_start = None
        self._failure_press_start = None
        
        self._print("   ✅ Reset complete")
        self._print("=" * 70)
    
    def run(self):
        """Main calibration loop."""
        self._print("")
        self._print("=" * 70)
        self._print("🚀 Starting Camera Calibration")
        self._print("=" * 70)
        
        # Reset robot first
        self._print("🤖 Resetting robot to home position...")
        self.env.reset(randomize=False)
        self._print("   ✅ Robot reset complete")
        
        while not self._shutdown_requested:
            # Step 1: Wait for user to position robot
            if not self.wait_for_start_position():
                break
            
            # Step 2: Run calibration trajectory
            if not self.run_calibration():
                # Calibration cancelled, reset and try again
                self.reset_for_recalibration()
                continue
            
            # Step 3: Evaluate calibration
            if not self.evaluate_calibration():
                # Calibration failed accuracy check
                decision = self.wait_for_user_decision()
                if decision == "accept":
                    self._print("⚠️ Accepting low-accuracy calibration...")
                elif decision == "reject":
                    self.reset_for_recalibration()
                    continue
                else:
                    break
            
            # Step 4: Wait for user decision
            decision = self.wait_for_user_decision()
            
            if decision == "accept":
                self.accept_calibration()
                self._print("")
                self._print("🎉 Calibration complete! Static TF is being published.")
                self._print("   You can now use the calibrated camera.")
                self._print("")
                
                # Keep running to maintain TF publishing
                self._print("   Press Ctrl+C to exit (TF publisher will stop)")
                while not self._shutdown_requested:
                    time.sleep(1.0)
                break
                
            elif decision == "reject":
                self.reset_for_recalibration()
                continue
            else:
                break
        
        self._print("")
        self._print("👋 Calibration finished.")
    
    def shutdown(self):
        """Clean shutdown."""
        self._shutdown_requested = True
        
        if hasattr(self, 'env'):
            self.env.shutdown()
        
        if hasattr(self, '_node'):
            self._node.destroy_node()


def parse_args():
    """Parse command line arguments."""
    # Get default output path relative to script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_output = os.path.join(script_dir, "..", "config", "calibration_results.yaml")
    
    parser = argparse.ArgumentParser(
        description='Camera calibration for role-ros2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate static camera
  python3 calibrate_camera.py --camera_id 24285872 --mode third
  
  # Calibrate hand camera
  python3 calibrate_camera.py --camera_id 11022812 --mode hand
  
  # Custom parameters
  python3 calibrate_camera.py --camera_id 24285872 --mode third --step_size 0.005 --image_freq 5
  
  # Custom output path
  python3 calibrate_camera.py --camera_id 24285872 --mode third --output /path/to/output.yaml
        """
    )
    
    parser.add_argument(
        '--camera_id',
        type=str,
        required=True,
        help='Camera identifier (serial number)'
    )
    parser.add_argument(
        '--mode',
        type=str,
        required=True,
        choices=['hand', 'third'],
        help='Calibration mode: "hand" for hand-mounted camera, "third" for static camera'
    )
    parser.add_argument(
        '--output',
        type=str,
        default=default_output,
        help='Output file path for calibration results (default: ../config/calibration_results.yaml)'
    )
    parser.add_argument(
        '--step_size',
        type=float,
        default=0.01,
        help='Trajectory step size (default: 0.01)'
    )
    parser.add_argument(
        '--pause_time',
        type=float,
        default=0.5,
        help='Pause time before capturing image (default: 0.5s)'
    )
    parser.add_argument(
        '--image_freq',
        type=int,
        default=10,
        help='Take image every N steps (default: 10)'
    )
    parser.add_argument(
        '--right-controller',
        action='store_true',
        default=True,
        help='Use right VR controller (default: True)'
    )
    parser.add_argument(
        '--left-controller',
        dest='right_controller',
        action='store_false',
        help='Use left VR controller'
    )
    
    return parser.parse_args()


def main():
    """Main function."""
    args = parse_args()
    
    calibrator = None
    
    def signal_handler(signum, frame):
        nonlocal calibrator
        print("\n⚠️ Shutdown signal received (Ctrl+C)")
        if calibrator is not None:
            calibrator.shutdown()
        raise KeyboardInterrupt()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        calibrator = CameraCalibrator(args)
        calibrator.run()
        
    except KeyboardInterrupt:
        print("👋 Exiting...")
    except Exception as e:
        print(f"❌ Error: {e}")
        print(traceback.format_exc())
    finally:
        if calibrator is not None:
            calibrator.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

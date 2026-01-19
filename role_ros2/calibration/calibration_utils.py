"""
Calibration utilities for role-ros2.

This module provides calibration utilities for hand-eye calibration using Charuco boards.
Based on droid/calibration/calibration_utils.py.
"""

import os
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from role_ros2.calibration.config import (
    ARUCO_DICT,
    CHARUCOBOARD_CHECKER_SIZE,
    CHARUCOBOARD_COLCOUNT,
    CHARUCOBOARD_MARKER_SIZE,
    CHARUCOBOARD_ROWCOUNT,
    DEFAULT_INLIER_ERROR_THRESHOLD,
    DEFAULT_LIN_ERROR_THRESHOLD,
    DEFAULT_NUM_CORNER_THRESHOLD,
    DEFAULT_NUM_IMG_THRESHOLD,
    DEFAULT_REPROJECTION_ERROR_THRESHOLD,
    DEFAULT_ROT_ERROR_THRESHOLD,
    DEFAULT_TRAIN_PERCENTAGE,
)
from role_ros2.misc.transformations import pose_diff
from role_ros2.misc.config_loader import get_source_config_path

# Create Board
try:
    from cv2 import aruco
except ImportError:
    aruco = cv2.aruco

CHARUCO_BOARD = aruco.CharucoBoard_create(
    squaresX=CHARUCOBOARD_COLCOUNT,
    squaresY=CHARUCOBOARD_ROWCOUNT,
    squareLength=CHARUCOBOARD_CHECKER_SIZE,
    markerLength=CHARUCOBOARD_MARKER_SIZE,
    dictionary=ARUCO_DICT,
)

# Detector Parameters
detector_params = cv2.aruco.DetectorParameters_create()
detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
calib_flags = (
    cv2.CALIB_USE_INTRINSIC_GUESS + 
    cv2.CALIB_FIX_PRINCIPAL_POINT + 
    cv2.CALIB_FIX_FOCAL_LENGTH
)

def load_calibration_info(filepath: Optional[str] = None) -> Dict:
    """
    Load calibration information from YAML file.
    
    Args:
        filepath: Path to calibration results YAML file.
                  If None, uses source directory config/calibration_results.yaml
    
    Returns:
        Dictionary with calibration information
    """
    if filepath is None:
        # Use source directory path, not install directory
        filepath = str(get_source_config_path('calibration_results.yaml'))
    
    if not os.path.isfile(filepath):
        return {}
    
    try:
        with open(filepath, "r") as f:
            data = yaml.safe_load(f) or {}
        
        # Convert to camera_id -> pose dict for compatibility
        calibration_info = {}
        cameras = data.get("cameras", [])
        for cam in cameras:
            cam_id = cam.get("camera_id")
            if cam_id and "transform" in cam:
                t = cam["transform"]
                pose = [
                    t.get("x", 0), t.get("y", 0), t.get("z", 0),
                    t.get("rx", 0), t.get("ry", 0), t.get("rz", 0)
                ]
                calibration_info[cam_id] = pose
        return calibration_info
    except Exception:
        return {}


def save_calibration_results(
    camera_id: str,
    rgb_topic: str,
    child_frame: str,
    parent_frame: str,
    transformation: np.ndarray,
    output_filepath: str
) -> None:
    """
    Save calibration results to YAML file.
    
    This function updates the transformation for the specified camera_id while
    preserving transformations for all other cameras in the file.
    
    Output format:
        cameras:
        - camera_id: "xxx"
          rgb_topic: "/xxx/rgb/image_rect_color"
          child_frame: "camera_base_frame"
          parent_frame: "camera_base_parent_frame"
          transform:
            x: 0.0
            y: 0.0
            z: 0.0
            rx: 0.0
            ry: 0.0
            rz: 0.0
    
    Args:
        camera_id: Camera identifier (used to update existing entry or add new one)
        rgb_topic: RGB topic name for this camera
        child_frame: Child frame name (camera_base_frame)
        parent_frame: Parent frame name (camera_base_parent_frame)
        transformation: 6D pose [x, y, z, rx, ry, rz]
        output_filepath: Output file path (fixed file path for all cameras)
    
    Note:
        - If camera_id already exists, its transformation will be updated
        - All other camera transformations will be preserved
        - If file doesn't exist, a new file will be created
    """
    # Normalize file path
    output_filepath = os.path.abspath(output_filepath)
    
    # Load existing results or create new
    results = {}
    if os.path.isfile(output_filepath):
        try:
            with open(output_filepath, "r", encoding="utf-8") as f:
                results = yaml.safe_load(f) or {}
        except Exception as e:
            # If file exists but can't be read, create new results
            # This preserves data integrity
            print(f"Warning: Could not read existing calibration file {output_filepath}: {e}")
            print("Creating new calibration file...")
            results = {}
    
    # Ensure cameras list exists
    if "cameras" not in results:
        results["cameras"] = []
    
    # Create calibration entry
    entry = {
        "camera_id": camera_id,
        "rgb_topic": rgb_topic,
        "child_frame": child_frame,
        "parent_frame": parent_frame,
        "transform": {
            "x": float(transformation[0]),
            "y": float(transformation[1]),
            "z": float(transformation[2]),
            "rx": float(transformation[3]),
            "ry": float(transformation[4]),
            "rz": float(transformation[5])
        }
    }
    
    # Update or append entry based on camera_id
    # This preserves all other camera transformations
    found = False
    for i, cam in enumerate(results["cameras"]):
        if cam.get("camera_id") == camera_id:
            results["cameras"][i] = entry
            found = True
            break
    
    if not found:
        results["cameras"].append(entry)
    
    # Ensure output directory exists
    output_dir = os.path.dirname(output_filepath)
    if output_dir:  # Only create directory if path has directory component
        os.makedirs(output_dir, exist_ok=True)
    
    # Write to file atomically (write to temp file, then rename)
    # This prevents data loss if write is interrupted
    temp_filepath = output_filepath + ".tmp"
    try:
        with open(temp_filepath, "w", encoding="utf-8") as f:
            yaml.dump(results, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        
        # Atomic rename (works on Unix and Windows)
        if os.name == 'nt':  # Windows
            # On Windows, need to remove target first if it exists
            if os.path.exists(output_filepath):
                os.remove(output_filepath)
        os.rename(temp_filepath, output_filepath)
    except Exception as e:
        # Clean up temp file on error
        if os.path.exists(temp_filepath):
            try:
                os.remove(temp_filepath)
            except:
                pass
        raise IOError(f"Failed to save calibration results to {output_filepath}: {e}")


def calibration_traj(
    t: float,
    pos_scale: float = 0.1,
    angle_scale: float = 0.2,
    hand_camera: bool = False
) -> np.ndarray:
    """
    Generate calibration trajectory point.
    
    Args:
        t: Trajectory parameter (0 to 2*pi for one complete cycle)
        pos_scale: Position scaling factor
        angle_scale: Angle scaling factor
        hand_camera: If True, use hand camera trajectory profile
    
    Returns:
        6D pose offset [x, y, z, rx, ry, rz]
    """
    x = -np.abs(np.sin(3 * t)) * pos_scale
    y = -0.8 * np.sin(2 * t) * pos_scale
    z = 0.5 * np.sin(4 * t) * pos_scale
    a = -np.sin(4 * t) * angle_scale
    b = np.sin(3 * t) * angle_scale
    c = np.sin(2 * t) * angle_scale
    
    if hand_camera:
        value = np.array([z, y, -x, c / 1.5, b / 1.5, -a / 1.5])
    else:
        value = np.array([x, y, z, a, b, c])
    
    return value


class CharucoDetector:
    """
    Charuco board detector for camera calibration.
    
    This class handles detecting Charuco board corners in images and
    calculating camera-to-target transformations.
    """
    
    def __init__(
        self,
        intrinsics_dict: Dict,
        inlier_error_threshold: float = DEFAULT_INLIER_ERROR_THRESHOLD,
        reprojection_error_threshold: float = DEFAULT_REPROJECTION_ERROR_THRESHOLD,
        num_img_threshold: int = DEFAULT_NUM_IMG_THRESHOLD,
        num_corner_threshold: int = DEFAULT_NUM_CORNER_THRESHOLD,
    ):
        """
        Initialize Charuco detector.
        
        Args:
            intrinsics_dict: Dictionary of camera intrinsics {cam_id: {"cameraMatrix": ..., "distCoeffs": ...}}
            inlier_error_threshold: Maximum reprojection error for inlier images
            reprojection_error_threshold: Maximum average reprojection error for calibration
            num_img_threshold: Minimum number of images required for calibration
            num_corner_threshold: Minimum number of corners required per image
        """
        self.inlier_error_threshold = inlier_error_threshold
        self.reprojection_error_threshold = reprojection_error_threshold
        self.num_img_threshold = num_img_threshold
        self.num_corner_threshold = num_corner_threshold
        self.intrinsic_params = {}
        self._intrinsics_dict = intrinsics_dict
        self._readings_dict = defaultdict(list)
        self._pose_dict = defaultdict(list)
        self._curr_cam_id = None

    def process_image(self, image: np.ndarray) -> Optional[Tuple]:
        """
        Process image to detect Charuco corners.
        
        Args:
            image: Input image (BGR or BGRA)
        
        Returns:
            Tuple of (corners, charuco_corners, charuco_ids, img_size) or None if detection failed
        """
        if image.shape[2] == 4:
            gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        elif image.shape[2] == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            raise ValueError(f"Unsupported image format with {image.shape[2]} channels")
        
        img_size = image.shape[:2]

        # Find Aruco Markers In Image
        corners, ids, rejected = aruco.detectMarkers(
            image=gray,
            dictionary=ARUCO_DICT,
            parameters=detector_params
        )

        corners, ids, _, _ = cv2.aruco.refineDetectedMarkers(
            gray,
            CHARUCO_BOARD,
            corners,
            ids,
            rejected,
            parameters=detector_params,
            **self._intrinsics_dict[self._curr_cam_id],
        )

        # Find Charuco Corners
        if len(corners) == 0:
            return None

        num_corners_found, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=CHARUCO_BOARD,
            **self.intrinsic_params
        )

        if num_corners_found < self.num_corner_threshold:
            return None

        return corners, charuco_corners, charuco_ids, img_size

    def add_sample(self, cam_id: str, image: np.ndarray, pose: np.ndarray) -> None:
        """
        Add calibration sample.
        
        Args:
            cam_id: Camera identifier
            image: Image containing Charuco board
            pose: Robot gripper pose at time of image capture
        """
        self._curr_cam_id = cam_id
        readings = self.process_image(image)
        if readings is None:
            return
        self._readings_dict[cam_id].append(readings)
        self._pose_dict[cam_id].append(pose)

    def calculate_target_to_cam(
        self,
        readings: List,
        train: bool = True
    ) -> Optional[Tuple]:
        """
        Calculate target-to-camera transformation from readings.
        
        Args:
            readings: List of detection readings
            train: If True, use training threshold; else use evaluation threshold
        
        Returns:
            Tuple of (rmats, tvecs, successes) or None if calibration failed
        """
        init_corners_all = []
        init_ids_all = []
        fixed_image_size = readings[0][3]

        # Process Readings
        init_successes = []
        for i in range(len(readings)):
            corners, charuco_corners, charuco_ids, img_size = readings[i]
            assert img_size == fixed_image_size
            init_corners_all.append(charuco_corners)
            init_ids_all.append(charuco_ids)
            init_successes.append(i)

        # First Pass: Find Outliers
        threshold = self.num_img_threshold if train else 5
        if len(init_successes) < threshold:
            return None

        (
            calibration_error,
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs,
            stdIntrinsics,
            stdExtrinsics,
            perViewErrors
        ) = aruco.calibrateCameraCharucoExtended(
            charucoCorners=init_corners_all,
            charucoIds=init_ids_all,
            board=CHARUCO_BOARD,
            imageSize=fixed_image_size,
            flags=calib_flags,
            **self._intrinsics_dict[self._curr_cam_id],
        )

        # Remove Outliers
        threshold = self.num_img_threshold if train else 5
        final_corners_all = [
            init_corners_all[i]
            for i in range(len(perViewErrors))
            if perViewErrors[i] <= self.inlier_error_threshold
        ]
        final_ids_all = [
            init_ids_all[i]
            for i in range(len(perViewErrors))
            if perViewErrors[i] <= self.inlier_error_threshold
        ]
        final_successes = [
            init_successes[i]
            for i in range(len(perViewErrors))
            if perViewErrors[i] <= self.inlier_error_threshold
        ]
        if len(final_successes) < threshold:
            return None

        # Second Pass: Calculate Finalized Extrinsics
        calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
            charucoCorners=final_corners_all,
            charucoIds=final_ids_all,
            board=CHARUCO_BOARD,
            imageSize=fixed_image_size,
            flags=calib_flags,
            **self._intrinsics_dict[self._curr_cam_id],
        )

        # Return Transformation
        if calibration_error > self.reprojection_error_threshold:
            return None

        rmats = [R.from_rotvec(rvec.flatten()).as_matrix() for rvec in rvecs]
        tvecs = [tvec.flatten() for tvec in tvecs]

        return rmats, tvecs, final_successes

    def augment_image(
        self,
        cam_id: str,
        image: np.ndarray,
        visualize: bool = False,
        visual_type: List[str] = None
    ) -> np.ndarray:
        """
        Augment image with detected Charuco board markers.
        
        Args:
            cam_id: Camera identifier
            image: Input image
            visualize: If True, display augmented image
            visual_type: List of visualization types ("markers", "charuco", "axes")
        
        Returns:
            Augmented image
        """
        if visual_type is None:
            visual_type = ["markers", "axes"]
        if not isinstance(visual_type, list):
            visual_type = [visual_type]
        assert all([t in ["markers", "charuco", "axes"] for t in visual_type])
        
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        self._curr_cam_id = cam_id

        image = np.copy(image)
        readings = self.process_image(image)

        if readings is None:
            if visualize:
                cv2.imshow(f"Charuco board: {cam_id}", image)
                cv2.waitKey(20)
            return image

        corners, charuco_corners, charuco_ids, image_size = readings

        # Outline the aruco markers found in query image
        if "markers" in visual_type:
            image = aruco.drawDetectedMarkers(image=image, corners=corners)

        # Draw the Charuco board detected
        if "charuco" in visual_type:
            image = aruco.drawDetectedCornersCharuco(
                image=image,
                charucoCorners=charuco_corners,
                charucoIds=charuco_ids
            )

        if "axes" in visual_type:
            calibration_error, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=[charuco_corners],
                charucoIds=[charuco_ids],
                board=CHARUCO_BOARD,
                imageSize=image_size,
                flags=calib_flags,
                **self._intrinsics_dict[self._curr_cam_id],
            )
            cv2.drawFrameAxes(image, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1)

        # Visualize
        if visualize:
            cv2.imshow(f"Charuco board: {cam_id}", image)
            cv2.waitKey(20)

        return image

    def get_num_samples(self, cam_id: str) -> int:
        """
        Get number of collected samples for a camera.
        
        Args:
            cam_id: Camera identifier
        
        Returns:
            Number of samples
        """
        return len(self._readings_dict[cam_id])


class ThirdPersonCameraCalibrator(CharucoDetector):
    """
    Calibrator for third-person (static) cameras.
    
    This calibrator computes the transformation from camera to robot base.
    """
    
    def __init__(
        self,
        intrinsics_dict: Dict,
        lin_error_threshold: float = DEFAULT_LIN_ERROR_THRESHOLD,
        rot_error_threshold: float = DEFAULT_ROT_ERROR_THRESHOLD,
        train_percentage: float = DEFAULT_TRAIN_PERCENTAGE,
        **kwargs
    ):
        """
        Initialize third-person camera calibrator.
        
        Args:
            intrinsics_dict: Camera intrinsics dictionary
            lin_error_threshold: Maximum linear error for successful calibration
            rot_error_threshold: Maximum rotation error for successful calibration
            train_percentage: Percentage of data to use for training
            **kwargs: Additional arguments for CharucoDetector
        """
        self.lin_error_threshold = lin_error_threshold
        self.rot_error_threshold = rot_error_threshold
        self.train_percentage = train_percentage
        super().__init__(intrinsics_dict, **kwargs)

    def calibrate(self, cam_id: str) -> Optional[np.ndarray]:
        """
        Calibrate camera and return transformation.
        
        Args:
            cam_id: Camera identifier
        
        Returns:
            6D pose [x, y, z, rx, ry, rz] or None if calibration failed
        """
        return self._calibrate_cam_to_base(cam_id=cam_id)

    def _calibrate_cam_to_base(
        self,
        cam_id: Optional[str] = None,
        readings: Optional[List] = None,
        gripper_poses: Optional[List] = None,
        target2cam_results: Optional[Tuple] = None
    ) -> Optional[np.ndarray]:
        """
        Calibrate camera-to-base transformation.
        
        Args:
            cam_id: Camera identifier
            readings: Optional pre-computed readings
            gripper_poses: Optional pre-computed gripper poses
            target2cam_results: Optional pre-computed target-to-camera results
        
        Returns:
            6D pose or None if calibration failed
        """
        # Get Calibration Data
        if cam_id is not None:
            readings = self._readings_dict[cam_id]
            gripper_poses = self._pose_dict[cam_id]
            self._curr_cam_id = cam_id

        # Get Target2Cam Transformation
        if target2cam_results is None:
            target2cam_results = self.calculate_target_to_cam(readings)
        if target2cam_results is None:
            return None

        R_target2cam, t_target2cam, successes = target2cam_results
        gripper_poses = np.array(gripper_poses)[successes]

        # Calculate Appropriate Transformations
        t_base2gripper = [
            -R.from_euler("xyz", pose[3:6]).inv().as_matrix() @ np.array(pose[:3])
            for pose in gripper_poses
        ]
        R_base2gripper = [
            R.from_euler("xyz", pose[3:6]).inv().as_matrix()
            for pose in gripper_poses
        ]

        # Perform Calibration
        rmat, pos = cv2.calibrateHandEye(
            R_gripper2base=R_base2gripper,
            t_gripper2base=t_base2gripper,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=4,
        )

        # Return Pose
        pos = pos.flatten()
        angle = R.from_matrix(rmat).as_euler("xyz")
        pose = np.concatenate([pos, angle])

        return pose

    def _calibrate_gripper_to_target(
        self,
        cam_id: Optional[str] = None,
        readings: Optional[List] = None,
        gripper_poses: Optional[List] = None,
        target2cam_results: Optional[Tuple] = None
    ) -> Optional[np.ndarray]:
        """Calibrate gripper-to-target transformation."""
        # Get Calibration Data
        if cam_id is not None:
            readings = self._readings_dict[cam_id]
            gripper_poses = self._pose_dict[cam_id]
            self._curr_cam_id = cam_id

        # Get Target2Cam Transformation
        if target2cam_results is None:
            target2cam_results = self.calculate_target_to_cam(readings)
        if target2cam_results is None:
            return None

        R_target2cam, t_target2cam, successes = target2cam_results
        gripper_poses = np.array(gripper_poses)[successes]

        # Calculate Appropriate Transformations
        t_base2gripper = [
            -R.from_euler("xyz", pose[3:6]).inv().as_matrix() @ np.array(pose[:3])
            for pose in gripper_poses
        ]
        R_base2gripper = [
            R.from_euler("xyz", pose[3:6]).inv().as_matrix()
            for pose in gripper_poses
        ]

        # Perform Calibration
        rmat, pos = cv2.calibrateHandEye(
            R_gripper2base=R_target2cam,
            t_gripper2base=t_target2cam,
            R_target2cam=R_base2gripper,
            t_target2cam=t_base2gripper,
            method=4,
        )

        # Return Pose
        pos = pos.flatten()
        angle = R.from_matrix(rmat).as_euler("xyz")
        pose = np.concatenate([pos, angle])

        return pose

    def _calculate_gripper_to_base(
        self,
        train_readings: List,
        train_gripper_poses: List,
        eval_readings: Optional[List] = None
    ) -> Optional[Tuple]:
        """Calculate gripper-to-base transformation for evaluation."""
        if eval_readings is None:
            eval_readings = train_readings

        # Get Eval Target2Cam Transformations
        eval_results = self.calculate_target_to_cam(eval_readings, train=False)
        if eval_results is None:
            return None
        eval_R_target2cam, eval_t_target2cam, eval_successes = eval_results
        rmats, tvecs = [], []

        # Get Train Target2Cam Transformations
        train_results = self.calculate_target_to_cam(train_readings)
        if train_results is None:
            return None

        # Use Training Data For Calibrations
        gripper2target = self._calibrate_gripper_to_target(
            gripper_poses=train_gripper_poses,
            target2cam_results=train_results
        )
        R_gripper2target = R.from_euler("xyz", gripper2target[3:]).as_matrix()
        t_gripper2target = np.array(gripper2target[:3])

        cam2base = self._calibrate_cam_to_base(
            gripper_poses=train_gripper_poses,
            target2cam_results=train_results
        )
        R_cam2base = R.from_euler("xyz", cam2base[3:]).as_matrix()
        t_cam2base = np.array(cam2base[:3])

        # Calculate Gripper2Base
        for i in range(len(eval_R_target2cam)):
            R_gripper2cam = eval_R_target2cam[i] @ R_gripper2target
            t_gripper2cam = eval_R_target2cam[i] @ t_gripper2target + eval_t_target2cam[i]

            R_gripper2base = R_cam2base @ R_gripper2cam
            t_gripper2base = R_cam2base @ t_gripper2cam + t_cam2base

            rmats.append(R_gripper2base)
            tvecs.append(t_gripper2base)

        # Return Poses
        eulers = np.array([R.from_matrix(rmat).as_euler("xyz") for rmat in rmats])
        eval_poses = np.concatenate([np.array(tvecs), eulers], axis=1)

        return eval_poses, eval_successes

    def is_calibration_accurate(self, cam_id: str) -> Tuple[bool, Dict]:
        """
        Check if calibration is accurate.
        
        Args:
            cam_id: Camera identifier
        
        Returns:
            Tuple of (success, accuracy_info)
        """
        # Set Camera
        self._curr_cam_id = cam_id

        # Split Into Train / Test
        readings = self._readings_dict[cam_id]
        if len(readings) == 0:
            return False, {"error": "No readings collected"}
        
        poses = np.array(self._pose_dict[cam_id])
        ind = np.random.choice(len(readings), size=len(readings), replace=False)
        num_train = int(len(readings) * self.train_percentage)

        train_ind, test_ind = ind[:num_train], ind[num_train:]
        train_poses, test_poses = poses[train_ind], poses[test_ind]
        train_readings = [readings[i] for i in train_ind]
        test_readings = [readings[i] for i in test_ind]

        # Calculate Approximate Gripper2Base Transformations
        results = self._calculate_gripper_to_base(
            train_readings, train_poses, eval_readings=test_readings
        )
        if results is None:
            return False, {"error": "Failed to calculate gripper-to-base transformation"}
        
        approx_poses, successes = results
        test_poses = np.array(test_poses)[successes]

        # Calculate Per Dimension Error
        pose_error = np.array([
            pose_diff(pose, approx_pose)
            for pose, approx_pose in zip(test_poses, approx_poses)
        ])
        lin_error = np.linalg.norm(pose_error[:, :3], axis=0) ** 2 / pose_error.shape[0]
        rot_error = np.linalg.norm(pose_error[:, 3:6], axis=0) ** 2 / pose_error.shape[0]

        # Check Calibration Error
        lin_success = np.all(lin_error < self.lin_error_threshold)
        rot_success = np.all(rot_error < self.rot_error_threshold)

        accuracy_info = {
            "pose_std": poses.std(axis=0).tolist(),
            "lin_error": lin_error.tolist(),
            "rot_error": rot_error.tolist(),
            "num_samples": len(readings),
            "num_valid_samples": len(successes),
            "lin_success": bool(lin_success),
            "rot_success": bool(rot_success),
        }

        print(f"Pose Std: {poses.std(axis=0)}")
        print(f"Lin Error: {lin_error}")
        print(f"Rot Error: {rot_error}")

        return lin_success and rot_success, accuracy_info


class HandCameraCalibrator(CharucoDetector):
    """
    Calibrator for hand-mounted cameras.
    
    This calibrator computes the transformation from camera to gripper (end-effector).
    """
    
    def __init__(
        self,
        intrinsics_dict: Dict,
        lin_error_threshold: float = DEFAULT_LIN_ERROR_THRESHOLD,
        rot_error_threshold: float = DEFAULT_ROT_ERROR_THRESHOLD,
        train_percentage: float = DEFAULT_TRAIN_PERCENTAGE,
        **kwargs
    ):
        """
        Initialize hand camera calibrator.
        
        Args:
            intrinsics_dict: Camera intrinsics dictionary
            lin_error_threshold: Maximum linear error for successful calibration
            rot_error_threshold: Maximum rotation error for successful calibration
            train_percentage: Percentage of data to use for training
            **kwargs: Additional arguments for CharucoDetector
        """
        self.lin_error_threshold = lin_error_threshold
        self.rot_error_threshold = rot_error_threshold
        self.train_percentage = train_percentage
        super().__init__(intrinsics_dict, **kwargs)

    def calibrate(self, cam_id: str) -> Optional[np.ndarray]:
        """
        Calibrate camera and return transformation.
        
        Args:
            cam_id: Camera identifier
        
        Returns:
            6D pose [x, y, z, rx, ry, rz] or None if calibration failed
        """
        return self._calibrate_cam_to_gripper(cam_id=cam_id)

    def _calibrate_cam_to_gripper(
        self,
        cam_id: Optional[str] = None,
        readings: Optional[List] = None,
        gripper_poses: Optional[List] = None,
        target2cam_results: Optional[Tuple] = None
    ) -> Optional[np.ndarray]:
        """
        Calibrate camera-to-gripper transformation.
        
        Args:
            cam_id: Camera identifier
            readings: Optional pre-computed readings
            gripper_poses: Optional pre-computed gripper poses
            target2cam_results: Optional pre-computed target-to-camera results
        
        Returns:
            6D pose or None if calibration failed
        """
        # Get Calibration Data
        if cam_id is not None:
            readings = self._readings_dict[cam_id]
            gripper_poses = self._pose_dict[cam_id]
            self._curr_cam_id = cam_id

        # Get Target2Cam Transformation
        if target2cam_results is None:
            target2cam_results = self.calculate_target_to_cam(readings)
        if target2cam_results is None:
            return None

        R_target2cam, t_target2cam, successes = target2cam_results
        gripper_poses = np.array(gripper_poses)[successes]

        # Calculate Appropriate Transformations
        t_gripper2base = [np.array(pose[:3]) for pose in gripper_poses]
        R_gripper2base = [
            R.from_euler("xyz", pose[3:6]).as_matrix()
            for pose in gripper_poses
        ]

        # Perform Calibration
        rmat, pos = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=4,
        )

        # Return Pose
        pos = pos.flatten()
        angle = R.from_matrix(rmat).as_euler("xyz")
        pose = np.concatenate([pos, angle])

        return pose

    def _calibrate_base_to_target(
        self,
        cam_id: Optional[str] = None,
        readings: Optional[List] = None,
        gripper_poses: Optional[List] = None,
        target2cam_results: Optional[Tuple] = None
    ) -> Optional[np.ndarray]:
        """Calibrate base-to-target transformation."""
        # Get Calibration Data
        if cam_id is not None:
            readings = self._readings_dict[cam_id]
            gripper_poses = self._pose_dict[cam_id]
            self._curr_cam_id = cam_id

        # Get Target2Cam Transformation
        if target2cam_results is None:
            target2cam_results = self.calculate_target_to_cam(readings)
        if target2cam_results is None:
            return None

        R_target2cam, t_target2cam, successes = target2cam_results
        gripper_poses = np.array(gripper_poses)[successes]

        # Calculate Appropriate Transformations
        t_gripper2base = [np.array(pose[:3]) for pose in gripper_poses]
        R_gripper2base = [
            R.from_euler("xyz", pose[3:6]).as_matrix()
            for pose in gripper_poses
        ]

        # Perform Calibration
        rmat, pos = cv2.calibrateHandEye(
            R_gripper2base=R_target2cam,
            t_gripper2base=t_target2cam,
            R_target2cam=R_gripper2base,
            t_target2cam=t_gripper2base,
            method=4,
        )

        # Return Pose
        pos = pos.flatten()
        angle = R.from_matrix(rmat).as_euler("xyz")
        pose = np.concatenate([pos, angle])

        return pose

    def _calculate_gripper_to_base(
        self,
        train_readings: List,
        train_gripper_poses: List,
        eval_readings: Optional[List] = None
    ) -> Optional[Tuple]:
        """Calculate gripper-to-base transformation for evaluation."""
        if eval_readings is None:
            eval_readings = train_readings

        # Get Eval Target2Cam Transformations
        eval_results = self.calculate_target_to_cam(eval_readings, train=False)
        if eval_results is None:
            return None
        eval_R_target2cam, eval_t_target2cam, eval_successes = eval_results
        rmats, tvecs = [], []

        # Get Train Target2Cam Transformations
        train_results = self.calculate_target_to_cam(train_readings)
        if train_results is None:
            return None

        # Use Training Data For Calibrations
        base2target = self._calibrate_base_to_target(
            gripper_poses=train_gripper_poses,
            target2cam_results=train_results
        )
        R_base2target = R.from_euler("xyz", base2target[3:]).as_matrix()
        t_base2target = np.array(base2target[:3])

        cam2gripper = self._calibrate_cam_to_gripper(
            gripper_poses=train_gripper_poses,
            target2cam_results=train_results
        )
        R_cam2gripper = R.from_euler("xyz", cam2gripper[3:]).as_matrix()
        t_cam2gripper = np.array(cam2gripper[:3])

        # Calculate Gripper2Base
        for i in range(len(eval_R_target2cam)):
            R_base2cam = eval_R_target2cam[i] @ R_base2target
            t_base2cam = eval_R_target2cam[i] @ t_base2target + eval_t_target2cam[i]

            R_base2gripper = R_cam2gripper @ R_base2cam
            t_base2gripper = R_cam2gripper @ t_base2cam + t_cam2gripper

            R_gripper2base = R.from_matrix(R_base2gripper).inv().as_matrix()
            t_gripper2base = -R_gripper2base @ t_base2gripper

            rmats.append(R_gripper2base)
            tvecs.append(t_gripper2base)

        # Return Poses
        eulers = np.array([R.from_matrix(rmat).as_euler("xyz") for rmat in rmats])
        eval_poses = np.concatenate([np.array(tvecs), eulers], axis=1)

        return eval_poses, eval_successes

    def is_calibration_accurate(self, cam_id: str) -> Tuple[bool, Dict]:
        """
        Check if calibration is accurate.
        
        Args:
            cam_id: Camera identifier
        
        Returns:
            Tuple of (success, accuracy_info)
        """
        # Set Camera
        self._curr_cam_id = cam_id

        # Split Into Train / Test
        readings = self._readings_dict[cam_id]
        if len(readings) == 0:
            return False, {"error": "No readings collected"}
        
        poses = np.array(self._pose_dict[cam_id])
        ind = np.random.choice(len(readings), size=len(readings), replace=False)
        num_train = int(len(readings) * self.train_percentage)

        train_ind, test_ind = ind[:num_train], ind[num_train:]
        train_poses, test_poses = poses[train_ind], poses[test_ind]
        train_readings = [readings[i] for i in train_ind]
        test_readings = [readings[i] for i in test_ind]

        # Calculate Approximate Gripper2Base Transformations
        results = self._calculate_gripper_to_base(
            train_readings, train_poses, eval_readings=test_readings
        )
        if results is None:
            return False, {"error": "Failed to calculate gripper-to-base transformation"}
        
        approx_poses, successes = results
        test_poses = np.array(test_poses)[successes]

        # Calculate Per Dimension Error
        pose_error = np.array([
            pose_diff(pose, approx_pose)
            for pose, approx_pose in zip(test_poses, approx_poses)
        ])
        lin_error = np.linalg.norm(pose_error[:, :3], axis=0) ** 2 / pose_error.shape[0]
        rot_error = np.linalg.norm(pose_error[:, 3:6], axis=0) ** 2 / pose_error.shape[0]

        # Check Calibration Error
        lin_success = np.all(lin_error < self.lin_error_threshold)
        rot_success = np.all(rot_error < self.rot_error_threshold)

        accuracy_info = {
            "pose_std": poses.std(axis=0).tolist(),
            "lin_error": lin_error.tolist(),
            "rot_error": rot_error.tolist(),
            "num_samples": len(readings),
            "num_valid_samples": len(successes),
            "lin_success": bool(lin_success),
            "rot_success": bool(rot_success),
        }

        print(f"Pose Std: {poses.std(axis=0)}")
        print(f"Lin Error: {lin_error}")
        print(f"Rot Error: {rot_error}")

        return lin_success and rot_success, accuracy_info

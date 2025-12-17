from copy import deepcopy
import threading
import queue

import gym
import numpy as np
import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time

from role_ros2.calibration.calibration_utils import load_calibration_info
from role_ros2.camera_utils.info import camera_type_dict
from role_ros2.misc.parameters import hand_camera_id, nuc_ip
from role_ros2.misc.time import time_ms
from role_ros2.misc.transformations import change_pose_frame
from role_ros2.robot import FrankaRobot


class ROS2CameraSubscriber(Node):
    """ROS2 Node for subscribing to camera topics with approximate time synchronization"""
    
    def __init__(self, camera_topics_config, sync_slop=0.1):
        super().__init__('ros2_camera_subscriber')
        self.cv_bridge = CvBridge()
        self.sync_slop = sync_slop
        
        # Store camera topics configuration
        # Format: {camera_id: {'rgb_topic': '/camera/rgb/image_raw', 'depth_topic': '/camera/depth/image_raw', ...}}
        self.camera_topics_config = camera_topics_config
        self.camera_dict = {}
        
        # Synchronized data storage
        self.latest_data = {}
        self.data_lock = threading.Lock()
        self.data_queue = queue.Queue(maxsize=10)
        
        # Create subscribers for each camera
        self.subscribers = []
        self.sync = None
        
        self._setup_subscribers()
    
    def _setup_subscribers(self):
        """Setup subscribers and approximate time synchronizer"""
        all_subs = []
        
        for cam_id, topics in self.camera_topics_config.items():
            # Subscribe to RGB image
            if 'rgb_topic' in topics:
                rgb_sub = Subscriber(self, Image, topics['rgb_topic'])
                all_subs.append(rgb_sub)
                self.camera_dict[cam_id] = {'rgb_sub': rgb_sub}
            
            # Subscribe to depth image
            if 'depth_topic' in topics:
                depth_sub = Subscriber(self, Image, topics['depth_topic'])
                all_subs.append(depth_sub)
                if cam_id not in self.camera_dict:
                    self.camera_dict[cam_id] = {}
                self.camera_dict[cam_id]['depth_sub'] = depth_sub
            
            # Subscribe to camera info
            if 'camera_info_topic' in topics:
                info_sub = self.create_subscription(
                    CameraInfo, topics['camera_info_topic'], 
                    lambda msg, cid=cam_id: self._camera_info_callback(msg, cid), 10
                )
                self.camera_dict[cam_id]['camera_info'] = None
        
        # Create approximate time synchronizer if we have multiple topics
        if len(all_subs) > 1:
            self.sync = ApproximateTimeSynchronizer(
                all_subs, queue_size=10, slop=self.sync_slop
            )
            self.sync.registerCallback(self._sync_callback)
        elif len(all_subs) == 1:
            # Single subscriber, no sync needed
            all_subs[0].registerCallback(self._single_callback)
    
    def _camera_info_callback(self, msg, camera_id):
        """Callback for camera info"""
        with self.data_lock:
            if camera_id not in self.latest_data:
                self.latest_data[camera_id] = {}
            self.latest_data[camera_id]['camera_info'] = msg
    
    def _sync_callback(self, *msgs):
        """Callback for synchronized messages"""
        # Process synchronized messages
        msg_idx = 0
        synced_data = {}
        timestamp = None
        
        for cam_id, topics in self.camera_topics_config.items():
            cam_data = {}
            
            if 'rgb_topic' in topics and msg_idx < len(msgs):
                rgb_msg = msgs[msg_idx]
                cam_data['rgb'] = self.cv_bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
                if timestamp is None:
                    timestamp = rgb_msg.header.stamp
                msg_idx += 1
            
            if 'depth_topic' in topics and msg_idx < len(msgs):
                depth_msg = msgs[msg_idx]
                cam_data['depth'] = self.cv_bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                if timestamp is None:
                    timestamp = depth_msg.header.stamp
                msg_idx += 1
            
            if cam_data:
                synced_data[cam_id] = cam_data
        
        # Store synchronized data
        with self.data_lock:
            self.latest_data.update(synced_data)
            if timestamp:
                self.latest_data['_timestamp'] = timestamp
        
        # Add to queue (non-blocking)
        try:
            self.data_queue.put_nowait(synced_data)
        except queue.Full:
            # Remove oldest if queue is full
            try:
                self.data_queue.get_nowait()
                self.data_queue.put_nowait(synced_data)
            except queue.Empty:
                pass
    
    def _single_callback(self, msg):
        """Callback for single subscriber"""
        # Handle single camera case
        pass
    
    def read_cameras(self):
        """Read latest synchronized camera data"""
        full_obs_dict = {}
        full_timestamp_dict = {}
        
        with self.data_lock:
            # Get latest synchronized data
            synced_data = {}
            for cam_id in self.camera_topics_config.keys():
                if cam_id in self.latest_data and cam_id != '_timestamp':
                    synced_data[cam_id] = self.latest_data[cam_id].copy()
            
            timestamp = self.latest_data.get('_timestamp', None)
        
        # Convert to expected format
        for cam_id, cam_data in synced_data.items():
            if 'rgb' in cam_data:
                full_obs_dict[f'{cam_id}_image'] = cam_data['rgb']
            if 'depth' in cam_data:
                full_obs_dict[f'{cam_id}_depth'] = cam_data['depth']
        
        if timestamp:
            # Convert ROS2 Time to milliseconds
            timestamp_ms = int(timestamp.sec * 1000 + timestamp.nanosec / 1e6)
            full_timestamp_dict['cameras'] = timestamp_ms
        
        return full_obs_dict, full_timestamp_dict
    
    def get_intrinsics(self, camera_id):
        """Get camera intrinsics for a specific camera"""
        with self.data_lock:
            if camera_id in self.latest_data and 'camera_info' in self.latest_data[camera_id]:
                info = self.latest_data[camera_id]['camera_info']
                if info:
                    K = np.array(info.k).reshape(3, 3)
                    return {'cameraMatrix': K}
        return None


class RobotEnv(gym.Env):
    def __init__(self, action_space="cartesian_velocity", gripper_action_space=None, 
                 camera_kwargs={}, do_reset=True, arm_id="fr3", controller_name="fr3_arm_controller",
                 camera_topics_config=None, use_ros2_cameras=True):
        # Initialize Gym Environment
        super().__init__()
        
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Define Action Space #
        assert action_space in ["cartesian_position", "joint_position", "cartesian_velocity", "joint_velocity"]
        self.action_space = action_space
        self.gripper_action_space = gripper_action_space
        self.check_action_range = "velocity" in action_space
        
        # Robot Configuration
        self.reset_joints = np.array([0, -1 / 5 * np.pi, 0, -4 / 5 * np.pi, 0, 3 / 5 * np.pi, 0.0])
        self.randomize_low = np.array([-0.1, -0.2, -0.1, -0.3, -0.3, -0.3])
        self.randomize_high = np.array([0.1, 0.2, 0.1, 0.3, 0.3, 0.3])
        self.DoF = 7 if ("cartesian" in action_space) else 8
        self.control_hz = 15
        
        # Initialize Robot
        if nuc_ip is None or nuc_ip == "":
            self._robot = FrankaRobot(arm_id=arm_id, controller_name=controller_name)
            # Run robot node in separate thread
            self._robot_executor = rclpy.executors.SingleThreadedExecutor()
            self._robot_executor.add_node(self._robot)
            self._robot_thread = threading.Thread(target=self._robot_executor.spin, daemon=True)
            self._robot_thread.start()
        else:
            # ServerInterface not available in role-ros2, use ROS2 robot instead
            self._robot = FrankaRobot(arm_id=arm_id, controller_name=controller_name)
            self._robot_executor = rclpy.executors.SingleThreadedExecutor()
            self._robot_executor.add_node(self._robot)
            self._robot_thread = threading.Thread(target=self._robot_executor.spin, daemon=True)
            self._robot_thread.start()
        
        # Create Cameras
        self.use_ros2_cameras = use_ros2_cameras
        if use_ros2_cameras and camera_topics_config:
            self.camera_reader = ROS2CameraSubscriber(camera_topics_config)
            # Run camera subscriber in separate thread
            self._camera_executor = rclpy.executors.SingleThreadedExecutor()
            self._camera_executor.add_node(self.camera_reader)
            self._camera_thread = threading.Thread(target=self._camera_executor.spin, daemon=True)
            self._camera_thread.start()
        else:
            # MultiCameraWrapper not available in role-ros2, use ROS2 cameras instead
            if camera_topics_config is None:
                raise ValueError("camera_topics_config must be provided when use_ros2_cameras=False")
            self.camera_reader = ROS2CameraSubscriber(camera_topics_config)
            self._camera_executor = rclpy.executors.SingleThreadedExecutor()
            self._camera_executor.add_node(self.camera_reader)
            self._camera_thread = threading.Thread(target=self._camera_executor.spin, daemon=True)
            self._camera_thread.start()
        
        self.calibration_dict = load_calibration_info()
        self.camera_type_dict = camera_type_dict
        
        # Reset Robot
        if do_reset:
            self.reset()
    
    def step(self, action):
        # Check Action
        assert len(action) == self.DoF
        if self.check_action_range:
            assert (action.max() <= 1) and (action.min() >= -1)
        
        # Update Robot
        action_info = self.update_robot(
            action,
            action_space=self.action_space,
            gripper_action_space=self.gripper_action_space,
        )
        
        # Return Action Info
        return action_info
    
    def reset(self, randomize=False):
        self._robot.update_gripper(0, velocity=False, blocking=True)
        
        if randomize:
            noise = np.random.uniform(low=self.randomize_low, high=self.randomize_high)
        else:
            noise = None
        
        self._robot.update_joints(self.reset_joints, velocity=False, blocking=True, cartesian_noise=noise)
    
    def update_robot(self, action, action_space="cartesian_velocity", gripper_action_space=None, blocking=False):
        action_info = self._robot.update_command(
            action,
            action_space=action_space,
            gripper_action_space=gripper_action_space,
            blocking=blocking
        )
        return action_info
    
    def create_action_dict(self, action):
        return self._robot.create_action_dict(action)
    
    def read_cameras(self):
        return self.camera_reader.read_cameras()
    
    def get_state(self):
        read_start = time_ms()
        state_dict, timestamp_dict = self._robot.get_robot_state()
        timestamp_dict["read_start"] = read_start
        timestamp_dict["read_end"] = time_ms()
        return state_dict, timestamp_dict
    
    def get_camera_extrinsics(self, state_dict):
        # Adjust gripper camera by current pose
        extrinsics = deepcopy(self.calibration_dict)
        for cam_id in self.calibration_dict:
            if hand_camera_id not in cam_id:
                continue
            gripper_pose = state_dict["cartesian_position"]
            extrinsics[cam_id + "_gripper_offset"] = extrinsics[cam_id]
            extrinsics[cam_id] = change_pose_frame(extrinsics[cam_id], gripper_pose)
        return extrinsics
    
    def get_observation(self):
        obs_dict = {"timestamp": {}}
        
        # Robot State #
        state_dict, timestamp_dict = self.get_state()
        obs_dict["robot_state"] = state_dict
        obs_dict["timestamp"]["robot_state"] = timestamp_dict
        
        # Camera Readings (synchronized via approximate time synchronizer) #
        camera_obs, camera_timestamp = self.read_cameras()
        obs_dict.update(camera_obs)
        obs_dict["timestamp"]["cameras"] = camera_timestamp
        
        # Camera Info #
        obs_dict["camera_type"] = deepcopy(self.camera_type_dict)
        extrinsics = self.get_camera_extrinsics(state_dict)
        obs_dict["camera_extrinsics"] = extrinsics
        
        intrinsics = {}
        if self.use_ros2_cameras and hasattr(self.camera_reader, 'get_intrinsics'):
            for cam_id in self.camera_reader.camera_topics_config.keys():
                cam_intr = self.camera_reader.get_intrinsics(cam_id)
                if cam_intr:
                    intrinsics[cam_id] = cam_intr["cameraMatrix"]
        else:
            for cam in self.camera_reader.camera_dict.values():
                cam_intr_info = cam.get_intrinsics()
                for (full_cam_id, info) in cam_intr_info.items():
                    intrinsics[full_cam_id] = info["cameraMatrix"]
        obs_dict["camera_intrinsics"] = intrinsics
        
        return obs_dict


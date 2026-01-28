"""
Trajectory Writer for role_ros2.

This module provides TrajectoryWriter class for saving robot trajectory data to HDF5 files.
Adapted from droid.trajectory_utils.trajectory_writer.
"""

import io
import os
import tempfile
from collections import defaultdict
from copy import deepcopy
from queue import Empty, Queue

import h5py
import imageio
import numpy as np
import json

from role_ros2.misc.subprocess_utils import run_threaded_command


def write_dict_to_hdf5(hdf5_file, data_dict, keys_to_ignore=["image", "depth", "pointcloud"]):
    """
    Write a dictionary to HDF5 file.
    
    Args:
        hdf5_file: HDF5 file or group to write to
        data_dict: Dictionary of data to write
        keys_to_ignore: List of keys to skip (e.g., image data handled separately)
    """
    for key in data_dict.keys():
        # Pass Over Specified Keys
        if key in keys_to_ignore:
            continue

        # Examine Data
        curr_data = data_dict[key]
        
        # Skip None values - HDF5 does not support NoneType
        if curr_data is None:
            continue
        
        # Convert list to numpy array if needed
        if type(curr_data) == list:
            curr_data = np.array(curr_data)
            # Check if resulting array has object dtype (contains non-numeric data)
            # Object dtype arrays cannot be saved to HDF5, skip this key
            if curr_data.dtype == object:
                continue
        
        dtype = type(curr_data)

        # Unwrap If Dictionary
        if dtype == dict:
            if key not in hdf5_file:
                hdf5_file.create_group(key)
            write_dict_to_hdf5(hdf5_file[key], curr_data)
            continue

        # Make Room For Data
        if key not in hdf5_file:
            if dtype != np.ndarray:
                dshape = ()
                # Use numpy dtype conversion for scalar types
                if dtype == int:
                    hdf5_dtype = np.int64
                elif dtype == float:
                    hdf5_dtype = np.float64
                elif dtype == bool:
                    hdf5_dtype = np.bool_
                else:
                    # For other scalar types, try to convert to numpy array first
                    try:
                        curr_data = np.array(curr_data)
                        if curr_data.dtype == object:
                            # Cannot save object dtype, skip
                            continue
                        hdf5_dtype = curr_data.dtype
                        dshape = curr_data.shape
                    except (TypeError, ValueError):
                        # Cannot convert, skip this key
                        continue
            else:
                hdf5_dtype, dshape = curr_data.dtype, curr_data.shape
                # Double-check: skip object dtype arrays
                if hdf5_dtype == object:
                    continue
            
            hdf5_file.create_dataset(key, (1, *dshape), maxshape=(None, *dshape), dtype=hdf5_dtype)
        else:
            hdf5_file[key].resize(hdf5_file[key].shape[0] + 1, axis=0)

        # Save Data
        hdf5_file[key][-1] = curr_data


class TrajectoryWriter:
    """
    Writer for trajectory HDF5 files.
    
    Provides asynchronous writing of trajectory timesteps to HDF5 format.
    Optionally saves video and depth data.
    """
    
    def __init__(self, filepath, metadata=None, exists_ok=False, save_images=False, save_depths=False):
        """
        Initialize trajectory writer.
        
        Args:
            filepath: Path to save the HDF5 file
            metadata: Optional metadata dictionary to save
            exists_ok: If True, allow overwriting existing files
            save_images: If True, save RGB image data as MP4 video
            save_depths: If True, save depth data as PNG-in-HDF5 (requires save_images=True)
        
        Note:
            save_depths=True requires save_images=True. Depth-only saving is not supported.
        """
        # Validate: save_depths requires save_images
        if save_depths and not save_images:
            raise ValueError("save_depths=True requires save_images=True. Cannot save depth without images.")
        
        assert (not os.path.isfile(filepath)) or exists_ok
        self._filepath = filepath
        self._save_images = save_images
        self._save_depths = save_depths
        self._hdf5_file = h5py.File(filepath, "w")
        self._queue_dict = defaultdict(Queue)
        self._video_writers = {}
        self._video_files = {}
        self._video_buffers = {}  # Track video buffers
        self._depth_datasets = {}  # Track depth HDF5 datasets (lossless compressed)
        self._depth_frame_counts = {}  # Track frame counts for each depth dataset
        self._open = True

        # Add Metadata
        if metadata is not None:
            self._update_metadata(metadata)

        # Start HDF5 Writer Thread
        def hdf5_writer(data):
            return write_dict_to_hdf5(self._hdf5_file, data)

        run_threaded_command(self._write_from_queue, args=(hdf5_writer, self._queue_dict["hdf5"]))

    def write_timestep(self, timestep):
        """
        Write a single timestep to the trajectory file.
        
        Args:
            timestep: Dictionary containing observation and action data
        """
        if self._save_images:
            self._update_video_files(timestep)
        self._queue_dict["hdf5"].put(timestep)

    def _update_metadata(self, metadata):
        """Update file metadata with type safety for HDF5/NumPy 2.0."""
        metadata = {k: v for k, v in metadata.items() if k != "trajectory_id"}
        for key, value in metadata.items():
            # 1. Handle None (HDF5 does not support NoneType, store as string)
            if value is None:
                self._hdf5_file.attrs[key] = "None"
                continue
            
            # 2. Handle boolean values
            # [Important] Must check before int, since in Python isinstance(True, int) is True
            if isinstance(value, (bool, np.bool_)):
                self._hdf5_file.attrs[key] = np.int64(value)  # Store as 0 or 1
            
            # 3. Handle integers (combine Python int and NumPy integer)
            elif isinstance(value, (int, np.integer)):
                self._hdf5_file.attrs[key] = np.int64(value)
            
            # 4. Handle floating point numbers (combine Python float and NumPy floating)
            elif isinstance(value, (float, np.floating)):
                self._hdf5_file.attrs[key] = np.float64(value)
            
            # 5. Handle natively supported types (string, bytes, NumPy array)
            elif isinstance(value, (str, bytes, np.ndarray)):
                self._hdf5_file.attrs[key] = value
            
            # 6. Handle complex structures (list, dict, tuple) -> convert to JSON string
            # Note: deepcopy is often ineffective for HDF5 attrs as HDF5 cannot directly store dicts
            elif isinstance(value, (list, dict, tuple)):
                try:
                    self._hdf5_file.attrs[key] = json.dumps(value)
                except (TypeError, ValueError):
                    # If the structure contains unserializable objects (such as functions), fall back to string
                    self._hdf5_file.attrs[key] = str(value)
            
            # 7. Fallback: all other unknown types are converted to string
            else:
                self._hdf5_file.attrs[key] = str(value)

    def _write_from_queue(self, writer, queue):
        """Background thread function for writing data from queue."""
        while self._open:
            try:
                data = queue.get(timeout=1)
            except Empty:
                continue
            writer(data)
            queue.task_done()

    def _update_video_files(self, timestep):
        """
        Handle video file writing for image and depth data.
        
        Args:
            timestep: Dictionary containing observation data with images and/or depth
        """
        # Check if observation exists
        if "observation" not in timestep:
            return
        obs = timestep["observation"]

        # Handle RGB image data (MP4 video)
        if "image" in obs:
            image_dict = obs["image"]

            for video_id in list(image_dict.keys()):
                # Get Frame
                img = image_dict[video_id]
                del image_dict[video_id]

                # Create Writer And Buffer
                if video_id not in self._video_buffers:
                    filename = self.create_video_file(video_id, ".mp4")
                    self._video_writers[video_id] = imageio.get_writer(filename, macro_block_size=1)
                    self._video_buffers[video_id] = True
                    run_threaded_command(
                        self._write_from_queue, 
                        args=(self._video_writers[video_id].append_data, self._queue_dict[video_id])
                    )

                # Add Image To Queue
                self._queue_dict[video_id].put(img)

            # Remove empty image dict
            if not image_dict:
                del obs["image"]
        
        # Handle depth data - save as PNG-in-HDF5 (lossless uint16 compression, more efficient than GZIP arrays)
        # Only process if save_depths is enabled
        if self._save_depths and "depth" in obs:
            depth_dict = obs["depth"]

            for depth_id in list(depth_dict.keys()):
                # Get Frame
                depth_img = depth_dict[depth_id]
                del depth_dict[depth_id]

                # Create HDF5 dataset path
                dataset_key = f"depth_{depth_id}"

                # Create dataset if not exists
                if dataset_key not in self._depth_datasets:
                    # Create group structure
                    if "observations" not in self._hdf5_file:
                        self._hdf5_file.create_group("observations")
                    if "depth" not in self._hdf5_file["observations"]:
                        self._hdf5_file["observations"].create_group("depth")
                    
                    # Create variable-length uint8 dataset for PNG-encoded depth images
                    # Using vlen=np.uint8 to store binary data (supports NULL bytes)
                    # This is more efficient than GZIP-compressed uint16 arrays
                    dt = h5py.special_dtype(vlen=np.uint8)
                    self._hdf5_file["observations"]["depth"].create_dataset(
                        depth_id,
                        shape=(1,),
                        maxshape=(None,),
                        dtype=dt,
                        chunks=(1,),  # Chunk by frame
                        compression="gzip",  # Compress the byte arrays
                        compression_opts=4  # Compression level 1-9 (4 is balanced)
                    )
                    self._depth_datasets[dataset_key] = self._hdf5_file["observations"]["depth"][depth_id]
                    self._depth_frame_counts[dataset_key] = 0

                # Encode depth frame as PNG bytes (lossless uint16 compression)
                # PNG is much more efficient than raw GZIP for uint16 depth images
                png_bytes = io.BytesIO()
                imageio.imwrite(png_bytes, depth_img, format='PNG', compress_level=3)
                png_bytes.seek(0)
                encoded_bytes = png_bytes.read()
                
                # Convert bytes to uint8 numpy array for HDF5 storage
                encoded_array = np.frombuffer(encoded_bytes, dtype=np.uint8)

                # Write depth frame to dataset
                dataset = self._depth_datasets[dataset_key]
                frame_idx = self._depth_frame_counts[dataset_key]
                
                # Resize dataset if needed
                if frame_idx >= dataset.shape[0]:
                    dataset.resize(frame_idx + 1, axis=0)
                
                dataset[frame_idx] = encoded_array
                self._depth_frame_counts[dataset_key] = frame_idx + 1

            # Remove empty depth dict
            if not depth_dict:
                del obs["depth"]

    def create_video_file(self, video_id, suffix):
        """Create a temporary video file for storing video data."""
        temp_file = tempfile.NamedTemporaryFile(suffix=suffix, delete=True)
        self._video_files[video_id] = temp_file
        return temp_file.name

    def _create_concatenated_preview(self, video_paths_dict, output_path):
        """
        Create a concatenated preview video from multiple camera videos.
        
        Args:
            video_paths_dict: Dictionary mapping camera_id to video file path
            output_path: Path to save the concatenated preview video
        """
        # Read all videos
        video_readers = {}
        frame_counts = {}
        
        try:
            # Open all video readers
            for camera_id, video_path in video_paths_dict.items():
                reader = imageio.get_reader(video_path)
                video_readers[camera_id] = reader
                frame_counts[camera_id] = reader.count_frames()
            
            # Get minimum frame count (to ensure all videos are same length)
            min_frames = min(frame_counts.values()) if frame_counts else 0
            
            if min_frames == 0:
                return  # No frames to process
            
            # Get video properties from first video
            first_reader = list(video_readers.values())[0]
            first_frame = first_reader.get_data(0)
            frame_height, frame_width = first_frame.shape[:2]
            
            # Create concatenated video writer
            # Arrange cameras horizontally (side by side)
            num_cameras = len(video_readers)
            concatenated_width = frame_width * num_cameras
            concatenated_height = frame_height
            
            writer = imageio.get_writer(output_path, fps=first_reader.get_meta_data().get('fps', 30))
            
            # Process each frame
            for frame_idx in range(min_frames):
                # Read frame from each camera
                frames = []
                for camera_id in sorted(video_readers.keys()):  # Sort for consistent order
                    reader = video_readers[camera_id]
                    frame = reader.get_data(frame_idx)
                    frames.append(frame)
                
                # Concatenate frames horizontally
                concatenated_frame = np.hstack(frames)
                
                # Write concatenated frame
                writer.append_data(concatenated_frame)
            
            # Close writer and readers
            writer.close()
            for reader in video_readers.values():
                reader.close()
                
        except Exception as e:
            # Clean up on error
            for reader in video_readers.values():
                try:
                    reader.close()
                except:
                    pass
            raise

    def close(self, metadata=None):
        """
        Close the trajectory file.
        
        Args:
            metadata: Optional additional metadata to save before closing
        """
        # Add Metadata
        if metadata is not None:
            self._update_metadata(metadata)

        # Finish Remaining Jobs
        for queue in self._queue_dict.values():
            queue.join()

        # Close Video Writers (this finalizes the video files)
        for video_id in self._video_writers:
            self._video_writers[video_id].close()

        # Get trajectory file directory for saving preview videos
        traj_dir = os.path.dirname(self._filepath)
        
        # Create mp4 subdirectory for preview videos
        mp4_dir = None
        if self._save_images and traj_dir:
            try:
                mp4_dir = os.path.join(traj_dir, "mp4")
                os.makedirs(mp4_dir, exist_ok=True)
            except Exception:
                mp4_dir = None  # Silently fail, will skip preview video saving

        # Store video file paths for potential concatenation
        saved_video_paths = {}

        # Save Serialized Videos and create preview MP4 files
        for video_id in self._video_files:
            # Create Folder
            if "observations" not in self._hdf5_file:
                self._hdf5_file.create_group("observations")
            if "videos" not in self._hdf5_file["observations"]:
                self._hdf5_file["observations"].create_group("videos")

            # Read video file content before closing
            video_file = self._video_files[video_id]
            video_file_path = video_file.name
            
            # Read video content from file (since file is still open)
            video_file.seek(0)
            video_content = video_file.read()
            
            # Convert to numpy array for HDF5 storage
            serialized_video = np.asarray(bytearray(video_content))

            # Save Data to HDF5
            self._hdf5_file["observations"]["videos"].create_dataset(video_id, data=serialized_video)
            
            # Save individual camera MP4 file to mp4/ subdirectory
            if mp4_dir:
                try:
                    # Save each camera video: mp4/{camera_id}.mp4
                    camera_video_path = os.path.join(mp4_dir, f"{video_id}.mp4")
                    with open(camera_video_path, 'wb') as camera_file:
                        camera_file.write(video_content)
                    saved_video_paths[video_id] = camera_video_path
                except Exception:
                    # Silently fail if preview video cannot be saved
                    pass
            
            # Close and clean up temp file
            video_file.close()
            try:
                os.unlink(video_file_path)
            except Exception:
                pass

        # Create concatenated preview video if multiple cameras exist
        if mp4_dir and len(saved_video_paths) > 1:
            try:
                self._create_concatenated_preview(saved_video_paths, os.path.join(mp4_dir, "preview.mp4"))
            except Exception:
                # Silently fail if concatenation cannot be done
                pass
        elif mp4_dir and len(saved_video_paths) == 1:
            # Single camera: rename the only video to preview.mp4
            try:
                single_camera_path = list(saved_video_paths.values())[0]
                preview_path = os.path.join(mp4_dir, "preview.mp4")
                if single_camera_path != preview_path:
                    # Only rename if different
                    if os.path.exists(preview_path):
                        os.remove(preview_path)
                    os.rename(single_camera_path, preview_path)
            except Exception:
                # If rename fails, try copying instead
                try:
                    single_camera_path = list(saved_video_paths.values())[0]
                    preview_path = os.path.join(mp4_dir, "preview.mp4")
                    with open(single_camera_path, 'rb') as src, open(preview_path, 'wb') as dst:
                        dst.write(src.read())
                except Exception:
                    pass

        # Close File
        self._hdf5_file.close()
        self._open = False

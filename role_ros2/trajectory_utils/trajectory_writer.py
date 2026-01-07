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

        # Close Video Writers
        for video_id in self._video_writers:
            self._video_writers[video_id].close()

        # Save Serialized Videos
        for video_id in self._video_files:
            # Create Folder
            if "observations" not in self._hdf5_file:
                self._hdf5_file.create_group("observations")
            if "videos" not in self._hdf5_file["observations"]:
                self._hdf5_file["observations"].create_group("videos")

            # Get Serialized Video
            self._video_files[video_id].seek(0)
            serialized_video = np.asarray(bytearray(self._video_files[video_id].read()))

            # Save Data
            self._hdf5_file["observations"]["videos"].create_dataset(video_id, data=serialized_video)
            self._video_files[video_id].close()
            # Clean up temp file
            try:
                os.unlink(self._video_files[video_id].name)
            except Exception:
                pass

        # Close File
        self._hdf5_file.close()
        self._open = False

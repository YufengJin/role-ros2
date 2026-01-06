"""
Trajectory Writer for role_ros2.

This module provides TrajectoryWriter class for saving robot trajectory data to HDF5 files.
Adapted from droid.trajectory_utils.trajectory_writer.
"""

import os
import tempfile
from collections import defaultdict
from copy import deepcopy
from queue import Empty, Queue

import h5py
import imageio
import numpy as np

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
        if type(curr_data) == list:
            curr_data = np.array(curr_data)
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
            else:
                dtype, dshape = curr_data.dtype, curr_data.shape
            hdf5_file.create_dataset(key, (1, *dshape), maxshape=(None, *dshape), dtype=dtype)
        else:
            hdf5_file[key].resize(hdf5_file[key].shape[0] + 1, axis=0)

        # Save Data
        hdf5_file[key][-1] = curr_data


class TrajectoryWriter:
    """
    Writer for trajectory HDF5 files.
    
    Provides asynchronous writing of trajectory timesteps to HDF5 format.
    Optionally saves video data.
    """
    
    def __init__(self, filepath, metadata=None, exists_ok=False, save_images=True):
        """
        Initialize trajectory writer.
        
        Args:
            filepath: Path to save the HDF5 file
            metadata: Optional metadata dictionary to save
            exists_ok: If True, allow overwriting existing files
            save_images: If True, save image data as video
        """
        assert (not os.path.isfile(filepath)) or exists_ok
        self._filepath = filepath
        self._save_images = save_images
        self._hdf5_file = h5py.File(filepath, "w")
        self._queue_dict = defaultdict(Queue)
        self._video_writers = {}
        self._video_files = {}
        self._video_buffers = {}  # Track video buffers
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
        """Update file metadata."""
        for key in metadata:
            value = metadata[key]
            # Handle non-serializable types
            if isinstance(value, bool):
                self._hdf5_file.attrs[key] = int(value)
            elif isinstance(value, (int, float, str)):
                self._hdf5_file.attrs[key] = value
            elif isinstance(value, np.ndarray):
                self._hdf5_file.attrs[key] = value
            elif value is None:
                self._hdf5_file.attrs[key] = "None"
            else:
                try:
                    self._hdf5_file.attrs[key] = deepcopy(value)
                except TypeError:
                    # Skip non-serializable values
                    pass

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
        Handle video file writing for image data.
        
        Args:
            timestep: Dictionary containing observation data with images
        """
        # Check if observation has images
        if "observation" not in timestep:
            return
        obs = timestep["observation"]
        if "image" not in obs:
            return
            
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

    def create_video_file(self, video_id, suffix):
        """Create a temporary video file for storing video data."""
        temp_file = tempfile.NamedTemporaryFile(suffix=suffix, delete=False)
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

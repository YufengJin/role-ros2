"""
Trajectory Reader for role_ros2.

This module provides TrajectoryReader class for reading saved trajectory data from HDF5 files.
Adapted from droid.trajectory_utils.trajectory_reader.
"""

import json
import tempfile

import h5py
import imageio
import numpy as np


def create_video_file(suffix=".mp4", byte_contents=None):
    """
    Create a temporary video file.
    
    Args:
        suffix: File suffix (default: ".mp4")
        byte_contents: Optional byte contents to write to file
    
    Returns:
        Filename of the temporary file
    """
    temp_file = tempfile.NamedTemporaryFile(suffix=suffix)
    filename = temp_file.name

    if byte_contents is not None:
        with open(filename, "wb") as binary_file:
            binary_file.write(byte_contents)

    return filename


def get_hdf5_length(hdf5_file, keys_to_ignore=[], strict=False):
    """
    Get the length of an HDF5 file (number of timesteps).
    
    Args:
        hdf5_file: HDF5 file or group
        keys_to_ignore: List of keys to skip
        strict: If True, assert all datasets have same length. 
                If False (default), use minimum length found.
    
    Returns:
        Length of the trajectory (minimum length across all datasets if not strict)
    """
    length = None
    lengths_found = []

    for key in hdf5_file.keys():
        if key in keys_to_ignore:
            continue

        curr_data = hdf5_file[key]
        if isinstance(curr_data, h5py.Group):
            curr_length = get_hdf5_length(curr_data, keys_to_ignore=keys_to_ignore, strict=strict)
            if curr_length is not None:
                lengths_found.append(curr_length)
        elif isinstance(curr_data, h5py.Dataset):
            curr_length = len(curr_data)
            lengths_found.append(curr_length)
        else:
            # Skip unknown types
            continue

        if length is None:
            length = curr_length
        elif strict:
            assert curr_length == length, f"Length mismatch: {curr_length} != {length}"
        else:
            # Use minimum length to ensure we don't read beyond bounds
            length = min(length, curr_length)

    # Return minimum length found, or None if no datasets
    if lengths_found:
        return min(lengths_found)
    return length


def _convert_hdf5_value(value):
    """
    Convert HDF5 attribute value back to Python type.
    
    This function handles the reverse conversion of _update_metadata in trajectory_writer.py:
    - JSON strings -> list/dict/tuple
    - "None" string -> None
    - numpy types -> native Python types (if scalar)
    
    Args:
        value: Value read from HDF5 attribute
    
    Returns:
        Converted Python value
    """
    # Handle numpy scalar types
    if isinstance(value, (np.integer, np.int64, np.int32, np.int16, np.int8)):
        return int(value)
    elif isinstance(value, (np.floating, np.float64, np.float32)):
        return float(value)
    elif isinstance(value, np.bool_):
        return bool(value)
    
    # Handle string types (may be JSON or "None")
    if isinstance(value, (str, bytes)):
        # Try to decode bytes to string
        if isinstance(value, bytes):
            try:
                value = value.decode('utf-8')
            except UnicodeDecodeError:
                return value  # Return as-is if not valid UTF-8
        
        # Check for "None" string
        if value == "None":
            return None
        
        # Try to parse as JSON (for list/dict/tuple that were serialized)
        if value.startswith(('[', '{', '(')) or value.startswith(('"', "'")):
            try:
                return json.loads(value)
            except (json.JSONDecodeError, ValueError):
                pass  # Not JSON, return as string
    
    # Handle numpy arrays (keep as numpy array)
    if isinstance(value, np.ndarray):
        return value
    
    # Return as-is for other types
    return value


def load_hdf5_to_dict(hdf5_file, index, keys_to_ignore=[]):
    """
    Load a single timestep from HDF5 file to dictionary.
    
    Args:
        hdf5_file: HDF5 file or group
        index: Timestep index to load
        keys_to_ignore: List of keys to skip
    
    Returns:
        Dictionary containing the timestep data
    """
    data_dict = {}

    for key in hdf5_file.keys():
        if key in keys_to_ignore:
            continue

        curr_data = hdf5_file[key]
        if isinstance(curr_data, h5py.Group):
            data_dict[key] = load_hdf5_to_dict(curr_data, index, keys_to_ignore=keys_to_ignore)
        elif isinstance(curr_data, h5py.Dataset):
            value = curr_data[index]
            # Convert numpy scalar to Python native type if needed
            # Note: Dataset values are typically numeric or arrays, not JSON strings
            # JSON strings are only used in attrs (metadata), handled by get_metadata()
            if isinstance(value, (np.integer, np.int64, np.int32, np.int16, np.int8)):
                data_dict[key] = int(value)
            elif isinstance(value, (np.floating, np.float64, np.float32)):
                data_dict[key] = float(value)
            elif isinstance(value, np.bool_):
                data_dict[key] = bool(value)
            else:
                # For arrays, strings, and other types, return as-is
                data_dict[key] = value
        else:
            raise ValueError

    return data_dict


class TrajectoryReader:
    """
    Reader for trajectory HDF5 files.
    
    Provides sequential or random access to trajectory timesteps.
    """
    
    def __init__(self, filepath, read_images=True):
        """
        Initialize trajectory reader.
        
        Args:
            filepath: Path to the HDF5 trajectory file
            read_images: If True, attempt to read video data
        """
        self._hdf5_file = h5py.File(filepath, "r")
        is_video_folder = "observations/videos" in self._hdf5_file
        self._read_images = read_images and is_video_folder
        self._length = get_hdf5_length(self._hdf5_file)
        self._video_readers = {}
        self._index = 0
        self._metadata = None  # Cache for metadata

    def length(self):
        """Get trajectory length."""
        return self._length
    
    def get_metadata(self):
        """
        Get file-level metadata with proper type conversion.
        
        This method reads HDF5 attributes and converts them back to their original types:
        - JSON strings -> list/dict/tuple
        - "None" string -> None
        - numpy types -> native Python types
        
        Returns:
            Dictionary containing file metadata
        """
        if self._metadata is not None:
            return self._metadata
        
        metadata = {}
        for key in self._hdf5_file.attrs.keys():
            value = self._hdf5_file.attrs[key]
            metadata[key] = _convert_hdf5_value(value)
        
        self._metadata = metadata
        return metadata

    def read_timestep(self, index=None, keys_to_ignore=[]):
        """
        Read a single timestep from the trajectory.
        
        Args:
            index: Timestep index (None for sequential reading)
            keys_to_ignore: List of keys to skip
        
        Returns:
            Dictionary containing the timestep data
        """
        # Make Sure We Read Within Range
        if index is None:
            index = self._index
        else:
            assert not self._read_images
            self._index = index
        assert index < self._length

        # Load Low Dimensional Data
        keys_to_ignore = [*keys_to_ignore.copy(), "videos"]
        timestep = load_hdf5_to_dict(self._hdf5_file, self._index, keys_to_ignore=keys_to_ignore)

        # Load High Dimensional Data
        if self._read_images:
            camera_obs = self._uncompress_images()
            timestep["observations"]["image"] = camera_obs

        # Increment Read Index
        self._index += 1

        return timestep

    def _uncompress_images(self):
        """
        Uncompress images from video data.
        
        WARNING: This function has not been fully tested.
        """
        video_folder = self._hdf5_file["observations/videos"]
        camera_obs = {}

        for video_id in video_folder:
            # Create Video Reader If One Hasn't Been Made
            if video_id not in self._video_readers:
                serialized_video = video_folder[video_id]
                filename = create_video_file(byte_contents=serialized_video)
                self._video_readers[video_id] = imageio.get_reader(filename)

            # Read Next Frame
            camera_obs[video_id] = yield self._video_readers[video_id]

        return camera_obs

    def close(self):
        """Close the trajectory file."""
        self._hdf5_file.close()

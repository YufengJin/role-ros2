#!/usr/bin/env python3
"""
Control System Performance Analyzer for Robot Trajectory HDF5 Files.

This tool analyzes robotic control system performance from HDF5 trajectory files,
calculating Key Performance Indicators (KPIs) and generating professional visualizations.

Features:
- Rise Time calculation (10% to 90%)
- Percentage Overshoot calculation
- Steady-State Error (MAE) analysis
- Control Effort and Action Smoothness metrics
- Professional 3-subplot visualization
- Multi-dimensional data support (analyzes each dimension separately)

Usage:
    python3 control_analysis.py /path/to/trajectory.h5
    python3 control_analysis.py trajectory.h5 --dim 0  # Analyze specific dimension
    python3 control_analysis.py trajectory.h5 --control-hz 15  # Custom control frequency
    python3 control_analysis.py trajectory.h5 --output analysis.png  # Custom output path

Author: Role-ROS2 Team
"""

import argparse
import os
import sys
import warnings
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple, Union

import h5py
import numpy as np

# Suppress matplotlib warnings
warnings.filterwarnings('ignore', category=UserWarning)

# =============================================================================
# CONFIGURABLE DATASET KEYS
# =============================================================================
# These keys can be customized based on your HDF5 file structure.
# See docs/HDF5_TRAJECTORY_DATA_FORMAT.md for the full data structure.

# Target/Reference trajectory (what we want the robot to achieve)
DS_TARGET = "action/target_cartesian_position"

# Actual robot state (what the robot actually achieved)
DS_STATE = "observation/robot_state/cartesian_position"

# Control action sent to the robot
DS_ACTION = "action/cartesian_velocity"

# Alternative dataset keys for different action spaces
DS_TARGET_ALT = [
    "action/cartesian_position",
    "action/joint_position",
    "action/target_gripper_position",
]

DS_STATE_ALT = [
    "action/robot_state/cartesian_position",
    "action/robot_state/joint_positions",
    "observation/robot_state/joint_positions",
    "observation/robot_state/gripper_position",
]

DS_ACTION_ALT = [
    "action/joint_velocity",
    "action/cartesian_position",
    "action/joint_position",
    "action/gripper_velocity",
]

# Timestamp datasets (optional, will generate if missing)
DS_TIMESTAMP = "observation/timestamp/control/step_start"
DS_TIMESTAMP_ALT = [
    "observation/timestamp/control/step_end",
    "observation/timestamp/robot_state/robot_pub_t",
]

# Default control frequency (Hz) - used when timestamp data is missing
DEFAULT_CONTROL_HZ = 15.0

# =============================================================================
# DATA CLASSES
# =============================================================================


@dataclass
class ControlMetrics:
    """Container for control system performance metrics."""
    
    # Rise Time
    rise_time: Optional[float] = None  # seconds
    rise_time_start_idx: Optional[int] = None
    rise_time_end_idx: Optional[int] = None
    
    # Overshoot
    overshoot_percent: Optional[float] = None
    overshoot_value: Optional[float] = None
    overshoot_idx: Optional[int] = None
    
    # Steady-State Error
    steady_state_error: Optional[float] = None  # MAE in stable region
    steady_state_std: Optional[float] = None
    
    # Control Effort
    control_energy: Optional[float] = None  # sum of squared actions
    action_smoothness: Optional[float] = None  # variance of action derivative
    action_jerk_rms: Optional[float] = None  # RMS of second derivative
    
    # Additional info
    step_detected: bool = False
    step_start_idx: Optional[int] = None
    step_magnitude: Optional[float] = None
    final_value: Optional[float] = None
    target_value: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert metrics to dictionary."""
        return {
            "rise_time_s": self.rise_time,
            "overshoot_percent": self.overshoot_percent,
            "steady_state_error": self.steady_state_error,
            "steady_state_std": self.steady_state_std,
            "control_energy": self.control_energy,
            "action_smoothness": self.action_smoothness,
            "action_jerk_rms": self.action_jerk_rms,
            "step_detected": self.step_detected,
            "step_magnitude": self.step_magnitude,
        }


# =============================================================================
# DATA LOADING FUNCTIONS
# =============================================================================


def find_dataset(hdf5_file: h5py.File, primary_key: str, alt_keys: List[str]) -> Optional[str]:
    """
    Find an existing dataset key from primary and alternative options.
    
    Args:
        hdf5_file: HDF5 file object
        primary_key: Primary dataset key to try first
        alt_keys: List of alternative keys to try if primary doesn't exist
    
    Returns:
        The key that exists, or None if none found
    """
    if primary_key in hdf5_file:
        return primary_key
    
    for key in alt_keys:
        if key in hdf5_file:
            return key
    
    return None


def load_dataset(hdf5_file: h5py.File, key: str) -> Optional[np.ndarray]:
    """
    Load a dataset from HDF5 file.
    
    Args:
        hdf5_file: HDF5 file object
        key: Dataset key
    
    Returns:
        Numpy array or None if not found
    """
    if key not in hdf5_file:
        return None
    
    try:
        data = hdf5_file[key][:]
        return np.array(data, dtype=np.float64)
    except Exception as e:
        print(f"  ⚠️ Warning: Could not load dataset '{key}': {e}")
        return None


def generate_time_vector(n_points: int, control_hz: float) -> np.ndarray:
    """
    Generate time vector based on control frequency.
    
    Args:
        n_points: Number of data points
        control_hz: Control frequency in Hz
    
    Returns:
        Time vector in seconds
    """
    dt = 1.0 / control_hz
    return np.arange(n_points) * dt


def load_both_spaces_data(
    filepath: str,
    control_hz: float = DEFAULT_CONTROL_HZ
) -> Tuple[
    Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, str]],  # Cartesian
    Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, str]]   # Joint
]:
    """
    Load both Cartesian and Joint space data from HDF5 file.
    
    Args:
        filepath: Path to HDF5 file
        control_hz: Control frequency for time generation
    
    Returns:
        Tuple of (cartesian_data, joint_data) where each is (time, target, state, action, keys_used)
    """
    with h5py.File(filepath, 'r') as f:
        # Find Cartesian space keys
        cart_target_k = find_dataset(f, "action/target_cartesian_position", 
                                     ["action/cartesian_position"])
        cart_state_k = find_dataset(f, "observation/robot_state/cartesian_position",
                                    ["action/robot_state/cartesian_position"])
        cart_action_k = find_dataset(f, "action/cartesian_velocity", [])
        
        # Find Joint space keys
        joint_target_k = find_dataset(f, "action/target_joint_position",
                                      ["action/joint_position"])
        joint_state_k = find_dataset(f, "observation/robot_state/joint_positions",
                                     ["action/robot_state/joint_positions"])
        joint_action_k = find_dataset(f, "action/joint_velocity", [])
        
        # Find timestamp
        time_k = find_dataset(f, DS_TIMESTAMP, DS_TIMESTAMP_ALT)
        
        # Load time vector
        if time_k:
            time_data = load_dataset(f, time_k)
            if time_data is not None:
                if np.mean(time_data) > 1e9:  # Likely nanoseconds
                    time_data = (time_data - time_data[0]) / 1e9
                time = time_data
            else:
                # Need to determine length from available data
                if cart_state_k:
                    n_points = len(f[cart_state_k])
                elif joint_state_k:
                    n_points = len(f[joint_state_k])
                else:
                    n_points = 100  # Default
                time = generate_time_vector(n_points, control_hz)
        else:
            # Determine length from available data
            if cart_state_k:
                n_points = len(f[cart_state_k])
            elif joint_state_k:
                n_points = len(f[joint_state_k])
            else:
                n_points = 100  # Default
            time = generate_time_vector(n_points, control_hz)
        
        # Load Cartesian data
        cart_target = load_dataset(f, cart_target_k) if cart_target_k else None
        cart_state = load_dataset(f, cart_state_k) if cart_state_k else None
        cart_action = load_dataset(f, cart_action_k) if cart_action_k else None
        
        # Load Joint data
        joint_target = load_dataset(f, joint_target_k) if joint_target_k else None
        joint_state = load_dataset(f, joint_state_k) if joint_state_k else None
        joint_action = load_dataset(f, joint_action_k) if joint_action_k else None
        
        # Ensure consistent lengths for Cartesian
        if cart_state is not None:
            min_len = len(time)
            if cart_target is not None:
                min_len = min(min_len, len(cart_target))
            if cart_state is not None:
                min_len = min(min_len, len(cart_state))
            if cart_action is not None:
                min_len = min(min_len, len(cart_action))
            
            time_cart = time[:min_len]
            cart_target = cart_target[:min_len] if cart_target is not None else None
            cart_state = cart_state[:min_len]
            if cart_action is not None:
                cart_action = cart_action[:min_len]
            else:
                # Create zero action with same dimensions as state
                n_dims = cart_state.shape[1] if cart_state.ndim > 1 else 6
                cart_action = np.zeros((min_len, n_dims))
            
            # Handle missing target
            if cart_target is None:
                cart_target = np.full_like(cart_state, np.mean(cart_state, axis=0))
        else:
            time_cart = None
            cart_target = None
            cart_state = None
            cart_action = None
        
        # Ensure consistent lengths for Joint
        if joint_state is not None:
            min_len = len(time)
            if joint_target is not None:
                min_len = min(min_len, len(joint_target))
            if joint_state is not None:
                min_len = min(min_len, len(joint_state))
            if joint_action is not None:
                min_len = min(min_len, len(joint_action))
            
            time_joint = time[:min_len]
            joint_target = joint_target[:min_len] if joint_target is not None else None
            joint_state = joint_state[:min_len]
            if joint_action is not None:
                joint_action = joint_action[:min_len]
            else:
                # Create zero action with same dimensions as state
                n_dims = joint_state.shape[1] if joint_state.ndim > 1 else 7
                joint_action = np.zeros((min_len, n_dims))
            
            # Handle missing target
            if joint_target is None:
                joint_target = np.full_like(joint_state, np.mean(joint_state, axis=0))
        else:
            time_joint = None
            joint_target = None
            joint_state = None
            joint_action = None
        
        cart_keys = {
            "target": cart_target_k,
            "state": cart_state_k,
            "action": cart_action_k,
            "time": time_k,
        }
        
        joint_keys = {
            "target": joint_target_k,
            "state": joint_state_k,
            "action": joint_action_k,
            "time": time_k,
        }
        
        cart_data = (time_cart, cart_target, cart_state, cart_action, cart_keys)
        joint_data = (time_joint, joint_target, joint_state, joint_action, joint_keys)
        
        return cart_data, joint_data


def load_trajectory_data(
    filepath: str,
    target_key: Optional[str] = None,
    state_key: Optional[str] = None,
    action_key: Optional[str] = None,
    control_hz: float = DEFAULT_CONTROL_HZ
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, str]]:
    """
    Load trajectory data from HDF5 file.
    
    Args:
        filepath: Path to HDF5 file
        target_key: Custom target dataset key (optional)
        state_key: Custom state dataset key (optional)
        action_key: Custom action dataset key (optional)
        control_hz: Control frequency for time generation
    
    Returns:
        Tuple of (time, target, state, action, keys_used)
    
    Raises:
        ValueError: If required datasets cannot be found
    """
    with h5py.File(filepath, 'r') as f:
        # Find dataset keys
        target_k = target_key or find_dataset(f, DS_TARGET, DS_TARGET_ALT)
        state_k = state_key or find_dataset(f, DS_STATE, DS_STATE_ALT)
        action_k = action_key or find_dataset(f, DS_ACTION, DS_ACTION_ALT)
        time_k = find_dataset(f, DS_TIMESTAMP, DS_TIMESTAMP_ALT)
        
        keys_used = {
            "target": target_k,
            "state": state_k,
            "action": action_k,
            "time": time_k,
        }
        
        # Load datasets
        target = load_dataset(f, target_k) if target_k else None
        state = load_dataset(f, state_k) if state_k else None
        action = load_dataset(f, action_k) if action_k else None
        
        # Check required data
        if state is None:
            raise ValueError(
                f"Could not find state dataset. Tried: {DS_STATE}, {DS_STATE_ALT}"
            )
        
        n_points = len(state)
        
        # Load or generate time vector
        if time_k:
            time_data = load_dataset(f, time_k)
            if time_data is not None:
                # Convert from nanoseconds to seconds if needed
                if np.mean(time_data) > 1e9:  # Likely nanoseconds
                    time_data = (time_data - time_data[0]) / 1e9
                time = time_data[:n_points]
            else:
                time = generate_time_vector(n_points, control_hz)
        else:
            time = generate_time_vector(n_points, control_hz)
        
        # Handle missing target (use state as reference)
        if target is None:
            print("  ⚠️ Warning: Target dataset not found. Using state mean as reference.")
            target = np.full_like(state, np.mean(state, axis=0))
        
        # Handle missing action
        if action is None:
            print("  ⚠️ Warning: Action dataset not found. Control effort analysis will be limited.")
            action = np.zeros((n_points, 1))
        
        # Ensure consistent lengths
        min_len = min(len(time), len(target), len(state), len(action))
        time = time[:min_len]
        target = target[:min_len]
        state = state[:min_len]
        action = action[:min_len]
        
        return time, target, state, action, keys_used


# =============================================================================
# METRIC CALCULATION FUNCTIONS
# =============================================================================


def detect_step_change(
    target: np.ndarray,
    threshold_ratio: float = 0.1
) -> Tuple[bool, int, float, float]:
    """
    Detect if there's a step change in the target signal.
    
    Args:
        target: Target signal array (1D)
        threshold_ratio: Minimum change ratio to consider as step
    
    Returns:
        Tuple of (step_detected, step_start_idx, initial_value, final_value)
    """
    if len(target) < 10:
        return False, 0, target[0], target[-1]
    
    # Calculate derivative
    diff = np.diff(target)
    abs_diff = np.abs(diff)
    
    # Find significant changes
    range_val = np.max(target) - np.min(target)
    if range_val < 1e-10:
        return False, 0, target[0], target[-1]
    
    threshold = range_val * threshold_ratio
    step_indices = np.where(abs_diff > threshold)[0]
    
    if len(step_indices) == 0:
        return False, 0, target[0], target[-1]
    
    step_start = step_indices[0]
    initial_value = np.mean(target[max(0, step_start - 5):step_start + 1])
    final_value = np.mean(target[-10:])
    
    return True, step_start, initial_value, final_value


def calculate_rise_time(
    time: np.ndarray,
    state: np.ndarray,
    step_start_idx: int,
    initial_value: float,
    final_value: float,
    low_percent: float = 0.1,
    high_percent: float = 0.9
) -> Tuple[Optional[float], Optional[int], Optional[int]]:
    """
    Calculate rise time from low_percent to high_percent of step magnitude.
    
    Args:
        time: Time array
        state: State array (1D)
        step_start_idx: Index where step begins
        initial_value: Initial value before step
        final_value: Final target value
        low_percent: Lower threshold (default 10%)
        high_percent: Upper threshold (default 90%)
    
    Returns:
        Tuple of (rise_time, start_idx, end_idx)
    """
    step_magnitude = final_value - initial_value
    
    if abs(step_magnitude) < 1e-10:
        return None, None, None
    
    # Calculate threshold values
    if step_magnitude > 0:
        low_threshold = initial_value + low_percent * step_magnitude
        high_threshold = initial_value + high_percent * step_magnitude
    else:
        low_threshold = initial_value + high_percent * step_magnitude
        high_threshold = initial_value + low_percent * step_magnitude
    
    # Find crossing points after step starts
    state_after_step = state[step_start_idx:]
    
    # Find low threshold crossing
    if step_magnitude > 0:
        low_crossings = np.where(state_after_step >= low_threshold)[0]
        high_crossings = np.where(state_after_step >= high_threshold)[0]
    else:
        low_crossings = np.where(state_after_step <= low_threshold)[0]
        high_crossings = np.where(state_after_step <= high_threshold)[0]
    
    if len(low_crossings) == 0 or len(high_crossings) == 0:
        return None, None, None
    
    low_idx = step_start_idx + low_crossings[0]
    high_idx = step_start_idx + high_crossings[0]
    
    if high_idx <= low_idx:
        return None, None, None
    
    rise_time = time[high_idx] - time[low_idx]
    
    return rise_time, low_idx, high_idx


def calculate_overshoot(
    state: np.ndarray,
    step_start_idx: int,
    initial_value: float,
    final_value: float
) -> Tuple[Optional[float], Optional[float], Optional[int]]:
    """
    Calculate percentage overshoot.
    
    Args:
        state: State array (1D)
        step_start_idx: Index where step begins
        initial_value: Initial value before step
        final_value: Final target value
    
    Returns:
        Tuple of (overshoot_percent, overshoot_value, overshoot_idx)
    """
    step_magnitude = final_value - initial_value
    
    if abs(step_magnitude) < 1e-10:
        return None, None, None
    
    state_after_step = state[step_start_idx:]
    
    if step_magnitude > 0:
        max_value = np.max(state_after_step)
        max_idx = step_start_idx + np.argmax(state_after_step)
        overshoot = max_value - final_value
    else:
        min_value = np.min(state_after_step)
        max_idx = step_start_idx + np.argmin(state_after_step)
        overshoot = final_value - min_value
    
    # Only report overshoot if state exceeds target
    if overshoot <= 0:
        return 0.0, 0.0, None
    
    overshoot_percent = (overshoot / abs(step_magnitude)) * 100
    
    return overshoot_percent, overshoot, max_idx


def calculate_steady_state_error(
    target: np.ndarray,
    state: np.ndarray,
    stable_region_ratio: float = 0.1
) -> Tuple[Optional[float], Optional[float]]:
    """
    Calculate steady-state error (MAE) in the last portion of data.
    
    Args:
        target: Target array (1D)
        state: State array (1D)
        stable_region_ratio: Portion of data to consider as stable (default 10%)
    
    Returns:
        Tuple of (mae, std)
    """
    n = len(target)
    start_idx = int(n * (1 - stable_region_ratio))
    
    if start_idx >= n - 1:
        start_idx = max(0, n - 10)
    
    error = np.abs(target[start_idx:] - state[start_idx:])
    
    mae = np.nanmean(error)
    std = np.nanstd(error)
    
    return mae, std


def calculate_control_effort(
    action: np.ndarray,
    time: np.ndarray
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """
    Calculate control effort metrics.
    
    Args:
        action: Action array (1D or 2D)
        time: Time array
    
    Returns:
        Tuple of (total_energy, smoothness, jerk_rms)
    """
    if action is None or len(action) == 0:
        return None, None, None
    
    # Handle multi-dimensional action
    if action.ndim > 1:
        # Sum energy across all dimensions
        action_flat = action.reshape(len(action), -1)
        energy = np.sum(action_flat ** 2)
    else:
        energy = np.sum(action ** 2)
    
    # Calculate dt
    if len(time) > 1:
        dt = np.mean(np.diff(time))
    else:
        dt = 1.0 / DEFAULT_CONTROL_HZ
    
    # Action derivative (for smoothness)
    if action.ndim > 1:
        action_1d = np.linalg.norm(action, axis=1) if action.shape[1] > 1 else action[:, 0]
    else:
        action_1d = action
    
    if len(action_1d) > 1:
        action_diff = np.diff(action_1d) / dt
        smoothness = np.var(action_diff)
        
        # Jerk (second derivative)
        if len(action_diff) > 1:
            action_jerk = np.diff(action_diff) / dt
            jerk_rms = np.sqrt(np.mean(action_jerk ** 2))
        else:
            jerk_rms = None
    else:
        smoothness = None
        jerk_rms = None
    
    return energy, smoothness, jerk_rms


def analyze_dimension(
    time: np.ndarray,
    target: np.ndarray,
    state: np.ndarray,
    action: Optional[np.ndarray] = None
) -> ControlMetrics:
    """
    Analyze a single dimension of control performance.
    
    Args:
        time: Time array
        target: Target array (1D)
        state: State array (1D)
        action: Action array (1D, optional)
    
    Returns:
        ControlMetrics object with calculated metrics
    """
    metrics = ControlMetrics()
    
    # Detect step change
    step_detected, step_idx, init_val, final_val = detect_step_change(target)
    metrics.step_detected = step_detected
    metrics.step_start_idx = step_idx
    metrics.step_magnitude = final_val - init_val
    metrics.target_value = final_val
    metrics.final_value = np.mean(state[-10:]) if len(state) >= 10 else state[-1]
    
    # Calculate metrics
    if step_detected:
        # Rise time
        rt, rt_start, rt_end = calculate_rise_time(
            time, state, step_idx, init_val, final_val
        )
        metrics.rise_time = rt
        metrics.rise_time_start_idx = rt_start
        metrics.rise_time_end_idx = rt_end
        
        # Overshoot
        os_pct, os_val, os_idx = calculate_overshoot(
            state, step_idx, init_val, final_val
        )
        metrics.overshoot_percent = os_pct
        metrics.overshoot_value = os_val
        metrics.overshoot_idx = os_idx
    
    # Steady-state error (always calculated)
    sse, sse_std = calculate_steady_state_error(target, state)
    metrics.steady_state_error = sse
    metrics.steady_state_std = sse_std
    
    # Control effort
    if action is not None:
        energy, smooth, jerk = calculate_control_effort(action, time)
        metrics.control_energy = energy
        metrics.action_smoothness = smooth
        metrics.action_jerk_rms = jerk
    
    return metrics


# =============================================================================
# VISUALIZATION FUNCTIONS
# =============================================================================


def create_multi_dim_analysis_figure(
    time: np.ndarray,
    target: np.ndarray,
    state: np.ndarray,
    action: np.ndarray,
    all_metrics: Dict[int, ControlMetrics],
    dim_labels: List[str],
    title: Optional[str] = None
) -> 'matplotlib.figure.Figure':
    """
    Create a comprehensive figure with all dimensions in one plot.
    
    Args:
        time: Time array
        target: Target array (2D: [n_timesteps, n_dims])
        state: State array (2D: [n_timesteps, n_dims])
        action: Action array (2D: [n_timesteps, n_dims])
        all_metrics: Dictionary of metrics by dimension index
        dim_labels: Labels for each dimension
        title: Custom figure title
    
    Returns:
        Matplotlib figure object
    """
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
    
    # Set style
    plt.style.use('seaborn-v0_8-whitegrid')
    
    n_dims = state.shape[1] if state.ndim > 1 else 1
    
    # Create figure with subplots: 3 rows, n_dims columns
    fig, axes = plt.subplots(3, n_dims, figsize=(6 * n_dims, 10), sharex='col', sharey='row')
    if n_dims == 1:
        axes = axes.reshape(-1, 1)
    fig.patch.set_facecolor('#f8f9fa')
    
    # Color palette - use colormap for different dimensions
    cmap = cm.get_cmap('tab10')
    base_colors = {
        'target': '#2c3e50',
        'error': '#e74c3c',
        'tolerance': '#f39c12',
    }
    
    for dim_idx in range(n_dims):
        dim_label = dim_labels[dim_idx] if dim_idx < len(dim_labels) else f"Dim {dim_idx}"
        metrics = all_metrics.get(dim_idx, ControlMetrics())
        
        # Get data for this dimension
        target_1d = target[:, dim_idx] if target.ndim > 1 else target
        state_1d = state[:, dim_idx]
        if action.ndim > 1:
            action_1d = action[:, min(dim_idx, action.shape[1] - 1)]
        else:
            action_1d = action.flatten() if action.ndim > 1 else action
        
        dim_color = cmap(dim_idx / max(n_dims, 1))
        
        # ===================
        # Subplot 1: Tracking Performance
        # ===================
        ax1 = axes[0, dim_idx]
        ax1.set_facecolor('#ffffff')
        
        # Plot target (dashed line)
        ax1.plot(time, target_1d, '--', color=base_colors['target'], linewidth=2, 
                 label='Set Point', alpha=0.9)
        
        # Plot state (solid line)
        ax1.plot(time, state_1d, '-', color=dim_color, linewidth=1.5, 
                 label='Actual', alpha=0.9)
        
        # Mark overshoot
        if metrics.overshoot_idx is not None and metrics.overshoot_percent and metrics.overshoot_percent > 0.1:
            ax1.plot(time[metrics.overshoot_idx], state_1d[metrics.overshoot_idx], 
                     'o', color='red', markersize=8, zorder=5)
            ax1.annotate(f'{metrics.overshoot_percent:.1f}%',
                         xy=(time[metrics.overshoot_idx], state_1d[metrics.overshoot_idx]),
                         xytext=(5, 5), textcoords='offset points',
                         fontsize=8, color='red',
                         arrowprops=dict(arrowstyle='->', color='red', alpha=0.7, lw=0.5))
        
        # Mark rise time region
        if metrics.rise_time_start_idx is not None and metrics.rise_time_end_idx is not None:
            ax1.axvspan(time[metrics.rise_time_start_idx], time[metrics.rise_time_end_idx],
                        alpha=0.2, color='purple')
        
        ax1.set_ylabel(f'{dim_label}', fontsize=10, fontweight='bold')
        if dim_idx == 0:
            ax1.set_title('Tracking Performance', fontsize=11, fontweight='bold', pad=10)
        else:
            ax1.set_title(f'{dim_label}', fontsize=11, fontweight='bold', pad=10)
        ax1.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax1.grid(True, alpha=0.3)
        
        # ===================
        # Subplot 2: Tracking Error
        # ===================
        ax2 = axes[1, dim_idx]
        ax2.set_facecolor('#ffffff')
        
        error = state_1d - target_1d
        
        # Plot error
        ax2.plot(time, error, '-', color=base_colors['error'], linewidth=1.5, 
                 label='Error', alpha=0.9)
        
        # Tolerance band
        if metrics.steady_state_error is not None:
            tolerance = 3 * metrics.steady_state_error
            ax2.fill_between(time, -tolerance, tolerance, 
                             color=base_colors['tolerance'], alpha=0.2)
        
        # Zero line
        ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
        
        # Mark steady-state region
        stable_start = int(len(time) * 0.9)
        ax2.axvspan(time[stable_start], time[-1], alpha=0.1, color='gray')
        
        if dim_idx == 0:
            ax2.set_ylabel('Error', fontsize=10, fontweight='bold')
            ax2.set_title('Tracking Error', fontsize=11, fontweight='bold', pad=10)
        ax2.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax2.grid(True, alpha=0.3)
        
        # ===================
        # Subplot 3: Control Action
        # ===================
        ax3 = axes[2, dim_idx]
        ax3.set_facecolor('#ffffff')
        
        ax3.plot(time, action_1d, '-', color=dim_color, linewidth=1.5, 
                 label='Action', alpha=0.9)
        
        # Action mean line
        action_mean = np.mean(action_1d)
        ax3.axhline(y=action_mean, color=dim_color, linestyle='--', 
                    linewidth=1, alpha=0.5)
        
        if dim_idx == 0:
            ax3.set_ylabel('Control Signal', fontsize=10, fontweight='bold')
            ax3.set_title('Control Action', fontsize=11, fontweight='bold', pad=10)
        ax3.set_xlabel('Time [s]', fontsize=10, fontweight='bold')
        ax3.legend(loc='upper right', fontsize=8, framealpha=0.9)
        ax3.grid(True, alpha=0.3)
    
    # ===================
    # Figure title and layout
    # ===================
    if title:
        fig.suptitle(title, fontsize=14, fontweight='bold', y=0.98)
    else:
        fig.suptitle('Control System Performance Analysis - All Dimensions', 
                     fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.93, hspace=0.2, wspace=0.3)
    
    return fig


def create_analysis_figure(
    time: np.ndarray,
    target: np.ndarray,
    state: np.ndarray,
    action: np.ndarray,
    metrics: ControlMetrics,
    dim_idx: int = 0,
    dim_labels: Optional[List[str]] = None,
    title: Optional[str] = None
) -> 'matplotlib.figure.Figure':
    """
    Create professional 3-subplot analysis figure.
    
    Args:
        time: Time array
        target: Target array (1D)
        state: State array (1D)
        action: Action array (1D)
        metrics: Calculated metrics
        dim_idx: Dimension index being analyzed
        dim_labels: Labels for each dimension
        title: Custom figure title
    
    Returns:
        Matplotlib figure object
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch
    import matplotlib.patches as mpatches
    
    # Set style
    plt.style.use('seaborn-v0_8-whitegrid')
    
    # Create figure
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.patch.set_facecolor('#f8f9fa')
    
    # Color palette
    colors = {
        'target': '#2c3e50',
        'state': '#3498db',
        'error': '#e74c3c',
        'action': '#27ae60',
        'overshoot': '#e74c3c',
        'tolerance': '#f39c12',
        'rise': '#9b59b6',
    }
    
    # Dimension label
    if dim_labels and dim_idx < len(dim_labels):
        dim_label = dim_labels[dim_idx]
    else:
        dim_label = f"Dimension {dim_idx}"
    
    # ===================
    # Subplot 1: Tracking Performance
    # ===================
    ax1 = axes[0]
    ax1.set_facecolor('#ffffff')
    
    # Plot target (dashed line)
    ax1.plot(time, target, '--', color=colors['target'], linewidth=2, 
             label='Set Point', alpha=0.9)
    
    # Plot state (solid line)
    ax1.plot(time, state, '-', color=colors['state'], linewidth=1.5, 
             label='Actual', alpha=0.9)
    
    # Mark overshoot
    if metrics.overshoot_idx is not None and metrics.overshoot_percent > 0.1:
        ax1.plot(time[metrics.overshoot_idx], state[metrics.overshoot_idx], 
                 'o', color=colors['overshoot'], markersize=10, 
                 label=f'Max Overshoot ({metrics.overshoot_percent:.1f}%)', zorder=5)
        ax1.annotate(f'{metrics.overshoot_percent:.1f}%',
                     xy=(time[metrics.overshoot_idx], state[metrics.overshoot_idx]),
                     xytext=(10, 10), textcoords='offset points',
                     fontsize=9, color=colors['overshoot'],
                     arrowprops=dict(arrowstyle='->', color=colors['overshoot'], alpha=0.7))
    
    # Mark rise time region
    if metrics.rise_time_start_idx is not None and metrics.rise_time_end_idx is not None:
        ax1.axvspan(time[metrics.rise_time_start_idx], time[metrics.rise_time_end_idx],
                    alpha=0.2, color=colors['rise'], label=f'Rise Time ({metrics.rise_time:.3f}s)')
    
    ax1.set_ylabel(f'{dim_label}', fontsize=11, fontweight='bold')
    ax1.set_title('Tracking Performance', fontsize=12, fontweight='bold', pad=10)
    ax1.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    
    # ===================
    # Subplot 2: Tracking Error
    # ===================
    ax2 = axes[1]
    ax2.set_facecolor('#ffffff')
    
    error = state - target
    
    # Plot error
    ax2.plot(time, error, '-', color=colors['error'], linewidth=1.5, 
             label='Error (State - Target)', alpha=0.9)
    
    # Tolerance band
    if metrics.steady_state_error is not None:
        tolerance = 3 * metrics.steady_state_error  # 3-sigma band
        ax2.fill_between(time, -tolerance, tolerance, 
                         color=colors['tolerance'], alpha=0.2,
                         label=f'±3σ Tolerance ({tolerance:.4f})')
    
    # Zero line
    ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.5, alpha=0.5)
    
    # Mark steady-state region
    stable_start = int(len(time) * 0.9)
    ax2.axvspan(time[stable_start], time[-1], alpha=0.1, color='gray',
                label='Stable Region (SSE analysis)')
    
    ax2.set_ylabel('Error', fontsize=11, fontweight='bold')
    ax2.set_title('Tracking Error', fontsize=12, fontweight='bold', pad=10)
    ax2.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax2.grid(True, alpha=0.3)
    
    # ===================
    # Subplot 3: Control Action
    # ===================
    ax3 = axes[2]
    ax3.set_facecolor('#ffffff')
    
    # Handle multi-dimensional action for display
    if action.ndim > 1 and action.shape[1] > 1:
        action_display = action[:, min(dim_idx, action.shape[1] - 1)]
        action_label = f'Control Action (dim {dim_idx})'
    else:
        action_display = action.flatten() if action.ndim > 1 else action
        action_label = 'Control Action'
    
    ax3.plot(time, action_display, '-', color=colors['action'], linewidth=1.5, 
             label=action_label, alpha=0.9)
    
    # Action bounds visualization
    action_mean = np.mean(action_display)
    action_std = np.std(action_display)
    ax3.axhline(y=action_mean, color=colors['action'], linestyle='--', 
                linewidth=1, alpha=0.5, label=f'Mean ({action_mean:.4f})')
    
    ax3.set_xlabel('Time [s]', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Control Signal\n(Velocity/Torque)', fontsize=11, fontweight='bold')
    ax3.set_title('Control Action', fontsize=12, fontweight='bold', pad=10)
    ax3.legend(loc='upper right', fontsize=9, framealpha=0.9)
    ax3.grid(True, alpha=0.3)
    
    # ===================
    # Figure title and layout
    # ===================
    if title:
        fig.suptitle(title, fontsize=14, fontweight='bold', y=0.98)
    else:
        fig.suptitle(f'Control System Performance Analysis - {dim_label}', 
                     fontsize=14, fontweight='bold', y=0.98)
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.93, hspace=0.15)
    
    return fig


# =============================================================================
# OUTPUT FUNCTIONS
# =============================================================================


def print_metrics_summary(
    metrics: ControlMetrics,
    dim_idx: int = 0,
    dim_label: Optional[str] = None
):
    """
    Print formatted metrics summary to console.
    
    Args:
        metrics: Calculated metrics
        dim_idx: Dimension index
        dim_label: Optional dimension label
    """
    label = dim_label or f"Dimension {dim_idx}"
    
    print()
    print("=" * 70)
    print(f"  CONTROL PERFORMANCE METRICS - {label}")
    print("=" * 70)
    print()
    
    # Step Detection
    if metrics.step_detected:
        print(f"  📊 Step Change Detected:")
        print(f"     └─ Magnitude: {metrics.step_magnitude:.6f}")
        print(f"     └─ Target Value: {metrics.target_value:.6f}")
        print(f"     └─ Final State: {metrics.final_value:.6f}")
        print()
    
    # Rise Time
    print(f"  ⏱️  Rise Time (10%-90%):")
    if metrics.rise_time is not None:
        print(f"     └─ {metrics.rise_time:.4f} s")
    else:
        print(f"     └─ N/A (no step detected or threshold not reached)")
    print()
    
    # Overshoot
    print(f"  📈 Percentage Overshoot:")
    if metrics.overshoot_percent is not None:
        if metrics.overshoot_percent > 0.1:
            print(f"     └─ {metrics.overshoot_percent:.2f} %")
        else:
            print(f"     └─ None (no overshoot)")
    else:
        print(f"     └─ N/A")
    print()
    
    # Steady-State Error
    print(f"  🎯 Steady-State Error (MAE):")
    if metrics.steady_state_error is not None:
        print(f"     └─ MAE: {metrics.steady_state_error:.6f}")
        print(f"     └─ STD: {metrics.steady_state_std:.6f}")
    else:
        print(f"     └─ N/A")
    print()
    
    # Control Effort
    print(f"  ⚡ Control Effort Analysis:")
    if metrics.control_energy is not None:
        print(f"     └─ Total Energy: {metrics.control_energy:.4f}")
    else:
        print(f"     └─ Total Energy: N/A")
    
    if metrics.action_smoothness is not None:
        print(f"     └─ Action Smoothness (Var[dU/dt]): {metrics.action_smoothness:.6f}")
    else:
        print(f"     └─ Action Smoothness: N/A")
    
    if metrics.action_jerk_rms is not None:
        print(f"     └─ Action Jerk RMS: {metrics.action_jerk_rms:.6f}")
    else:
        print(f"     └─ Action Jerk RMS: N/A")
    
    print()
    print("-" * 70)


def print_multi_dim_summary(all_metrics: Dict[int, ControlMetrics], dim_labels: List[str]):
    """
    Print summary table for all dimensions.
    
    Args:
        all_metrics: Dictionary of metrics by dimension index
        dim_labels: Labels for each dimension
    """
    print()
    print("=" * 90)
    print("  MULTI-DIMENSIONAL SUMMARY")
    print("=" * 90)
    print()
    
    # Header
    print(f"  {'Dimension':<12} {'Rise Time':<12} {'Overshoot':<12} {'SSE (MAE)':<14} {'Energy':<14} {'Smoothness':<14}")
    print("  " + "-" * 86)
    
    for idx, metrics in sorted(all_metrics.items()):
        label = dim_labels[idx] if idx < len(dim_labels) else f"Dim {idx}"
        rt = f"{metrics.rise_time:.4f}s" if metrics.rise_time else "N/A"
        os_pct = f"{metrics.overshoot_percent:.2f}%" if metrics.overshoot_percent else "0.00%"
        sse = f"{metrics.steady_state_error:.6f}" if metrics.steady_state_error else "N/A"
        energy = f"{metrics.control_energy:.4f}" if metrics.control_energy else "N/A"
        smooth = f"{metrics.action_smoothness:.6f}" if metrics.action_smoothness else "N/A"
        
        print(f"  {label:<12} {rt:<12} {os_pct:<12} {sse:<14} {energy:<14} {smooth:<14}")
    
    print()


# =============================================================================
# MAIN FUNCTION
# =============================================================================


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Control System Performance Analyzer for Robot Trajectory HDF5 Files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic analysis (analyzes all dimensions)
  python3 control_analysis.py trajectory.h5
  
  # Analyze specific dimension
  python3 control_analysis.py trajectory.h5 --dim 0
  
  # Custom control frequency
  python3 control_analysis.py trajectory.h5 --control-hz 20
  
  # Custom dataset keys
  python3 control_analysis.py trajectory.h5 \\
      --target-key "action/target_cartesian_position" \\
      --state-key "observation/robot_state/cartesian_position" \\
      --action-key "action/cartesian_velocity"
  
  # Custom output path
  python3 control_analysis.py trajectory.h5 --output analysis.png
  
  # Show only metrics (no plot)
  python3 control_analysis.py trajectory.h5 --no-plot
        """
    )
    
    parser.add_argument(
        'filepath',
        type=str,
        help='Path to HDF5 trajectory file'
    )
    parser.add_argument(
        '--dim', '-d',
        type=int,
        default=None,
        metavar='INDEX',
        help='Analyze specific dimension only (0-indexed). Default: analyze all'
    )
    parser.add_argument(
        '--control-hz',
        type=float,
        default=DEFAULT_CONTROL_HZ,
        metavar='HZ',
        help=f'Control frequency in Hz (default: {DEFAULT_CONTROL_HZ})'
    )
    parser.add_argument(
        '--target-key',
        type=str,
        default=None,
        metavar='KEY',
        help=f'Custom target dataset key (default: {DS_TARGET})'
    )
    parser.add_argument(
        '--state-key',
        type=str,
        default=None,
        metavar='KEY',
        help=f'Custom state dataset key (default: {DS_STATE})'
    )
    parser.add_argument(
        '--action-key',
        type=str,
        default=None,
        metavar='KEY',
        help=f'Custom action dataset key (default: {DS_ACTION})'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default='control_analysis_result.png',
        metavar='PATH',
        help='Output figure path (default: control_analysis_result.png)'
    )
    parser.add_argument(
        '--no-plot',
        action='store_true',
        help='Skip visualization, only print metrics'
    )
    parser.add_argument(
        '--dpi',
        type=int,
        default=300,
        help='Figure DPI (default: 300)'
    )
    
    args = parser.parse_args()
    
    # Validate file
    if not os.path.exists(args.filepath):
        print(f"❌ Error: File not found: {args.filepath}")
        sys.exit(1)
    
    print()
    print("=" * 70)
    print("  🤖 CONTROL SYSTEM PERFORMANCE ANALYZER")
    print("=" * 70)
    print(f"  File: {args.filepath}")
    print(f"  Control Hz: {args.control_hz}")
    print(f"  Analysis Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    try:
        # Load both Cartesian and Joint space data
        print("📂 Loading trajectory data...")
        (cart_time, cart_target, cart_state, cart_action, cart_keys), \
        (joint_time, joint_target, joint_state, joint_action, joint_keys) = \
            load_both_spaces_data(args.filepath, control_hz=args.control_hz)
        
        # Generate output filename base
        hdf5_basename = os.path.splitext(os.path.basename(args.filepath))[0]
        
        # Process Cartesian space data
        if cart_state is not None:
            print()
            print("=" * 70)
            print("  📊 CARTESIAN SPACE ANALYSIS")
            print("=" * 70)
            print(f"   ✅ Cartesian data loaded")
            print(f"   └─ Timesteps: {len(cart_time)}")
            print(f"   └─ Duration: {cart_time[-1] - cart_time[0]:.2f} s")
            print(f"   └─ Target key: {cart_keys['target']}")
            print(f"   └─ State key: {cart_keys['state']}")
            print(f"   └─ Action key: {cart_keys['action']}")
            print(f"   └─ State shape: {cart_state.shape}")
            print(f"   └─ Action shape: {cart_action.shape}")
            
            n_dims = cart_state.shape[1] if cart_state.ndim > 1 else 1
            if cart_state.ndim == 1:
                cart_state = cart_state.reshape(-1, 1)
                cart_target = cart_target.reshape(-1, 1) if cart_target.ndim == 1 else cart_target
            
            cart_dim_labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'][:n_dims]
            
            # Select dimensions to analyze
            if args.dim is not None:
                if args.dim >= n_dims:
                    print(f"❌ Error: Dimension {args.dim} out of range (max: {n_dims - 1})")
                    sys.exit(1)
                dims_to_analyze = [args.dim]
            else:
                dims_to_analyze = list(range(n_dims))
            
            print(f"   └─ Dimensions to analyze: {dims_to_analyze}")
            print()
            
            # Analyze Cartesian dimensions
            cart_all_metrics = {}
            for dim_idx in dims_to_analyze:
                print(f"📊 Analyzing Cartesian dimension {dim_idx} ({cart_dim_labels[dim_idx]})...")
                
                target_1d = cart_target[:, dim_idx] if cart_target.ndim > 1 else cart_target
                state_1d = cart_state[:, dim_idx]
                action_1d = cart_action[:, min(dim_idx, cart_action.shape[1] - 1)] if cart_action.ndim > 1 else cart_action
                
                metrics = analyze_dimension(cart_time, target_1d, state_1d, action_1d)
                cart_all_metrics[dim_idx] = metrics
                
                print_metrics_summary(metrics, dim_idx, cart_dim_labels[dim_idx])
            
            if len(dims_to_analyze) > 1:
                print_multi_dim_summary(cart_all_metrics, cart_dim_labels)
            
            # Create Cartesian visualization
            if not args.no_plot:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                
                print("📈 Generating Cartesian space visualization...")
                
                cart_output = f"control_analysis_result_{hdf5_basename}_cartesian.png"
                
                fig = create_multi_dim_analysis_figure(
                    cart_time, cart_target, cart_state, cart_action,
                    cart_all_metrics,
                    cart_dim_labels,
                    title=f'Cartesian Space Analysis: {os.path.basename(args.filepath)}'
                )
                
                fig.savefig(cart_output, dpi=args.dpi, bbox_inches='tight',
                            facecolor=fig.get_facecolor(), edgecolor='none')
                plt.close(fig)
                
                print(f"   ✅ Cartesian figure saved: {cart_output}")
        
        # Process Joint space data
        if joint_state is not None:
            print()
            print("=" * 70)
            print("  📊 JOINT SPACE ANALYSIS")
            print("=" * 70)
            print(f"   ✅ Joint data loaded")
            print(f"   └─ Timesteps: {len(joint_time)}")
            print(f"   └─ Duration: {joint_time[-1] - joint_time[0]:.2f} s")
            print(f"   └─ Target key: {joint_keys['target']}")
            print(f"   └─ State key: {joint_keys['state']}")
            print(f"   └─ Action key: {joint_keys['action']}")
            print(f"   └─ State shape: {joint_state.shape}")
            print(f"   └─ Action shape: {joint_action.shape}")
            
            n_dims = joint_state.shape[1] if joint_state.ndim > 1 else 1
            if joint_state.ndim == 1:
                joint_state = joint_state.reshape(-1, 1)
                joint_target = joint_target.reshape(-1, 1) if joint_target.ndim == 1 else joint_target
            
            joint_dim_labels = [f'Joint {i+1}' for i in range(n_dims)]
            
            # Select dimensions to analyze
            if args.dim is not None:
                if args.dim >= n_dims:
                    print(f"❌ Error: Dimension {args.dim} out of range (max: {n_dims - 1})")
                    sys.exit(1)
                dims_to_analyze = [args.dim]
            else:
                dims_to_analyze = list(range(n_dims))
            
            print(f"   └─ Dimensions to analyze: {dims_to_analyze}")
            print()
            
            # Analyze Joint dimensions
            joint_all_metrics = {}
            for dim_idx in dims_to_analyze:
                print(f"📊 Analyzing Joint dimension {dim_idx} ({joint_dim_labels[dim_idx]})...")
                
                target_1d = joint_target[:, dim_idx] if joint_target.ndim > 1 else joint_target
                state_1d = joint_state[:, dim_idx]
                action_1d = joint_action[:, min(dim_idx, joint_action.shape[1] - 1)] if joint_action.ndim > 1 else joint_action
                
                metrics = analyze_dimension(joint_time, target_1d, state_1d, action_1d)
                joint_all_metrics[dim_idx] = metrics
                
                print_metrics_summary(metrics, dim_idx, joint_dim_labels[dim_idx])
            
            if len(dims_to_analyze) > 1:
                print_multi_dim_summary(joint_all_metrics, joint_dim_labels)
            
            # Create Joint visualization
            if not args.no_plot:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                
                print("📈 Generating Joint space visualization...")
                
                joint_output = f"control_analysis_result_{hdf5_basename}_joint.png"
                
                fig = create_multi_dim_analysis_figure(
                    joint_time, joint_target, joint_state, joint_action,
                    joint_all_metrics,
                    joint_dim_labels,
                    title=f'Joint Space Analysis: {os.path.basename(args.filepath)}'
                )
                
                fig.savefig(joint_output, dpi=args.dpi, bbox_inches='tight',
                            facecolor=fig.get_facecolor(), edgecolor='none')
                plt.close(fig)
                
                print(f"   ✅ Joint figure saved: {joint_output}")
        
        # Check if no data was found
        if cart_state is None and joint_state is None:
            print("❌ Error: No valid Cartesian or Joint space data found in file.")
            print("   Please check that the HDF5 file contains:")
            print("   - Cartesian: observation/robot_state/cartesian_position")
            print("   - Joint: observation/robot_state/joint_positions")
            sys.exit(1)
        
        print()
        print("=" * 70)
        print("  ✅ Analysis Complete")
        print("=" * 70)
        print()
        
    except Exception as e:
        print(f"❌ Error during analysis: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

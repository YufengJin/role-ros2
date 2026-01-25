#!/usr/bin/env python3
"""
Trajectory Visualizer - Advanced GUI to visualize robot learning HDF5 datasets.

Usage:
    python3 trajectory_visualizer.py /path/to/trajectory.h5

Features:
    - Interactive slider to navigate timestamps
    - 4x4 subplot layout with:
        - Row 0-1: Camera RGB/Depth images + Timestamps panel (merged, scrollable)
        - Row 2: Robot state time series + Gantt chart (step timing)
        - Row 3: Action time series + Box plot (global latency stats)
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional, Any, Tuple
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Rectangle
from matplotlib.gridspec import GridSpec
import matplotlib.colors as mcolors

from role_ros2.trajectory_utils.trajectory_reader import TrajectoryReader


class TrajectoryVisualizer:
    """Advanced interactive trajectory visualizer using matplotlib."""
    
    # Colors for Gantt chart bars
    GANTT_COLORS = {
        'robot_read': '#2ecc71',      # Green
        'policy_active': '#3498db',   # Blue
        'policy_sleep': '#9b59b6',    # Purple
        'control_step': '#e74c3c',    # Red
        'camera_read': '#f39c12',     # Orange
    }
    
    # Colors for Gantt chart markers
    MARKER_COLORS = {
        'camera_pub': '#1abc9c',      # Teal
        'sync': '#e91e63',            # Pink
        'robot_polymetis': '#ff5722', # Deep Orange
        'robot_pub': '#795548',       # Brown
    }
    
    def __init__(self, filepath: str, debug: bool = False):
        """
        Initialize visualizer.
        
        Args:
            filepath: Path to HDF5 trajectory file
            debug: Enable debug output
        """
        self.filepath = filepath
        self.debug = debug
        self.reader = TrajectoryReader(filepath, read_images=True)
        self.length = self.reader.length()
        
        if self.length is None or self.length == 0:
            raise ValueError(f"Empty or invalid trajectory file: {filepath}")
        
        print(f"Loaded trajectory: {filepath}")
        print(f"  Length: {self.length} timesteps")
        
        # Pre-load all data
        print("Loading trajectory data...")
        self._load_all_data()
        
        # Calculate global statistics for box plot
        print("Calculating global statistics...")
        self._calculate_global_stats()
        
        print("Data loaded successfully!")
        
        # Timestamps scroll state
        self._ts_scroll_offset = 0
        self._ts_lines_per_page = 25
        
        # Setup figure
        self._setup_figure()
        
    def _load_all_data(self):
        """Pre-load all trajectory data into memory."""
        self.timesteps = []
        self.camera_ids = []
        
        # Reset reader index
        self.reader._index = 0
        
        for i in range(self.length):
            timestep = self.reader.read_timestep()
            self.timesteps.append(timestep)
            
            # Detect camera IDs from first timestep
            if i == 0:
                self._detect_cameras(timestep)
                if self.debug:
                    self._debug_print_structure(timestep)
        
        print(f"  Detected cameras: {self.camera_ids}")
        
        # Extract time series data
        self._extract_time_series()
        self._extract_timestamps()
    
    def _debug_print_structure(self, timestep: Dict, prefix: str = "", depth: int = 0):
        """Debug print the structure of a timestep."""
        if depth > 5:
            return
        for key, value in timestep.items():
            if isinstance(value, dict):
                print(f"{prefix}{key}: {{")
                self._debug_print_structure(value, prefix + "  ", depth + 1)
                print(f"{prefix}}}")
            elif isinstance(value, np.ndarray):
                print(f"{prefix}{key}: ndarray shape={value.shape}, dtype={value.dtype}")
            else:
                val_str = str(value)[:50] if len(str(value)) > 50 else str(value)
                print(f"{prefix}{key}: {type(value).__name__} = {val_str}")
    
    def _detect_cameras(self, timestep: Dict):
        """Detect camera IDs from timestep."""
        # Try observations first (from reader)
        if "observations" in timestep:
            obs = timestep["observations"]
            if "image" in obs:
                self.camera_ids = sorted(list(obs["image"].keys()))[:3]
            elif "depth" in obs:
                self.camera_ids = sorted(list(obs["depth"].keys()))[:3]
        
        # Also check in observation (not observations)
        if not self.camera_ids and "observation" in timestep:
            obs = timestep["observation"]
            if "image" in obs:
                self.camera_ids = sorted(list(obs["image"].keys()))[:3]
            elif "depth" in obs:
                self.camera_ids = sorted(list(obs["depth"].keys()))[:3]
    
    def _extract_time_series(self):
        """Extract robot state and action time series."""
        self.robot_joint_positions = []
        self.robot_joint_velocities = []
        self.robot_gripper_positions = []
        
        self.action_joint_positions = []
        self.action_joint_velocities = []
        self.action_gripper_positions = []
        
        for timestep in self.timesteps:
            obs = timestep.get("observation", {})
            robot_state = obs.get("robot_state", {})
            
            # Robot state
            self._append_data(self.robot_joint_positions, robot_state.get("joint_positions"))
            self._append_data(self.robot_joint_velocities, robot_state.get("joint_velocities"))
            self._append_scalar(self.robot_gripper_positions, robot_state.get("gripper_position"))
            
            # Action
            action = timestep.get("action", {})
            self._append_data(self.action_joint_positions, action.get("joint_position"))
            self._append_data(self.action_joint_velocities, 
                            action.get("joint_velocity", action.get("joint_velocities")))
            self._append_scalar(self.action_gripper_positions, action.get("gripper_position"))
    
    def _append_data(self, data_list: List, value):
        """Append array data to list."""
        if value is not None:
            data_list.append(np.array(value).flatten())
        else:
            data_list.append(None)
    
    def _append_scalar(self, data_list: List, value):
        """Append scalar data to list."""
        if value is not None:
            if np.isscalar(value):
                data_list.append(float(value))
            else:
                arr = np.array(value).flatten()
                data_list.append(float(arr[0]) if len(arr) > 0 else None)
        else:
            data_list.append(None)
    
    def _extract_timestamps(self):
        """Extract timestamp data for Gantt chart and display."""
        self.step_timestamps = []
        self.raw_timestamps = []  # Store raw timestamps for display
        
        for idx, timestep in enumerate(self.timesteps):
            obs = timestep.get("observation", {})
            ts_data = obs.get("timestamp", {})
            
            # Collect ALL timestamps recursively for display
            raw_ts = {}
            self._collect_timestamps_recursive(ts_data, raw_ts, prefix="")
            self.raw_timestamps.append(raw_ts)
            
            # Extract specific timestamps for Gantt chart
            # Control timestamps are under ts_data["control"]
            control_ts = ts_data.get("control", {})
            # Robot timestamps are under ts_data["robot_state"]
            robot_ts = ts_data.get("robot_state", {})
            # Camera timestamps are under ts_data["cameras"]
            camera_ts = ts_data.get("cameras", {})
            
            step_ts = {
                # Control timestamps
                'step_start': self._get_ts_value(control_ts, 'step_start'),
                'step_end': self._get_ts_value(control_ts, 'step_end'),
                'policy_start': self._get_ts_value(control_ts, 'policy_start'),
                'policy_end': self._get_ts_value(control_ts, 'policy_end'),
                'sleep_start': self._get_ts_value(control_ts, 'sleep_start'),
                'control_start': self._get_ts_value(control_ts, 'control_start'),
                
                # Robot timestamps
                'robot_read_start': self._get_ts_value(camera_ts, 'read_start'),
                'robot_read_end': self._get_ts_value(camera_ts, 'read_end'),
                'robot_polymetis_t': self._get_ts_value(robot_ts, 'robot_polymetis_t'),
                'robot_pub_t': self._get_ts_value(robot_ts, 'robot_pub_t'),
                'robot_sub_t': self._get_ts_value(robot_ts, 'robot_sub_t'),
                'robot_end_t': self._get_ts_value(robot_ts, 'robot_end_t'),
                
                # Camera timestamps
                'camera_read_start': self._get_ts_value(camera_ts, 'read_start'),
                'camera_read_end': self._get_ts_value(camera_ts, 'read_end'),
                'multi_camera_sync_start': self._get_ts_value(camera_ts, 'multi_camera_sync_start'),
                'multi_camera_sync_end': self._get_ts_value(camera_ts, 'multi_camera_sync_end'),
                
                # Per-camera pub timestamps
                'camera_pub_ts': {}
            }
            
            # Extract per-camera timestamps from camera_ts
            for cam_id in self.camera_ids:
                pub_key = f"{cam_id}_pub_t"
                step_ts['camera_pub_ts'][cam_id] = self._get_ts_value(camera_ts, pub_key)
            
            self.step_timestamps.append(step_ts)
            
            # Debug output for first step
            if idx == 0 and self.debug:
                print("\n=== DEBUG: First step timestamps ===")
                print(f"ts_data keys: {list(ts_data.keys())}")
                print(f"control_ts: {control_ts}")
                print(f"robot_ts: {robot_ts}")
                print(f"camera_ts: {camera_ts}")
                print(f"Extracted step_ts: {step_ts}")
                print(f"raw_ts: {raw_ts}")
                print("=" * 50)
    
    def _collect_timestamps_recursive(self, data: Any, result: Dict, prefix: str):
        """Recursively collect all timestamps from nested dict."""
        if not isinstance(data, dict):
            return
        for key, value in data.items():
            full_key = f"{prefix}{key}" if prefix else key
            if isinstance(value, dict):
                self._collect_timestamps_recursive(value, result, f"{full_key}/")
            elif value is not None:
                # Check if it looks like a timestamp (large number or specific keys)
                try:
                    v = float(value)
                    # Timestamps are typically > 1e9 (nanoseconds since epoch)
                    # or could be smaller if relative
                    if v > 1e9 or '_t' in key or 'time' in key.lower() or 'start' in key or 'end' in key:
                        result[full_key] = int(v) if v > 1e6 else v
                except (ValueError, TypeError):
                    pass
    
    def _get_ts_value(self, data: Dict, key: str) -> Optional[float]:
        """Get timestamp value, converting ns to ms."""
        value = data.get(key)
        if value is None:
            return None
        try:
            v = float(value)
            # Assume timestamps are in nanoseconds, convert to ms
            if v > 1e12:  # Likely nanoseconds
                return v / 1e6  # Convert to ms
            elif v > 1e9:  # Likely microseconds or seconds as ns
                return v / 1e6  # Treat as ns, convert to ms
            return v
        except (ValueError, TypeError):
            return None
    
    def _calculate_global_stats(self):
        """Calculate global latency statistics for box plot."""
        self.camera_sync_latencies = []
        self.robot_cam_latencies = []
        self.policy_robot_latencies = []
        
        for idx, step_ts in enumerate(self.step_timestamps):
            # Camera sync latency: max(pub_t) - min(pub_t)
            cam_pub_ts = [v for v in step_ts['camera_pub_ts'].values() if v is not None]
            if len(cam_pub_ts) >= 2:
                latency = max(cam_pub_ts) - min(cam_pub_ts)
                self.camera_sync_latencies.append(latency)
                if idx == 0 and self.debug:
                    print(f"[DEBUG] Step 0 camera_sync_latency: {latency:.3f} ms")
                    print(f"  cam_pub_ts: {cam_pub_ts}")
            
            # Robot-Cam latency: abs(robot_pub_t - max(camera_pub_t))
            robot_pub = step_ts.get('robot_pub_t')
            if robot_pub is not None and cam_pub_ts:
                latency = abs(robot_pub - max(cam_pub_ts))
                self.robot_cam_latencies.append(latency)
                if idx == 0 and self.debug:
                    print(f"[DEBUG] Step 0 robot_cam_latency: {latency:.3f} ms")
                    print(f"  robot_pub_t: {robot_pub}, max(cam_pub_ts): {max(cam_pub_ts)}")
            
            # Policy-Robot latency: step_start - robot_pub_t
            step_start = step_ts.get('step_start')
            if step_start is not None and robot_pub is not None:
                latency = step_start - robot_pub
                self.policy_robot_latencies.append(latency)
                if idx == 0 and self.debug:
                    print(f"[DEBUG] Step 0 policy_robot_latency: {latency:.3f} ms")
                    print(f"  step_start: {step_start}, robot_pub_t: {robot_pub}")
        
        print(f"  Camera sync latencies: {len(self.camera_sync_latencies)} samples")
        print(f"  Robot-Cam latencies: {len(self.robot_cam_latencies)} samples")
        print(f"  Policy-Robot latencies: {len(self.policy_robot_latencies)} samples")
        
        # Debug: print stats if available
        if self.debug:
            if self.camera_sync_latencies:
                arr = np.array(self.camera_sync_latencies)
                print(f"  Camera sync stats: mean={arr.mean():.2f}ms, std={arr.std():.2f}ms")
            if self.robot_cam_latencies:
                arr = np.array(self.robot_cam_latencies)
                print(f"  Robot-Cam stats: mean={arr.mean():.2f}ms, std={arr.std():.2f}ms")
            if self.policy_robot_latencies:
                arr = np.array(self.policy_robot_latencies)
                print(f"  Policy-Robot stats: mean={arr.mean():.2f}ms, std={arr.std():.2f}ms")
    
    def _setup_figure(self):
        """Setup matplotlib figure with GridSpec for custom layout."""
        self.fig = plt.figure(figsize=(18, 14))
        
        # Create GridSpec: 4 rows, 4 columns
        gs = GridSpec(4, 4, figure=self.fig, 
                     height_ratios=[1, 1, 1, 1],
                     width_ratios=[1, 1, 1, 1],
                     hspace=0.35, wspace=0.3)
        
        # Create axes dictionary for easy access
        self.axes = {}
        
        # Row 0: RGB images (cols 0-2) + All Timestamps (col 3)
        for i in range(3):
            self.axes[f'rgb_{i}'] = self.fig.add_subplot(gs[0, i])
        self.axes['timestamps'] = self.fig.add_subplot(gs[0, 3])
        
        # Row 1: Depth images (cols 0-2) + Time Markers (col 3)
        for i in range(3):
            self.axes[f'depth_{i}'] = self.fig.add_subplot(gs[1, i])
        self.axes['markers'] = self.fig.add_subplot(gs[1, 3])
        
        # Row 2: Robot state (cols 0-2) + Gantt (col 3)
        self.axes['robot_jp'] = self.fig.add_subplot(gs[2, 0])
        self.axes['robot_jv'] = self.fig.add_subplot(gs[2, 1])
        self.axes['robot_gp'] = self.fig.add_subplot(gs[2, 2])
        self.axes['gantt'] = self.fig.add_subplot(gs[2, 3])
        
        # Row 3: Action (cols 0-2) + Box plot (col 3)
        self.axes['action_jp'] = self.fig.add_subplot(gs[3, 0])
        self.axes['action_jv'] = self.fig.add_subplot(gs[3, 1])
        self.axes['action_gp'] = self.fig.add_subplot(gs[3, 2])
        self.axes['boxplot'] = self.fig.add_subplot(gs[3, 3])
        
        plt.subplots_adjust(bottom=0.1, top=0.93, left=0.05, right=0.98)
        
        # Title
        self.fig.suptitle(f"Trajectory: {Path(self.filepath).name}", fontsize=14, fontweight='bold')
        
        # Create slider
        ax_slider = plt.axes([0.15, 0.02, 0.55, 0.025])
        self.slider = Slider(
            ax_slider, 'Timestep', 0, self.length - 1,
            valinit=0, valstep=1, valfmt='%d'
        )
        self.slider.on_changed(self._update)
        
        # Create scroll buttons for timestamps (positioned above row 0, col 3)
        ax_scroll_up = plt.axes([0.88, 0.91, 0.04, 0.02])
        ax_scroll_down = plt.axes([0.93, 0.91, 0.04, 0.02])
        self.btn_scroll_up = Button(ax_scroll_up, '▲')
        self.btn_scroll_down = Button(ax_scroll_down, '▼')
        self.btn_scroll_up.on_clicked(self._scroll_up)
        self.btn_scroll_down.on_clicked(self._scroll_down)
        
        # Draw static box plot (doesn't change with slider)
        self._draw_box_plot()
        
        # Initial plot
        self._update(0)
    
    def _scroll_up(self, event):
        """Scroll timestamps up."""
        self._ts_scroll_offset = max(0, self._ts_scroll_offset - 5)
        self._update_timestamps_display(int(self.slider.val))
        self.fig.canvas.draw_idle()
    
    def _scroll_down(self, event):
        """Scroll timestamps down."""
        self._ts_scroll_offset += 5
        self._update_timestamps_display(int(self.slider.val))
        self.fig.canvas.draw_idle()
    
    def _draw_box_plot(self):
        """Draw static box plot for global latency statistics."""
        ax = self.axes['boxplot']
        ax.clear()
        
        data = []
        labels = []
        
        if self.camera_sync_latencies:
            data.append(self.camera_sync_latencies)
            labels.append('Cam\nSync')
        
        if self.robot_cam_latencies:
            data.append(self.robot_cam_latencies)
            labels.append('Robot\n-Cam')
        
        if self.policy_robot_latencies:
            data.append(self.policy_robot_latencies)
            labels.append('Policy\n-Robot')
        
        if data:
            bp = ax.boxplot(data, labels=labels, patch_artist=True)
            colors = ['#3498db', '#2ecc71', '#e74c3c']
            for patch, color in zip(bp['boxes'], colors[:len(data)]):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
            
            ax.set_ylabel('Latency (ms)', fontsize=8)
            ax.set_title('Global Latency Stats', fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.3, axis='y')
            ax.tick_params(labelsize=7)
            
            # Add stats text
            stats_text = []
            for i, (d, lbl) in enumerate(zip(data, labels)):
                arr = np.array(d)
                stats_text.append(f"{lbl.replace(chr(10), '')}: μ={arr.mean():.1f}, σ={arr.std():.1f}")
            ax.text(0.02, 0.98, '\n'.join(stats_text), transform=ax.transAxes, fontsize=6,
                   verticalalignment='top', fontfamily='monospace',
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        else:
            # Show debug info when no latency data
            ax.text(0.5, 0.6, 'No Latency Data', ha='center', va='center', 
                   transform=ax.transAxes, fontsize=10, fontweight='bold')
            
            # Show what's missing
            missing = []
            if not self.camera_sync_latencies:
                missing.append("Camera Sync: Need ≥2 camera_pub_t")
            if not self.robot_cam_latencies:
                missing.append("Robot-Cam: Need robot_pub_t & camera_pub_t")
            if not self.policy_robot_latencies:
                missing.append("Policy-Robot: Need step_start & robot_pub_t")
            
            ax.text(0.5, 0.3, '\n'.join(missing), ha='center', va='center', 
                   transform=ax.transAxes, fontsize=7, color='red')
            ax.set_title('Global Latency Stats', fontsize=10)
    
    def _get_image(self, timestep: Dict, image_type: str, camera_id: str):
        """Get image from timestep."""
        # Try observations first (from reader)
        if "observations" in timestep:
            obs = timestep["observations"]
            if image_type in obs and camera_id in obs[image_type]:
                return obs[image_type][camera_id]
        
        # Try observation (original structure)
        if "observation" in timestep:
            obs = timestep["observation"]
            if image_type in obs and camera_id in obs[image_type]:
                return obs[image_type][camera_id]
        
        return None
    
    def _format_timestamp_ns(self, ns: int) -> str:
        """Format nanosecond timestamp to readable string."""
        try:
            # Convert ns to seconds
            seconds = ns / 1e9
            dt = datetime.fromtimestamp(seconds)
            # Format with milliseconds
            ms = int((ns % 1e9) / 1e6)
            return dt.strftime('%H:%M:%S.') + f'{ms:03d}'
        except (ValueError, OSError, OverflowError):
            # For relative timestamps or small values
            if ns < 1e9:
                return f"{ns / 1e6:.3f}ms"
            return f"{ns}"
    
    def _update(self, val):
        """Update visualization for given timestamp."""
        t = int(val)
        timestep = self.timesteps[t]
        
        # Reset scroll offset when changing timestep
        self._ts_scroll_offset = 0
        
        # Update images (Rows 0-1, cols 0-2)
        self._update_images(t, timestep)
        
        # Update timestamps display (Row 0, col 3)
        self._update_timestamps_display(t)
        
        # Update markers display (Row 1, col 3)
        self._update_markers_display(t)
        
        # Update robot state plots (Row 2, cols 0-2)
        self._update_robot_state(t)
        
        # Update Gantt chart (Row 2, col 3) - bars only
        self._update_gantt_chart(t)
        
        # Update action plots (Row 3, cols 0-2)
        self._update_actions(t)
        
        # Box plot is static, no update needed
        
        # Update figure title
        self.fig.suptitle(
            f"Trajectory: {Path(self.filepath).name} | Timestep: {t}/{self.length-1}",
            fontsize=14, fontweight='bold'
        )
        
        self.fig.canvas.draw_idle()
    
    def _update_images(self, t: int, timestep: Dict):
        """Update camera image displays."""
        # Row 0: RGB images
        for i in range(3):
            ax = self.axes[f'rgb_{i}']
            ax.clear()
            
            if i < len(self.camera_ids):
                cam_id = self.camera_ids[i]
                img = self._get_image(timestep, 'image', cam_id)
                if img is not None:
                    ax.imshow(img)
                    ax.set_title(f"{cam_id} (RGB)", fontsize=9)
                else:
                    ax.text(0.5, 0.5, 'No Image', ha='center', va='center', 
                           transform=ax.transAxes)
                    ax.set_title(f"{cam_id} (RGB)", fontsize=9)
            ax.axis('off')
        
        # Row 1: Depth images
        for i in range(3):
            ax = self.axes[f'depth_{i}']
            ax.clear()
            
            if i < len(self.camera_ids):
                cam_id = self.camera_ids[i]
                depth = self._get_image(timestep, 'depth', cam_id)
                if depth is not None:
                    depth_display = depth.astype(np.float32)
                    if depth_display.max() > 0:
                        depth_display = depth_display / depth_display.max()
                    ax.imshow(depth_display, cmap='viridis')
                    ax.set_title(f"{cam_id} (Depth)", fontsize=9)
                else:
                    ax.text(0.5, 0.5, 'No Depth', ha='center', va='center', 
                           transform=ax.transAxes)
                    ax.set_title(f"{cam_id} (Depth)", fontsize=9)
            ax.axis('off')
    
    def _update_timestamps_display(self, t: int):
        """Update the timestamps display panel with scrolling support."""
        ax = self.axes['timestamps']
        ax.clear()
        ax.axis('off')
        
        raw_ts = self.raw_timestamps[t]
        
        if not raw_ts:
            ax.text(0.5, 0.5, 'No Timestamps Found', ha='center', va='center', 
                   transform=ax.transAxes, fontsize=10)
            ax.set_title('All Timestamps', fontsize=9, fontweight='bold')
            return
        
        # Sort timestamps by key
        sorted_keys = sorted(raw_ts.keys())
        
        # Build all lines (compact format)
        all_lines = []
        all_lines.append(f"Step {t} | {len(raw_ts)} items")
        all_lines.append("─" * 32)
        
        # Group by category
        categories = {
            'control': [],
            'robot': [],
            'cameras': [],
            'other': []
        }
        
        for key in sorted_keys:
            value = raw_ts[key]
            formatted_time = self._format_timestamp_ns(value)
            # Shorten key names for compact display
            short_key = key.replace('control/', 'ctrl/').replace('robot_state/', 'rob/').replace('cameras/', 'cam/')
            entry = f"{short_key}: {formatted_time}"
            
            key_lower = key.lower()
            if key.startswith('control/') or 'step' in key_lower or 'policy' in key_lower or 'sleep' in key_lower:
                categories['control'].append(entry)
            elif key.startswith('robot_state/') or 'robot' in key_lower or 'polymetis' in key_lower:
                categories['robot'].append(entry)
            elif key.startswith('cameras/') or 'camera' in key_lower or 'sync' in key_lower or any(cam in key for cam in self.camera_ids):
                categories['cameras'].append(entry)
            else:
                categories['other'].append(entry)
        
        # Add each category
        if categories['control']:
            all_lines.append("[Control]")
            all_lines.extend(categories['control'])
        
        if categories['robot']:
            all_lines.append("[Robot]")
            all_lines.extend(categories['robot'])
        
        if categories['cameras']:
            all_lines.append("[Cameras]")
            all_lines.extend(categories['cameras'])
        
        if categories['other']:
            all_lines.append("[Other]")
            all_lines.extend(categories['other'])
        
        # Apply scrolling (smaller page for single row)
        lines_per_page = 12
        total_lines = len(all_lines)
        start_idx = min(self._ts_scroll_offset, max(0, total_lines - lines_per_page))
        end_idx = min(start_idx + lines_per_page, total_lines)
        
        visible_lines = all_lines[start_idx:end_idx]
        
        # Add scroll indicator
        if start_idx > 0:
            visible_lines.insert(0, f"▲ ({start_idx} above)")
        if end_idx < total_lines:
            visible_lines.append(f"▼ ({total_lines - end_idx} below)")
        
        # Display text
        text = '\n'.join(visible_lines)
        ax.text(0.02, 0.98, text, transform=ax.transAxes, fontsize=6.5,
               verticalalignment='top', horizontalalignment='left',
               fontfamily='monospace',
               bbox=dict(boxstyle='round', facecolor='#f8f9fa', alpha=0.95, edgecolor='#dee2e6'))
        
        ax.set_title(f'All Timestamps ({total_lines} items)', fontsize=9, fontweight='bold')
    
    def _update_markers_display(self, t: int):
        """Update the time markers visualization panel (Row 1, Col 3)."""
        ax = self.axes['markers']
        ax.clear()
        
        step_ts = self.step_timestamps[t]
        step_start = step_ts.get('step_start')
        
        if step_start is None:
            ax.text(0.5, 0.5, 'No step_start', ha='center', va='center', 
                   transform=ax.transAxes, fontsize=10)
            ax.set_title('Time Markers', fontsize=9, fontweight='bold')
            ax.axis('off')
            return
        
        # Normalize: subtract step_start (convert to relative time in ms)
        def norm(v):
            if v is None:
                return None
            return v - step_start
        
        # Collect ALL markers with their positions
        markers_data = []  # (x_value, y_row, marker, color, label, size)
        legend_entries = {}  # label -> (marker, color) for legend
        all_x_values = []  # Track all x values for dynamic axis range
        
        y_row = 0
        
        # Row 0: Camera pub_t (one per camera)
        for cam_id in self.camera_ids:
            pub_t = step_ts['camera_pub_ts'].get(cam_id)
            if pub_t is not None:
                x = norm(pub_t)
                all_x_values.append(x)
                markers_data.append((x, y_row, 'v', self.MARKER_COLORS['camera_pub'], f'{cam_id[-4:]}', 70))
        if any(step_ts['camera_pub_ts'].get(cam_id) is not None for cam_id in self.camera_ids):
            legend_entries['Cam pub_t'] = ('v', self.MARKER_COLORS['camera_pub'])
        y_row += 1
        
        # Row 1: Camera Sync (start and end)
        sync_start = norm(step_ts.get('multi_camera_sync_start'))
        sync_end = norm(step_ts.get('multi_camera_sync_end'))
        if sync_start is not None:
            all_x_values.append(sync_start)
            markers_data.append((sync_start, y_row, '<', self.MARKER_COLORS['sync'], 'SyncS', 70))
            legend_entries['Sync Start'] = ('<', self.MARKER_COLORS['sync'])
        if sync_end is not None:
            all_x_values.append(sync_end)
            markers_data.append((sync_end, y_row, '>', self.MARKER_COLORS['sync'], 'SyncE', 70))
            legend_entries['Sync End'] = ('>', self.MARKER_COLORS['sync'])
        y_row += 1
        
        # Row 2: Robot timestamps (polymetis_t, pub_t, sub_t, end_t)
        polymetis_t = norm(step_ts.get('robot_polymetis_t'))
        robot_pub_t = norm(step_ts.get('robot_pub_t'))
        robot_sub_t = norm(step_ts.get('robot_sub_t'))
        robot_end_t = norm(step_ts.get('robot_end_t'))
        
        if polymetis_t is not None:
            all_x_values.append(polymetis_t)
            markers_data.append((polymetis_t, y_row, 'D', self.MARKER_COLORS['robot_polymetis'], 'poly', 60))
            legend_entries['Polymetis'] = ('D', self.MARKER_COLORS['robot_polymetis'])
        if robot_pub_t is not None:
            all_x_values.append(robot_pub_t)
            markers_data.append((robot_pub_t, y_row, 's', self.MARKER_COLORS['robot_pub'], 'pub', 60))
            legend_entries['Robot pub'] = ('s', self.MARKER_COLORS['robot_pub'])
        if robot_sub_t is not None:
            all_x_values.append(robot_sub_t)
            markers_data.append((robot_sub_t, y_row, 'o', '#3498db', 'sub', 50))
            legend_entries['Robot sub'] = ('o', '#3498db')
        if robot_end_t is not None:
            all_x_values.append(robot_end_t)
            markers_data.append((robot_end_t, y_row, '^', '#27ae60', 'end', 50))
            legend_entries['Robot end'] = ('^', '#27ae60')
        y_row += 1
        
        # Row 3: Camera read times
        cam_read_start = norm(step_ts.get('camera_read_start'))
        cam_read_end = norm(step_ts.get('camera_read_end'))
        if cam_read_start is not None:
            all_x_values.append(cam_read_start)
            markers_data.append((cam_read_start, y_row, '|', '#f39c12', 'CamRS', 80))
            legend_entries['Cam Read S'] = ('|', '#f39c12')
        if cam_read_end is not None:
            all_x_values.append(cam_read_end)
            markers_data.append((cam_read_end, y_row, '|', '#e67e22', 'CamRE', 80))
            legend_entries['Cam Read E'] = ('|', '#e67e22')
        y_row += 1
        
        # Plot markers
        if not markers_data:
            ax.text(0.5, 0.5, 'No markers available', ha='center', va='center', 
                   transform=ax.transAxes, fontsize=10)
            ax.set_title(f'Step {t} Markers', fontsize=9, fontweight='bold')
            ax.axis('off')
            return
        
        # Draw horizontal lines for each row
        y_labels = ['Cam pub_t', 'Sync', 'Robot', 'Cam Read']
        for row in range(y_row):
            ax.axhline(y=row, color='#e0e0e0', linewidth=1, linestyle='-', zorder=1)
        
        # Plot each marker
        for x, y, marker, color, label, size in markers_data:
            ax.scatter([x], [y], marker=marker, s=size, c=color, 
                      edgecolors='black', linewidths=0.5, zorder=5)
            # Add value label above marker
            ax.annotate(f'{x:.1f}', (x, y), textcoords="offset points", 
                       xytext=(0, 8), ha='center', fontsize=5, color='gray')
        
        # Create legend
        legend_handles = []
        legend_labels = []
        for label, (marker, color) in legend_entries.items():
            handle = ax.scatter([], [], marker=marker, c=color, edgecolors='black', 
                               linewidths=0.5, s=40)
            legend_handles.append(handle)
            legend_labels.append(label)
        
        if legend_handles:
            ax.legend(legend_handles, legend_labels, loc='upper right', fontsize=5,
                     framealpha=0.9, markerscale=0.7, ncol=2)
        
        # Y-axis labels
        ax.set_yticks(range(len(y_labels)))
        ax.set_yticklabels(y_labels, fontsize=6)
        ax.set_ylim(-0.5, y_row - 0.5)
        
        # X-axis: Dynamic range based on all markers
        if all_x_values:
            x_min = min(all_x_values)
            x_max = max(all_x_values)
            x_padding = max(5, (x_max - x_min) * 0.1)  # 10% padding or minimum 5ms
            ax.set_xlim(x_min - x_padding, x_max + x_padding)
        else:
            ax.set_xlim(-5, 100)
        
        ax.set_xlabel('Time (ms)', fontsize=6)
        ax.grid(True, alpha=0.3, axis='x')
        ax.tick_params(labelsize=5)
        
        ax.set_title(f'Step {t} Markers', fontsize=9, fontweight='bold')
    
    def _update_robot_state(self, t: int):
        """Update robot state time series plots."""
        # Joint positions
        ax = self.axes['robot_jp']
        ax.clear()
        self._plot_joint_data(ax, self.robot_joint_positions[:t+1], 
                             'Robot Joint Positions', 'Position (rad)')
        
        # Joint velocities
        ax = self.axes['robot_jv']
        ax.clear()
        self._plot_joint_data(ax, self.robot_joint_velocities[:t+1], 
                             'Robot Joint Velocities', 'Velocity (rad/s)')
        
        # Gripper position
        ax = self.axes['robot_gp']
        ax.clear()
        self._plot_scalar_data(ax, self.robot_gripper_positions[:t+1], 
                              'Robot Gripper', 'Position', 'b-')
    
    def _update_actions(self, t: int):
        """Update action time series plots."""
        # Joint positions
        ax = self.axes['action_jp']
        ax.clear()
        self._plot_joint_data(ax, self.action_joint_positions[:t+1], 
                             'Action Joint Position', 'Position (rad)')
        
        # Joint velocities
        ax = self.axes['action_jv']
        ax.clear()
        self._plot_joint_data(ax, self.action_joint_velocities[:t+1], 
                             'Action Joint Velocity', 'Velocity (rad/s)')
        
        # Gripper position
        ax = self.axes['action_gp']
        ax.clear()
        self._plot_scalar_data(ax, self.action_gripper_positions[:t+1], 
                              'Action Gripper', 'Position', 'r-')
    
    def _plot_joint_data(self, ax, data_list: List, title: str, ylabel: str):
        """Plot joint data (7 DOF)."""
        data = [d for d in data_list if d is not None]
        if data:
            data_arr = np.array(data)
            colors = plt.cm.tab10(np.linspace(0, 1, data_arr.shape[1]))
            for j in range(data_arr.shape[1]):
                ax.plot(range(len(data)), data_arr[:, j], 
                       color=colors[j], label=f'J{j+1}', linewidth=1)
            ax.legend(loc='upper right', fontsize=6, ncol=2)
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', 
                   transform=ax.transAxes)
        ax.set_title(title, fontsize=9, fontweight='bold')
        ax.set_xlabel('Timestep', fontsize=8)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.tick_params(labelsize=7)
    
    def _plot_scalar_data(self, ax, data_list: List, title: str, ylabel: str, style: str):
        """Plot scalar data."""
        data = [d for d in data_list if d is not None]
        if data:
            ax.plot(range(len(data)), data, style, linewidth=2)
            ax.grid(True, alpha=0.3)
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', 
                   transform=ax.transAxes)
        ax.set_title(title, fontsize=9, fontweight='bold')
        ax.set_xlabel('Timestep', fontsize=8)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.tick_params(labelsize=7)
    
    def _update_gantt_chart(self, t: int):
        """Update Gantt chart for current step timing."""
        ax = self.axes['gantt']
        ax.clear()
        
        step_ts = self.step_timestamps[t]
        step_start = step_ts.get('step_start')
        
        if step_start is None:
            # Show debug info
            ax.text(0.5, 0.6, 'No Timing Data', ha='center', va='center', 
                   transform=ax.transAxes, fontweight='bold')
            
            # Show what keys we have
            available = [k for k, v in step_ts.items() if v is not None and k != 'camera_pub_ts']
            cam_ts = [f"{k}={v:.1f}" for k, v in step_ts['camera_pub_ts'].items() if v is not None]
            
            debug_text = f"Available: {available}\nCam pub_t: {cam_ts}"
            ax.text(0.5, 0.3, debug_text, ha='center', va='center', 
                   transform=ax.transAxes, fontsize=6, color='gray')
            
            ax.set_title(f'Step_{t}_Periods', fontsize=9)
            return
        
        # Normalize: subtract step_start (convert to relative time)
        def norm(v):
            if v is None:
                return None
            return v - step_start
        
        # ===== BARS (with y-axis labels) =====
        bar_labels = []
        bar_y = 0
        
        # Bar 1: robot_state/read (robot_read_start to robot_read_end)
        # Note: robot read times might be stored separately or use robot_sub_t/robot_end_t
        robot_sub = norm(step_ts.get('robot_sub_t'))
        robot_end = norm(step_ts.get('robot_end_t'))
        if robot_sub is not None and robot_end is not None:
            width = max(0.5, robot_end - robot_sub)
            ax.barh(bar_y, width, left=robot_sub, height=0.6, 
                   color=self.GANTT_COLORS['robot_read'])
            bar_labels.append(('Robot Read', bar_y))
            bar_y += 1
        
        # Bar 2: control/policy_active (policy_start to sleep_start)
        ps = norm(step_ts.get('policy_start'))
        ss = norm(step_ts.get('sleep_start'))
        if ps is not None and ss is not None:
            width = max(0.5, ss - ps)
            ax.barh(bar_y, width, left=ps, height=0.6,
                   color=self.GANTT_COLORS['policy_active'])
            bar_labels.append(('Policy Active', bar_y))
            bar_y += 1
        
        # Bar 3: control/policy_sleep (sleep_start to control_start or policy_end)
        cs = norm(step_ts.get('control_start'))
        pe = norm(step_ts.get('policy_end'))
        sleep_end = cs if cs is not None else pe
        if ss is not None and sleep_end is not None:
            width = max(0.5, sleep_end - ss)
            ax.barh(bar_y, width, left=ss, height=0.6,
                   color=self.GANTT_COLORS['policy_sleep'])
            bar_labels.append(('Policy Sleep', bar_y))
            bar_y += 1
        
        # Bar 4: control/step (step_start to step_end, relative: 0 to step_end-step_start)
        se = norm(step_ts.get('step_end'))
        if se is not None:
            ax.barh(bar_y, se, left=0, height=0.6,
                   color=self.GANTT_COLORS['control_step'])
            bar_labels.append(('Control Step', bar_y))
            bar_y += 1
        
        # Bar 5: cameras/read (camera_read_start to camera_read_end)
        crs = norm(step_ts.get('camera_read_start'))
        cre = norm(step_ts.get('camera_read_end'))
        if crs is not None and cre is not None:
            width = max(0.5, cre - crs)
            ax.barh(bar_y, width, left=crs, height=0.6, 
                   color=self.GANTT_COLORS['camera_read'])
            bar_labels.append(('Camera Read', bar_y))
            bar_y += 1
        
        # Format y-axis: show bar labels
        if bar_labels:
            ax.set_yticks([y for _, y in bar_labels])
            ax.set_yticklabels([lbl for lbl, _ in bar_labels], fontsize=7)
            ax.set_ylim(-0.5, bar_y - 0.5)
        
        ax.set_xlabel('Time (ms)', fontsize=8)
        ax.set_title(f'Step_{t}_Periods', fontsize=9, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        ax.tick_params(labelsize=7)
        
        # Set reasonable x limits
        ax.set_xlim(left=-5)
    
    def show(self):
        """Display the visualization."""
        plt.show()
    
    def close(self):
        """Close the reader."""
        self.reader.close()


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Advanced trajectory visualizer for HDF5 robot learning datasets',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Layout (4x4 Grid):
  Row 0: Camera RGB images (cols 0-2) + Timestamps Panel (col 3, merged with row 1)
  Row 1: Camera Depth images (cols 0-2) + Timestamps Panel (col 3, merged)
  Row 2: Robot State (joint_pos, joint_vel, gripper) + Gantt Chart
  Row 3: Actions (joint_pos, joint_vel, gripper) + Global Latency Box Plot

Examples:
    python3 trajectory_visualizer.py /path/to/trajectory.h5
    python3 trajectory_visualizer.py /path/to/trajectory.h5 --debug
        """
    )
    
    parser.add_argument(
        'filepath',
        type=str,
        help='Path to HDF5 trajectory file'
    )
    
    parser.add_argument(
        '--debug', '-d',
        action='store_true',
        help='Enable debug output to show data structure and timestamp extraction'
    )
    
    args = parser.parse_args()
    
    # Check file exists
    if not Path(args.filepath).exists():
        print(f"Error: File not found: {args.filepath}")
        sys.exit(1)
    
    try:
        visualizer = TrajectoryVisualizer(args.filepath, debug=args.debug)
        visualizer.show()
        visualizer.close()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

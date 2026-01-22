#!/usr/bin/env python3
"""
Collect Trajectory - Python script for collecting robot trajectories using VR controller.

This script provides a pure Python interface for trajectory collection.

Usage:
    # Teleoperation only (no saving)
    python3 collect_trajectory_node.py
    
    # Save trajectory with task name
    python3 collect_trajectory_node.py --task pick_and_place
    
    # Full options
    python3 collect_trajectory_node.py \
        --task pick_and_place \
        --save-folder /path/to/save \
        --save-images \
        --action-space cartesian_velocity \
        --control-hz 15.0

Control:
    • Hold GRIP button to start recording and enable movement
    • Long press A (right) or X (left) to mark SUCCESS and SAVE trajectory
    • Long press B (right) or Y (left) to mark FAILURE and DISCARD trajectory
    • Ctrl+C to exit

Author: Role-ROS2 Team
"""

import argparse
import os
import signal
import time
import traceback
import threading
from datetime import datetime
from typing import Optional, Dict

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from role_ros2.controllers.oculus_controller import VRPolicy
from role_ros2.robot_env import RobotEnv
from role_ros2.trajectory_utils.trajectory_writer import TrajectoryWriter
from role_ros2.misc.ros2_utils import get_ros_time_ns


# Version number (fixed, cannot be adjusted)
VERSION = "1.0"

# Long press duration threshold (seconds)
LONG_PRESS_THRESHOLD = 0.5


class TrajectoryGUI:
    """
    GUI for trajectory collection with input dialog, introduction, and camera display.
    
    Uses thread-safe image dictionary for communication with main control loop.
    """
    
    def __init__(self, image_dict: Dict, image_lock: threading.Lock):
        """
        Initialize GUI.
        
        Args:
            image_dict: Thread-safe dictionary to store camera images
            image_lock: Lock for thread-safe access to image_dict
        """
        self.image_dict = image_dict
        self.image_lock = image_lock
        self.task_name = None
        self.user = None
        self.scene = None
        self.save_images = True
        self.save_depths = False
        self.gui_ready = False
        self.recording_active = False
        
        # GUI state
        self.root = None
        self.current_frame = None
        self.camera_labels = {}
        
        # Start GUI thread
        self.gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self.gui_thread.start()
    
    def _run_gui(self):
        """Run GUI in separate thread."""
        try:
            import tkinter as tk
            from tkinter import ttk, messagebox
            
            self.tk = tk
            self.ttk = ttk
            self.messagebox = messagebox
            
            self.root = tk.Tk()
            self.root.title("Trajectory Collection")
            self.root.geometry("800x600")
            
            # Hide main window initially (will show after input dialog)
            self.root.withdraw()
            
            # Show input dialog first (modal)
            self._show_input_dialog()
            
            # Show main window after dialog is confirmed
            self.root.deiconify()
            
            # Start GUI main loop
            self.root.mainloop()
        except Exception as e:
            print(f"GUI Error: {e}\n{traceback.format_exc()}")
    
    def _show_input_dialog(self):
        """Show input dialog for task name, user, scene and save options."""
        # Create dialog as top-level window (not child of root since root is hidden)
        dialog = self.tk.Toplevel(self.root)
        dialog.title("Trajectory Collection Setup")
        dialog.geometry("500x450")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.focus_set()
        
        # Center the dialog
        dialog.update_idletasks()
        x = (dialog.winfo_screenwidth() // 2) - (500 // 2)
        y = (dialog.winfo_screenheight() // 2) - (450 // 2)
        dialog.geometry(f"500x450+{x}+{y}")
        
        # User input
        self.tk.Label(dialog, text="User:", font=("Arial", 12)).pack(pady=5)
        user_entry = self.tk.Entry(dialog, font=("Arial", 12), width=30)
        user_entry.insert(0, "superman")  # Default value
        user_entry.pack(pady=5)
        
        # Scene input
        self.tk.Label(dialog, text="Scene:", font=("Arial", 12)).pack(pady=5)
        scene_entry = self.tk.Entry(dialog, font=("Arial", 12), width=30)
        scene_entry.insert(0, "franka_irobman")  # Default value
        scene_entry.pack(pady=5)
        
        # Task name input
        self.tk.Label(dialog, text="Task Name:", font=("Arial", 12)).pack(pady=5)
        task_entry = self.tk.Entry(dialog, font=("Arial", 12), width=30)
        task_entry.pack(pady=5)
        task_entry.focus()
        
        # Save images checkbox
        save_images_var = self.tk.BooleanVar(value=True)
        save_images_check = self.tk.Checkbutton(
            dialog, 
            text="Save Images (RGB)", 
            variable=save_images_var,
            font=("Arial", 11)
        )
        save_images_check.pack(pady=5)
        
        # Save depths checkbox
        save_depths_var = self.tk.BooleanVar(value=False)
        save_depths_check = self.tk.Checkbutton(
            dialog, 
            text="Save Depths", 
            variable=save_depths_var,
            font=("Arial", 11)
        )
        save_depths_check.pack(pady=5)
        
        def confirm():
            task_name = task_entry.get().strip()
            if not task_name:
                self.messagebox.showwarning("Warning", "Please enter a task name")
                return
            
            self.task_name = task_name
            self.user = user_entry.get().strip() or "superman"
            self.scene = scene_entry.get().strip() or "franka_irobman"
            self.save_images = save_images_var.get()
            self.save_depths = save_depths_var.get()
            dialog.destroy()
            self._show_introduction()
        
        # Confirm button
        confirm_btn = self.tk.Button(
            dialog, 
            text="Confirm", 
            command=confirm,
            font=("Arial", 12),
            bg="#4CAF50",
            fg="white",
            padx=20,
            pady=5
        )
        confirm_btn.pack(pady=20)
        
        # Bind Enter key
        task_entry.bind('<Return>', lambda e: confirm())
    
    def _show_introduction(self):
        """Show introduction screen."""
        # Clear root window
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Introduction text
        intro_frame = self.tk.Frame(self.root, padx=20, pady=20)
        intro_frame.pack(fill=self.tk.BOTH, expand=True)
        
        title = self.tk.Label(
            intro_frame,
            text="Trajectory Collection",
            font=("Arial", 24, "bold"),
            fg="#2196F3"
        )
        title.pack(pady=20)
        
        instructions = """
CONTROL INSTRUCTIONS:

🎮 Hold GRIP button      → Start recording & move robot
✅ Long press A/X (0.5s)  → SUCCESS: Save & Reset
❌ Long press B/Y (0.5s)  → FAILURE: Discard & Reset
🛑 Ctrl+C                → Exit program

User: {user}
Scene: {scene}
Task: {task_name}
Save Images: {save_images}
Save Depths: {save_depths}

Click 'Start Recording' when ready to begin.
        """.format(
            user=self.user or "superman",
            scene=self.scene or "franka_irobman",
            task_name=self.task_name,
            save_images="Yes" if self.save_images else "No",
            save_depths="Yes" if self.save_depths else "No"
        )
        
        text_label = self.tk.Label(
            intro_frame,
            text=instructions,
            font=("Arial", 12),
            justify=self.tk.LEFT
        )
        text_label.pack(pady=20)
        
        def start_recording():
            self.recording_active = True
            self.gui_ready = True
            self._show_camera_display()
        
        start_btn = self.tk.Button(
            intro_frame,
            text="Start Recording",
            command=start_recording,
            font=("Arial", 14, "bold"),
            bg="#4CAF50",
            fg="white",
            padx=30,
            pady=10
        )
        start_btn.pack(pady=20)
    
    def _show_camera_display(self):
        """Show camera display screen."""
        # Clear root window
        for widget in self.root.winfo_children():
            widget.destroy()
        
        # Main frame
        main_frame = self.tk.Frame(self.root)
        main_frame.pack(fill=self.tk.BOTH, expand=True, padx=10, pady=10)
        
        # Status bar
        status_frame = self.tk.Frame(main_frame)
        status_frame.pack(fill=self.tk.X, pady=5)
        
        self.status_label = self.tk.Label(
            status_frame,
            text="Status: Waiting for recording to start...",
            font=("Arial", 12),
            anchor="w"
        )
        self.status_label.pack(side=self.tk.LEFT)
        
        # Camera display frame
        self.camera_frame = self.tk.Frame(main_frame, bg="black")
        self.camera_frame.pack(fill=self.tk.BOTH, expand=True)
        
        # Initialize camera labels (will be created dynamically)
        self.camera_labels = {}
        self.current_frame = None
        
        # Start update loop
        self._update_camera_display()
    
    def _update_camera_display(self):
        """Update camera display with latest images."""
        if not self.root or not self.recording_active:
            return
        
        try:
            # Get latest images with lock
            with self.image_lock:
                images = self.image_dict.copy()
            
            # Update status
            if images:
                status_text = f"Status: Recording - {len(images)} camera(s) active"
            else:
                status_text = "Status: Waiting for camera data..."
            
            if hasattr(self, 'status_label'):
                self.status_label.config(text=status_text)
            
            # Update camera displays
            if images:
                # Create or clear current frame
                if self.current_frame is None:
                    self.current_frame = self.tk.Frame(self.camera_frame)
                    self.current_frame.pack(fill=self.tk.BOTH, expand=True)
                else:
                    # Clear existing labels
                    for widget in self.current_frame.winfo_children():
                        widget.destroy()
                
                # Create grid layout for cameras
                num_cameras = len(images)
                cols = 2 if num_cameras > 1 else 1
                rows = (num_cameras + cols - 1) // cols
                
                for idx, (camera_id, img) in enumerate(images.items()):
                    if img is None:
                        continue
                    
                    # Resize image for display (max 400x300)
                    display_img = img.copy()
                    h, w = display_img.shape[:2]
                    max_w, max_h = 400, 300
                    
                    if w > max_w or h > max_h:
                        scale = min(max_w / w, max_h / h)
                        new_w, new_h = int(w * scale), int(h * scale)
                        display_img = cv2.resize(display_img, (new_w, new_h))
                    
                    # Convert BGR to RGB for tkinter
                    display_img_rgb = cv2.cvtColor(display_img, cv2.COLOR_BGR2RGB)
                    
                    # Convert to PhotoImage
                    try:
                        from PIL import Image, ImageTk
                        pil_img = Image.fromarray(display_img_rgb)
                        photo = ImageTk.PhotoImage(image=pil_img)
                    except ImportError:
                        # Fallback if PIL not available
                        continue
                    
                    # Create label frame
                    label_frame = self.tk.Frame(self.current_frame)
                    label_frame.grid(row=idx // cols, column=idx % cols, padx=5, pady=5)
                    
                    camera_name_label = self.tk.Label(
                        label_frame,
                        text=f"Camera: {camera_id}",
                        font=("Arial", 10, "bold"),
                        bg="black",
                        fg="white"
                    )
                    camera_name_label.pack()
                    
                    img_label = self.tk.Label(label_frame, image=photo, bg="black")
                    img_label.pack()
                    img_label.image = photo  # Keep a reference
                    
                    self.camera_labels[camera_id] = {
                        'frame': label_frame,
                        'name_label': camera_name_label,
                        'img_label': img_label
                    }
            
        except Exception as e:
            print(f"GUI update error: {e}")
        
        # Schedule next update (30 FPS)
        if self.root:
            self.root.after(33, self._update_camera_display)
    
    def wait_for_ready(self):
        """Wait for GUI to be ready (user confirmed input)."""
        while not self.gui_ready:
            time.sleep(0.1)
    
    def get_task_config(self):
        """Get task configuration from GUI."""
        return {
            'task_name': self.task_name,
            'user': self.user,
            'scene': self.scene,
            'save_images': self.save_images,
            'save_depths': self.save_depths
        }
    
    def shutdown(self):
        """Shutdown GUI."""
        if self.root:
            try:
                self.root.quit()
                self.root.destroy()
            except Exception:
                pass


class CollectTrajectory:
    """
    Python class for collecting robot trajectories using VR controller.
    
    This class:
    1. Initializes RobotEnv and VRPolicy (controller)
    2. Continuously collects observations from RobotEnv
    3. Computes actions from VRPolicy
    4. Executes actions on RobotEnv
    5. Optionally saves trajectory data (if task is provided)
    
    Control Logic:
    - First grip button press starts recording automatically
    - Always loops (no single trajectory mode)
    - Long press A/X: SUCCESS → Save trajectory, start new trajectory
    - Long press B/Y: FAILURE → Discard trajectory, reset robot, start new trajectory
    - Ctrl+C: Clean shutdown
    """
    
    def __init__(self, args):
        """Initialize CollectTrajectory with parsed arguments."""
        # Parse arguments
        self.save_folder = args.save_folder
        self.task_name = args.task
        self.user = args.user
        self.scene = args.scene
        self.action_space = args.action_space
        self.control_hz = args.control_hz
        self.reset_robot_on_start = args.reset_robot
        self.randomize_reset = args.randomize_reset
        self.wait_for_controller = args.wait_for_controller
        self.save_images = args.save_images
        self.save_depths = args.save_depths
        self.right_controller = args.right_controller
        self.horizon = None if args.horizon <= 0 else args.horizon
        self.enable_viz = args.viz
        
        # GUI setup (if enabled)
        self.gui = None
        self.image_dict = {}
        self.image_lock = threading.Lock()
        
        if self.enable_viz:
            self.gui = TrajectoryGUI(self.image_dict, self.image_lock)
            # Wait for GUI input
            self.gui.wait_for_ready()
            # Get task config from GUI
            gui_config = self.gui.get_task_config()
            if not self.task_name:
                self.task_name = gui_config['task_name']
            if not self.user:
                self.user = gui_config.get('user', 'superman')
            if not self.scene:
                self.scene = gui_config.get('scene', 'franka_irobman')
            if not args.save_images:  # Only override if not explicitly set
                self.save_images = gui_config['save_images']
            if not args.save_depths:  # Only override if not explicitly set
                self.save_depths = gui_config['save_depths']
        
        # Set defaults if not provided
        if not self.user:
            self.user = "superman"
        if not self.scene:
            self.scene = "franka_irobman"
        
        # Validate: save_depths requires save_images
        if self.save_depths and not self.save_images:
            raise ValueError("--save-depths requires --save-images. Cannot save depth without images.")
        
        # Build save_folder structure: success/failure folders (like droid)
        # Auto-enable saving when task is provided
        if self.task_name:
            self.save_trajectory = True
            # Create success and failure directories
            self.success_logdir = os.path.join(self.save_folder, "success")
            self.failure_logdir = os.path.join(self.save_folder, "failure")
            os.makedirs(self.success_logdir, exist_ok=True)
            os.makedirs(self.failure_logdir, exist_ok=True)
        else:
            self.save_folder = ''
            self.task_name = None
            self.save_trajectory = False  # No saving if no task
            self.success_logdir = None
            self.failure_logdir = None
        
        # State variables
        self._num_steps = 0
        self._traj_count = 0
        self._shutdown_requested = False
        self._traj_writer: Optional[TrajectoryWriter] = None
        self._current_traj_filepath: Optional[str] = None
        self._recording_started = False  # Track if recording has started (after A button press)
        
        # Long press detection state
        self._success_button_press_start: Optional[float] = None
        self._failure_button_press_start: Optional[float] = None
        
        # Initialize ROS2 node (for RobotEnv)
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('collect_trajectory_node')
        
        # Initialize RobotEnv
        self._print("=" * 70)
        self._print("🚀 Collect Trajectory - Starting Initialization")
        self._print("=" * 70)
        self._print("📝 Parameters:")
        self._print(f"   • Action space: {self.action_space}")
        self._print(f"   • Control Hz: {self.control_hz}")
        self._print(f"   • Wait for controller: {self.wait_for_controller}")
        self._print(f"   • Right controller: {self.right_controller}")
        self._print(f"   • Horizon: {self.horizon if self.horizon else 'unlimited'}")
        self._print("-" * 70)
        
        self._print("🔧 [1/4] Initializing RobotEnv...")
        try:
            self.env = RobotEnv(
                action_space=self.action_space,
                do_reset=False,
                node=self._node,
            )
            self.env.control_hz = self.control_hz
            self._print("   ✅ RobotEnv initialized successfully")
        except Exception as e:
            self._print(f"   ❌ Failed to initialize RobotEnv: {e}")
            raise
        
        # Initialize VRPolicy (controller)
        self._print("🎮 [2/4] Initializing VRPolicy controller...")
        try:
            self.controller = VRPolicy(right_controller=self.right_controller)
            controller_side = "RIGHT" if self.right_controller else "LEFT"
            self._print(f"   ✅ VRPolicy initialized ({controller_side} controller)")
        except Exception as e:
            self._print(f"   ❌ Failed to initialize VRPolicy: {e}")
            raise
        
        # Reset robot if requested
        if self.reset_robot_on_start:
            self._print("🤖 [3/4] Resetting robot to home position...")
            try:
                self.env.reset(randomize=self.randomize_reset)
                self._print("   ✅ Robot reset completed")
            except Exception as e:
                self._print(f"   ⚠️ Robot reset failed: {e}")
        else:
            self._print("🤖 [3/4] Skipping robot reset (reset_robot=false)")
        
        # Mode info
        self._print("📁 [4/4] Setting up trajectory recording...")
        if self.save_trajectory and self.save_folder:
            os.makedirs(self.save_folder, exist_ok=True)
            self._print(f"   ✅ Mode: Recording trajectory")
            self._print(f"   📂 Save folder: {self.save_folder}")
            self._print(f"   📋 Task name: {self.task_name}")
            self._print(f"   🖼️ Save images: {self.save_images}")
            self._print(f"   📏 Save depths: {self.save_depths}")
        else:
            self._print("   ✅ Mode: Teleoperation only (no saving)")
            self._print("   ℹ️  No task specified (provide --task to enable saving)")
        
        # Start new trajectory
        self._start_new_trajectory()
        
        self._print("-" * 70)
        self._print("✅ Initialization Complete!")
        self._print("=" * 70)
        self._print("")
        self._print("📋 CONTROL INSTRUCTIONS:")
        self._print("   ┌─────────────────────────────────────────────────────────┐")
        self._print("   │  🎮 Hold GRIP button      → Start recording & move robot │")
        self._print(f"   │  ✅ Long press A/X ({LONG_PRESS_THRESHOLD}s)  → SUCCESS: Save & Reset        │")
        self._print(f"   │  ❌ Long press B/Y ({LONG_PRESS_THRESHOLD}s)  → FAILURE: Discard & Reset     │")
        self._print("   │  🛑 Ctrl+C               → Exit program                │")
        self._print("   └─────────────────────────────────────────────────────────┘")
        self._print("")
        self._print("=" * 70)
        self._print("🎯 Ready! Hold GRIP button to start recording...")
    
    def _print(self, msg: str):
        """Print message with timestamp."""
        print(msg)
    
    def _start_new_trajectory(self):
        """Start a new trajectory recording."""
        self._traj_count += 1
        self._num_steps = 0
        self._recording_started = False  # Reset recording state
        
        # Reset long press detection
        self._success_button_press_start = None
        self._failure_button_press_start = None
        
        # Reset controller state
        self.controller.reset_state()
        
        # Create trajectory writer if task is provided (auto-enable saving)
        if self.save_trajectory and self.save_folder:
            # Generate time string (like droid: time.asctime().replace(" ", "_"))
            time_str = time.asctime().replace(" ", "_")
            
            # Save to failure folder initially (will be moved to success if successful)
            traj_dir = os.path.join(self.failure_logdir, time_str)
            os.makedirs(traj_dir, exist_ok=True)
            
            self._current_traj_filepath = os.path.join(traj_dir, "trajectory.h5")
            self._current_traj_dir = traj_dir
            self._current_time_str = time_str
            
            # Build metadata (like droid)
            metadata = {
                "trajectory_id": self._traj_count,
                "time": time_str,
                "user": self.user,
                "scene": self.scene,
                "version": VERSION,
            }
            if self.task_name:
                metadata["task_name"] = self.task_name
            
            self._traj_writer = TrajectoryWriter(
                self._current_traj_filepath, 
                metadata=metadata,
                save_images=self.save_images,
                save_depths=self.save_depths
            )
            self._print(f"📁 Recording trajectory #{self._traj_count}...")
        else:
            self._traj_writer = None
            self._current_traj_filepath = None
            self._current_traj_dir = None
            self._current_time_str = None
            self._print(f"🎮 Trajectory #{self._traj_count} (teleoperation only)")
    
    def _check_long_press(self, controller_info: dict) -> tuple:
        """
        Check for long press of SUCCESS (A/X) or FAILURE (B/Y) buttons.
        
        Returns:
            tuple: (is_success_long_press, is_failure_long_press)
        """
        current_time = time.time()
        success_pressed = controller_info.get("success", False)
        failure_pressed = controller_info.get("failure", False)
        
        # Check SUCCESS button (A/X)
        if success_pressed:
            if self._success_button_press_start is None:
                self._success_button_press_start = current_time
            press_duration = current_time - self._success_button_press_start
            is_success_long_press = press_duration >= LONG_PRESS_THRESHOLD
        else:
            self._success_button_press_start = None
            is_success_long_press = False
        
        # Check FAILURE button (B/Y)
        if failure_pressed:
            if self._failure_button_press_start is None:
                self._failure_button_press_start = current_time
            press_duration = current_time - self._failure_button_press_start
            is_failure_long_press = press_duration >= LONG_PRESS_THRESHOLD
        else:
            self._failure_button_press_start = None
            is_failure_long_press = False
        
        return is_success_long_press, is_failure_long_press
    
    def _control_loop(self):
        """Main control loop."""
        if self._shutdown_requested:
            return
        
        try:
            # Get controller info
            controller_info = self.controller.get_info()
            skip_action = self.wait_for_controller and (not controller_info["movement_enabled"])
            
            # Check if recording has started (wait for first grip button press)
            if not self._recording_started:
                movement_enabled = controller_info.get("movement_enabled", False)
                if movement_enabled:
                    self._recording_started = True
                    # Reset long press detection to avoid immediate trigger
                    self._success_button_press_start = None
                    self._failure_button_press_start = None
                    self._print("")
                    self._print("=" * 70)
                    self._print("✅ Recording started! Long press A/X to mark SUCCESS, B/Y to mark FAILURE")
                    self._print("=" * 70)
                    self._print("")
                else:
                    # Not started yet, just wait
                    time.sleep(0.05)
                    return
            
            # Check for long press termination (only after recording started)
            is_success, is_failure = self._check_long_press(controller_info)
            
            # Handle trajectory end
            if is_success:
                self._handle_trajectory_success()
                return
            elif is_failure:
                self._handle_trajectory_failure()
                return
            
            # Use ROS time for timestamps (nanoseconds)
            control_timestamps = {"step_start": get_ros_time_ns(self._node)}
            
            # Get Observation
            obs = self.env.get_observation(use_sync=False)
            obs["controller_info"] = controller_info
            if "timestamp" not in obs:
                obs["timestamp"] = {}
            obs["timestamp"]["skip_action"] = skip_action
            
            # Update GUI images (if enabled)
            if self.enable_viz and self.gui and self.gui.recording_active:
                with self.image_lock:
                    # Copy camera images for GUI
                    if "image" in obs:
                        self.image_dict.clear()
                        for camera_id, img in obs["image"].items():
                            if img is not None:
                                self.image_dict[camera_id] = img.copy()
            
            # Get Action
            control_timestamps["policy_start"] = get_ros_time_ns(self._node)
            action, controller_action_info = self.controller.forward(obs, include_info=True)
            control_timestamps["policy_end"] = get_ros_time_ns(self._node)
            
            # Regularize Control Frequency
            control_timestamps["sleep_start"] = get_ros_time_ns(self._node)
            comp_time_ns = get_ros_time_ns(self._node) - control_timestamps["step_start"]
            comp_time_s = comp_time_ns / 1e9
            sleep_left = (1 / self.env.control_hz) - comp_time_s
            if sleep_left > 0:
                time.sleep(sleep_left)
            
            # Step Environment
            control_timestamps["control_start"] = get_ros_time_ns(self._node)
            if skip_action:
                action_info = self.env.create_action_dict(np.zeros_like(action))
            else:
                action_info = self.env.step(action)
            action_info.update(controller_action_info)
            
            # Save Data
            control_timestamps["step_end"] = get_ros_time_ns(self._node)
            obs["timestamp"]["control"] = control_timestamps
            timestep = {"observation": obs, "action": action_info}
            if self._traj_writer is not None:
                self._traj_writer.write_timestep(timestep)
            
            # Check horizon termination
            self._num_steps += 1
            if self.horizon is not None and self._num_steps >= self.horizon:
                self._print(f"⏱️ Reached horizon ({self.horizon} steps)")
                self._handle_trajectory_success()
                return
            
            # Progress logging
            if self._num_steps % 50 == 0 and self._num_steps > 0:
                movement_status = "🟢 MOVING" if controller_info["movement_enabled"] else "🔴 STOPPED"
                self._print(f"Step {self._num_steps}: {movement_status}")
                
        except Exception as e:
            self._print(f"Error in control loop: {e}\n{traceback.format_exc()}")
    
    def _handle_trajectory_success(self):
        """Handle successful trajectory completion - SAVE, reset robot, and start new."""
        self._print("")
        self._print("=" * 70)
        self._print(f"✅ Trajectory #{self._traj_count} SUCCESS")
        self._print(f"   Total steps: {self._num_steps}")
        
        # Close and save trajectory (like droid: both success and failure in metadata)
        if self._traj_writer is not None:
            self._traj_writer.close(metadata={"success": True, "failure": False})
            
            # Move from failure folder to success folder (like droid)
            if self._current_traj_dir and os.path.exists(self._current_traj_dir):
                new_traj_dir = os.path.join(self.success_logdir, self._current_time_str)
                try:
                    os.rename(self._current_traj_dir, new_traj_dir)
                    self._print(f"💾 Saved: {os.path.join(new_traj_dir, 'trajectory.h5')}")
                except Exception as e:
                    self._print(f"⚠️ Failed to move trajectory to success folder: {e}")
                    self._print(f"💾 Saved: {self._current_traj_filepath}")
            else:
                self._print(f"💾 Saved: {self._current_traj_filepath}")
        
        self._print("=" * 70)
        
        # Reset robot after success
        self._print("🤖 Resetting robot to home position...")
        try:
            self.env.reset(randomize=self.randomize_reset)
            self._print("✅ Robot reset completed")
        except Exception as e:
            self._print(f"⚠️ Robot reset failed: {e}")
        
        # Wait a moment then start new trajectory
        time.sleep(0.5)
        self._print("🔄 Starting new trajectory...")
        self._start_new_trajectory()
        self._print("⏸️  Hold GRIP button to start recording...")
    
    def _handle_trajectory_failure(self):
        """Handle failed trajectory - SAVE to failure folder and reset robot."""
        self._print("")
        self._print("=" * 70)
        self._print(f"❌ Trajectory #{self._traj_count} FAILURE")
        self._print(f"   Total steps: {self._num_steps}")
        
        # Save trajectory to failure folder (like droid: both success and failure in metadata)
        if self._traj_writer is not None:
            try:
                self._traj_writer.close(metadata={"success": False, "failure": True})
                # Trajectory is already in failure folder, no need to move
                self._print(f"💾 Saved to failure folder: {self._current_traj_filepath}")
            except Exception as e:
                self._print(f"⚠️ Failed to close trajectory writer: {e}")
        
        self._print("=" * 70)
        
        # Reset robot
        self._print("🤖 Resetting robot...")
        try:
            self.env.reset(randomize=self.randomize_reset)
            self._print("✅ Robot reset completed")
        except Exception as e:
            self._print(f"⚠️ Robot reset failed: {e}")
        
        # Wait a moment then start new trajectory
        time.sleep(0.5)
        self._print("🔄 Starting new trajectory...")
        self._start_new_trajectory()
        self._print("⏸️  Hold GRIP button to start recording...")
    
    def shutdown(self):
        """Clean shutdown."""
        self._shutdown_requested = True
        
        # Shutdown GUI
        if self.gui:
            self.gui.shutdown()
        
        # Close trajectory writer if open
        if self._traj_writer is not None:
            try:
                self._traj_writer.close(metadata={"interrupted": True})
                self._print(f"💾 Saved interrupted trajectory: {self._current_traj_filepath}")
            except Exception as e:
                self._print(f"Error closing trajectory writer: {e}")
        
        # Shutdown RobotEnv
        if hasattr(self, 'env'):
            self.env.shutdown()
        
        # Destroy node
        if hasattr(self, '_node'):
            self._node.destroy_node()


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description='Collect robot trajectories using VR controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Teleoperation only (no saving)
  python3 collect_trajectory_node.py
  
  # Save trajectory with task name
  python3 collect_trajectory_node.py --task pick_and_place
  
  # Full options
  python3 collect_trajectory_node.py \\
      --task pick_and_place \\
      --save-folder /path/to/save \\
      --save-images \\
      --action-space cartesian_velocity \\
      --control-hz 15.0
        """
    )
    
    # Save settings
    parser.add_argument(
        '--save-folder',
        type=str,
        default='/app/ros2_ws/src/role-ros2/data',
        help='Base folder to save trajectory files (default: /app/ros2_ws/src/role-ros2/data)'
    )
    parser.add_argument(
        '--task',
        type=str,
        default='',
        help='Task name. If provided, automatically enables saving to save_folder/success|failure/time/trajectory.h5'
    )
    parser.add_argument(
        '--user',
        type=str,
        default='superman',
        help='User name for metadata (default: superman)'
    )
    parser.add_argument(
        '--scene',
        type=str,
        default='franka_irobman',
        help='Scene name for metadata (default: franka_irobman)'
    )
    parser.add_argument(
        '--save-images',
        action='store_true',
        help='Save RGB images in trajectory files (MP4 video)'
    )
    parser.add_argument(
        '--save-depths',
        action='store_true',
        help='Save depth images in trajectory files (PNG-in-HDF5, lossless). Requires --save-images.'
    )
    
    # Control settings
    parser.add_argument(
        '--action-space',
        type=str,
        default='cartesian_velocity',
        choices=['cartesian_velocity', 'cartesian_position', 'joint_velocity', 'joint_position'],
        help='Action space for robot control (default: cartesian_velocity)'
    )
    parser.add_argument(
        '--control-hz',
        type=float,
        default=15.0,
        help='Control frequency in Hz (default: 15.0)'
    )
    parser.add_argument(
        '--wait-for-controller',
        action='store_true',
        default=True,
        help='Wait for controller movement before executing actions (default: True)'
    )
    parser.add_argument(
        '--no-wait-for-controller',
        dest='wait_for_controller',
        action='store_false',
        help='Disable waiting for controller movement'
    )
    
    # Controller settings
    parser.add_argument(
        '--right-controller',
        action='store_true',
        default=True,
        help='Use right controller (default: True)'
    )
    parser.add_argument(
        '--left-controller',
        dest='right_controller',
        action='store_false',
        help='Use left controller'
    )
    
    # Robot reset settings
    parser.add_argument(
        '--reset-robot',
        action='store_true',
        default=True,
        help='Reset robot to home position on startup (default: True)'
    )
    parser.add_argument(
        '--no-reset-robot',
        dest='reset_robot',
        action='store_false',
        help='Skip robot reset on startup'
    )
    parser.add_argument(
        '--randomize-reset',
        action='store_true',
        help='Add random offset to home position on reset'
    )
    
    # Trajectory settings
    parser.add_argument(
        '--horizon',
        type=int,
        default=-1,
        help='Maximum steps per trajectory (-1 for unlimited, default: -1)'
    )
    
    # Visualization settings
    parser.add_argument(
        '--viz',
        action='store_true',
        help='Enable GUI visualization with camera display'
    )
    
    return parser.parse_args()


def main():
    """Main function."""
    args = parse_args()
    
    collector = None
    
    # Setup signal handler for clean shutdown
    def signal_handler(signum, frame):
        nonlocal collector
        print("\n⚠️ Shutdown signal received (Ctrl+C)")
        if collector is not None:
            collector.shutdown()
        raise KeyboardInterrupt()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        collector = CollectTrajectory(args)
        
        # Main control loop
        # Note: RobotEnv has its own MultiThreadedExecutor that spins the node
        # in a background thread. We just need to keep the main thread alive.
        # Note: Frequency control is handled inside _control_loop(), so no external sleep needed
        
        while rclpy.ok() and not collector._shutdown_requested:
            collector._control_loop()
            
    except KeyboardInterrupt:
        print("👋 Exiting...")
    except Exception as e:
        print(f"❌ Error: {e}\n{traceback.format_exc()}")
    finally:
        if collector is not None:
            collector.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # May already be shutdown


if __name__ == '__main__':
    main()

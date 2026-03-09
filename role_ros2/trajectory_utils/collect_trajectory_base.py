"""
Base class for trajectory collection with VR controller.

Provides CollectTrajectoryBase (abstract), TrajectoryGUI, common argparse helpers,
and run_collector() entry-point.  Robot-specific scripts subclass
CollectTrajectoryBase and override three methods:

    _create_robot(node)          -> BaseRobot or None
    _create_controller()         -> VR policy object
    _get_movement_enabled(info)  -> bool
"""

import argparse
import os
import shutil
import signal
import time
import traceback
import threading
from typing import Optional, Dict

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from role_ros2.robot_env import RobotEnv
from role_ros2.trajectory_utils.trajectory_writer import TrajectoryWriter
from role_ros2.misc.ros2_utils import get_ros_time_ns

VERSION = "1.0"
LONG_PRESS_THRESHOLD = 0.5
MIN_FAILURE_STEPS = 10


# ---------------------------------------------------------------------------
# TrajectoryGUI  (robot-agnostic)
# ---------------------------------------------------------------------------

class TrajectoryGUI:
    """
    GUI for trajectory collection with input dialog, introduction, and camera display.

    Uses thread-safe image dictionary for communication with main control loop.
    Real-time camera reading in separate thread to avoid white flash.
    """

    def __init__(
        self,
        image_dict: Dict,
        image_lock: threading.Lock,
        flip_cameras: Optional[Dict[str, bool]] = None,
        camera_reader=None,
        read_cameras_func=None,
        shutdown_callback=None,
    ):
        self.image_dict = image_dict
        self.image_lock = image_lock
        self.task_name = None
        self.user = None
        self.scene = None
        self.save_images = True
        self.save_depths = False
        self.gui_ready = False
        self.recording_active = False

        self.camera_reader = camera_reader
        self.read_cameras_func = read_cameras_func
        self._camera_read_thread = None
        self._camera_read_active = False

        self.shutdown_callback = shutdown_callback
        self.flip_cameras = flip_cameras or {}

        self.status_message = "Waiting for recording to start..."
        self.status_lock = threading.Lock()

        self.root = None
        self.current_frame = None
        self.camera_labels = {}

        self.gui_thread = threading.Thread(target=self._run_gui, daemon=True)
        self.gui_thread.start()

    # ---- GUI lifecycle -----------------------------------------------------

    def _run_gui(self):
        try:
            import tkinter as tk
            from tkinter import ttk, messagebox

            self.tk = tk
            self.ttk = ttk
            self.messagebox = messagebox

            self.root = tk.Tk()
            self.root.title("Trajectory Collection")
            self.root.geometry("1600x900")

            def on_closing():
                if self.shutdown_callback is not None:
                    self.shutdown_callback()
                else:
                    self.root.destroy()

            self.root.protocol("WM_DELETE_WINDOW", on_closing)
            self.root.withdraw()
            self._show_input_dialog()
            self.root.deiconify()
            self.root.mainloop()
        except Exception as e:
            print(f"GUI Error: {e}\n{traceback.format_exc()}")

    def _show_input_dialog(self):
        dialog = self.tk.Toplevel(self.root)
        dialog.title("Trajectory Collection Setup")
        dialog.geometry("500x450")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.focus_set()

        dialog.update_idletasks()
        x = (dialog.winfo_screenwidth() // 2) - 250
        y = (dialog.winfo_screenheight() // 2) - 225
        dialog.geometry(f"500x450+{x}+{y}")

        self.tk.Label(dialog, text="User:", font=("Arial", 12)).pack(pady=5)
        user_entry = self.tk.Entry(dialog, font=("Arial", 12), width=30)
        user_entry.insert(0, "superman")
        user_entry.pack(pady=5)

        self.tk.Label(dialog, text="Scene:", font=("Arial", 12)).pack(pady=5)
        scene_entry = self.tk.Entry(dialog, font=("Arial", 12), width=30)
        scene_entry.insert(0, "franka_irobman")
        scene_entry.pack(pady=5)

        self.tk.Label(dialog, text="Task Name:", font=("Arial", 12)).pack(pady=5)
        task_entry = self.tk.Entry(dialog, font=("Arial", 12), width=30)
        task_entry.pack(pady=5)
        task_entry.focus()

        save_images_var = self.tk.BooleanVar(value=True)
        self.tk.Checkbutton(dialog, text="Save Images (RGB)", variable=save_images_var, font=("Arial", 11)).pack(pady=5)

        save_depths_var = self.tk.BooleanVar(value=False)
        self.tk.Checkbutton(dialog, text="Save Depths", variable=save_depths_var, font=("Arial", 11)).pack(pady=5)

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

        btn = self.tk.Button(dialog, text="Confirm", command=confirm, font=("Arial", 12),
                             bg="#4CAF50", fg="white", padx=20, pady=5)
        btn.pack(pady=20)
        task_entry.bind('<Return>', lambda e: confirm())

    def _show_introduction(self):
        for widget in self.root.winfo_children():
            widget.destroy()

        intro_frame = self.tk.Frame(self.root, padx=20, pady=20)
        intro_frame.pack(fill=self.tk.BOTH, expand=True)

        self.tk.Label(intro_frame, text="Trajectory Collection",
                      font=("Arial", 24, "bold"), fg="#2196F3").pack(pady=20)

        instructions = (
            "\nCONTROL INSTRUCTIONS:\n\n"
            "Hold GRIP button      -> Start recording & move robot\n"
            f"Long press A/X ({LONG_PRESS_THRESHOLD}s)  -> SUCCESS: Save & Reset\n"
            f"Long press B/Y ({LONG_PRESS_THRESHOLD}s)  -> FAILURE: Discard & Reset\n"
            "Ctrl+C                -> Exit program\n\n"
            f"User: {self.user or 'superman'}\n"
            f"Scene: {self.scene or 'franka_irobman'}\n"
            f"Task: {self.task_name}\n"
            f"Save Images: {'Yes' if self.save_images else 'No'}\n"
            f"Save Depths: {'Yes' if self.save_depths else 'No'}\n\n"
            "Click 'Start Recording' when ready to begin.\n"
        )

        self.tk.Label(intro_frame, text=instructions, font=("Arial", 12),
                      justify=self.tk.LEFT).pack(pady=20)

        def start_recording():
            self.recording_active = True
            self.gui_ready = True
            self._show_camera_display()

        self.tk.Button(intro_frame, text="Start Recording", command=start_recording,
                       font=("Arial", 14, "bold"), bg="#4CAF50", fg="white",
                       padx=30, pady=10).pack(pady=20)

    # ---- camera display ----------------------------------------------------

    def _show_camera_display(self):
        for widget in self.root.winfo_children():
            widget.destroy()

        main_frame = self.tk.Frame(self.root)
        main_frame.pack(fill=self.tk.BOTH, expand=True, padx=10, pady=10)

        status_frame = self.tk.Frame(main_frame, bg="#2C3E50")
        status_frame.pack(fill=self.tk.X, pady=5)

        self.status_label = self.tk.Label(
            status_frame, text="Status: Waiting for recording to start...",
            font=("Arial", 12, "bold"), anchor="w", bg="#2C3E50", fg="white", padx=10, pady=5,
        )
        self.status_label.pack(side=self.tk.LEFT, fill=self.tk.X, expand=True)

        self.camera_frame = self.tk.Frame(main_frame, bg="black")
        self.camera_frame.pack(fill=self.tk.BOTH, expand=True)

        self.camera_labels = {}
        self.current_frame = None

        if self.read_cameras_func is not None:
            self._start_camera_reading_thread()
        self._update_camera_display()

    def _start_camera_reading_thread(self):
        if self._camera_read_thread is not None and self._camera_read_thread.is_alive():
            return
        self._camera_read_active = True
        self._camera_read_thread = threading.Thread(target=self._camera_read_loop, daemon=True)
        self._camera_read_thread.start()

    def _camera_read_loop(self):
        while self._camera_read_active and self.recording_active:
            try:
                if self.read_cameras_func is not None:
                    camera_obs, _ = self.read_cameras_func(use_sync=False)
                    if "image" in camera_obs:
                        with self.image_lock:
                            new_images = {
                                cid: img.copy()
                                for cid, img in camera_obs["image"].items()
                                if img is not None
                            }
                            if new_images:
                                self.image_dict.update(new_images)
                    time.sleep(0.016)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"Camera read thread error: {e}")
                time.sleep(0.1)

    # ---- status helpers ----------------------------------------------------

    def set_status(self, message: str):
        with self.status_lock:
            self.status_message = message

    def get_status(self) -> str:
        with self.status_lock:
            return self.status_message

    # ---- display helpers ---------------------------------------------------

    _CAMERA_NAME_MAP = {
        "11022812": "Hand Camera (ZED-M)",
        "24285872": "Static Camera (ZED 2)",
        "24285877": "Static Camera 2 (ZED 2)",
        "336222076118": "Hand Camera (RealSense)",
        "233522078330": "Static Camera (RealSense)",
    }

    def _get_camera_display_name(self, camera_id: str) -> str:
        name = self._CAMERA_NAME_MAP.get(camera_id)
        if name:
            return f"{name}\nID: {camera_id}"
        return f"Camera ID: {camera_id}"

    def _should_flip_camera(self, camera_id: str) -> bool:
        return self.flip_cameras.get(camera_id, False)

    def _update_camera_display(self):
        if not self.root:
            return
        try:
            with self.image_lock:
                images = self.image_dict.copy()

            status_text = f"Status: {self.get_status()}"
            if images:
                status_text += f" - {len(images)} camera(s) active [{', '.join(sorted(images.keys()))}]"
            if hasattr(self, "status_label"):
                self.status_label.config(text=status_text)

            if images:
                if self.current_frame is None:
                    self.current_frame = self.tk.Frame(self.camera_frame)
                    self.current_frame.pack(fill=self.tk.BOTH, expand=True)
                else:
                    try:
                        self.current_frame.winfo_exists()
                    except Exception:
                        self.current_frame = self.tk.Frame(self.camera_frame)
                        self.current_frame.pack(fill=self.tk.BOTH, expand=True)
                        self.camera_labels = {}

                sorted_ids = sorted(images.keys())
                cols = 2 if len(sorted_ids) > 1 else 1

                # Remove stale labels
                to_remove = []
                for cid in list(self.camera_labels.keys()):
                    if cid not in sorted_ids:
                        to_remove.append(cid)
                    else:
                        try:
                            lbl = self.camera_labels[cid]["img_label"]
                            lbl.winfo_exists()
                            _ = lbl.cget("bg")
                        except Exception:
                            to_remove.append(cid)
                for cid in to_remove:
                    self.camera_labels.pop(cid, None)

                for idx, camera_id in enumerate(sorted_ids):
                    img = images[camera_id]
                    if img is None:
                        continue
                    display_img = img.copy()
                    if self._should_flip_camera(camera_id):
                        display_img = cv2.flip(display_img, 1)
                    h, w = display_img.shape[:2]
                    max_w, max_h = 800, 600
                    if w > max_w or h > max_h:
                        scale = min(max_w / w, max_h / h)
                        display_img = cv2.resize(display_img, (int(w * scale), int(h * scale)))
                    try:
                        from PIL import Image, ImageTk
                        photo = ImageTk.PhotoImage(image=Image.fromarray(display_img))
                    except ImportError:
                        continue

                    exists = camera_id in self.camera_labels
                    valid = False
                    if exists:
                        try:
                            lbl = self.camera_labels[camera_id]["img_label"]
                            lbl.winfo_exists()
                            _ = lbl.cget("bg")
                            valid = True
                        except Exception:
                            self.camera_labels.pop(camera_id, None)
                            exists = False

                    if exists and valid:
                        try:
                            lbl = self.camera_labels[camera_id]["img_label"]
                            if not lbl.winfo_exists():
                                raise RuntimeError
                            lbl.config(image=photo)
                            lbl.image = photo
                        except Exception:
                            self.camera_labels.pop(camera_id, None)
                            exists = False
                            valid = False

                    if not exists or not valid:
                        try:
                            lf = self.tk.Frame(self.current_frame, bg="black")
                            lf.grid(row=idx // cols, column=idx % cols, padx=5, pady=5, sticky="nsew")
                            name_lbl = self.tk.Label(lf, text=self._get_camera_display_name(camera_id),
                                                     font=("Arial", 10, "bold"), bg="black", fg="white",
                                                     justify=self.tk.LEFT)
                            name_lbl.pack(pady=2)
                            if self._should_flip_camera(camera_id):
                                self.tk.Label(lf, text="[Flipped]", font=("Arial", 8),
                                              bg="black", fg="yellow").pack()
                            img_lbl = self.tk.Label(lf, image=photo, bg="black")
                            img_lbl.pack()
                            img_lbl.image = photo
                            self.camera_labels[camera_id] = {"frame": lf, "name_label": name_lbl, "img_label": img_lbl}
                        except Exception as e:
                            print(f"Failed to create camera label for {camera_id}: {e}")
                            continue
        except Exception as e:
            print(f"GUI update error: {e}")
            traceback.print_exc()

        if self.root:
            self.root.after(16, self._update_camera_display)

    # ---- public helpers ----------------------------------------------------

    def wait_for_ready(self):
        while not self.gui_ready:
            time.sleep(0.1)

    def get_task_config(self):
        return {
            "task_name": self.task_name,
            "user": self.user,
            "scene": self.scene,
            "save_images": self.save_images,
            "save_depths": self.save_depths,
        }

    def shutdown(self):
        self._camera_read_active = False
        if self._camera_read_thread is not None:
            self._camera_read_thread.join(timeout=1.0)
        if self.root:
            try:
                self.root.quit()
                self.root.destroy()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# CollectTrajectoryBase  (abstract base class)
# ---------------------------------------------------------------------------

class CollectTrajectoryBase:
    """
    Abstract base class for trajectory collection.

    Subclasses must implement:
        _create_robot(node)          -> BaseRobot | None
        _create_controller()         -> controller with .get_info() / .forward() / .reset_state()
        _get_movement_enabled(info)  -> bool
    """

    # --- abstract methods (must be overridden) ------------------------------

    def _create_robot(self, node):
        """
        Return a BaseRobot instance, or None to let RobotEnv use its default.
        """
        raise NotImplementedError

    def _create_controller(self):
        """
        Return a VR controller policy with get_info(), forward(obs), reset_state().
        """
        raise NotImplementedError

    def _get_movement_enabled(self, controller_info: dict) -> bool:
        """
        Extract a single *movement enabled* boolean from controller_info.
        """
        raise NotImplementedError

    # --- init ---------------------------------------------------------------

    def __init__(self, args):
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
        self.pos_vel_scale = args.pos_vel_scale
        self.rot_vel_scale = args.rot_vel_scale
        self.mirror = getattr(args, "mirror", False)
        self.horizon = None if args.horizon <= 0 else args.horizon
        self.enable_viz = args.viz

        # GUI ----------------------------------------------------------------
        self.gui = None
        self.image_dict: Dict = {}
        self.image_lock = threading.Lock()

        flip_camera_list = getattr(args, "flip_camera", [])
        self.flip_cameras = {cid: True for cid in flip_camera_list}

        if self.enable_viz:
            collector_ref = self

            def gui_shutdown_callback():
                print("\nGUI window closed - shutting down...")
                collector_ref._shutdown_requested = True
                collector_ref.shutdown()
                os.kill(os.getpid(), signal.SIGINT)

            self.gui = TrajectoryGUI(
                self.image_dict, self.image_lock,
                flip_cameras=self.flip_cameras,
                read_cameras_func=None,
                shutdown_callback=gui_shutdown_callback,
            )
            self.gui.wait_for_ready()
            gui_config = self.gui.get_task_config()
            if not self.task_name:
                self.task_name = gui_config["task_name"]
            if not self.user:
                self.user = gui_config.get("user", "superman")
            if not self.scene:
                self.scene = gui_config.get("scene", "franka_irobman")
            if not args.save_images:
                self.save_images = gui_config["save_images"]
            if not args.save_depths:
                self.save_depths = gui_config["save_depths"]

        if not self.user:
            self.user = "superman"
        if not self.scene:
            self.scene = "franka_irobman"

        if self.save_depths and not self.save_images:
            raise ValueError("--save-depths requires --save-images.")

        # Save folder structure -----------------------------------------------
        if self.task_name:
            self.save_trajectory = True
            self.success_logdir = os.path.join(self.save_folder, "success")
            self.failure_logdir = os.path.join(self.save_folder, "failure")
            os.makedirs(self.success_logdir, exist_ok=True)
            os.makedirs(self.failure_logdir, exist_ok=True)
        else:
            self.save_folder = ""
            self.task_name = None
            self.save_trajectory = False
            self.success_logdir = None
            self.failure_logdir = None

        # State variables -----------------------------------------------------
        self._num_steps = 0
        self._traj_count = 0
        self._shutdown_requested = False
        self._traj_writer: Optional[TrajectoryWriter] = None
        self._current_traj_filepath: Optional[str] = None
        self._current_traj_dir: Optional[str] = None
        self._current_time_str: Optional[str] = None
        self._recording_started = False
        self._success_button_press_start: Optional[float] = None
        self._failure_button_press_start: Optional[float] = None

        # ROS2 node -----------------------------------------------------------
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("collect_trajectory_node")

        # Print parameters
        self._print("=" * 70)
        self._print("Collect Trajectory - Starting Initialization")
        self._print("=" * 70)
        self._print(f"   Action space: {self.action_space}")
        self._print(f"   Control Hz:   {self.control_hz}")
        self._print(f"   Horizon:      {self.horizon if self.horizon else 'unlimited'}")
        self._print("-" * 70)

        # [1/4] RobotEnv
        self._print("[1/4] Initializing RobotEnv...")
        try:
            robot = self._create_robot(self._node)
            self.env = RobotEnv(
                action_space=self.action_space,
                do_reset=False,
                node=self._node,
                robot=robot,
            )
            self.env.control_hz = self.control_hz
            if self.enable_viz and self.gui:
                self.gui.read_cameras_func = self.env.read_cameras
            self._print("   RobotEnv initialized successfully")
        except Exception as e:
            self._print(f"   Failed to initialize RobotEnv: {e}")
            raise

        # [2/4] Controller
        self._print("[2/4] Initializing VR controller...")
        try:
            self.controller = self._create_controller()
            self._print(f"   Controller initialized: {type(self.controller).__name__}")
        except Exception as e:
            self._print(f"   Failed to initialize controller: {e}")
            raise

        # [3/4] Robot reset
        if self.reset_robot_on_start:
            self._print("[3/4] Resetting robot to home position...")
            try:
                self.env.reset(randomize=self.randomize_reset)
                self._print("   Robot reset completed")
            except Exception as e:
                self._print(f"   Robot reset failed: {e}")
        else:
            self._print("[3/4] Skipping robot reset")

        # [4/4] Trajectory recording
        self._print("[4/4] Setting up trajectory recording...")
        if self.save_trajectory and self.save_folder:
            os.makedirs(self.save_folder, exist_ok=True)
            self._print(f"   Mode: Recording trajectory")
            self._print(f"   Save folder: {self.save_folder}")
            self._print(f"   Task name:   {self.task_name}")
        else:
            self._print("   Mode: Teleoperation only (no saving)")

        self._start_new_trajectory()

        self._print("-" * 70)
        self._print("Initialization Complete!")
        self._print("=" * 70)
        self._print("")
        self._print("CONTROL INSTRUCTIONS:")
        self._print("   Hold GRIP button      -> Start recording & move robot")
        self._print(f"   Long press A/X ({LONG_PRESS_THRESHOLD}s)  -> SUCCESS: Save & Reset")
        self._print(f"   Long press B/Y ({LONG_PRESS_THRESHOLD}s)  -> FAILURE: Discard & Reset")
        self._print("   Ctrl+C               -> Exit program")
        self._print("")
        self._print("=" * 70)
        self._print("Ready! Hold GRIP button to start recording...")

    # --- helpers ------------------------------------------------------------

    def _print(self, msg: str):
        print(msg)

    def _start_new_trajectory(self):
        self._traj_count += 1
        self._num_steps = 0
        self._recording_started = False
        self._success_button_press_start = None
        self._failure_button_press_start = None
        self.controller.reset_state()

        if self.save_trajectory and self.save_folder:
            base = time.asctime().replace(" ", "_")
            traj_dir = os.path.join(self.failure_logdir, base)
            idx = 0
            while os.path.exists(traj_dir):
                idx += 1
                traj_dir = os.path.join(self.failure_logdir, f"{base}_{idx}")
            os.makedirs(traj_dir, exist_ok=True)
            time_str = os.path.basename(traj_dir)

            self._current_traj_filepath = os.path.join(traj_dir, "trajectory.h5")
            self._current_traj_dir = traj_dir
            self._current_time_str = time_str

            metadata = {
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
                save_depths=self.save_depths,
            )
            self._print(f"Recording trajectory #{self._traj_count}...")
        else:
            self._traj_writer = None
            self._current_traj_filepath = None
            self._current_traj_dir = None
            self._current_time_str = None
            self._print(f"Trajectory #{self._traj_count} (teleoperation only)")

    def _check_long_press(self, controller_info: dict) -> tuple:
        current_time = time.time()
        success_pressed = controller_info.get("success", False)
        failure_pressed = controller_info.get("failure", False)

        if success_pressed:
            if self._success_button_press_start is None:
                self._success_button_press_start = current_time
            is_success = (current_time - self._success_button_press_start) >= LONG_PRESS_THRESHOLD
        else:
            self._success_button_press_start = None
            is_success = False

        if failure_pressed:
            if self._failure_button_press_start is None:
                self._failure_button_press_start = current_time
            is_failure = (current_time - self._failure_button_press_start) >= LONG_PRESS_THRESHOLD
        else:
            self._failure_button_press_start = None
            is_failure = False

        return is_success, is_failure

    # --- control loop -------------------------------------------------------

    def _control_loop(self):
        if self._shutdown_requested:
            return

        try:
            controller_info = self.controller.get_info()
            movement_enabled = self._get_movement_enabled(controller_info)
            skip_action = self.wait_for_controller and (not movement_enabled)

            if not self._recording_started:
                if movement_enabled:
                    self._recording_started = True
                    self._success_button_press_start = None
                    self._failure_button_press_start = None
                    if self.enable_viz and self.gui:
                        self.gui.recording_active = True
                        self.gui.set_status("Recording started")
                        if self.gui.read_cameras_func is not None:
                            self.gui._start_camera_reading_thread()
                    self._print("")
                    self._print("=" * 70)
                    self._print("Recording started! Long press A/X -> SUCCESS, B/Y -> FAILURE")
                    self._print("=" * 70)
                    self._print("")
                else:
                    if self.enable_viz and self.gui:
                        self.gui.set_status("Waiting for GRIP button...")
                    time.sleep(0.05)
                    return

            is_success, is_failure = self._check_long_press(controller_info)
            if is_success:
                self._handle_trajectory_success()
                return
            elif is_failure:
                self._handle_trajectory_failure()
                return

            control_timestamps = {"step_start": get_ros_time_ns(self._node)}

            obs = self.env.get_observation(use_sync=True)
            obs["controller_info"] = controller_info
            if "timestamp" not in obs:
                obs["timestamp"] = {}
            obs["timestamp"]["skip_action"] = skip_action

            if self.enable_viz and self.gui:
                self.gui.set_status("Recording")

            control_timestamps["policy_start"] = get_ros_time_ns(self._node)
            action, controller_action_info = self.controller.forward(obs, include_info=True)
            control_timestamps["policy_end"] = get_ros_time_ns(self._node)

            control_timestamps["sleep_start"] = get_ros_time_ns(self._node)
            comp_time_s = (get_ros_time_ns(self._node) - control_timestamps["step_start"]) / 1e9
            sleep_left = (1 / self.env.control_hz) - comp_time_s
            if sleep_left > 0:
                time.sleep(sleep_left)

            control_timestamps["control_start"] = get_ros_time_ns(self._node)
            if skip_action:
                action_info = self.env.create_action_dict(np.zeros_like(action))
            else:
                action_info = self.env.step(action)
            action_info.update(controller_action_info)

            control_timestamps["step_end"] = get_ros_time_ns(self._node)
            obs["timestamp"]["control"] = control_timestamps
            timestep = {"observation": obs, "action": action_info}
            if self._traj_writer is not None:
                self._traj_writer.write_timestep(timestep)

            self._num_steps += 1
            if self.horizon is not None and self._num_steps >= self.horizon:
                self._print(f"Reached horizon ({self.horizon} steps)")
                self._handle_trajectory_success()
                return

        except Exception as e:
            self._print(f"Error in control loop: {e}\n{traceback.format_exc()}")

    # --- trajectory success / failure ---------------------------------------

    def _handle_trajectory_success(self):
        self._print("")
        self._print("=" * 70)
        self._print(f"Trajectory #{self._traj_count} SUCCESS  (steps: {self._num_steps})")

        if self._traj_writer is not None:
            self._traj_writer.close(metadata={"success": True, "failure": False})
            if self._current_traj_dir and os.path.exists(self._current_traj_dir):
                new_dir = os.path.join(self.success_logdir, self._current_time_str)
                try:
                    os.rename(self._current_traj_dir, new_dir)
                    self._print(f"Saved: {os.path.join(new_dir, 'trajectory.h5')}")
                except Exception as e:
                    self._print(f"Failed to move to success folder: {e}")
                    self._print(f"Saved: {self._current_traj_filepath}")
            else:
                self._print(f"Saved: {self._current_traj_filepath}")

        self._print("=" * 70)

        if self.enable_viz and self.gui:
            self.gui.set_status("Resetting robot...")
        self._print("Resetting robot to home position...")
        try:
            self.env.reset(randomize=self.randomize_reset)
            self._print("Robot reset completed")
        except Exception as e:
            self._print(f"Robot reset failed: {e}")

        time.sleep(0.5)
        if self.enable_viz and self.gui:
            self.gui.set_status("Starting new trajectory...")
        self._print("Starting new trajectory...")
        self._start_new_trajectory()
        if self.enable_viz and self.gui:
            self.gui.set_status("Hold GRIP button to start recording...")
        self._print("Hold GRIP button to start recording...")

    def _handle_trajectory_failure(self):
        self._print("")
        self._print("=" * 70)
        self._print(f"Trajectory #{self._traj_count} FAILURE  (steps: {self._num_steps})")

        if self._traj_writer is not None:
            try:
                self._traj_writer.close(metadata={"success": False, "failure": True})
                if (self._num_steps < MIN_FAILURE_STEPS
                        and self._current_traj_dir
                        and os.path.isdir(self._current_traj_dir)):
                    shutil.rmtree(self._current_traj_dir)
                    self._print(f"Discarded short failure ({self._num_steps} steps < {MIN_FAILURE_STEPS})")
                else:
                    self._print(f"Saved to failure folder: {self._current_traj_filepath}")
            except Exception as e:
                self._print(f"Failed to close trajectory writer: {e}")

        self._print("=" * 70)

        if self.enable_viz and self.gui:
            self.gui.set_status("Resetting robot...")
        self._print("Resetting robot...")
        try:
            self.env.reset(randomize=self.randomize_reset)
            self._print("Robot reset completed")
        except Exception as e:
            self._print(f"Robot reset failed: {e}")

        time.sleep(0.5)
        if self.enable_viz and self.gui:
            self.gui.set_status("Starting new trajectory...")
        self._print("Starting new trajectory...")
        self._start_new_trajectory()
        if self.enable_viz and self.gui:
            self.gui.set_status("Hold GRIP button to start recording...")
        self._print("Hold GRIP button to start recording...")

    # --- shutdown -----------------------------------------------------------

    def shutdown(self):
        self._shutdown_requested = True
        if self.gui:
            self.gui.shutdown()
        if self._traj_writer is not None:
            try:
                self._traj_writer.close(metadata={"interrupted": True})
                self._print(f"Saved interrupted trajectory: {self._current_traj_filepath}")
            except Exception as e:
                self._print(f"Error closing trajectory writer: {e}")
        if hasattr(self, "env"):
            self.env.shutdown()
        if hasattr(self, "_node"):
            self._node.destroy_node()


# ---------------------------------------------------------------------------
# Common argparse helpers
# ---------------------------------------------------------------------------

def add_common_args(parser: argparse.ArgumentParser):
    """Add arguments shared by all collect_trajectory scripts."""

    # Save settings
    parser.add_argument("--save-folder", type=str,
                        default="/app/ros2_ws/src/role-ros2/data",
                        help="Base folder to save trajectory files")
    parser.add_argument("--task", type=str, default="",
                        help="Task name (enables saving when provided)")
    parser.add_argument("--user", type=str, default="superman")
    parser.add_argument("--scene", type=str, default="franka_irobman")
    parser.add_argument("--save-images", action="store_true",
                        help="Save RGB images (MP4 video)")
    parser.add_argument("--save-depths", action="store_true",
                        help="Save depth images (requires --save-images)")

    # Control settings
    parser.add_argument("--action-space", type=str, default="cartesian_velocity",
                        choices=["cartesian_velocity", "cartesian_position",
                                 "joint_velocity", "joint_position"])
    parser.add_argument("--control-hz", type=float, default=15.0)
    parser.add_argument("--wait-for-controller", action="store_true", default=True)
    parser.add_argument("--no-wait-for-controller", dest="wait_for_controller",
                        action="store_false")

    # Velocity scale / mirror
    parser.add_argument("--pos-vel-scale", type=float, default=1.0,
                        help="Scale factor for position velocity (0.0-1.0)")
    parser.add_argument("--rot-vel-scale", type=float, default=1.0,
                        help="Scale factor for rotation velocity (0.0-1.0)")
    parser.add_argument("--mirror", action="store_true",
                        help="Mirror mode: mirror XY axes and swap arms (left joystick->right arm, right->left)")

    # Robot reset
    parser.add_argument("--reset-robot", action="store_true", default=True)
    parser.add_argument("--no-reset-robot", dest="reset_robot", action="store_false")
    parser.add_argument("--randomize-reset", action="store_true")

    # Trajectory
    parser.add_argument("--horizon", type=int, default=-1,
                        help="Max steps per trajectory (-1 = unlimited)")

    # Visualization
    parser.add_argument("--viz", action="store_true",
                        help="Enable GUI with camera display")
    parser.add_argument("--flip-camera", type=str, nargs="+", default=[],
                        help="Camera IDs to flip horizontally in GUI")


# ---------------------------------------------------------------------------
# run_collector  (main-loop helper)
# ---------------------------------------------------------------------------

def run_collector(collector_class, args):
    """
    Instantiate *collector_class(args)* and run the control loop until Ctrl+C.
    """
    collector = None

    def signal_handler(signum, frame):
        nonlocal collector
        print("\nShutdown signal received (Ctrl+C)")
        if collector is not None:
            collector.shutdown()
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        collector = collector_class(args)
        while rclpy.ok() and not collector._shutdown_requested:
            collector._control_loop()
    except KeyboardInterrupt:
        print("Exiting...")
    except Exception as e:
        print(f"Error: {e}\n{traceback.format_exc()}")
    finally:
        if collector is not None:
            collector.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass

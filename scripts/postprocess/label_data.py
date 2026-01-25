import argparse
import glob
import h5py
import json
import os
import tempfile
import numpy as np
import imageio.v3 as iio

# GUI Imports
import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk, ImageDraw, ImageFont

# ==========================================
# 1. Robust Video Decoder
# ==========================================
class H5VideoReader:
    def __init__(self, h5_path):
        self.h5_path = h5_path
        self.images = {}  # {cam_id: [frames...]}
        self.length = 0
        self.cam_ids = []
        self.error_msg = None  # To display on screen if missing
        self._load_data()

    def _load_data(self):
        try:
            with h5py.File(self.h5_path, 'r') as f:
                # 1. Determine Trajectory Length (from actions)
                if 'action/cartesian_position' in f:
                    self.length = f['action/cartesian_position'].shape[0]
                elif 'action/joint_position' in f:
                    self.length = f['action/joint_position'].shape[0]
                
                # 2. Try to find Image/Video Data
                # Priority 1: Encoded Videos in 'observations/videos' (Most likely based on your logs)
                if 'observations/videos' in f:
                    self._load_encoded_videos(f['observations/videos'])
                
                # Priority 2: Raw Images in 'observation/image' (Common in RLDS/DROID)
                elif 'observation/image' in f:
                    self._load_raw_images(f['observation/image'])
                
                # Priority 3: Raw Images in 'observations/images'
                elif 'observations/images' in f:
                    self._load_raw_images(f['observations/images'])
                
                else:
                    self.error_msg = "No 'videos' or 'image' group found in HDF5."
                    print(f"Warning: {self.error_msg} ({self.h5_path})")

        except Exception as e:
            self.error_msg = f"HDF5 Read Error: {str(e)}"
            print(f"Error loading {self.h5_path}: {e}")

    def _load_encoded_videos(self, group):
        """Decode uint8 byte arrays (mp4/avi) to frames"""
        self.cam_ids = sorted(list(group.keys()))
        for cam_id in self.cam_ids:
            try:
                video_bytes = group[cam_id][()]
                # Check if it's actually bytes
                if video_bytes.size == 0:
                    continue
                    
                frames = self._decode_video_bytes(video_bytes)
                if len(frames) > 0:
                    self.images[cam_id] = frames
                    # Update length if we didn't find actions, or clamp to video length
                    if self.length == 0: self.length = len(frames)
            except Exception as e:
                print(f"Failed to decode camera {cam_id}: {e}")
        
        if not self.images:
            self.error_msg = "Found video keys, but decoding failed (0 frames)."

    def _load_raw_images(self, group):
        """Load raw image arrays [T, H, W, C]"""
        self.cam_ids = sorted(list(group.keys()))
        for cam_id in self.cam_ids:
            try:
                # This might be a dataset of shape (T, H, W, C)
                data = group[cam_id]
                # Don't load everything to RAM if huge, but for now we assume it fits
                # Or use lazy loading. Here we convert to list of arrays for consistency.
                if len(data.shape) == 4: # T, H, W, C
                    self.images[cam_id] = data[:] # Load into memory
                    if self.length == 0: self.length = data.shape[0]
            except Exception as e:
                print(f"Failed to load raw image {cam_id}: {e}")

    def _decode_video_bytes(self, video_bytes):
        """Use ImageIO to decode bytes"""
        frames = []
        with tempfile.NamedTemporaryFile(suffix='.mp4', delete=False) as tmp_file:
            tmp_file.write(video_bytes.tobytes())
            tmp_path = tmp_file.name
        
        try:
            # Plugin 'pyav' is robust for ffmpeg decoding
            frames = iio.imread(tmp_path, plugin="pyav")
        except Exception:
            try:
                # Fallback to default plugin
                frames = iio.imread(tmp_path, index=None)
            except Exception as e:
                print(f"FFMPEG Decode Error: {e}")
                frames = []
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)
        return frames

    def get_concatenated_frame(self, t, max_height=300):
        # Case 1: Error or No Images
        if self.error_msg or not self.images:
            img = Image.new('RGB', (600, 400), color='black')
            draw = ImageDraw.Draw(img)
            text = self.error_msg if self.error_msg else "No Image Data"
            draw.text((20, 180), text, fill="white")
            return img

        # Case 2: Render Images
        frame_list = []
        # Clamp t
        valid_lens = [len(v) for v in self.images.values()]
        max_len = max(valid_lens) if valid_lens else 0
        t = max(0, min(t, max_len - 1))

        for cam_id in self.cam_ids:
            if cam_id not in self.images: continue
            
            frames = self.images[cam_id]
            # Handle if this specific camera has fewer frames than t
            local_t = min(t, len(frames) - 1)
            
            np_frame = frames[local_t]
            img = Image.fromarray(np_frame)
            
            # Resize
            aspect_ratio = img.width / img.height
            new_width = int(max_height * aspect_ratio)
            img = img.resize((new_width, max_height), Image.Resampling.LANCZOS)
            
            # Add Cam ID text
            draw = ImageDraw.Draw(img)
            draw.text((10, 10), cam_id[-4:], fill="yellow") # Show last 4 chars of ID
            
            frame_list.append(img)

        # Concatenate
        if not frame_list:
            return Image.new('RGB', (400, 300), color='gray')
            
        total_width = sum(img.width for img in frame_list)
        combined_img = Image.new('RGB', (total_width, max_height))
        
        current_x = 0
        for img in frame_list:
            combined_img.paste(img, (current_x, 0))
            current_x += img.width
            
        return combined_img

# ==========================================
# 2. Manager (Unchanged)
# ==========================================
class LabelManager:
    def __init__(self, data_folder):
        self.data_folder = os.path.abspath(data_folder)
        self.json_path = os.path.join(self.data_folder, "task_labels.json")
        self.trajectories = self._find_trajectories()
        self.labels = self._load_json_labels()

    def get_initial_index(self):
        """Last viewed index (from task_labels.json) for resume on next launch."""
        return self._last_index

    def save_last_index(self, idx):
        """Update last index and persist into task_labels.json."""
        self._last_index = idx
        self.save_labels()

    def _find_trajectories(self):
        traj_files = glob.glob(os.path.join(self.data_folder, "**", "trajectory.h5"), recursive=True)
        traj_files.sort()
        return traj_files

    def _load_json_labels(self):
        if not os.path.exists(self.json_path):
            self._last_index = 0
            return {}
        with open(self.json_path, 'r') as f:
            try:
                d = json.load(f)
            except json.JSONDecodeError:
                self._last_index = 0
                return {}
        try:
            self._last_index = int(d.pop("_last_index", 0))
        except (TypeError, ValueError):
            self._last_index = 0
        return d

    def save_labels(self):
        out = {**self.labels, "_last_index": self._last_index}
        with open(self.json_path, 'w') as f:
            json.dump(out, f, indent=4, sort_keys=True)
        print(f"Saved to {self.json_path}")

    def get_relative_path(self, abs_path):
        return os.path.relpath(abs_path, self.data_folder)

    def get_metadata(self, idx):
        path = self.trajectories[idx]
        rel_path = self.get_relative_path(path)
        
        meta = {"success": False, "failure": False, "task_name": "unknown"}
        
        if rel_path in self.labels:
            meta.update(self.labels[rel_path])
            return meta

        try:
            with h5py.File(path, 'r') as f:
                if 'success' in f.attrs: meta['success'] = bool(f.attrs['success'])
                if 'failure' in f.attrs: meta['failure'] = bool(f.attrs['failure'])
                if 'task_name' in f.attrs:
                    tn = f.attrs['task_name']
                    meta['task_name'] = tn.decode('utf-8') if isinstance(tn, bytes) else tn
        except:
            pass
        return meta

    def update_label(self, idx, success, failure, task_name):
        rel_path = self.get_relative_path(self.trajectories[idx])
        self.labels[rel_path] = {
            "success": bool(success),
            "failure": bool(failure),
            "task_name": str(task_name)
        }

# ==========================================
# 3. GUI App (Updated)
# ==========================================
class LabelApp:
    def __init__(self, root, manager):
        self.root = root
        self.manager = manager
        self.root.title("Robot Data Labeler")
        self.root.geometry("1100x750")

        self.current_idx = 0
        self.reader = None
        self.playing = False
        self.current_frame_idx = 0
        
        self._setup_ui()
        # Start from last viewed index (resume from previous session)
        start_idx = self.manager.get_initial_index()
        n = len(self.manager.trajectories)
        start_idx = max(0, min(start_idx, n - 1)) if n > 0 else 0
        self.load_trajectory(start_idx)

    def _setup_ui(self):
        # 1. Video Area
        self.video_frame = tk.Frame(self.root, bg="#222")
        self.video_frame.pack(fill=tk.BOTH, expand=True)
        
        self.img_label = tk.Label(self.video_frame, bg="#222")
        self.img_label.pack(expand=True, pady=10)

        # 2. Controls Area
        control_panel = tk.Frame(self.root, padx=10, pady=10)
        control_panel.pack(fill=tk.X)

        # Slider + Step Counter
        slider_frame = tk.Frame(control_panel)
        slider_frame.pack(fill=tk.X, pady=5)
        
        self.slider = tk.Scale(slider_frame, from_=0, to=100, orient=tk.HORIZONTAL, 
                             showvalue=0, command=self.on_slider_move)
        self.slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # [NEW] Step Counter Label
        self.lbl_step = tk.Label(slider_frame, text="Step: 0 / 0", width=15, font=("Mono", 10))
        self.lbl_step.pack(side=tk.RIGHT, padx=5)

        # Play Buttons
        btn_frame = tk.Frame(control_panel)
        btn_frame.pack(fill=tk.X, pady=5)
        tk.Button(btn_frame, text="Replay", command=self.replay_video).pack(side=tk.LEFT, padx=2)
        self.btn_play = tk.Button(btn_frame, text="Play", command=self.toggle_play, width=8)
        self.btn_play.pack(side=tk.LEFT, padx=2)

        # 3. Labeling Area
        label_frame = tk.LabelFrame(self.root, text="Annotation", padx=10, pady=10)
        label_frame.pack(fill=tk.X, padx=10, pady=10)

        # Info
        self.lbl_info = tk.Label(label_frame, text="File: ...", anchor="w")
        self.lbl_info.pack(fill=tk.X)

        # Save last index on window close (resume on next launch)
        def _on_closing():
            self.manager.save_last_index(self.current_idx)
            self.root.destroy()
        self.root.protocol("WM_DELETE_WINDOW", _on_closing)

        # Task Name
        input_row = tk.Frame(label_frame)
        input_row.pack(fill=tk.X, pady=5)
        tk.Label(input_row, text="Task Name: ").pack(side=tk.LEFT)
        self.entry_task = tk.Entry(input_row)
        self.entry_task.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # [UPDATED] Outcome - Removed Neutral
        self.status_var = tk.StringVar(value="none")
        radio_row = tk.Frame(label_frame)
        radio_row.pack(fill=tk.X, pady=5)
        tk.Label(radio_row, text="Outcome: ").pack(side=tk.LEFT)
        
        # Only Success and Failure
        tk.Radiobutton(radio_row, text="Success", variable=self.status_var, value="success").pack(side=tk.LEFT, padx=10)
        tk.Radiobutton(radio_row, text="Failure", variable=self.status_var, value="failure").pack(side=tk.LEFT, padx=10)

        # Navigation
        nav_row = tk.Frame(self.root, pady=10)
        nav_row.pack(fill=tk.X)
        tk.Button(nav_row, text="<< Save & Prev", command=self.prev_traj, height=2).pack(side=tk.LEFT, padx=20)
        tk.Button(nav_row, text="Save & Next >>", command=self.next_traj, height=2).pack(side=tk.RIGHT, padx=20)
        self.lbl_progress = tk.Label(nav_row, text="1 / 10")
        self.lbl_progress.pack(side=tk.BOTTOM)

    def load_trajectory(self, idx):
        if idx < 0 or idx >= len(self.manager.trajectories): return
        
        self.playing = False
        self.btn_play.config(text="Play")
        self.current_idx = idx
        path = self.manager.trajectories[idx]
        
        # Load Data
        self.root.title(f"Loading... {os.path.basename(path)}")
        self.root.update()
        self.reader = H5VideoReader(path)
        
        # Setup Timeline
        total_steps = self.reader.length
        self.slider.config(to=max(0, total_steps - 1))
        self.slider.set(0)
        self.current_frame_idx = 0
        
        # Load Metadata
        meta = self.manager.get_metadata(idx)
        
        # Update UI
        self.lbl_info.config(text=f"Path: {self.manager.get_relative_path(path)}")
        self.lbl_progress.config(text=f"{idx + 1} / {len(self.manager.trajectories)}")
        self.lbl_step.config(text=f"Step: 0 / {total_steps}")
        
        self.entry_task.delete(0, tk.END)
        self.entry_task.insert(0, meta['task_name'])
        
        # Set Radio
        if meta['success']: self.status_var.set("success")
        elif meta['failure']: self.status_var.set("failure")
        else: self.status_var.set("none") # No selection implies neutral/unlabeled
        
        self.root.title(f"Labeler - {os.path.basename(path)}")
        self.update_display(0)

    def update_display(self, t):
        if not self.reader: return
        # Get image
        pil_img = self.reader.get_concatenated_frame(t, max_height=400)
        self.tk_img = ImageTk.PhotoImage(pil_img)
        self.img_label.config(image=self.tk_img)

    def on_slider_move(self, val):
        t = int(val)
        self.current_frame_idx = t
        if self.reader:
            self.lbl_step.config(text=f"Step: {t} / {self.reader.length}")
        self.update_display(t)

    def toggle_play(self):
        if self.playing:
            self.playing = False
            self.btn_play.config(text="Play")
        else:
            self.playing = True
            self.btn_play.config(text="Pause")
            self.play_loop()

    def replay_video(self):
        self.playing = True
        self.slider.set(0)
        self.btn_play.config(text="Pause")
        self.play_loop()

    def play_loop(self):
        if self.playing and self.reader:
            if self.current_frame_idx < self.reader.length - 1:
                self.current_frame_idx += 1
                self.slider.set(self.current_frame_idx)
                self.root.after(33, self.play_loop)
            else:
                self.playing = False
                self.btn_play.config(text="Replay")

    def save_current(self):
        task = self.entry_task.get()
        status = self.status_var.get()
        self.manager.update_label(self.current_idx, status=="success", status=="failure", task)
        self.manager.save_labels()

    def next_traj(self):
        self.save_current()
        if self.current_idx < len(self.manager.trajectories) - 1:
            self.load_trajectory(self.current_idx + 1)
            self.manager.save_last_index(self.current_idx)
        else:
            messagebox.showinfo("Info", "End of dataset.")

    def prev_traj(self):
        self.save_current()
        if self.current_idx > 0:
            self.load_trajectory(self.current_idx - 1)
            self.manager.save_last_index(self.current_idx)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('data_folder', type=str)
    args = parser.parse_args()

    if not os.path.exists(args.data_folder):
        print("Data folder not found")
        exit(1)

    manager = LabelManager(args.data_folder)
    if not manager.trajectories:
        print("No .h5 files found in folder")
        exit(1)

    root = tk.Tk()
    app = LabelApp(root, manager)
    root.mainloop()
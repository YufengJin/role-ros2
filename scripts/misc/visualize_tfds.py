#!/usr/bin/env python3
"""
visualize_tfds.py - Visualize TFDS/RLDS examples with matplotlib.

Author: Chaser Robotics Team

- Row 0: 3 camera images (exterior_image_1_left, exterior_image_2_left, wrist_image_left)
- Row 1: Joint position (7), gripper position, action (7) time series up to current step
- Sliders: Episode, Step. Optional: --max-episodes to cap cached episodes.

Usage:
  python scripts/misc/visualize_tfds.py --tfrecord-dir /path/to/tfrecord/role_ros2/1.0.0
  python scripts/misc/visualize_tfds.py --tfrecord-dir .../1.0.0 --save out.png  # Use --save when headless (no DISPLAY)

Headless: Without DISPLAY, script uses Agg and requires --save; otherwise Qt/xcb may crash.
Reduce TF log spam: TF_CPP_MIN_LOG_LEVEL=2 or 3.
Why trajectory_visualizer works headless: it uses h5py/imageio/matplotlib only; tfds pulls in cv2 (Qt/xcb), which
crashes without DISPLAY. This script uses QT_QPA_PLATFORM=offscreen and TkAgg to avoid that.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# Must set before importing tfds/matplotlib to reduce TF log spam and avoid cv2 Qt/xcb crash when no DISPLAY
os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")
_linux_no_display = sys.platform == "linux" and not os.environ.get("DISPLAY")

# tfds pulls in cv2 (Qt/xcb). Use offscreen to avoid xcb crash when no DISPLAY.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# Backend: Agg when headless or --save; TkAgg otherwise to avoid cv2 Qt conflict
if "--save" in sys.argv or _linux_no_display:
    import matplotlib
    matplotlib.use("Agg")
else:
    import matplotlib
    try:
        matplotlib.use("TkAgg")  # Tk and cv2 Qt do not conflict, window can open
    except Exception:
        pass  # Fallback if Tk unavailable

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

try:
    import tensorflow_datasets as tfds
except ImportError:
    tfds = None

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TFRECORD_DIR = REPO_ROOT / "data" / "tfrecord"


def _resolve_tfds_builder_dir(tfrecord_dir: Path) -> Path | None:
    """Resolve TFDS builder dir: must contain dataset_info.json."""
    p = tfrecord_dir.resolve()
    if (p / "dataset_info.json").exists():
        return p
    for v in ["1.0.0", "2.0.0"]:
        cand = p / v
        if (cand / "dataset_info.json").exists():
            return cand
    return None


def _to_numpy(x):
    if hasattr(x, "numpy"):
        return x.numpy()
    return np.asarray(x)


def _decode_bytes(b):
    if b is None:
        return ""
    if hasattr(b, "numpy"):
        b = b.numpy()
    if isinstance(b, (bytes, np.bytes_)):
        return b.decode("utf-8", errors="replace") if len(b) > 0 else ""
    return str(b)


class TFDSVisualizer:
    """Interactive visualizer for TFDS RLDS episodes."""

    IMG_KEYS = ["exterior_image_1_left", "exterior_image_2_left", "wrist_image_left"]

    def __init__(self, builder_dir: Path, split: str = "train", max_episodes: int = 10):
        if tfds is None:
            raise SystemExit("tensorflow_datasets is required. pip install tensorflow-datasets")

        self.builder_dir = Path(builder_dir).resolve()
        self.split = split
        self.max_episodes = max_episodes

        builder = tfds.builder_from_directory(str(self.builder_dir))
        ds = builder.as_dataset(split=split, shuffle_files=False)

        self.episodes = []
        for i, ep in enumerate(ds):
            if i >= max_episodes:
                break
            steps = list(ep["steps"])
            meta = dict(ep.get("episode_metadata") or {})
            self.episodes.append({"steps": steps, "episode_metadata": meta})

        if not self.episodes:
            raise ValueError(f"No episodes found in {builder_dir} split={split}")

        self.ep_idx = 0
        self.step_idx = 0
        self._setup_figure()

    def _get_episode(self):
        return self.episodes[self.ep_idx]

    def _get_steps(self):
        return self._get_episode()["steps"]

    def _setup_figure(self):
        self.fig = plt.figure(figsize=(14, 10))
        gs = self.fig.add_gridspec(2, 4, hspace=0.35, wspace=0.3)

        self.axes = {}
        for i, k in enumerate(self.IMG_KEYS):
            self.axes[f"img_{i}"] = self.fig.add_subplot(gs[0, i])
        self.axes["info"] = self.fig.add_subplot(gs[0, 3])
        self.axes["joint"] = self.fig.add_subplot(gs[1, 0])
        self.axes["gripper"] = self.fig.add_subplot(gs[1, 1])
        self.axes["action"] = self.fig.add_subplot(gs[1, 2])
        self.axes["legend_txt"] = self.fig.add_subplot(gs[1, 3])

        ax_ep = plt.axes([0.15, 0.02, 0.25, 0.025])
        ax_st = plt.axes([0.5, 0.02, 0.25, 0.025])
        self.slider_ep = Slider(ax_ep, "Episode", 0, max(0, len(self.episodes) - 1), valinit=0, valstep=1)
        # Step slider: valmax 2000 to support long episodes; we clamp in _on_step
        self.slider_st = Slider(ax_st, "Step", 0, 2000, valinit=0, valstep=1)
        self.slider_ep.on_changed(self._on_episode)
        self.slider_st.on_changed(self._on_step)

        self._update()

    def _on_episode(self, val):
        self.ep_idx = int(val)
        steps = self._get_steps()
        n = len(steps)
        self.step_idx = min(self.step_idx, max(0, n - 1))
        self.slider_st.set_val(self.step_idx)
        self._update()

    def _on_step(self, val):
        n = len(self._get_steps())
        self.step_idx = min(int(val), max(0, n - 1))
        self._update()

    def _update(self):
        steps = self._get_steps()
        if not steps:
            return
        t = min(self.step_idx, len(steps) - 1)
        step = steps[t]
        obs = step.get("observation") or {}

        for i, k in enumerate(self.IMG_KEYS):
            ax = self.axes[f"img_{i}"]
            ax.clear()
            img = obs.get(k)
            if img is not None:
                arr = _to_numpy(img)
                if arr.ndim == 3:
                    ax.imshow(arr)
                else:
                    ax.text(0.5, 0.5, "N/A", ha="center", va="center", transform=ax.transAxes)
            else:
                ax.text(0.5, 0.5, "No image", ha="center", va="center", transform=ax.transAxes)
            ax.set_title(k.replace("_", " ").title(), fontsize=9)
            ax.axis("off")

        ax = self.axes["info"]
        ax.clear()
        ax.axis("off")
        lang = _decode_bytes(step.get("language_instruction", b""))
        meta = self._get_episode().get("episode_metadata") or {}
        fp = _decode_bytes(meta.get("file_path", b""))
        lines = [
            f"Episode {self.ep_idx + 1}/{len(self.episodes)}",
            f"Step {t + 1}/{len(steps)}",
            "",
            "instruction: " + (lang[:80] + "…" if len(lang) > 80 else lang or "(none)"),
            "",
            "file: " + (Path(fp).name if fp else "—"),
        ]
        ax.text(0.02, 0.98, "\n".join(lines), transform=ax.transAxes, fontsize=8,
                verticalalignment="top", fontfamily="monospace",
                bbox=dict(boxstyle="round", facecolor="#f8f9fa", alpha=0.9))

        r = t + 1
        jp = []
        gp = []
        ac = []
        for i in range(r):
            s = steps[i]
            o = s.get("observation") or {}
            x = o.get("joint_position")
            jp.append(_to_numpy(x).flatten() if x is not None else np.full(7, np.nan))
            x = o.get("gripper_position")
            gp.append(_to_numpy(x).flatten()[0] if x is not None and np.size(x) > 0 else np.nan)
            x = s.get("action")
            ac.append(_to_numpy(x).flatten() if x is not None else np.full(7, np.nan))

        jp = np.array(jp)
        gp = np.array(gp)
        ac = np.array(ac)

        ax = self.axes["joint"]
        ax.clear()
        if len(jp) > 0 and jp.size > 0:
            n = jp.shape[1] if jp.ndim > 1 else 1
            for j in range(min(7, n)):
                ax.plot(range(len(jp)), jp[:, j] if jp.ndim > 1 else jp, label=f"J{j+1}", lw=1)
            ax.legend(loc="upper right", fontsize=6, ncol=2)
        ax.set_title("Observation: joint_position", fontsize=9)
        ax.set_xlabel("Step")
        ax.grid(True, alpha=0.3)

        ax = self.axes["gripper"]
        ax.clear()
        if len(gp) > 0:
            ax.plot(range(len(gp)), gp, "b-", lw=2)
        ax.set_title("Observation: gripper_position", fontsize=9)
        ax.set_xlabel("Step")
        ax.grid(True, alpha=0.3)

        ax = self.axes["action"]
        ax.clear()
        if len(ac) > 0 and ac.size > 0:
            n = ac.shape[1] if ac.ndim > 1 else 1
            for j in range(min(7, n)):
                ax.plot(range(len(ac)), ac[:, j] if ac.ndim > 1 else ac, label=f"A{j+1}", lw=1)
            ax.legend(loc="upper right", fontsize=6, ncol=2)
        ax.set_title("Action (7)", fontsize=9)
        ax.set_xlabel("Step")
        ax.grid(True, alpha=0.3)

        ax = self.axes["legend_txt"]
        ax.clear()
        ax.axis("off")
        ax.text(0.5, 0.5, "Action: cartesian(6)\n+ gripper(1)", ha="center", va="center",
                transform=ax.transAxes, fontsize=9)

        self.fig.suptitle(
            f"TFDS: {self.builder_dir.name}  |  Episode {self.ep_idx + 1}  Step {t + 1}/{len(steps)}",
            fontsize=12, fontweight="bold"
        )
        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


def main():
    ap = argparse.ArgumentParser(
        description="Visualize TFDS/RLDS examples (to_tfrecord, DROID).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Layout:
  Row 0: exterior_image_1_left, exterior_image_2_left, wrist_image_left, info (instruction, file)
  Row 1: joint_position (7) ts, gripper ts, action (7) ts

Examples:
  python scripts/misc/visualize_tfds.py --tfrecord-dir data/tfrecord/role_ros2/1.0.0
  python scripts/misc/visualize_tfds.py --tfrecord-dir /app/datasets/droid/droid_100/1.0.0 --max-episodes 5
"""
    )
    ap.add_argument(
        "--tfrecord-dir",
        type=Path,
        default=DEFAULT_TFRECORD_DIR,
        help=f"TFDS versioned dir (dataset_info.json, *-train.tfrecord-*). Default: {DEFAULT_TFRECORD_DIR}",
    )
    ap.add_argument("--split", choices=("train", "test"), default="train")
    ap.add_argument("--max-episodes", type=int, default=10, help="Max episodes to cache (default 10)")
    ap.add_argument("--save", type=Path, default=None, help="Save figure to file and exit (e.g. --save out.png) instead of plt.show()")
    args = ap.parse_args()

    if _linux_no_display and args.save is None:
        print("No DISPLAY (headless). Use --save to export a PNG instead of interactive window:")
        print("  python scripts/misc/visualize_tfds.py --tfrecord-dir <path> --save /tmp/out.png")
        sys.exit(1)

    builder_dir = _resolve_tfds_builder_dir(args.tfrecord_dir)
    if builder_dir is None:
        print(f"Not a TFDS dataset dir (no dataset_info.json): {args.tfrecord_dir}")
        print("Use the versioned dir, e.g. .../tfrecord/role_ros2/1.0.0 or .../droid_100/1.0.0")
        return

    vis = TFDSVisualizer(builder_dir, split=args.split, max_episodes=args.max_episodes)
    if args.save is not None:
        vis.fig.savefig(args.save, dpi=120, bbox_inches="tight")
        print(f"Saved to {args.save}")
    else:
        vis.show()


if __name__ == "__main__":
    main()

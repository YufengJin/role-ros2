#!/usr/bin/env python3
"""
tfds_loader.py - Inspect TFDS datasets and provide get_tfds_dataloader.

Author: Chaser Robotics Team

Inspect TFDS datasets (to_tfrecord.py output, DROID, etc.).
Layout: TFDS versioned dir with dataset_info.json, features.json,
{DATASET}-{SPLIT}.tfrecord-{X}-of-{Y}. Uses tfds.builder_from_directory + as_dataset.

Usage:
  python scripts/misc/tfds_loader.py --tfrecord-dir /path/to/tfrecord/role_ros2/1.0.0
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

try:
    import tensorflow as tf
except ImportError:
    tf = None

try:
    import tensorflow_datasets as tfds
except ImportError:
    tfds = None

# scripts/tests/test_tf_load.py -> parents[0]=tests, parents[1]=scripts, parents[2]=repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TFRECORD_DIR = REPO_ROOT / "data" / "tfrecord"


def _describe_tf_value(v, indent: str, prefix: str) -> None:
    """Recursively describe a TF/numpy value (dict, Tensor, scalar). Always prints shape and dtype."""
    if hasattr(v, "numpy"):
        v = v.numpy()
    if isinstance(v, dict):
        for k in sorted(v.keys()):
            _describe_tf_value(v[k], indent + "  ", f"{prefix}{k}/")
    elif isinstance(v, np.ndarray):
        extra = ""
        if np.issubdtype(v.dtype, np.floating) and v.size > 0:
            extra = f"  min={float(v.min()):.4f} max={float(v.max()):.4f}"
        elif np.issubdtype(v.dtype, np.integer) and v.size > 0:
            extra = f"  min={int(v.min())} max={int(v.max())}"
        print(f"{indent}{prefix.rstrip('/')}: shape={v.shape} dtype={v.dtype}{extra}")
    elif isinstance(v, (bytes, np.bytes_)):
        try:
            s = v.decode("utf-8", errors="replace") if isinstance(v, bytes) else (v.tobytes().decode("utf-8", errors="replace") if hasattr(v, "tobytes") else str(v))
        except Exception:
            s = repr(v)[:80]
        n = len(v) if isinstance(v, bytes) else getattr(v, "nbytes", "?")
        print(f"{indent}{prefix.rstrip('/')}: shape=() dtype=bytes (len={n}) = {s!r}")
    elif isinstance(v, (np.generic, np.integer, np.floating, np.bool_)):
        print(f"{indent}{prefix.rstrip('/')}: shape=() dtype={v.dtype} = {v}")
    else:
        t = type(v).__name__
        d = getattr(v, "dtype", t)
        print(f"{indent}{prefix.rstrip('/')}: shape=() dtype={d} = {v}")


def _resolve_tfds_builder_dir(tfrecord_dir: Path) -> Path | None:
    """Resolve TFDS builder dir: must contain dataset_info.json (and usually features.json)."""
    p = tfrecord_dir.resolve()
    if (p / "dataset_info.json").exists():
        return p
    for v in ["1.0.0", "2.0.0"]:
        cand = p / v
        if (cand / "dataset_info.json").exists():
            return cand
    return None


def get_tfds_dataloader(
    builder_dir: str | Path,
    split: str = "train",
    shuffle_files: bool = True,
    batch_size: int | None = None,
    **as_dataset_kwargs,
):
    """
    TFDS dataloader: tf.data.Dataset of RLDS episodes from a standard TFDS directory.

    Args:
        builder_dir: Path to TFDS versioned dir (dataset_info.json, features.json,
            *-train.tfrecord-*). E.g. <output-dir>/role_ros2/1.0.0 or .../droid_100/1.0.0.
        split: 'train' or 'test'.
        shuffle_files: Shuffle shards when reading.
        batch_size: If set, batch episodes; otherwise unbatched.
        **as_dataset_kwargs: Passed to builder.as_dataset() (e.g. decoders, read_config).

    Returns:
        tf.data.Dataset of episodes: { steps: Dataset, episode_metadata: {...} }.
        Each step: observation, action_dict, action, discount, reward, is_first, is_last,
        is_terminal, language_instruction.

    Example:
        ds = get_tfds_dataloader("/path/to/role_ros2/1.0.0", split="train", batch_size=4)
        for batch in ds.take(10):
            ...
    """
    if tfds is None:
        raise SystemExit("tensorflow_datasets is required. pip install tensorflow-datasets")
    builder = tfds.builder_from_directory(str(builder_dir))
    ds = builder.as_dataset(split=split, shuffle_files=shuffle_files, **as_dataset_kwargs)
    if batch_size is not None:
        ds = ds.batch(batch_size)
    return ds


def run_tfds(
    tfrecord_dir: Path,
    split: str = "train",
    max_examples: int = 2,
    max_steps_per_episode: int = 2,
) -> None:
    if tf is None:
        raise SystemExit("tensorflow is required. pip install tensorflow")
    if tfds is None:
        raise SystemExit("tensorflow_datasets is required. pip install tensorflow-datasets")

    builder_dir = _resolve_tfds_builder_dir(tfrecord_dir)
    if builder_dir is None:
        print(f"Not a TFDS dataset dir (no dataset_info.json): {tfrecord_dir}")
        print("Use the versioned dir, e.g. .../tfrecord/role_ros2/1.0.0 or .../droid_100/1.0.0")
        return

    builder = tfds.builder_from_directory(str(builder_dir))
    ds = builder.as_dataset(split=split, shuffle_files=True)

    print(f"[tfds] Builder dir: {builder_dir}")
    print(f"Split: {split}, dataset: {getattr(builder.info, 'name', '?')}")
    print("-" * 60)

    total = 0
    for ep in ds:
        if total >= max_examples:
            break
        print(f"\n--- Episode {total + 1} ---")
        if "episode_metadata" in ep:
            print("  episode_metadata:")
            _describe_tf_value(ep["episode_metadata"], "    ", "")

        step_count = 0
        for step in ep["steps"]:
            if step_count >= max_steps_per_episode:
                print(f"  ... ({max_steps_per_episode} steps shown)")
                break
            print(f"  --- step {step_count + 1} ---")
            if "observation" in step:
                print("    observation:")
                _describe_tf_value(step["observation"], "      ", "")
            if "action_dict" in step:
                print("    action_dict:")
                _describe_tf_value(step["action_dict"], "      ", "")
            if "action" in step:
                _describe_tf_value(step["action"], "    ", "action")
            for k in [
                "reward", "discount", "is_first", "is_last", "is_terminal",
                "language_instruction", "language_instruction_2", "language_instruction_3",
            ]:
                if k in step:
                    _describe_tf_value(step[k], "    ", k)
            step_count += 1

        total += 1

    print(f"\n--- Described {total} episode(s) ---")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Inspect TFDS dataset (to_tfrecord/DROID). get_tfds_dataloader() for training."
    )
    ap.add_argument(
        "--tfrecord-dir",
        type=Path,
        default=DEFAULT_TFRECORD_DIR,
        help=f"TFDS versioned dir (dataset_info.json, *-train.tfrecord-*). Default: {DEFAULT_TFRECORD_DIR}",
    )
    ap.add_argument("--split", choices=("train", "test"), default="train")
    ap.add_argument("--max-examples", type=int, default=2, help="Max episodes to describe")
    ap.add_argument(
        "--max-steps-per-episode",
        type=int,
        default=2,
        help="Max steps to show per episode",
    )
    args = ap.parse_args()

    run_tfds(
        tfrecord_dir=args.tfrecord_dir.resolve(),
        split=args.split,
        max_examples=args.max_examples,
        max_steps_per_episode=args.max_steps_per_episode,
    )


if __name__ == "__main__":
    main()

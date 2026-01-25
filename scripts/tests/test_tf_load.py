#!/usr/bin/env python3
"""
test_tf_load.py

Load TFRecord shards produced by droid_to_tfrecord.py and print data format:
  - Feature keys and value kinds (tensor vs bytes_list of JPEGs)
  - Shapes and dtypes for tensors
  - Sample min/max for float tensors, frame count for videos

Usage:
  python tests/test_tf_load.py
  python tests/test_tf_load.py --tfrecord-dir /path/to/tfrecords --split train --max-examples 3
"""

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np

try:
    import tensorflow as tf
except ImportError:
    tf = None

# scripts/tests/test_tf_load.py -> parents[0]=tests, parents[1]=scripts, parents[2]=repo root
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TFRECORD_DIR = REPO_ROOT / "data" / "tfrecord"


def _parse_tensor_feature(raw: bytes) -> tf.Tensor:
    return tf.io.parse_tensor(raw, out_type=tf.float32)


def parse_example(serialized: bytes) -> dict:
    """
    Parse one tf.train.Example from droid_to_tfrecord.py output.

    Returns:
        dict: keys = feature names. Values are either np.ndarray (from
        serialized tensors) or list of bytes (JPEG frames for observations/videos/*).
    """
    ex = tf.train.Example()
    ex.MergeFromString(serialized)
    out = {}
    for name, feat in ex.features.feature.items():
        if feat.HasField("bytes_list") and feat.bytes_list.value:
            vals = list(feat.bytes_list.value)
            if name.startswith("observations/videos/"):
                out[name] = vals  # list of JPEG bytes
            else:
                # Tensor: bytes_list has one element (serialized tensor)
                t = _parse_tensor_feature(vals[0])
                out[name] = np.array(t)
        elif feat.HasField("float_list") and feat.float_list.value:
            out[name] = np.array(feat.float_list.value, dtype=np.float32)
        elif feat.HasField("int64_list") and feat.int64_list.value:
            out[name] = np.array(feat.int64_list.value, dtype=np.int64)
    return out


def describe_example(parsed: dict, indent: str = "  ") -> None:
    """Print shape, dtype, and basic stats for each feature."""
    for k in sorted(parsed.keys()):
        v = parsed[k]
        if isinstance(v, np.ndarray):
            shp = f"shape={v.shape}, dtype={v.dtype}"
            if np.issubdtype(v.dtype, np.floating):
                shp += f", min={v.min():.4f}, max={v.max():.4f}"
            elif np.issubdtype(v.dtype, np.integer):
                shp += f", min={v.min()}, max={v.max()}"
            print(f"{indent}{k}: {shp}")
        elif isinstance(v, list) and v and isinstance(v[0], bytes):
            print(f"{indent}{k}: list of {len(v)} JPEG frames (bytes, len[0]={len(v[0])})")
        else:
            print(f"{indent}{k}: {type(v).__name__} len={len(v) if hasattr(v, '__len__') else '?'}")


def run(
    tfrecord_dir: Path,
    split: str = "train",
    max_examples: int = 2,
    max_shards: int = 2,
) -> None:
    if tf is None:
        raise SystemExit("tensorflow is required. pip install tensorflow")

    dir_path = tfrecord_dir / split
    if not dir_path.is_dir():
        print(f"Directory not found: {dir_path}")
        return

    shards = sorted(glob.glob(str(dir_path / "*.tfrecord")))[:max_shards]
    if not shards:
        print(f"No .tfrecord files in {dir_path}")
        return

    print(f"TFRecord dir: {tfrecord_dir}")
    print(f"Split: {split}, shards: {shards}")
    print("-" * 60)

    total = 0
    for shard_path in shards:
        ds = tf.data.TFRecordDataset(shard_path)
        for raw in ds:
            if total >= max_examples:
                break
            parsed = parse_example(raw.numpy())
            print(f"\n--- Example {total + 1} (from {Path(shard_path).name}) ---")
            describe_example(parsed)
            total += 1
        if total >= max_examples:
            break

    print(f"\n--- Described {total} example(s) ---")


def main() -> None:
    ap = argparse.ArgumentParser(description="Inspect TFRecord format from droid_to_tfrecord.py")
    ap.add_argument(
        "--tfrecord-dir",
        type=Path,
        default=DEFAULT_TFRECORD_DIR,
        help=f"TFRecord root (train/, test/). Default: {DEFAULT_TFRECORD_DIR}",
    )
    ap.add_argument("--split", choices=("train", "test"), default="train")
    ap.add_argument("--max-examples", type=int, default=2, help="Max examples to describe")
    ap.add_argument("--max-shards", type=int, default=2, help="Max shard files to open")
    args = ap.parse_args()

    run(
        tfrecord_dir=args.tfrecord_dir.resolve(),
        split=args.split,
        max_examples=args.max_examples,
        max_shards=args.max_shards,
    )


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
droid_to_tfrecord.py

Convert role-ros2 .h5 trajectories in data/ to TFRecord dataset.

- Input: --data-dir. task_labels.json is read from <data-dir>/task_labels.json only.
- If task_labels.json does NOT exist: use success/**/trajectory.h5, print warning
  "data not relabeled", and require user to type 'yes' to continue.
- If task_labels.json exists: use all h5 with success=True in labels. If some
  success/ trajectories are not in task_labels (not relabeled), warn and ask:
  continue (include them) or skip (use only relabeled success).

Output TFRecord fields (ACTION_KEYS, OBS_KEYS) and camera mapping
(DEFAULT_CAMERA_ID_TO_NAME) are defined at the top of this file.

  action/cartesian_position, action/cartesian_velocity,
  action/gripper_position, action/gripper_velocity,
  action/joint_position, action/joint_velocity,
  observation/robot_state/cartesian_position,
  observation/robot_state/gripper_position,
  observation/robot_state/joint_positions,
  observations/videos/{hand_camera,varied_camera_1,...}

Camera mapping: defined in DEFAULT_CAMERA_ID_TO_NAME at top of file.
"""

from __future__ import annotations

import argparse
import glob
import json
import warnings
from multiprocessing import Pool
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

try:
    import tqdm as _tqdm
    tqdm = _tqdm.tqdm
except ImportError:
    def tqdm(iterable, **kwargs):
        return iterable  # no-op if tqdm not installed

try:
    import tensorflow as tf
except ImportError:
    tf = None  # Required only when writing TFRecords

# Camera_id -> tfrecord key. Edit here to add/change cameras.
CAMERA_ID_TO_NAME: Dict[str, str] = {
    "11022812": "hand_camera",
    "24285872": "varied_camera_1",
}

IMAGE_SIZE = (180, 320)  # (H, W), match droid_dataset_builder

# Keys to save (nested with /). Observation uses 'joint_positions' (state).
ACTION_KEYS = [
    "action/cartesian_position",
    "action/cartesian_velocity",
    "action/gripper_position",
    "action/gripper_velocity",
    "action/joint_position",
    "action/joint_velocity",
]
OBS_KEYS = [
    "observation/robot_state/cartesian_position",
    "observation/robot_state/gripper_position",
    "observation/robot_state/joint_positions",
]


def _get_nested(d: Dict[str, Any], path: str) -> Any:
    for k in path.split("/"):
        d = d.get(k)
        if d is None:
            return None
    return d


def _ensure_shape(x: np.ndarray, ndim: int) -> np.ndarray:
    x = np.asarray(x, dtype=np.float32)
    if x.ndim == 0:
        x = x.reshape(1)
    if x.ndim == 1 and ndim == 2:
        x = x.reshape(1, -1)
    return x


def _tensor_feature(value: tf.Tensor) -> tf.train.Feature:
    return tf.train.Feature(
        bytes_list=tf.train.BytesList(value=[tf.io.serialize_tensor(value).numpy()])
    )


def _resize_and_encode(image: np.ndarray, size: Tuple[int, int]) -> bytes:
    # size: (H, W)
    img = tf.cast(image, tf.float32)
    img = tf.image.resize(img, size, method="bicubic")
    img = tf.cast(tf.round(tf.clip_by_value(img, 0, 255)), tf.uint8)
    return tf.io.encode_jpeg(img).numpy()


def collect_h5_paths(
    data_dir: Path,
    task_labels_path: Path,
) -> Tuple[List[Path], List[Path], bool]:
    """
    Collect h5 paths under data_dir. task_labels_path = data_dir / "task_labels.json".

    Returns:
        (paths_to_use, unrelabeled_in_success, no_task_labels)
        - no_task_labels: True when task_labels.json does not exist; paths are success/ only.
        - unrelabeled_in_success: trajectories under success/ but not in task_labels (only
          when task_labels exists).
    """
    data_dir = data_dir.resolve()
    all_h5 = sorted(
        Path(p) for p in glob.glob(str(data_dir / "**" / "trajectory.h5"), recursive=True)
    )

    if not task_labels_path.is_file():
        chosen = [p for p in all_h5 if "success" in p.parts]
        return chosen, [], True

    with open(task_labels_path) as f:
        labels = json.load(f)

    in_labels_success: List[Path] = []
    in_success_folder_not_in_labels: List[Path] = []

    for p in all_h5:
        r = str(p.relative_to(data_dir)).replace("\\", "/")
        if r in labels and labels[r].get("success") is True:
            in_labels_success.append(p)
        elif "success" in p.parts and r not in labels:
            in_success_folder_not_in_labels.append(p)

    chosen = list(dict.fromkeys(in_labels_success + in_success_folder_not_in_labels))
    return chosen, in_success_folder_not_in_labels, False


def load_trajectory_as_steps(h5_path: Path, read_images: bool = True) -> Optional[List[Dict]]:
    """Load one trajectory into list of step dicts. Keys: observation, action, observations."""
    from role_ros2.trajectory_utils.trajectory_reader import TrajectoryReader

    try:
        reader = TrajectoryReader(str(h5_path), read_images=read_images)
    except Exception as e:
        warnings.warn(f"Could not open {h5_path}: {e}")
        return None

    steps = []
    try:
        for i in range(reader.length()):
            step = reader.read_timestep()
            steps.append(step)
    except Exception as e:
        warnings.warn(f"Error reading {h5_path}: {e}")
        return None
    finally:
        reader.close()

    return steps if steps else None


def _process_shard(
    path_list: List[Path],
    out_path: Path,
    image_size: Tuple[int, int],
    camera_id_to_name: Dict[str, str],
) -> int:
    """Load trajectories in path_list, convert to TFRecord, write to out_path. Returns count written."""
    count = 0
    with tf.io.TFRecordWriter(str(out_path)) as w:
        for h5 in path_list:
            steps = load_trajectory_as_steps(h5, read_images=True)
            if steps is None:
                continue
            ex = trajectory_to_example(steps, camera_id_to_name, image_size)
            if ex is not None:
                w.write(ex.SerializeToString())
                count += 1
    return count


def _run_one_shard(
    args: Tuple[List[Path], Path, Tuple[int, int], Dict[str, str]],
) -> int:
    """Adapter for pool.imap: unpack and call _process_shard. Must be at module level for pickling."""
    return _process_shard(*args)


def trajectory_to_example(
    steps: List[Dict],
    camera_id_to_name: Dict[str, str],
    image_size: Tuple[int, int],
) -> Optional[tf.train.Example]:
    """Build one tf.train.Example from a list of steps."""
    if not steps:
        return None

    def get(path: str, step: Dict) -> Optional[np.ndarray]:
        v = _get_nested(step, path)
        if v is None:
            return None
        return np.asarray(v, dtype=np.float32)

    # Actions and observations
    action_arrays: Dict[str, np.ndarray] = {}
    obs_arrays: Dict[str, np.ndarray] = {}

    for key in ACTION_KEYS:
        vals = []
        for s in steps:
            v = get(key, s)
            if v is None:
                break
            vals.append(_ensure_shape(v, 2))
        if len(vals) != len(steps):
            continue
        action_arrays[key] = np.stack(vals)

    for key in OBS_KEYS:
        vals = []
        for s in steps:
            v = _get_nested(s, key)
            if v is None:
                # try common alias
                alt = key.replace("joint_positions", "joint_position")
                v = _get_nested(s, alt)
            if v is None:
                break
            vals.append(_ensure_shape(np.asarray(v), 2))
        if len(vals) != len(steps):
            continue
        obs_arrays[key] = np.stack(vals)

    # Videos: observations/image in reader is {camera_id: ndarray}
    video_arrays: Dict[str, List[bytes]] = {}
    for s in steps:
        img = (s.get("observations") or {}).get("image") or {}
        for cid, frame in img.items():
            name = camera_id_to_name.get(cid, cid)
            if name not in video_arrays:
                video_arrays[name] = []
            video_arrays[name].append(
                _resize_and_encode(np.asarray(frame), image_size)
            )

    # All video streams must have same length as steps
    for k, lst in list(video_arrays.items()):
        if len(lst) != len(steps):
            del video_arrays[k]

    features: Dict[str, tf.train.Feature] = {}

    for k, arr in action_arrays.items():
        features[k] = _tensor_feature(tf.constant(arr))

    for k, arr in obs_arrays.items():
        features[k] = _tensor_feature(tf.constant(arr))

    for k, jpegs in video_arrays.items():
        key = f"observations/videos/{k}"
        features[key] = tf.train.Feature(
            bytes_list=tf.train.BytesList(value=jpegs)
        )

    if not features:
        return None

    return tf.train.Example(features=tf.train.Features(feature=features))


def run(
    data_dir: Path,
    output_dir: Path,
    task_labels_path: Optional[Path] = None,
    shard_size: int = 200,
    train_fraction: float = 0.8,
    overwrite: bool = False,
    image_size: Tuple[int, int] = IMAGE_SIZE,
    num_workers: int = 4,
) -> None:
    if tf is None:
        raise SystemExit(
            "tensorflow is required for TFRecord writing. pip install tensorflow"
        )

    tl_path = task_labels_path if task_labels_path is not None else data_dir / "task_labels.json"
    paths, unrelabeled, no_task_labels = collect_h5_paths(data_dir, tl_path)

    if no_task_labels:
        print("[WARNING] Data has not been relabeled (no task_labels.json in data-dir).")
        print("          Using all trajectories under success/. Type 'yes' to continue:")
        if input().strip().lower() != "yes":
            print("Aborted.")
            return
    elif unrelabeled:
        print("[WARNING] Some trajectories under success/ are not in task_labels.json (not relabeled):")
        for p in unrelabeled:
            print(f"  {p}")
        print("  Continue (include unrelabeled) or Skip (use only relabeled success)? [c/s]:")
        ans = input().strip().lower()
        if ans == "s" or ans == "skip":
            unrelabeled_set = set(unrelabeled)
            paths = [p for p in paths if p not in unrelabeled_set]
        # else: continue with all paths (include unrelabeled)

    if not paths:
        print("No trajectories to convert.")
        return

    if output_dir.exists() and not overwrite:
        print(f"Output exists: {output_dir}. Use --overwrite to replace.")
        return

    import shutil
    if overwrite and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    np.random.shuffle(paths)
    n_train = int(len(paths) * train_fraction)
    train_paths = paths[:n_train]
    test_paths = paths[n_train:]

    (output_dir / "train").mkdir(parents=True, exist_ok=True)
    (output_dir / "test").mkdir(parents=True, exist_ok=True)

    tasks: List[Tuple[List[Path], Path, Tuple[int, int], Dict[str, str]]] = []
    for name, path_list in [("train", train_paths), ("test", test_paths)]:
        n = max(1, int(np.ceil(len(path_list) / shard_size)))
        for i, shard in enumerate(np.array_split(path_list, n)):
            out_path = output_dir / name / f"{i}.tfrecord"
            tasks.append((list(shard), out_path, image_size, CAMERA_ID_TO_NAME))

    if num_workers <= 1:
        results = [
            _run_one_shard(t)
            for t in tqdm(tasks, desc="Shards")
        ]
    else:
        with Pool(processes=num_workers) as pool:
            results = list(
                tqdm(
                    pool.imap(_run_one_shard, tasks, chunksize=1),
                    total=len(tasks),
                    desc="Shards",
                )
            )

    total_written = sum(results)
    print(f"Done. train={len(train_paths)}, test={len(test_paths)}; wrote {total_written} examples -> {output_dir}")


def main() -> None:
    ap = argparse.ArgumentParser(description="Convert role-ros2 .h5 to TFRecord.")
    ap.add_argument(
        "--data-dir",
        type=Path,
        required=True,
        help="Data root (contains success/, failure/, task_labels.json)",
    )
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output TFRecord directory (train/, test/). Default: <data-dir>/tfrecord",
    )
    ap.add_argument(
        "--shard-size",
        type=int,
        default=200,
        help="Max trajectories per TFRecord shard (default: 200)",
    )
    ap.add_argument(
        "--train-fraction",
        type=float,
        default=0.8,
        help="Fraction of data for train split; rest for test (default: 0.8)",
    )
    ap.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing output directory if it exists",
    )
    ap.add_argument(
        "--image-height",
        type=int,
        default=IMAGE_SIZE[0],
        help="Resize height for video frames (default: %(default)s)",
    )
    ap.add_argument(
        "--image-width",
        type=int,
        default=IMAGE_SIZE[1],
        help="Resize width for video frames (default: %(default)s)",
    )
    ap.add_argument(
        "--num-workers",
        type=int,
        default=4,
        help="Number of parallel workers for shard conversion (default: 4). Use 1 for sequential.",
    )

    args = ap.parse_args()

    data_dir = args.data_dir.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else (data_dir / "tfrecord")
    )
    task_labels_path = data_dir / "task_labels.json"

    run(
        data_dir=data_dir,
        output_dir=output_dir,
        task_labels_path=task_labels_path,
        shard_size=args.shard_size,
        train_fraction=args.train_fraction,
        overwrite=args.overwrite,
        image_size=(args.image_height, args.image_width),
        num_workers=args.num_workers,
    )


if __name__ == "__main__":
    main()

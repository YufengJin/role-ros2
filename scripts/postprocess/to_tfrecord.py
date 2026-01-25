#!/usr/bin/env python3
"""
to_tfrecord.py

Convert role-ros2 .h5 trajectories to TFRecord in TFDS/RLDS format (keys match
droid_dataset_builder/droid). Language instruction = task_name from
task_labels.json or HDF5 metadata.

- Input: --data-dir. task_labels.json from <data-dir>/task_labels.json.
- If no task_labels.json: use success/**/trajectory.h5, prompt to continue.
- If task_labels.json exists: use h5 with success=True; task_name from
  labels[rel]["task_name"] or HDF5 attrs["task_name"] or "unknown".

Output: Standard TFDS dataset in <output-dir>/<dataset-name>/<version>/
  - dataset_info.json, features.json
  - {DATASET}-{SPLIT}.tfrecord-{X}-of-{Y}  (e.g. role_ros2-train.tfrecord-00000-of-00005)
  - Usable with tfds.builder_from_directory() and tfds.load().

Each TFRecord entry = one episode:
  - steps: [ { observation: exterior_image_1/2_left, wrist_image_left,
               cartesian_position(6), gripper_position(1), joint_position(7);
               action_dict; action(7)=cartesian6+gripper1; discount; reward;
               is_first; is_last; is_terminal; language_instruction=task_name }, ... ]
  - episode_metadata: file_path, recording_folderpath

Camera mapping: CAMERA_ID_TO_IMAGE_KEYS (cam_id -> RLDS/DROID observation key).
Requires tensorflow_datasets.
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

try:
    import tensorflow_datasets as tfds
    from tensorflow_datasets.core import example_serializer
except ImportError:
    tfds = None
    example_serializer = None

# cam_id -> RLDS/DROID observation key. Edit here to add/change cameras.
CAMERA_ID_TO_IMAGE_KEYS: Dict[str, str] = {
    "11022812": "wrist_image_left",
    "24285872": "exterior_image_1_left",
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

# TFDS/RLDS FeaturesDict matching droid_dataset_builder/droid/droid.py
_RLDS_FEATURES = None
_RLDS_SERIALIZER = None


def _get_rlds_features():
    global _RLDS_FEATURES, _RLDS_SERIALIZER
    if _RLDS_FEATURES is not None:
        return _RLDS_FEATURES, _RLDS_SERIALIZER
    if tfds is None or example_serializer is None:
        raise RuntimeError("tensorflow_datasets is required for TFDS output. pip install tensorflow-datasets")
    _RLDS_FEATURES = tfds.features.FeaturesDict({
        "steps": tfds.features.Dataset({
            "observation": tfds.features.FeaturesDict({
                "exterior_image_1_left": tfds.features.Image(
                    shape=(*IMAGE_SIZE, 3), dtype=np.uint8, encoding_format="jpeg"
                ),
                "exterior_image_2_left": tfds.features.Image(
                    shape=(*IMAGE_SIZE, 3), dtype=np.uint8, encoding_format="jpeg"
                ),
                "wrist_image_left": tfds.features.Image(
                    shape=(*IMAGE_SIZE, 3), dtype=np.uint8, encoding_format="jpeg"
                ),
                "cartesian_position": tfds.features.Tensor(shape=(6,), dtype=np.float64),
                "gripper_position": tfds.features.Tensor(shape=(1,), dtype=np.float64),
                "joint_position": tfds.features.Tensor(shape=(7,), dtype=np.float64),
            }),
            "action_dict": tfds.features.FeaturesDict({
                "cartesian_position": tfds.features.Tensor(shape=(6,), dtype=np.float64),
                "cartesian_velocity": tfds.features.Tensor(shape=(6,), dtype=np.float64),
                "gripper_position": tfds.features.Tensor(shape=(1,), dtype=np.float64),
                "gripper_velocity": tfds.features.Tensor(shape=(1,), dtype=np.float64),
                "joint_position": tfds.features.Tensor(shape=(7,), dtype=np.float64),
                "joint_velocity": tfds.features.Tensor(shape=(7,), dtype=np.float64),
            }),
            "action": tfds.features.Tensor(shape=(7,), dtype=np.float64),
            "discount": tfds.features.Scalar(dtype=np.float32),
            "reward": tfds.features.Scalar(dtype=np.float32),
            "is_first": tfds.features.Scalar(dtype=np.bool_),
            "is_last": tfds.features.Scalar(dtype=np.bool_),
            "is_terminal": tfds.features.Scalar(dtype=np.bool_),
            "language_instruction": tfds.features.Text(),
        }),
        "episode_metadata": tfds.features.FeaturesDict({
            "file_path": tfds.features.Text(),
            "recording_folderpath": tfds.features.Text(),
        }),
    })
    ser_info = _RLDS_FEATURES.get_serialized_info()
    _RLDS_SERIALIZER = example_serializer.ExampleSerializer(ser_info)
    return _RLDS_FEATURES, _RLDS_SERIALIZER


def _get_nested(d: Dict[str, Any], path: str) -> Any:
    for k in path.split("/"):
        d = d.get(k)
        if d is None:
            return None
    return d


def _resize_image(image: np.ndarray, size: Tuple[int, int]) -> np.ndarray:
    """Resize to (H, W) and return uint8 (H, W, 3)."""
    img = np.asarray(image, dtype=np.uint8)
    if img.ndim == 2:
        img = np.stack([img] * 3, axis=-1)
    img = tf.cast(img, tf.float32)
    img = tf.image.resize(img, size, method="bicubic")
    img = tf.cast(tf.round(tf.clip_by_value(img, 0, 255)), tf.uint8)
    return img.numpy()


def collect_h5_paths(
    data_dir: Path,
    task_labels_path: Path,
) -> Tuple[List[Path], List[Path], bool, Dict[str, Dict[str, Any]]]:
    """
    Collect h5 paths under data_dir. task_labels_path = data_dir / "task_labels.json".

    Returns:
        (paths_to_use, unrelabeled_in_success, no_task_labels, labels)
        - no_task_labels: True when task_labels.json does not exist; paths are success/ only.
        - unrelabeled_in_success: trajectories under success/ but not in task_labels (only
          when task_labels exists).
        - labels: dict mapping rel_path -> {success, failure, task_name}; {} when no file.
    """
    data_dir = data_dir.resolve()
    all_h5 = sorted(
        Path(p) for p in glob.glob(str(data_dir / "**" / "trajectory.h5"), recursive=True)
    )

    if not task_labels_path.is_file():
        chosen = [p for p in all_h5 if "success" in p.parts]
        return chosen, [], True, {}

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
    return chosen, in_success_folder_not_in_labels, False, labels


def load_trajectory_as_steps(
    h5_path: Path, read_images: bool = True
) -> Optional[Tuple[List[Dict], Dict[str, Any]]]:
    """Load one trajectory into list of step dicts and file metadata.

    Returns:
        (steps, metadata) or None. metadata from HDF5 attrs (e.g. task_name).
    """
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
        metadata = reader.get_metadata()
    except Exception as e:
        warnings.warn(f"Error reading {h5_path}: {e}")
        return None
    finally:
        reader.close()

    return (steps, metadata) if steps else None


def _process_shard(
    path_list: List[Path],
    out_path: Path,
    image_size: Tuple[int, int],
    camera_id_to_image_keys: Dict[str, str],
    data_dir: Path,
    labels: Dict[str, Dict[str, Any]],
    split_name: str = "",
) -> int:
    """Load trajectories, convert to TFDS/RLDS, write to out_path. Returns count written."""
    count = 0
    with tf.io.TFRecordWriter(str(out_path)) as w:
        for h5 in path_list:
            out = load_trajectory_as_steps(h5, read_images=True)
            if out is None:
                continue
            steps, metadata = out
            rel = str(h5.relative_to(data_dir)).replace("\\", "/")
            task_name = (
                (labels.get(rel) or {}).get("task_name")
                or metadata.get("task_name")
                or "unknown"
            )
            if isinstance(task_name, bytes):
                task_name = task_name.decode("utf-8", errors="replace")
            rec_dir = h5.parent / "recordings" / "MP4"
            recording_folderpath = str(rec_dir) if rec_dir.exists() else ""
            serialized = trajectory_to_example_tfds(
                steps,
                task_name=task_name,
                h5_path=h5,
                recording_folderpath=recording_folderpath,
                camera_id_to_image_keys=camera_id_to_image_keys,
                image_size=image_size,
            )
            if serialized is not None:
                w.write(serialized)
                count += 1
    return count


def _run_one_shard(args) -> int:
    """Adapter for pool.imap: unpack and call _process_shard. Must be at module level for pickling."""
    return _process_shard(*args)


def _vec(x: Any, size: int, dtype=np.float64) -> np.ndarray:
    """Ensure 1d array of given size, zero-pad or trim. None -> zeros."""
    if x is None:
        return np.zeros(size, dtype=dtype)
    a = np.asarray(x, dtype=dtype).flatten()
    if len(a) >= size:
        return a[:size].astype(dtype)
    return np.concatenate([a, np.zeros(size - len(a), dtype=dtype)]).astype(dtype)


def trajectory_to_example_tfds(
    steps: List[Dict],
    task_name: str,
    h5_path: Path,
    recording_folderpath: str,
    camera_id_to_image_keys: Dict[str, str],
    image_size: Tuple[int, int],
) -> Optional[bytes]:
    """Build one TFDS/RLDS episode (steps + episode_metadata) and return serialized bytes."""
    if not steps:
        return None
    feats, ser = _get_rlds_features()

    # Per-step images: map cam_id -> rlds key -> list of (H,W,3) uint8
    img_by_rlds: Dict[str, List[np.ndarray]] = {}
    for s in steps:
        imgs = (s.get("observations") or {}).get("image") or {}
        for cid, frame in imgs.items():
            rlds_key = camera_id_to_image_keys.get(cid)
            if rlds_key is None:
                continue
            if rlds_key not in img_by_rlds:
                img_by_rlds[rlds_key] = []
            img_by_rlds[rlds_key].append(_resize_image(np.asarray(frame), image_size))
    # Require at least wrist and one exterior for 3-image schema
    if "wrist_image_left" not in img_by_rlds or "exterior_image_1_left" not in img_by_rlds:
        return None
    for k, lst in list(img_by_rlds.items()):
        if len(lst) != len(steps):
            del img_by_rlds[k]
    if "exterior_image_2_left" not in img_by_rlds:
        img_by_rlds["exterior_image_2_left"] = img_by_rlds["exterior_image_1_left"]

    rlds_steps = []
    for i, s in enumerate(steps):
        # observation
        cart = _get_nested(s, "observation/robot_state/cartesian_position")
        grip = _get_nested(s, "observation/robot_state/gripper_position")
        joint = _get_nested(s, "observation/robot_state/joint_positions")
        if joint is None:
            joint = _get_nested(s, "observation/robot_state/joint_position")
        obs = {
            "exterior_image_1_left": img_by_rlds["exterior_image_1_left"][i],
            "exterior_image_2_left": img_by_rlds["exterior_image_2_left"][i],
            "wrist_image_left": img_by_rlds["wrist_image_left"][i],
            "cartesian_position": _vec(cart, 6),
            "gripper_position": _vec(grip, 1),
            "joint_position": _vec(joint, 7),
        }
        # action_dict
        ac = s.get("action") or {}
        if isinstance(ac, dict):
            ad = {
                "cartesian_position": _vec(ac.get("cartesian_position"), 6),
                "cartesian_velocity": _vec(ac.get("cartesian_velocity"), 6),
                "gripper_position": _vec(ac.get("gripper_position"), 1),
                "gripper_velocity": _vec(ac.get("gripper_velocity"), 1),
                "joint_position": _vec(ac.get("joint_position"), 7),
                "joint_velocity": _vec(ac.get("joint_velocity"), 7),
            }
        else:
            ad = {
                "cartesian_position": _vec(_get_nested(s, "action/cartesian_position"), 6),
                "cartesian_velocity": _vec(_get_nested(s, "action/cartesian_velocity"), 6),
                "gripper_position": _vec(_get_nested(s, "action/gripper_position"), 1),
                "gripper_velocity": _vec(_get_nested(s, "action/gripper_velocity"), 1),
                "joint_position": _vec(_get_nested(s, "action/joint_position"), 7),
                "joint_velocity": _vec(_get_nested(s, "action/joint_velocity"), 7),
            }
        # action (7) = cartesian 6 + gripper 1
        act_cart = ad["cartesian_position"]
        act_grip = ad["gripper_position"]
        action = np.concatenate([act_cart, act_grip]).astype(np.float64)
        last = i == len(steps) - 1
        rlds_steps.append({
            "observation": obs,
            "action_dict": ad,
            "action": action,
            "discount": np.float32(1.0),
            "reward": np.float32(1.0 if last else 0.0),
            "is_first": i == 0,
            "is_last": last,
            "is_terminal": last,
            "language_instruction": str(task_name),
        })

    episode = {
        "steps": rlds_steps,
        "episode_metadata": {
            "file_path": str(h5_path),
            "recording_folderpath": str(recording_folderpath),
        },
    }
    try:
        encoded = feats.encode_example(episode)
        return ser.serialize_example(encoded)
    except Exception as e:
        warnings.warn(f"encode/serialize failed for {h5_path}: {e}")
        return None


def run(
    data_dir: Path,
    output_dir: Path,
    task_labels_path: Optional[Path] = None,
    shard_size: int = 200,
    train_fraction: float = 0.8,
    overwrite: bool = False,
    image_size: Tuple[int, int] = IMAGE_SIZE,
    num_workers: int = 4,
    dataset_name: str = "role_ros2",
    version: str = "1.0.0",
) -> None:
    if tf is None:
        raise SystemExit(
            "tensorflow is required for TFRecord writing. pip install tensorflow"
        )

    tl_path = task_labels_path if task_labels_path is not None else data_dir / "task_labels.json"
    paths, unrelabeled, no_task_labels, labels = collect_h5_paths(data_dir, tl_path)

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

    version_dir = output_dir / dataset_name / version
    if version_dir.exists() and not overwrite:
        print(f"Output exists: {version_dir}. Use --overwrite to replace.")
        return

    import shutil
    if overwrite and version_dir.exists():
        shutil.rmtree(version_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    version_dir.mkdir(parents=True, exist_ok=True)

    np.random.shuffle(paths)
    n_train = int(len(paths) * train_fraction)
    train_paths = paths[:n_train]
    test_paths = paths[n_train:]

    # Resolve data_dir for path.relative_to in _process_shard
    data_dir = data_dir.resolve()
    feats, _ = _get_rlds_features()  # raise early if tensorflow_datasets missing

    tasks: List[tuple] = []
    for name, path_list in [("train", train_paths), ("test", test_paths)]:
        n_shards = max(1, int(np.ceil(len(path_list) / shard_size)))
        for i, shard in enumerate(np.array_split(path_list, n_shards)):
            out_path = version_dir / f"{dataset_name}-{name}.tfrecord-{i:05d}-of-{n_shards:05d}"
            tasks.append((
                list(shard), out_path, image_size, CAMERA_ID_TO_IMAGE_KEYS,
                data_dir, labels, name,
            ))

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

    # Aggregate shard lengths and bytes per split for dataset_info.json
    split_info: Dict[str, Dict[str, Any]] = {"train": {"shardLengths": [], "numBytes": 0}, "test": {"shardLengths": [], "numBytes": 0}}
    for task, count in zip(tasks, results):
        _, out_path, _, _, _, _, split_name = task
        split_info[split_name]["shardLengths"].append(str(count))
        split_info[split_name]["numBytes"] += out_path.stat().st_size

    # dataset_info.json (TFDS DatasetInfo proto–compatible)
    dataset_info_obj = {
        "fileFormat": "tfrecord",
        "moduleName": "role_ros2.to_tfrecord",
        "name": dataset_name,
        "version": version,
        "releaseNotes": {version: "Initial release."},
        "splits": [
            {
                "filepathTemplate": "{DATASET}-{SPLIT}.{FILEFORMAT}-{SHARD_X_OF_Y}",
                "name": sn,
                "numBytes": str(split_info[sn]["numBytes"]),
                "shardLengths": split_info[sn]["shardLengths"],
            }
            for sn in ["train", "test"]
        ],
    }
    with open(version_dir / "dataset_info.json", "w", encoding="utf-8") as f:
        json.dump(dataset_info_obj, f, indent=2, ensure_ascii=False)

    # features.json (tfds FeatureConnector.to_json format for from_json)
    with open(version_dir / "features.json", "w", encoding="utf-8") as f:
        json.dump(feats.to_json(), f, indent=2, ensure_ascii=False)

    total_written = sum(results)
    print(f"Done. train={len(train_paths)}, test={len(test_paths)}; wrote {total_written} examples -> {version_dir}")
    print(f"  TFDS: tfds.builder_from_directory({repr(str(version_dir))})  ->  builder.as_dataset(split='train')")


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
    ap.add_argument(
        "--dataset-name",
        type=str,
        default="role_ros2",
        help="TFDS dataset name for dir and filenames (default: role_ros2).",
    )
    ap.add_argument(
        "--version",
        type=str,
        default="1.0.0",
        help="TFDS version subdir (default: 1.0.0).",
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
        dataset_name=args.dataset_name,
        version=args.version,
    )


if __name__ == "__main__":
    main()

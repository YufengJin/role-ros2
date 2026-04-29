"""Configuration helpers for FACTR teleoperation."""

import os
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

from role_ros2.misc.config_loader import get_package_config_path


DEFAULT_FACTR_CONFIG = "factr_teleop_config.yaml"


def resolve_factr_config_path(config_path: Optional[str] = None) -> Path:
    """Resolve a FACTR config path from an explicit path or role_ros2 config/."""
    if config_path:
        path = Path(os.path.expandvars(os.path.expanduser(config_path)))
        if path.exists():
            return path
        raise FileNotFoundError(f"FACTR config file not found: {path}")
    return get_package_config_path(DEFAULT_FACTR_CONFIG)


def load_factr_config(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Load FACTR YAML config."""
    path = resolve_factr_config_path(config_path)
    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    cfg["_config_path"] = str(path)
    cfg["_config_dir"] = str(path.parent)
    return cfg


def resolve_urdf_path(config: Dict[str, Any]) -> Path:
    """Resolve the leader URDF path from config."""
    arm_cfg = config.get("arm_teleop", {})
    urdf_name = arm_cfg.get("leader_urdf", "")
    if not urdf_name:
        raise ValueError("FACTR config missing arm_teleop.leader_urdf")

    raw_path = Path(os.path.expandvars(os.path.expanduser(str(urdf_name))))
    if raw_path.is_absolute():
        if raw_path.exists():
            return raw_path
        raise FileNotFoundError(f"FACTR leader URDF not found: {raw_path}")

    config_dir = Path(config.get("_config_dir", "."))
    candidates = [
        config_dir / "factr" / "urdf" / raw_path,
        config_dir / "urdf" / raw_path,
        config_dir / raw_path,
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError(
        "FACTR leader URDF not found. Tried: "
        + ", ".join(str(candidate) for candidate in candidates)
    )

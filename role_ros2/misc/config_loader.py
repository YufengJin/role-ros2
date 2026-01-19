"""
Configuration loader utilities for role_ros2.

Provides unified interface for loading configurations from:
1. ROS2 package config/ directory (for launch files and ROS2 nodes)
2. Module-specific config files in config/ directory
"""

import os
import yaml
from pathlib import Path
from typing import Optional, Dict, Any

try:
    from ament_index_python.packages import get_package_share_directory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def get_package_config_path(config_file: str, package_name: str = "role_ros2") -> Path:
    """
    Get path to configuration file in ROS2 package config/ directory.
    
    This function tries to find the config file through ROS2 package system first.
    If ROS2 is not available (e.g., when running Python code directly), it falls
    back to finding the file relative to the workspace root.
    
    Args:
        config_file: Configuration file name (e.g., 'multi_camera_config.yaml')
        package_name: ROS2 package name (default: 'role_ros2')
    
    Returns:
        Path to configuration file
    
    Raises:
        FileNotFoundError: If config file cannot be found
    """
    if ROS2_AVAILABLE:
        try:
            package_share_dir = get_package_share_directory(package_name)
            config_path = Path(package_share_dir) / "config" / config_file
            if config_path.exists():
                return config_path
        except Exception:
            # Fall through to fallback method
            pass
    
    # Fallback: try to find relative to workspace root
    # This works when running Python code directly (not through ROS2)
    current_file = Path(__file__)
    # role_ros2/config_loader.py -> role_ros2/ -> workspace root
    workspace_root = current_file.parent.parent
    config_path = workspace_root / "config" / config_file
    
    if config_path.exists():
        return config_path
    
    # Build error message
    error_msg = f"Config file not found: {config_file}.\n"
    if ROS2_AVAILABLE:
        try:
            package_share_dir = get_package_share_directory(package_name)
            error_msg += f"  Tried ROS2 package path: {Path(package_share_dir) / 'config' / config_file}\n"
        except Exception:
            error_msg += f"  ROS2 package '{package_name}' not found\n"
    error_msg += f"  Tried workspace path: {config_path}"
    raise FileNotFoundError(error_msg)


def load_yaml_config(config_file: str, package_name: str = "role_ros2") -> Dict[str, Any]:
    """
    Load YAML configuration file from ROS2 package config/ directory.
    
    Args:
        config_file: Configuration file name (e.g., 'multi_camera_config.yaml')
        package_name: ROS2 package name (default: 'role_ros2')
    
    Returns:
        Configuration dictionary (empty dict if file is empty)
    
    Raises:
        FileNotFoundError: If config file not found
        yaml.YAMLError: If YAML parsing fails
    """
    config_path = get_package_config_path(config_file, package_name)
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    return config if config else {}


def get_source_config_path(config_file: str) -> Path:
    """
    Get path to configuration file in source directory config/ folder.
    
    This function always returns the path relative to the workspace source root,
    regardless of ROS2 installation. Use this for writing files that should
    be saved in the source directory (e.g., calibration results).
    
    Args:
        config_file: Configuration file name (e.g., 'calibration_results.yaml')
    
    Returns:
        Path to configuration file in source directory
    """
    current_file = Path(__file__)
    # role_ros2/config_loader.py -> role_ros2/ -> workspace root
    workspace_root = current_file.parent.parent
    config_path = workspace_root / "config" / config_file
    return config_path


def load_yaml_config_safe(config_file: str, package_name: str = "role_ros2", default: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """
    Load YAML configuration file safely (returns default if file not found).
    
    This is a safe wrapper around load_yaml_config that catches FileNotFoundError
    and returns a default value instead.
    
    Args:
        config_file: Configuration file name
        package_name: ROS2 package name
        default: Default value to return if file not found (default: empty dict)
    
    Returns:
        Configuration dictionary or default value
    """
    if default is None:
        default = {}
    
    try:
        return load_yaml_config(config_file, package_name)
    except FileNotFoundError:
        return default


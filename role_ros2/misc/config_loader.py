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


def get_package_config_path(config_file: str, package_name: str = "role_ros2", use_source: bool = True) -> Path:
    """
    Get path to configuration file in config/ directory.
    
    By default (use_source=True), this function prioritizes the source directory
    over the ROS2 package install directory. This ensures that configuration files
    are read from and written to the source directory, not the install directory.
    
    Args:
        config_file: Configuration file name (e.g., 'multi_camera_config.yaml')
        package_name: ROS2 package name (default: 'role_ros2')
        use_source: If True, prioritize source directory over install directory (default: True)
    
    Returns:
        Path to configuration file
    
    Raises:
        FileNotFoundError: If config file cannot be found
    """
    # If use_source is True, try source directory first
    source_path = None
    if use_source:
        source_path = get_source_config_path(config_file)
        if source_path.exists():
            return source_path
    
    # Fallback: try ROS2 package install directory (for backward compatibility)
    if ROS2_AVAILABLE:
        try:
            package_share_dir = get_package_share_directory(package_name)
            config_path = Path(package_share_dir) / "config" / config_file
            if config_path.exists():
                return config_path
        except Exception:
            # Fall through to fallback method
            pass
    
    # Final fallback: try to find relative to workspace root (when running from source)
    current_file = Path(__file__).resolve()
    workspace_root = current_file.parent.parent
    config_path = workspace_root / "config" / config_file
    
    if config_path.exists():
        return config_path
    
    # If use_source is True and file doesn't exist, return source path anyway (for writing)
    if use_source and source_path is not None:
        return source_path
    
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
    
    The function tries multiple methods to find the source directory:
    1. Environment variable ROLE_ROS2_SOURCE_DIR (if set)
    2. Infer from workspace structure (install/... -> src/role-ros2/config/)
    3. Infer from current file location (when running from source)
    
    Args:
        config_file: Configuration file name (e.g., 'calibration_results.yaml')
    
    Returns:
        Path to configuration file in source directory
    """
    # Method 1: Check environment variable (highest priority)
    if 'ROLE_ROS2_SOURCE_DIR' in os.environ:
        source_dir = Path(os.environ['ROLE_ROS2_SOURCE_DIR'])
        config_path = source_dir / "config" / config_file
        if config_path.exists():
            return config_path
    
    # Method 2: Try to infer from workspace structure (for Docker/install environments)
    # Look for install directory and go up to find src/role-ros2/config/
    current_file = Path(__file__).resolve()
    
    # Try to find workspace root by going up from install directory
    # install/role_ros2/local/lib/python3.10/dist-packages/role_ros2/config_loader.py
    # -> install -> workspace root -> src/role-ros2/config/
    for parent in current_file.parents:
        if parent.name == 'install':
            # Found install directory, go up to workspace root
            workspace_root = parent.parent
            src_config_path = workspace_root / "src" / "role-ros2" / "config" / config_file
            if src_config_path.exists():
                return src_config_path
    
    # Method 3: Infer from current file location (when running from source)
    # role_ros2/config_loader.py -> role_ros2/ -> workspace root -> config/
    workspace_root = current_file.parent.parent
    config_path = workspace_root / "config" / config_file
    
    # If file exists at this path, return it
    if config_path.exists():
        return config_path
    
    # If file doesn't exist, still return the path (for writing)
    # This allows the function to be used for both reading and writing
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


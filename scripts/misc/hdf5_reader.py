#!/usr/bin/env python3
"""
HDF5 Reader - Tool for inspecting HDF5 trajectory files.

This tool displays detailed information about HDF5 files including:
- Basic file info (size, creation time, etc.)
- Total number of timestamps
- Data structure hierarchy
- Array shapes and dtypes for all datasets
- Metadata/attributes

Usage:
    python3 hdf5_reader.py /path/to/trajectory.h5
    python3 hdf5_reader.py /path/to/trajectory.h5 --show-data 0  # Show data at index 0
    python3 hdf5_reader.py /path/to/trajectory.h5 --show-all-data  # Show all data
    python3 hdf5_reader.py /path/to/trajectory.h5 --max-depth 3  # Limit structure depth

Author: Chaser Robotics Team
"""

import argparse
import json
import os
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Union

import h5py
import numpy as np


def format_size(size_bytes: int) -> str:
    """Format byte size to human readable string."""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size_bytes < 1024.0:
            return f"{size_bytes:.2f} {unit}"
        size_bytes /= 1024.0
    return f"{size_bytes:.2f} PB"


def format_dtype(dtype) -> str:
    """Format numpy dtype to readable string."""
    if dtype.kind == 'S':
        return f"string[{dtype.itemsize}]"
    elif dtype.kind == 'U':
        return f"unicode[{dtype.itemsize // 4}]"
    elif dtype.kind == 'V':
        return f"void[{dtype.itemsize}]"
    return str(dtype)


def format_value(value: Any, max_len: int = 100) -> str:
    """Format a value for display, truncating if necessary."""
    if isinstance(value, np.ndarray):
        if value.size <= 10:
            val_str = np.array2string(value, separator=', ', threshold=10)
        else:
            val_str = f"array(shape={value.shape}, dtype={value.dtype})"
        return val_str[:max_len] + "..." if len(val_str) > max_len else val_str
    elif isinstance(value, bytes):
        try:
            decoded = value.decode('utf-8')
            return f'"{decoded}"' if len(decoded) <= max_len else f'"{decoded[:max_len]}..."'
        except UnicodeDecodeError:
            return f"<bytes[{len(value)}]>"
    elif isinstance(value, str):
        return f'"{value}"' if len(value) <= max_len else f'"{value[:max_len]}..."'
    else:
        val_str = str(value)
        return val_str[:max_len] + "..." if len(val_str) > max_len else val_str


def get_hdf5_structure(
    hdf5_obj: Union[h5py.File, h5py.Group],
    prefix: str = "",
    max_depth: int = -1,
    current_depth: int = 0
) -> List[Dict]:
    """
    Recursively get the structure of an HDF5 file/group.
    
    Args:
        hdf5_obj: HDF5 file or group
        prefix: Path prefix for nested items
        max_depth: Maximum depth to traverse (-1 for unlimited)
        current_depth: Current recursion depth
    
    Returns:
        List of dictionaries describing each item
    """
    structure = []
    
    if max_depth != -1 and current_depth >= max_depth:
        return structure
    
    for key in hdf5_obj.keys():
        item = hdf5_obj[key]
        path = f"{prefix}/{key}" if prefix else key
        
        if isinstance(item, h5py.Group):
            info = {
                "path": path,
                "type": "Group",
                "num_children": len(item.keys()),
                "attrs": dict(item.attrs) if item.attrs else {}
            }
            structure.append(info)
            # Recursively get children
            structure.extend(get_hdf5_structure(
                item, path, max_depth, current_depth + 1
            ))
        elif isinstance(item, h5py.Dataset):
            info = {
                "path": path,
                "type": "Dataset",
                "shape": item.shape,
                "dtype": format_dtype(item.dtype),
                "size": item.size,
                "nbytes": item.nbytes if hasattr(item, 'nbytes') else item.size * item.dtype.itemsize,
                "attrs": dict(item.attrs) if item.attrs else {}
            }
            structure.append(info)
    
    return structure


def get_trajectory_length(hdf5_file: h5py.File) -> Optional[int]:
    """
    Get the length of the trajectory (number of timestamps).
    
    Args:
        hdf5_file: HDF5 file object
    
    Returns:
        Length of trajectory or None if cannot be determined
    """
    lengths = []
    
    def find_lengths(obj, path=""):
        if isinstance(obj, h5py.Dataset):
            if len(obj.shape) > 0:
                lengths.append(obj.shape[0])
        elif isinstance(obj, h5py.Group):
            for key in obj.keys():
                find_lengths(obj[key], f"{path}/{key}")
    
    find_lengths(hdf5_file)
    
    if not lengths:
        return None
    
    # Return minimum length (most conservative estimate)
    return min(lengths)


def print_separator(char: str = "=", length: int = 80):
    """Print a separator line."""
    print(char * length)


def print_header(title: str):
    """Print a section header."""
    print()
    print_separator("=")
    print(f"  {title}")
    print_separator("=")


def print_subheader(title: str):
    """Print a subsection header."""
    print()
    print_separator("-")
    print(f"  {title}")
    print_separator("-")


def print_file_info(filepath: str, hdf5_file: h5py.File):
    """Print basic file information."""
    print_header("FILE INFORMATION")
    
    # File stats
    stat = os.stat(filepath)
    file_size = format_size(stat.st_size)
    mod_time = datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
    
    print(f"  Path:          {filepath}")
    print(f"  Size:          {file_size}")
    print(f"  Modified:      {mod_time}")
    print(f"  HDF5 driver:   {hdf5_file.driver}")
    print(f"  HDF5 libver:   {hdf5_file.libver}")
    
    # Trajectory length
    length = get_trajectory_length(hdf5_file)
    if length is not None:
        print(f"  Timestamps:    {length}")
    else:
        print(f"  Timestamps:    Unknown")


def print_attributes(hdf5_file: h5py.File):
    """Print file-level attributes (metadata)."""
    print_header("METADATA (File Attributes)")
    
    if not hdf5_file.attrs:
        print("  No file-level attributes found.")
        return
    
    for key in sorted(hdf5_file.attrs.keys()):
        value = hdf5_file.attrs[key]
        formatted_value = format_value(value, max_len=60)
        
        # Try to parse JSON strings
        if isinstance(value, (str, bytes)):
            try:
                if isinstance(value, bytes):
                    value = value.decode('utf-8')
                if value.startswith(('[', '{')):
                    parsed = json.loads(value)
                    formatted_value = f"{type(parsed).__name__}: {format_value(parsed, max_len=60)}"
            except (json.JSONDecodeError, UnicodeDecodeError):
                pass
        
        print(f"  {key}: {formatted_value}")


def print_structure(structure: List[Dict], show_attrs: bool = False):
    """Print the data structure."""
    print_header("DATA STRUCTURE")
    
    if not structure:
        print("  Empty file.")
        return
    
    # Count groups and datasets
    groups = [s for s in structure if s["type"] == "Group"]
    datasets = [s for s in structure if s["type"] == "Dataset"]
    
    print(f"  Total Groups:   {len(groups)}")
    print(f"  Total Datasets: {len(datasets)}")
    print()
    
    # Print tree structure
    print("  Structure Tree:")
    print("  " + "-" * 76)
    
    for item in structure:
        depth = item["path"].count("/")
        indent = "  " * (depth + 1)
        
        if item["type"] == "Group":
            print(f"  {indent}📁 {item['path'].split('/')[-1]}/  ({item['num_children']} items)")
        else:
            name = item["path"].split("/")[-1]
            shape_str = str(item["shape"])
            dtype_str = item["dtype"]
            size_str = format_size(item["nbytes"])
            print(f"  {indent}📄 {name}")
            print(f"  {indent}   ├─ shape: {shape_str}")
            print(f"  {indent}   ├─ dtype: {dtype_str}")
            print(f"  {indent}   └─ size:  {size_str}")
        
        if show_attrs and item.get("attrs"):
            for attr_key, attr_val in item["attrs"].items():
                print(f"  {indent}   [attr] {attr_key}: {format_value(attr_val)}")


def print_datasets_summary(structure: List[Dict]):
    """Print a summary table of all datasets."""
    print_header("DATASETS SUMMARY")
    
    datasets = [s for s in structure if s["type"] == "Dataset"]
    
    if not datasets:
        print("  No datasets found.")
        return
    
    # Table header
    print(f"  {'Path':<45} {'Shape':<20} {'Dtype':<15} {'Size':<12}")
    print("  " + "-" * 92)
    
    total_bytes = 0
    for ds in datasets:
        path = ds["path"]
        if len(path) > 44:
            path = "..." + path[-41:]
        shape = str(ds["shape"])
        if len(shape) > 19:
            shape = shape[:16] + "..."
        dtype = ds["dtype"]
        if len(dtype) > 14:
            dtype = dtype[:11] + "..."
        size = format_size(ds["nbytes"])
        total_bytes += ds["nbytes"]
        
        print(f"  {path:<45} {shape:<20} {dtype:<15} {size:<12}")
    
    print("  " + "-" * 92)
    print(f"  {'Total:':<45} {'':<20} {'':<15} {format_size(total_bytes):<12}")


def print_data_at_index(hdf5_file: h5py.File, index: int, max_depth: int = 3):
    """Print data at a specific timestamp index."""
    print_header(f"DATA AT INDEX {index}")
    
    def print_data_recursive(obj, path="", depth=0):
        if depth > max_depth:
            return
        
        indent = "  " * (depth + 1)
        
        if isinstance(obj, h5py.Group):
            for key in sorted(obj.keys()):
                child = obj[key]
                if isinstance(child, h5py.Group):
                    print(f"{indent}📁 {key}/")
                    print_data_recursive(child, f"{path}/{key}", depth + 1)
                elif isinstance(child, h5py.Dataset):
                    print(f"{indent}📄 {key}:")
                    try:
                        if len(child.shape) == 0:
                            # Scalar
                            value = child[()]
                            print(f"{indent}   value: {format_value(value)}")
                        elif child.shape[0] > index:
                            # Array with enough elements
                            value = child[index]
                            if isinstance(value, np.ndarray):
                                print(f"{indent}   shape: {value.shape}")
                                print(f"{indent}   dtype: {value.dtype}")
                                if value.size <= 20:
                                    print(f"{indent}   value: {np.array2string(value, separator=', ', threshold=20)}")
                                else:
                                    print(f"{indent}   value: [first 5: {value.flat[:5]}... last 5: ...{value.flat[-5:]}]")
                            else:
                                print(f"{indent}   value: {format_value(value)}")
                        else:
                            print(f"{indent}   <index {index} out of range, max: {child.shape[0]-1}>")
                    except Exception as e:
                        print(f"{indent}   <error reading: {e}>")
    
    print_data_recursive(hdf5_file)


def print_timestamp_analysis(hdf5_file: h5py.File):
    """Analyze and print timestamp information if available."""
    print_header("TIMESTAMP ANALYSIS")
    
    # Look for timestamp-related datasets
    timestamp_paths = []
    
    def find_timestamps(obj, path=""):
        if isinstance(obj, h5py.Dataset):
            name_lower = path.lower()
            if any(t in name_lower for t in ['timestamp', 'time', '_t']):
                timestamp_paths.append(path)
        elif isinstance(obj, h5py.Group):
            for key in obj.keys():
                find_timestamps(obj[key], f"{path}/{key}" if path else key)
    
    find_timestamps(hdf5_file)
    
    if not timestamp_paths:
        print("  No timestamp datasets found.")
        return
    
    print(f"  Found {len(timestamp_paths)} timestamp-related datasets:")
    print()
    
    for path in timestamp_paths[:10]:  # Limit to first 10
        try:
            ds = hdf5_file[path]
            if len(ds.shape) > 0 and ds.shape[0] > 0:
                data = ds[:]
                if data.dtype.kind in ['i', 'u', 'f']:  # numeric
                    print(f"  📊 {path}")
                    print(f"     Length: {len(data)}")
                    print(f"     Dtype:  {ds.dtype}")
                    print(f"     Min:    {np.min(data)}")
                    print(f"     Max:    {np.max(data)}")
                    print(f"     Mean:   {np.mean(data):.2f}")
                    if len(data) > 1:
                        diffs = np.diff(data)
                        print(f"     Diff (mean): {np.mean(diffs):.2f}")
                        print(f"     Diff (std):  {np.std(diffs):.2f}")
                    print()
        except Exception as e:
            print(f"  ⚠️ {path}: Error - {e}")
    
    if len(timestamp_paths) > 10:
        print(f"  ... and {len(timestamp_paths) - 10} more timestamp datasets")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='HDF5 Reader - Inspect HDF5 trajectory files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic inspection
  python3 hdf5_reader.py trajectory.h5
  
  # Show data at specific index
  python3 hdf5_reader.py trajectory.h5 --show-data 0
  
  # Show all data (first 5 timestamps)
  python3 hdf5_reader.py trajectory.h5 --show-all-data
  
  # Limit structure depth
  python3 hdf5_reader.py trajectory.h5 --max-depth 2
  
  # Show attributes
  python3 hdf5_reader.py trajectory.h5 --show-attrs
        """
    )
    
    parser.add_argument(
        'filepath',
        type=str,
        help='Path to HDF5 file'
    )
    parser.add_argument(
        '--show-data',
        type=int,
        default=None,
        metavar='INDEX',
        help='Show data at specific timestamp index'
    )
    parser.add_argument(
        '--show-all-data',
        action='store_true',
        help='Show data for first 5 timestamps'
    )
    parser.add_argument(
        '--max-depth',
        type=int,
        default=-1,
        help='Maximum depth for structure traversal (-1 for unlimited)'
    )
    parser.add_argument(
        '--show-attrs',
        action='store_true',
        help='Show attributes for each group/dataset'
    )
    parser.add_argument(
        '--timestamps-only',
        action='store_true',
        help='Only show timestamp analysis'
    )
    parser.add_argument(
        '--summary-only',
        action='store_true',
        help='Only show summary (no detailed structure)'
    )
    
    args = parser.parse_args()
    
    # Validate file
    if not os.path.exists(args.filepath):
        print(f"❌ Error: File not found: {args.filepath}")
        sys.exit(1)
    
    try:
        with h5py.File(args.filepath, 'r') as f:
            # Always show file info
            print_file_info(args.filepath, f)
            
            # Show metadata
            print_attributes(f)
            
            if args.timestamps_only:
                print_timestamp_analysis(f)
                return
            
            # Get structure
            structure = get_hdf5_structure(f, max_depth=args.max_depth)
            
            if args.summary_only:
                print_datasets_summary(structure)
                return
            
            # Print structure
            print_structure(structure, show_attrs=args.show_attrs)
            
            # Print datasets summary
            print_datasets_summary(structure)
            
            # Print timestamp analysis
            print_timestamp_analysis(f)
            
            # Show data at index
            if args.show_data is not None:
                print_data_at_index(f, args.show_data)
            
            # Show all data (first 5)
            if args.show_all_data:
                length = get_trajectory_length(f)
                if length:
                    for i in range(min(5, length)):
                        print_data_at_index(f, i)
                        
    except Exception as e:
        print(f"❌ Error reading HDF5 file: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()

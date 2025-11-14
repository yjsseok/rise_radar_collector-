#!/usr/bin/env python3
"""Utility functions for ROS2 Bag Player"""

import os
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional
from datetime import datetime
import tkinter as tk
from tkinter import filedialog


def find_bag_files(directory: Path, recursive: bool = True) -> List[Path]:
    """
    Find all ROS2 bag directories in the given directory

    Args:
        directory: Directory to search
        recursive: Whether to search recursively

    Returns:
        List of bag directory paths
    """
    bag_dirs = []

    if recursive:
        for root, dirs, files in os.walk(directory):
            if 'metadata.yaml' in files:
                bag_dirs.append(Path(root))
    else:
        for item in directory.iterdir():
            if item.is_dir() and (item / 'metadata.yaml').exists():
                bag_dirs.append(item)

    return sorted(bag_dirs, key=lambda p: p.stat().st_mtime, reverse=True)


def find_latest_bag(directory: Path) -> Optional[Path]:
    """
    Find the most recently modified bag directory

    Args:
        directory: Directory to search

    Returns:
        Path to latest bag directory or None
    """
    bags = find_bag_files(directory)
    return bags[0] if bags else None


def format_timestamp(timestamp_ns: int) -> str:
    """
    Format nanosecond timestamp to readable string

    Args:
        timestamp_ns: Timestamp in nanoseconds

    Returns:
        Formatted timestamp string
    """
    timestamp_s = timestamp_ns / 1e9
    dt = datetime.fromtimestamp(timestamp_s)
    return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def select_folder_dialog(initial_dir: Optional[str] = None, title: str = "Select Folder") -> Optional[Path]:
    """
    Show folder selection dialog

    Args:
        initial_dir: Initial directory to show
        title: Dialog title

    Returns:
        Selected folder path or None if cancelled
    """
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)

    folder = filedialog.askdirectory(
        title=title,
        initialdir=initial_dir
    )

    root.destroy()

    return Path(folder) if folder else None


def select_file_dialog(initial_dir: Optional[str] = None, title: str = "Select Bag Directory") -> Optional[Path]:
    """
    Show file/directory selection dialog for bag files

    Args:
        initial_dir: Initial directory to show
        title: Dialog title

    Returns:
        Selected bag directory path or None if cancelled
    """
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)

    # Use askdirectory for bag selection (bags are directories)
    folder = filedialog.askdirectory(
        title=title,
        initialdir=initial_dir
    )

    root.destroy()

    if not folder:
        return None

    folder_path = Path(folder)

    # Check if it's a valid bag directory
    if (folder_path / 'metadata.yaml').exists():
        return folder_path

    # If not, try to find bags inside
    bags = find_bag_files(folder_path, recursive=False)
    if bags:
        return bags[0]

    return None


def load_config(config_path: Path) -> Dict[str, Any]:
    """
    Load YAML configuration file

    Args:
        config_path: Path to config file

    Returns:
        Configuration dictionary
    """
    if not config_path.exists():
        return get_default_config()

    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def get_default_config() -> Dict[str, Any]:
    """
    Get default configuration

    Returns:
        Default configuration dictionary
    """
    return {
        'player': {
            'default_fps': 10.0,
            'default_speed': 1.0,
            'loop_playback': False,
            'auto_play': False,
        },
        'visualization': {
            'point_size': 2.0,
            'background_color': [0.0, 0.0, 0.0, 1.0],
            'show_axes': True,
            'intensity_colormap': 'viridis',
        },
        'window': {
            'width': 1280,
            'height': 720,
            'title': 'ROS2 Bag Player - SENSR Radar',
        },
        'search': {
            'default_directory': '../radar_v3/output',
            'recursive_search': True,
            'auto_select_latest': False,
        }
    }


def get_bag_info(bag_path: Path) -> Dict[str, Any]:
    """
    Get basic information about a bag file

    Args:
        bag_path: Path to bag directory

    Returns:
        Dictionary with bag information
    """
    metadata_path = bag_path / 'metadata.yaml'

    if not metadata_path.exists():
        return {}

    with open(metadata_path, 'r') as f:
        metadata = yaml.safe_load(f)

    return {
        'path': str(bag_path),
        'name': bag_path.name,
        'metadata': metadata,
    }

"""
Path utilities for Pegasus Simulator extension.

This module provides centralized path management for the extension,
eliminating the need for fragile parent directory navigation.
"""

import os
from pathlib import Path
from typing import Optional


def get_extension_root() -> Path:
    """
    Get the root directory of the Pegasus Simulator extension.

    Returns:
        Path: The extension root directory path
    """
    # Navigate from this file to the extension root
    # /pegasus/simulator/utils/paths.py -> /
    return Path(__file__).parent.parent.parent.parent


def get_config_dir() -> Path:
    """
    Get the config directory of the extension.

    Returns:
        Path: The config directory path
    """
    return get_extension_root() / "config"


def get_vehicle_config_path(filename: str) -> Path:
    """
    Get the full path to a vehicle configuration file.

    Args:
        filename: The vehicle config filename (e.g., "quadrotor.yaml")

    Returns:
        Path: Full path to the vehicle config file
    """
    return get_config_dir() / "vehicles" / filename


def get_gimbal_config_path(filename: str) -> Path:
    """
    Get the full path to a gimbal configuration file.

    Args:
        filename: The gimbal config filename (e.g., "multi_sensor_gimbal.yaml")

    Returns:
        Path: Full path to the gimbal config file
    """
    return get_config_dir() / "gimbals" / filename


def get_motor_db_path() -> Path:
    """
    Get the full path to the motor database file.

    Returns:
        Path: Full path to the motor database YAML file
    """
    return get_config_dir() / "motors" / "motor_database.yaml"


def resolve_config_path(relative_path: str) -> Path:
    """
    Resolve a config file path, handling both absolute and relative paths.

    For relative paths starting with "config/", resolves them relative to the extension root.
    For absolute paths, returns them as-is.
    For other relative paths, treats them as relative to the current working directory.

    Args:
        relative_path: Path to resolve (e.g., "config/gimbals/multi_sensor_gimbal.yaml")

    Returns:
        Path: Resolved absolute path

    Examples:
        >>> resolve_config_path("config/gimbals/multi_sensor_gimbal.yaml")
        Path("/path/to/extension/config/gimbals/multi_sensor_gimbal.yaml")

        >>> resolve_config_path("/absolute/path/to/file.yaml")
        Path("/absolute/path/to/file.yaml")
    """
    path = Path(relative_path)

    # If it's already absolute, return as-is
    if path.is_absolute():
        return path

    # If it starts with "config/", resolve relative to extension root
    if str(path).startswith("config/"):
        # Remove "config/" prefix and resolve relative to config directory
        relative_to_config = Path(*path.parts[1:])
        return get_config_dir() / relative_to_config

    # For other relative paths, resolve relative to current working directory
    return path.resolve()


def ensure_config_file_exists(file_path: Path, file_type: str = "config") -> Path:
    """
    Ensure a config file exists and return its path.

    Args:
        file_path: Path to the config file
        file_type: Type of file for error message (e.g., "gimbal config", "vehicle config")

    Returns:
        Path: The validated file path

    Raises:
        FileNotFoundError: If the file doesn't exist
    """
    if not file_path.exists():
        raise FileNotFoundError(f"{file_type.title()} file not found: {file_path}")

    return file_path
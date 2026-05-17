# Copyright 2026 mdicicco
# SPDX-License-Identifier: Apache-2.0
"""Shared launch helper: resolve an RViz config merged with test_traj_panel overlay."""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _ensure_launch_import_path() -> None:
    launch_dir = os.path.join(get_package_share_directory('test_traj_panel'), 'launch')
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)


def resolve_rviz_config_from_share(package: str, relative_rviz_path: str) -> str:
    """Absolute path to base RViz config merged with test_traj_panel overlay."""
    _ensure_launch_import_path()
    from rviz_merge import resolve_merged_rviz_config  # noqa: PLC0415

    base_path = os.path.join(get_package_share_directory(package), relative_rviz_path)
    return resolve_merged_rviz_config(base_path)


def rviz_config_substitution(package: str, relative_rviz_path: str) -> PathJoinSubstitution:
    """Substitution for packages that still use PathJoinSubstitution in Node args.

    Prefer resolve_rviz_config_from_share() inside an OpaqueFunction when possible.
    """
    return PathJoinSubstitution([FindPackageShare(package), relative_rviz_path])

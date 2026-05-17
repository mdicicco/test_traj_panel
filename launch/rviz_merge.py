# Copyright 2026 mdicicco
# SPDX-License-Identifier: Apache-2.0
"""Merge test_traj_panel RViz overlay into a base .rviz config at launch time."""

from __future__ import annotations

import os
import tempfile
from typing import Any

from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        'PyYAML is required to merge RViz configs. Install python3-yaml.'
    ) from exc

OVERLAY_RELATIVE = os.path.join('rviz', 'overlay.rviz')
PANEL_CLASS = 'test_traj_panel/TestTrajPanel'


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f'RViz config at {path} is not a YAML mapping.')
    return data


def _write_yaml(path: str, data: dict[str, Any]) -> None:
    with open(path, 'w', encoding='utf-8') as handle:
        yaml.safe_dump(data, handle, default_flow_style=False, sort_keys=False)


def merge_rviz_dict(base: dict[str, Any], overlay: dict[str, Any]) -> dict[str, Any]:
    """Return a copy of base with overlay panels and displays merged in."""
    merged = dict(base)

    base_panels = list(merged.get('Panels') or [])
    overlay_panels = list(overlay.get('Panels') or [])
    existing_classes = {p.get('Class') for p in base_panels if isinstance(p, dict)}
    for panel in overlay_panels:
        if not isinstance(panel, dict):
            continue
        if panel.get('Class') not in existing_classes:
            base_panels.append(panel)
            existing_classes.add(panel.get('Class'))
    if base_panels:
        merged['Panels'] = base_panels

    base_vm = dict(merged.get('Visualization Manager') or {})
    overlay_vm = overlay.get('Visualization Manager') or {}
    base_displays = list(base_vm.get('Displays') or [])
    overlay_displays = list(overlay_vm.get('Displays') or [])
    existing_names = {d.get('Name') for d in base_displays if isinstance(d, dict)}
    for display in overlay_displays:
        if not isinstance(display, dict):
            continue
        if display.get('Name') not in existing_names:
            base_displays.append(display)
            existing_names.add(display.get('Name'))
    base_vm['Displays'] = base_displays
    merged['Visualization Manager'] = base_vm

    return merged


def merge_rviz_files(base_path: str, overlay_path: str, output_path: str) -> str:
    merged = merge_rviz_dict(_load_yaml(base_path), _load_yaml(overlay_path))
    _write_yaml(output_path, merged)
    return output_path


def overlay_path() -> str:
    return os.path.join(get_package_share_directory('test_traj_panel'), OVERLAY_RELATIVE)


def resolve_merged_rviz_config(base_rviz_path: str) -> str:
    """Merge overlay into base config; write a persistent file beside the base config."""
    base_rviz_path = os.path.abspath(base_rviz_path)
    if not os.path.isfile(base_rviz_path):
        raise FileNotFoundError(f'Base RViz config not found: {base_rviz_path}')

    merged_dir = os.path.join(os.path.dirname(base_rviz_path), '.merged')
    os.makedirs(merged_dir, exist_ok=True)
    output_path = os.path.join(merged_dir, os.path.basename(base_rviz_path))

    merge_rviz_files(base_rviz_path, overlay_path(), output_path)
    return output_path


def resolve_merged_rviz_config_temp(base_rviz_path: str) -> str:
    """Merge overlay into base config; write to a temporary file (for standalone configs)."""
    fd, output_path = tempfile.mkstemp(prefix='rviz_merged_', suffix='.rviz')
    os.close(fd)
    merge_rviz_files(os.path.abspath(base_rviz_path), overlay_path(), output_path)
    return output_path

#!/usr/bin/env python3
"""Debug: trace _set_navigation_switches BT resolution logic."""
import yaml, os, sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

p = Path('/ws/src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/nav2_params.yaml')
d = yaml.safe_load(p.read_text())
target_data = d

sw_entry = target_data.get('pb_navigation_switches', {})
sw = sw_entry.get('ros__parameters', {})

bt_params_entry = target_data.get('rm_behavior_tree', {})
rm_bt_params = bt_params_entry.get('ros__parameters', {})

behavior_tree_selector = sw.get('behavior_tree', '').strip()
style_file = rm_bt_params.get('style', 'rmuc_01.xml')
rm_bt_executable = rm_bt_params.get('executable', 'rm_behavior_tree')
enable_rm_bt = bool(sw.get('enable_rm_behavior_tree', False))

print(f'behavior_tree_selector: {repr(behavior_tree_selector)}')
print(f'enable_rm_bt (from switches): {enable_rm_bt}')
print(f'style_file (from rm_bt_params): {repr(style_file)}')
print(f'rm_bt_executable: {repr(rm_bt_executable)}')

if behavior_tree_selector:
    sel_lower = behavior_tree_selector.strip().lower()
    if sel_lower in {'disabled', 'none', 'nav2', 'default'}:
        enable_rm_bt = False
    else:
        enable_rm_bt = True
        style_file = behavior_tree_selector
else:
    enable_rm_bt = enable_rm_bt and bool(rm_bt_params)

print(f'Final enable_rm_bt: {enable_rm_bt}')
print(f'Final style_file: {repr(style_file)}')

default_style_file = 'rmuc_01.xml'
def _resolve_bt_style_path(style_value):
    candidate = style_value.strip() if isinstance(style_value, str) else ''
    if not candidate:
        candidate = default_style_file
    if candidate.startswith('$('):
        return candidate
    expanded = os.path.expanduser(candidate)
    if os.path.isabs(expanded):
        return expanded
    package_name = None
    relative_path = expanded
    if ':' in expanded:
        pkg, rel = expanded.split(':', 1)
        pkg = pkg.strip()
        if pkg:
            package_name = pkg
            relative_path = rel.lstrip('/') or default_style_file
    try:
        share = get_package_share_directory(package_name or 'rm_behavior_tree')
    except PackageNotFoundError:
        share = get_package_share_directory('rm_behavior_tree')
    if package_name:
        return os.path.join(share, relative_path)
    if relative_path.startswith('./') or relative_path.startswith('../'):
        return os.path.normpath(os.path.join(str(p.parent), relative_path))
    if os.path.sep in relative_path:
        return os.path.join(share, relative_path)
    return os.path.join(share, 'config', relative_path)

resolved = _resolve_bt_style_path(style_file)
print(f'Resolved style path: {resolved}')
print(f'File exists: {os.path.exists(resolved)}')

import re

with open('scripts/launch_wrapper.py', 'r') as f:
    text = f.read()

# Add imports
text = text.replace(
    'from dataclasses import dataclass\nfrom pathlib import Path\nfrom typing import Optional',
    'from dataclasses import dataclass\nfrom datetime import datetime, timedelta, timezone\nfrom pathlib import Path\nfrom typing import Callable, Optional'
)

constants = """
# 仿真启动后等待时间（秒）：用于等待 Gazebo 物理仿真稳定再拉起 SLAM/Nav。
# 可通过环境变量 GAZEBO_STARTUP_DELAY 覆盖，默认 3 秒。
GAZEBO_STARTUP_DELAY = float(os.environ.get("GAZEBO_STARTUP_DELAY", "3"))
AUTO_MAP_DIR_NAME = "maps"
AUTO_MAP_SIM_NS = os.environ.get("AUTO_MAP_SIM_NS", "/red_standard_robot1").strip() or "/red_standard_robot1"

RUNTIME_LOG_DIR_NAME = "launch_logs"
BEIJING_TZ = timezone(timedelta(hours=8), name="Asia/Shanghai")

def _beijing_timestamp() -> str:
    return datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S")

def _auto_map_prefix(ws_dir: Path, mode: str, branch_safe: str, timestamp: str | None = None) -> Path:
    maps_dir = ws_dir / AUTO_MAP_DIR_NAME / branch_safe / mode
    maps_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _beijing_timestamp()
    return maps_dir / f"map_{ts}"

def _validate_extra_launch_args(args: list[str]) -> None:
    allowed_prefixes = ("--", "__")
    for arg in args:
        if not arg:
            continue
        if arg.startswith(allowed_prefixes):
            continue
        if ":=" in arg:
            continue
        raise RuntimeError(
            f"Malformed launch argument '{arg}', expected '<name>:=<value>'"
        )

def _save_map_now(cfg, mode: str, timestamp: str, namespace: str | None = None) -> None:
    branch = _current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    map_prefix = _auto_map_prefix(cfg.ws_dir, mode, branch_safe, timestamp)
    base_env = _build_base_env(cfg)
    import shlex
    cmd = f"{base_env}; ros2 run nav2_map_server map_saver_cli -f {shlex.quote(str(map_prefix))}"
    if namespace:
        cmd += f" --ros-args -r __ns:={shlex.quote(namespace)}"
    import sys
    print(f"[{cfg.script_name}] Auto-saving map to {map_prefix}.*", file=sys.stderr)
    try:
        _run_shell(cmd)
        print(f"[{cfg.script_name}] Auto-save map done: {map_prefix}.yaml / {map_prefix}.pgm", file=sys.stderr)
    except Exception as exc:
        print(f"[{cfg.script_name}] Auto-save map failed: {exc}", file=sys.stderr)
"""
text = text.replace(
    'def _is_truthy(value: str | None) -> bool:',
    constants + '\n\ndef _is_truthy(value: str | None) -> bool:'
)

with open('scripts/launch_wrapper.py', 'w') as f:
    f.write(text)

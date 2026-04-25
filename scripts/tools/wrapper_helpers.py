#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Optional

from tools.wrapper_models import CommonConfig


# 仿真启动后等待时间（秒）：用于等待 Gazebo 物理仿真稳定再拉起 SLAM/Nav。
# 可通过环境变量 GAZEBO_STARTUP_DELAY 覆盖，默认 3 秒。
GAZEBO_STARTUP_DELAY = float(os.environ.get("GAZEBO_STARTUP_DELAY", "3"))
AUTO_MAP_DIR_NAME = "maps"
AUTO_MAP_SIM_NS = os.environ.get("AUTO_MAP_SIM_NS", "/red_standard_robot1").strip() or "/red_standard_robot1"

RUNTIME_LOG_DIR_NAME = "launch_logs"
BEIJING_TZ = timezone(timedelta(hours=8), name="Asia/Shanghai")

# 记录上次启动的前台进程组，供下次重启时精确杀干净。
PGID_FILES: dict[str, Path] = {
    "sim": Path("/tmp/ros2_nav_sim.pgid"),
    "reality": Path("/tmp/ros2_nav_reality.pgid"),
}


def wrapper_log_tag() -> str:
    return os.environ.get("WRAPPER_ENTRY_NAME", "").strip() or "wrapper_main"


def beijing_timestamp() -> str:
    return datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S")


def is_truthy(value: str | None) -> bool:
    if value is None:
        return False
    return value.strip().lower() in {"1", "true", "yes", "on"}


def in_docker() -> bool:
    if Path("/.dockerenv").exists():
        return True
    try:
        cgroup = Path("/proc/1/cgroup").read_bytes()
        return b"docker" in cgroup or b"containerd" in cgroup
    except Exception:
        return False


def slugify(text: str) -> str:
    s = text.lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = re.sub(r"^_+|_+$", "", s)
    return s or "log"


def which(cmd: str) -> Optional[str]:
    try:
        out = subprocess.check_output(["bash", "-lc", f"command -v {shlex.quote(cmd)}"], text=True)
        p = out.strip()
        return p if p else None
    except Exception:
        return None


def run_shell(cmd: str, *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["bash", "-lc", cmd], text=True, check=check)


def pgrep(pattern: str) -> list[int]:
    try:
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True)
        return [int(x) for x in out.split() if x.strip().isdigit()]
    except subprocess.CalledProcessError:
        return []


def kill_by_pattern(pattern: str, title: str, script_name: str) -> None:
    pids = pgrep(pattern)
    if not pids:
        return
    print(f"[{script_name}] Killing existing {title} pids: {' '.join(map(str, pids))}", file=sys.stderr)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in list(pids):
            try:
                os.kill(pid, 0)
            except OSError:
                continue
            try:
                os.killpg(os.getpgid(pid), sig)
            except Exception:
                try:
                    os.kill(pid, sig)
                except Exception:
                    pass
        time.sleep(0.8)
        pids = pgrep(pattern)
        if not pids:
            break
    if pids:
        print(f"[{script_name}] Warning: {title} pids still alive: {' '.join(map(str, pids))}", file=sys.stderr)


def pid_gone(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return False
    except OSError:
        return True


def cleanup_fastdds_shm() -> None:
    """清理 FastDDS 遗留共享内存，避免重启时 DDS 初始化挂死。"""
    import glob

    cleaned = 0
    for f in glob.glob("/dev/shm/fastrtps_*"):
        try:
            Path(f).unlink()
            cleaned += 1
        except Exception:
            pass
    if cleaned:
        print(f"[fastdds] Cleaned {cleaned} shm segment(s).", file=sys.stderr)


def kill_by_pgid_file(pgid_file: Path, title: str, script_name: str) -> None:
    """通过 PGID 文件终止上次启动的进程组。"""
    if not pgid_file.exists():
        return
    try:
        pgid = int(pgid_file.read_text().strip())
    except Exception:
        pgid_file.unlink(missing_ok=True)
        return

    print(f"[{script_name}] Killing {title} PGID={pgid} via pgid file ...", file=sys.stderr)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            break
        except Exception:
            pass
        time.sleep(0.8)
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            break

    try:
        pgid_file.unlink()
    except FileNotFoundError:
        pass


def kill_sim(script_name: str) -> None:
    """启动仿真前清理残留的 Gazebo 和仿真导航/SLAM 进程。"""
    kill_by_pgid_file(PGID_FILES["sim"], "sim", script_name)
    cleanup_fastdds_shm()
    for pat, title in [
        (r"bringup_sim\.launch\.py", "bringup_sim"),
        (r"ruby.*ign|ign.*gazebo|gz-server|gz-gui", "Gazebo"),
        (r"rm_navigation_simulation_launch\.py", "sim nav/SLAM"),
    ]:
        kill_by_pattern(pat, title, script_name)


def kill_reality(script_name: str) -> None:
    """启动实车前清理残留进程。"""
    kill_by_pgid_file(PGID_FILES["reality"], "reality", script_name)
    cleanup_fastdds_shm()
    for pat, title in [
        (r"rm_navigation_reality_launch\.py", "reality nav/SLAM"),
        (r"(^|/)host_sdk_sample(\s|$)", "odin_driver"),
        (r"(^|/)mid360_driver_node(\s|$)", "mid360_driver"),
        (r"(^|/)pointlio_mapping(\s|$)", "pointlio_mapping"),
        (r"(^|/)(timestamp_sync_monitor\.py|tools\.timestamp_sync_monitor)(\s|$)", "timestamp_monitor"),
        (r"(^|/)joint_state_publisher(\s|$)", "joint_state_publisher"),
        (r"(^|/)robot_state_publisher(\s|$)", "robot_state_publisher"),
        (r"(^|/)auto_aim_yaw_joint_state_bridge(\s|$)", "auto_aim_yaw_bridge"),
        (r"component_container_isolated.*nav2_container", "nav2_container"),
    ]:
        kill_by_pattern(pat, title, script_name)


def read_yaml_params(path: Path, node_key: str) -> dict:
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(path.read_text())
        node = (data or {}).get(node_key, {})
        return (node.get("ros__parameters") or {}) if isinstance(node, dict) else {}
    except Exception:
        return {}


def controller_plugin(params_file: Path) -> str:
    if not params_file.exists():
        return ""
    p = read_yaml_params(params_file, "pb_navigation_switches").get("controller_plugin", "")
    return p.strip() if isinstance(p, str) else ""


def current_branch(ws_dir: Path) -> str:
    override = os.environ.get("BUILD_PROFILE", "").strip()
    head_file = ws_dir / ".git/HEAD"
    git_branch = ""
    if head_file.exists():
        try:
            head = head_file.read_text().strip()
            if head.startswith("ref: refs/heads/"):
                git_branch = head[len("ref: refs/heads/") :]
        except Exception:
            pass

    if override:
        if is_truthy(os.environ.get("ALLOW_BUILD_PROFILE_OVERRIDE")):
            return override
        if git_branch and override != git_branch:
            print(
                f"[{wrapper_log_tag()}] Ignore stale BUILD_PROFILE='{override}', use git branch '{git_branch}'.",
                file=sys.stderr,
            )
        if git_branch:
            return git_branch
        return override

    if git_branch:
        return git_branch
    return "default"


def resolve_overlay_setup(ws_dir: Path) -> Path:
    branch = current_branch(ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)

    def _matches_branch_profile(path: Path) -> bool:
        normalized = str(path)
        token = f"/{branch_safe}/install/"
        return token in normalized or normalized.endswith(f"/{branch_safe}/install/setup.bash")

    allow_overlay_override = is_truthy(os.environ.get("ALLOW_OVERLAY_SETUP_OVERRIDE"))

    env_overlay = os.environ.get("OVERLAY_SETUP", "").strip()
    if env_overlay:
        candidate = Path(env_overlay)
        if candidate.exists() and (allow_overlay_override or _matches_branch_profile(candidate)):
            return candidate
        print(
            f"[{wrapper_log_tag()}] Ignore OVERLAY_SETUP='{env_overlay}' (branch={branch_safe}).",
            file=sys.stderr,
        )

    colcon_install_base = os.environ.get("COLCON_INSTALL_BASE", "").strip()
    if colcon_install_base:
        candidate = Path(colcon_install_base) / "setup.bash"
        if candidate.exists() and (allow_overlay_override or _matches_branch_profile(candidate)):
            return candidate
        print(
            f"[{wrapper_log_tag()}] Ignore COLCON_INSTALL_BASE='{colcon_install_base}' (branch={branch_safe}).",
            file=sys.stderr,
        )

    cache_root_env = os.environ.get("COLCON_CACHE_ROOT", "").strip()
    cache_roots = []
    if cache_root_env:
        cache_roots.append(Path(cache_root_env))
    cache_roots.extend([ws_dir / ".buildcache", ws_dir / "build/.buildcache"])

    for cache_root in cache_roots:
        candidate = cache_root / branch_safe / "install/setup.bash"
        if candidate.exists():
            return candidate

    return ws_dir / "install/setup.bash"


def enable_chassis_odometry_gt(params_file: Path) -> bool:
    if not params_file.exists():
        return True
    return bool(read_yaml_params(params_file, "pb_navigation_switches").get("enable_chassis_odometry_gt", True))


def neupan_env(
    controller_plugin_name: str,
    *,
    neupan_activate: Path | None = None,
    neupan_site_packages: Path | None = None,
    script_name: str,
) -> str:
    # NeuPAN is installed in system Python; no virtualenv activation is needed.
    if controller_plugin_name != "neupan_nav2_controller":
        return ""
    return ""


def build_base_env(cfg: CommonConfig) -> str:
    parts = []

    overlay_prefix = cfg.overlay_setup.parent
    profile_root = overlay_prefix.parent if overlay_prefix.name == "install" else overlay_prefix
    selected_install_prefix = str(overlay_prefix)
    branch_name = current_branch(cfg.ws_dir)
    branch_cache_roots = [
        str(cfg.ws_dir / ".buildcache"),
        str(cfg.ws_dir / "build/.buildcache"),
    ]
    workspace_install_roots = [
        str(cfg.ws_dir / "install"),
        str(cfg.ws_dir / "build/install"),
    ]

    def _filter_branch_cache_entries(value: str) -> str:
        kept: list[str] = []
        for entry in value.split(":"):
            item = entry.strip()
            if not item:
                continue

            if any(item.startswith(root) for root in workspace_install_roots):
                if item.startswith(selected_install_prefix):
                    kept.append(item)
                continue

            if any(item.startswith(root) for root in branch_cache_roots):
                if item.startswith(str(profile_root)):
                    kept.append(item)
                continue
            kept.append(item)
        return ":".join(kept)

    for var_name in (
        "LD_LIBRARY_PATH",
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "CMAKE_PREFIX_PATH",
        "PYTHONPATH",
    ):
        raw = os.environ.get(var_name, "")
        if not raw:
            continue
        filtered = _filter_branch_cache_entries(raw)
        if filtered:
            parts.append(f"export {var_name}={shlex.quote(filtered)}")
        else:
            parts.append(f"unset {var_name}")

    overlay_source = cfg.overlay_setup
    if cfg.overlay_setup.name == "setup.bash":
        local_setup = cfg.overlay_setup.with_name("local_setup.bash")
        if local_setup.exists():
            overlay_source = local_setup

    parts.extend(
        [
            f"source {shlex.quote(str(cfg.ros_setup))}",
            f"source {shlex.quote(str(overlay_source))}",
        ]
    )

    parts.append("unset OVERLAY_SETUP COLCON_INSTALL_BASE COLCON_BUILD_BASE COLCON_LOG_BASE")
    parts.append(f"export BUILD_PROFILE={shlex.quote(branch_name)}")

    home_dir = os.environ.get("HOME", "")
    home_path = Path(home_dir) if home_dir else None

    writable_home: Optional[Path] = None
    if home_path is not None and home_path.is_dir() and os.access(home_path, os.W_OK):
        writable_home = home_path
    else:
        home_candidates = [cfg.ws_dir / "log", cfg.ws_dir, Path("/tmp")]
        for candidate in home_candidates:
            if candidate.is_dir() and os.access(candidate, os.W_OK):
                writable_home = candidate
                break

    if writable_home is not None:
        parts.append(f"export HOME={shlex.quote(str(writable_home))}")

    ros_home_env = os.environ.get("ROS_HOME", "").strip()
    if ros_home_env:
        ros_home = Path(ros_home_env)
    else:
        ros_home = None
        ros_home_candidates = [
            cfg.ws_dir / "log/.ros",
            cfg.ws_dir / ".ros",
            Path("/tmp") / f"ros_home_{os.getuid()}",
        ]
        for candidate in ros_home_candidates:
            parent = candidate.parent
            if parent.is_dir() and os.access(parent, os.W_OK):
                ros_home = candidate
                break
        if ros_home is None:
            ros_home = Path("/tmp") / f"ros_home_{os.getuid()}"

    parts.append(f"export ROS_HOME={shlex.quote(str(ros_home))}")
    parts.append('mkdir -p "${ROS_HOME}"')

    wrapper_tz = os.environ.get("WRAPPER_TZ", "").strip() or "Asia/Shanghai"
    parts.append(f"export TZ={shlex.quote(wrapper_tz)}")

    odin_driver_source_dir = cfg.ws_dir / "src/odin_ros_driver"
    if odin_driver_source_dir.is_dir():
        parts.append(
            f"export ODIN_ROS_DRIVER_SOURCE_DIR={shlex.quote(str(odin_driver_source_dir))}"
        )

    if cfg.rcutils_logging_severity:
        parts.append(f"export RCUTILS_LOGGING_SEVERITY={shlex.quote(cfg.rcutils_logging_severity)}")

    for var_name in (
        "__NV_PRIME_RENDER_OFFLOAD",
        "__GLX_VENDOR_LIBRARY_NAME",
        "IGN_GAZEBO_RENDER_ENGINE_SERVER",
        "IGN_GAZEBO_RENDER_ENGINE_GUI",
    ):
        val = os.environ.get(var_name, "")
        if val:
            parts.append(f"export {var_name}={shlex.quote(val)}")
    return "; ".join(parts)


def pick_terminal_cmd(cfg: CommonConfig) -> str:
    if cfg.no_new_terminal:
        return ""

    requested = os.environ.get("TERMINAL_CMD", "").strip()
    if requested:
        if not which(requested):
            raise RuntimeError(f"Requested terminal '{requested}' not found")
        return requested

    if which("gnome-terminal"):
        return "gnome-terminal"
    if which("x-terminal-emulator"):
        return "x-terminal-emulator"

    return ""

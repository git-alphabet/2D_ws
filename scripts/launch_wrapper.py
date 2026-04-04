#!/usr/bin/env python3
import atexit
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Callable, Optional
from datetime import datetime, timezone, timedelta



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


def _is_truthy(value: str | None) -> bool:
    if value is None:
        return False
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _in_docker() -> bool:
    if Path("/.dockerenv").exists():
        return True
    try:
        cgroup = Path("/proc/1/cgroup").read_bytes()
        return b"docker" in cgroup or b"containerd" in cgroup
    except Exception:
        return False


def _slugify(text: str) -> str:
    s = text.lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = re.sub(r"^_+|_+$", "", s)
    return s or "log"


def _ensure_launch_arg(cmd: str, name: str, value: str) -> str:
    # If user already specified `<name>:=...`, don't override.
    # This is intentionally simple string matching (good enough for our scripts).
    if re.search(rf"(^|\s){re.escape(name)}:=", cmd):
        return cmd
    return cmd + f" {name}:={value}"


def _which(cmd: str) -> Optional[str]:
    try:
        out = subprocess.check_output(["bash", "-lc", f"command -v {shlex.quote(cmd)}"], text=True)
        p = out.strip()
        return p if p else None
    except Exception:
        return None


def _read_yaml_params(path: Path, node_key: str) -> dict:
    try:
        import yaml  # type: ignore
        data = yaml.safe_load(path.read_text())
        node = (data or {}).get(node_key, {})
        return (node.get("ros__parameters") or {}) if isinstance(node, dict) else {}
    except Exception:
        return {}


def _controller_plugin(params_file: Path) -> str:
    if not params_file.exists():
        return ""
    p = _read_yaml_params(params_file, "pb_navigation_switches").get("controller_plugin", "")
    return p.strip() if isinstance(p, str) else ""


def _current_branch(ws_dir: Path) -> str:
    override = os.environ.get("BUILD_PROFILE", "").strip()
    if override:
        return override

    head_file = ws_dir / ".git/HEAD"
    if head_file.exists():
        try:
            head = head_file.read_text().strip()
            if head.startswith("ref: refs/heads/"):
                return head[len("ref: refs/heads/"):]
        except Exception:
            pass
    return "default"


def _resolve_overlay_setup(ws_dir: Path) -> Path:
    env_overlay = os.environ.get("OVERLAY_SETUP", "").strip()
    if env_overlay:
        return Path(env_overlay)

    colcon_install_base = os.environ.get("COLCON_INSTALL_BASE", "").strip()
    if colcon_install_base:
        candidate = Path(colcon_install_base) / "setup.bash"
        if candidate.exists():
            return candidate

    branch = _current_branch(ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)

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


def _enable_chassis_odometry_gt(params_file: Path) -> bool:
    if not params_file.exists():
        return True
    return bool(_read_yaml_params(params_file, "pb_navigation_switches").get("enable_chassis_odometry_gt", True))


def _neupan_env(controller_plugin: str, *, neupan_activate: Path,
                neupan_site_packages: Path, script_name: str) -> str:
    # Returns a shell snippet; only activates for neupan_nav2_controller.
    if controller_plugin != "neupan_nav2_controller":
        return ""

    if not neupan_activate.exists():
        raise RuntimeError(f"NeuPAN virtualenv not found at {neupan_activate}")

    parts = [f"source {shlex.quote(str(neupan_activate))}"]
    if neupan_site_packages.is_dir():
        # Use double-quoted assignment so $PYTHONPATH expands correctly at runtime.
        # (A backslash-escaped \$PYTHONPATH would be treated as a literal string,
        #  overwriting PYTHONPATH with '$PYTHONPATH:/path' instead of the real value.)
        parts.append(f'export PYTHONPATH="$PYTHONPATH:{neupan_site_packages}"')

    return "; ".join(parts)


@dataclass
class CommonConfig:
    script_name: str
    ws_dir: Path
    ros_setup: Path
    overlay_setup: Path
    params_file: Path
    no_new_terminal: bool
    terminal_cmd: str
    kill_existing: bool
    rcutils_logging_severity: Optional[str] = None
    log_type: str = "nav"


class BackgroundGroup:
    def __init__(self, script_name: str):
        self._script_name = script_name
        self._pids: list[int] = []

    def add(self, pid: int) -> None:
        self._pids.append(pid)

    def cleanup(self) -> None:
        if not self._pids:
            return
        print(f"[{self._script_name}] Cleaning up background processes...", file=sys.stderr)

        # First try TERM, then KILL.
        for sig in (signal.SIGTERM, signal.SIGKILL):
            for pid in list(self._pids):
                try:
                    os.kill(pid, 0)
                except OSError:
                    continue
                try:
                    os.killpg(pid, sig)
                except Exception:
                    try:
                        os.kill(pid, sig)
                    except Exception:
                        pass
            time.sleep(1.0)


def _build_base_env(cfg: CommonConfig) -> str:
    parts = [
        f"source {shlex.quote(str(cfg.ros_setup))}",
        f"source {shlex.quote(str(cfg.overlay_setup))}",
    ]

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
        ros_home_candidates = [cfg.ws_dir / "log/.ros", cfg.ws_dir / ".ros", Path("/tmp") / f"ros_home_{os.getuid()}"]
        for candidate in ros_home_candidates:
            parent = candidate.parent
            if parent.is_dir() and os.access(parent, os.W_OK):
                ros_home = candidate
                break
        if ros_home is None:
            ros_home = Path("/tmp") / f"ros_home_{os.getuid()}"

    parts.append(f"export ROS_HOME={shlex.quote(str(ros_home))}")
    parts.append("mkdir -p \"${ROS_HOME}\"")

    if cfg.rcutils_logging_severity:
        parts.append(f"export RCUTILS_LOGGING_SEVERITY={shlex.quote(cfg.rcutils_logging_severity)}")
    # 显式传递 NVIDIA / Gazebo 渲染变量，确保 gnome-terminal 新窗口和后台进程都能调用 GPU
    for _var in ("__NV_PRIME_RENDER_OFFLOAD", "__GLX_VENDOR_LIBRARY_NAME",
                 "IGN_GAZEBO_RENDER_ENGINE_SERVER", "IGN_GAZEBO_RENDER_ENGINE_GUI"):
        _val = os.environ.get(_var, "")
        if _val:
            parts.append(f"export {_var}={shlex.quote(_val)}")
    return "; ".join(parts)


def _pick_terminal_cmd(cfg: CommonConfig) -> str:
    if cfg.no_new_terminal:
        return ""

    requested = os.environ.get("TERMINAL_CMD", "").strip()
    if requested:
        if not _which(requested):
            raise RuntimeError(f"Requested terminal '{requested}' not found")
        return requested

    if _which("gnome-terminal"):
        return "gnome-terminal"
    if _which("x-terminal-emulator"):
        return "x-terminal-emulator"

    return ""  # fallback to single-terminal


def _run_shell(cmd: str, *, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["bash", "-lc", cmd], text=True, check=check)


def _pgrep(pattern: str) -> list[int]:
    try:
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True)
        return [int(x) for x in out.split() if x.strip().isdigit()]
    except subprocess.CalledProcessError:
        return []


def _kill_by_pattern(pattern: str, title: str, script_name: str) -> None:
    pids = _pgrep(pattern)
    if not pids:
        return
    print(f"[{script_name}] Killing existing {title} pids: {' '.join(map(str, pids))}", file=sys.stderr)
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in list(pids):
            try:
                os.kill(pid, 0)  # 检查进程是否还存在
            except OSError:
                continue
            try:
                os.killpg(os.getpgid(pid), sig)  # 杀整个进程组（含子节点）
            except Exception:
                try:
                    os.kill(pid, sig)
                except Exception:
                    pass
        time.sleep(0.8)
        pids = _pgrep(pattern)
        if not pids:
            break
    if pids:
        print(f"[{script_name}] Warning: {title} pids still alive: {' '.join(map(str, pids))}", file=sys.stderr)


def _pid_gone(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return False
    except OSError:
        return True


# ── PGID 文件路径：记录上次启动的前台进程组，供下次重启时精确杀干净 ────────────
_PGID_FILES: dict[str, Path] = {
    "sim":     Path("/tmp/ros2_nav_sim.pgid"),
    "reality": Path("/tmp/ros2_nav_reality.pgid"),
}


def _cleanup_fastdds_shm() -> None:
    """清理 FastDDS 遗留的共享内存段，避免进程重启时 DDS 初始化挂死。"""
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


def _kill_by_pgid_file(pgid_file: Path, title: str, script_name: str) -> None:
    """通过 PGID 文件直接终止上次启动的整个进程组（精确，无 pattern 依赖）。"""
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
            break   # 进程组已不存在
        except Exception:
            pass
        time.sleep(0.8)
        try:
            os.killpg(pgid, 0)  # 检查是否还活着
        except ProcessLookupError:
            break
    try:
        pgid_file.unlink()
    except FileNotFoundError:
        pass


def _start_watchdog(cfg: CommonConfig, topics: list[tuple[str, float]], bg: "BackgroundGroup") -> None:
    """启动话题频率 watchdog（后台进程）。仅当 ENABLE_WATCHDOG=1 时生效。
    topics: [(topic_name, min_expected_hz), ...]
    每 10 秒轮询一次，低于阈值时打 WARN 日志。
    """
    if not _is_truthy(os.environ.get("ENABLE_WATCHDOG")):
        return

    checks = " ".join(
        f"{shlex.quote(t)}:{hz}" for t, hz in topics
    )
    base_env = _build_base_env(cfg)
    branch = _current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe / cfg.log_type
    log_dir.mkdir(parents=True, exist_ok=True)
    bj_tz = timezone(timedelta(hours=8))
    ts = datetime.now(bj_tz).strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"{Path(cfg.script_name).stem}_watchdog_{ts}.log"

    # Python one-liner: 每 10s 用 ros2 topic hz --window 10 轮询一次
    py_script = r"""
import subprocess, time, sys
checks = []
for item in sys.argv[1:]:
    t, hz = item.rsplit(':', 1)
    checks.append((t, float(hz)))
while True:
    for topic, min_hz in checks:
        try:
            out = subprocess.check_output(
                ['ros2', 'topic', 'hz', '--window', '5', topic],
                timeout=6, text=True, stderr=subprocess.DEVNULL
            )
            line = [l for l in out.splitlines() if 'average rate' in l.lower()]
            if line:
                hz = float(line[0].split(':')[1].strip().split()[0])
                if hz < min_hz:
                    print(f'[watchdog] WARN {topic}: {hz:.1f} Hz < {min_hz} Hz', flush=True)
                else:
                    print(f'[watchdog] OK   {topic}: {hz:.1f} Hz', flush=True)
            else:
                print(f'[watchdog] WARN {topic}: no data', flush=True)
        except subprocess.TimeoutExpired:
            print(f'[watchdog] WARN {topic}: timeout (no publisher?)', flush=True)
        except Exception as e:
            print(f'[watchdog] ERR  {topic}: {e}', flush=True)
    time.sleep(10)
"""
    cmd = f"{base_env}; python3 -c {shlex.quote(py_script)} {checks} 2>&1 | tee -a {shlex.quote(str(log_file))}"
    print(f"[{cfg.script_name}] (watchdog) monitoring {len(topics)} topics -> {log_file}", file=sys.stderr)
    p = subprocess.Popen(["bash", "-lc", cmd], preexec_fn=os.setsid)
    bg.add(p.pid)


def _kill_sim(script_name: str) -> None:
    """启动仿真前清理残留的 Gazebo 和仿真导航/SLAM 进程。"""
    _kill_by_pgid_file(_PGID_FILES["sim"], "sim", script_name)
    _cleanup_fastdds_shm()
    for pat, title in [
        (r"bringup_sim\.launch\.py",               "bringup_sim"),
        (r"ruby.*ign|ign.*gazebo|gz-server|gz-gui", "Gazebo"),
        (r"rm_navigation_simulation_launch\.py",   "sim nav/SLAM"),
    ]:
        _kill_by_pattern(pat, title, script_name)


def _kill_reality(script_name: str) -> None:
    """启动实车前清理残留进程。"""
    _kill_by_pgid_file(_PGID_FILES["reality"], "reality", script_name)
    _cleanup_fastdds_shm()
    for pat, title in [
        (r"rm_navigation_reality_launch\.py",            "reality nav/SLAM"),
        (r"(^|/)joint_state_publisher(\s|$)",            "joint_state_publisher"),
        (r"(^|/)robot_state_publisher(\s|$)",            "robot_state_publisher"),
        (r"(^|/)auto_aim_yaw_joint_state_bridge(\s|$)",  "auto_aim_yaw_bridge"),
        (r"component_container_isolated.*nav2_container", "nav2_container"),
    ]:
        _kill_by_pattern(pat, title, script_name)


def _launch_in_terminal(cfg: CommonConfig, title: str, command: str, extra_env: str, *, background: bool = False, bg: Optional[BackgroundGroup] = None, pgid_file: Optional[Path] = None) -> None:
    base_env = _build_base_env(cfg)

    full_cmd = f"cd {shlex.quote(str(cfg.ws_dir))}; {base_env}"
    if extra_env:
        full_cmd += f"; {extra_env}"
    full_cmd += f"; {command}"

    # Background processes: always run detached (log to file), regardless of terminal mode.
    branch = _current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe / cfg.log_type
    bj_tz = timezone(timedelta(hours=8))
    ts = datetime.now(bj_tz).strftime("%Y%m%d_%H%M%S")
    slug = _slugify(title)
    log_file_name = f"{Path(cfg.script_name).stem}_{slug}_{ts}.log"

    if background:
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / log_file_name

        print(f"[{cfg.script_name}] (background) {title} -> {log_file}", file=sys.stderr)
        # Start in its own session so we can kill the whole group (when tracked).
        p = subprocess.Popen(
            ["bash", "-lc", full_cmd],
            stdout=open(log_file, "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        if bg is not None:
            bg.add(p.pid)
        return

    if cfg.no_new_terminal or not cfg.terminal_cmd:
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / log_file_name

        print(f"[{cfg.script_name}] (single-terminal) {title} (foreground)", file=sys.stderr)
        print(f"[{cfg.script_name}] Log: {log_file}", file=sys.stderr)
        # 在独立 session（setsid）中启动，使 PID==PGID，方便下次启动前按 PGID 精确杀干净。
        # tee 通过 bash 非交互管道继承同一 PGID，killpg 可一并终止。
        wrap_cmd = f"{full_cmd} 2>&1 | tee -a {shlex.quote(str(log_file))}"
        p = subprocess.Popen(["bash", "-lc", wrap_cmd], preexec_fn=os.setsid)
        if pgid_file:
            try:
                pgid_file.write_text(str(p.pid))
            except Exception:
                pass
        _child_pgid = p.pid

        # 优雅关闭超时（秒）：SIGINT 后等待这么久，超时或第二次 Ctrl+C 则 SIGKILL。
        _shutdown_timeout = int(os.environ.get("SHUTDOWN_TIMEOUT", "15"))
        _shutdown_requested = [False]   # mutable cell 供嵌套函数修改

        def _force_kill() -> None:
            try:
                os.killpg(_child_pgid, signal.SIGKILL)
            except Exception:
                pass

        def _forward_signal(signum: int, _frame: object) -> None:
            if _shutdown_requested[0]:
                # 第二次 Ctrl+C：立即强杀
                print(f"\n[{cfg.script_name}] Force killing (SIGKILL)...", file=sys.stderr)
                _force_kill()
                return
            _shutdown_requested[0] = True
            print(
                f"\n[{cfg.script_name}] Shutting down (timeout {_shutdown_timeout}s)..."
                " Press Ctrl+C again to force kill.",
                file=sys.stderr,
            )
            try:
                os.killpg(_child_pgid, signum)
            except Exception:
                pass

        prev_sigint  = signal.signal(signal.SIGINT,  _forward_signal)  # type: ignore[arg-type]
        prev_sigterm = signal.signal(signal.SIGTERM, _forward_signal)  # type: ignore[arg-type]
        try:
            # 分段 poll：每秒检查一次，超时后强杀
            deadline = time.monotonic() + _shutdown_timeout
            while True:
                try:
                    p.wait(timeout=1.0)
                    break   # 正常退出
                except subprocess.TimeoutExpired:
                    pass
                if _shutdown_requested[0] and time.monotonic() > deadline:
                    print(
                        f"[{cfg.script_name}] Shutdown timeout ({_shutdown_timeout}s), force killing...",
                        file=sys.stderr,
                    )
                    _force_kill()
                    p.wait()
                    break
        finally:
            signal.signal(signal.SIGINT,  prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)
            if pgid_file:
                try:
                    pgid_file.unlink()
                except FileNotFoundError:
                    pass
        return

    # Multi-terminal mode.
    term = cfg.terminal_cmd
    keep_shell = f"{full_cmd}; exec bash"
    if term == "gnome-terminal":
        _run_shell(f"gnome-terminal --title={shlex.quote(title)} -- bash -c {shlex.quote(keep_shell)}")
        return
    if term == "x-terminal-emulator":
        # xterm 是前台阻塞进程（不像 gnome-terminal 会 fork daemon），
        # 必须用 Popen 非阻塞启动，否则第二个窗口永远不会打开。
        # 直接调用 xterm（而非 x-terminal-emulator 包装器），可传入样式参数。
        subprocess.Popen(
            [
                "xterm",
                "-T", title,
                "-u8",                              # UTF-8 模式，中文正常显示
                "-bg", "#1e1e2e",                    # 深色背景
                "-fg", "#cdd6f4",                    # 浅色前景
                "-fa", "Monospace",                  # 主字体，CJK 由 fontconfig 自动 fallback
                "-fs", "13",                         # 字号 pt
                "-geometry", "220x55",               # 列×行
                "-sl", "5000",                       # 滚动缓冲行数
                "-e", "bash", "-lc", keep_shell,
            ],
            preexec_fn=os.setsid,
        )
        return
    # fallback: 其他终端模拟器同样非阻塞处理
    subprocess.Popen(
        [term, "-T", title, "-e", "bash", "-lc", keep_shell],
        preexec_fn=os.setsid,
    )


def _wait_for_background(bg: BackgroundGroup, script_name: str) -> int:
    """阻塞直到所有后台进程退出（multi-terminal 模式下防止 atexit 过早 kill Gazebo）。"""
    if not bg._pids:
        return 0
    print(f"[{script_name}] Waiting for background pids: {bg._pids}. Ctrl+C to stop all.", file=sys.stderr)
    try:
        while not all(_pid_gone(p) for p in bg._pids):
            time.sleep(1.0)
    except (KeyboardInterrupt, SystemExit):
        pass
    return 0


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("usage: launch_wrapper.py <mode> [extra args...]", file=sys.stderr)
        return 1

    script_name = Path(argv[0]).name
    mode = argv[1]
    extra_args = argv[2:]

    VALID_MODES = {"sim_mapping", "sim_nav", "reality_mapping", "reality_navigation"}
    if mode not in VALID_MODES:
        print(f"[{script_name}] Unknown mode '{mode}'. Valid: {sorted(VALID_MODES)}", file=sys.stderr)
        return 1

    # ws_dir: 优先 WS_DIR 环境变量，否则脚本所在目录的上一级
    _ws_env = os.environ.get("WS_DIR", "").strip()
    ws_dir = Path(_ws_env).expanduser() if _ws_env else Path(__file__).resolve().parent.parent

    ros_setup = Path(os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash"))
    overlay_setup = _resolve_overlay_setup(ws_dir)

    for p, label in [(ros_setup, "ROS setup"), (overlay_setup, "workspace overlay")]:
        if not p.exists():
            print(f"[{script_name}] Missing {label}: {p}", file=sys.stderr)
            return 1

    kill_existing     = _is_truthy(os.environ.get("KILL_EXISTING", "1"))
    no_new_terminal_e = os.environ.get("NO_NEW_TERMINAL")
    no_new_terminal   = _is_truthy(no_new_terminal_e) or (_in_docker() and not no_new_terminal_e)

    is_sim = mode.startswith("sim_")
    log_type = "slam" if "mapping" in mode else "nav"
    params_env_key  = "SIM_PARAMS_FILE" if is_sim else "REALITY_PARAMS_FILE"
    params_default  = (ws_dir / "src/gxu2026_sentry_nav/gxu2026_nav_bringup/config"
                       / ("simulation" if is_sim else "reality") / "nav2_params.yaml")
    params_file = Path(os.environ.get(params_env_key, str(params_default)))

    cfg = CommonConfig(
        script_name=script_name,
        ws_dir=ws_dir,
        ros_setup=ros_setup,
        overlay_setup=overlay_setup,
        params_file=params_file,
        no_new_terminal=no_new_terminal,
        terminal_cmd="",
        kill_existing=kill_existing,
        rcutils_logging_severity=os.environ.get("RCUTILS_LOGGING_SEVERITY"),
        log_type=log_type,
    )
    cfg.terminal_cmd = _pick_terminal_cmd(cfg)
    if not cfg.terminal_cmd:
        cfg.no_new_terminal = True

    # NeuPAN 虚拟环境片段
    controller_plugin = _controller_plugin(cfg.params_file)
    if controller_plugin:
        print(f"[{script_name}] controller_plugin='{controller_plugin}'", file=sys.stderr)
    else:
        print(f"[{script_name}] controller_plugin unset; NeuPAN venv will not be activated.", file=sys.stderr)

    try:
        neupan_env = _neupan_env(
            controller_plugin,
            neupan_activate=Path(os.environ.get("NEUPAN_ACTIVATE",
                                                 str(ws_dir / "neupan_env/bin/activate"))),
            neupan_site_packages=Path(os.environ.get("NEUPAN_SITE_PACKAGES",
                                                      str(ws_dir / "neupan_env/lib/python3.10/site-packages"))),
            script_name=script_name,
        )
    except Exception as exc:
        print(f"[{script_name}] {exc}", file=sys.stderr)
        return 1

    # ── 启动前清理残留进程 ────────────────────────────────────────────────────
    if kill_existing:
        if is_sim:
            _kill_sim(script_name)
        else:
            _kill_reality(script_name)

    # ── 仿真模式：Gazebo (后台) + SLAM/Nav (前台) ─────────────────────────────
    if is_sim:
        bg = BackgroundGroup(script_name)
        atexit.register(bg.cleanup)

        def _sig(_signum, _frame):
            bg.cleanup()
            raise SystemExit(130)

        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        enable_gt = _enable_chassis_odometry_gt(cfg.params_file)
        print(f"[{script_name}] enable_chassis_odometry_gt={'true' if enable_gt else 'false'}", file=sys.stderr)

        gazebo_cmd = os.environ.get("GAZEBO_CMD", "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py")
        if "enable_chassis_odometry_gt:=" not in gazebo_cmd:
            gazebo_cmd += f" enable_chassis_odometry_gt:={'true' if enable_gt else 'false'}"
        if "use_gui:=" not in gazebo_cmd:
            _headless = (_is_truthy(os.environ.get("GAZEBO_HEADLESS"))
                         or not os.environ.get("DISPLAY", "").strip()
                         or cfg.no_new_terminal)
            if _headless:
                gazebo_cmd = _ensure_launch_arg(gazebo_cmd, "use_gui", "false")
                print(f"[{script_name}] Headless Gazebo (DISPLAY={os.environ.get('DISPLAY', '(unset)')!r})", file=sys.stderr)

        if mode == "sim_mapping":
            ros_cmd  = os.environ.get("SLAM_CMD", "ros2 launch gxu2026_nav_bringup rm_navigation_simulation_launch.py slam:=True")
            fg_title = "SLAM"
        else:  # sim_nav
            ros_cmd  = os.environ.get("NAV_CMD", "ros2 launch gxu2026_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False")
            fg_title = "Nav"

        if extra_args:
            ros_cmd += " " + " ".join(map(shlex.quote, extra_args))

        _is_multi_terminal = bool(cfg.terminal_cmd) and not cfg.no_new_terminal

        if _is_multi_terminal:
            # 多终端模式：Gazebo 和 SLAM/Nav 各弹一个 gnome 窗口，用户可直接在窗口内 Ctrl+C 关闭。
            # wrapper 弹完两个窗口后直接退出，不需要 atexit/BackgroundGroup。
            _launch_in_terminal(cfg, "Gazebo Sim", gazebo_cmd, "")
            time.sleep(1.0)
            _launch_in_terminal(cfg, fg_title, ros_cmd, neupan_env)
            return 0

        # 单终端/Docker 模式：Gazebo 后台 Popen 写日志，SLAM/Nav 前台阻塞，Ctrl+C 统一清理。
        _launch_in_terminal(cfg, "Gazebo Sim", gazebo_cmd, "", background=True, bg=bg)
        time.sleep(1.0)
        _start_watchdog(cfg, [
            ("/registered_scan", 5.0),
            ("/Odometry", 10.0),
        ], bg)
        _launch_in_terminal(cfg, fg_title, ros_cmd, neupan_env, pgid_file=_PGID_FILES["sim"])
        return 0

    # ── 实车模式：SLAM/Nav (前台) ─────────────────────────────────────────────
    if mode == "reality_mapping":
        ros_cmd  = os.environ.get("MAPPING_CMD",
                                  "ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True")
        fg_title = "Reality Mapping"
    else:  # reality_navigation
        ros_cmd  = os.environ.get("NAVIGATION_CMD",
                                  "ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True")
        fg_title = "Reality Navigation"

    if _is_truthy(os.environ.get("ENABLE_WATCHDOG")):
        _wd_bg = BackgroundGroup(script_name)
        atexit.register(_wd_bg.cleanup)
        _start_watchdog(cfg, [
            ("/registered_scan", 5.0),
            ("/Odometry", 10.0),
            ("/scan", 5.0),
        ], _wd_bg)

    _launch_in_terminal(cfg, fg_title, ros_cmd, neupan_env, pgid_file=_PGID_FILES["reality"])
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[{Path(sys.argv[0]).name}] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)

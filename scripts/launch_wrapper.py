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
from pathlib import Path
from typing import Optional


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


def _kill_sim(script_name: str) -> None:
    """启动仿真前清理残留的 Gazebo 和仿真导航/SLAM 进程。"""
    for pat, title in [
        (r"bringup_sim\.launch\.py",               "bringup_sim"),
        (r"ruby.*ign|ign.*gazebo|gz-server|gz-gui", "Gazebo"),
        (r"rm_navigation_simulation_launch\.py",   "sim nav/SLAM"),
    ]:
        _kill_by_pattern(pat, title, script_name)


def _kill_reality(script_name: str) -> None:
    """启动实车前清理残留进程。"""
    for pat, title in [
        (r"rm_navigation_reality_launch\.py",            "reality nav/SLAM"),
        (r"(^|/)joint_state_publisher(\s|$)",            "joint_state_publisher"),
        (r"(^|/)robot_state_publisher(\s|$)",            "robot_state_publisher"),
        (r"(^|/)auto_aim_yaw_joint_state_bridge(\s|$)",  "auto_aim_yaw_bridge"),
        (r"component_container_isolated.*nav2_container", "nav2_container"),
    ]:
        _kill_by_pattern(pat, title, script_name)


def _launch_in_terminal(cfg: CommonConfig, title: str, command: str, extra_env: str, *, background: bool = False, bg: Optional[BackgroundGroup] = None) -> None:
    base_env = _build_base_env(cfg)

    full_cmd = f"cd {shlex.quote(str(cfg.ws_dir))}; {base_env}"
    if extra_env:
        full_cmd += f"; {extra_env}"
    full_cmd += f"; {command}"

    # Background processes: always run detached (log to file), regardless of terminal mode.
    if background:
        log_dir = cfg.ws_dir / "log"
        log_dir.mkdir(parents=True, exist_ok=True)
        slug = _slugify(title)
        log_file = log_dir / f"{Path(cfg.script_name).stem}_{slug}.log"

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
        log_dir = cfg.ws_dir / "log"
        log_dir.mkdir(parents=True, exist_ok=True)
        slug = _slugify(title)
        log_file = log_dir / f"{Path(cfg.script_name).stem}_{slug}.log"

        print(f"[{cfg.script_name}] (single-terminal) {title} (foreground)", file=sys.stderr)
        print(f"[{cfg.script_name}] Log: {log_file}", file=sys.stderr)
        # tee output
        _run_shell(f"{full_cmd} 2>&1 | tee -a {shlex.quote(str(log_file))}")
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

    ros_setup     = Path(os.environ.get("ROS_SETUP",     "/opt/ros/humble/setup.bash"))
    overlay_setup = Path(os.environ.get("OVERLAY_SETUP", str(ws_dir / "install/setup.bash")))

    for p, label in [(ros_setup, "ROS setup"), (overlay_setup, "workspace overlay")]:
        if not p.exists():
            print(f"[{script_name}] Missing {label}: {p}", file=sys.stderr)
            return 1

    kill_existing     = _is_truthy(os.environ.get("KILL_EXISTING", "1"))
    no_new_terminal_e = os.environ.get("NO_NEW_TERMINAL")
    no_new_terminal   = _is_truthy(no_new_terminal_e) or (_in_docker() and not no_new_terminal_e)

    is_sim = mode.startswith("sim_")
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
        _launch_in_terminal(cfg, fg_title, ros_cmd, neupan_env)
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

    _launch_in_terminal(cfg, fg_title, ros_cmd, neupan_env)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[{Path(sys.argv[0]).name}] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)

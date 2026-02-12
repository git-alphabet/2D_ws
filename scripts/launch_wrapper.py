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


def _foxglove_bridge_command() -> str:
    # Configure via env vars (strings):
    # - START_FOXGLOVE=0 to disable
    # - FOXGLOVE_PORT / FOXGLOVE_ADDRESS
    # - FOXGLOVE_TOPIC_WHITELIST / FOXGLOVE_PARAM_WHITELIST / FOXGLOVE_SERVICE_WHITELIST
    # - FOXGLOVE_DEBUG / FOXGLOVE_TLS / FOXGLOVE_CERTFILE / FOXGLOVE_KEYFILE
    # - FOXGLOVE_SEND_BUFFER_LIMIT / FOXGLOVE_USE_SIM_TIME / FOXGLOVE_CAPABILITIES
    def pick(key: str, default: str) -> str:
        v = os.environ.get(key, "").strip()
        return v if v else default

    args: dict[str, str] = {
        "port": pick("FOXGLOVE_PORT", "8765"),
        "address": pick("FOXGLOVE_ADDRESS", "0.0.0.0"),
        "debug": pick("FOXGLOVE_DEBUG", "false"),
        "tls": pick("FOXGLOVE_TLS", "false"),
        "certfile": pick("FOXGLOVE_CERTFILE", ""),
        "keyfile": pick("FOXGLOVE_KEYFILE", ""),
        # NOTE: These values include brackets/quotes; always quote to avoid shell globbing.
        "topic_whitelist": pick("FOXGLOVE_TOPIC_WHITELIST", "['.*']"),
        "param_whitelist": pick("FOXGLOVE_PARAM_WHITELIST", "['.*']"),
        "service_whitelist": pick("FOXGLOVE_SERVICE_WHITELIST", "['.*']"),
        "send_buffer_limit": pick("FOXGLOVE_SEND_BUFFER_LIMIT", "10000000"),
        "use_sim_time": pick("FOXGLOVE_USE_SIM_TIME", "false"),
    }

    capabilities = os.environ.get("FOXGLOVE_CAPABILITIES", "").strip()
    if capabilities:
        args["capabilities"] = capabilities

    parts = ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml"]
    for k, v in args.items():
        # Avoid passing empty-string values through a shell-parsed command.
        # Example: certfile:='' becomes certfile:= after shlex splitting, which ros2 launch rejects.
        if k in {"certfile", "keyfile"} and not v:
            continue
        parts.append(f"{k}:={shlex.quote(v)}")

    return " ".join(parts)


def _which(cmd: str) -> Optional[str]:
    try:
        out = subprocess.check_output(["bash", "-lc", f"command -v {shlex.quote(cmd)}"], text=True)
        p = out.strip()
        return p if p else None
    except Exception:
        return None


def _read_yaml(path: Path) -> dict:
    try:
        import yaml  # type: ignore
    except Exception:
        return {}

    try:
        data = yaml.safe_load(path.read_text())
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _get_ros_params(root: dict, key: str) -> dict:
    node = root.get(key)
    if not isinstance(node, dict):
        return {}
    params = node.get("ros__parameters")
    return params if isinstance(params, dict) else {}


def _controller_plugin(params_file: Path) -> str:
    if not params_file.exists():
        return ""
    root = _read_yaml(params_file)
    switches = _get_ros_params(root, "pb_navigation_switches")
    plugin = switches.get("controller_plugin")
    return plugin.strip() if isinstance(plugin, str) else ""


def _enable_chassis_odometry_gt(params_file: Path) -> bool:
    # default true (matches existing scripts)
    if not params_file.exists():
        return True
    root = _read_yaml(params_file)
    switches = _get_ros_params(root, "pb_navigation_switches")
    v = switches.get("enable_chassis_odometry_gt", True)
    return bool(v)


def _bt_report(params_file: Path, script_name: str) -> str:
    if not params_file.exists():
        return ""
    root = _read_yaml(params_file)
    if not root:
        return ""

    switches = _get_ros_params(root, "pb_navigation_switches")
    rm_bt = _get_ros_params(root, "rm_behavior_tree")

    selector = switches.get("behavior_tree") if isinstance(switches, dict) else None
    selector = selector.strip() if isinstance(selector, str) else ""
    enable_flag = bool(switches.get("enable_rm_behavior_tree", False)) if isinstance(switches, dict) else False
    style = rm_bt.get("style", "rmuc_01.xml") if isinstance(rm_bt, dict) else "rmuc_01.xml"

    enabled = False
    reason = ""
    if selector:
        lowered = selector.lower()
        if lowered in {"disabled", "none", "nav2", "default"}:
            enabled = False
            reason = selector
        else:
            enabled = True
            reason = selector
            style = selector
    else:
        enabled = enable_flag and bool(rm_bt)
        reason = f"enable_rm_behavior_tree={'true' if enable_flag else 'false'}"

    prefix = f"[{script_name}]"
    if not enabled:
        return f"{prefix} Behavior tree disabled (selector='{reason}')"
    return f"{prefix} Behavior tree enabled; style='{style}'"


def _neupan_env(controller_plugin: str, *, neupan_activate: Path, neupan_site_packages: Path, neupan_model_setup: str, script_name: str) -> str:
    # Returns a shell snippet.
    if controller_plugin not in {"neupan_nav2_controller", "neupan_slam_controller"}:
        return ""

    if not neupan_activate.exists():
        raise RuntimeError(f"NeuPAN virtualenv not found at {neupan_activate}")

    parts = [f"source {shlex.quote(str(neupan_activate))}"]
    if neupan_site_packages.is_dir():
        parts.append(f"export PYTHONPATH=\$PYTHONPATH:{shlex.quote(str(neupan_site_packages))}")

    if neupan_model_setup:
        model_setup = Path(os.path.expanduser(neupan_model_setup))
        if model_setup.exists():
            parts.append(f"source {shlex.quote(str(model_setup))}")
        else:
            print(f"[{script_name}] Warning: {model_setup} not found; skipping model setup.", file=sys.stderr)

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
    strict: bool
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
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            pass
    time.sleep(0.3)
    pids2 = _pgrep(pattern)
    if pids2:
        print(f"[{script_name}] Force-killing remaining {title} pids: {' '.join(map(str, pids2))}", file=sys.stderr)
        for pid in pids2:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass


def _check_conflicts(script_name: str, strict: bool) -> None:
    patterns = [
        r"(^|/)joint_state_publisher(\\s|$)",
        r"(^|/)robot_state_publisher(\\s|$)",
        r"(^|/)auto_aim_yaw_joint_state_bridge(\\s|$)",
    ]
    titles = [
        "joint_state_publisher",
        "robot_state_publisher",
        "auto_aim_yaw_joint_state_bridge",
    ]

    any_hit = False
    for pat, title in zip(patterns, titles, strict=False):
        pids = _pgrep(pat)
        if pids:
            any_hit = True
            print(f"[{script_name}] Warning: Detected running {title} pids: {' '.join(map(str, pids))}", file=sys.stderr)

    if any_hit and strict:
        raise RuntimeError("Conflicting processes are running. Set KILL_EXISTING=1 or stop them manually.")


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
        _run_shell(f"x-terminal-emulator -T {shlex.quote(title)} -e bash -lc {shlex.quote(keep_shell)}")
        return
    _run_shell(f"{shlex.quote(term)} -T {shlex.quote(title)} -e bash -lc {shlex.quote(keep_shell)}")


def _parse_args(argv: list[str]) -> tuple[str, list[str]]:
    if len(argv) < 2:
        raise RuntimeError("usage: launch_wrapper.py <mode> [-- extra args]")
    mode = argv[1]
    extra = argv[2:]
    return mode, extra


def main(argv: list[str]) -> int:
    script_name = Path(argv[0]).name
    mode, extra_args = _parse_args(argv)

    ws_dir = Path(os.environ.get("WS_DIR", "")).expanduser()
    if not ws_dir:
        ws_dir = Path(__file__).resolve().parent.parent

    ros_setup = Path(os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash"))
    overlay_setup = Path(os.environ.get("OVERLAY_SETUP", str(ws_dir / "install/setup.bash")))

    if not ros_setup.exists():
        print(f"[{script_name}] Missing ROS setup: {ros_setup}", file=sys.stderr)
        return 1
    if not overlay_setup.exists():
        print(f"[{script_name}] Missing workspace overlay: {overlay_setup}", file=sys.stderr)
        return 1

    strict = _is_truthy(os.environ.get("STRICT"))
    kill_existing = _is_truthy(os.environ.get("KILL_EXISTING", "1"))

    no_new_terminal_env = os.environ.get("NO_NEW_TERMINAL")
    no_new_terminal = _is_truthy(no_new_terminal_env) or (_in_docker() and not no_new_terminal_env)

    # Mode-specific params file
    if mode in {"reality_mapping", "reality_navigation"}:
        params_file = Path(os.environ.get("REALITY_PARAMS_FILE", str(ws_dir / "src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params.yaml")))
    elif mode in {"sim_mapping", "sim_nav"}:
        params_file = Path(os.environ.get("SIM_PARAMS_FILE", str(ws_dir / "src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml")))
    else:
        raise RuntimeError(f"unknown mode '{mode}'")

    rcutils = os.environ.get("RCUTILS_LOGGING_SEVERITY") if mode == "sim_nav" else None

    cfg = CommonConfig(
        script_name=script_name,
        ws_dir=ws_dir,
        ros_setup=ros_setup,
        overlay_setup=overlay_setup,
        params_file=params_file,
        no_new_terminal=no_new_terminal,
        terminal_cmd="",
        strict=strict,
        kill_existing=kill_existing,
        rcutils_logging_severity=rcutils,
    )
    cfg.terminal_cmd = _pick_terminal_cmd(cfg)
    if not cfg.terminal_cmd:
        cfg.no_new_terminal = True

    # Print behavior tree report (optional)
    if _is_truthy(os.environ.get("PRINT_BT_REPORT", "1")):
        rep = _bt_report(cfg.params_file, cfg.script_name)
        if rep:
            print(rep)

    # NeuPAN env snippet
    controller_plugin = _controller_plugin(cfg.params_file)
    if controller_plugin:
        print(f"[{cfg.script_name}] controller_plugin='{controller_plugin}'", file=sys.stderr)

    neupan_env = ""
    try:
        neupan_env = _neupan_env(
            controller_plugin,
            neupan_activate=Path(os.environ.get("NEUPAN_ACTIVATE", str(ws_dir / "neupan_env/bin/activate"))),
            neupan_site_packages=Path(os.environ.get("NEUPAN_SITE_PACKAGES", str(ws_dir / "neupan_env/lib/python3.10/site-packages"))),
            neupan_model_setup=os.environ.get("NEUPAN_MODEL_SETUP", str(ws_dir / "install/neupan_models/share/neupan_models/local_setup.bash"))
            if mode in {"reality_mapping", "reality_navigation", "sim_mapping"}
            else os.environ.get("NEUPAN_MODEL_SETUP", ""),
            script_name=cfg.script_name,
        )
    except Exception as exc:
        print(f"[{cfg.script_name}] {exc}", file=sys.stderr)
        return 1

    if not controller_plugin:
        print(f"[{cfg.script_name}] controller_plugin='unset'; NeuPAN virtualenv will not be activated.", file=sys.stderr)

    # Kill/conflict handling
    if not kill_existing:
        _check_conflicts(cfg.script_name, cfg.strict)

    if mode == "reality_mapping":
        # Reality modes: default to Foxglove (headless) instead of RViz.
        start_foxglove = _is_truthy(os.environ.get("START_FOXGLOVE", "1"))
        start_rviz = _is_truthy(os.environ.get("START_RVIZ", "0"))

        # In single-terminal mode we keep this wrapper alive, so we can clean up bg processes.
        bg: Optional[BackgroundGroup] = None
        if cfg.no_new_terminal:
            bg = BackgroundGroup(cfg.script_name)
            atexit.register(bg.cleanup)

            def _sig(_signum, _frame):
                bg.cleanup()
                raise SystemExit(130)

            signal.signal(signal.SIGINT, _sig)
            signal.signal(signal.SIGTERM, _sig)

        if start_foxglove:
            if kill_existing:
                _kill_by_pattern(r"ros2 launch foxglove_bridge foxglove_bridge_launch.xml", "foxglove_bridge", cfg.script_name)
            foxglove_cmd = _foxglove_bridge_command()
            _launch_in_terminal(cfg, "Foxglove Bridge", foxglove_cmd, "", background=True, bg=bg)

        mapping_cmd = os.environ.get(
            "MAPPING_CMD",
            "ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True",
        )
        if extra_args:
            mapping_cmd = mapping_cmd + " " + " ".join(map(shlex.quote, extra_args))

        # Default: do not start RViz unless explicitly requested.
        mapping_cmd = _ensure_launch_arg(mapping_cmd, "use_rviz", "True" if start_rviz else "False")

        if kill_existing:
            _kill_by_pattern(r"ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py", "rm_navigation_reality_launch.py", cfg.script_name)
            _kill_by_pattern(r"(^|/)joint_state_publisher(\\s|$)", "joint_state_publisher", cfg.script_name)
            _kill_by_pattern(r"(^|/)robot_state_publisher(\\s|$)", "robot_state_publisher", cfg.script_name)
            _kill_by_pattern(r"(^|/)auto_aim_yaw_joint_state_bridge(\\s|$)", "auto_aim_yaw_joint_state_bridge", cfg.script_name)

        _launch_in_terminal(cfg, "Reality Mapping", mapping_cmd, neupan_env)
        return 0

    if mode == "reality_navigation":
        start_foxglove = _is_truthy(os.environ.get("START_FOXGLOVE", "1"))
        start_rviz = _is_truthy(os.environ.get("START_RVIZ", "0"))

        bg: Optional[BackgroundGroup] = None
        if cfg.no_new_terminal:
            bg = BackgroundGroup(cfg.script_name)
            atexit.register(bg.cleanup)

            def _sig(_signum, _frame):
                bg.cleanup()
                raise SystemExit(130)

            signal.signal(signal.SIGINT, _sig)
            signal.signal(signal.SIGTERM, _sig)

        if start_foxglove:
            if kill_existing:
                _kill_by_pattern(r"ros2 launch foxglove_bridge foxglove_bridge_launch.xml", "foxglove_bridge", cfg.script_name)
            foxglove_cmd = _foxglove_bridge_command()
            _launch_in_terminal(cfg, "Foxglove Bridge", foxglove_cmd, "", background=True, bg=bg)

        nav_cmd = os.environ.get(
            "NAVIGATION_CMD",
            "ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True",
        )
        if extra_args:
            nav_cmd = nav_cmd + " " + " ".join(map(shlex.quote, extra_args))

        # Default: do not start RViz unless explicitly requested.
        nav_cmd = _ensure_launch_arg(nav_cmd, "use_rviz", "True" if start_rviz else "False")

        if kill_existing:
            _kill_by_pattern(r"ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py", "rm_navigation_reality_launch.py", cfg.script_name)
            _kill_by_pattern(r"(^|/)joint_state_publisher(\\s|$)", "joint_state_publisher", cfg.script_name)
            _kill_by_pattern(r"(^|/)robot_state_publisher(\\s|$)", "robot_state_publisher", cfg.script_name)
            _kill_by_pattern(r"(^|/)auto_aim_yaw_joint_state_bridge(\\s|$)", "auto_aim_yaw_joint_state_bridge", cfg.script_name)

        _launch_in_terminal(cfg, "Reality Navigation", nav_cmd, neupan_env)
        return 0

    # Simulation modes: optionally run Gazebo in bg (single-terminal)
    bg = BackgroundGroup(cfg.script_name)
    atexit.register(bg.cleanup)

    def _sig(_signum, _frame):
        bg.cleanup()
        raise SystemExit(130)

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    enable_gt = _enable_chassis_odometry_gt(cfg.params_file)
    print(f"[{cfg.script_name}] enable_chassis_odometry_gt={'true' if enable_gt else 'false'} (from {cfg.params_file})", file=sys.stderr)

    gazebo_cmd = os.environ.get("GAZEBO_CMD", "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py")
    if "enable_chassis_odometry_gt:=" not in gazebo_cmd:
        gazebo_cmd = gazebo_cmd + f" enable_chassis_odometry_gt:={'true' if enable_gt else 'false'}"

    # Auto-detect headless environment: if DISPLAY is unset/empty or we are
    # running inside NO_NEW_TERMINAL mode (nohup / background), default to
    # headless Gazebo to avoid EGL / GPU rendering crashes.
    if "use_gui:=" not in gazebo_cmd:
        _force_headless = _is_truthy(os.environ.get("GAZEBO_HEADLESS"))
        _no_display = not os.environ.get("DISPLAY", "").strip()
        if _force_headless or _no_display or cfg.no_new_terminal:
            gazebo_cmd = _ensure_launch_arg(gazebo_cmd, "use_gui", "false")
            print(f"[{cfg.script_name}] Headless Gazebo (no GUI): "
                  f"DISPLAY={os.environ.get('DISPLAY', '(unset)')!r}, "
                  f"no_new_terminal={cfg.no_new_terminal}, "
                  f"GAZEBO_HEADLESS={os.environ.get('GAZEBO_HEADLESS', '(unset)')!r}",
                  file=sys.stderr)

    def _wait_for_background(bg: BackgroundGroup) -> int:
        """Block until all background processes exit (used in multi-terminal mode
        where the foreground command returns immediately after spawning a window).
        This prevents atexit from killing Gazebo prematurely."""
        if not bg._pids:
            return 0
        print(f"[{cfg.script_name}] Waiting for background processes (pids: {' '.join(map(str, bg._pids))})...", file=sys.stderr)
        print(f"[{cfg.script_name}] Press Ctrl+C to stop all.", file=sys.stderr)
        import time as _time
        try:
            while True:
                alive = []
                for pid in bg._pids:
                    try:
                        os.kill(pid, 0)  # check if alive
                        alive.append(pid)
                    except OSError:
                        pass
                if not alive:
                    break
                _time.sleep(1.0)
        except (KeyboardInterrupt, SystemExit):
            pass
        return 0

    # Helper: does the foreground launch return immediately (multi-terminal)?
    _fg_returns_immediately = bool(cfg.terminal_cmd) and not cfg.no_new_terminal

    if mode == "sim_mapping":
        slam_cmd = os.environ.get("SLAM_CMD", "ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py slam:=True")
        _launch_in_terminal(cfg, "Gazebo Sim", gazebo_cmd, "", background=True, bg=bg)
        time.sleep(1.0)
        _launch_in_terminal(cfg, "SLAM", slam_cmd, neupan_env)
        if _fg_returns_immediately:
            return _wait_for_background(bg)
        return 0

    if mode == "sim_nav":
        nav_cmd = os.environ.get("NAV_CMD", "ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False")
        _launch_in_terminal(cfg, "Gazebo Sim", gazebo_cmd, "", background=True, bg=bg)
        time.sleep(1.0)
        _launch_in_terminal(cfg, "Nav", nav_cmd, neupan_env)

        if _is_truthy(os.environ.get("RQT_GRAPH", "false")):
            rqt_ns = os.environ.get("RQT_GRAPH_NAMESPACE", "").strip()
            rqt_args = os.environ.get("RQT_GRAPH_ARGS", "").strip()
            rqt_env = f"export ROS_NAMESPACE={shlex.quote(rqt_ns)}" if rqt_ns else ""
            rqt_cmd = "rqt_graph" + (" " + rqt_args if rqt_args else "")
            _launch_in_terminal(cfg, "rqt_graph", rqt_cmd, rqt_env)

        if _fg_returns_immediately:
            return _wait_for_background(bg)
        return 0

    raise RuntimeError(f"unhandled mode '{mode}'")


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[{Path(sys.argv[0]).name}] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)

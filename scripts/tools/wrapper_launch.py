#!/usr/bin/env python3
"""Launch orchestration, bag recording, map/bin save, odin mode consistency."""
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
from typing import Callable, Optional

from tools.wrapper_config import (
    AUTO_MAP_DIR_NAME,
    BEIJING_TZ,
    build_base_env,
    current_branch,
    is_truthy,
    slugify,
    run_shell,
)
from tools.wrapper_process import kill_odin_driver, pid_gone
from tools.wrapper_models import BackgroundGroup, CommonConfig


ODIN_SAVE_GRACE_SEC = float(os.environ.get("ODIN_SAVE_GRACE_SEC", "8"))


# ---------------------------------------------------------------------------
# Launch argument helpers
# ---------------------------------------------------------------------------

def ensure_launch_arg(cmd: str, name: str, value: str) -> str:
    if re.search(rf"(^|\s){re.escape(name)}:=", cmd):
        return cmd
    return cmd + f" {name}:={value}"


def upsert_launch_arg(cmd: str, name: str, value: str) -> str:
    pattern = rf"(^|\s){re.escape(name)}:=([^\s]+)"
    replacement = rf"\1{name}:={value}"
    if re.search(pattern, cmd):
        return re.sub(pattern, replacement, cmd, count=1)
    return cmd + f" {name}:={value}"


def extract_launch_arg(cmd: str, name: str) -> Optional[str]:
    try:
        for token in shlex.split(cmd):
            prefix = f"{name}:="
            if token.startswith(prefix):
                return token[len(prefix):]
    except Exception:
        pass
    return None


# ---------------------------------------------------------------------------
# Odin mode consistency
# ---------------------------------------------------------------------------

def _resolve_odin_config_path(ws_dir: Path, ros_cmd: str) -> Path:
    odin_cfg_from_cmd = extract_launch_arg(ros_cmd, "odin_config_file")
    odin_cfg_env = os.environ.get("ODIN_CONFIG_FILE", "").strip()
    odin_cfg_default = ws_dir / "src/odin_ros_driver/config/control_command.yaml"
    return Path(odin_cfg_from_cmd or odin_cfg_env or str(odin_cfg_default)).expanduser()


def _read_odin_custom_map_mode(ws_dir: Path, ros_cmd: str) -> Optional[int]:
    cfg_path = _resolve_odin_config_path(ws_dir, ros_cmd)
    if not cfg_path.exists():
        return None
    try:
        import yaml  # type: ignore

        data = yaml.safe_load(cfg_path.read_text()) or {}
        register_keys = data.get("register_keys", {}) if isinstance(data, dict) else {}
        raw_mode = register_keys.get("custom_map_mode")
        if raw_mode is None:
            return None
        return int(raw_mode)
    except Exception:
        return None


def _force_odin_custom_map_mode(cfg_path: Path, desired_mode: int, script_name: str) -> None:
    if not cfg_path.exists():
        print(f"[{script_name}] WARN odin config not found: {cfg_path}; skip mode override.", file=sys.stderr)
        return

    text = cfg_path.read_text()
    pattern = r"(?m)^(\s*custom_map_mode:\s*)(\d+)(\s*(?:#.*)?)$"
    match = re.search(pattern, text)
    if not match:
        raise RuntimeError(f"custom_map_mode not found in odin config: {cfg_path}")

    current_mode = int(match.group(2))
    if current_mode == desired_mode:
        return

    new_text, count = re.subn(pattern, rf"\g<1>{desired_mode}\g<3>", text, count=1)
    if count != 1:
        raise RuntimeError(f"Failed to update custom_map_mode in odin config: {cfg_path}")

    cfg_path.write_text(new_text)
    print(f"[{script_name}] Override odin custom_map_mode: {current_mode} -> {desired_mode} ({cfg_path})", file=sys.stderr)


def ensure_odin_mode_consistency(cfg: CommonConfig, mode: str, ros_cmd: str) -> str:
    """仅对实车入口做自动一致性处理，避免 launch 参数与 odin YAML 配置漂移。"""
    if mode not in {"reality_mapping", "reality_navigation"}:
        return ros_cmd

    preset_raw = os.environ.get("ODIN_MODE_PRESET", "").strip()
    if preset_raw:
        try:
            preset_mode = int(preset_raw)
        except ValueError as exc:
            raise RuntimeError(f"Invalid ODIN_MODE_PRESET '{preset_raw}', expected 0/1/2") from exc
        if preset_mode not in {0, 1, 2}:
            raise RuntimeError(f"Invalid ODIN_MODE_PRESET '{preset_raw}', expected 0/1/2")

        cfg_path = _resolve_odin_config_path(cfg.ws_dir, ros_cmd)
        _force_odin_custom_map_mode(cfg_path, preset_mode, cfg.script_name)
        ros_cmd = upsert_launch_arg(ros_cmd, "odin_map_mode", str(preset_mode))
        print(f"[{cfg.script_name}] Force odin_map_mode:={preset_mode} by ODIN_MODE_PRESET", file=sys.stderr)
        return ros_cmd

    custom_mode = _read_odin_custom_map_mode(cfg.ws_dir, ros_cmd)
    if custom_mode is None:
        print(f"[{cfg.script_name}] WARN cannot read custom_map_mode from odin config; skip mode auto-check.", file=sys.stderr)
        return ros_cmd

    launch_mode = extract_launch_arg(ros_cmd, "odin_map_mode")
    if launch_mode is not None:
        try:
            launch_mode_int = int(launch_mode)
        except ValueError as exc:
            raise RuntimeError(f"Invalid odin_map_mode '{launch_mode}' in launch cmd") from exc

        if launch_mode_int != custom_mode:
            raise RuntimeError(
                "odin_map_mode mismatch: "
                f"launch={launch_mode_int}, control_command.yaml custom_map_mode={custom_mode}. "
                "Please keep them identical."
            )
        return ros_cmd

    ros_cmd = ensure_launch_arg(ros_cmd, "odin_map_mode", str(custom_mode))
    print(f"[{cfg.script_name}] Auto-inject odin_map_mode:={custom_mode} to match control_command.yaml", file=sys.stderr)
    return ros_cmd


# ---------------------------------------------------------------------------
# Map / bin save
# ---------------------------------------------------------------------------

def save_map_now(cfg: CommonConfig, mode: str, timestamp: str, namespace: str | None = None) -> None:
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    maps_dir = cfg.ws_dir / AUTO_MAP_DIR_NAME / branch_safe
    maps_dir.mkdir(parents=True, exist_ok=True)
    map_prefix = maps_dir / f"{Path(cfg.script_name).stem}_{timestamp}"

    base_env = build_base_env(cfg)
    cmd = f"{base_env}; ros2 run nav2_map_server map_saver_cli -f {shlex.quote(str(map_prefix))}"
    if namespace:
        cmd += f" --ros-args -r __ns:={shlex.quote(namespace)}"

    print(f"[{cfg.script_name}] Auto-saving map to {map_prefix}.*", file=sys.stderr)
    try:
        run_shell(cmd)
        print(f"[{cfg.script_name}] Auto-save map done: {map_prefix}.yaml / {map_prefix}.pgm", file=sys.stderr)
    except Exception as exc:
        print(f"[{cfg.script_name}] Auto-save map failed: {exc}", file=sys.stderr)


def save_odin_bin_now(cfg: CommonConfig) -> None:
    odin_dir = cfg.ws_dir / "src/odin_ros_driver"
    set_param_sh = odin_dir / "set_param.sh"
    if not set_param_sh.exists():
        print(f"[{cfg.script_name}] Skip odin bin save: {set_param_sh} not found", file=sys.stderr)
        return

    base_env = build_base_env(cfg)
    cmd = f"{base_env}; cd {shlex.quote(str(odin_dir))}; bash set_param.sh save_map 1"

    print(f"[{cfg.script_name}] Trigger odin save_map=1 for bin export", file=sys.stderr)
    try:
        run_shell(cmd)
    except Exception as exc:
        print(f"[{cfg.script_name}] Trigger odin bin save failed: {exc}", file=sys.stderr)


# ---------------------------------------------------------------------------
# Bag recording
# ---------------------------------------------------------------------------

def _normalize_bag_out_dir(ws_dir: Path, out_dir: str) -> str:
    raw = out_dir.strip()
    if not raw:
        return ""

    path = Path(raw).expanduser()
    if path.is_absolute():
        try:
            rel_path = path.relative_to("/ws")
        except ValueError:
            return str(path)
        return str(ws_dir / rel_path)

    return str(ws_dir / path)


def start_bag_recording(cfg: CommonConfig, bg: BackgroundGroup) -> None:
    """按 nav2 参数中的 bag_record 配置启动录包。"""
    params_path = cfg.params_file
    if not params_path.exists():
        print(f"[{cfg.script_name}] WARN bag config not found: {params_path}; skip bag recording.", file=sys.stderr)
        return

    try:
        import yaml  # type: ignore

        raw_yaml = yaml.safe_load(params_path.read_text()) or {}
    except Exception as exc:
        print(f"[{cfg.script_name}] WARN cannot read bag config from {params_path}: {exc}; skip bag recording.", file=sys.stderr)
        return

    robot_runtime = {}
    if isinstance(raw_yaml, dict):
        robot_node = raw_yaml.get("robot_description_runtime")
        if isinstance(robot_node, dict):
            robot_runtime = robot_node.get("ros__parameters", {}) or {}

    bag_cfg = robot_runtime.get("bag_record") if isinstance(robot_runtime, dict) else None
    if not isinstance(bag_cfg, dict):
        bag_cfg = {}

    auto_record = is_truthy(os.environ.get("AUTO_RECORD_BAG"))
    if auto_record:
        bag_cfg["enabled"] = True

    if not bool(bag_cfg.get("enabled", False)):
        print(f"[{cfg.script_name}] bag_record enabled=false; skip bag recording.", file=sys.stderr)
        return

    profiles = bag_cfg.get("profiles", {})
    if not isinstance(profiles, dict) or not profiles:
        print(f"[{cfg.script_name}] bag_record profiles empty; skip bag recording.", file=sys.stderr)
        return

    base_env = build_base_env(cfg)
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe
    log_dir.mkdir(parents=True, exist_ok=True)
    session_tag = datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S_%f")

    auto_mode = os.environ.get("AUTO_RECORD_BAG_MODE", "").strip()

    started = 0
    for profile_name, profile in profiles.items():
        if not isinstance(profile, dict):
            print(f"[{cfg.script_name}] WARN bag profile '{profile_name}' is not a mapping; skip.", file=sys.stderr)
            continue
        if not bool(profile.get("enabled", True)):
            continue

        mode = str(profile.get("mode", "")).strip()
        if auto_mode and mode != auto_mode:
            continue
        if mode not in {"basic", "full", "raw"}:
            print(f"[{cfg.script_name}] WARN unsupported bag mode '{mode}' in {params_path}; skip.", file=sys.stderr)
            continue

        storage = str(profile.get("storage", "mcap")).strip() or "mcap"
        compress = str(profile.get("compress", "")).strip()
        max_bag_size_mb = int(profile.get("max_bag_size_mb", 0))
        out_dir = _normalize_bag_out_dir(cfg.ws_dir, str(profile.get("out_dir", "")))

        bag_cmd = [
            "ros2", "run", "ros2_bag_tools", "record_bag",
            "--mode", mode, "--storage", storage, "--session-tag", session_tag,
        ]
        if max_bag_size_mb > 0:
            bag_cmd += ["--max-bag-size-mb", str(max_bag_size_mb)]
        if compress:
            bag_cmd += ["--compress", compress]
        if out_dir:
            bag_cmd += ["--out-dir", out_dir]

        cmd = f"cd {shlex.quote(str(cfg.ws_dir))}; {base_env}; {' '.join(shlex.quote(part) for part in bag_cmd)}"

        is_bag_script = Path(cfg.script_name).stem.startswith("bag")
        if is_bag_script:
            log_file = log_dir / f"{Path(cfg.script_name).stem}_{slugify(str(profile_name))}_{session_tag}.log"
            out_fd = open(log_file, "w")
            print(f"[{cfg.script_name}] (bag) {profile_name} -> {log_file}", file=sys.stderr)
        else:
            out_fd = subprocess.DEVNULL
            print(f"[{cfg.script_name}] (bag) {profile_name} started (no log)", file=sys.stderr)

        p = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=out_fd,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        bg.add(p.pid)
        started += 1

    if started == 0:
        print(f"[{cfg.script_name}] bag_record profiles produced no bag processes.", file=sys.stderr)


# ---------------------------------------------------------------------------
# Terminal launch
# ---------------------------------------------------------------------------

def launch_in_terminal(
    cfg: CommonConfig,
    title: str,
    command: str,
    extra_env: str,
    *,
    background: bool = False,
    bg: Optional[BackgroundGroup] = None,
    pgid_file: Optional[Path] = None,
    pre_shutdown_hook: Optional[Callable[[], None]] = None,
) -> int:
    base_env = build_base_env(cfg)
    full_cmd = f"cd {shlex.quote(str(cfg.ws_dir))}; {base_env}"
    if extra_env:
        full_cmd += f"; {extra_env}"
    full_cmd += f"; {command}"

    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe
    ts = datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S_%f")
    log_file_name = f"{Path(cfg.script_name).stem}_{ts}.log"

    if background:
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / log_file_name
        print(f"[{cfg.script_name}] (background) {title} -> {log_file}", file=sys.stderr)
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
        wrap_cmd = f"{full_cmd} 2>&1 | tee -a {shlex.quote(str(log_file))}"
        p = subprocess.Popen(["bash", "-lc", wrap_cmd], preexec_fn=os.setsid)
        if pgid_file:
            try:
                pgid_file.write_text(str(p.pid))
            except Exception:
                pass
        child_pgid = p.pid

        other_shutdown_timeout = int(os.environ.get("OTHER_SHUTDOWN_TIMEOUT", "5"))
        odin_shutdown_timeout = float(os.environ.get("ODIN_SHUTDOWN_TIMEOUT", "15"))
        force_kill_after_timeout = is_truthy(os.environ.get("FORCE_KILL_AFTER_TIMEOUT", "0"))
        odin_force_kill = is_truthy(os.environ.get("ODIN_FORCE_KILL", "0"))
        shutdown_requested = [False]
        pre_shutdown_done = [False]
        odin_shutdown_done = [False]
        timeout_notified = [False]

        def _force_kill() -> None:
            try:
                os.killpg(child_pgid, signal.SIGKILL)
            except Exception:
                pass

        def _forward_signal(signum: int, _frame: object) -> None:
            if shutdown_requested[0]:
                print(f"\n[{cfg.script_name}] Force killing (SIGKILL)...", file=sys.stderr)
                _force_kill()
                return

            shutdown_requested[0] = True

            if pre_shutdown_hook is not None and not pre_shutdown_done[0]:
                pre_shutdown_done[0] = True
                try:
                    pre_shutdown_hook()
                except Exception as exc:
                    print(f"[{cfg.script_name}] pre-shutdown hook failed: {exc}", file=sys.stderr)

            if not odin_shutdown_done[0]:
                odin_shutdown_done[0] = True
                print(f"\n[{cfg.script_name}] Shutting down odin driver first (timeout {odin_shutdown_timeout}s)...", file=sys.stderr)
                kill_odin_driver(cfg.script_name, timeout=odin_shutdown_timeout, force_kill=odin_force_kill)

            print(
                f"[{cfg.script_name}] Shutting down other processes (timeout {other_shutdown_timeout}s)..."
                " Press Ctrl+C again to force kill.",
                file=sys.stderr,
            )
            try:
                os.killpg(child_pgid, signum)
            except Exception:
                pass

        prev_sigint = signal.signal(signal.SIGINT, _forward_signal)
        prev_sigterm = signal.signal(signal.SIGTERM, _forward_signal)
        try:
            deadline = None
            while True:
                try:
                    p.wait(timeout=1.0)
                    break
                except subprocess.TimeoutExpired:
                    pass
                if shutdown_requested[0]:
                    if deadline is None:
                        deadline = time.monotonic() + other_shutdown_timeout
                    if time.monotonic() > deadline:
                        if force_kill_after_timeout:
                            print(f"[{cfg.script_name}] Shutdown timeout ({other_shutdown_timeout}s), force killing...", file=sys.stderr)
                            _force_kill()
                            p.wait()
                            break
                        if not timeout_notified[0]:
                            print(
                                f"[{cfg.script_name}] Shutdown timeout ({other_shutdown_timeout}s), keep waiting."
                                " Press Ctrl+C again to force kill.",
                                file=sys.stderr,
                            )
                            timeout_notified[0] = True
        finally:
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)
            if pgid_file:
                try:
                    pgid_file.unlink()
                except FileNotFoundError:
                    pass
        return p.returncode or 0

    # gnome-terminal branch
    term = cfg.terminal_cmd
    keep_shell = f"{full_cmd}; exec bash"
    if term == "gnome-terminal":
        run_shell(f"gnome-terminal --title={shlex.quote(title)} -- bash -c {shlex.quote(keep_shell)}")
        return 0

    subprocess.Popen([term, "-T", title, "-e", "bash", "-lc", keep_shell], preexec_fn=os.setsid)


def wait_for_background(bg: BackgroundGroup, script_name: str) -> int:
    if not bg._pids:
        return 0

    print(f"[{script_name}] Waiting for background pids: {bg._pids}. Ctrl+C to stop all.", file=sys.stderr)
    try:
        while not all(pid_gone(p) for p in bg._pids):
            time.sleep(1.0)
    except (KeyboardInterrupt, SystemExit):
        pass
    return 0

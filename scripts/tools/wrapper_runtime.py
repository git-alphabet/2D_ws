#!/usr/bin/env python3
from __future__ import annotations

import os
import re
import shlex
import signal
import subprocess
import sys
import time
from datetime import timedelta, timezone, datetime
from pathlib import Path
from typing import Callable, Optional

from tools.wrapper_helpers import (
    AUTO_MAP_DIR_NAME,
    BEIJING_TZ,
    build_base_env,
    current_branch,
    run_shell,
    pid_gone,
)
from tools.wrapper_models import BackgroundGroup, CommonConfig


def validate_extra_launch_args(args: list[str]) -> None:
    allowed_prefixes = ("--", "__")
    for arg in args:
        if not arg:
            continue
        if arg.startswith(allowed_prefixes):
            continue
        if ":=" in arg:
            continue
        raise RuntimeError(f"Malformed launch argument '{arg}', expected '<name>:=<value>'")


def _beijing_timestamp() -> str:
    return datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S")


def _auto_map_prefix(ws_dir: Path, branch_safe: str, script_name: str, timestamp: str | None = None) -> Path:
    maps_dir = ws_dir / AUTO_MAP_DIR_NAME / branch_safe
    maps_dir.mkdir(parents=True, exist_ok=True)
    ts = timestamp or _beijing_timestamp()
    return maps_dir / f"{Path(script_name).stem}_{ts}"


def save_map_now(cfg: CommonConfig, mode: str, timestamp: str, namespace: str | None = None) -> None:
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    map_prefix = _auto_map_prefix(cfg.ws_dir, branch_safe, cfg.script_name, timestamp)
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


def ensure_launch_arg(cmd: str, name: str, value: str) -> str:
    if re.search(rf"(^|\s){re.escape(name)}:=", cmd):
        return cmd
    return cmd + f" {name}:={value}"


def extract_launch_arg(cmd: str, name: str) -> Optional[str]:
    try:
        for token in shlex.split(cmd):
            prefix = f"{name}:="
            if token.startswith(prefix):
                return token[len(prefix) :]
    except Exception:
        pass
    return None


def start_watchdog(cfg: CommonConfig, topics: list[tuple[str, float]], bg: BackgroundGroup) -> None:
    """启动话题频率 watchdog。仅当 ENABLE_WATCHDOG=1 时生效。"""
    from tools.wrapper_helpers import is_truthy

    if not is_truthy(os.environ.get("ENABLE_WATCHDOG")):
        return

    checks = " ".join(f"{shlex.quote(t)}:{hz}" for t, hz in topics)
    base_env = build_base_env(cfg)
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe
    log_dir.mkdir(parents=True, exist_ok=True)
    bj_tz = timezone(timedelta(hours=8))
    ts = datetime.now(bj_tz).strftime("%Y%m%d_%H%M%S_%f")
    log_file = log_dir / f"{Path(cfg.script_name).stem}_{ts}.log"

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


def start_timestamp_monitor(cfg: CommonConfig, bg: BackgroundGroup) -> None:
    """启动时间戳同步监控。默认关闭，可通过 ENABLE_TIMESTAMP_MONITOR=1 开启。"""
    from tools.wrapper_helpers import is_truthy

    if not is_truthy(os.environ.get("ENABLE_TIMESTAMP_MONITOR", "0")):
        return

    config_env = os.environ.get("TIMESTAMP_SYNC_MONITOR_CONFIG", "").strip()
    if config_env:
        config_path = Path(config_env).expanduser()
    else:
        config_path = cfg.ws_dir / "scripts/config/timestamp_sync_monitor.yaml"

    if not config_path.exists():
        print(
            f"[{cfg.script_name}] WARN timestamp monitor config not found: {config_path}; skip monitor.",
            file=sys.stderr,
        )
        return

    base_env = build_base_env(cfg)
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe
    log_dir.mkdir(parents=True, exist_ok=True)
    bj_tz = timezone(timedelta(hours=8))
    ts = datetime.now(bj_tz).strftime("%Y%m%d_%H%M%S_%f")
    log_file = log_dir / f"{Path(cfg.script_name).stem}_timestamp_sync_{ts}.log"

    cmd = (
        f"{base_env}; "
        f"export TIMESTAMP_SYNC_MONITOR_CONFIG={shlex.quote(str(config_path))}; "
        f"python3 -m tools.timestamp_sync_monitor 2>&1 | tee -a {shlex.quote(str(log_file))}"
    )
    print(
        f"[{cfg.script_name}] (timestamp-monitor) using {config_path} -> {log_file}",
        file=sys.stderr,
    )
    p = subprocess.Popen(["bash", "-lc", cmd], preexec_fn=os.setsid)
    bg.add(p.pid)


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
) -> None:
    base_env = build_base_env(cfg)

    full_cmd = f"cd {shlex.quote(str(cfg.ws_dir))}; {base_env}"
    if extra_env:
        full_cmd += f"; {extra_env}"
    full_cmd += f"; {command}"

    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe
    bj_tz = timezone(timedelta(hours=8))
    ts = datetime.now(bj_tz).strftime("%Y%m%d_%H%M%S_%f")
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

        shutdown_timeout = int(os.environ.get("SHUTDOWN_TIMEOUT", "15"))
        shutdown_requested = [False]
        pre_shutdown_done = [False]

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

            if pre_shutdown_hook is not None and not pre_shutdown_done[0]:
                pre_shutdown_done[0] = True
                try:
                    pre_shutdown_hook()
                except Exception as exc:
                    print(f"[{cfg.script_name}] pre-shutdown hook failed: {exc}", file=sys.stderr)

            shutdown_requested[0] = True
            print(
                f"\n[{cfg.script_name}] Shutting down (timeout {shutdown_timeout}s)..."
                " Press Ctrl+C again to force kill.",
                file=sys.stderr,
            )
            try:
                os.killpg(child_pgid, signum)
            except Exception:
                pass

        prev_sigint = signal.signal(signal.SIGINT, _forward_signal)  # type: ignore[arg-type]
        prev_sigterm = signal.signal(signal.SIGTERM, _forward_signal)  # type: ignore[arg-type]
        try:
            deadline = time.monotonic() + shutdown_timeout
            while True:
                try:
                    p.wait(timeout=1.0)
                    break
                except subprocess.TimeoutExpired:
                    pass
                if shutdown_requested[0] and time.monotonic() > deadline:
                    print(
                        f"[{cfg.script_name}] Shutdown timeout ({shutdown_timeout}s), force killing...",
                        file=sys.stderr,
                    )
                    _force_kill()
                    p.wait()
                    break
        finally:
            signal.signal(signal.SIGINT, prev_sigint)
            signal.signal(signal.SIGTERM, prev_sigterm)
            if pgid_file:
                try:
                    pgid_file.unlink()
                except FileNotFoundError:
                    pass
        return

    term = cfg.terminal_cmd
    keep_shell = f"{full_cmd}; exec bash"
    if term == "gnome-terminal":
        run_shell(f"gnome-terminal --title={shlex.quote(title)} -- bash -c {shlex.quote(keep_shell)}")
        return

    if term == "x-terminal-emulator":
        subprocess.Popen(
            [
                "xterm",
                "-T",
                title,
                "-u8",
                "-bg",
                "#1e1e2e",
                "-fg",
                "#cdd6f4",
                "-fa",
                "Monospace",
                "-fs",
                "13",
                "-geometry",
                "220x55",
                "-sl",
                "5000",
                "-e",
                "bash",
                "-lc",
                keep_shell,
            ],
            preexec_fn=os.setsid,
        )
        return

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

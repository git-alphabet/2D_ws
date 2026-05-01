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
from typing import Any, Callable, Optional

from tools.wrapper_helpers import (
    AUTO_MAP_DIR_NAME,
    BEIJING_TZ,
    build_base_env,
    current_branch,
    is_truthy,
    read_yaml_params,
    run_shell,
    pid_gone,
    slugify,
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


def _normalize_namespace(namespace: str | None) -> str | None:
    if namespace is None:
        return None
    normalized = namespace.strip()
    if not normalized:
        return None
    if not normalized.startswith("/"):
        normalized = f"/{normalized}"
    return normalized


def _extract_map_saver_namespaces(service_list_text: str) -> list[str | None]:
    namespaces: list[str | None] = []
    suffix = "/map_saver/save_map"
    for raw_line in service_list_text.splitlines():
        service_name = raw_line.strip()
        if not service_name.endswith(suffix):
            continue

        if service_name in {suffix, "map_saver/save_map"}:
            namespace = None
        else:
            namespace = _normalize_namespace(service_name[: -len(suffix)])

        if namespace not in namespaces:
            namespaces.append(namespace)

    return namespaces


def _discover_map_saver_namespaces(base_env: str, script_name: str) -> list[str | None]:
    probe_cmd = f"{base_env}; ros2 service list"
    try:
        result = subprocess.run(
            ["bash", "-lc", probe_cmd],
            text=True,
            capture_output=True,
            check=False,
        )
    except Exception as exc:
        print(f"[{script_name}] Auto-save probe failed: {exc}", file=sys.stderr)
        return []

    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip()
        if detail:
            print(f"[{script_name}] Auto-save probe stderr: {detail}", file=sys.stderr)
        return []

    return _extract_map_saver_namespaces(result.stdout)


def _build_map_save_cmd(base_env: str, map_prefix: Path, namespace: str | None) -> str:
    cmd = f"{base_env}; ros2 run nav2_map_server map_saver_cli -f {shlex.quote(str(map_prefix))}"
    normalized_namespace = _normalize_namespace(namespace)
    if normalized_namespace:
        cmd += f" --ros-args -r __ns:={shlex.quote(normalized_namespace)}"
    return cmd


def save_map_now(cfg: CommonConfig, mode: str, timestamp: str, namespace: str | None = None) -> None:
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    map_prefix = _auto_map_prefix(cfg.ws_dir, branch_safe, cfg.script_name, timestamp)
    base_env = build_base_env(cfg)
    cmd = f"{base_env}; ros2 run nav2_map_server map_saver_cli -f {shlex.quote(str(map_prefix))}"
    if namespace:
        cmd += f" --ros-args -r __ns:={shlex.quote(namespace)}"

    print(f"[{cfg.script_name}] Auto-saving map to {map_prefix}.*", file=sys.stderr)
    requested_namespace = _normalize_namespace(namespace)
    candidate_namespaces: list[str | None] = []
    if requested_namespace not in candidate_namespaces:
        candidate_namespaces.append(requested_namespace)

    for discovered_namespace in _discover_map_saver_namespaces(base_env, cfg.script_name):
        if discovered_namespace not in candidate_namespaces:
            candidate_namespaces.append(discovered_namespace)

    if None not in candidate_namespaces:
        candidate_namespaces.append(None)

    failure_details: list[str] = []
    for candidate_namespace in candidate_namespaces:
        cmd = _build_map_save_cmd(base_env, map_prefix, candidate_namespace)
        label = candidate_namespace or "/"
        print(
            f"[{cfg.script_name}] Auto-save attempt namespace={label}",
            file=sys.stderr,
        )
        result = subprocess.run(
            ["bash", "-lc", cmd],
            text=True,
            capture_output=True,
            check=False,
        )
        if result.returncode == 0:
            if result.stdout.strip():
                print(result.stdout.strip(), file=sys.stderr)
            if result.stderr.strip():
                print(result.stderr.strip(), file=sys.stderr)
            print(f"[{cfg.script_name}] Auto-save map done: {map_prefix}.yaml / {map_prefix}.pgm", file=sys.stderr)
            return

        detail = (result.stderr or result.stdout).strip()
        if not detail:
            detail = f"exit code {result.returncode}"
        failure_details.append(f"namespace={label}: {detail}")

    joined_details = " | ".join(failure_details)
    print(f"[{cfg.script_name}] Auto-save map failed: {joined_details}", file=sys.stderr)


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


def _load_bag_record_config(cfg: CommonConfig) -> tuple[Path, dict[str, Any]] | None:
    config_path = cfg.params_file
    if not config_path.exists():
        print(f"[{cfg.script_name}] WARN params file not found: {config_path}; skip auto record.", file=sys.stderr)
        return None

    root = read_yaml_params(config_path, "pb_navigation_switches").get("bag_record")
    if not isinstance(root, dict):
        print(f"[{cfg.script_name}] WARN missing pb_navigation_switches.ros__parameters.bag_record in {config_path}", file=sys.stderr)
        return None

    return config_path, root


def start_bag_recorders(cfg: CommonConfig, bg: BackgroundGroup) -> None:
    auto_record_env = os.environ.get("AUTO_RECORD_BAG")
    if auto_record_env is not None and not is_truthy(auto_record_env):
        print(f"[{cfg.script_name}] Auto bag record disabled by AUTO_RECORD_BAG={auto_record_env!r}", file=sys.stderr)
        return

    loaded = _load_bag_record_config(cfg)
    if loaded is None:
        return
    config_path, root = loaded

    enabled = bool(root.get("enabled", True))
    if auto_record_env is not None:
        enabled = is_truthy(auto_record_env)
    if not enabled:
        print(f"[{cfg.script_name}] Auto bag record disabled by config: {config_path}", file=sys.stderr)
        return

    profiles = root.get("profiles", [])
    if isinstance(profiles, dict):
        profiles = list(profiles.values())
    
    if not isinstance(profiles, list):
        print(f"[{cfg.script_name}] WARN 'profiles' must be a list or dict in {config_path}", file=sys.stderr)
        return

    base_env = build_base_env(cfg)
    branch = current_branch(cfg.ws_dir)
    branch_safe = re.sub(r"[^A-Za-z0-9._-]", "_", branch)
    log_dir = cfg.ws_dir / "launch_logs" / branch_safe
    log_dir.mkdir(parents=True, exist_ok=True)
    session_tag = datetime.now(BEIJING_TZ).strftime("%Y%m%d_%H%M%S")

    started = 0
    for index, profile in enumerate(profiles):
        if not isinstance(profile, dict):
            print(f"[{cfg.script_name}] WARN bag profile #{index} is not a mapping; skip.", file=sys.stderr)
            continue
        if not bool(profile.get("enabled", True)):
            continue

        mode = str(profile.get("mode", "")).strip()
        if mode not in {"basic", "full", "raw"}:
            print(f"[{cfg.script_name}] WARN unsupported bag mode '{mode}' in {config_path}; skip.", file=sys.stderr)
            continue

        profile_name = str(profile.get("name", mode)).strip() or mode
        storage = str(profile.get("storage", "mcap")).strip() or "mcap"
        compress = str(profile.get("compress", "")).strip()
        max_bag_size_mb = int(profile.get("max_bag_size_mb", 0))
        out_dir = str(profile.get("out_dir", "")).strip()

        bag_cmd = [
            "./scripts/bag.sh",
            "--mode",
            mode,
            "--storage",
            storage,
            "--session-tag",
            session_tag,
        ]
        if max_bag_size_mb > 0:
            bag_cmd += ["--max-bag-size-mb", str(max_bag_size_mb)]
        if compress:
            bag_cmd += ["--compress", compress]
        if out_dir:
            bag_cmd += ["--out-dir", out_dir]

        cmd = (
            f"cd {shlex.quote(str(cfg.ws_dir))}; "
            f"{base_env}; "
            f"{' '.join(shlex.quote(part) for part in bag_cmd)}"
        )
        log_file = log_dir / f"{Path(cfg.script_name).stem}_bag_{slugify(profile_name)}_{session_tag}.log"
        print(
            f"[{cfg.script_name}] (bag-record) {profile_name}: mode={mode}, storage={storage}, config={config_path} -> {log_file}",
            file=sys.stderr,
        )
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            stdout=open(log_file, "w"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        bg.add(proc.pid)
        started += 1

    if started == 0:
        print(f"[{cfg.script_name}] WARN no bag profiles started from {config_path}", file=sys.stderr)


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

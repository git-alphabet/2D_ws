#!/usr/bin/env python3
"""Process lifecycle: kill, cleanup, pgid management."""
from __future__ import annotations

import glob
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

PGID_FILES: dict[str, Path] = {
    "sim": Path("/tmp/ros2_nav_sim.pgid"),
    "reality": Path("/tmp/ros2_nav_reality.pgid"),
}


def pgrep(pattern: str) -> list[int]:
    try:
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True)
        return [int(x) for x in out.split() if x.strip().isdigit()]
    except subprocess.CalledProcessError:
        return []


def pid_gone(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return False
    except OSError:
        return True


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


def kill_odin_driver(script_name: str, timeout: float = 15.0, *, force_kill: bool = False) -> bool:
    """Gracefully shutdown odin driver (host_sdk_sample) and wait for exit."""
    pattern = r"(^|/)host_sdk_sample(\s|$)"
    pids = pgrep(pattern)
    if not pids:
        print(f"[{script_name}] No odin driver (host_sdk_sample) found, skip.", file=sys.stderr)
        return True

    print(f"[{script_name}] Gracefully shutting down odin driver pids: {' '.join(map(str, pids))}", file=sys.stderr)
    for pid in list(pids):
        try:
            os.kill(pid, 0)
        except OSError:
            continue
        try:
            os.kill(pid, signal.SIGINT)
        except Exception:
            pass

    deadline = None if timeout <= 0 else time.monotonic() + timeout
    while True:
        time.sleep(0.5)
        if not pgrep(pattern):
            print(f"[{script_name}] Odin driver exited cleanly.", file=sys.stderr)
            return True
        if deadline is not None and time.monotonic() >= deadline:
            break

    remaining = pgrep(pattern)
    if not remaining:
        print(f"[{script_name}] Odin driver exited cleanly.", file=sys.stderr)
        return True

    if not force_kill:
        print(
            f"[{script_name}] WARNING: Odin driver still alive after {timeout}s, "
            f"pids: {' '.join(map(str, remaining))}. Skip SIGKILL.",
            file=sys.stderr,
        )
        return False

    print(f"[{script_name}] WARNING: Odin driver did not exit after {timeout}s, force killing.", file=sys.stderr)
    for pid in remaining:
        try:
            os.killpg(os.getpgid(pid), signal.SIGKILL)
        except Exception:
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass
    time.sleep(0.5)
    return False


def cleanup_fastdds_shm() -> None:
    """清理 FastDDS 遗留共享内存，避免重启时 DDS 初始化挂死。"""
    cleaned = 0
    for f in glob.glob("/dev/shm/fastrtps_*"):
        try:
            Path(f).unlink()
            cleaned += 1
        except Exception:
            pass
    if cleaned:
        print(f"[fastdds] Cleaned {cleaned} shm segment(s).", file=sys.stderr)


def kill_by_pgid_file(pgid_file: Path, title: str, script_name: str, *, force_kill: bool = True) -> None:
    """通过 PGID 文件终止上次启动的进程组。"""
    if not pgid_file.exists():
        return
    try:
        pgid = int(pgid_file.read_text().strip())
    except Exception:
        pgid_file.unlink(missing_ok=True)
        return

    print(f"[{script_name}] Killing {title} PGID={pgid} via pgid file ...", file=sys.stderr)
    signals = (signal.SIGTERM, signal.SIGKILL) if force_kill else (signal.SIGTERM,)
    for sig in signals:
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
    kill_by_pgid_file(PGID_FILES["sim"], "sim", script_name, force_kill=True)
    cleanup_fastdds_shm()
    for pat, title in [
        (r"bringup_sim\.launch\.py", "bringup_sim"),
        (r"ruby.*ign|ign.*gazebo|gz-server|gz-gui", "Gazebo"),
        (r"rm_navigation_simulation_launch\.py", "sim nav/SLAM"),
    ]:
        kill_by_pattern(pat, title, script_name)


def kill_reality(script_name: str) -> None:
    """启动实车前清理残留进程。"""
    kill_by_pgid_file(PGID_FILES["reality"], "reality", script_name, force_kill=False)
    cleanup_fastdds_shm()
    odin_timeout = float(os.environ.get("ODIN_SHUTDOWN_TIMEOUT", "15"))
    kill_odin_driver(script_name, timeout=odin_timeout, force_kill=False)
    for pat, title in [
        (r"rm_navigation_reality_launch\.py", "reality nav/SLAM"),
        (r"(^|/)joint_state_publisher(\s|$)", "joint_state_publisher"),
        (r"(^|/)robot_state_publisher(\s|$)", "robot_state_publisher"),
        (r"component_container_isolated.*nav2_container", "nav2_container"),
    ]:
        kill_by_pattern(pat, title, script_name)

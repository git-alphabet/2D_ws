#!/usr/bin/env python3
from __future__ import annotations

import os
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


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

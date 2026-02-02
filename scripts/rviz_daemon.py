#!/usr/bin/env python3
import os
import shlex
import signal
import subprocess
import sys
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def _is_truthy(value: Optional[str]) -> bool:
    if value is None:
        return False
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _resolve_default_rviz_config() -> str:
    # Requires ROS environment (and workspace overlay) already sourced.
    try:
        from ament_index_python.packages import get_package_share_directory

        bringup_dir = Path(get_package_share_directory("pb2025_nav_bringup"))
        default_cfg = bringup_dir / "rviz" / "nav2_default_view.rviz"
        if default_cfg.exists():
            return str(default_cfg)
    except Exception:
        pass

    # Fallback: let rviz2 decide, or user provides RVIZ_CONFIG.
    return ""


class RvizDaemon(Node):
    def __init__(self) -> None:
        super().__init__("rviz_daemon")

        self._rviz_proc: Optional[subprocess.Popen] = None

        self._srv_start = self.create_service(Trigger, "rviz/start", self._on_start)
        self._srv_stop = self.create_service(Trigger, "rviz/stop", self._on_stop)

        self.get_logger().info("Service ready: /rviz/start (std_srvs/Trigger)")
        self.get_logger().info("Service ready: /rviz/stop  (std_srvs/Trigger)")

    def _build_env(self) -> dict:
        env = dict(os.environ)

        # Optional: NVIDIA PRIME render offload on local GUI machine.
        if _is_truthy(env.get("RVIZ_NVIDIA", "1")):
            env.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
            env.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

        return env

    def _start_rviz(self) -> tuple[bool, str]:
        if self._rviz_proc and self._rviz_proc.poll() is None:
            return True, "rviz2 already running"

        rviz_cmd = os.environ.get("RVIZ_CMD", "").strip()
        rviz_config = os.environ.get("RVIZ_CONFIG", "").strip()

        if not rviz_cmd:
            # Default: use this workspace's rviz config if available.
            if not rviz_config:
                rviz_config = _resolve_default_rviz_config()

            if rviz_config:
                rviz_cmd = f"rviz2 -d {shlex.quote(rviz_config)}"
            else:
                rviz_cmd = "rviz2"

        # Start in a new process group so we can stop it cleanly.
        env = self._build_env()
        try:
            self.get_logger().info(f"Starting: {rviz_cmd}")
            self._rviz_proc = subprocess.Popen(
                ["bash", "-lc", rviz_cmd],
                env=env,
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return True, f"started (pid={self._rviz_proc.pid})"
        except Exception as exc:
            self._rviz_proc = None
            return False, f"failed to start rviz2: {exc}"

    def _stop_rviz(self) -> tuple[bool, str]:
        if not self._rviz_proc or self._rviz_proc.poll() is not None:
            self._rviz_proc = None
            return True, "rviz2 not running"

        pid = self._rviz_proc.pid
        try:
            pgid = os.getpgid(pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception:
            try:
                self._rviz_proc.terminate()
            except Exception:
                pass

        return True, f"stopped (pid={pid})"

    def _on_start(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._start_rviz()
        resp.success = bool(ok)
        resp.message = msg
        return resp

    def _on_stop(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, msg = self._stop_rviz()
        resp.success = bool(ok)
        resp.message = msg
        return resp


def main() -> int:
    rclpy.init()
    node = RvizDaemon()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

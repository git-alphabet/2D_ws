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


# ====== Local defaults (edit here if you prefer) ======
# These defaults apply when the corresponding environment variables are not set.
# You can still override them per-run via env vars:
# - RVIZ_QT_SCALE / QT_SCALE_FACTOR
# - RVIZ_QT_FONT_DPI / QT_FONT_DPI
# - RVIZ_NVIDIA
# - RVIZ_CONFIG
# - RVIZ_CMD

# UI scaling for RViz (Qt). Example: "1.2" for 120%.
DEFAULT_QT_SCALE = "1.0"

# Font DPI hint for Qt. Example: "120". Leave empty to disable.
DEFAULT_QT_FONT_DPI = "120"

# Enable NVIDIA PRIME render offload by default on the GUI machine.
DEFAULT_RVIZ_NVIDIA = True

# Force a specific rviz config (absolute path). Leave empty to auto-resolve.
DEFAULT_RVIZ_CONFIG = "/home/alphabet/temporary/ros2_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/rviz/nav2_default_view.rviz"

# Force a full command (highest priority when set, after RVIZ_CMD env). Leave empty for auto.
DEFAULT_RVIZ_CMD = ""


def _is_truthy(value: Optional[str]) -> bool:
    if value is None:
        return False
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _pick_env(env: dict, key: str, fallback_key: str) -> str:
    v = env.get(key, "").strip()
    if v:
        return v
    return env.get(fallback_key, "").strip()


def _pick_env_with_default(env: dict, key: str, fallback_key: str, default: str) -> str:
    v = _pick_env(env, key, fallback_key)
    return v if v else default


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

        # Optional: UI scale / font DPI (Qt).
        # - RVIZ_QT_SCALE=1.2  -> QT_SCALE_FACTOR=1.2
        # - RVIZ_QT_FONT_DPI=120 -> QT_FONT_DPI=120
        qt_scale = _pick_env_with_default(env, "RVIZ_QT_SCALE", "QT_SCALE_FACTOR", DEFAULT_QT_SCALE)
        if qt_scale and "QT_SCALE_FACTOR" not in env:
            env["QT_SCALE_FACTOR"] = qt_scale

        qt_dpi = _pick_env_with_default(env, "RVIZ_QT_FONT_DPI", "QT_FONT_DPI", DEFAULT_QT_FONT_DPI)
        if qt_dpi and "QT_FONT_DPI" not in env:
            env["QT_FONT_DPI"] = qt_dpi

        # Optional: NVIDIA PRIME render offload on local GUI machine.
        default_nvidia = "1" if DEFAULT_RVIZ_NVIDIA else "0"
        if _is_truthy(env.get("RVIZ_NVIDIA", default_nvidia)):
            env.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
            env.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

        return env

    def _start_rviz(self) -> tuple[bool, str]:
        if self._rviz_proc and self._rviz_proc.poll() is None:
            return True, "rviz2 already running"

        rviz_cmd = os.environ.get("RVIZ_CMD", "").strip() or DEFAULT_RVIZ_CMD.strip()
        rviz_config = os.environ.get("RVIZ_CONFIG", "").strip() or DEFAULT_RVIZ_CONFIG.strip()

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
        ws_dir = Path(os.environ.get("WS_DIR", "")).expanduser()
        if not ws_dir:
            ws_dir = Path(__file__).resolve().parent.parent
        log_dir = ws_dir / "log"
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / "rviz_daemon_rviz2.log"
        try:
            self.get_logger().info(f"Starting: {rviz_cmd}")
            with open(log_file, "a") as f:
                f.write("\n=== rviz2 start ===\n")
                f.flush()
                self._rviz_proc = subprocess.Popen(
                    ["bash", "-lc", rviz_cmd],
                    env=env,
                    preexec_fn=os.setsid,
                    stdout=f,
                    stderr=subprocess.STDOUT,
                )
            return True, f"started (pid={self._rviz_proc.pid}); log={log_file}"
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
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            return 130
    finally:
        try:
            node.destroy_node()
        finally:
            # Ctrl+C during spin can lead to shutdown being called already.
            rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

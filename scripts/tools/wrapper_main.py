#!/usr/bin/env python3
from __future__ import annotations

import atexit
import os
import shlex
import signal
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

from tools.wrapper_config import (
    GAZEBO_STARTUP_DELAY,
    controller_plugin,
    enable_chassis_odometry_gt,
    in_docker,
    is_truthy,
    pick_terminal_cmd,
    resolve_overlay_setup,
)
from tools.wrapper_launch import (
    ODIN_SAVE_GRACE_SEC,
    ensure_launch_arg,
    ensure_odin_mode_consistency,
    extract_launch_arg,
    launch_in_terminal,
    save_map_now,
    save_odin_bin_now,
    start_bag_recording,
)
from tools.wrapper_process import PGID_FILES, kill_reality, kill_sim
from tools.wrapper_models import BackgroundGroup, CommonConfig


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print("usage: python3 -m tools.wrapper_main <mode> [extra args...]", file=sys.stderr)
        return 1

    script_name = os.environ.get("WRAPPER_ENTRY_NAME", "").strip() or Path(argv[0]).name
    mode = argv[1]
    extra_args = argv[2:]

    valid_modes = {"sim_mapping", "sim_nav", "reality_mapping", "reality_navigation"}
    if mode not in valid_modes:
        print(f"[{script_name}] Unknown mode '{mode}'. Valid: {sorted(valid_modes)}", file=sys.stderr)
        return 1

    ws_env = os.environ.get("WS_DIR", "").strip()
    ws_dir = Path(ws_env).expanduser() if ws_env else Path(__file__).resolve().parent.parent.parent

    ros_setup = Path(os.environ.get("ROS_SETUP", "/opt/ros/humble/setup.bash"))
    overlay_setup = resolve_overlay_setup(ws_dir)

    for p, label in ((ros_setup, "ROS setup"), (overlay_setup, "workspace overlay")):
        if not p.exists():
            print(f"[{script_name}] Missing {label}: {p}", file=sys.stderr)
            return 1

    kill_existing = is_truthy(os.environ.get("KILL_EXISTING", "1"))
    no_new_terminal_env = os.environ.get("NO_NEW_TERMINAL")
    no_new_terminal = is_truthy(no_new_terminal_env) or (in_docker() and not no_new_terminal_env)

    is_sim = mode.startswith("sim_")
    log_type = "slam" if "mapping" in mode else "nav"
    params_env_key = "SIM_PARAMS_FILE" if is_sim else "REALITY_PARAMS_FILE"
    params_default = (
        ws_dir
        / "src/gxu2026_sentry_nav/gxu2026_nav_bringup/config"
        / ("simulation" if is_sim else "reality")
        / "nav2_params.yaml"
    )
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
    cfg.terminal_cmd = pick_terminal_cmd(cfg)
    if not cfg.terminal_cmd:
        cfg.no_new_terminal = True

    plugin = controller_plugin(cfg.params_file)
    if plugin:
        print(f"[{script_name}] controller_plugin='{plugin}'", file=sys.stderr)
    else:
        print(f"[{script_name}] controller_plugin unset.", file=sys.stderr)

    if kill_existing:
        if is_sim:
            kill_sim(script_name)
        else:
            kill_reality(script_name)

    if is_sim:
        bg = BackgroundGroup(script_name)
        atexit.register(bg.cleanup)

        def _sig(_signum, _frame):
            bg.cleanup()
            raise SystemExit(130)

        signal.signal(signal.SIGINT, _sig)
        signal.signal(signal.SIGTERM, _sig)

        enable_gt = enable_chassis_odometry_gt(cfg.params_file)
        print(f"[{script_name}] enable_chassis_odometry_gt={'true' if enable_gt else 'false'}", file=sys.stderr)

        gazebo_cmd = os.environ.get("GAZEBO_CMD", "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py")
        if "enable_chassis_odometry_gt:=" not in gazebo_cmd:
            gazebo_cmd += f" enable_chassis_odometry_gt:={'true' if enable_gt else 'false'}"

        if "use_gui:=" not in gazebo_cmd:
            headless = (
                is_truthy(os.environ.get("GAZEBO_HEADLESS"))
                or not os.environ.get("DISPLAY", "").strip()
                or cfg.no_new_terminal
            )
            if headless:
                gazebo_cmd = ensure_launch_arg(gazebo_cmd, "use_gui", "false")
                print(
                    f"[{script_name}] Headless Gazebo (DISPLAY={os.environ.get('DISPLAY', '(unset)')!r})",
                    file=sys.stderr,
                )

        if mode == "sim_mapping":
            ros_cmd = os.environ.get(
                "SLAM_CMD",
                "ros2 launch gxu2026_nav_bringup rm_navigation_simulation_launch.py slam:=True",
            )
            fg_title = "SLAM"
        else:
            ros_cmd = os.environ.get(
                "NAV_CMD",
                "ros2 launch gxu2026_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False",
            )
            fg_title = "Nav"

        if extra_args:
            ros_cmd += " " + " ".join(map(shlex.quote, extra_args))

        ros_cmd = ensure_odin_mode_consistency(cfg, mode, ros_cmd)

        is_multi_terminal = bool(cfg.terminal_cmd) and not cfg.no_new_terminal
        if is_multi_terminal:
            launch_in_terminal(cfg, "Gazebo Sim", gazebo_cmd, "")
            time.sleep(1.0)
            launch_in_terminal(cfg, fg_title, ros_cmd, "")
            return 0

        launch_in_terminal(cfg, "Gazebo Sim", gazebo_cmd, "", background=True, bg=bg)
        time.sleep(GAZEBO_STARTUP_DELAY)
        launch_in_terminal(cfg, fg_title, ros_cmd, "", pgid_file=PGID_FILES["sim"])
        return 0

    if mode == "reality_mapping":
        ros_cmd = os.environ.get(
            "MAPPING_CMD",
            "ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True",
        )
        fg_title = "Reality Mapping"
    else:
        ros_cmd = os.environ.get(
            "NAVIGATION_CMD",
            "ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True",
        )
        fg_title = "Reality Navigation"

    if extra_args:
        ros_cmd += " " + " ".join(map(shlex.quote, extra_args))

    ros_cmd = ensure_odin_mode_consistency(cfg, mode, ros_cmd)

    reality_bg = BackgroundGroup(script_name)
    atexit.register(reality_bg.cleanup)

    start_bag_recording(cfg, reality_bg)

    pre_shutdown_hook = None
    if mode == "reality_mapping":
        mapping_ns = extract_launch_arg(ros_cmd, "namespace")
        if mapping_ns is not None:
            mapping_ns = mapping_ns.strip()
            if mapping_ns == "":
                mapping_ns = None

        mapping_ts = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")

        def _mapping_presave() -> None:
            print(f"[{cfg.script_name}] Mapping pre-shutdown save: bin + 2D map", file=sys.stderr)
            save_odin_bin_now(cfg)
            save_map_now(cfg, "reality", mapping_ts, mapping_ns)
            if ODIN_SAVE_GRACE_SEC > 0:
                print(
                    f"[{cfg.script_name}] Wait {ODIN_SAVE_GRACE_SEC:.1f}s for odin map transfer before shutdown",
                    file=sys.stderr,
                )
                time.sleep(ODIN_SAVE_GRACE_SEC)

        pre_shutdown_hook = _mapping_presave

    launch_in_terminal(
        cfg,
        fg_title,
        ros_cmd,
        "",
        pgid_file=PGID_FILES["reality"],
        pre_shutdown_hook=pre_shutdown_hook,
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"[{Path(sys.argv[0]).name}] ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)

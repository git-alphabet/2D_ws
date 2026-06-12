#!/usr/bin/env python3
# Copyright 2025 Alphabet
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Nav2 startup gate — readiness check before triggering lifecycle manager.

Phase 1 (optional): TF warmup — wait for a specific TF transform to be available.
Phase 2: Container readiness — wait for all expected composable nodes to be loaded.
Phase 3: Trigger lifecycle manager startup via service call.

Exit codes:
  0  -- all checks passed
  1  -- timeout or error
"""

import argparse
import os
import subprocess
import sys
import time


# ---------------------------------------------------------------------------
# Expected composable nodes — keep in sync with navigation_runtime.py
# ---------------------------------------------------------------------------
# All composable nodes loaded into the container (for Phase 2).
# nonlinear_spin_publisher is a standalone Node, not in the container.
EXPECTED_CONTAINER_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "sensor_scan_generation",
]



def parse_bool(raw: str) -> bool:
    return str(raw).strip().lower() in {"true", "1", "yes", "on"}


def parse_args():
    p = argparse.ArgumentParser(
        description="Nav2 startup gate: TF warmup + container readiness."
    )
    # TF warmup - read from environment variables with fallback to defaults
    p.add_argument("--tf-warmup-enabled",
                    default=os.environ.get("NAV2_TF_WARMUP_ENABLED", "true"),
                    help="Enable TF warmup phase (default: true).")
    p.add_argument("--tf-target-frame",
                    default=os.environ.get("NAV2_TF_WARMUP_TARGET_FRAME", "map"),
                    help="Target frame for TF warmup (default: map for global_costmap).")
    p.add_argument("--tf-source-frame",
                    default=os.environ.get("NAV2_TF_WARMUP_SOURCE_FRAME", "gimbal_yaw"),
                    help="Source frame for TF warmup (default: gimbal_yaw).")
    p.add_argument("--tf-timeout-sec", type=float,
                    default=float(os.environ.get("NAV2_TF_WARMUP_TIMEOUT_SEC", "25.0")),
                    help="TF warmup timeout; <=0 disables timeout.")
    p.add_argument("--tf-check-hz", type=float,
                    default=float(os.environ.get("NAV2_TF_WARMUP_CHECK_HZ", "20.0")))
    # Container readiness
    p.add_argument("--container-timeout-sec", type=float,
                    default=float(os.environ.get("NAV2_CONTAINER_TIMEOUT_SEC", "30.0")),
                    help="Container readiness timeout (default: 30.0).")
    p.add_argument("--container-check-hz", type=float,
                    default=float(os.environ.get("NAV2_CONTAINER_CHECK_HZ", "2.0")))
    # Common
    p.add_argument("--namespace",
                    default=os.environ.get("NAV2_NAMESPACE", ""))
    p.add_argument("--use-sim-time",
                    default=os.environ.get("NAV2_USE_SIM_TIME", "false"))
    p.add_argument("--container-name",
                    default=os.environ.get("NAV2_CONTAINER_NAME", "nav2_container"))
    p.add_argument("--expected-nodes", type=str, nargs="*", default=None,
                    help="Override expected node names.")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Phase 1: TF warmup
# ---------------------------------------------------------------------------
def phase_tf_warmup(args):
    """Block until the TF chain is available. Returns True if ready, False on timeout."""
    import rclpy
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    import tf2_ros

    ns_value = args.namespace.strip().lstrip("/")
    init_args = ["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]
    if ns_value:
        init_args.extend(["-r", f"__ns:=/{ns_value}"])

    rclpy.init(args=init_args)
    node = rclpy.create_node("nav2_startup_gate_tf")
    node.set_parameters([Parameter("use_sim_time", value=parse_bool(args.use_sim_time))])

    tf_buffer = tf2_ros.Buffer(node=node)
    tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)

    timeout_sec = max(0.0, args.tf_timeout_sec)
    poll_interval = 1.0 / max(1.0, args.tf_check_hz)
    deadline = None if timeout_sec <= 0.0 else (time.monotonic() + timeout_sec)
    ready = False

    if deadline is None:
        node.get_logger().info(
            "TF warmup: waiting '%s' <- '%s', timeout=disabled"
            % (args.tf_target_frame, args.tf_source_frame)
        )
    else:
        node.get_logger().info(
            "TF warmup: waiting '%s' <- '%s', timeout=%.1fs"
            % (args.tf_target_frame, args.tf_source_frame, timeout_sec)
        )

    try:
        while rclpy.ok() and (deadline is None or time.monotonic() <= deadline):
            rclpy.spin_once(node, timeout_sec=poll_interval)
            if tf_buffer.can_transform(
                args.tf_target_frame,
                args.tf_source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                ready = True
                break

        if ready:
            node.get_logger().info(
                "TF warmup: transform '%s' <- '%s' is available"
                % (args.tf_target_frame, args.tf_source_frame)
            )
        else:
            node.get_logger().error(
                "TF warmup: timeout, transform '%s' <- '%s' not ready. "
                "Aborting startup."
                % (args.tf_target_frame, args.tf_source_frame)
            )
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()

    return ready


# ---------------------------------------------------------------------------
# Phase 2: Container readiness
# ---------------------------------------------------------------------------
def get_loaded_nodes(container_fq_name):
    """Query loaded component nodes via 'ros2 component list'."""
    try:
        result = subprocess.run(
            ["ros2", "component", "list", container_fq_name],
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            return None
        names = set()
        for line in result.stdout.strip().splitlines():
            line = line.strip()
            if not line:
                continue
            name = line.rsplit("/", 1)[-1] if "/" in line else line
            names.add(name)
        return names
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return None


def phase_container_ready(args):
    """Block until all expected composable nodes are loaded. Returns True if ready."""
    expected_nodes = set(args.expected_nodes or EXPECTED_CONTAINER_NODES)
    timeout_sec = args.container_timeout_sec
    check_period = 1.0 / max(args.container_check_hz, 0.1)
    ns = (args.namespace or "").strip()

    if ns:
        fq_container = f"/{ns}/{args.container_name}"
    else:
        fq_container = f"/{args.container_name}"

    print(
        f"[startup_gate] Phase 2: waiting for container '{fq_container}' "
        f"({len(expected_nodes)} nodes, timeout={timeout_sec}s)...",
        flush=True,
    )

    t0 = time.monotonic()
    last_log_time = 0

    while True:
        elapsed = time.monotonic() - t0
        if elapsed >= timeout_sec:
            break

        loaded_names = get_loaded_nodes(fq_container)

        if loaded_names is not None:
            missing = expected_nodes - loaded_names
            if not missing:
                print(
                    f"[startup_gate] All {len(expected_nodes)} expected nodes loaded "
                    f"after {elapsed:.1f}s. Total loaded: {len(loaded_names)}.",
                    flush=True,
                )
                return True

            if elapsed - last_log_time >= 1.0:
                last_log_time = elapsed
                print(
                    f"[startup_gate] Waiting for {len(missing)} nodes: "
                    f"{sorted(missing)} ({elapsed:.1f}s / {timeout_sec:.1f}s)",
                    flush=True,
                )
        else:
            if elapsed - last_log_time >= 2.0:
                last_log_time = elapsed
                print(
                    f"[startup_gate] Container '{fq_container}' not reachable yet "
                    f"({elapsed:.1f}s / {timeout_sec:.1f}s)",
                    flush=True,
                )

        time.sleep(check_period)

    # Timeout
    loaded_names = get_loaded_nodes(fq_container)
    if loaded_names is not None:
        missing = expected_nodes - loaded_names
        print(
            f"[startup_gate] TIMEOUT after {timeout_sec:.1f}s. "
            f"Loaded: {len(loaded_names)}, Missing: {sorted(missing)}",
            flush=True,
        )
    else:
        print(
            f"[startup_gate] TIMEOUT after {timeout_sec:.1f}s. "
            f"Container '{fq_container}' was never reachable.",
            flush=True,
        )
    return False


# ---------------------------------------------------------------------------
# Phase 3: Trigger lifecycle manager startup
# ---------------------------------------------------------------------------
def trigger_lifecycle_startup(namespace=""):
    """Call lifecycle_manager/startup service to activate all nodes."""
    import rclpy
    from std_srvs.srv import Trigger

    ns_value = namespace.strip().lstrip("/")
    init_args = ["--ros-args"]
    if ns_value:
        init_args.extend(["-r", f"__ns:=/{ns_value}"])

    try:
        rclpy.init(args=init_args)
    except RuntimeError:
        # Already initialized — shutdown and reinit
        rclpy.shutdown()
        rclpy.init(args=init_args)

    node = rclpy.create_node("startup_gate_trigger")

    # Build service name
    if ns_value:
        service_name = f"/{ns_value}/lifecycle_manager_navigation/startup"
    else:
        service_name = "/lifecycle_manager_navigation/startup"

    print(f"[startup_gate] Phase 3: Calling {service_name}...", flush=True)

    client = node.create_client(Trigger, service_name)

    # Wait for service with timeout
    if not client.wait_for_service(timeout_sec=10.0):
        print(f"[startup_gate] ERROR: Service {service_name} not available after 10s", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return False

    # Call service
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        response = future.result()
        if response.success:
            print(f"[startup_gate] Lifecycle manager started successfully: {response.message}", flush=True)
            node.destroy_node()
            rclpy.shutdown()
            return True
        else:
            print(f"[startup_gate] Lifecycle manager startup failed: {response.message}", flush=True)
    else:
        print(f"[startup_gate] ERROR: Service call failed or timed out", flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return False


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    args = parse_args()

    # Phase 1: TF warmup (optional but blocking on failure)
    if parse_bool(args.tf_warmup_enabled):
        print("[startup_gate] Phase 1: TF warmup...", flush=True)
        if not phase_tf_warmup(args):
            print("[startup_gate] TF warmup failed. Aborting.", flush=True)
            return 1

    # Phase 2: Container readiness (mandatory)
    if not phase_container_ready(args):
        return 1

    # Phase 3: Trigger lifecycle manager startup
    ns = (args.namespace or "").strip()
    if not trigger_lifecycle_startup(ns):
        print("[startup_gate] Failed to trigger lifecycle manager startup. Aborting.", flush=True)
        return 1

    print("[startup_gate] All phases passed. Navigation should start now.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

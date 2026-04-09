#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
import tf2_ros


def parse_bool(raw: str) -> bool:
    return str(raw).strip().lower() in {"true", "1", "yes", "on"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Wait for TF readiness before Nav2 lifecycle startup")
    parser.add_argument("--target-frame", default="odom")
    parser.add_argument("--source-frame", default="gimbal_yaw_fake")
    parser.add_argument("--timeout-sec", type=float, default=25.0)
    parser.add_argument("--check-hz", type=float, default=20.0)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--use-sim-time", default="false")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    ns_value = args.namespace.strip().lstrip("/")
    init_args = ["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"]
    if ns_value:
        init_args.extend(["-r", f"__ns:=/{ns_value}"])

    rclpy.init(args=init_args)
    node = rclpy.create_node("nav2_tf_warmup_waiter")
    node.set_parameters([Parameter("use_sim_time", value=parse_bool(args.use_sim_time))])

    tf_buffer = tf2_ros.Buffer(node=node)
    tf_listener = tf2_ros.TransformListener(tf_buffer, node, spin_thread=False)

    timeout_sec = max(0.0, float(args.timeout_sec))
    check_hz = max(1.0, float(args.check_hz))
    poll_interval = 1.0 / check_hz
    deadline = time.monotonic() + timeout_sec
    ready = False

    node.get_logger().info(
        "TF warmup start: waiting transform '%s' <- '%s', timeout=%.1fs"
        % (args.target_frame, args.source_frame, timeout_sec)
    )

    try:
        while rclpy.ok() and time.monotonic() <= deadline:
            rclpy.spin_once(node, timeout_sec=poll_interval)
            if tf_buffer.can_transform(
                args.target_frame,
                args.source_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                ready = True
                break

        if ready:
            node.get_logger().info(
                "TF warmup ready: transform '%s' <- '%s' is available"
                % (args.target_frame, args.source_frame)
            )
        else:
            node.get_logger().warn(
                "TF warmup timeout: transform '%s' <- '%s' not ready, continue startup"
                % (args.target_frame, args.source_frame)
            )
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

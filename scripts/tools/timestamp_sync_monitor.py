#!/usr/bin/env python3
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
import importlib
import math
import os
import statistics
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
import yaml


def _load_msg_class(type_name: str):
    pkg, msg_part = type_name.split("/msg/")
    module = importlib.import_module(f"{pkg}.msg")
    return getattr(module, msg_part)


def _stamp_to_sec(msg: Any) -> float:
    h = getattr(msg, "header", None)
    if h is None:
        return math.nan
    s = getattr(h.stamp, "sec", None)
    ns = getattr(h.stamp, "nanosec", None)
    if s is None or ns is None:
        return math.nan
    return float(s) + float(ns) * 1e-9


def _percentile(values: list[float], p: float) -> float:
    if not values:
        return math.nan
    if p <= 0.0:
        return min(values)
    if p >= 100.0:
        return max(values)
    arr = sorted(values)
    k = (len(arr) - 1) * p / 100.0
    lo = int(math.floor(k))
    hi = int(math.ceil(k))
    if lo == hi:
        return arr[lo]
    return arr[lo] * (hi - k) + arr[hi] * (k - lo)


@dataclass
class PairState:
    name: str
    left_topic: str
    right_topic: str
    msg_type: str
    tune_param: str | None = None
    left_times: deque[float] = field(default_factory=deque)
    right_times: deque[float] = field(default_factory=deque)
    samples: deque[tuple[float, float]] = field(default_factory=deque)
    left_rx_count: int = 0
    right_rx_count: int = 0
    left_last_rx_mono: float | None = None
    right_last_rx_mono: float | None = None


class SyncMonitor(Node):
    def __init__(self, config: dict[str, Any]):
        super().__init__("timestamp_sync_monitor")

        self.start_mono = time.monotonic()
        self.window_sec = float(config.get("window_sec", 30.0))
        self.report_every_sec = float(config.get("report_every_sec", 5.0))
        self.max_pair_dt_sec = float(config.get("max_pair_dt_sec", 0.2))
        self.source_detect_sec = float(config.get("source_detect_sec", 8.0))
        qos_mode = str(config.get("qos", "sensor_data")).strip().lower()
        if qos_mode == "sensor_data":
            qos = QoSPresetProfiles.SENSOR_DATA.value
        else:
            qos = QoSPresetProfiles.SYSTEM_DEFAULT.value

        self.pairs: list[PairState] = []
        # Keep strong references to subscriptions to prevent garbage collection.
        # Avoid Node.reserved property names (e.g. 'subscriptions').
        self._subs = []

        raw_pairs = config.get("pairs", [])
        if not isinstance(raw_pairs, list) or not raw_pairs:
            raise RuntimeError("config.pairs is empty")

        for idx, item in enumerate(raw_pairs):
            if not isinstance(item, dict):
                raise RuntimeError(f"pairs[{idx}] is not a map")
            pair = PairState(
                name=str(item.get("name", f"pair_{idx}")),
                left_topic=str(item["left_topic"]),
                right_topic=str(item["right_topic"]),
                msg_type=str(item["msg_type"]),
                tune_param=str(item.get("tune_param")) if item.get("tune_param") else None,
            )
            self.pairs.append(pair)

            msg_cls = _load_msg_class(pair.msg_type)
            self._subs.append(
                self.create_subscription(
                    msg_cls,
                    pair.left_topic,
                    self._make_cb(pair, "left"),
                    qos,
                )
            )
            self._subs.append(
                self.create_subscription(
                    msg_cls,
                    pair.right_topic,
                    self._make_cb(pair, "right"),
                    qos,
                )
            )

        self.create_timer(self.report_every_sec, self._report)

        self.get_logger().info(
            f"monitor started: window={self.window_sec:.1f}s report={self.report_every_sec:.1f}s max_pair_dt={self.max_pair_dt_sec:.3f}s"
        )
        for p in self.pairs:
            self.get_logger().info(
                f"pair={p.name} type={p.msg_type} left={p.left_topic} right={p.right_topic}"
            )

    def _make_cb(self, pair: PairState, side: str):
        def _cb(msg: Any) -> None:
            ts = _stamp_to_sec(msg)
            if not math.isfinite(ts):
                return
            now = time.monotonic()

            if side == "left":
                pair.left_rx_count += 1
                pair.left_last_rx_mono = now
            else:
                pair.right_rx_count += 1
                pair.right_last_rx_mono = now

            current = pair.left_times if side == "left" else pair.right_times
            other = pair.right_times if side == "left" else pair.left_times
            current.append(ts)

            min_keep = ts - self.window_sec
            while current and current[0] < min_keep:
                current.popleft()
            while other and other[0] < min_keep:
                other.popleft()

            if not other:
                return

            nearest = min(other, key=lambda v: abs(v - ts))
            abs_dt = abs(ts - nearest)
            if abs_dt > self.max_pair_dt_sec:
                return

            if side == "left":
                signed = ts - nearest
            else:
                signed = nearest - ts

            pair.samples.append((now, signed))
            old_limit = now - self.window_sec
            while pair.samples and pair.samples[0][0] < old_limit:
                pair.samples.popleft()

        return _cb

    def _report(self) -> None:
        now = time.monotonic()
        for pair in self.pairs:
            if not pair.samples:
                left_live = bool(pair.left_last_rx_mono) and (now - pair.left_last_rx_mono <= self.window_sec)
                right_live = bool(pair.right_last_rx_mono) and (now - pair.right_last_rx_mono <= self.window_sec)

                if now - self.start_mono < self.source_detect_sec:
                    self.get_logger().info(
                        f"[{pair.name}] waiting source detection... left={pair.left_rx_count} right={pair.right_rx_count}"
                    )
                    continue

                if not left_live and not right_live:
                    self.get_logger().info(
                        f"[{pair.name}] skipped: both sources inactive, left={pair.left_topic} right={pair.right_topic}"
                    )
                    continue

                if left_live and not right_live:
                    self.get_logger().info(
                        f"[{pair.name}] skipped: right source inactive ({pair.right_topic}), evaluate left-source-only chain first"
                    )
                    continue

                if right_live and not left_live:
                    self.get_logger().info(
                        f"[{pair.name}] skipped: left source inactive ({pair.left_topic})"
                    )
                    continue

                self.get_logger().warn(
                    f"[{pair.name}] both sources active but no matched samples within max_pair_dt={self.max_pair_dt_sec:.3f}s"
                )
                continue

            signed_vals = [v for _, v in pair.samples]
            abs_vals = [abs(v) for v in signed_vals]

            p50 = _percentile(abs_vals, 50.0)
            p95 = _percentile(abs_vals, 95.0)
            mx = max(abs_vals)
            mean_signed = statistics.fmean(signed_vals)

            self.get_logger().info(
                f"[{pair.name}] n={len(abs_vals)} abs_dt(s): p50={p50:.4f} p95={p95:.4f} max={mx:.4f} mean_signed={mean_signed:+.4f}"
            )

            if pair.tune_param:
                step = min(0.01, max(0.001, abs(mean_signed) * 0.5))
                if mean_signed > 0.01:
                    self.get_logger().warn(
                        f"[{pair.name}] right stream tends older than left; try decreasing {pair.tune_param} by {step:.4f}s"
                    )
                elif mean_signed < -0.01:
                    self.get_logger().warn(
                        f"[{pair.name}] right stream tends newer than left; try increasing {pair.tune_param} by {step:.4f}s"
                    )
                else:
                    self.get_logger().info(
                        f"[{pair.name}] mean_signed within +/-10ms; keep {pair.tune_param} unchanged"
                    )


def _default_config_path() -> Path:
    cfg_env = os.environ.get("TIMESTAMP_SYNC_MONITOR_CONFIG", "").strip()
    if cfg_env:
        return Path(cfg_env).expanduser()
    return Path(__file__).resolve().parent.parent / "config" / "timestamp_sync_monitor.yaml"


def _load_config(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError("config root must be a map")
    return data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Timestamp sync monitor")
    parser.add_argument(
        "--config",
        default=str(_default_config_path()),
        help="Path to YAML config",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cfg_path = Path(args.config).expanduser().resolve()
    if not cfg_path.is_file():
        print(f"config not found: {cfg_path}", file=sys.stderr)
        return 2

    cfg = _load_config(cfg_path)

    rclpy.init()
    node = None
    try:
        node = SyncMonitor(cfg)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
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
from sensor_msgs_py import point_cloud2
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


def _pointcloud2_max_field(msg: Any, field_name: str) -> float:
    try:
        values = point_cloud2.read_points(msg, field_names=[field_name], skip_nans=True)
        max_value = math.nan
        for item in values:
            value = item[0] if isinstance(item, tuple) else item[field_name]
            value = float(value)
            if not math.isfinite(value):
                continue
            if not math.isfinite(max_value) or value > max_value:
                max_value = value
        return max_value
    except Exception:
        return math.nan


def _end_stamp_to_sec(msg: Any, base_stamp: float, field_name: str, mode: str) -> float:
    if not field_name or not mode:
        return math.nan

    mode = mode.strip().lower()
    if mode == "livox_custom_offset_ns":
        points = getattr(msg, "points", None)
        if points is None:
            return math.nan
        max_offset_ns = math.nan
        for point in points:
            value = float(getattr(point, field_name, math.nan))
            if not math.isfinite(value):
                continue
            if not math.isfinite(max_offset_ns) or value > max_offset_ns:
                max_offset_ns = value
        if not math.isfinite(max_offset_ns):
            return math.nan
        return base_stamp + max_offset_ns * 1e-9

    max_field = _pointcloud2_max_field(msg, field_name)
    if not math.isfinite(max_field):
        return math.nan
    if mode == "absolute_sec":
        return max_field
    if mode == "relative_sec":
        return base_stamp + max_field
    if mode == "relative_ns":
        return base_stamp + max_field * 1e-9
    return math.nan


@dataclass
class TopicState:
    name: str
    topic: str
    msg_type: str
    end_stamp_field: str = ""
    end_stamp_mode: str = ""
    samples: deque[tuple[float, float, float, float]] = field(default_factory=deque)
    rx_count: int = 0
    last_rx_mono: float | None = None


@dataclass
class ChainState:
    name: str
    topics: list[str]


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

        self.topics: list[TopicState] = []
        self.topic_by_name_or_topic: dict[str, TopicState] = {}
        raw_topics = config.get("topics", [])
        if isinstance(raw_topics, list):
            for idx, item in enumerate(raw_topics):
                if isinstance(item, str):
                    name = item
                    topic = item
                    msg_type = "sensor_msgs/msg/PointCloud2"
                elif isinstance(item, dict):
                    topic = str(item["topic"])
                    name = str(item.get("name", topic))
                    msg_type = str(item.get("msg_type", "sensor_msgs/msg/PointCloud2"))
                    end_stamp_field = str(item.get("end_stamp_field", ""))
                    end_stamp_mode = str(item.get("end_stamp_mode", ""))
                else:
                    raise RuntimeError(f"topics[{idx}] must be a string or map")
                if isinstance(item, str):
                    end_stamp_field = ""
                    end_stamp_mode = ""

                state = TopicState(
                    name=name,
                    topic=topic,
                    msg_type=msg_type,
                    end_stamp_field=end_stamp_field,
                    end_stamp_mode=end_stamp_mode,
                )
                self.topics.append(state)
                self.topic_by_name_or_topic[name] = state
                self.topic_by_name_or_topic[topic] = state

        self.pairs: list[PairState] = []
        self.chains: list[ChainState] = []
        # Keep strong references to subscriptions to prevent garbage collection.
        # Avoid Node.reserved property names (e.g. 'subscriptions').
        self._subs = []

        for topic_state in self.topics:
            msg_cls = _load_msg_class(topic_state.msg_type)
            self._subs.append(
                self.create_subscription(
                    msg_cls,
                    topic_state.topic,
                    self._make_topic_cb(topic_state),
                    qos,
                )
            )

        raw_pairs = config.get("pairs", [])
        if isinstance(raw_pairs, list):
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

        raw_chains = config.get("chains", [])
        if isinstance(raw_chains, list):
            for idx, item in enumerate(raw_chains):
                if not isinstance(item, dict):
                    raise RuntimeError(f"chains[{idx}] is not a map")
                topics = [str(v) for v in item.get("topics", [])]
                if len(topics) < 2:
                    raise RuntimeError(f"chains[{idx}].topics must contain at least two topics")
                self.chains.append(ChainState(name=str(item.get("name", f"chain_{idx}")), topics=topics))

        if not self.topics and not self.pairs:
            raise RuntimeError("config must contain at least one topic or pair")

        self.create_timer(self.report_every_sec, self._report)
        self.get_logger().info(
            f"monitor started: window={self.window_sec:.1f}s report={self.report_every_sec:.1f}s max_pair_dt={self.max_pair_dt_sec:.3f}s"
        )
        for t in self.topics:
            end_stamp = ""
            if t.end_stamp_field and t.end_stamp_mode:
                end_stamp = f" end_stamp={t.end_stamp_field}/{t.end_stamp_mode}"
            self.get_logger().info(
                f"topic={t.name} type={t.msg_type} topic={t.topic}{end_stamp}"
            )
        for p in self.pairs:
            self.get_logger().info(
                f"pair={p.name} type={p.msg_type} left={p.left_topic} right={p.right_topic}"
            )
        for c in self.chains:
            self.get_logger().info(f"chain={c.name} topics={' -> '.join(c.topics)}")

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _make_topic_cb(self, topic: TopicState):
        def _cb(msg: Any) -> None:
            ts = _stamp_to_sec(msg)
            if not math.isfinite(ts):
                return
            now_mono = time.monotonic()
            now_ros = self._now_sec()
            age = now_ros - ts
            end_ts = _end_stamp_to_sec(
                msg, ts, topic.end_stamp_field, topic.end_stamp_mode
            )
            end_age = now_ros - end_ts if math.isfinite(end_ts) else math.nan
            topic.rx_count += 1
            topic.last_rx_mono = now_mono
            topic.samples.append((now_mono, ts, age, end_age))

            old_limit = now_mono - self.window_sec
            while topic.samples and topic.samples[0][0] < old_limit:
                topic.samples.popleft()

        return _cb

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

    @staticmethod
    def _topic_summary(topic: TopicState) -> dict[str, float] | None:
        if not topic.samples:
            return None

        ages = [age for _, _, age, _ in topic.samples if math.isfinite(age)]
        if not ages:
            return None

        rx_times = [mono for mono, _, _, _ in topic.samples]
        duration = max(rx_times) - min(rx_times) if len(rx_times) >= 2 else 0.0
        rate = (len(rx_times) - 1) / duration if duration > 0.0 else math.nan
        summary = {
            "n": float(len(ages)),
            "rate": rate,
            "mean": statistics.fmean(ages),
            "p50": _percentile(ages, 50.0),
            "p95": _percentile(ages, 95.0),
            "max": max(ages),
            "last": ages[-1],
        }
        gaps = [
            rx_times[idx] - rx_times[idx - 1]
            for idx in range(1, len(rx_times))
            if rx_times[idx] >= rx_times[idx - 1]
        ]
        if gaps:
            summary.update(
                {
                    "gap_mean": statistics.fmean(gaps),
                    "gap_p95": _percentile(gaps, 95.0),
                    "gap_max": max(gaps),
                    "gap_last": gaps[-1],
                }
            )

        end_ages = [end_age for _, _, _, end_age in topic.samples if math.isfinite(end_age)]
        if end_ages:
            summary.update(
                {
                    "end_mean": statistics.fmean(end_ages),
                    "end_p50": _percentile(end_ages, 50.0),
                    "end_p95": _percentile(end_ages, 95.0),
                    "end_max": max(end_ages),
                    "end_last": end_ages[-1],
                }
            )
        return summary

    def _report(self) -> None:
        now = time.monotonic()

        topic_summaries: dict[str, dict[str, float]] = {}
        for topic in self.topics:
            summary = self._topic_summary(topic)
            live = bool(topic.last_rx_mono) and (now - topic.last_rx_mono <= self.window_sec)
            if summary is None:
                if now - self.start_mono < self.source_detect_sec:
                    self.get_logger().info(
                        f"[topic:{topic.name}] waiting source detection... rx={topic.rx_count} topic={topic.topic}"
                    )
                else:
                    self.get_logger().info(
                        f"[topic:{topic.name}] inactive rx={topic.rx_count} topic={topic.topic}"
                    )
                continue

            topic_summaries[topic.name] = summary
            topic_summaries[topic.topic] = summary
            live_flag = "live" if live else "stale"
            self.get_logger().info(
                f"[topic:{topic.name}] {live_flag} n={summary['n']:.0f} rate={summary['rate']:.2f}Hz "
                f"age(s): last={summary['last']:.3f} mean={summary['mean']:.3f} "
                f"p50={summary['p50']:.3f} p95={summary['p95']:.3f} max={summary['max']:.3f}"
            )
            if "end_mean" in summary:
                self.get_logger().info(
                    f"[topic:{topic.name}] end_age(s): last={summary['end_last']:.3f} "
                    f"mean={summary['end_mean']:.3f} p50={summary['end_p50']:.3f} "
                    f"p95={summary['end_p95']:.3f} max={summary['end_max']:.3f}"
                )
            if "gap_mean" in summary:
                self.get_logger().info(
                    f"[topic:{topic.name}] rx_gap(s): last={summary['gap_last']:.3f} "
                    f"mean={summary['gap_mean']:.3f} p95={summary['gap_p95']:.3f} "
                    f"max={summary['gap_max']:.3f}"
                )

        for chain in self.chains:
            missing = [topic for topic in chain.topics if topic not in topic_summaries]
            if missing:
                self.get_logger().info(
                    f"[chain:{chain.name}] insufficient samples; missing={','.join(missing)}"
                )
                continue

            mean_parts = []
            last_parts = []
            prev_topic = chain.topics[0]
            prev_mean = topic_summaries[prev_topic]["mean"]
            prev_last = topic_summaries[prev_topic]["last"]
            for topic in chain.topics[1:]:
                mean = topic_summaries[topic]["mean"]
                last = topic_summaries[topic]["last"]
                mean_parts.append(f"{prev_topic}->{topic}:{mean - prev_mean:+.3f}s")
                last_parts.append(f"{prev_topic}->{topic}:{last - prev_last:+.3f}s")
                prev_topic = topic
                prev_mean = mean
                prev_last = last

            self.get_logger().info(
                f"[chain:{chain.name}] mean_age_delta {' '.join(mean_parts)}"
            )
            self.get_logger().info(
                f"[chain:{chain.name}] last_age_delta {' '.join(last_parts)}"
            )

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

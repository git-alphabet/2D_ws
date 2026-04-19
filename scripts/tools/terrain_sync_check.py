#!/usr/bin/env python3
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
import yaml


@dataclass
class TopicStats:
    topic: str
    delays: list[float] = field(default_factory=list)


@dataclass
class PairStats:
    name: str
    left_topic: str
    right_topic: str
    left_stamps: list[float] = field(default_factory=list)
    right_stamps: list[float] = field(default_factory=list)
    stamp_diffs: list[float] = field(default_factory=list)


class TerrainSyncChecker(Node):
    def __init__(self, cfg: dict[str, Any]):
        super().__init__("terrain_sync_check")

        self.duration_sec = float(cfg.get("duration_sec", 12.0))
        self.match_window_sec = float(cfg.get("match_window_sec", 0.08))
        self.threshold_sec = float(cfg.get("threshold_sec", 0.05))

        qos_mode = str(cfg.get("qos", "sensor_data")).strip().lower()
        if qos_mode == "system_default":
            qos = QoSPresetProfiles.SYSTEM_DEFAULT.value
        else:
            qos = QoSPresetProfiles.SENSOR_DATA.value

        topic_list = cfg.get("topics", [])
        if not isinstance(topic_list, list) or not topic_list:
            raise RuntimeError("topics is empty")

        pair_list = cfg.get("pairs", [])
        if not isinstance(pair_list, list) or not pair_list:
            raise RuntimeError("pairs is empty")

        self.topic_stats: dict[str, TopicStats] = {}
        self.pair_stats: list[PairStats] = []
        self._subs = []

        for topic in topic_list:
            t = str(topic).strip()
            if not t:
                continue
            self.topic_stats[t] = TopicStats(topic=t)
            self._subs.append(
                self.create_subscription(
                    PointCloud2,
                    t,
                    lambda msg, tt=t: self._topic_cb(tt, msg),
                    qos,
                )
            )

        for raw in pair_list:
            if not isinstance(raw, dict):
                continue
            name = str(raw.get("name", "pair")).strip() or "pair"
            left = str(raw.get("left_topic", "")).strip()
            right = str(raw.get("right_topic", "")).strip()
            if not left or not right:
                continue
            p = PairStats(name=name, left_topic=left, right_topic=right)
            self.pair_stats.append(p)
            self._subs.append(
                self.create_subscription(
                    PointCloud2,
                    left,
                    lambda msg, pp=p: self._pair_cb_left(pp, msg),
                    qos,
                )
            )
            self._subs.append(
                self.create_subscription(
                    PointCloud2,
                    right,
                    lambda msg, pp=p: self._pair_cb_right(pp, msg),
                    qos,
                )
            )

        if not self.topic_stats:
            raise RuntimeError("no valid topics")
        if not self.pair_stats:
            raise RuntimeError("no valid pairs")

    @staticmethod
    def _stamp_to_sec(msg: PointCloud2) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _topic_cb(self, topic: str, msg: PointCloud2) -> None:
        stamp = self._stamp_to_sec(msg)
        delay = time.time() - stamp
        self.topic_stats[topic].delays.append(delay)

    def _pair_cb_left(self, pair: PairStats, msg: PointCloud2) -> None:
        pair.left_stamps.append(self._stamp_to_sec(msg))
        self._try_match(pair)

    def _pair_cb_right(self, pair: PairStats, msg: PointCloud2) -> None:
        pair.right_stamps.append(self._stamp_to_sec(msg))
        self._try_match(pair)

    def _try_match(self, pair: PairStats) -> None:
        if not pair.left_stamps or not pair.right_stamps:
            return

        l = pair.left_stamps[-1]
        nearest = min(pair.right_stamps, key=lambda x: abs(x - l))
        d = abs(l - nearest)
        if d <= self.match_window_sec:
            pair.stamp_diffs.append(d)

        max_len = 256
        if len(pair.left_stamps) > max_len:
            del pair.left_stamps[: len(pair.left_stamps) - max_len]
        if len(pair.right_stamps) > max_len:
            del pair.right_stamps[: len(pair.right_stamps) - max_len]

    def run_and_report(self) -> int:
        start = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start) < self.duration_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        print("=== Terrain Sync Check ===")
        print(f"duration_sec={self.duration_sec:.1f}")
        print(f"threshold_sec={self.threshold_sec:.3f}")

        for topic, stat in self.topic_stats.items():
            if not stat.delays:
                print(f"topic {topic}: n=0")
                continue
            print(
                f"topic {topic}: n={len(stat.delays)} avg={statistics.mean(stat.delays):.3f}s "
                f"min={min(stat.delays):.3f}s max={max(stat.delays):.3f}s"
            )

        all_pass = True
        for pair in self.pair_stats:
            left = self.topic_stats.get(pair.left_topic)
            right = self.topic_stats.get(pair.right_topic)

            if left is None or right is None or not left.delays or not right.delays:
                print(f"pair {pair.name}: insufficient delay samples")
                all_pass = False
                continue

            diff_delay = abs(statistics.mean(left.delays) - statistics.mean(right.delays))

            if pair.stamp_diffs:
                avg_stamp_diff = statistics.mean(pair.stamp_diffs)
                max_stamp_diff = max(pair.stamp_diffs)
                stamp_info = f"stamp_avg_diff={avg_stamp_diff:.3f}s stamp_max_diff={max_stamp_diff:.3f}s"
            else:
                stamp_info = "stamp_avg_diff=NaN stamp_max_diff=NaN"

            ok = diff_delay < self.threshold_sec
            all_pass = all_pass and ok
            print(
                f"pair {pair.name}: delay_diff={diff_delay:.3f}s {stamp_info} "
                f"threshold={self.threshold_sec:.3f}s result={'PASS' if ok else 'FAIL'}"
            )

        print(f"final_result={'PASS' if all_pass else 'FAIL'}")
        return 0 if all_pass else 1


def _default_config_path() -> Path:
    env_path = os.environ.get("TERRAIN_SYNC_CHECK_CONFIG", "").strip()
    if env_path:
        return Path(env_path).expanduser()
    return Path(__file__).resolve().parent.parent / "config" / "terrain_sync_check.yaml"


def _load_config(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise RuntimeError("config root must be a map")
    return data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="terrain sync checker")
    parser.add_argument("--config", default=str(_default_config_path()), help="YAML config path")
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
        node = TerrainSyncChecker(cfg)
        return node.run_and_report()
    except KeyboardInterrupt:
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
RMUC 标定点辅助节点 —— 单个 PublishPoint 工具 + 自动轮转模式。

使用方法：
  1. 在 rviz 工具栏选择 "标定点 Calib" (PublishPoint)
  2. 终端会显示当前要标定的点名称，在地图上单击即可标定
  3. 标定完一个点后自动跳到下一个
  4. 发布 /calib/select 话题 (std_msgs/String) 可跳到指定点：
       ros2 topic pub --once /calib/select std_msgs/msg/String "data: supply_zone"
  5. 发布 /calib/prev 或 /calib/next (std_msgs/Empty) 可上翻/下翻

话题:
  订阅: /calib/clicked_point  (geometry_msgs/msg/PointStamped) — rviz 点击
  订阅: /calib/select          (std_msgs/msg/String) — 跳到指定标定点
  订阅: /calib/prev, /calib/next (std_msgs/msg/Empty) — 上翻/下翻
  发布: /calibrated_points     (visualization_msgs/msg/MarkerArray) — 十字标志+标签
  发布: /calib/current_label   (visualization_msgs/msg/MarkerArray) — 当前待标定提示
"""

import argparse
import csv
import os
import sys
from collections import OrderedDict
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Empty
from visualization_msgs.msg import Marker, MarkerArray

# ── 标定点定义 ──────────────────────────────────────────────
# (key, display_name, color_rgba)
CALIB_POINTS: list[tuple[str, str, tuple[float, float, float, float]]] = [
    ("supply_zone",            "1-补给区 Supply",            (0.0, 0.6, 1.0, 1.0)),
    # ── 以下暂时隐藏，需要时取消注释即可 ──
    # ("outpost_buff",           "2-前哨增益 OutpostBuff",     (0.8, 0.0, 0.8, 1.0)),
    # ("base_buff",              "3-基地增益 BaseBuff",        (1.0, 0.8, 0.0, 1.0)),
    # ("fortress_ally",          "4-堡垒区 Fortress",          (1.0, 0.0, 0.0, 1.0)),
    ("central_highland",       "5-中央高地 CentralHL",       (1.0, 1.0, 0.0, 1.0)),
    ("ladder_highland",        "6-梯形高地 LadderHL",        (0.0, 1.0, 1.0, 1.0)),
    # ("defend_anchor",          "7-防御锚点 Defend",          (1.0, 0.4, 0.4, 1.0)),
    # ── 巡逻点（前哨站被毁后，在梯形高地附近巡逻的路点）──
    ("patrol_1",               "P1-巡逻点1 Patrol1",        (0.4, 1.0, 0.4, 1.0)),
    ("patrol_2",               "P2-巡逻点2 Patrol2",        (0.4, 1.0, 0.6, 1.0)),
    ("patrol_3",               "P3-巡逻点3 Patrol3",        (0.4, 1.0, 0.8, 1.0)),
]

# ── rmuc_calibration.csv 默认路径 ──
DEFAULT_CSV_PATH = os.path.join(
    os.path.dirname(__file__), "..", "src", "RM_Behavior_Tree",
    "rm_behavior_tree", "config", "RMUC_2026", "rmuc_calibration.csv"
)

MAP_FRAME = "map"
CROSS_HALF_SIZE = 0.4
CROSS_LINE_WIDTH = 0.08
TEXT_HEIGHT = 0.30
TEXT_OFFSET_Z = 0.08  # ← 标签离十字的高度 (越小越贴近十字)
MARKER_Z = 0.05


class CalibPointHelper(Node):
    def __init__(self, csv_path: str):
        super().__init__("calib_point_helper")

        self.calibrated: OrderedDict[str, tuple[float, float]] = OrderedDict()
        self.csv_path = csv_path
        self.current_idx = 0

        # 启动时从 CSV 加载已有的标定数据
        self._load_csv()

        # 发布标记
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, "/calibrated_points", latching_qos)
        self.hint_pub = self.create_publisher(MarkerArray, "/calib/current_label", latching_qos)

        # 订阅 rviz 点击
        self.create_subscription(PointStamped, "/calib/clicked_point", self._on_click, 10)
        # 订阅控制话题
        self.create_subscription(String, "/calib/select", self._on_select, 10)
        self.create_subscription(Empty, "/calib/next", self._on_next, 10)
        self.create_subscription(Empty, "/calib/prev", self._on_prev, 10)

        # 如果有已标定的点，跳到第一个未标定的
        self._advance_to_first_uncalibrated()
        if self.calibrated:
            self._publish_all_markers()
        self._print_current_hint()
        self._publish_hint_marker()

        # 等待数据的周期提示
        self._interaction_started = False
        self._wait_timer = self.create_timer(5.0, self._wait_callback)

        loaded = len(self.calibrated)
        total = len(CALIB_POINTS)
        if loaded > 0:
            loaded_keys = ", ".join(self.calibrated.keys())
            self.get_logger().info(
                f"[CALIB] 标定辅助节点已启动 | 从 CSV 加载了 {loaded}/{total} 个旧坐标\n"
                f"  已有: {loaded_keys}\n"
                f"  点击地图可覆盖更新对应坐标\n"
                f"  CSV: {csv_path}"
            )
        else:
            self.get_logger().info(
                f"[CALIB] 标定辅助节点已启动 | 共 {total} 个标定点，从空白开始\n"
                f"  CSV: {csv_path}"
            )

    def _current_point(self) -> tuple[str, str, tuple[float, float, float, float]]:
        return CALIB_POINTS[self.current_idx]

    def _advance_to_first_uncalibrated(self):
        """启动时跳到第一个还没标定过的点。"""
        for idx, (key, _, _) in enumerate(CALIB_POINTS):
            if key not in self.calibrated:
                self.current_idx = idx
                return

    def _print_current_hint(self):
        key, name, _ = self._current_point()
        total = len(CALIB_POINTS)
        done = len(self.calibrated)
        if key in self.calibrated:
            cx, cy = self.calibrated[key]
            status = f"  当前值: ({cx:.3f}, {cy:.3f})  ← 点击地图可覆盖更新"
        else:
            status = "  ← 未标定，在地图上点击标定位置"
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"  [CALIB] 当前标定: {name} ({self.current_idx+1}/{total})\n"
            f"  已完成: {done}/{total}\n"
            f"{status}\n"
            f"  切换: ros2 topic pub --once /calib/next std_msgs/msg/Empty '{{}}'\n"
            f"{'='*50}"
        )

    def _wait_callback(self):
        """仿真已启动但还没开始标定时，只在 DEBUG 级别打印。"""
        if self._interaction_started:
            self._wait_timer.cancel()
            return
        self.get_logger().debug("[CALIB] 等待标定操作...")

    def _on_click(self, msg: PointStamped):
        self._interaction_started = True
        key, name, color = self._current_point()
        x, y = msg.point.x, msg.point.y
        self.calibrated[key] = (x, y)
        self.get_logger().info(f"[CALIB] ✓ {name}: ({x:.3f}, {y:.3f})")

        self._publish_all_markers()
        self._save_csv()

        # 自动跳到下一个未标定的点
        self._advance_to_next()
        self._print_current_hint()
        self._publish_hint_marker()

    def _advance_to_next(self):
        """跳到下一个还未标定的点；如果全部已标定则循环到下一个。"""
        total = len(CALIB_POINTS)
        # 优先找未标定的点
        for offset in range(1, total + 1):
            idx = (self.current_idx + offset) % total
            key = CALIB_POINTS[idx][0]
            if key not in self.calibrated:
                self.current_idx = idx
                return
        # 全部已标定 → 循环到下一个，方便逐个重新标定
        self.current_idx = (self.current_idx + 1) % total
        self.get_logger().info(
            f"[CALIB] ✅ 所有 {total} 个标定点均已有坐标，继续轮转可重新标定"
        )

    def _on_select(self, msg: String):
        target = msg.data.strip()
        for idx, (key, name, _) in enumerate(CALIB_POINTS):
            if key == target or target in name:
                self.current_idx = idx
                self._print_current_hint()
                self._publish_hint_marker()
                return
        self.get_logger().warn(f"[CALIB] 未知标定点: '{target}'")

    def _on_next(self, _msg: Empty):
        self.current_idx = (self.current_idx + 1) % len(CALIB_POINTS)
        self._print_current_hint()
        self._publish_hint_marker()

    def _on_prev(self, _msg: Empty):
        self.current_idx = (self.current_idx - 1) % len(CALIB_POINTS)
        self._print_current_hint()
        self._publish_hint_marker()

    def _publish_hint_marker(self):
        """在 rviz 地图正上方显示当前待标定点名称。"""
        key, name, color = self._current_point()
        r, g, b, a = color
        status = "✓ 已标定" if key in self.calibrated else "← 待标定"

        ma = MarkerArray()
        hint = Marker()
        hint.header.frame_id = MAP_FRAME
        hint.header.stamp = self.get_clock().now().to_msg()
        hint.ns = "calib_hint"
        hint.id = 0
        hint.type = Marker.TEXT_VIEW_FACING
        hint.action = Marker.ADD
        hint.pose.position.x = 8.0
        hint.pose.position.y = 0.0
        hint.pose.position.z = 3.0
        hint.pose.orientation.w = 1.0
        hint.scale.z = 0.8
        hint.color.r = r
        hint.color.g = g
        hint.color.b = b
        hint.color.a = 1.0
        hint.text = f"[{self.current_idx+1}/{len(CALIB_POINTS)}] {name} {status}"
        ma.markers.append(hint)
        self.hint_pub.publish(ma)

    def _publish_all_markers(self):
        """发布所有已标定点的十字标志 + 名称标签。"""
        ma = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        now = self.get_clock().now().to_msg()

        for idx, (key, display_name, color) in enumerate(CALIB_POINTS):
            if key not in self.calibrated:
                continue
            cx, cy = self.calibrated[key]
            r, g, b, a = color

            # ── 十字标志 ──
            cross = Marker()
            cross.header.frame_id = MAP_FRAME
            cross.header.stamp = now
            cross.ns = f"calib_{key}"
            cross.id = idx * 2
            cross.type = Marker.LINE_LIST
            cross.action = Marker.ADD
            cross.pose.orientation.w = 1.0
            cross.scale.x = CROSS_LINE_WIDTH
            cross.color.r = r
            cross.color.g = g
            cross.color.b = b
            cross.color.a = a
            cross.points.append(Point(x=cx - CROSS_HALF_SIZE, y=cy, z=MARKER_Z))
            cross.points.append(Point(x=cx + CROSS_HALF_SIZE, y=cy, z=MARKER_Z))
            cross.points.append(Point(x=cx, y=cy - CROSS_HALF_SIZE, z=MARKER_Z))
            cross.points.append(Point(x=cx, y=cy + CROSS_HALF_SIZE, z=MARKER_Z))
            ma.markers.append(cross)

            # ── 名称标签 ──
            label = Marker()
            label.header.frame_id = MAP_FRAME
            label.header.stamp = now
            label.ns = f"calib_{key}_label"
            label.id = idx * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = TEXT_OFFSET_Z
            label.pose.orientation.w = 1.0
            label.scale.z = TEXT_HEIGHT
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"({cx:.2f}, {cy:.2f})\n{display_name}"
            ma.markers.append(label)

        self.marker_pub.publish(ma)

    # ── CSV 读写 ──────────────────────────────────────────

    def _load_csv(self):
        """启动时从 CSV 加载已有标定坐标。"""
        path = Path(self.csv_path)
        if not path.exists():
            self.get_logger().info(f"[CALIB] CSV 文件不存在，从空白开始: {path}")
            return

        known_keys = {pt[0] for pt in CALIB_POINTS}
        loaded = 0
        try:
            with open(path, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.split(",")
                    if len(parts) < 3:
                        continue
                    name = parts[0].strip()
                    if name == "point_name":  # 跳过表头
                        continue
                    if name not in known_keys:
                        continue
                    try:
                        x = float(parts[1].strip())
                        y = float(parts[2].strip())
                        self.calibrated[name] = (x, y)
                        loaded += 1
                    except ValueError:
                        pass
            if loaded:
                self.get_logger().info(f"[CALIB] 从 CSV 加载了 {loaded} 个已有标定点")
        except Exception as e:
            self.get_logger().warn(f"[CALIB] 读取 CSV 失败: {e}")

    def _save_csv(self):
        """直接覆写 rmuc_calibration.csv，保留原有注释头。"""
        try:
            path = Path(self.csv_path)
            path.parent.mkdir(parents=True, exist_ok=True)

            # 读取原文件中的注释头部分 (巡逻航点等非标定内容也保留)
            header_lines = []
            patrol_lines = []
            if path.exists():
                with open(path, "r") as f:
                    in_patrol_section = False
                    for line in f:
                        stripped = line.strip()
                        if "巡逻航点" in stripped:
                            in_patrol_section = True
                        if in_patrol_section:
                            patrol_lines.append(line)
                        elif stripped.startswith("#") or stripped == "":
                            header_lines.append(line)
                        # 跳过旧的数据行（会被新数据覆盖）

            with open(path, "w") as f:
                # 写注释头（如果原文件有则保留，否则新建）
                if header_lines:
                    for line in header_lines:
                        f.write(line)
                else:
                    f.write("# RMUC 2026 标定坐标文件\n")
                    f.write("# 格式: 点位名称,x,y\n")
                    f.write("# 以 # 开头的行为注释\n")
                    f.write("#\n")
                    f.write("# 使用方法:\n")
                    f.write("#   1. 启动 sim_mapping.sh, 标定辅助节点会自动启动\n")
                    f.write("#   2. 在 rviz 工具栏选择 \"标定点 Calib\", 点击地图标定\n")
                    f.write("#   3. 跳到指定标定点: ros2 topic pub --once /calib/select std_msgs/msg/String \"data: fortress_ally\"\n")
                    f.write("#   4. 上/下翻: ros2 topic pub --once /calib/next std_msgs/msg/Empty '{}'\n")
                    f.write("#\n")

                # 写所有标定点数据（已标定的写坐标，未标定的写注释占位）
                for key, display_name, _ in CALIB_POINTS:
                    if key in self.calibrated:
                        x, y = self.calibrated[key]
                        f.write(f"{key},{x:.4f},{y:.4f}\n")
                    else:
                        f.write(f"# {key},0.0,0.0\n")

                # 保留巡逻航点段
                if patrol_lines:
                    f.write("\n")
                    for line in patrol_lines:
                        f.write(line)

            self.get_logger().info(f"[CALIB] CSV 已保存: {path} ({len(self.calibrated)} 点)")
        except Exception as e:
            self.get_logger().error(f"[CALIB] CSV 保存失败: {e}")


def main(args=None):
    parser = argparse.ArgumentParser(description="RMUC 标定点辅助节点")
    parser.add_argument("--csv", type=str, default=None,
                        help="CSV 文件路径 (默认: rmuc_calibration.csv)")
    parser.add_argument("--no-csv", action="store_true",
                        help="不保存 CSV")
    known, unknown = parser.parse_known_args()

    csv_path = known.csv
    if not csv_path and not known.no_csv:
        # 默认写到 rmuc_calibration.csv
        resolved = Path(DEFAULT_CSV_PATH).resolve()
        if resolved.parent.exists():
            csv_path = str(resolved)
        else:
            csv_path = None

    rclpy.init(args=args)
    node = CalibPointHelper(csv_path=csv_path if not known.no_csv else "")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

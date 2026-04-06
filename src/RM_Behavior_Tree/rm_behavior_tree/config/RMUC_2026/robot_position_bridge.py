#!/usr/bin/env python3
"""
robot_position_bridge.py — 仿真环境下的 /robot_position 发布桥接节点

功能：
  - 从 TF (map → base_footprint) 获取机器人位姿
  - 监听 Nav2 的 navigate_to_pose action 状态判断是否到达目标
  - 以 50Hz 发布 /robot_position (RMUCRobotPosition)

用法：
  source ~/important_code/ros2_ws/install/setup.bash
  python3 robot_position_bridge.py

  # 自定义参数：
  python3 robot_position_bridge.py --ros-args \
      -p map_frame:=map \
      -p base_frame:=base_footprint \
      -p publish_rate:=50.0
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatusArray, GoalStatus
from sp_msgs.msg import RMUCRobotPosition


class RobotPositionBridge(Node):
    def __init__(self):
        super().__init__('robot_position_bridge')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('nav_action', '/navigate_to_pose')

        rate = self.get_parameter('publish_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        nav_action = self.get_parameter('nav_action').value

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action status → 判断 is_at_nav_goal
        self.is_at_nav_goal = True
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            f'{nav_action}/_action/status',
            self._status_cb,
            qos)

        # 发布 /robot_position
        self.pub = self.create_publisher(RMUCRobotPosition, '/robot_position', qos)

        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'[Bridge] TF: {self.map_frame} → {self.base_frame} | '
            f'Nav2: {nav_action} | rate: {rate} Hz')

    def _status_cb(self, msg: GoalStatusArray):
        if not msg.status_list:
            self.is_at_nav_goal = True
            return
        latest = msg.status_list[-1]
        if latest.status in (GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_ACCEPTED):
            self.is_at_nav_goal = False
        else:
            self.is_at_nav_goal = True

    def _timer_cb(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return

        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        msg = RMUCRobotPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose_x = float(t.transform.translation.x)
        msg.pose_y = float(t.transform.translation.y)
        msg.pose_yaw = float(yaw)
        msg.is_at_nav_goal = self.is_at_nav_goal
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

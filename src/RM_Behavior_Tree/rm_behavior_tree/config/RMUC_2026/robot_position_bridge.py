#!/usr/bin/env python3
"""
robot_position_bridge.py — 仿真环境下的 robot_position 发布桥接节点

功能：
  - 从 TF (map → base_footprint) 获取机器人位姿
  - 监听 Nav2 的 navigate_to_pose action 状态判断是否到达目标
  - 以 50Hz 发布 robot_position (RMUCRobotPosition)，跟随 namespace

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
from action_msgs.msg import GoalStatus, GoalStatusArray
from tf2_ros import Buffer, TransformListener
from sp_msgs.msg import RMUCRobotPosition


class RobotPositionBridge(Node):
    def __init__(self):
        super().__init__('robot_position_bridge')

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'base_frame_fallbacks',
            ['gimbal_yaw_fake', 'gimbal_yaw', 'chassis'])

        rate = self.get_parameter('publish_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.base_frame_fallbacks = [
            str(frame) for frame in self.get_parameter('base_frame_fallbacks').value
            if str(frame)
        ]
        self.lookup_frames = [self.base_frame]
        for frame in self.base_frame_fallbacks:
            if frame not in self.lookup_frames:
                self.lookup_frames.append(frame)
        self.is_at_nav_goal = True

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布 robot_position (相对话题名，跟随 namespace)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(RMUCRobotPosition, 'robot_position', qos)
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            'navigate_to_pose/_action/status',
            self._nav_status_cb,
            10)

        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'[Bridge] TF: {self.map_frame} → {self.lookup_frames} | rate: {rate} Hz')

    def _nav_status_cb(self, msg):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        self.is_at_nav_goal = not any(status.status in active_states for status in msg.status_list)

    def _timer_cb(self):
        transform = None
        for frame in self.lookup_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, frame, rclpy.time.Time())
                break
            except Exception:
                pass

        if transform is None:
            return

        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        msg = RMUCRobotPosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose_x = float(transform.transform.translation.x)
        msg.pose_y = float(transform.transform.translation.y)
        msg.is_at_nav_goal = bool(self.is_at_nav_goal)
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

#!/usr/bin/env bash
set -e

# 用法：
#   ./scripts/pub_auto_aim_yaw_test.sh                  # 发布到 /auto_aim_yaw
#   ./scripts/pub_auto_aim_yaw_test.sh /red_standard_robot1  # 发布到 /red_standard_robot1/auto_aim_yaw

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS="${1:-}"

# ROS 的 setup 脚本里会读取未定义变量（如 AMENT_TRACE_SETUP_FILES）。
# 如果开启 nounset（set -u）会直接报错退出，所以这里临时关闭。
set +u
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/local_setup.bash"
set -u

# 关键：让 ros2 CLI 能解析到 sp_msgs/msg/Float32Stamped
export AMENT_PREFIX_PATH="$WS_ROOT/install/sp_msgs:${AMENT_PREFIX_PATH:-}"

TOPIC="/auto_aim_yaw"
if [[ -n "$NS" ]]; then
  TOPIC="$NS$TOPIC"
fi

echo "Publishing sp_msgs/msg/Float32Stamped -> $TOPIC"
export TOPIC

# 默认发布一个正弦波（幅值 1.0 rad，10Hz 频率采样，约 0.2Hz 波形）
python3 - <<'PY'
import os
import time
import math

import rclpy
from rclpy.node import Node

try:
  from sp_msgs.msg import Float32Stamped
except Exception as e:
  raise SystemExit(f"Failed to import sp_msgs.msg.Float32Stamped: {e}")

TOPIC = os.environ.get('TOPIC', '/auto_aim_yaw')

class SinePub(Node):
  def __init__(self):
    super().__init__('auto_aim_yaw_test_publisher')
    self.pub = self.create_publisher(Float32Stamped, TOPIC, 10)

def main():
  rclpy.init()
  node = SinePub()
  amp = 1.0
  freq = 0.2  # Hz of the sine wave
  rate_hz = 200.0
  period = 1.0 / rate_hz
  try:
    while rclpy.ok():
      t = time.time()
      value = amp * math.sin(2.0 * math.pi * freq * t)
      msg = Float32Stamped()
      msg.data = float(value)
      msg.header.stamp = node.get_clock().now().to_msg()
      node.pub.publish(msg)
      rclpy.spin_once(node, timeout_sec=0.0)
      time.sleep(period)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
PY

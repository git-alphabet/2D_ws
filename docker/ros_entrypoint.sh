#!/usr/bin/env bash
set -e

ROS_DISTRO=${ROS_DISTRO:-humble}

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -f "/ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "/ws/install/setup.bash"
fi

# 让交互式 shell（Attach Shell）也能自动 source ROS 环境
# 写入 .bashrc，只写一次
if ! grep -q "ros/humble/setup.bash" ~/.bashrc 2>/dev/null; then
  echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
  echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> ~/.bashrc
fi

exec "$@"

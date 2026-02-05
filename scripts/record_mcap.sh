#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

: "${ROS_DISTRO:=humble}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
fi

if [[ $# -lt 1 ]]; then
  cat <<'EOF'
Usage:
  ./scripts/record_mcap.sh <output_name_or_dir> [topic1 topic2 ...]

Examples:
  ./scripts/record_mcap.sh run1 /tf /tf_static /odom /scan
  ./scripts/record_mcap.sh run_all --all

Notes:
- This uses rosbag2 with MCAP storage: `ros2 bag record -s mcap ...`
- If your system doesn't have the MCAP storage plugin, install:
    sudo apt install ros-$ROS_DISTRO-rosbag2-storage-mcap
- Foxglove Studio can open the generated .mcap file directly.
EOF
  exit 2
fi

OUT="$1"
shift || true

# Allow passing through flags like --all.
exec ros2 bag record -s mcap -o "$OUT" "$@"

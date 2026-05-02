#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# bag 回放导航固定使用仿真时钟，并关闭自动录包，避免回放时又录一层 bag。
export REALITY_USE_SIM_TIME="True"
export AUTO_RECORD_BAG="0"
unset NAVIGATION_CMD

exec "$SCRIPT_DIR/nav.sh" "$@"

#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# odin1 bag 回放建图：固定使用仿真时钟，并关闭自动录包，避免回放时又录一层 bag。
export REALITY_USE_SIM_TIME="True"
unset MAPPING_CMD

exec "$SCRIPT_DIR/odin1_mapping.sh" "$@"

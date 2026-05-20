#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  ./scripts/rmuc_map_calib.sh <map.yaml|map.pgm> [rviz_config]

Purpose:
  Open a static map in RViz and start the RMUC calibration helper.
  By default, only points enabled in both calib_point_helper.py and
  uncommented rmuc_calibration.csv rows are calibrated.
  Clicking calibration points updates both:
    1) rmuc_calibration.csv
    2) rmuc_2026_params.yaml

Examples:
  ./scripts/rmuc_map_calib.sh src/gxu2026_sentry_nav/gxu2026_nav_bringup/map/simulation/rmuc_2025.yaml
  ./scripts/rmuc_map_calib.sh /ws/src/gxu2026_sentry_nav/gxu2026_nav_bringup/map/simulation/rmuc_2025.pgm

Environment overrides:
  CALIB_CSV_PATH     default: RMUC_2026/rmuc_calibration.csv
  RMUC_PARAMS_PATH   default: RMUC_2026/rmuc_2026_params.yaml
  RVIZ_CONFIG        default: gxu2026_nav_bringup/rviz/nav2_default_view.rviz
EOF
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

MAP_ARG="$1"
RVIZ_CONFIG_ARG="${2:-${RVIZ_CONFIG:-}}"

resolve_path() {
  local path="$1"
  if [[ "$path" = /* ]]; then
    printf '%s\n' "$path"
  else
    printf '%s\n' "$WS_DIR/$path"
  fi
}

MAP_PATH="$(resolve_path "$MAP_ARG")"
case "$MAP_PATH" in
  *.pgm)
    YAML_CANDIDATE="${MAP_PATH%.pgm}.yaml"
    if [[ ! -f "$YAML_CANDIDATE" ]]; then
      echo "[rmuc_map_calib] ERROR: got pgm, but sibling yaml not found: $YAML_CANDIDATE" >&2
      exit 1
    fi
    MAP_YAML="$YAML_CANDIDATE"
    ;;
  *.yaml|*.yml)
    MAP_YAML="$MAP_PATH"
    ;;
  *)
    echo "[rmuc_map_calib] ERROR: map file must be .yaml/.yml or .pgm: $MAP_PATH" >&2
    exit 1
    ;;
esac

if [[ ! -f "$MAP_YAML" ]]; then
  echo "[rmuc_map_calib] ERROR: map yaml not found: $MAP_YAML" >&2
  exit 1
fi

BT_CONFIG_DIR="$WS_DIR/src/RM_Behavior_Tree/rm_behavior_tree/config/RMUC_2026"
CALIB_CSV_PATH="${CALIB_CSV_PATH:-$BT_CONFIG_DIR/rmuc_calibration.csv}"
RMUC_PARAMS_PATH="${RMUC_PARAMS_PATH:-$BT_CONFIG_DIR/rmuc_2026_params.yaml}"
RVIZ_CONFIG_ARG="${RVIZ_CONFIG_ARG:-$WS_DIR/src/gxu2026_sentry_nav/gxu2026_nav_bringup/rviz/nav2_default_view.rviz}"

if [[ ! -f "$RMUC_PARAMS_PATH" ]]; then
  echo "[rmuc_map_calib] ERROR: params file not found: $RMUC_PARAMS_PATH" >&2
  exit 1
fi
if [[ ! -f "$RVIZ_CONFIG_ARG" ]]; then
  echo "[rmuc_map_calib] ERROR: rviz config not found: $RVIZ_CONFIG_ARG" >&2
  exit 1
fi

export QT_FONT_DPI="${QT_FONT_DPI:-192}"

source_setup() {
  local setup_file="$1"
  # ROS/colcon setup scripts read optional variables such as
  # AMENT_TRACE_SETUP_FILES and COLCON_TRACE before defining them.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
fi

BRANCH_NAME="${BUILD_PROFILE:-}"
if [[ -z "$BRANCH_NAME" && -f "$WS_DIR/.git/HEAD" ]]; then
  git_head="$(<"$WS_DIR/.git/HEAD")"
  if [[ "$git_head" == ref:\ refs/heads/* ]]; then
    BRANCH_NAME="${git_head#ref: refs/heads/}"
  fi
fi
BRANCH_SAFE="$(echo "${BRANCH_NAME:-Alphabet}" | sed 's#[^A-Za-z0-9._-]#_#g')"
INSTALL_SETUP="$WS_DIR/.buildcache/$BRANCH_SAFE/install/setup.bash"
if [[ -f "$INSTALL_SETUP" ]]; then
  source_setup "$INSTALL_SETUP"
elif [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source_setup "$WS_DIR/install/setup.bash"
fi

pids=()
cleanup() {
  local code=$?
  echo "[rmuc_map_calib] Cleaning up..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait 2>/dev/null || true
  exit "$code"
}
trap cleanup EXIT INT TERM

echo "[rmuc_map_calib] map yaml: $MAP_YAML"
echo "[rmuc_map_calib] csv:      $CALIB_CSV_PATH"
echo "[rmuc_map_calib] params:   $RMUC_PARAMS_PATH"
echo "[rmuc_map_calib] rviz:     $RVIZ_CONFIG_ARG"
echo "[rmuc_map_calib] note:     only uncommented points in calib_point_helper.py / rmuc_calibration.csv are calibrated."

ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:="$MAP_YAML" \
  -p topic_name:=map \
  -p frame_id:=map &
pids+=("$!")

ros2 run nav2_util lifecycle_bringup map_server &
pids+=("$!")

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
pids+=("$!")

python3 "$SCRIPT_DIR/calib_point_helper.py" \
  --csv "$CALIB_CSV_PATH" \
  --params "$RMUC_PARAMS_PATH" \
  --sync-params &
pids+=("$!")

rviz2 -d "$RVIZ_CONFIG_ARG" &
pids+=("$!")

wait "${pids[-1]}"

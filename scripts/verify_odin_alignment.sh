#!/usr/bin/env bash
set -euo pipefail

# 配置区：支持通过环境变量覆盖，默认值可直接用于当前仓库。
CONTAINER_NAME="${CONTAINER_NAME:-gxu2026-nav-laptop}"
NAMESPACE="${NAMESPACE:-}"
WS_IN_CONTAINER="${WS_IN_CONTAINER:-/ws}"

NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-$WS_IN_CONTAINER/src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/reality/nav2_params.yaml}"
ODIN_CTRL_FILE="${ODIN_CTRL_FILE:-$WS_IN_CONTAINER/src/odin_ros_driver/config/control_command.yaml}"
SENSOR_SCAN_SRC="${SENSOR_SCAN_SRC:-$WS_IN_CONTAINER/src/gxu2026_sentry_nav/sensor_scan_generation/src/sensor_scan_generation.cpp}"

ODOM_TOPIC="${ODOM_TOPIC:-/odin1/odometry_highfreq}"
CLOUD_TOPIC="${CLOUD_TOPIC:-/odin1/cloud_slam}"
BASE_FRAME="${BASE_FRAME:-base_footprint}"
LIDAR_FRAME="${LIDAR_FRAME:-front_odin1}"
ROBOT_FRAME="${ROBOT_FRAME:-gimbal_yaw}"

if ! command -v docker >/dev/null 2>&1; then
  echo "[ERROR] docker command not found"
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  mapfile -t running_containers < <(docker ps --format '{{.Names}}')
  if [[ ${#running_containers[@]} -eq 1 ]]; then
    CONTAINER_NAME="${running_containers[0]}"
    echo "[WARN] CONTAINER_NAME not running, fallback to the only running container: $CONTAINER_NAME"
  else
    echo "[ERROR] container '$CONTAINER_NAME' is not running"
    echo "[INFO] running containers:"
    docker ps --format '  - {{.Names}}'
    exit 1
  fi
fi

run_in_container() {
  local cmd="$1"
  docker exec "$CONTAINER_NAME" bash -lc "set -e; source /opt/ros/humble/setup.bash; cd '$WS_IN_CONTAINER'; if [[ -f install/setup.bash ]]; then source install/setup.bash; fi; $cmd"
}

if [[ -n "$NAMESPACE" ]]; then
  TF_ROS_ARGS="--ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=$NAMESPACE"
else
  TF_ROS_ARGS=""
fi

echo "[INFO] ===== verify_odin_alignment ====="
echo "[INFO] container: $CONTAINER_NAME"
echo "[INFO] workspace: $WS_IN_CONTAINER"
echo "[INFO] namespace: ${NAMESPACE:-<empty>}"
echo "[INFO] odom topic: $ODOM_TOPIC"
echo "[INFO] cloud topic: $CLOUD_TOPIC"
echo "[INFO] frames: odom -> $BASE_FRAME -> $ROBOT_FRAME -> $LIDAR_FRAME"

echo
echo "[STEP] 1/5 Dump key config values"
run_in_container "python3 - <<'PY'
import yaml

def load_yaml(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}

nav2 = load_yaml('$NAV2_PARAMS_FILE')
ctrl = load_yaml('$ODIN_CTRL_FILE')

switches = nav2.get('pb_navigation_switches', {}).get('ros__parameters', {})
sensor = nav2.get('sensor_scan_generation', {}).get('ros__parameters', {})
ctrl_keys = ctrl.get('register_keys', {})

print('[nav2] odometry_source =', switches.get('odometry_source'))
print('[nav2] robot_description_file =', switches.get('robot_description_file'))
print('[nav2] terrain_registered_scan_topic =', switches.get('terrain_registered_scan_topic'))
print('[nav2] terrain_lidar_odometry_topic =', switches.get('terrain_lidar_odometry_topic'))
print('[nav2] sensor_scan_registered_scan_topic =', switches.get('sensor_scan_registered_scan_topic'))
print('[nav2] sensor_scan_lidar_odometry_topic =', switches.get('sensor_scan_lidar_odometry_topic'))
print('[nav2] sensor_scan_generation.lidar_frame =', sensor.get('lidar_frame'))
print('[nav2] sensor_scan_generation.base_frame =', sensor.get('base_frame'))
print('[nav2] sensor_scan_generation.robot_base_frame =', sensor.get('robot_base_frame'))
print('[nav2] sensor_scan_generation.publish_base_tf =', sensor.get('publish_base_tf'))
print('[odin] send_odom_baselink_tf =', ctrl_keys.get('send_odom_baselink_tf'))
print('[odin] robot_base_frame_id =', ctrl_keys.get('robot_base_frame_id'))
PY"

echo
echo "[STEP] 2/5 Validate sensor_scan_generation TF direction in source"
run_in_container "grep -n 'tf_odom_child_to_lidar' '$SENSOR_SCAN_SRC' && grep -n 'getTransform(lidar_frame_, odom_child_frame' '$SENSOR_SCAN_SRC'"

echo
echo "[STEP] 3/5 Sample odometry/cloud headers"
run_in_container "echo '[topic list snapshot]'; ros2 topic list | grep -E '^${ODOM_TOPIC}$|^${CLOUD_TOPIC}$' || true"
run_in_container "echo '[odometry sample]'; timeout 6s ros2 topic echo --once ${ODOM_TOPIC} | sed -n '1,20p' || true"
run_in_container "echo '[cloud sample]'; timeout 6s ros2 topic echo --once ${CLOUD_TOPIC} | sed -n '1,20p' || true"

echo
echo "[STEP] 4/5 Check key TF links"
run_in_container "echo '[tf] odom -> ${BASE_FRAME}'; timeout 6s ros2 run tf2_ros tf2_echo odom ${BASE_FRAME} ${TF_ROS_ARGS} | sed -n '1,25p' || true"
run_in_container "echo '[tf] ${BASE_FRAME} -> ${ROBOT_FRAME}'; timeout 6s ros2 run tf2_ros tf2_echo ${BASE_FRAME} ${ROBOT_FRAME} ${TF_ROS_ARGS} | sed -n '1,25p' || true"
run_in_container "echo '[tf] ${ROBOT_FRAME} -> ${LIDAR_FRAME}'; timeout 6s ros2 run tf2_ros tf2_echo ${ROBOT_FRAME} ${LIDAR_FRAME} ${TF_ROS_ARGS} | sed -n '1,25p' || true"

echo
echo "[STEP] 5/5 Optional TF tree command"
if [[ -n "$NAMESPACE" ]]; then
  echo "ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=$NAMESPACE"
else
  echo "ros2 run rqt_tf_tree rqt_tf_tree"
fi

echo
echo "[DONE] verify_odin_alignment finished"
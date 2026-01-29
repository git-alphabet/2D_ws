#!/bin/bash
set -euo pipefail

# Launch simulator and Nav2, enabling the NeuPAN virtualenv only for the controller.

export QT_FONT_DPI=120
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
OVERLAY_SETUP=${OVERLAY_SETUP:-$WS_DIR/install/setup.bash}
NAV_PARAMS_FILE=${NAV_PARAMS_FILE:-$WS_DIR/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml}
NEUPAN_VENV=${NEUPAN_VENV:-$WS_DIR/neupan_env}
NEUPAN_ACTIVATE="$NEUPAN_VENV/bin/activate"
NEUPAN_SITE_PACKAGES="$NEUPAN_VENV/lib/python3.10/site-packages"
# Optional legacy hook for external model bundles; leave empty when using packaged models.
NEUPAN_MODEL_SETUP=${NEUPAN_MODEL_SETUP:-}
RCUTILS_LOGGING_SEVERITY=${RCUTILS_LOGGING_SEVERITY:-INFO}
RQT_GRAPH=${RQT_GRAPH:-false}
RQT_GRAPH_NAMESPACE=${RQT_GRAPH_NAMESPACE:-}
RQT_GRAPH_ARGS=${RQT_GRAPH_ARGS:-}

if [[ ! -f $ROS_SETUP ]]; then
	echo "[simulation_nav.sh] Missing ROS setup: $ROS_SETUP" >&2
	exit 1
fi

if [[ ! -f $OVERLAY_SETUP ]]; then
	echo "[simulation_nav.sh] Missing workspace overlay: $OVERLAY_SETUP" >&2
	exit 1
fi

is_truthy() {
	local value=${1:-}
	case "${value,,}" in
		1|true|yes|on) return 0 ;;
		*) return 1 ;;
	esac
}

in_docker() {
	[[ -f /.dockerenv ]] && return 0
	grep -qaE '(docker|containerd)' /proc/1/cgroup 2>/dev/null
}

NO_NEW_TERMINAL=${NO_NEW_TERMINAL:-}
if [[ -z ${NO_NEW_TERMINAL} ]] && in_docker; then
	NO_NEW_TERMINAL=1
fi

BG_PIDS=()

cleanup_bg() {
	if ! is_truthy "${NO_NEW_TERMINAL:-}"; then
		return 0
	fi
	if [[ ${#BG_PIDS[@]} -eq 0 ]]; then
		return 0
	fi

	echo "[simulation_nav.sh] Cleaning up background processes..." >&2
	for pid in "${BG_PIDS[@]}"; do
		if [[ -z ${pid} ]]; then
			continue
		fi
		if kill -0 "$pid" 2>/dev/null; then
			kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
		fi
	done
	sleep 1
	for pid in "${BG_PIDS[@]}"; do
		if [[ -n ${pid} ]] && kill -0 "$pid" 2>/dev/null; then
			kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
		fi
	done
}

trap cleanup_bg INT TERM EXIT

TERMINAL_CMD=${TERMINAL_CMD:-}

if is_truthy "$NO_NEW_TERMINAL"; then
	TERMINAL_CMD=""
	elif [[ -n $TERMINAL_CMD ]]; then
	if ! command -v "$TERMINAL_CMD" >/dev/null 2>&1; then
		echo "[simulation_nav.sh] Requested terminal '$TERMINAL_CMD' not found." >&2
		exit 1
	fi
else
	if command -v gnome-terminal >/dev/null 2>&1; then
		TERMINAL_CMD="gnome-terminal"
	elif command -v x-terminal-emulator >/dev/null 2>&1; then
		TERMINAL_CMD="x-terminal-emulator"
	else
		NO_NEW_TERMINAL=1
	fi
fi

BASE_ENV="source '$ROS_SETUP'; source '$OVERLAY_SETUP'; export RCUTILS_LOGGING_SEVERITY='$RCUTILS_LOGGING_SEVERITY'"
NEUPAN_ENV=""

python_with_ros_env() {
	(
		set +u
		source "$ROS_SETUP"
		source "$OVERLAY_SETUP"
		set -u
		python3 - "$@"
	)
}



controller_plugin="$(python_with_ros_env "$NAV_PARAMS_FILE" <<'PY'
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1])
if not cfg_path.exists():
	sys.exit(0)

try:
	import yaml  # type: ignore
except Exception:
	sys.exit(0)

data = yaml.safe_load(cfg_path.read_text()) or {}
switches = (data.get("pb_navigation_switches") or {}).get("ros__parameters") or {}
plugin = switches.get("controller_plugin")
if isinstance(plugin, str):
	plugin = plugin.strip()
	if plugin:
		print(plugin)
PY
)"

enable_chassis_odometry_gt="$(python_with_ros_env "$NAV_PARAMS_FILE" <<'PY'
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1])
if not cfg_path.exists():
	print("true")
	raise SystemExit(0)

try:
	import yaml  # type: ignore
except Exception:
	print("true")
	raise SystemExit(0)

data = yaml.safe_load(cfg_path.read_text()) or {}
switches = (data.get("pb_navigation_switches") or {}).get("ros__parameters") or {}
value = switches.get("enable_chassis_odometry_gt", True)
print("true" if bool(value) else "false")
PY
)"

echo "[simulation_nav.sh] enable_chassis_odometry_gt=${enable_chassis_odometry_gt} (from ${NAV_PARAMS_FILE})" >&2

if [[ "$controller_plugin" == "neupan_nav2_controller" ]]; then
	if [[ ! -f $NEUPAN_ACTIVATE ]]; then
		echo "[simulation_nav.sh] NeuPAN virtualenv not found at $NEUPAN_ACTIVATE" >&2
		exit 1
	fi

	NEUPAN_ENV="source '$NEUPAN_ACTIVATE'"
	if [[ -d $NEUPAN_SITE_PACKAGES ]]; then
		NEUPAN_ENV="$NEUPAN_ENV; export PYTHONPATH=\$PYTHONPATH:'$NEUPAN_SITE_PACKAGES'"
	fi

	if [[ -n $NEUPAN_MODEL_SETUP ]]; then
		if [[ -f $NEUPAN_MODEL_SETUP ]]; then
			NEUPAN_ENV="$NEUPAN_ENV; source '$NEUPAN_MODEL_SETUP'"
		else
			echo "[simulation_nav.sh] Warning: $NEUPAN_MODEL_SETUP not found; skipping model setup." >&2
		fi
	fi
else
	echo "[simulation_nav.sh] controller_plugin='${controller_plugin:-unset}'; NeuPAN virtualenv will not be activated." >&2
fi

bt_report="$(python_with_ros_env "$NAV_PARAMS_FILE" <<'PY'
import os
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1])
if not cfg_path.exists():
	sys.exit(0)

try:
	import yaml  # type: ignore
except Exception as exc:
	print(f"[simulation_nav.sh] Warning: unable to read behavior tree from {cfg_path}: {exc}")
	sys.exit(0)

try:
	from ament_index_python.packages import (
		PackageNotFoundError,
		get_package_share_directory,
	)
except Exception:
	class PackageNotFoundError(Exception):
		pass

	def get_package_share_directory(_pkg: str) -> str:
		raise PackageNotFoundError("ament_index_python not available")


def _get_ros_params(container, key):
	entry = container.get(key) if isinstance(container, dict) else None
	if isinstance(entry, dict):
		params = entry.get("ros__parameters")
		if isinstance(params, dict):
			return params
	return {}


def _resolve_style(style_value):
	default_style = "rmuc_01.xml"
	candidate = (style_value or "").strip() or default_style
	if candidate.startswith("$("):
		return candidate, False, "substitution"
	expanded = os.path.expanduser(candidate)
	base_dir = cfg_path.parent
	if os.path.isabs(expanded):
		resolved = Path(expanded)
		return str(resolved), resolved.exists(), "absolute"
	pkg_name = None
	relative_path = expanded
	if ":" in expanded:
		pkg_part, rel_part = expanded.split(":", 1)
		pkg_part = pkg_part.strip()
		if pkg_part:
			pkg_name = pkg_part
			relative_path = rel_part.lstrip("/") or default_style
	if pkg_name:
		try:
			share_dir = Path(get_package_share_directory(pkg_name))
		except PackageNotFoundError as exc:
			return candidate, False, f"package '{pkg_name}' not found ({exc})"
		resolved = share_dir / relative_path
		return str(resolved), resolved.exists(), "package"
	if relative_path.startswith("./") or relative_path.startswith("../"):
		resolved = (base_dir / relative_path).resolve()
		return str(resolved), resolved.exists(), "relative"
	try:
		share_dir = Path(get_package_share_directory("rm_behavior_tree"))
	except PackageNotFoundError as exc:
		return candidate, False, f"package 'rm_behavior_tree' not found ({exc})"
	if os.path.sep in relative_path:
		resolved = share_dir / relative_path
	else:
		resolved = share_dir / "config" / relative_path
	return str(resolved), resolved.exists(), "default-package"


root = yaml.safe_load(cfg_path.read_text()) or {}
switches = _get_ros_params(root, "pb_navigation_switches")
rm_bt = _get_ros_params(root, "rm_behavior_tree")
behavior_tree_selector = switches.get("behavior_tree") if isinstance(switches, dict) else None
behavior_tree_selector = behavior_tree_selector.strip() if isinstance(behavior_tree_selector, str) else ""
enable_rm_bt = bool(switches.get("enable_rm_behavior_tree", False)) if isinstance(switches, dict) else False
style_value = rm_bt.get("style", "rmuc_01.xml") if isinstance(rm_bt, dict) else "rmuc_01.xml"

if behavior_tree_selector:
	lowered = behavior_tree_selector.lower()
	if lowered in {"disabled", "none", "nav2", "default"}:
		enable_rm_bt = False
	else:
		enable_rm_bt = True
		style_value = behavior_tree_selector
else:
	enable_rm_bt = enable_rm_bt and bool(rm_bt)

if not enable_rm_bt:
	flag = switches.get("enable_rm_behavior_tree") if isinstance(switches, dict) else False
	reason = behavior_tree_selector or ("enable_rm_behavior_tree=" + ("true" if flag else "false"))
	print(f"[simulation_nav.sh] Behavior tree disabled (selector='{reason}')")
	sys.exit(0)

resolved_path, exists, origin = _resolve_style(style_value)
status = "found" if exists else "missing"
print(
	f"[simulation_nav.sh] Behavior tree enabled via YAML ({origin}); style='{style_value}' => {resolved_path} [{status}]"
)
PY
)"

if [[ -n ${bt_report:-} ]]; then
	echo "$bt_report"
fi

launch_in_terminal() {
	local title="$1"
	local command="$2"
	local extra_env="$3"
	local background="${4:-}"

	echo "[simulation_nav.sh] Launching $title: $command"

	local full_cmd="cd '$WS_DIR'; $BASE_ENV"
	if [[ -n $extra_env ]]; then
		full_cmd="$full_cmd; $extra_env"
	fi
	full_cmd="$full_cmd; $command"

	if is_truthy "$NO_NEW_TERMINAL"; then
		local log_dir="$WS_DIR/log"
		mkdir -p "$log_dir"
		local slug
		slug="$(echo "$title" | tr '[:upper:]' '[:lower:]' | sed -E 's/[^a-z0-9]+/_/g; s/^_+|_+$//g')"
		local log_file="$log_dir/${SCRIPT_NAME%.*}_${slug}.log"
		if [[ "$background" == "bg" ]]; then
			echo "[simulation_nav.sh] (single-terminal) $title -> $log_file" >&2
			if command -v setsid >/dev/null 2>&1; then
				setsid bash -lc "$full_cmd" >"$log_file" 2>&1 &
			else
				bash -lc "$full_cmd" >"$log_file" 2>&1 &
			fi
			BG_PIDS+=("$!")
			return 0
		fi
		echo "[simulation_nav.sh] (single-terminal) $title (foreground)" >&2
		echo "[simulation_nav.sh] Log: $log_file" >&2
		bash -lc "$full_cmd" 2>&1 | tee -a "$log_file"
		return $?
	fi

	full_cmd="$full_cmd; exec bash"

	case "$TERMINAL_CMD" in
		gnome-terminal)
			gnome-terminal --title="$title" -- bash -c "$full_cmd"
			;;
		x-terminal-emulator)
			x-terminal-emulator -T "$title" -e bash -lc "$full_cmd"
			;;
		*)
			"$TERMINAL_CMD" -T "$title" -e bash -lc "$full_cmd"
			;;
	esac
}

GAZEBO_CMD=${GAZEBO_CMD:-"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"}
if [[ "$GAZEBO_CMD" != *"enable_chassis_odometry_gt:="* ]]; then
	GAZEBO_CMD="$GAZEBO_CMD enable_chassis_odometry_gt:=${enable_chassis_odometry_gt}"
fi
NAV_CMD=${NAV_CMD:-"ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False"}

launch_in_terminal "Gazebo Sim" "$GAZEBO_CMD" "" "bg"
sleep 1
launch_in_terminal "Nav" "$NAV_CMD" "$NEUPAN_ENV"

if is_truthy "$RQT_GRAPH"; then
	rqt_env=""
	if [[ -n $RQT_GRAPH_NAMESPACE ]]; then
		rqt_env="export ROS_NAMESPACE='$RQT_GRAPH_NAMESPACE'"
	fi
	rqt_cmd="rqt_graph"
	if [[ -n $RQT_GRAPH_ARGS ]]; then
		rqt_cmd="$rqt_cmd $RQT_GRAPH_ARGS"
	fi
	launch_in_terminal "rqt_graph" "$rqt_cmd" "$rqt_env"
fi

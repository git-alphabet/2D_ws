#!/bin/bash

set -euo pipefail

# Get the absolute workspace path (parent directory of the script)
WS_DIR=$(realpath "$(dirname "$0")/..")

BRANCH_NAME="${BUILD_PROFILE:-}"
if [ -z "$BRANCH_NAME" ] && [ -f "$WS_DIR/.git/HEAD" ]; then
    git_head="$(<"$WS_DIR/.git/HEAD")"
    if [[ "$git_head" == ref:\ refs/heads/* ]]; then
        BRANCH_NAME="${git_head#ref: refs/heads/}"
    fi
fi
BRANCH_NAME="${BRANCH_NAME:-default}"
BRANCH_SAFE="$(echo "$BRANCH_NAME" | sed 's#[^A-Za-z0-9._-]#_#g')"

# Safety check 1: Ensure the target directory contains typical ROS2 workspace structures
# This prevents accidental execution in unrelated directories like ~ or /
if [ ! -d "${WS_DIR}/src" ] || [ ! -f "${WS_DIR}/README.md" ]; then
    echo "[ERROR] Dangerous operation blocked: The current path (${WS_DIR}) does not appear to be your ROS2 workspace."
    exit 1
fi

echo "========================================="
echo "Safely unlocking workspace: ${WS_DIR}"
echo "Branch profile: ${BRANCH_NAME}"
echo "Automatically finding files locked by Docker root and returning them to the current user..."
echo "========================================="

# Clean up old symlinks if any
[ -L "$WS_DIR/maps" ] && rm -f "$WS_DIR/maps"
[ -L "$WS_DIR/maps/sim" ] && rm -f "$WS_DIR/maps/sim"
[ -L "$WS_DIR/maps/reality" ] && rm -f "$WS_DIR/maps/reality"
[ -L "$WS_DIR/launch_logs" ] && rm -f "$WS_DIR/launch_logs"

mkdir -p "$WS_DIR/maps/$BRANCH_SAFE/sim" "$WS_DIR/maps/$BRANCH_SAFE/reality"
mkdir -p "$WS_DIR/launch_logs/$BRANCH_SAFE/nav" "$WS_DIR/launch_logs/$BRANCH_SAFE/slam"

# Safety check 2: Only touch entries owned by root in workspace.
# Use non-dereference mode for symlinks to avoid broken-link chown failures.
if [ "$(id -u)" -eq 0 ]; then
    RUN_AS_ROOT=""
elif command -v sudo >/dev/null 2>&1; then
    RUN_AS_ROOT="sudo"
else
    RUN_AS_ROOT=""
fi

if [ -n "$RUN_AS_ROOT" ] && ! $RUN_AS_ROOT -n true >/dev/null 2>&1; then
    echo "[WARN] sudo requires password or is unavailable in non-interactive mode."
    echo "[WARN] Skip chown. If needed, run manually with privileges:" 
    echo "       sudo find ${WS_DIR} -xdev -uid 0 ! -xtype l -exec chown ${USER}:${USER} {} +"
    echo "       sudo find ${WS_DIR} -xdev -uid 0 -xtype l -exec chown -h ${USER}:${USER} {} +"
    echo "[SUCCESS] Unlock complete (directory layout prepared; ownership unchanged)."
    exit 0
fi

CHOWN_PREFIX=()
if [ -n "$RUN_AS_ROOT" ]; then
    CHOWN_PREFIX=("$RUN_AS_ROOT")
fi

# Regular files/dirs
"${CHOWN_PREFIX[@]}" find "${WS_DIR}" -xdev -uid 0 ! -xtype l -exec chown "$USER:$USER" {} +
# Symlinks (no-dereference)
"${CHOWN_PREFIX[@]}" find "${WS_DIR}" -xdev -uid 0 -xtype l -exec chown -h "$USER:$USER" {} +

echo "[SUCCESS] Unlock complete. All files in the workspace have been returned to you!"

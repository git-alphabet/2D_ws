#!/bin/bash

set -euo pipefail

# Get the absolute workspace path (parent directory of the script)
WS_DIR=$(realpath "$(dirname "$0")/..")

# Safety check: Ensure the target directory contains typical ROS2 workspace structures
if [ ! -d "${WS_DIR}/src" ] || [ ! -f "${WS_DIR}/README.md" ]; then
    echo "[ERROR] Dangerous operation blocked: The current path (${WS_DIR}) does not appear to be your ROS2 workspace."
    exit 1
fi

echo "========================================="
echo "Safely unlocking workspace: ${WS_DIR}"
echo "Finding files locked by Docker root and returning them to the current user..."
echo "========================================="

# Determine sudo command
if [ "$(id -u)" -eq 0 ]; then
    RUN_AS_ROOT=""
elif command -v sudo >/dev/null 2>&1; then
    RUN_AS_ROOT="sudo"
    if ! sudo -n true >/dev/null 2>&1; then
        if [ -t 0 ] && [ -t 1 ]; then
            echo "[INFO] sudo authentication required. Please enter your password to continue..."
            if ! sudo -v; then
                echo "[WARN] sudo authentication failed. Cannot fix ownership."
                exit 1
            fi
        else
            echo "[ERROR] sudo requires password but current shell is non-interactive."
            echo "Run manually: sudo chown -R ${USER}:${USER} ${WS_DIR}"
            exit 1
        fi
    fi
else
    echo "[ERROR] sudo is unavailable and current user is not root. Cannot fix ownership."
    exit 1
fi

CHOWN_PREFIX=()
if [ -n "$RUN_AS_ROOT" ]; then
    CHOWN_PREFIX=("$RUN_AS_ROOT")
fi

# Fix ownership: regular files/dirs
"${CHOWN_PREFIX[@]}" find "${WS_DIR}" -xdev -uid 0 ! -xtype l -exec chown "$USER:$USER" {} +
# Fix ownership: symlinks (no-dereference)
"${CHOWN_PREFIX[@]}" find "${WS_DIR}" -xdev -uid 0 -xtype l -exec chown -h "$USER:$USER" {} +

echo "[SUCCESS] Unlock complete. All files in the workspace have been returned to you!"

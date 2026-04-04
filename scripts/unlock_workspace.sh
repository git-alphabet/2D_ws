#!/bin/bash

# Get the absolute workspace path (parent directory of the script)
WS_DIR=$(realpath "$(dirname "$0")/..")

# Safety check 1: Ensure the target directory contains typical ROS2 workspace structures
# This prevents accidental execution in unrelated directories like ~ or /
if [ ! -d "${WS_DIR}/src" ] || [ ! -f "${WS_DIR}/README.md" ]; then
    echo "[ERROR] Dangerous operation blocked: The current path (${WS_DIR}) does not appear to be your ROS2 workspace."
    exit 1
fi

echo "========================================="
echo "Safely unlocking workspace: ${WS_DIR}"
echo "Automatically finding files locked by Docker root and returning them to the current user..."
echo "========================================="

# Safety check 2: Find all files/directories owned by root within the restricted workspace directory and unlock them
# This handles build/ install/ log/ maps/ and root-owned __pycache__ etc.
sudo find "${WS_DIR}" -user root -exec chown "$USER:$USER" {} +

echo "[SUCCESS] Unlock complete. All files in the workspace have been returned to you!"

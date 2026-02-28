#!/usr/bin/env bash
set -euo pipefail

_d="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [[ "$_d" != "/" && ! -f "$_d/install/setup.bash" ]]; do _d="$(dirname "$_d")"; done
WS_ROOT="$_d"; unset _d
SRC_DIR="$WS_ROOT/src"
OUT_DIR="$WS_ROOT/docker/rosdep_src"

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

# Prevent colcon from treating the snapshot as a workspace package tree
touch "$OUT_DIR/COLCON_IGNORE"

count=0
while IFS= read -r -d '' pkgxml; do
  rel="${pkgxml#"$SRC_DIR/"}"
  dst="$OUT_DIR/$rel"
  mkdir -p "$(dirname "$dst")"
  cp -a "$pkgxml" "$dst"
  count=$((count+1))
done < <(find "$SRC_DIR" -name package.xml -print0)

echo "[gen_rosdep_src] copied $count package.xml files into $OUT_DIR"
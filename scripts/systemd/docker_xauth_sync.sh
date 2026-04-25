#!/usr/bin/env bash
set -euo pipefail

TARGET_FILE="/tmp/.docker.xauth"
TMP_FILE="$(mktemp /tmp/.docker.xauth.XXXXXX)"

cleanup() {
  rm -f "$TMP_FILE"
}
trap cleanup EXIT

# Always keep TARGET_FILE as a regular file to avoid Docker bind-mount type conflicts.
: > "$TMP_FILE"
chmod 0644 "$TMP_FILE"

merge_from_xauth_file() {
  local src="$1"

  if [[ ! -f "$src" || ! -s "$src" ]]; then
    return
  fi

  # Convert address family to FamilyWild for container access.
  if xauth -f "$src" nlist 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$TMP_FILE" nmerge - 2>/dev/null; then
    return
  fi

  return
}

declare -a candidates=()

while IFS= read -r -d '' f; do
  candidates+=("$f")
done < <(find /home -maxdepth 3 -type f -name ".Xauthority" -print0 2>/dev/null)

while IFS= read -r -d '' f; do
  candidates+=("$f")
done < <(find /run/user -maxdepth 4 -type f \( -name "Xauthority" -o -name ".Xauthority" -o -name ".mutter-Xwaylandauth.*" \) -print0 2>/dev/null)

for file in "${candidates[@]}"; do
  merge_from_xauth_file "$file"
done

# Keep ownership predictable even when script runs as root.
install -m 0644 "$TMP_FILE" "$TARGET_FILE"

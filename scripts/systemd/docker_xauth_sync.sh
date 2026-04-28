#!/usr/bin/env bash
set -euo pipefail

GUI_DIR="/tmp/gxu2026-docker-gui"
TARGET_FILE="${GUI_DIR}/xauth"
LEGACY_TARGET_FILE="/tmp/.docker.xauth"
DISPLAY_FILE="${GUI_DIR}/display"
DISPLAY_LIST_FILE="${GUI_DIR}/displays"
TMP_XAUTH_FILE="$(mktemp /tmp/.docker.xauth.XXXXXX)"
TMP_DISPLAY_FILE="$(mktemp /tmp/.docker.display.XXXXXX)"
TMP_DISPLAY_LIST_FILE="$(mktemp /tmp/.docker.displays.XXXXXX)"

declare -A display_scores=()

cleanup() {
  rm -f "$TMP_XAUTH_FILE" "$TMP_DISPLAY_FILE" "$TMP_DISPLAY_LIST_FILE"
}
trap cleanup EXIT

# 保持 GUI 目录和目标文件类型稳定，避免 Docker bind-mount 类型漂移。
mkdir -p "$GUI_DIR"
chmod 0755 "$GUI_DIR"
: > "$TMP_XAUTH_FILE"
: > "$TMP_DISPLAY_FILE"
: > "$TMP_DISPLAY_LIST_FILE"
chmod 0644 "$TMP_XAUTH_FILE" "$TMP_DISPLAY_FILE" "$TMP_DISPLAY_LIST_FILE"

normalize_display() {
  local raw="${1:-}"

  if [[ -z "$raw" ]]; then
    return 1
  fi

  if [[ "$raw" =~ ^:([0-9]+)(\.[0-9]+)?$ ]]; then
    printf ':%s\n' "${BASH_REMATCH[1]}"
    return 0
  fi

  if [[ "$raw" =~ /unix:([0-9]+)(\.[0-9]+)?$ ]]; then
    printf ':%s\n' "${BASH_REMATCH[1]}"
    return 0
  fi

  if [[ "$raw" =~ :([0-9]+)(\.[0-9]+)?$ ]]; then
    printf ':%s\n' "${BASH_REMATCH[1]}"
    return 0
  fi

  return 1
}

display_socket_exists() {
  local display="${1:-}"
  local display_num="${display#:}"

  [[ -S "/tmp/.X11-unix/X${display_num}" ]]
}

record_display_candidate() {
  local raw_display="${1:-}"
  local score="${2:-0}"
  local normalized
  local existing_score

  normalized="$(normalize_display "$raw_display")" || return 0
  display_socket_exists "$normalized" || return 0

  existing_score="${display_scores[$normalized]:-}"
  if [[ -z "$existing_score" || "$score" -gt "$existing_score" ]]; then
    display_scores["$normalized"]="$score"
  fi
  return 0
}

merge_from_xauth_file() {
  local src="$1"

  if [[ ! -f "$src" || ! -s "$src" ]]; then
    return
  fi

  # Convert address family to FamilyWild for container access.
  xauth -f "$src" nlist 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$TMP_XAUTH_FILE" nmerge - 2>/dev/null || true

  while read -r display_name _; do
    record_display_candidate "$display_name" 80
  done < <(xauth -f "$src" list 2>/dev/null || true)
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

while IFS= read -r -d '' env_file; do
  pid="${env_file#/proc/}"
  pid="${pid%%/*}"
  display_value=""
  score=0
  cmdline=""
  env_dump=""

  env_dump="$(cat "$env_file" 2>/dev/null | tr '\0' '\n' || true)"
  if [[ -z "$env_dump" ]]; then
    continue
  fi

  while IFS= read -r line; do
    if [[ "$line" == DISPLAY=* ]]; then
      display_value="${line#DISPLAY=}"
      break
    fi
  done <<< "$env_dump"

  if [[ -z "$display_value" ]]; then
    continue
  fi

  cmdline="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"
  case "$cmdline" in
    *nxnode*|*nxagent*|*NoMachine*|*nxserver*)
      score=400
      ;;
    *Xwayland*|*gnome-shell*|*mutter*|*gnome-session*|*kwin_wayland*|*plasmashell*)
      score=300
      ;;
    *Xorg*|*lightdm*|*sddm*|*weston*)
      score=200
      ;;
  esac

  if [[ "$score" -le 0 ]]; then
    continue
  fi

  record_display_candidate "$display_value" "$score"
done < <(find /proc -maxdepth 2 -path '/proc/[0-9]*/environ' -readable -print0 2>/dev/null)

while IFS= read -r -d '' socket_path; do
  socket_name="${socket_path##*/}"
  display_num="${socket_name#X}"
  record_display_candidate ":${display_num}" 10
done < <(find /tmp/.X11-unix -maxdepth 1 -type s -name 'X*' -print0 2>/dev/null)

if [[ "${#display_scores[@]}" -gt 0 ]]; then
  while read -r _score _display_num display_name; do
    printf '%s\n' "$display_name" >> "$TMP_DISPLAY_LIST_FILE"
  done < <(
    for display_name in "${!display_scores[@]}"; do
      printf '%s %s %s\n' "${display_scores[$display_name]}" "${display_name#:}" "$display_name"
    done | sort -k1,1nr -k2,2nr
  )

  head -n 1 "$TMP_DISPLAY_LIST_FILE" > "$TMP_DISPLAY_FILE"
fi

# 保持宿主机目标文件始终为普通文件，兼容旧挂载路径并为容器提供显示提示。
install -m 0644 "$TMP_XAUTH_FILE" "$TARGET_FILE"
install -m 0644 "$TMP_XAUTH_FILE" "$LEGACY_TARGET_FILE" 2>/dev/null || true
install -m 0644 "$TMP_DISPLAY_FILE" "$DISPLAY_FILE"
install -m 0644 "$TMP_DISPLAY_LIST_FILE" "$DISPLAY_LIST_FILE"

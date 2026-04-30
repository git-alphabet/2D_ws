#!/usr/bin/env bash
set -euo pipefail

GUI_DIR="/tmp/gxu2026-docker-gui"
TARGET_FILE="${GUI_DIR}/xauth"
LEGACY_TARGET_FILE="/tmp/.docker.xauth"
DISPLAY_FILE="${GUI_DIR}/display"
DISPLAY_LIST_FILE="${GUI_DIR}/displays"
READY_FILE="${GUI_DIR}/ready"
SESSION_FINGERPRINT_FILE="${GUI_DIR}/session_fingerprint"
STATUS_FILE="${GUI_DIR}/status"
TMP_XAUTH_FILE="$(mktemp /tmp/.docker.xauth.XXXXXX)"
TMP_DISPLAY_FILE="$(mktemp /tmp/.docker.display.XXXXXX)"
TMP_DISPLAY_LIST_FILE="$(mktemp /tmp/.docker.displays.XXXXXX)"
TMP_READY_FILE="$(mktemp /tmp/.docker.ready.XXXXXX)"
TMP_SESSION_FINGERPRINT_FILE="$(mktemp /tmp/.docker.session.XXXXXX)"
TMP_STATUS_FILE="$(mktemp /tmp/.docker.status.XXXXXX)"

declare -A display_scores=()
declare -A auth_cookies=()

cleanup() {
  rm -f \
    "$TMP_XAUTH_FILE" \
    "$TMP_DISPLAY_FILE" \
    "$TMP_DISPLAY_LIST_FILE" \
    "$TMP_READY_FILE" \
    "$TMP_SESSION_FINGERPRINT_FILE" \
    "$TMP_STATUS_FILE"
}
trap cleanup EXIT

# 保持 GUI 目录和目标文件类型稳定，避免 Docker bind-mount 类型漂移。
mkdir -p "$GUI_DIR"
chmod 0755 "$GUI_DIR"
: > "$TMP_XAUTH_FILE"
: > "$TMP_DISPLAY_FILE"
: > "$TMP_DISPLAY_LIST_FILE"
: > "$TMP_READY_FILE"
: > "$TMP_SESSION_FINGERPRINT_FILE"
: > "$TMP_STATUS_FILE"
chmod 0644 \
  "$TMP_XAUTH_FILE" \
  "$TMP_DISPLAY_FILE" \
  "$TMP_DISPLAY_LIST_FILE" \
  "$TMP_READY_FILE" \
  "$TMP_SESSION_FINGERPRINT_FILE" \
  "$TMP_STATUS_FILE"

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

record_cookie_candidate() {
  local cookie_value="${1:-}"

  if [[ -n "$cookie_value" ]]; then
    auth_cookies["$cookie_value"]=1
  fi
}

merge_from_xauth_file() {
  local src="$1"
  local display_name=""
  local auth_type=""
  local cookie_value=""

  if [[ ! -f "$src" || ! -s "$src" ]]; then
    return
  fi

  # Convert address family to FamilyWild for container access.
  xauth -f "$src" nlist 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$TMP_XAUTH_FILE" nmerge - 2>/dev/null || true

  while read -r display_name auth_type cookie_value _; do
    record_display_candidate "$display_name" 80
    if [[ "$auth_type" == "MIT-MAGIC-COOKIE-1" ]]; then
      record_cookie_candidate "$cookie_value"
    fi
  done < <(xauth -f "$src" list 2>/dev/null || true)
}

add_precise_display_cookies() {
  local display_name=""
  local cookie_value=""

  for display_name in "${!display_scores[@]}"; do
    for cookie_value in "${!auth_cookies[@]}"; do
      xauth -f "$TMP_XAUTH_FILE" add "$display_name" MIT-MAGIC-COOKIE-1 "$cookie_value" 2>/dev/null || true
    done
  done
}

display_has_cookie() {
  local display_name="${1:-}"
  local display_num=""

  if [[ -z "$display_name" ]]; then
    return 1
  fi

  display_num="${display_name#:}"

  xauth -f "$TMP_XAUTH_FILE" list 2>/dev/null | awk -v display_num="$display_num" '
    $2 == "MIT-MAGIC-COOKIE-1" && (
      $1 == ":" display_num ||
      $1 ~ ("/unix:" display_num "$") ||
      $1 ~ (":" display_num "$")
    ) { found = 1 }
    END { exit(found ? 0 : 1) }
  '
}

print_display_cookies() {
  local display_name="${1:-}"
  local display_num=""

  if [[ -z "$display_name" ]]; then
    return 0
  fi

  display_num="${display_name#:}"

  xauth -f "$TMP_XAUTH_FILE" list 2>/dev/null | awk -v display_num="$display_num" '
    $2 == "MIT-MAGIC-COOKIE-1" && (
      $1 == ":" display_num ||
      $1 ~ ("/unix:" display_num "$") ||
      $1 ~ (":" display_num "$")
    ) { print }
  '
}

count_any_cookies() {
  xauth -f "$TMP_XAUTH_FILE" list 2>/dev/null | awk '
    $2 == "MIT-MAGIC-COOKIE-1" { count++ }
    END { print count + 0 }
  '
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

add_precise_display_cookies

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

selected_display="$(head -n 1 "$TMP_DISPLAY_FILE" 2>/dev/null || true)"
boot_id="$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || true)"
cookie_count="$(count_any_cookies)"
candidate_count="${#display_scores[@]}"
ready_reason="missing_display"

if [[ -n "$selected_display" ]]; then
  ready_reason="missing_socket"
  if display_socket_exists "$selected_display"; then
    ready_reason="missing_cookie"
    if [[ "$cookie_count" -gt 0 ]]; then
      ready_reason="ready"
    fi
  fi
fi

{
  printf 'selected_display=%s\n' "${selected_display:-}"
  printf 'candidate_count=%s\n' "$candidate_count"
  printf 'cookie_count=%s\n' "$cookie_count"
  printf 'reason=%s\n' "$ready_reason"
} > "$TMP_STATUS_FILE"

if [[ -n "$selected_display" ]] \
  && display_socket_exists "$selected_display" \
  && [[ "$cookie_count" -gt 0 ]]; then
  printf 'display=%s\n' "$selected_display" > "$TMP_READY_FILE"
  printf 'boot_id=%s\n' "$boot_id" >> "$TMP_READY_FILE"
  {
    printf '%s\n' "$boot_id"
    printf '%s\n' "$selected_display"
    sha256sum "$TMP_XAUTH_FILE" | awk '{print $1}'
  } | sha256sum | awk '{print $1}' > "$TMP_SESSION_FINGERPRINT_FILE"
fi

# 保持宿主机目标文件始终为普通文件，兼容旧挂载路径并为容器提供显示提示。
# 为了防止 Docker 对于“单文件绑定挂载”在 inode 变更（如 install 或 mv）后导致容器内仍锁定在旧文件，我们原地覆写文件内容以保持 inode 不变。
cat "$TMP_XAUTH_FILE" > "$TARGET_FILE"
chmod 0644 "$TARGET_FILE"
cat "$TMP_XAUTH_FILE" > "$LEGACY_TARGET_FILE" 2>/dev/null || true
chmod 0644 "$LEGACY_TARGET_FILE" 2>/dev/null || true
cat "$TMP_DISPLAY_FILE" > "$DISPLAY_FILE"
chmod 0644 "$DISPLAY_FILE"
cat "$TMP_DISPLAY_LIST_FILE" > "$DISPLAY_LIST_FILE"
chmod 0644 "$DISPLAY_LIST_FILE"
cat "$TMP_STATUS_FILE" > "$STATUS_FILE"
chmod 0644 "$STATUS_FILE"

if [[ -s "$TMP_READY_FILE" && -s "$TMP_SESSION_FINGERPRINT_FILE" ]]; then
  cat "$TMP_READY_FILE" > "$READY_FILE"
  chmod 0644 "$READY_FILE"
  cat "$TMP_SESSION_FINGERPRINT_FILE" > "$SESSION_FINGERPRINT_FILE"
  chmod 0644 "$SESSION_FINGERPRINT_FILE"
else
  rm -f "$READY_FILE" "$SESSION_FINGERPRINT_FILE"
fi

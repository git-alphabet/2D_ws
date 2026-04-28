#!/usr/bin/env bash

gxu_normalize_display() {
  local raw="${1:-}"

  if [[ -z "$raw" ]]; then
    return 1
  fi

  if [[ "$raw" =~ ^:([0-9]+)(\.[0-9]+)?$ ]]; then
    printf ':%s\n' "${BASH_REMATCH[1]}"
    return 0
  fi

  return 1
}

gxu_display_socket_exists() {
  local display="${1:-}"
  local display_num="${display#:}"

  [[ -S "/tmp/.X11-unix/X${display_num}" ]]
}

gxu_add_display_candidate() {
  local raw_display="${1:-}"
  local normalized_display=""
  local existing_display=""

  normalized_display="$(gxu_normalize_display "$raw_display")" || return 0

  for existing_display in "${GXU_DISPLAY_CANDIDATES[@]}"; do
    if [[ "$existing_display" == "$normalized_display" ]]; then
      return 0
    fi
  done

  GXU_DISPLAY_CANDIDATES+=("$normalized_display")
  return 0
}

gxu_auto_detect_display() {
  if [[ "${GXU_GUI_AUTO_DETECT:-1}" != "1" ]]; then
    return 0
  fi

  local gui_dir="${GXU_GUI_DIR:-/tmp/gxu2026-docker-gui}"
  local display_file="${gui_dir}/display"
  local display_list_file="${gui_dir}/displays"
  local xauth_file="${gui_dir}/xauth"
  local display_override="${GXU_DISPLAY_OVERRIDE:-}"
  local selected_display=""
  local socket_path=""
  local display_num=""

  GXU_DISPLAY_CANDIDATES=()

  gxu_add_display_candidate "$display_override"

  if [[ -f "$display_file" ]]; then
    gxu_add_display_candidate "$(head -n 1 "$display_file")"
  fi

  if [[ -f "$display_list_file" ]]; then
    while IFS= read -r line; do
      gxu_add_display_candidate "$line"
    done < "$display_list_file"
  fi

  gxu_add_display_candidate "${DISPLAY:-}"

  for socket_path in /tmp/.X11-unix/X*; do
    if [[ ! -S "$socket_path" ]]; then
      continue
    fi
    display_num="${socket_path##*/X}"
    gxu_add_display_candidate ":${display_num}"
  done

  for selected_display in "${GXU_DISPLAY_CANDIDATES[@]}"; do
    if gxu_display_socket_exists "$selected_display"; then
      export DISPLAY="$selected_display"
      break
    fi
  done

  if [[ -f "$xauth_file" && -s "$xauth_file" ]]; then
    export XAUTHORITY="$xauth_file"
  elif [[ -f /tmp/.docker.xauth && -s /tmp/.docker.xauth ]]; then
    export XAUTHORITY=/tmp/.docker.xauth
  fi

  return 0
}

gxu_auto_detect_display

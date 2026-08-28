#!/usr/bin/env bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)

usage() {
  cat <<'EOF'
Usage:
  scripts/reality.sh <nav[n]|map[m]> [world] [launch_arg:=value ...]

Modes:
  nav | n   定位/导航模式 (slam:=False)
  map | m   建图模式     (slam:=True)

Examples:
  scripts/reality.sh n
  scripts/reality.sh map reserve
  scripts/reality.sh nav rmul_2024 use_rviz:=True use_decision:=True
EOF
}

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  fi
}

require_workspace_setup() {
  if [ -z "${ROS_DISTRO:-}" ]; then
    source_setup /opt/ros/humble/setup.bash
  fi
  if [ ! -f "$WS/install/setup.bash" ]; then
    echo "Missing workspace setup: $WS/install/setup.bash" >&2
    exit 1
  fi
  source_setup "$WS/install/setup.bash"
  cd "$WS"
}

is_true() {
  case "${1,,}" in
    true|1|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

# 实车清理:杀 reality_launch 相关节点,不动 /dev/shm
REALITY_SHUTDOWN_FILE="/tmp/guganav_reality_shutdown_$$"
REALITY_CLEANUP_STARTED=0

cleanup_reality_processes() {
  if [ "$REALITY_CLEANUP_STARTED" -eq 1 ]; then return 0; fi
  REALITY_CLEANUP_STARTED=1
  touch "$REALITY_SHUTDOWN_FILE" 2>/dev/null || true

  local all_pids=()
  local pid
  local pattern
  local patterns=(
    "ros2 launch guga_bringup reality_launch"
    "component_container"
    "point_lio"
    "small_gicp"
    "loam_interface"
    "sensor_scan_generation"
    "terrain_analysis"
    "terrain_analysis_ext"
    "controller_server"
    "planner_server"
    "bt_navigator"
    "smoother_server"
    "behavior_server"
    "waypoint_follower"
    "velocity_smoother"
    "lifecycle_manager"
    "nonrotating_vel_transform"
    "simple_decision"
    "guga_ui"
    "serial_driver"
    "rviz2"
  )

  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "${BASHPID:-$$}" ]; then
        all_pids+=("$pid")
      fi
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
  mapfile -t all_mppis < <(printf '%s\n' "${all_pids[@]}" | sort -u | grep -v '^$')

  if [ "${#all_pids[@]}" -gt 0 ]; then
    printf '%s\n' "${all_pids[@]}" | xargs -r kill -TERM -- 2>/dev/null || true
    sleep 3
    local remaining=()
    for pid in "${all_pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then remaining+=("$pid"); fi
    done
    if [ "${#remaining[@]}" -gt 0 ]; then
      printf '%s\n' "${remaining[@]}" | xargs -r kill -KILL -- 2>/dev/null || true
    fi
  fi

  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
}

exit_with_launch_status() {
  local status=$1
  if [ "$status" -eq 130 ] || [ "$status" -eq 143 ] || \
     { [ "$status" -ne 0 ] && [ -f "$REALITY_SHUTDOWN_FILE" ]; }; then
    exit 0
  fi
  exit "$status"
}

install_reality_cleanup_traps() {
  trap 'cleanup_reality_processes' EXIT
  trap 'cleanup_reality_processes; exit 0' INT
  trap 'cleanup_reality_processes; exit 0' TERM HUP
}

ensure_reality_map_and_pcd() {
  local world_arg=$1
  local map_arg=${2:-}
  local prior_pcd_arg=${3:-}
  local map_yaml="$WS/src/guga_bringup/map/reality/${world_arg}.yaml"
  local prior_pcd="$WS/src/guga_bringup/pcd/reality/${world_arg}.pcd"

  if [ -n "$map_arg" ]; then
    [ -f "$map_arg" ] || { echo "Missing map YAML: $map_arg" >&2; return 1; }
  elif [ -f "$map_yaml" ]; then
    map_arg="$map_yaml"
  fi

  if [ -n "$prior_pcd_arg" ]; then
    [ -f "$prior_pcd_arg" ] || { echo "Missing prior PCD: $prior_pcd_arg" >&2; return 1; }
  elif [ -f "$prior_pcd" ]; then
    prior_pcd_arg="$prior_pcd"
  fi

  if [ -z "$map_arg" ] || [ -z "$prior_pcd_arg" ]; then
    cat >&2 <<EOF
Missing reality map/prior PCD for world '$world_arg'.
Expected:
  $map_yaml
  $prior_pcd
Options:
  1. scripts/reality.sh map $world_arg
  2. scripts/reality.sh nav $world_arg map:=/path map prior_pcd_file:=/path.pcd
EOF
    return 1
  fi
  printf 'Using map=%s\nUsing prior_pcd=%s\n' "$map_arg" "$prior_pcd_arg"
}

mode=${1:-}
if [ -z "$mode" ]; then
  if [ -t 0 ]; then
    printf "Select reality mode [nav[n]/map[m]]: "
    read -r mode
  else
    usage >&2
    exit 2
  fi
else
  shift
fi

while true ;do
case "$mode" in
  n|nav|navigation) launch_mode=nav; slam_value=False; break ;;
  m|map|mapping|slam) launch_mode=map; slam_value=True; break ;;
  -h|--help|help) usage; exit 0 ;;
  *) echo "Unknown reality mode: $mode" >&2; usage >&2; read -r mode; continue ;;
esac
done

world=rmul_2024
slam=$slam_value
launch_args=()
map_arg=""
prior_pcd_arg=""

for arg in "$@"; do
  case "$arg" in
    world:=*) world=${arg#world:=} ;;
    slam:=*) slam=${arg#slam:=} ;;
    map:=*) map_arg=${arg#map:=}; launch_args+=("$arg") ;;
    prior_pcd_file:=*) prior_pcd_arg=${arg#prior_pcd_file:=}; launch_args+=("$arg") ;;
    *) launch_args+=("$arg") ;;
  esac
done

if [ "$launch_mode" = nav ] && ! is_true "$slam"; then
  ensure_reality_map_and_pcd "$world" "$map_arg" "$prior_pcd_arg"
fi

require_workspace_setup
install_reality_cleanup_traps
set +e
ros2 launch guga_bringup reality_launch.py \
  world:="$world" \
  slam:="$slam" \
  "${launch_args[@]}"
launch_status=$?
set -e
exit_with_launch_status "$launch_status"

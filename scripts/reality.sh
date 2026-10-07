#!/usr/bin/env bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
GUGANAV_MODE="reality"
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/lib/common.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/reality.sh <nav[n]|map[m]> [world] [launch_arg:=value ...]

Modes:
  nav | n   定位/导航模式 (slam:=False)
  map | m   建图模式     (slam:=True)

Planner (planner:=):
  planner:=jps         JPS global planner
  planner:=smac2d      SmacPlanner2D global planner

Controller (controller:=):
  controller:=pid      omni PID controller
  controller:=mppi     MPPI controller

Parameters are merged at launch time from config/reality/{base,controller,planner}
layered yaml files. nav 模式未显式给出 planner:=/controller:= 时，会先弹出与
simulation.sh 相同的编号菜单（先 planner 后 controller）；没有可读终端时取默认值
planner:=smac2d controller:=mppi。

Examples:
  scripts/reality.sh n
  scripts/reality.sh n floor2 planner:=smac2d controller:=mppi
  scripts/reality.sh map reserve
  scripts/reality.sh nav floor2 use_rviz:=True use_decision:=True
EOF
}




# 实车清理:杀 reality_launch 相关节点,不动 /dev/shm
REALITY_SHUTDOWN_FILE="/tmp/guganav_reality_shutdown_$$"
GUGANAV_SHUTDOWN_FILE="$REALITY_SHUTDOWN_FILE"
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
    "scan_to_sensor_frame"
    "terrain_analysis"
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
  mapfile -t all_pids < <(printf '%s\n' "${all_pids[@]}" | sort -u | grep -v '^$')

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

# ────────────────────────────────────────────────────────────────
# planner/controller 选择（与 simulation.sh 同一套交互）
# ────────────────────────────────────────────────────────────────
# 候选值取自 config/reality/ 下实际存在的 profile 文件：
#   planner/    jps.yaml、smac2d.yaml
#   controller/ pid.yaml、mppi.yaml
# 以后新增 profile（例如 mpc、smachybrid）时，把名字同时加进下面的
# choice 列表与对应菜单即可。
PLANNER_CHOICES="jps smac2d"
CONTROLLER_CHOICES="pid mppi"
DEFAULT_PLANNER="smac2d"
DEFAULT_CONTROLLER="mppi"

select_planner() {
  select_profile "Select global planner:" "Planner" "$DEFAULT_PLANNER" \
    "jps|JPS (jps)" \
    "smac2d|SmacPlanner2D (smac2d)"
}

select_controller() {
  select_profile "Select controller:" "Controller" "$DEFAULT_CONTROLLER" \
    "pid|omni PID (pid)" \
    "mppi|MPPI (mppi)"
}

# 把用户输入整理成透传给 reality_launch.py 的参数：
#   - planner:=/controller:= 原样透传并校验取值；
#   - legacy navigation_profile:= 映射为对应组合
#     （jps_pid → jps+pid、2d_mppi → smac2d+mppi）；
#   - 都没给且终端可交互时弹菜单，非交互取默认值。
# 注意：params_file:= 只原样透传，不算"已选择"——reality_launch 目前把
# params_file 强制置空（三文件合并），单文件覆盖未启用。
build_nav_args() {
  nav_args=()
  local arg
  local explicit_spec=""
  local chosen_planner=""
  local chosen_controller=""

  for arg in "$@"; do
    case "$arg" in
      navigation_profile:=*)
        case "${arg#navigation_profile:=}" in
          jps_pid)
            nav_args+=(planner:=jps controller:=pid)
            ;;
          2d_mppi)
            nav_args+=(planner:=smac2d controller:=mppi)
            ;;
          *)
            echo "Invalid navigation_profile: ${arg#navigation_profile:=}（reality 支持 jps_pid、2d_mppi）" >&2
            return 1
            ;;
        esac
        explicit_spec=1
        ;;
      planner:=*)
        chosen_planner=${arg#planner:=}
        nav_args+=("$arg")
        explicit_spec=1
        ;;
      controller:=*)
        chosen_controller=${arg#controller:=}
        nav_args+=("$arg")
        explicit_spec=1
        ;;
      *)
        nav_args+=("$arg")
        ;;
    esac
  done

  if [ -z "$explicit_spec" ]; then
    chosen_planner=$(select_planner) || return 1
    chosen_controller=$(select_controller) || return 1
    nav_args+=(planner:="$chosen_planner" controller:="$chosen_controller")
  fi

  if [ -n "$chosen_planner" ]; then
    validate_choice planner "$chosen_planner" "$PLANNER_CHOICES" || return 1
  fi
  if [ -n "$chosen_controller" ]; then
    validate_choice controller "$chosen_controller" "$CONTROLLER_CHOICES" || return 1
  fi
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
  n|nav|navigation)
    launch_mode=nav
    slam_value=False
    build_nav_args "$@" || exit 2
    set -- "${nav_args[@]}"
    break
    ;;
  m|map|mapping|slam) launch_mode=map; slam_value=True; break ;;
  -h|--help|help) usage; exit 0 ;;
  *) echo "Unknown reality mode: $mode" >&2; usage >&2; read -r mode; continue ;;
esac
done

world=floor2
slam=$slam_value
launch_args=()
map_arg=""
prior_pcd_arg=""

# 位置参数 world（如 `reality.sh n floor2`）：与 simulation.sh 一样在这里消费掉。
# 不消费的话它会被下面的 *) 分支塞进 launch_args，launch 会多收到一个裸参数，
# 而且 map 模式下 world 会一直停在默认值。
if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]]; then
  world=$1
  shift
fi

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

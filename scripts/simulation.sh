#!/usr/bin/env bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)
if [ -z "${SIMULATION_SESSION_ID:-}" ]; then
  export SIMULATION_SESSION_ID="$$-${RANDOM:-0}"
  rm -f "/tmp/guganav_simulation_shutdown_${SIMULATION_SESSION_ID}"
else
  export SIMULATION_SESSION_ID
fi
SIMULATION_SHUTDOWN_FILE="/tmp/guganav_simulation_shutdown_${SIMULATION_SESSION_ID}"
SIMULATION_CLEANUP_STARTED=0

usage() {
  cat <<'EOF'
Usage:
  scripts/simulation.sh <nav[n]|map[m]> [world] [launch_arg:=value ...]

Examples:
  scripts/simulation.sh n
  scripts/simulation.sh nav
  scripts/simulation.sh m rmul_2025
  scripts/simulation.sh map rmul_2025
  scripts/simulation.sh nav rmuc_2025 planner:=jps controller:=mppi use_rviz:=False

Planner (planner:=):
  planner:=jps         JPS global planner
  planner:=smac2d      SmacPlanner2D global planner
  planner:=smachybrid  SmacPlannerHybrid global planner

Controller (controller:=):
  controller:=pid   omni PID controller (default)
  controller:=mppi  MPPI controller
  controller:=mpc   MPC controller

All 9 planner/controller combinations are supported; parameters are
merged at runtime from the existing nav2_params*.yaml files (no new
files are added). Default: planner:=jps controller:=pid.

Legacy navigation_profile:=jps_pid|2d_mppi|jps_mpc still works and
takes precedence over planner:=/controller:=.

When a navigation command is run from a terminal without
planner:=/controller:= or navigation_profile:=..., an interactive
numbered selection menu is shown: first the planner (1-3), then the
controller (1-3).
EOF
}

pause_if_interactive() {
  if [ -t 0 ] && [ -t 1 ]; then
    printf "\nPress Enter to close..."
    read -r _
  fi
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
    echo "Run colcon build before starting simulation." >&2
    exit 1
  fi

  source_setup "$WS/install/setup.bash"
  cd "$WS"
}

quote_command() {
  printf "%q " "$@"
}

is_true() {
  case "${1,,}" in
    true | 1 | yes | on)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

PLANNER_CHOICES="jps smac2d smachybrid"
CONTROLLER_CHOICES="pid mppi mpc"

SIMULATION_PARAMS_DIR="$WS/src/guga_bringup/config/simulation"

validate_choice() {
  local name=$1
  local value=$2
  local valid=$3
  local choice
  for choice in $valid; do
    if [ "$choice" = "$value" ]; then
      return 0
    fi
  done
  echo "Invalid ${name}: '$value'. Valid values: $valid" >&2
  return 1
}

select_planner() {
  local selection

  # stdout is captured by the caller, so use /dev/tty to detect and read an
  # actual interactive terminal instead of checking stdout's terminal state.
  if [ ! -r /dev/tty ]; then
    printf 'jps'
    return 0
  fi

  cat >&2 <<'EOF'

Select global planner:
  1) JPS (jps)
  2) SmacPlanner2D (smac2d)
  3) SmacPlannerHybrid (smachybrid)
EOF
  while true; do
    printf 'Planner [1]: ' >&2
    if ! read -r selection < /dev/tty; then
      selection=1
    fi
    case "${selection:-1}" in
      1)
        printf 'jps'
        return 0
        ;;
      2)
        printf 'smac2d'
        return 0
        ;;
      3)
        printf 'smachybrid'
        return 0
        ;;
      *)
        echo "Please select 1, 2, or 3." >&2
        ;;
    esac
  done
}

select_controller() {
  local selection

  # stdout is captured by the caller, so use /dev/tty to detect and read an
  # actual interactive terminal instead of checking stdout's terminal state.
  if [ ! -r /dev/tty ]; then
    printf 'pid'
    return 0
  fi

  cat >&2 <<'EOF'

Select controller:
  1) omni PID (pid)
  2) MPPI (mppi)
  3) MPC (mpc)
EOF
  while true; do
    printf 'Controller [1]: ' >&2
    if ! read -r selection < /dev/tty; then
      selection=1
    fi
    case "${selection:-1}" in
      1)
        printf 'pid'
        return 0
        ;;
      2)
        printf 'mppi'
        return 0
        ;;
      3)
        printf 'mpc'
        return 0
        ;;
      *)
        echo "Please select 1, 2, or 3." >&2
        ;;
    esac
  done
}

params_file_for_controller() {
  case "$1" in
    pid)
      printf '%s/nav2_params.yaml' "$SIMULATION_PARAMS_DIR"
      ;;
    mppi)
      printf '%s/nav2_params_mppi.yaml' "$SIMULATION_PARAMS_DIR"
      ;;
    mpc)
      printf '%s/nav2_params_mpc.yaml' "$SIMULATION_PARAMS_DIR"
      ;;
  esac
}

planner_params_file_for_planner() {
  case "$1" in
    jps)
      printf '%s/nav2_params.yaml' "$SIMULATION_PARAMS_DIR"
      ;;
    smac2d)
      printf '%s/nav2_params_mppi.yaml' "$SIMULATION_PARAMS_DIR"
      ;;
    smachybrid)
      printf '%s/nav2_params_mpc.yaml' "$SIMULATION_PARAMS_DIR"
      ;;
  esac
}

# Merge the chosen controller's params file (base) with the chosen
# planner's planner_server section, and print the path of the merged
# file. The result lives in /tmp; no repo params files are added.
generate_merged_params() {
  local planner=$1
  local controller=$2
  local base_file
  local planner_file
  local merged_file
  base_file=$(params_file_for_controller "$controller")
  planner_file=$(planner_params_file_for_planner "$planner")
  merged_file="/tmp/guganav_nav2_params_${SIMULATION_SESSION_ID}_${planner}_${controller}.yaml"

  python3 - "$base_file" "$planner_file" "$merged_file" <<'PY'
import sys
import yaml

base_path, planner_path, out_path = sys.argv[1:4]
with open(base_path, encoding="utf-8") as f:
    data = yaml.safe_load(f)
with open(planner_path, encoding="utf-8") as f:
    planner_data = yaml.safe_load(f)

data["planner_server"] = planner_data["planner_server"]

with open(out_path, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
PY
  if [ $? -ne 0 ]; then
    echo "Failed to generate merged params file for planner:=$planner controller:=$controller" >&2
    return 1
  fi
  printf '%s' "$merged_file"
}

# Fill the global nav_args array with the effective launch arguments.
# Converts separate planner:=/controller:= choices into a runtime-merged
# params_file:= for the launch stack.
build_nav_args() {
  nav_args=()
  local planner=""
  local controller=""
  local arg
  local merged_params
  local explicit_spec=""

  for arg in "$@"; do
    case "$arg" in
      navigation_profile:=* | params_file:=*)
        # 收集所有显式指定项（可能有多个，比如 nav-only 二次调用时
        # params_file:= 与 navigation_profile:= 同时出现），不提前 return
        nav_args+=("$arg")
        explicit_spec=1
        ;;
      planner:=*)
        planner=${arg#planner:=}
        ;;
      controller:=*)
        controller=${arg#controller:=}
        ;;
      *)
        nav_args+=("$arg")
        ;;
    esac
  done

  # 用户已显式指定 navigation_profile:= 或 params_file:= 时，直接透传，
  # 不再走 planner/controller 选择与合并
  if [ -n "$explicit_spec" ]; then
    return 0
  fi

  if [ -n "$planner" ] && ! validate_choice planner "$planner" "$PLANNER_CHOICES"; then
    return 1
  fi
  if [ -n "$controller" ] && ! validate_choice controller "$controller" "$CONTROLLER_CHOICES"; then
    return 1
  fi

  if [ -z "$planner" ]; then
    planner=$(select_planner) || return 1
  fi
  if [ -z "$controller" ]; then
    controller=$(select_controller) || return 1
  fi

  if ! merged_params=$(generate_merged_params "$planner" "$controller"); then
    return 1
  fi
  nav_args+=("params_file:=$merged_params")
  # controller 决定底盘模式：mppi/mpc → 启动即小陀螺，pid → 跟随。
  # 必须显式传 navigation_profile，否则 launch 里默认 jps_pid，
  # nonrotating_vel_transform 的 initial_chassis_mode 会被算成 0（不自旋）。
  case "$controller" in
    pid)
      nav_args+=("navigation_profile:=jps_pid")
      ;;
    mppi)
      nav_args+=("navigation_profile:=2d_mppi")
      ;;
    mpc)
      nav_args+=("navigation_profile:=jps_mpc")
      ;;
  esac
}

cleanup_simulation_processes() {
  local all_pids=()
  local pid
  local pattern
  # 覆盖 launch、Gazebo/Ignition 服务端/客户端、RViz、以及 Gazebo 生成的机器人节点。
  local patterns=(
    "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
    "ros2 launch guga_bringup simulation_launch.py"
    "ign gazebo"
    "gzserver"
    "gzclient"
    "gz sim"
    "rviz2"
    "rmua19_robot_base"
  )

  if [ "$SIMULATION_CLEANUP_STARTED" -eq 1 ]; then
    return 0
  fi
  SIMULATION_CLEANUP_STARTED=1

  touch "$SIMULATION_SHUTDOWN_FILE" 2>/dev/null || true

  # 收集所有相关进程并去重
  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "${BASHPID:-$$}" ]; then
        all_pids+=("$pid")
      fi
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
  mapfile -t all_pids < <(printf "%s\n" "${all_pids[@]}" | sort -u | grep -v '^$')

  if [ "${#all_pids[@]}" -eq 0 ]; then
    return 0
  fi

  # 第一阶段：优雅终止，给 Gazebo/Ignition 短暂关闭时间
  printf "%s\n" "${all_pids[@]}" | xargs -r kill -TERM -- 2>/dev/null || true
  sleep 3

  # 第二阶段：仍有残留则强制终止，避免拖慢下次仿真启动
  local remaining=()
  for pid in "${all_pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      remaining+=("$pid")
    fi
  done
  if [ "${#remaining[@]}" -gt 0 ]; then
    printf "%s\n" "${remaining[@]}" | xargs -r kill -KILL -- 2>/dev/null || true
  fi
}

exit_with_launch_status() {
  local status=$1

  # 130=SIGINT(Ctrl-C)、143=SIGTERM：主动关闭信号，视为正常退出；
  # 或者 SHUTDOWN_FILE 已创建（清理函数已执行）也视为主动关闭。
  if [ "$status" -eq 130 ] || [ "$status" -eq 143 ] || \
     { [ "$status" -ne 0 ] && [ -f "$SIMULATION_SHUTDOWN_FILE" ]; }; then
    exit 0
  fi

  exit "$status"
}

install_simulation_cleanup_traps() {
  trap 'cleanup_simulation_processes' EXIT
  # 主动关闭（Ctrl-C / 终端关闭 / TERM）视为正常退出，退出码 0，
  # 避免终端提示 "Press Enter to close"。
  trap 'cleanup_simulation_processes; exit 0' INT
  trap 'cleanup_simulation_processes; exit 0' TERM HUP
}

wait_for_simulation_terminal_close() {
  if [ -t 0 ] && [ -t 1 ]; then
    echo "Close this terminal or press Ctrl-C to stop gazebo and rviz."
    install_simulation_cleanup_traps
    while true; do
      sleep 3600 &
      wait $!
    done
  fi
}

ensure_simulation_prior_pcd() {
  local world_arg=$1
  local prior_pcd_arg=${2:-}
  local source_pcd="$WS/src/guga_bringup/pcd/simulation/${world_arg}.pcd"
  local install_pcd="$WS/install/guga_bringup/share/guga_bringup/pcd/simulation/${world_arg}.pcd"

  if [ -n "$prior_pcd_arg" ]; then
    if [ -f "$prior_pcd_arg" ]; then
      return 0
    fi
    echo "Missing prior PCD file: $prior_pcd_arg" >&2
    return 1
  fi

  if [ -f "$source_pcd" ] || [ -f "$install_pcd" ]; then
    return 0
  fi

  cat >&2 <<EOF
Missing simulation prior PCD for world '$world_arg'.

Expected one of:
  $source_pcd
  $install_pcd

The 'nav' mode uses localization with small_gicp and needs a prior PCD map.
Use one of these options:
  1. Download/copy ${world_arg}.pcd into src/guga_bringup/pcd/simulation/
     and rebuild or source the symlink-install workspace.
  2. Run SLAM mode instead:
     scripts/simulation.sh map $world_arg
  3. Explicitly pass a PCD:
     scripts/simulation.sh nav $world_arg prior_pcd_file:=/path/to/${world_arg}.pcd
EOF
  return 1
}

run_in_terminal() {
  local title=$1
  shift
  local command
  local wrapped_command
  command=$(quote_command "$@")
  wrapped_command="${command}; status=\$?; if [ \$status -ne 0 ]; then printf '\\nCommand failed with exit code %s. Press Enter to close...' \"\$status\"; read -r _; fi; exit \$status"

  if command -v gnome-terminal >/dev/null 2>&1; then
    if gnome-terminal --title "$title" -- bash -lc "$wrapped_command"; then
      return 0
    fi
  fi

  if command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -p tabtitle="$title" -e bash -lc "$wrapped_command" >/dev/null 2>&1 &
    return 0
  fi

  if command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "$wrapped_command" >/dev/null 2>&1 &
    return 0
  fi

  return 1
}

run_complete_simulation() {
  local run_mode=$1
  shift
  local nav_mode="${run_mode}-only"
  local start_delay=${SIMULATION_START_DELAY:-3}
  local world_arg=rmul_2025
  local slam_arg=False
  local prior_pcd_arg=""

  if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]]; then
    world_arg=$1
  fi

  for arg in "$@"; do
    case "$arg" in
      world:=*)
        world_arg=${arg#world:=}
        ;;
      slam:=*)
        slam_arg=${arg#slam:=}
        ;;
      prior_pcd_file:=*)
        prior_pcd_arg=${arg#prior_pcd_file:=}
        ;;
    esac
  done

  if [ "$run_mode" = nav ] && ! is_true "$slam_arg"; then
    ensure_simulation_prior_pcd "$world_arg" "$prior_pcd_arg"
  fi

  if run_in_terminal "guganav gazebo" "$WS/scripts/simulation.sh" __gazebo world:="$world_arg"; then
    sleep "$start_delay"
    if run_in_terminal "guganav ${run_mode}" "$WS/scripts/simulation.sh" "$nav_mode" "$@"; then
      echo "Started complete simulation: gazebo + ${run_mode}/rviz"
      wait_for_simulation_terminal_close
      return 0
    fi

    echo "Could not open a second terminal; running ${run_mode}/rviz in current terminal." >&2
    install_simulation_cleanup_traps
    "$WS/scripts/simulation.sh" "$nav_mode" "$@"
    return 0
  fi

  echo "No supported terminal emulator found; running gazebo in background." >&2
  "$WS/scripts/simulation.sh" __gazebo world:="$world_arg" &
  local gazebo_pid=$!
  trap 'if [ -n "${gazebo_pid:-}" ]; then kill "$gazebo_pid" 2>/dev/null || true; fi; cleanup_simulation_processes' EXIT
  trap 'if [ -n "${gazebo_pid:-}" ]; then kill "$gazebo_pid" 2>/dev/null || true; fi; cleanup_simulation_processes; exit 130' INT
  trap 'if [ -n "${gazebo_pid:-}" ]; then kill "$gazebo_pid" 2>/dev/null || true; fi; cleanup_simulation_processes; exit 143' TERM HUP
  sleep "$start_delay"
  "$WS/scripts/simulation.sh" "$nav_mode" "$@"
}

mode=${1:-}
if [ -z "$mode" ]; then
  if [ -t 0 ]; then
    printf "Select simulation mode [nav[n]/map[m]]: "
    read -r mode
  else
    usage >&2
    exit 2
  fi
else
  shift
fi

case "$mode" in
  n | nav | navigation)
    build_nav_args "$@" || exit 2
    run_complete_simulation nav "${nav_args[@]}"
    exit 0
    ;;
  m | map | mapping | slam)
    run_complete_simulation map "$@"
    exit 0
    ;;
  nav-only | navigation-only)
    build_nav_args "$@" || exit 2
    set -- "${nav_args[@]}"
    slam=False
    ;;
  map-only | mapping-only | slam-only)
    slam=True
    ;;
  __gazebo)
    require_workspace_setup
    gazebo_args=("$@")
    if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]]; then
      gazebo_args=("world:=$1")
      shift
      gazebo_args+=("$@")
    fi
    install_simulation_cleanup_traps
    set +e
    ros2 launch rmu_gazebo_simulator bringup_sim.launch.py "${gazebo_args[@]}"
    launch_status=$?
    set -e
    exit_with_launch_status "$launch_status"
    ;;
  -h | --help | help)
    usage
    pause_if_interactive
    exit 0
    ;;
  *)
    echo "Unknown simulation mode: $mode" >&2
    usage >&2
    exit 2
    ;;
esac

world=rmul_2025
if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]]; then
  world=$1
  shift
fi

launch_args=()
for arg in "$@"; do
  case "$arg" in
    world:=*)
      world=${arg#world:=}
      ;;
    slam:=*)
      slam=${arg#slam:=}
      ;;
    *)
      launch_args+=("$arg")
      ;;
  esac
done

if [ "$slam" = "False" ]; then
  prior_pcd_arg=""
  for arg in "${launch_args[@]}"; do
    if [[ "$arg" == prior_pcd_file:=* ]]; then
      prior_pcd_arg=${arg#prior_pcd_file:=}
    fi
  done
  ensure_simulation_prior_pcd "$world" "$prior_pcd_arg"
fi

require_workspace_setup

install_simulation_cleanup_traps
set +e
ros2 launch guga_bringup simulation_launch.py \
  world:="$world" \
  slam:="$slam" \
  "${launch_args[@]}"
launch_status=$?
set -e
exit_with_launch_status "$launch_status"

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

All 9 planner/controller combinations are supported. Parameters are
merged at launch time from config/simulation/{base,controller,planner}
layered yaml files. Default: planner:=jps controller:=pid.

Legacy navigation_profile:=jps_pid|2d_mppi|jps_mpc still works and
maps to the corresponding planner/controller combination.

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

# ────────────────────────────────────────────────────────────────
# 导航参数分层（9 组合运行时合并）
# ────────────────────────────────────────────────────────────────
# 控制器与规划器的 9 种组合由三个参数文件在 launch 侧合并得到：
#   config/simulation/base.yaml（公共）
#   + config/simulation/controller/<controller>.yaml（控制器差异，覆盖 base）
#   + config/simulation/planner/<planner>.yaml（规划器差异，覆盖前两层）
# 本脚本只负责把用户选择（planner:=/controller:=）透传给 launch，
# 不关心文件路径与合并细节（由 simulation_launch.py 处理）。

# 填充 nav_args 数组为透传给 simulation_launch.py 的有效参数。
# - planner:=/controller:=/params_file:= 原样透传
# - legacy navigation_profile:= 映射为对应的 planner/controller 组合
#   （jps_pid → jps+pid、2d_mppi → smac2d+mppi、jps_mpc → smachybrid+mpc）
# - 均未指定且终端交互可用时，弹出 planner/controller 选择菜单
build_nav_args() {
  nav_args=()
  local arg
  local explicit_spec=""
  local chosen_planner=""
  local chosen_controller=""

  for arg in "$@"; do
    case "$arg" in
      navigation_profile:=*)
        # legacy profile → 3 个代表组合
        case "${arg#navigation_profile:=}" in
          jps_pid)
            nav_args+=(planner:=jps controller:=pid)
            ;;
          2d_mppi)
            nav_args+=(planner:=smac2d controller:=mppi)
            ;;
          jps_mpc)
            nav_args+=(planner:=smachybrid controller:=mpc)
            ;;
          *)
            echo "Invalid navigation_profile: ${arg#navigation_profile:=}" >&2
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
      params_file:=*)
        # 单文件覆盖调试
        nav_args+=("$arg")
        explicit_spec=1
        ;;
      *)
        nav_args+=("$arg")
        ;;
    esac
  done

  # 未显式指定任何选择时：交互终端弹菜单，非交互默认 jps+pid
  if [ -z "$explicit_spec" ]; then
    chosen_planner=$(select_planner) || return 1
    chosen_controller=$(select_controller) || return 1
    nav_args+=(planner:="$chosen_planner" controller:="$chosen_controller")
  fi

  # 校验合法值（显式传入时）
  if [ -n "$chosen_planner" ]; then
    validate_choice planner "$chosen_planner" "$PLANNER_CHOICES" || return 1
  fi
  if [ -n "$chosen_controller" ]; then
    validate_choice controller "$chosen_controller" "$CONTROLLER_CHOICES" || return 1
  fi
}

# ────────────────────────────────────────────────────────────────
# 仿真清理：仿真结束（Ctrl-C / 终端关闭 / EXIT trap）时自动执行
#   1) 强杀仿真相关进程（TERM → 3s → KILL 两阶段）
#   2) 刷新 ros2 daemon，清话题/节点缓存（解决"话题残余很久才关"）
#   3) 清理 DDS 共享内存残留（FastDDS /dev/shm）
# 仅用于仿真环境：第 3 步会删除当前用户的所有 FastDDS 共享内存段，
# 若同时在跑实车/其他 DDS 程序，勿在本终端执行仿真清理。
# ────────────────────────────────────────────────────────────────
cleanup_simulation_processes() {
  local all_pids=()
  local pid
  local pattern

  # 覆盖 launch、Gazebo/Ignition 服务端/客户端、RViz、以及导航/感知/UI 节点。
  local patterns=(
    "ros2 launch rmu_gazebo_simulator"
    "ros2 launch guga_bringup"
    "ign gazebo"
    "gzserver"
    "gzclient"
    "gz sim"
    # Gazebo launch 被强制结束时，这些子进程可能被 systemd --user
    # 收养并继续发布 /clock、传感器和 TF，污染下一次仿真。
    "ros_gz_bridge/parameter_bridge"
    "robot_state_publisher"
    "pose_bridge"
    "rfid_bridge"
    "rviz2"
    "rmua19_robot_base"
    # 导航/感知/UI 节点（component_container 为 composition 模式容器）
    "component_container"
    "ign_sim_pointcloud_tool"
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
  )

  if [ "$SIMULATION_CLEANUP_STARTED" -eq 1 ]; then
    return 0
  fi
  SIMULATION_CLEANUP_STARTED=1

  touch "$SIMULATION_SHUTDOWN_FILE" 2>/dev/null || true

  # ── 1. 收集并强杀仿真相关进程 ──
  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "${BASHPID:-$$}" ]; then
        all_pids+=("$pid")
      fi
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
  mapfile -t all_pids < <(printf '%s\n' "${all_pids[@]}" | sort -u | grep -v '^$')

  if [ "${#all_pids[@]}" -gt 0 ]; then
    # 第一阶段：优雅终止，给 Gazebo/Ignition 短暂关闭时间
    printf '%s\n' "${all_pids[@]}" | xargs -r kill -TERM -- 2>/dev/null || true
    sleep 3

    # 第二阶段：仍有残留则强制终止，避免拖慢下次仿真启动
    local remaining=()
    for pid in "${all_pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        remaining+=("$pid")
      fi
    done
    if [ "${#remaining[@]}" -gt 0 ]; then
      printf '%s\n' "${remaining[@]}" | xargs -r kill -KILL -- 2>/dev/null || true
    fi
  fi

  # ── 2. 刷新 ros2 daemon：清话题/节点缓存 ──
  # 节点已退出但 `ros2 topic/node list` 仍显示旧话题，是 daemon 缓存所致；
  # stop 后下次调用会自动重启 daemon，缓存即清空。
  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
  fi

  # ── 3. 清理 DDS 共享内存残留（FastDDS）──
  # 进程被强杀后 /dev/shm 中的参与者段可能残留，影响下次仿真启动。
  rm -f \
    /dev/shm/fastdds_* \
    /dev/shm/fastrtps_* \
    /dev/shm/sem.fastdds_* \
    2>/dev/null || true
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

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
  scripts/simulation.sh nav rmuc_2025 navigation_profile:=2d_mppi use_rviz:=False

Navigation profiles:
  navigation_profile:=jps_pid  JPS global planner + omni PID controller (default)
  navigation_profile:=2d_mppi  SmacPlanner2D global planner + MPPI controller

When a navigation command is run from a terminal without
navigation_profile:=..., an interactive profile menu is shown.
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

has_navigation_profile() {
  local arg
  for arg in "$@"; do
    if [[ "$arg" == navigation_profile:=* ]]; then
      return 0
    fi
  done
  return 1
}

select_navigation_profile() {
  local selection

  # stdout is captured by the caller, so use /dev/tty to detect and read an
  # actual interactive terminal instead of checking stdout's terminal state.
  if [ ! -r /dev/tty ]; then
    printf 'navigation_profile:=jps_pid'
    return 0
  fi

  cat >&2 <<'EOF'

Select navigation profile:
  1) JPS global planner + omni PID controller (jps_pid)
  2) SmacPlanner2D + MPPI controller (2d_mppi)
EOF
  while true; do
    printf 'Profile [1]: ' >&2
    if ! read -r selection < /dev/tty; then
      selection=1
    fi
    case "${selection:-1}" in
      1)
        printf 'navigation_profile:=jps_pid'
        return 0
        ;;
      2)
        printf 'navigation_profile:=2d_mppi'
        return 0
        ;;
      *)
        echo "Please select 1 or 2." >&2
        ;;
    esac
  done
}

cleanup_simulation_processes() {
  local launch_pids=()
  local simulation_pids=()
  local pid
  local pattern
  local launch_patterns=(
    "ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
    "ros2 launch guga_bringup simulation_launch.py"
  )
  local simulation_patterns=(
    "rviz2"
    "gzserver"
    "gzclient"
    "gz sim"
    "ign gazebo"
  )

  if [ "$SIMULATION_CLEANUP_STARTED" -eq 1 ]; then
    return 0
  fi
  SIMULATION_CLEANUP_STARTED=1

  touch "$SIMULATION_SHUTDOWN_FILE" 2>/dev/null || true

  for pattern in "${launch_patterns[@]}"; do
    while IFS= read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "${BASHPID:-$$}" ]; then
        launch_pids+=("$pid")
      fi
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done

  if [ "${#launch_pids[@]}" -gt 0 ]; then
    printf "%s\n" "${launch_pids[@]}" | sort -u | xargs -r kill -INT -- 2>/dev/null || true
    sleep 1
  fi

  for pattern in "${simulation_patterns[@]}"; do
    while IFS= read -r pid; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && [ "$pid" != "${BASHPID:-$$}" ]; then
        simulation_pids+=("$pid")
      fi
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done

  if [ "${#simulation_pids[@]}" -gt 0 ]; then
    printf "%s\n" "${simulation_pids[@]}" | sort -u | xargs -r kill -TERM -- 2>/dev/null || true
  fi
}

exit_with_launch_status() {
  local status=$1

  if [ "$status" -ne 0 ] && [ -f "$SIMULATION_SHUTDOWN_FILE" ]; then
    exit 0
  fi

  exit "$status"
}

install_simulation_cleanup_traps() {
  trap 'cleanup_simulation_processes' EXIT
  trap 'cleanup_simulation_processes; exit 130' INT
  trap 'cleanup_simulation_processes; exit 143' TERM HUP
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
    nav_args=("$@")
    if ! has_navigation_profile "${nav_args[@]}"; then
      nav_args+=("$(select_navigation_profile)")
    fi
    run_complete_simulation nav "${nav_args[@]}"
    exit 0
    ;;
  m | map | mapping | slam)
    run_complete_simulation map "$@"
    exit 0
    ;;
  nav-only | navigation-only)
    nav_args=("$@")
    if ! has_navigation_profile "${nav_args[@]}"; then
      nav_args+=("$(select_navigation_profile)")
    fi
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

set +e
ros2 launch guga_bringup simulation_launch.py \
  world:="$world" \
  slam:="$slam" \
  "${launch_args[@]}"
launch_status=$?
set -e
exit_with_launch_status "$launch_status"

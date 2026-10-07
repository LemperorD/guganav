#!/usr/bin/env bash
# guganav 入口脚本公共函数库
#
# reality.sh 与 simulation.sh 共用这里的实现，两个入口脚本只保留各自独有的部分
# （实车：udev/串口/清理；仿真：Gazebo 启动与进程清理）。planner/controller
# 的交互式选择菜单由 select_profile 提供，两个入口脚本共用同一套交互。
#
# 使用方式：入口脚本先设置 GUGANAV_MODE 与 GUGANAV_SHUTDOWN_FILE，再 source 本文件。
#   GUGANAV_MODE           reality | simulation，仅用于提示信息
#   GUGANAV_SHUTDOWN_FILE  清理函数创建的标记文件，用于判断"主动关闭"还是"异常退出"

set -euo pipefail

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
    echo "Run colcon build before starting ${GUGANAV_MODE:-navigation}." >&2
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
pause_if_interactive() {
  if [ -t 0 ] && [ -t 1 ]; then
    printf "\nPress Enter to close..."
    read -r _
  fi
}
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
# 交互式选择菜单（reality.sh / simulation.sh 共用）。
#
# 用法：select_profile <菜单标题> <提示名> <默认值> <值|显示名>...
#   结果打印到 stdout（供调用方 $(...) 捕获），菜单本身走 stderr，
#   这样调用方的 stdout 仍然干净。
#   - 没有可读的 /dev/tty（管道、后台、CI）时直接返回默认值，不阻塞；
#   - 输入可以是序号，也可以直接是值名；直接回车取默认值。
select_profile() {
  local title=$1
  local label=$2
  local default=$3
  shift 3
  local entries=("$@")
  local count=${#entries[@]}
  local default_index=1
  local index value selection

  for index in "${!entries[@]}"; do
    value=${entries[$index]%%|*}
    if [ "$value" = "$default" ]; then
      default_index=$((index + 1))
    fi
  done

  if [ "$count" -eq 0 ]; then
    printf '%s' "$default"
    return 0
  fi

  # /dev/tty "可读"不等于"可用"：setsid/后台/无控制终端时打开会失败(ENXIO)，
  # 所以这里真正打开一次来判断；失败就直接用默认值，不打印菜单也不报错
  # （{ ...; } 让这次失败的打开把错误丢进 /dev/null，fd 3 仍留在当前 shell）。
  if ! { exec 3</dev/tty; } 2>/dev/null; then
    printf '%s' "$default"
    return 0
  fi

  printf '\n%s\n' "$title" >&2
  for index in "${!entries[@]}"; do
    printf '  %d) %s\n' "$((index + 1))" "${entries[$index]#*|}" >&2
  done

  while true; do
    printf '%s [%d]: ' "$label" "$default_index" >&2
    if ! read -r selection <&3 || [ -z "$selection" ]; then
      selection=$default_index
    fi
    for index in "${!entries[@]}"; do
      value=${entries[$index]%%|*}
      if [ "$selection" = "$value" ] || [ "$selection" = "$((index + 1))" ]; then
        exec 3<&-
        printf '%s' "$value"
        return 0
      fi
    done
    echo "Please select 1-${count}." >&2
  done
}
exit_with_launch_status() {
  local status=$1

  # 130=SIGINT(Ctrl-C)、143=SIGTERM：主动关闭信号，视为正常退出；
  # 或者 SHUTDOWN_FILE 已创建（清理函数已执行）也视为主动关闭。
  if [ "$status" -eq 130 ] || [ "$status" -eq 143 ] || \
     { [ "$status" -ne 0 ] && [ -f "${GUGANAV_SHUTDOWN_FILE:-/nonexistent}" ]; }; then
    exit 0
  fi

  exit "$status"
}

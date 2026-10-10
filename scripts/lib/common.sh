#!/usr/bin/env bash
# guganav 入口脚本公共函数库
#
# reality.sh 与 simulation.sh 共用这里的实现，两个入口脚本只保留各自独有的部分
# （实车：udev/串口/清理；仿真：Gazebo 启动与交互菜单）。
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

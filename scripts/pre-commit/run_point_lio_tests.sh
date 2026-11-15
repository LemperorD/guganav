#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)
POINT_LIO_ROOT="$WS/src/guga_localization/LIO/point_lio"

source_setup() {
  local setup_file=$1
  if [ -f "$setup_file" ]; then
    set +u
    source "$setup_file"
    set -u
  fi
}

if [ -z "${ROS_DISTRO:-}" ]; then
  source_setup /opt/ros/humble/setup.bash
fi
cd "$WS"

colcon build \
  --base-paths "$POINT_LIO_ROOT" \
  --packages-select point_lio \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON

source_setup "$WS/install/setup.bash"

# 单元测试目前不初始化 ROS 节点，但保留可写 ROS_HOME 的回退，
# 以免以后加入回放测试时在只读 $HOME 环境里失败。
if [ ! -w "${ROS_HOME:-$HOME/.ros}" ] 2>/dev/null; then
  ROS_HOME="$(mktemp -d)"
  export ROS_HOME
  export ROS_LOG_DIR="$ROS_HOME/log"
  echo "note: ROS_HOME 不可写，回退到 $ROS_HOME"
fi

colcon test \
  --packages-select point_lio \
  --event-handlers console_direct+

if ! colcon test-result --verbose --test-result-base "$WS/build/point_lio"; then
  echo "FAILED: point_lio tests"
  exit 1
fi
echo "point_lio: unit tests OK"

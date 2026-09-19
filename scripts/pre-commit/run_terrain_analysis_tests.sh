#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)
TERRAIN_ROOT="$WS/src/guga_perception/terrain_analysis"

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

rm -rf build/terrain_analysis \
  install/terrain_analysis \
  log/latest_build/terrain_analysis

# --allow-overriding 仅在新版 colcon（支持该参数）时添加；
# 旧版 colcon（如 ros:humble-ros-base 容器的 0.9.x）不认识它，直接传会报
# "unrecognized arguments: --allow-overriding" 导致 CI 失败
ALLOW_OVERRIDE_ARGS=()
if colcon build --help 2>/dev/null | grep -q -- "--allow-overriding"; then
  ALLOW_OVERRIDE_ARGS=(--allow-overriding terrain_analysis)
fi

colcon build \
  --base-paths "$TERRAIN_ROOT" "$WS/src/guga_common" \
  --symlink-install \
  --parallel-workers 1 \
  "${ALLOW_OVERRIDE_ARGS[@]}" \
  --packages-select guga_common terrain_analysis \
  --event-handlers console_direct+ \
  --cmake-clean-cache \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source_setup "$WS/install/setup.bash"

# test_terrain_analysis 会 rclcpp::init 并构造节点，rclcpp 需要一个可写目录
# 放日志/参数文件。若 $HOME 或 ~/.ros 不可写（只读挂载、受限沙箱），会直接
# 段错误，看起来像测试失败。这里在需要时回退到临时目录。
if [ ! -w "${ROS_HOME:-$HOME/.ros}" ] 2>/dev/null; then
  ROS_HOME="$(mktemp -d)"
  export ROS_HOME
  echo "note: ROS_HOME 不可写，回退到 $ROS_HOME"
fi

cd "$WS/build/terrain_analysis"
for t in test_terrain_analysis test_frame_ingest test_algorithm test_integration; do
  ./$t >/dev/null 2>&1 || { echo "FAILED: $t"; exit 1; }
done
echo "terrain_analysis: all 4 test suites OK"

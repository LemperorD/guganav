#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)

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
source_setup "$WS/install/setup.bash"
cd "$WS"

# xtensor/xsimd 由系统 apt 提供（libxtensor-dev / libxsimd-dev），CI 中经 rosdep 安装
colcon build --packages-select nav2_mppi_controller \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source_setup "$WS/install/setup.bash"

# rclcpp 默认写 ~/.ros/log，在沙箱/CI 里可能不可写，默认改到工作区内
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/log/ros_log_mppi}"
mkdir -p "$ROS_LOG_DIR"

cd "$WS/build/nav2_mppi_controller/test"
for t in \
  controller_state_transition_test \
  critic_manager_test \
  critics_tests \
  models_test \
  motion_model_tests \
  noise_generator_test \
  optimizer_smoke_test \
  optimizer_unit_tests \
  parameter_handler_test \
  path_handler_test \
  trajectory_visualizer_tests \
  utils_test; do
  ./$t >/dev/null 2>&1 || { echo "FAILED: $t"; exit 1; }
done
echo "MPPI: all tests OK"

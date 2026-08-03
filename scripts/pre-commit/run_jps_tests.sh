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
source_setup "$HOME/nav2_ws/install/setup.bash"
source_setup "$WS/install/setup.bash"
cd "$WS"

echo ">>> Building jps_planner with tests ..."
colcon build --symlink-install --packages-up-to jps_planner \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source_setup "$WS/install/setup.bash"

echo ">>> Running test_jps ..."
cd "$WS/build/jps_planner"
./test_jps
echo ">>> jps_planner: all tests OK"

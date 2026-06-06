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

colcon build --packages-up-to simple_decision \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

source_setup "$WS/install/setup.bash"

cd "$WS/build/simple_decision"
for t in test_transform test_environment_context test_decision test_simple_decision; do
  ./$t >/dev/null 2>&1 || { echo "FAILED: $t"; exit 1; }
done
echo "simple_decision: all tests OK"

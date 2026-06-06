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

colcon build --packages-select pb_omni_pid_pursuit_controller \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON

source_setup "$WS/install/setup.bash"

cd "$WS/build/pb_omni_pid_pursuit_controller"
for t in test_pid test_geometry_utils test_visualise test_pathhandler test_approach_scaling test_types; do
  ./$t >/dev/null 2>&1 || { echo "FAILED: $t"; exit 1; }
done
echo "PID: all tests OK"

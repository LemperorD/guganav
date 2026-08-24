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

echo "=== Build PointLIO smoke test ==="
colcon build \
  --packages-select livox_ros_driver2 point_lio \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON

source_setup "$WS/install/setup.bash"

# Keep smoke-test discovery and logs isolated from other ROS processes.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/log/ros_log_point_lio_smoke}"
mkdir -p "$ROS_LOG_DIR"

echo "=== Run PointLIO smoke test ==="
colcon test \
  --packages-select point_lio \
  --event-handlers console_direct+ \
  --return-code-on-test-failure

colcon test-result --test-result-base "$WS/build/point_lio" --verbose
echo "PointLIO smoke test: PASSED"

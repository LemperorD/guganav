#!/bin/bash
set -euo pipefail

# ── Source ROS 2 Humble ──
source /opt/ros/humble/setup.bash

# ── Optionally source a Nav2 workspace if it exists ──
if [ -f "$HOME/nav2_ws/install/setup.bash" ]; then
  source "$HOME/nav2_ws/install/setup.bash"
fi

# ── Source the current workspace ──
WS="$(dirname "$(dirname "$(readlink -f "$0")")")"
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

echo ">>> Building jps_planner with tests ..."
cd "$WS"
colcon build --symlink-install --packages-select jps_planner \
  --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  2>&1 | tail -5

source "$WS/install/setup.bash"

echo ">>> Running test_jps ..."
cd "$WS/build/jps_planner"
./test_jps
echo ">>> jps_planner: all tests OK"

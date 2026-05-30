#!/bin/bash
set -e
cd "$(dirname "$0")/../.."
colcon build --packages-select pb_omni_pid_pursuit_controller 2>&1 | tail -1
cd build/pb_omni_pid_pursuit_controller
for t in test_pid test_geometry_utils test_visualise test_pathhandler test_approach_scaling test_types; do
  ./$t >/dev/null 2>&1 || { echo "FAILED: $t"; exit 1; }
done
echo "PID: all tests OK"

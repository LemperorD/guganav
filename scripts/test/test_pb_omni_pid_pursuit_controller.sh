#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd ~/guganav

RESULT_FILE=~/guganav/test_result.ans
rm -f "$RESULT_FILE"

echo "=== Build pb_omni_pid_pursuit_controller ===" | tee -a "$RESULT_FILE"
colcon build --packages-select pb_omni_pid_pursuit_controller \
  --event-handlers console_direct+ \
  2>&1 | tee -a "$RESULT_FILE"

. ~/guganav/install/setup.bash

echo "=== Run tests ===" | tee -a "$RESULT_FILE"
cd ~/guganav/build/pb_omni_pid_pursuit_controller
for test_bin in test_pid test_geometry_utils; do
  echo "--- $test_bin ---" | tee -a "$RESULT_FILE"
  GTEST_COLOR=yes ./$test_bin 2>&1 | tee -a "$RESULT_FILE"
done

echo "" | tee -a "$RESULT_FILE"
echo "=== Done ===" | tee -a "$RESULT_FILE"

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

rm -rf build/pb_omni_pid_pursuit_controller
mkdir -p build/pb_omni_pid_pursuit_controller
RESULT_FILE=$WS/build/pb_omni_pid_pursuit_controller/coverage_result.ans
rm -f "$RESULT_FILE"

echo "=== Clean previous build data ===" | tee -a "$RESULT_FILE"

echo "=== Build pb_omni_pid_pursuit_controller with coverage flags ===" | tee -a "$RESULT_FILE"
colcon build --packages-select pb_omni_pid_pursuit_controller \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_CXX_FLAGS="--coverage -O0" \
    -DCMAKE_C_FLAGS="--coverage -O0" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage" \
  2>&1 | tee -a "$RESULT_FILE"

source_setup "$WS/install/setup.bash"

echo "=== Run tests ===" | tee -a "$RESULT_FILE"
cd "$WS/build/pb_omni_pid_pursuit_controller"
for test_bin in test_pid test_geometry_utils test_visualise test_pathhandler test_approach_scaling test_types; do
  echo "--- $test_bin ---" | tee -a "$RESULT_FILE"
  GTEST_COLOR=yes ./$test_bin 2>&1 | tee -a "$RESULT_FILE"
done

echo "" | tee -a "$RESULT_FILE"
echo "=== Generate coverage report ===" | tee -a "$RESULT_FILE"
cd "$WS"
FILTER_BASE='src/guga_controller/pb_omni_pid_pursuit_controller'
gcovr \
  --root . \
  --object-directory build/pb_omni_pid_pursuit_controller \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found \
  --html --html-details \
  -o build/pb_omni_pid_pursuit_controller/coverage.html 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Generate lcov info ===" | tee -a "$RESULT_FILE"
gcovr \
  --root . \
  --object-directory build/pb_omni_pid_pursuit_controller \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found \
  --lcov \
  -o build/pb_omni_pid_pursuit_controller/lcov_pb_omni_pid_pursuit_controller.info 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Coverage Summary ===" | tee -a "$RESULT_FILE"
gcovr \
  --root . \
  --object-directory build/pb_omni_pid_pursuit_controller \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --gcov-ignore-errors=source_not_found 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Done ===" | tee -a "$RESULT_FILE"
echo "lcov info:    build/pb_omni_pid_pursuit_controller/lcov_pb_omni_pid_pursuit_controller.info" | tee -a "$RESULT_FILE"
echo "coverage html: build/pb_omni_pid_pursuit_controller/coverage.html" | tee -a "$RESULT_FILE"

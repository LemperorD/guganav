#!/bin/bash
set -euo pipefail

WS=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)
PKG=${1:-terrain_analysis}

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

mkdir -p build/terrain_analysis
RESULT_FILE=$WS/build/terrain_analysis/coverage_result.ans
rm -f "$RESULT_FILE"

echo "=== Clean previous coverage data ===" | tee -a "$RESULT_FILE"
find build/terrain_analysis -name "*.gcda" -delete 2>/dev/null

echo "=== Build terrain_analysis with coverage flags ===" | tee -a "$RESULT_FILE"
colcon build --symlink-install --allow-overriding terrain_analysis --packages-select terrain_analysis \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_CXX_FLAGS="--coverage -O0" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
  2>&1 | tee -a "$RESULT_FILE"

source_setup "$WS/install/setup.bash"

echo "=== Run tests ===" | tee -a "$RESULT_FILE"
cd "$WS/build/terrain_analysis"
for test_bin in test_terrain_analysis test_context test_algorithm test_connectivity; do
  GTEST_COLOR=yes ./$test_bin 2>&1 | grep -E "FAILED|PASSED.*tests" | tee -a "$RESULT_FILE"
done

echo "" | tee -a "$RESULT_FILE"
echo "=== Generate coverage report ===" | tee -a "$RESULT_FILE"
cd "$WS"
FILTER_BASE='src/guga_perception/terrain_analysis'
gcovr \
  --root . \
  --object-directory build/terrain_analysis \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --exclude '.*gtest.*' \
  --gcov-ignore-errors=source_not_found \
  --html --html-details \
  -o build/terrain_analysis/coverage.html 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Generate lcov info ===" | tee -a "$RESULT_FILE"
gcovr \
  --root . \
  --object-directory build/terrain_analysis \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --exclude '.*gtest.*' \
  --gcov-ignore-errors=source_not_found \
  --lcov \
  -o build/terrain_analysis/lcov_terrain_analysis.info 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Coverage Summary ===" | tee -a "$RESULT_FILE"
gcovr \
  --root . \
  --object-directory build/terrain_analysis \
  --filter "${FILTER_BASE}/src/.*\\.cpp" \
  --filter "${FILTER_BASE}/include/.*\\.hpp" \
  --exclude '.*test.*' \
  --exclude '.*gtest.*' \
  --gcov-ignore-errors=source_not_found 2>&1 | tee -a "$RESULT_FILE"

echo "" | tee -a "$RESULT_FILE"
echo "=== Done ===" | tee -a "$RESULT_FILE"
echo "lcov info:    build/terrain_analysis/lcov_terrain_analysis.info" | tee -a "$RESULT_FILE"
echo "coverage html: build/terrain_analysis/coverage.html" | tee -a "$RESULT_FILE"

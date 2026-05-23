#!/bin/bash
if [ -z "$ROS_DISTRO" ]; then
  test -f /opt/ros/humble/setup.bash && source /opt/ros/humble/setup.bash
fi
if [ -f ~/guganav/install/setup.bash ]; then
  source ~/guganav/install/setup.bash
fi
cd ~/guganav

RESULT_FILE=~/guganav/build/terrain_analysis/coverage_result.ans
rm -f "$RESULT_FILE"

echo "=== Clean previous coverage data ===" | tee -a "$RESULT_FILE"
find build/terrain_analysis -name "*.gcda" -delete 2>/dev/null
mkdir -p build/terrain_analysis

echo "=== Build terrain_analysis with coverage flags ===" | tee -a "$RESULT_FILE"
colcon build --symlink-install --allow-overriding terrain_analysis --packages-select terrain_analysis \
  --event-handlers console_direct+ \
  --cmake-args \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS="--coverage -O0" \
    -DCMAKE_C_FLAGS="--coverage -O0" \
    -DCMAKE_EXE_LINKER_FLAGS="--coverage" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage" \
  2>&1 | tee -a "$RESULT_FILE"

. ~/guganav/install/setup.bash

echo "=== Run tests ===" | tee -a "$RESULT_FILE"
cd ~/guganav/build/terrain_analysis
for test_bin in test_terrain_analysis test_context test_algorithm test_connectivity; do
  GTEST_COLOR=yes ./$test_bin 2>&1 | grep -E "FAILED|PASSED.*tests" | tee -a "$RESULT_FILE"
done

echo "" | tee -a "$RESULT_FILE"
echo "=== Generate coverage report ===" | tee -a "$RESULT_FILE"
cd ~/guganav
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
  -o lcov.info 2>&1 | tee -a "$RESULT_FILE"

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
echo "lcov info:    lcov.info" | tee -a "$RESULT_FILE"
echo "coverage html: build/terrain_analysis/coverage.html" | tee -a "$RESULT_FILE"
